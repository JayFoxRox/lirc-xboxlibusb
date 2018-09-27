/****************************************************************************
** xboxlibusb.c ************************************************************
****************************************************************************
*  Userspace (libusb) driver for original Xbox DVD Movie Playback Kit Dongle.
*
*  Copyright (C) 2018 Jannik Vogel <email@jannikvogel.de>
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Library General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
*/

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <stdint.h>
#include <pthread.h>

#if defined(HAVE_LIBUSB_1_0_LIBUSB_H)
#include <libusb-1.0/libusb.h>
#else
#error Cannot find required libusb.h header
#endif

#include "lirc_driver.h"

#define USB_TIMEOUT (1000 * 60)

static int xbox_init(void);
static int xbox_deinit(void);
static char* xbox_rec(struct ir_remote* remotes);
static libusb_device* find_usb_device(void);
static int find_device_endpoints(libusb_device* dev);
static int drvctl_func(unsigned int cmd, void* arg);

static const logchannel_t logchannel = LOG_DRIVER;

const struct driver hw_xboxlibusb = {
	.name		= "xboxlibusb",
	.device		= NULL,
	.fd		= -1,
	.features	= LIRC_CAN_REC_LIRCCODE,
	.send_mode	= 0,
	.rec_mode	= LIRC_MODE_LIRCCODE,
	.code_length	= 24,
	.init_func	= xbox_init,
	.deinit_func	= xbox_deinit,
	.open_func	= default_open,
	.close_func	= default_close,
	.send_func	= NULL,
	.rec_func	= xbox_rec,
	.decode_func	= receive_decode,
	.drvctl_func	= drvctl_func,
	.readdata	= NULL,
	.api_version	= 3,
	.driver_version = "0.1.0",
	.info		= "No info available",
	.device_hint    = "auto",
};

const struct driver* hardwares[] = { &hw_xboxlibusb, (const struct driver*)NULL };

typedef struct {
	uint16_t vendor;
	uint16_t product;
} usb_device_id;

/* table of compatible remotes -- from lirc_atiusb */
static usb_device_id usb_remote_id_table[] = {
	{ 0x040b, 0x6521 }, /* Gamester Xbox DVD Movie Playback Kit IR */
	{ 0x045e, 0x0284 }, /* Microsoft Xbox DVD Movie Playback Kit IR */

	/* Some Chinese manufacturer -- conflicts with the joystick from the
	 * same manufacturer */
	{ 0xFFFF, 0xFFFF },

	{ 0,	  0	 }      /* Terminating entry */
};

static char device_path[10000] = {0};

static pthread_t usb_thread;
static uint8_t usb_buffer[6];
static libusb_context* usb_context = NULL;
static libusb_device_handle* dev_handle = NULL;
static uint8_t endpoint_address;
static struct libusb_transfer* usb_transfer = NULL;
static int completed;
static int skip_garbage;

/****/

/* returns 1 if the given device should be used, 0 otherwise */
static int is_device_ok(uint16_t vendor,  uint16_t product)
{
	usb_device_id* d;

	for (d = usb_remote_id_table; d->vendor; d++) {	
		if ((vendor == d->vendor) && (product == d->product))
			return 1;
	}
	return 0;
}


static int is_usb_device_ok(libusb_device* d)
{
	struct libusb_device_descriptor dev_descriptor;
	libusb_get_device_descriptor(d, &dev_descriptor);
	return is_device_ok(dev_descriptor.idVendor, dev_descriptor.idProduct);
}


static int drvctl_func(unsigned int cmd, void* arg)
{
	switch (cmd) {
	case DRVCTL_GET_DEVICES:
		return drv_enum_usb((glob_t*) arg, is_device_ok);
	case DRVCTL_FREE_DEVICES:
		drv_enum_free((glob_t*) arg);
		return 0;
	default:
		return DRV_ERR_NOT_IMPLEMENTED;
	}
}


static void usb_callback(struct libusb_transfer* transfer) {
	uint16_t check;
	uint16_t data;	
	uint8_t decode_buf[3];
	int bytes_r;
	int bytes_w;
	int pos;
	int fd = (intptr_t)transfer->user_data;
	uint8_t* buf = transfer->buffer;

	/* Abort if the transfer was cancelled */
	if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
		log_trace("Transfer has been cancelled");
		completed = 1;
		return;
	}

	if (!skip_garbage) {

		/* We reconstruct the original IR packet, including checksum,
		 * so we can use the same config as a serial driver */
		data = ((buf[3] & 0xF) << 8) | buf[2];
		check = data ^ 0xFFF;
		decode_buf[0] = (check >> 4) & 0xFF;
		decode_buf[1] = ((check & 0xF) << 4) | ((data >> 8) & 0xF);
		decode_buf[2] = data & 0xFF;


		/* write to the pipe */
		bytes_r = sizeof(decode_buf);
		for (pos = 0; pos < bytes_r; pos += bytes_w) {
			bytes_w = write(fd, decode_buf + pos, bytes_r - pos);
			if (bytes_w < 0) {
				log_perror_err("can't write to pipe");
				break;
			}
		}
	} else {
		skip_garbage = 0;
	}

	/* Resubmit the transfer */
	if (libusb_submit_transfer(transfer) != 0) {
		log_error("Unable to resubmit");
	}
}

static void* usb_loop(void* user_data) {

	/* Reset our event flags, so we discard the first USB transfer, and stay
	 * in the event loop */
	completed = 0;
	skip_garbage = 1;

	usb_transfer = libusb_alloc_transfer(0);
	libusb_fill_interrupt_transfer(usb_transfer, dev_handle,
				       endpoint_address,
				       usb_buffer, sizeof(usb_buffer),
				       usb_callback, user_data,
				       USB_TIMEOUT);

	if (libusb_submit_transfer(usb_transfer) != 0) {
		log_error("Unable to submit");
		return NULL;
	}

	while (!completed) {
		libusb_handle_events_completed(usb_context, &completed);
	}

	return NULL;
}

/* initialize driver -- returns 1 on success, 0 on error */
static int xbox_init(void)
{
	libusb_device* usb_dev = NULL;
	int pipe_fd[2] = { -1, -1 };

	log_trace("initializing USB receiver");

	rec_buffer_init();

	/* A separate process will be forked to read data from the USB receiver
	 * and write it to a pipe. drv.fd is set to the readable end of this
	 * pipe. */
	if (pipe(pipe_fd) != 0) {
		log_perror_err("couldn't open pipe");
		goto fail;
	}
	drv.fd = pipe_fd[0];

	if (libusb_init(&usb_context) != 0) {
		log_error("couldn't initialize libusb");
		goto fail;
	}

	usb_dev = find_usb_device();
	if (!usb_dev || !libusb_get_bus_number(usb_dev)) {
		log_error("couldn't find a compatible USB device");
		goto fail;
	}

	if (!find_device_endpoints(usb_dev)) {
		log_error("couldn't find device endpoints");
		goto fail;
	}

	if (libusb_open(usb_dev, &dev_handle) != 0) {
		log_error("couldn't open USB receiver");
		goto fail;
	}

	if (libusb_claim_interface(dev_handle, 0) != 0) {
		log_error("couldn't claim USB interface");
		goto fail;
	}

	snprintf(device_path, sizeof(device_path),
		 "/dev/bus/usb/%03d/%03d\n",
		 libusb_get_bus_number(usb_dev),
		 libusb_get_port_number(usb_dev));
	log_debug("xboxlibusb: using device: %s", device_path);
	drv.device = device_path;

	/* We can unreference the device, because libusb_open keeps track
	 * of it */
	libusb_unref_device(usb_dev);
	usb_dev = NULL;

	/* Create a thread to handle it */
	void* user_data = (void*)(intptr_t)pipe_fd[1];
	if (pthread_create(&usb_thread, NULL, usb_loop, user_data) != 0) {
		log_error("unable to start USB thread");
		goto fail;
	}

	return 1;

fail:
	if (usb_dev) {
		libusb_unref_device(usb_dev);
		usb_dev = NULL;
	}
	if (dev_handle) {
		libusb_close(dev_handle);
		dev_handle = NULL;
	}
	if (pipe_fd[0] >= 0)
		close(pipe_fd[0]);
	if (pipe_fd[1] >= 0)
		close(pipe_fd[1]);
	return 0;
}

/* deinitialize driver -- returns 1 on success, 0 on error */
static int xbox_deinit(void)
{
	int ret;
	int err = 0;

	if (usb_transfer) {
		ret = libusb_cancel_transfer(usb_transfer);	
		if (ret != 0 && ret != LIBUSB_ERROR_NOT_FOUND)
			err = 1;
	}

	if (pthread_join(usb_thread, NULL) != 0)
		err = 1;

	if (usb_transfer) {
		libusb_free_transfer(usb_transfer);
		usb_transfer = NULL;
	}

	if (dev_handle) {
		libusb_close(dev_handle);
		dev_handle = NULL;
	}

	if (usb_context) {
		libusb_exit(usb_context);
		usb_context = NULL;
	}

	if (drv.fd >= 0) {
		if (close(drv.fd) < 0)
			err = 1;
		drv.fd = -1;
	}

	return !err;
}

static char* xbox_rec(struct ir_remote* remotes)
{
	if (!rec_buffer_clear())
		return NULL;
	return decode_all(remotes);
}

/* find a compatible USB receiver and return a usb_device,
 * or NULL on failure. */
static libusb_device* find_usb_device(void)
{
	int i;
	libusb_device* dev = NULL;
	libusb_device** device_list;
	ssize_t device_count;

	device_count = libusb_get_device_list(usb_context, &device_list);
	if (device_count < 0) {
		log_error("can't get USB device list");
		return NULL;
	}

	for (i = 0; i < device_count; i++) {
		if (is_usb_device_ok(device_list[i])) {
			dev = device_list[i];
			break;
		}
	}

	if (dev != NULL)
		libusb_ref_device(dev);

	libusb_free_device_list(device_list, 1);
	return dev;
}




/* set endpoint_address for the given device.
 * returns 1 on success, 0 on failure. */
static int find_device_endpoints(libusb_device* dev)
{
	struct libusb_device_descriptor dev_descriptor;
	struct libusb_config_descriptor* config_descriptor;
	const struct libusb_interface_descriptor* interface_descriptor;
	const struct libusb_endpoint_descriptor* dev_ep_in;

	if (libusb_get_device_descriptor(dev, &dev_descriptor) != 0) {
		log_error("unable to get USB device descriptor");
		goto fail;
	}

	if (dev_descriptor.bNumConfigurations != 1)
		goto fail;

	if (libusb_get_config_descriptor(dev, 0, &config_descriptor) != 0) {
		log_error("unable to get USB configuration descriptor");
		goto fail;
	}
	if (config_descriptor->bNumInterfaces != 2)
		goto fail;
	if (config_descriptor->interface[0].num_altsetting != 1)
		goto fail;

	interface_descriptor = &config_descriptor->interface[0].altsetting[0];
	if (interface_descriptor->bNumEndpoints != 1)
		goto fail;

	dev_ep_in = &interface_descriptor->endpoint[0];
	if ((dev_ep_in->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK)
	    != LIBUSB_ENDPOINT_IN)
		goto fail;
	if ((dev_ep_in->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK)
	    != LIBUSB_TRANSFER_TYPE_INTERRUPT)
		goto fail;

	endpoint_address = dev_ep_in->bEndpointAddress;

	libusb_free_config_descriptor(config_descriptor);
	config_descriptor = NULL;

	return 1;

fail:
	if (config_descriptor) {
		libusb_free_config_descriptor(config_descriptor);
		config_descriptor = NULL;
	}
	return 0;
}
