/* drivers/usb/gadget/f_sh_mdlm.c
 * 
 * f_sh_mdlm.c - MDLM function driver
 *
 * Copyright (C) 2013 SHARP CORPORATION
 *
 * This code also borrows from f_serial.c, which is
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include "u_serial.h"
#include "gadget_chips.h"

#ifdef CONFIG_USB_ANDROID_SH_DTFER
#include <linux/switch.h>
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */

struct gser_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct f_gser {
	struct gserial			port;
	u8				data_id;
	u8				port_num;

	struct gser_descs		fs;
	struct gser_descs		hs;
	u8				online;
};

static inline struct f_gser *func_to_gser(struct usb_function *f)
{
	return container_of(f, struct f_gser, port.func);
}

#ifdef CONFIG_USB_ANDROID_SH_DTFER
#define SWITCH_NAME_ALT_DTFER	"dtfer_open_sts"

struct switch_dtfer_open_sts {
	struct switch_dev		sdev;
	struct work_struct		swork;
	int				open_sts;
	int				init;
};

static struct switch_dtfer_open_sts switch_dtfer;
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor gser_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =      USB_CLASS_COMM,
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_MDLM,
	.bInterfaceProtocol =	0x01, /* Control plane protocol */
	/* .iInterface = DYNAMIC */
};
static struct usb_cdc_header_desc mdlm_header_desc = {
	.bLength =		sizeof mdlm_header_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		cpu_to_le16(0x0110),
};
static struct usb_cdc_mdlm_desc mdlm_desc = {
	.bLength =		sizeof mdlm_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_MDLM_TYPE,
	.bcdVersion =		cpu_to_le16(0x0100),
/* 	.bGUID = DYNAMIC */
};

/* full speed support: */
static struct usb_endpoint_descriptor gser_fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gser_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *gser_fs_function[] = {
	(struct usb_descriptor_header *) &gser_interface_desc,
	(struct usb_descriptor_header *) &mdlm_header_desc,
	(struct usb_descriptor_header *) &mdlm_desc,
	(struct usb_descriptor_header *) &gser_fs_in_desc,
	(struct usb_descriptor_header *) &gser_fs_out_desc,
	NULL,
};

/* high speed support: */
static struct usb_endpoint_descriptor gser_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor gser_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *gser_hs_function[] = {
	(struct usb_descriptor_header *) &gser_interface_desc,
	(struct usb_descriptor_header *) &mdlm_header_desc,
	(struct usb_descriptor_header *) &mdlm_desc,
	(struct usb_descriptor_header *) &gser_hs_in_desc,
	(struct usb_descriptor_header *) &gser_hs_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string gser_string_defs[] = {
	[0].s = mdlm_iInterface,
	{  } /* end of list */
};

static struct usb_gadget_strings gser_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		gser_string_defs,
};

static struct usb_gadget_strings *gser_strings[] = {
	&gser_string_table,
	NULL,
};

#ifdef CONFIG_USB_ANDROID_SH_DTFER
static ssize_t print_switch_name_alt_sts_dtfer(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", SWITCH_NAME_ALT_DTFER);
}

static ssize_t print_switch_state_alt_sts_dtfer(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", (switch_dtfer.open_sts ? "dtfer_open" : "dtfer_close"));
}
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

/*-------------------------------------------------------------------------*/

static int gser_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_gser		*gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int rc = 0;

	/* we know alt == 0, so this is an activation or a reset */

	if (gser->port.in->driver_data) {
		DBG(cdev, "reset generic data ttyGS%d\n", gser->port_num);
		gserial_disconnect(&gser->port);
	}
	if (!gser->port.in->desc || !gser->port.out->desc) {
		DBG(cdev, "activate generic ttyGS%d\n", gser->port_num);
		if (config_ep_by_speed(cdev->gadget, f, gser->port.in) ||
			config_ep_by_speed(cdev->gadget, f, gser->port.out)) {
			gser->port.in->desc = NULL;
			gser->port.out->desc = NULL;
			return -EINVAL;
		}
	}

	gserial_connect(&gser->port, gser->port_num);

#ifdef CONFIG_USB_ANDROID_SH_DTFER
	switch_dtfer.open_sts = 1;
	schedule_work(&switch_dtfer.swork);
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

	gser->online = 1;
	return rc;
}

static void gser_disable(struct usb_function *f)
{
	struct f_gser	*gser = func_to_gser(f);

	gserial_disconnect(&gser->port);
	gser->online = 0;

#ifdef CONFIG_USB_ANDROID_SH_DTFER
	if (switch_dtfer.open_sts) {
		switch_dtfer.open_sts = 0;
		schedule_work(&switch_dtfer.swork);
	}
#endif /* CONFIG_USB_ANDROID_SH_DTFER */
}

static int
gser_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser		*gser = func_to_gser(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;
	gser_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(gser_fs_function);

	if (!f->descriptors)
		goto fail;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		gser_hs_in_desc.bEndpointAddress =
				gser_fs_in_desc.bEndpointAddress;
		gser_hs_out_desc.bEndpointAddress =
				gser_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(gser_hs_function);

		if (!f->hs_descriptors)
			goto fail;
	}

	DBG(cdev, "generic ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser->port.in->name, gser->port.out->name);
	return 0;

fail:
	if (f->descriptors)
		usb_free_descriptors(f->descriptors);

	/* we might as well release our claims on endpoints */
	if (gser->port.out)
		gser->port.out->driver_data = NULL;
	if (gser->port.in)
		gser->port.in->driver_data = NULL;

	return status;
}

static void
gser_unbind(struct usb_configuration *c, struct usb_function *f)
{
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(func_to_gser(f));
}

static void gser_suspend(struct usb_function *f)
{
	gser_disable(f);
}

static void gser_resume(struct usb_function *f)
{
	/* copy from gser_set_alt */
	struct f_gser		 *gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	if (gser->port.in->driver_data) {
		DBG(cdev, "reset generic data ttyGS%d\n", gser->port_num);
		gserial_disconnect(&gser->port);
	}
	if (!gser->port.in->desc || !gser->port.out->desc) {
		DBG(cdev, "activate generic ttyGS%d\n", gser->port_num);
		if (config_ep_by_speed(cdev->gadget, f, gser->port.in) ||
			config_ep_by_speed(cdev->gadget, f, gser->port.out)) {
			gser->port.in->desc = NULL;
			gser->port.out->desc = NULL;
			return;
		}
	}
	gserial_connect(&gser->port, gser->port_num);
#ifdef CONFIG_USB_ANDROID_SH_DTFER
	switch_dtfer.open_sts = 1;
	schedule_work(&switch_dtfer.swork);
#endif /* CONFIG_USB_ANDROID_SH_DTFER */
	gser->online = 1;
	return;
}

/**
 * set_guid_value - set GUID value at MDLM Functional Descriptor
 * @buf: 16 bytes of data
 */
void set_guid_value(unsigned char *buf)
{
	memcpy(mdlm_desc.bGUID, buf, sizeof(mdlm_desc.bGUID));
}

#ifdef CONFIG_USB_ANDROID_SH_DTFER
static void dtfer_setinterface_switch(struct work_struct *w)
{
	switch_set_state(&switch_dtfer.sdev, switch_dtfer.open_sts);
}
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

/**
 * gser_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int gser_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_gser	*gser;
	int		status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (gser_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		gser_string_defs[0].id = status;
		gser_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	gser = kzalloc(sizeof *gser, GFP_KERNEL);
	if (!gser)
		return -ENOMEM;

	gser->port_num = port_num;

	gser->port.func.name = "mdlm";
	gser->port.func.strings = gser_strings;
	gser->port.func.bind = gser_bind;
	gser->port.func.unbind = gser_unbind;
	gser->port.func.set_alt = gser_set_alt;
	gser->port.func.disable = gser_disable;
	gser->port.func.suspend = gser_suspend;
	gser->port.func.resume = gser_resume;

#ifdef CONFIG_USB_ANDROID_SH_DTFER
	if (!switch_dtfer.init) {
		switch_dtfer.sdev.name = SWITCH_NAME_ALT_DTFER;
		switch_dtfer.sdev.print_name = print_switch_name_alt_sts_dtfer;
		switch_dtfer.sdev.print_state = print_switch_state_alt_sts_dtfer;

		INIT_WORK(&switch_dtfer.swork, dtfer_setinterface_switch);

		status = switch_dev_register(&switch_dtfer.sdev);
		if (status < 0){
			printk(KERN_INFO "%s: can't register switch_dev , err %d\n", __func__, status); 
		}else{
			switch_dtfer.init = 1;
		}
	}
#endif /* CONFIG_USB_ANDROID_SH_DTFER */

	status = usb_add_function(c, &gser->port.func);
	if (status)
		kfree(gser);
	return status;
}
