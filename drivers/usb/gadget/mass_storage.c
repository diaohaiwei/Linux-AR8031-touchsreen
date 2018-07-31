/*
 * mass_storage.c -- Mass Storage USB Gadget
 *
 * Copyright (C) 2003-2008 Alan Stern
 * Copyright (C) 2009 Samsung Electronics
 *                    Author: Michal Nazarewicz <mina86@mina86.com>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


/*
 * The Mass Storage Gadget acts as a USB Mass Storage device,
 * appearing to the host as a disk drive or as a CD-ROM drive.  In
 * addition to providing an example of a genuinely useful gadget
 * driver for a USB device, it also illustrates a technique of
 * double-buffering for increased throughput.  Last but not least, it
 * gives an easy way to probe the behavior of the Mass Storage drivers
 * in a USB host.
 *
 * Since this file serves only administrative purposes and all the
 * business logic is implemented in f_mass_storage.* file.  Read
 * comments in this file for more detailed description.
 */


#include <linux/kernel.h>
#include <linux/usb/ch9.h>
#include <linux/module.h>

/*-------------------------------------------------------------------------*/

#define DRIVER_DESC		"Mass Storage Gadget"
#define DRIVER_VERSION		"2009/09/11"

/*-------------------------------------------------------------------------*/

/*
 * kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "f_mass_storage.c"

/*-------------------------------------------------------------------------*/
USB_GADGET_COMPOSITE_OPTIONS();

static struct usb_device_descriptor msg_device_desc = {
	.bLength =		sizeof msg_device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	/* Vendor and product id can be overridden by module parameters.  */
	.idVendor =		cpu_to_le16(FSG_VENDOR_ID),
	.idProduct =		cpu_to_le16(FSG_PRODUCT_ID),
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/*
	 * REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
	.bcdOTG =		cpu_to_le16(0x0200),
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = "",
	[USB_GADGET_PRODUCT_IDX].s = DRIVER_DESC,
	[USB_GADGET_SERIAL_IDX].s = "",
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language       = 0x0409,       /* en-us */
	.strings        = strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

/****************************** Configurations ******************************/

static struct fsg_module_parameters mod_data = {
	.stall = 1
};

FSG_MODULE_PARAMETERS(/* no prefix */, mod_data);

static unsigned long msg_registered;
static void msg_cleanup(void);

static int msg_thread_exits(struct fsg_common *common)
{
	msg_cleanup();
	return 0;
}

static int __init msg_do_config(struct usb_configuration *c)
{
	static const struct fsg_operations ops = {
		.thread_exits = msg_thread_exits,
	};
	static struct fsg_common common;

	struct fsg_common *retp;
	struct fsg_config config;
	int ret;

	pr_debug("=========msg_do_config\r\n");
	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
		pr_info("=========gadget_is_otg=true\r\n");
	}else{
		pr_info("=========gadget_is_otg=false\r\n");
	}

	pr_debug("=========fsg_config_from_params\r\n");
	fsg_config_from_params(&config, &mod_data);
	config.ops = &ops;

#if defined(CONFIG_FSL_UTP)
	pr_debug("============mod_data:file %s \r\n", mod_data.file[0]);
	pr_debug("============mod_data:removable %d \r\n", mod_data.removable[0]);
	pr_debug("============mod_data:cdrom %d \r\n", mod_data.cdrom[0]);
	pr_debug("============mod_data:ro %d \r\n", mod_data.ro[0]);
	mod_data.cdrom[0] = 0;
	mod_data.ro[0] = 0;
	pr_debug("============***mod_data:cdrom %d \r\n", mod_data.cdrom[0]);
	pr_debug("============***mod_data:ro %d \r\n", mod_data.ro[0]);
#endif


	retp = fsg_common_init(&common, c->cdev, &config);
	if (IS_ERR(retp))
		return PTR_ERR(retp);

	pr_debug("=========fsg_bind_config\r\n");
	ret = fsg_bind_config(c->cdev, c, &common);
	fsg_common_put(&common);
	pr_debug("=========msg_do_config leave\r\n");
	return ret;
}

static struct usb_configuration msg_config_driver = {
	.label			= "Linux File-Backed Storage",
	.bConfigurationValue	= 1,
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};


/****************************** Gadget Bind ******************************/

static int __init msg_bind(struct usb_composite_dev *cdev)
{
	int status;

	status = usb_string_ids_tab(cdev, strings_dev);
	if (status < 0)
		return status;
	msg_device_desc.iProduct = strings_dev[USB_GADGET_PRODUCT_IDX].id;

#if defined(CONFIG_FSL_UTP)
	pr_debug("============msg_device_desc.idVendor: %02x \r\n", msg_device_desc.idVendor);
	pr_debug("============msg_device_desc.idProduct: %02x \r\n", msg_device_desc.idProduct);
	msg_device_desc.idVendor = 0x066F;
	msg_device_desc.idProduct = 0x37FF;
	pr_debug("============***msg_device_desc.idVendor: %02x \r\n", msg_device_desc.idVendor);
	pr_debug("============***msg_device_desc.idProduct: %02x \r\n", msg_device_desc.idProduct);
#endif

	status = usb_add_config(cdev, &msg_config_driver, msg_do_config);
	if (status < 0)
		return status;
	usb_composite_overwrite_options(cdev, &coverwrite);
	dev_info(&cdev->gadget->dev,
		 DRIVER_DESC ", version: " DRIVER_VERSION "\n");
	set_bit(0, &msg_registered);
	return 0;
}


/****************************** Some noise ******************************/

static __refdata struct usb_composite_driver msg_driver = {
	.name		= "g_mass_storage",
	.dev		= &msg_device_desc,
	.max_speed	= USB_SPEED_SUPER,
	.needs_serial	= 1,
	.strings	= dev_strings,
	.bind		= msg_bind,
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Michal Nazarewicz");
MODULE_LICENSE("GPL");

static int __init msg_init(void)
{
	return usb_composite_probe(&msg_driver);
}
late_initcall(msg_init);

static void msg_cleanup(void)
{
	if (test_and_clear_bit(0, &msg_registered))
		usb_composite_unregister(&msg_driver);
}
module_exit(msg_cleanup);
