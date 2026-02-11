/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __MFD_FBOSS_MDIO_H__
#define __MFD_FBOSS_MDIO_H__

#include <linux/miscdevice.h>
#include <linux/phy.h>

struct fb_mdio_desc {
	struct miscdevice miscdev;
	char name[NAME_MAX];

	/*
	 * Cache structures for easier access in ioctl fileops.
	 */
	struct mii_bus *bus;
};

int fb_mdio_cdev_init(struct fb_mdio_desc *mdesc, struct mii_bus *bus, u32 id);
void fb_mdio_cdev_destroy(struct fb_mdio_desc *mdesc);

#endif /* __MFD_FBOSS_MDIO_H__ */
