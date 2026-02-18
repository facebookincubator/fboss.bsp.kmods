/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __FBCPLD_CDEV_H__
#define __FBCPLD_CDEV_H__

#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#include "fbcpld-ioctl.h"

/*
 * Dynamic sysfs entry created at runtime via ioctl.
 */
struct fbcpld_dyn_entry {
	struct device_attribute dev_attr;
	struct fbcpld_sysfs_attr attr_cfg;
};

/*
 * Per-device descriptor for the generic CPLD driver.
 */
struct fbcpld_cdev_desc {
	struct miscdevice miscdev;
	char name[NAME_MAX];		/* "fbcpld-<bus>-<addr>" */
	struct device *dev;
	struct i2c_client *client;

	struct mutex entries_lock;	/* Protects entries array */
	struct fbcpld_dyn_entry entries[FBCPLD_MAX_ATTRS];
	size_t num_entries;
};

int fbcpld_cdev_init(struct i2c_client *client);

#endif /* __FBCPLD_CDEV_H__ */
