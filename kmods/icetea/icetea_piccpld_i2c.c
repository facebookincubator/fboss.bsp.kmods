// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * piccpld.c
 *
 * PIC CPLD I2C Slave Module @addr 0x33, mainly for managing optical modules.
 *
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "regbit-sysfs.h"

#define DRIVER_NAME	"icetea_piccpld"

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 4,
		.num_bits = 4,
	},
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 0,
		.num_bits = 4,
	},
	{
		.name = "fw_ver",
		.mode = REGBIT_FMODE_RO,
		.show_func = cpld_fw_ver_show,
	},
	{
		.name = "cpld_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 1,
		.bit_offset = 0,
		.num_bits = 7,
	},
	{
		.name = "cpld_minor_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 2,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "cpld_sub_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 3,
		.bit_offset = 0,
		.num_bits = 8,
	},
};

static const struct i2c_device_id icetea_piccpld_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, icetea_piccpld_id);

static int piccpld_probe(struct i2c_client *client)
{
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
								 ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver icetea_piccpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = piccpld_probe,
	.id_table = icetea_piccpld_id,
};
module_i2c_driver(icetea_piccpld_driver);

MODULE_AUTHOR("Joy Wu <joy.wu@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS PICCPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
