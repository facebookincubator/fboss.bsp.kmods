// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "regbit-sysfs.h"

#define DRIVER_NAME	"anacapa_rtmcpld"

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

static int rtmcpld_probe(struct i2c_client *client)
{
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
								 ARRAY_SIZE(sysfs_files));
}

static const struct i2c_device_id anacapa_rtmcpld_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, anacapa_rtmcpld_id);

static struct i2c_driver anacapa_rtmcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = rtmcpld_probe,
	.id_table = anacapa_rtmcpld_id,
};
module_i2c_driver(anacapa_rtmcpld_driver);

MODULE_AUTHOR("Yves Wang <yves.wang@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS RTM CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
