// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * mtia_pwrcpld.c
 *
 * PWR CPLD I2C Slave Module @addr 0x60, mainly for power control.
 *
 * PWR CPLD takes care of power sequence control, reset control, power status monitoring.
 * Leakage detection and Leakage led control.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "regbit-sysfs.h"

#define DRIVER_NAME	"mtia_pwrcpld"

#define FBPWR_REG_SLOT_ID    0x0E

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 0,
		.num_bits = 4,
	},
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 4,
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
	{
		.name = "slot_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBPWR_REG_SLOT_ID,
		.bit_offset = 0,
		.num_bits = 4,
	},
};


static const struct i2c_device_id mtia_pwrcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mtia_pwrcpld_id);

#if KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE
static int mtia_pwrcpld_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
#else
static int mtia_pwrcpld_probe(struct i2c_client *client)
#endif
{
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver mtia_pwrcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mtia_pwrcpld_probe,
	.id_table = mtia_pwrcpld_id,
};
module_i2c_driver(mtia_pwrcpld_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lucas Liu <lucli@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS Minerva blade Power CPLD Driver");
MODULE_VERSION(BSP_VERSION);
