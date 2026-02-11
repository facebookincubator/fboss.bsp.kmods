// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * mcbcpld.c
 *
 * MCB CPLD I2C Slave Module @addr 0x60, mainly for power control.
 *
 * Refer to "fancpld.c" for MCB CPLD fan control component.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "regbit-sysfs.h"

#define DRIVER_NAME	"icecube_mcbcpld"

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
	{
		.name = "oob_eeprom_wp",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = 6,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "oob_eeprom_sel",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = 6,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "smb_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0A,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "hs_48v_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0A,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "smb_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x10,
		.bit_offset = 5,
		.num_bits = 1,
	},
};

static const struct i2c_device_id icecube_mcbcpld_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, icecube_mcbcpld_id);

static int mcbcpld_probe(struct i2c_client *client)
{
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
								 ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver icecube_mcbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcbcpld_probe,
	.id_table = icecube_mcbcpld_id,
};
module_i2c_driver(icecube_mcbcpld_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Qiuyun Xie <qxie@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS MCB Power CPLD Driver");
MODULE_VERSION(BSP_VERSION);
