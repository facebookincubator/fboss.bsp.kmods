// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * MCBCPLD I2C Slave Module @addr 0x60, mainly for power control.
 * Refer to FANCPLD for MCB CPLD fan control component.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "regbit-sysfs.h"

#define DRIVER_NAME	"leh800b_mcbcpld"

#define FBMCB_BOARD_VERSION_TYPE 0x00
#define FBMCB_CPLD_MAJOR_VERSION 0x01
#define FBMCB_CPLD_MINOR_VERSION 0x02
#define FBMCB_CPLD_SUB_VERSION 0x03
#define FBMCB_MEMORY_DEVICES_WP_CTRL 0x06
#define FBMCB_SYSTEM_PWR_STATUS_1 0x0A
#define FBMCB_SYSTEM_PWR_STATUS_2 0x0B
#define FBMCB_SYSTEM_MISC_STATUS_1 0x10
#define FBMCB_SYSTEM_MISC_STATUS_3 0x12

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_BOARD_VERSION_TYPE,
		.bit_offset = 4,
		.num_bits = 4,
	},
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_BOARD_VERSION_TYPE,
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
		.reg_addr = FBMCB_CPLD_MAJOR_VERSION,
		.bit_offset = 0,
		.num_bits = 7,
	},
	{
		.name = "cpld_minor_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_CPLD_MINOR_VERSION,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "cpld_sub_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_CPLD_SUB_VERSION,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "oob_eeprom_wp",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = FBMCB_MEMORY_DEVICES_WP_CTRL,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "oob_eeprom_sel",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = FBMCB_MEMORY_DEVICES_WP_CTRL,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "hs_48v_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYSTEM_PWR_STATUS_1,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "smbl_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYSTEM_PWR_STATUS_2,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "smbr_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYSTEM_PWR_STATUS_2,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "smbl_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYSTEM_MISC_STATUS_1,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "smbr_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYSTEM_MISC_STATUS_3,
		.bit_offset = 3,
		.num_bits = 1,
	},
};

static int mcbcpld_probe(struct i2c_client *client)
{
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					 ARRAY_SIZE(sysfs_files));
}

static const struct i2c_device_id leh800b_mcbcpld_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, leh800b_mcbcpld_id);

static struct i2c_driver leh800b_mcbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcbcpld_probe,
	.id_table = leh800b_mcbcpld_id,
};
module_i2c_driver(leh800b_mcbcpld_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Evan Zong <ezong@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS MCB CPLD (PWR) Driver");
MODULE_VERSION(BSP_VERSION);
