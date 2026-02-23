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

#define DRIVER_NAME	"w800_mcbcpld"

#define FBMCB_BOARD_TYPE_VER 0x00
#define FBMCB_BOARD_TV_VER_ID_OFF 4
#define FBMCB_BOARD_TV_VER_ID_MSK 3

#define FBMCB_MAJOR_VER 0x01
#define FBMCB_MINOR_VER 0x02
#define FBMCB_SUB_VER 0x03
#define FBMCB_MAIN_PWR_STA 0x11
#define FBMCB_PWRBRK_HS_FAULT_INTR_STA 0x6C
#define FBMCB_SYS_SUB_DESIGN_STA 0xB5

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_BOARD_TYPE_VER,
		.bit_offset = 4,
		.num_bits = 3,
	},
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_BOARD_TYPE_VER,
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
		.reg_addr = FBMCB_MAJOR_VER,
		.bit_offset = 0,
		.num_bits = 7,
	},
	{
		.name = "cpld_minor_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_MINOR_VER,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "cpld_sub_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SUB_VER,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "pwr_brick_pwrok",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_MAIN_PWR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "hotswap_pwrok",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_MAIN_PWR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},

	{
		.name = "scm_module_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYS_SUB_DESIGN_STA,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "smb_pwrok",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYS_SUB_DESIGN_STA,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "iob_pwrok",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYS_SUB_DESIGN_STA,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "come_pch_pwrok",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYS_SUB_DESIGN_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "come_pwrok",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYS_SUB_DESIGN_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},

	/*
	 * Register 0x6C: Power Brick and Hotswap Fault Status Register
	 * [Logic: 1 = Normal, 0 = Fault/Alert]
	 *
	 * - hotswap_cpld_fault:
	 * Hotswap fault (Overcurrent or FET_BAD).
	 *
	 * - pwr_brick_cpld_alert:
	 * Power Brick alert (OV, OC, UV, OT, or FET_BAD).
	 * (OV: Over-Voltage, OC: Over-Current, UV: Under-Voltage, OT: Over-Temperature)
	 *
	 * - hotswap_cpld_iout_oc_status:
	 * Hotswap Output Over-Current (OC) condition.
	 */
	{
		.name = "hotswap_cpld_fault",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_PWRBRK_HS_FAULT_INTR_STA,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "pwr_brick_cpld_alert",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_PWRBRK_HS_FAULT_INTR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "hotswap_cpld_iout_oc_status",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_PWRBRK_HS_FAULT_INTR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},
};

static const struct i2c_device_id w800_mcbcpld_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, w800_mcbcpld_id);

#if KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE
static int mcbcpld_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
#else
static int mcbcpld_probe(struct i2c_client *client)
#endif
{
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
								 ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver w800_mcbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcbcpld_probe,
	.id_table = w800_mcbcpld_id,
};
module_i2c_driver(w800_mcbcpld_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com>");
MODULE_DESCRIPTION("Meta FBOSS Wedge800 MCB Power CPLD Driver");
MODULE_VERSION(BSP_VERSION);
