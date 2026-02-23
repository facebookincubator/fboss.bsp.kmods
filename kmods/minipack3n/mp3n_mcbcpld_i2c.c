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

#define DRIVER_NAME	"mp3n_mcbcpld"

#define FBMCB_BOARD_TYPE_VER 0x00
#define FBMCB_BOARD_TV_VER_ID_OFF 4
#define FBMCB_BOARD_TV_VER_ID_MSK 3

#define FBMCB_MAJOR_VER 0x01
#define FBMCB_MINOR_VER 0x02
#define FBMCB_SUB_VER 0x03
#define FBMCB_PSU_PWR_MAIN_ENABLE 0x10
#define FBMCB_PSU_PWR_STATUS 0x11
#define FBMCB_SYS_SUB_DESIGN_STA 0xB5

#define FBMCB_CPLD_SCRATCH 0x04
#define FBMCB_CPLD_S_SOFT_SCRATCH_OFF 0
#define FBMCB_CPLD_S_SOFT_SCRATCH_MSK 8

#define FBMCB_TIMER_BASE_SET 0x20
#define FBMCB_TIMER_BS_TIME_BASE_10S_OFF 3
#define FBMCB_TIMER_BS_TIME_BASE_10S_MSK 1
#define FBMCB_TIMER_BS_TIME_BASE_1S_OFF 2
#define FBMCB_TIMER_BS_TIME_BASE_1S_MSK 1

#define FBMCB_TIMER_COUNTER_SET 0x21
#define FBMCB_TIMER_CS_TIMER_COUNTER_OFF 0
#define FBMCB_TIMER_CS_TIMER_COUNTER_MSK 8

#define FBMCB_TIMER_MISC 0x23
#define FBMCB_TIMER_M_TIMER_COUNT_SET_UPDT_OFF 1
#define FBMCB_TIMER_M_TIMER_COUNT_SET_UPDT_MSK 1
#define FBMCB_TIMER_M_POWER_CYCLE_GO_OFF 0
#define FBMCB_TIMER_M_POWER_CYCLE_GO_MSK 1

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
		.name = "psu_l_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_PSU_PWR_MAIN_ENABLE,
		.bit_offset = 6,
		.num_bits = 1,
		.flags = RBS_FLAG_VAL_NEGATE,
	},
	{
		.name = "psu_r_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_PSU_PWR_MAIN_ENABLE,
		.bit_offset = 7,
		.num_bits = 1,
		.flags = RBS_FLAG_VAL_NEGATE,
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
		.name = "come_pwrok",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_SYS_SUB_DESIGN_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},

	/*
	 * Register 0x11: MCB SYSTEM MISC-2 Control : PSU PWR Status
	 * 1: PSU Normal
	 * 0: PSU (high temperature) Alert
	 */
	{
		.name = "pdb_psu_r_alert",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_PSU_PWR_STATUS,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "pdb_psu_l_alert",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBMCB_PSU_PWR_STATUS,
		.bit_offset = 0,
		.num_bits = 1,
	},
};

static const struct i2c_device_id mp3n_mcbcpld_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mp3n_mcbcpld_id);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int mcbcpld_probe(struct i2c_client *client,
						 const struct i2c_device_id *id)
#else
static int mcbcpld_probe(struct i2c_client *client)
#endif
{
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
								 ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver mp3n_mcbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcbcpld_probe,
	.id_table = mp3n_mcbcpld_id,
};
module_i2c_driver(mp3n_mcbcpld_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com>");
MODULE_DESCRIPTION("Meta FBOSS Minipack3n MCB Power CPLD Driver");
MODULE_VERSION(BSP_VERSION);
