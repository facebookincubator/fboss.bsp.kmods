// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/version.h>

#include "regbit-sysfs.h"

#define DRIVER_NAME	"mp3n_smbcpld"

#define FBSMB_BOARD_VER_TYPE 0x00
#define FBSMB_MAJOR_VER 0x01
#define FBSMB_MINOR_VER 0x02
#define FBSMB_SUB_VER 0x03
#define FBSMB_MAC_MISC 0xC4

struct smb_hwmon_data {
	struct i2c_client *client;
};

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSMB_BOARD_VER_TYPE,
		.bit_offset = 0,
		.num_bits = 4,
	},
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSMB_BOARD_VER_TYPE,
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
		.reg_addr = FBSMB_MAJOR_VER,
		.bit_offset = 0,
		.num_bits = 7,
	},
	{
		.name = "cpld_minor_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSMB_MINOR_VER,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "cpld_sub_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSMB_SUB_VER,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "sp4_pwr_en",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = FBSMB_MAC_MISC,
		.bit_offset = 7,
		.num_bits = 1,
	}
};

static const struct i2c_device_id mp3n_smbcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mp3n_smbcpld_id);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int smbcpld_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
#else
static int smbcpld_probe(struct i2c_client *client)
#endif
{
	struct smb_hwmon_data *data;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);

	data->client = client;

	/* Register sysfs hooks */
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver mp3n_smbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = smbcpld_probe,
	.id_table = mp3n_smbcpld_id,
};
module_i2c_driver(mp3n_smbcpld_driver);

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com>");
MODULE_DESCRIPTION("Meta FBOSS Minipack3n SMB CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
