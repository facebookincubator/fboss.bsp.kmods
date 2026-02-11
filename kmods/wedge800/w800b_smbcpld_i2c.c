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

#define DRIVER_NAME	"w800b_smbcpld"

#define FBSMB_BOARD_VER_TYPE 0x00
#define FBSMB_MAJOR_VER 0x01
#define FBSMB_MINOR_VER 0x02
#define FBSMB_SUB_VER 0x03
#define FBSMB_MAC_MISC 0xC4
#define FBSMB_SYSTEM_PWR_CTRL 0x08
/*
 * ASIC Max and Min VTMON Values registers
 */
#define SMB_REG_MAX_TEMP_CLK_H_BYTE	0xb8
#define SMB_REG_MAX_TEMP_CLK_L_BYTE	0xb9

#define ESMBREGINVAL 0xFFFF

struct smb_hwmon_data {
	struct device *hwmon_dev;
	struct i2c_client *client;
};

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSMB_BOARD_VER_TYPE,
		.bit_offset = 4,
		.num_bits = 4,
	},
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSMB_BOARD_VER_TYPE,
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
		.name = "th5_pwr_en",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = FBSMB_SYSTEM_PWR_CTRL,
		.bit_offset = 0,
		.num_bits = 1,
	}
};

/*
 * SMB CPLD measure ASIC chip temperature function
 */
static int input_temp_read(struct smb_hwmon_data *data, u8 reg_h, u8 reg_l)
{
	u8 reg_temp_h;
	u8 reg_temp_l;
	u16 temp_var;
	int temp;

	reg_temp_h = i2c_smbus_read_byte_data(data->client, reg_h);
	reg_temp_l = i2c_smbus_read_byte_data(data->client, reg_l);
	if (reg_temp_h < 0 || reg_temp_l < 0) {
		dev_err(&data->client->dev,
			"Read register error, reg_h=0x%x,reg_l=0x%x\n",
			reg_temp_h, reg_temp_l);
		return -ESMBREGINVAL;
	}
	/*
	 * BCM switch ASIC chip temperature monitor,
	 * the calculation formula comes from the chip specification.
	 * Temp(℃) value amplified 1000 times output.
	 * User space usually performs corresponding conversions to follow pmbus protocol.
	 * 𝑇𝑒𝑚𝑝[𝐶]=476.359−(0.317704×25/(2×𝐹𝑟𝑒𝑞[𝐻𝑧]×10^(-6)))
	 */
	temp_var = (reg_temp_h << 8) + reg_temp_l;
	// check the special case, the register value is 0 return error code.
	if (temp_var == 0)
		return -ESMBREGINVAL;

	temp = 476359 - ((317704 * 25) / (temp_var * 2)) * 1000;

	return temp;
}

/*
 * temp data from max measure data based on HW design.
 */
static int smbcpld_read(struct device *dev,
		enum hwmon_sensor_types type, u32 attr, int channel, long *val)
{
	struct smb_hwmon_data *temp_data = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_temp_input:
		*val = input_temp_read(temp_data,
			SMB_REG_MAX_TEMP_CLK_H_BYTE, SMB_REG_MAX_TEMP_CLK_L_BYTE);
		if (*val == -ESMBREGINVAL)
			return -EINVAL;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static umode_t smbcpld_is_visible(const void *_data,
				enum hwmon_sensor_types type, u32 attr, int channel)
{
	umode_t mode = 0;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			mode = 0444;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return mode;
}

static const struct hwmon_ops smbcpld_hwmon_ops = {
	.is_visible = smbcpld_is_visible,
	.read = smbcpld_read,
};

static const struct hwmon_channel_info *smb_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_chip_info smbcpld_chip_info = {
	.ops = &smbcpld_hwmon_ops,
	.info = smb_info,
};
static const struct i2c_device_id w800b_smbcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, w800b_smbcpld_id);

#if KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE
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
	data->hwmon_dev = devm_hwmon_device_register_with_info(&client->dev,
				client->name, data, &smbcpld_chip_info, NULL);
	if (IS_ERR(data->hwmon_dev))
		return PTR_ERR(data->hwmon_dev);

	/* Register sysfs hooks */
	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver w800b_smbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = smbcpld_probe,
	.id_table = w800b_smbcpld_id,
};
module_i2c_driver(w800b_smbcpld_driver);

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com>");
MODULE_DESCRIPTION("Meta FBOSS Wedge800b SMB CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
