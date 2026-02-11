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

#define DRIVER_NAME	"tahan_smbcpld"

/*
 * ASIC Max and Min VTMON Values registers
 */
#define SMB_REG_MAX_TEMP_CLK_H_BYTE	0xb8
#define SMB_REG_MAX_TEMP_CLK_L_BYTE	0xb9
#define SMB_REG_MIN_TEMP_CLK_H_BYTE	0xba
#define SMB_REG_MIN_TEMP_CLK_L_BYTE	0xbb

#define SMB_REG_ASIC_OVER_TEMP_CTRL	0xbc
/*
 * ASIC OVER TEMP SETTING HIGH&LOW BYTES registers,
 * default value (2A57) Hex = (10839)Decimal,
 * which is 110 degrees Celsius
 */
#define SMB_REG_ASIC_OVER_TEMP_H_BYTE	0xbd
#define SMB_REG_ASIC_OVER_TEMP_L_BYTE	0xbe

#define ESMBREGINVAL 0xFFFF

struct smb_hwmon_data {
	struct device *hwmon_dev;
	struct i2c_client *client;
};

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "board_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 0,
		.num_bits = 3,
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
		.num_bits = 8,
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
		.name = "th5_pwr_en",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = 8,
		.bit_offset = 0,
		.num_bits = 1,
	},
	{
		.name = "asic_ot_status_changed",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = SMB_REG_ASIC_OVER_TEMP_CTRL,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "asic_ot_status",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = SMB_REG_ASIC_OVER_TEMP_CTRL,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "asic_ot_protect_en",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = SMB_REG_ASIC_OVER_TEMP_CTRL,
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

	temp = 476359 - 3971300000 / temp_var;

	return temp;
}

/*
 * temp data from max measure data based on HW design.
 */
static int smbcpld_read(struct device *dev,
		enum hwmon_sensor_types type, u32 attr, int channel, long *val)
{
	struct smb_hwmon_data *temp_data = dev_get_drvdata(dev);
	u8 reg_h, reg_l;

	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_input:
		reg_h = SMB_REG_MAX_TEMP_CLK_H_BYTE;
		reg_l = SMB_REG_MAX_TEMP_CLK_L_BYTE;
		break;
	case hwmon_temp_max_hyst:
		reg_h = SMB_REG_ASIC_OVER_TEMP_H_BYTE;
		reg_l = SMB_REG_ASIC_OVER_TEMP_L_BYTE;
		break;
	default:
		return -EOPNOTSUPP;
	}

	*val = input_temp_read(temp_data, reg_h, reg_l);
	if (*val == -ESMBREGINVAL)
		return -EINVAL;

	return 0;
}

/*
 * update threshold register value.
 */
static int smbcpld_write(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, long val)
{
	struct smb_hwmon_data *temp_data = dev_get_drvdata(dev);
	u16 freq;
	u8 regval_h, regval_l;
	int ret;

	if (type != hwmon_temp || attr != hwmon_temp_max_hyst)
		return -EOPNOTSUPP;

	/*
	 * The formula for calculating the register value from temperature is:
	 * Freq[Hz] = 317704*25 / (2*(476.359 - Temp[C]))
	 * The 'val' parameter is in degrees Celsius.
	 */
	if (val >= 476)
		return -EINVAL;

	freq = 7942600 / (952 - 2 * val);

	/*
	 * The register value is a 16-bit value,
	 * so we need to split it into two bytes.
	 */
	regval_h = (freq >> 8) & 0xFF;
	regval_l = freq & 0xFF;
	ret = i2c_smbus_write_byte_data(temp_data->client,
			SMB_REG_ASIC_OVER_TEMP_H_BYTE, regval_h);
	if (ret < 0)
		return -EIO;

	ret = i2c_smbus_write_byte_data(temp_data->client,
			SMB_REG_ASIC_OVER_TEMP_L_BYTE, regval_l);
	if (ret < 0)
		return -EIO;

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
		case hwmon_temp_max_hyst:
			mode = 0644;
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
	.write = smbcpld_write,
};

static const struct hwmon_channel_info *smb_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_MAX_HYST),
	NULL
};

static const struct hwmon_chip_info smbcpld_chip_info = {
	.ops = &smbcpld_hwmon_ops,
	.info = smb_info,
};

static const struct i2c_device_id tahan_smbcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tahan_smbcpld_id);

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

	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					ARRAY_SIZE(sysfs_files));
}

static struct i2c_driver tahan_smbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = smbcpld_probe,
	.id_table = tahan_smbcpld_id,
};
module_i2c_driver(tahan_smbcpld_driver);

MODULE_AUTHOR("Edward Zhong <edzhong@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS SMB CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
