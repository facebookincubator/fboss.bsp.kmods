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

#define DRIVER_NAME	"th6_smbcpld"

/*
 * ASIC Max VTMON Values registers
 */
#define SMB_REG_MAX_TEMP_CLK_H_BYTE	0xb8
#define SMB_REG_MAX_TEMP_CLK_L_BYTE	0xb9
#define SMB_REG_ASIC_OVER_TEMP_CTRL	0xbc

struct smb_hwmon_data {
	struct device *hwmon_dev;
	struct i2c_client *client;
};

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 0,
		.num_bits = 4,
	},
	{
		.name = "version_id",
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
		.name = "th6_pwr_en",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = 8,
		.bit_offset = 0,
		.num_bits = 1,
	},
	/* asic_ot_status_changed: record ASIC over-temperature status changed flag.
	 * 1: ASIC over-temperature status changed.
	 * 0: ASIC over-temperature status not changed.
	 */
	{
		.name = "asic_ot_status_changed",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = SMB_REG_ASIC_OVER_TEMP_CTRL,
		.bit_offset = 2,
		.num_bits = 1,
	},
	/* asic_ot_status: ASIC real-time over-temperature status.
	 * 1: The real-time ASIC temperature is normal.
	 * 0: The real-time ASIC is in an over-temperature state.
	 */
	{
		.name = "asic_ot_status",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = SMB_REG_ASIC_OVER_TEMP_CTRL,
		.bit_offset = 1,
		.num_bits = 1,
	}
};

/*
 * This function implements the formula: 378850 - (3246000000 / temp_var)
 * using kernel-safe and portable methods.
 *
 * The calculation is performed using 64-bit arithmetic to prevent overflow
 * from the large constant.
 */
static inline s64 calculate_sensor_value(s32 temp_var)
{
	/*
	 * Use named constants for clarity. The 'LL' suffix is crucial to ensure
	 * the compiler treats the number as a 64-bit literal, preventing
	 * overflow on 32-bit architectures.
	 */
	const s64 OFFSET = 378850LL;
	const s64 SCALE_FACTOR = 3246000000LL;

	/*
	 * CRITICAL: Always check for division by zero in kernel code.
	 * Returning a known error value (like the minimum for the type) is a
	 * common pattern.
	 */
	if (temp_var == 0)
		return -EINVAL;

	/*
	 * Use the kernel's 64-bit division helper. It's portable and optimized
	 * for both 32-bit and 64-bit architectures. The subtraction is then
	 * performed safely with 64-bit types.
	 */
	return OFFSET - div_s64(SCALE_FACTOR, temp_var);
}

/*
 * SMB CPLD measure ASIC chip temperature function
 */
static long input_temp_read(struct smb_hwmon_data *data, u8 reg_h, u8 reg_l)
{
	s32 reg_temp_h, reg_temp_l;
	u16 temp_var;


	reg_temp_h = i2c_smbus_read_byte_data(data->client, reg_h);
	reg_temp_l = i2c_smbus_read_byte_data(data->client, reg_l);
	if (reg_temp_h < 0 || reg_temp_l < 0)
		return -EIO;

	/*
	 * BCM switch TH6 chip temperature monitor,
	 * the calculation formula comes from the chip specification.
	 * Temp(℃) value amplified 1000 times output.
	 * User space usually performs corresponding conversions to follow pmbus protocol.
	 * 𝑇𝑒𝑚𝑝[𝐶]=378.85−(0.25968×25/(2×𝐹𝑟𝑒𝑞[𝐻𝑧]×10^(-6)))
	 */
	temp_var = ((u16)reg_temp_h << 8) + (u8)reg_temp_l;

	return calculate_sensor_value(temp_var);
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
		if (*val < 0)
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
static const struct i2c_device_id th6_smbcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, th6_smbcpld_id);

static int smbcpld_probe(struct i2c_client *client)
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

static struct i2c_driver th6_smbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = smbcpld_probe,
	.id_table = th6_smbcpld_id,
};
module_i2c_driver(th6_smbcpld_driver);

MODULE_AUTHOR("Joy Wu <joy.wu@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS SMB CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
