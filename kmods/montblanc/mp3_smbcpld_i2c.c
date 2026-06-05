// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/math64.h> /* For div_s64 */

#include "regbit-sysfs.h"

#define DRIVER_NAME	"mp3_smbcpld"

/*
 * ASIC Max and Min VTMON Values registers (RO)
 */
#define SMB_REG_MAX_TEMP_CLK_H_BYTE	0xb8
#define SMB_REG_MAX_TEMP_CLK_L_BYTE	0xb9
#define SMB_REG_MIN_TEMP_CLK_H_BYTE	0xba
#define SMB_REG_MIN_TEMP_CLK_L_BYTE	0xbb

#define SMB_REG_VR_ALERT_MASK		0x18

struct smb_hwmon_data {
	struct device *hwmon_dev;
	struct i2c_client *client;
};

/*
 * Regbit Sysfs Configuration (Static Mapping)
 */

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

	/* --- Register 0x0c: SYSTEM_PWR_Status_4_Misc (RO) --- */
	{
		.name = "xp3p3_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 7,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_smb_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_clk_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_l_osfp_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_r_osfp_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "xp1p8_clk_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "xp1p8_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "xp1p2_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0c,
		.bit_offset = 0,
		.num_bits = 1,
	},

	/*
	 * --- Register 0x16: SMB VR Alert INT ---
	 * WARNING: This register is Read-Clear (RC). Reading any single bit via sysfs
	 * will cause the I2C bus to read the full byte, subsequently clearing ALL
	 * interrupt bits in this hardware register.
	 */
	{
		.name = "vr_alert_xp12r0v_scm_int",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x16,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_xp3r3v_left_int",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x16,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_xp3r3v_right_int",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x16,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_th5_trvdd_1_int",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x16,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_th5_trvdd_0_int",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x16,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_th5_vdd_core_int",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x16,
		.bit_offset = 0,
		.num_bits = 1,
	},

	/*
	 * --- Register 0x17: SMB VR Alert Status ---
	 * This register is Read-Only (RO). Polling these bits is perfectly safe
	 * and will yield the live status of the power rails.
	 */
	{
		.name = "vr_alert_xp12r0v_scm_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x17,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_xp3r3v_left_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x17,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_xp3r3v_right_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x17,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_th5_trvdd_1_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x17,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_th5_trvdd_0_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x17,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "vr_alert_th5_vdd_core_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x17,
		.bit_offset = 0,
		.num_bits = 1,
	},
};


/*
 * This function implements the formula: 476359 - (3971300000 / temp_var)
 * using kernel-safe and portable methods.
 *
 * The calculation is performed using 64-bit arithmetic to prevent overflow
 * from the large constant, referenced from MTIA Janga implementation.
 */
static inline s64 calculate_sensor_value(s32 temp_var)
{
	const s64 OFFSET = 476359LL;
	const s64 SCALE_FACTOR = 3971300000LL;

	/*
	 * CRITICAL: Always check for division by zero in kernel code.
	 */
	if (temp_var == 0)
		return -EINVAL;

	return OFFSET - div_s64(SCALE_FACTOR, temp_var);
}

/*
 * SMB CPLD measure ASIC chip temperature function
 */
static int input_temp_read(struct smb_hwmon_data *data, u8 reg_h, u8 reg_l, long *val)
{
	s32 reg_temp_h, reg_temp_l;
	u16 temp_var;
	s64 res;

	reg_temp_h = i2c_smbus_read_byte_data(data->client, reg_h);
	if (reg_temp_h < 0)
		return reg_temp_h;

	reg_temp_l = i2c_smbus_read_byte_data(data->client, reg_l);
	if (reg_temp_l < 0)
		return reg_temp_l;

	temp_var = (reg_temp_h << 8) + reg_temp_l;
	res = calculate_sensor_value(temp_var);

	*val = (long)res;
	return 0;
}

static int smbcpld_read(struct device *dev,
		enum hwmon_sensor_types type, u32 attr, int channel, long *val)
{
	struct smb_hwmon_data *temp_data = dev_get_drvdata(dev);

	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_input:
		return input_temp_read(temp_data,
			SMB_REG_MAX_TEMP_CLK_H_BYTE, SMB_REG_MAX_TEMP_CLK_L_BYTE, val);
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

	if (type == hwmon_temp) {
		switch (attr) {
		case hwmon_temp_input:
			mode = 0444; /* Read Only */
			break;
		default:
			break;
		}
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

static const struct i2c_device_id mp3_smbcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mp3_smbcpld_id);


static int smbcpld_probe(struct i2c_client *client)
{
	struct smb_hwmon_data *data;
	int ret;

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
	ret = regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					ARRAY_SIZE(sysfs_files));
	if (ret)
		return ret;

	/*
	 * Unmask VR Alert Interrupts by default.
	 * Writing 0x00 to Register 0x18 enables bits [5:0] so that the INT
	 * register (0x16) and STA register (0x17) can accurately reflect
	 * the real-time hardware status when polled via sysfs.
	 */
	ret = i2c_smbus_write_byte_data(client, SMB_REG_VR_ALERT_MASK, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to unmask VR Alert Register: %d\n", ret);
	}

	return 0;
}

static struct i2c_driver mp3_smbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = smbcpld_probe,
	.id_table = mp3_smbcpld_id,
};
module_i2c_driver(mp3_smbcpld_driver);

MODULE_AUTHOR("Edward Zhong <edzhong@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS Minipack3 SMB CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
