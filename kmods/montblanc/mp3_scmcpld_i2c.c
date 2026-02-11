// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "regbit-sysfs.h"
#include "../fboss_iob_led_trigger.h"

#define DRIVER_NAME	"mp3_scmcpld"

#define FBSCM_BOARD_INFO 0x00
#define FBSCM_MAJOR_VER 0x01
#define FBSCM_MINOR_VER 0x02
#define FBSCM_SUB_VER 0x03
#define FBSCM_COME_STA_MISC_1 0x11
#define FBSCM_COME_STA_MISC_2 0x12
#define FBSCM_COME_STA_MISC_3 0x13
#define FBSCM_SYS_INTR 0x20
#define FBSCM_SYS_INTR_STA 0x21
#define FBSCM_LM75_ADC_INTR 0x23
#define FBSCM_LM75_ADC_INTR_STA 0x24
#define FBSCM_VR_ALERT_INTR 0x26
#define FBSCM_VR_ALERT_INTR_STA 0x27
#define FBSCM_VR_PG_INTR 0x29
#define FBSCM_VR_PG_INTR_STA 0x2A
#define FBSCM_CLK_ASIC_INTR 0x2C
#define FBSCM_CLK_ASIC_INTR_STA 0x2D
#define FBSCM_PRESENT_PWR_INTR 0x2F
#define FBSCM_PRESENT_PWR_INTR_STA 0x30

// Led controller resister address
#define FBSCM_SYS_LED_CTRL_2 0x41
#define FBSCM_SYS_LED_CTRL_3 0x42
#define FBSCM_SYS_LED_CTRL_4 0x43
#define FBSCM_SYS_LED_CTRL_5 0x44

//Leds controller resister values
#define FBSCM_CTL_LED_RED		0x06
#define FBSCM_CTL_LED_GRN		0x05
#define FBSCM_CTL_LED_AMBER		0x04
#define FBSCM_CTL_LED_BLUE		0x03
#define FBSCM_CTL_LED_OFF		0x07
#define FBSCM_CTL_REG_LED_MASK		0b00000111

// total 4 leds devices on front panel: sys, smb, psu, fan
#define SCM_CTL_LEDS_NUM		4

// Led controller color type number supported
#define FBSCM_LED_COLOR_NUM		4

static const struct {
	u8 reg_val;
	const char *color;
} led_color_info[FBSCM_LED_COLOR_NUM] = {
	{
		.reg_val = FBSCM_CTL_LED_BLUE,
		.color = "blue",
	},
	{
		.reg_val = FBSCM_CTL_LED_GRN,
		.color = "green",
	},
	{
		.reg_val = FBSCM_CTL_LED_RED,
		.color = "red",
	},
	{
		.reg_val = FBSCM_CTL_LED_AMBER,
		.color = "amber",
	},
};

struct fbscm_leds_info {
	const char name[NAME_MAX];
	const u8 reg_addr;
	const u8 bitsmask;
} leds_table[SCM_CTL_LEDS_NUM] = {
	{
		.name = "sys_led",
		.reg_addr = FBSCM_SYS_LED_CTRL_5,
		.bitsmask = FBSCM_CTL_REG_LED_MASK,
	},
	{
		.name = "smb_led",
		.reg_addr = FBSCM_SYS_LED_CTRL_2,
		.bitsmask = FBSCM_CTL_REG_LED_MASK
	},
	{
		.name = "psu_led",
		.reg_addr = FBSCM_SYS_LED_CTRL_3,
		.bitsmask = FBSCM_CTL_REG_LED_MASK,
	},
	{
		.name = "fan_led",
		.reg_addr = FBSCM_SYS_LED_CTRL_4,
		.bitsmask = FBSCM_CTL_REG_LED_MASK,
	}
};

struct fbscm_leds_data {
	char name[NAME_MAX];
	struct led_classdev cdev;
	struct i2c_client *client;
	u8 reg_addr;
	u8 reg_val;
	u8 bitsmask;
};

struct fbscm_ctl_leds_data {
	struct device *dev;
	struct i2c_client *client;
	struct mutex idd_lock;
	struct fbscm_leds_data leds[SCM_CTL_LEDS_NUM * FBSCM_LED_COLOR_NUM];
};

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_BOARD_INFO,
		.bit_offset = 0,
		.num_bits = 4,
	},
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_BOARD_INFO,
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
		.reg_addr = FBSCM_MAJOR_VER,
		.bit_offset = 0,
		.num_bits = 7,
	},
	{
		.name = "cpld_minor_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_MINOR_VER,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "cpld_sub_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_SUB_VER,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "come_sta_misc_1",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_COME_STA_MISC_1,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "come_sta_misc_2",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_COME_STA_MISC_2,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "come_sta_misc_3",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_COME_STA_MISC_3,
		.bit_offset = 0,
		.num_bits = 8,
	},
	{
		.name = "lm75_adc_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_SYS_INTR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},
	{
		.name = "scm_hs_alert_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_SYS_INTR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "scm_hs_pwr_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_SYS_INTR_STA,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "lm75_1_intr_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_LM75_ADC_INTR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},
	{
		.name = "lm75_2_intr_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_LM75_ADC_INTR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "adc_intr_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_LM75_ADC_INTR_STA,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "hs_alert1_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_ALERT_INTR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},
	{
		.name = "hs_alert2_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_ALERT_INTR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "hs_fault_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_ALERT_INTR_STA,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "usb_xp5p0_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_ALERT_INTR_STA,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "hs_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_bmc_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_scm_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_i210_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "xp3p3_ssd_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "xp5p0_come_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "xp12p0_come_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "xp1p1_oob_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_VR_PG_INTR_STA,
		.bit_offset = 7,
		.num_bits = 1,
	},
	{
		.name = "pcie_gen3_los_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},
	{
		.name = "pcie_gen4_los_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "mgnt_console_invd_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "dbg_console_prnt_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "oob_phy_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "iob_i2c_rdy_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "bmc_i2c_rdy_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "i210_lan_wake_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_CLK_ASIC_INTR_STA,
		.bit_offset = 7,
		.num_bits = 1,
	},
	{
		.name = "scm_sys_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_PRESENT_PWR_INTR_STA,
		.bit_offset = 0,
		.num_bits = 1,
	},
	{
		.name = "scm_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_PRESENT_PWR_INTR_STA,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "mcb_pg_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_PRESENT_PWR_INTR_STA,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "fan_b_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_PRESENT_PWR_INTR_STA,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "fan_t_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_PRESENT_PWR_INTR_STA,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "bmc_rdy_sta",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FBSCM_PRESENT_PWR_INTR_STA,
		.bit_offset = 5,
		.num_bits = 1,
	},
};

static enum led_brightness brightness_get(struct led_classdev *led_cdev)
{
	int ret;
	u8 reg_val;
	struct fbscm_leds_data *ldata = container_of(led_cdev,
					struct fbscm_leds_data, cdev);

	ret = i2c_smbus_read_byte_data(ldata->client, ldata->reg_addr);
	if (ret < 0)
		return ret;

	reg_val = (u8)(ret & ldata->bitsmask);
	if (reg_val == ldata->reg_val)
		return LED_ON;

	return LED_OFF;
}

static int brightness_set(struct led_classdev *led_cdev,
			  enum led_brightness value)
{
	int ret;
	u8 reg_val;
	struct fbscm_leds_data *ldata = container_of(led_cdev,
					struct fbscm_leds_data, cdev);
	struct fbscm_ctl_leds_data *leddata = i2c_get_clientdata(ldata->client);

	mutex_lock(&leddata->idd_lock);
	ret = i2c_smbus_read_byte_data(ldata->client, ldata->reg_addr);
	if (ret < 0)
		goto exit;

	// turn on this specified color led.
	if (value == LED_ON)
		reg_val = (u8)((ret & (~ldata->bitsmask))
				| ldata->reg_val);
	else
		/*
		 * Turn off the led of the specified color.
		 * If the color is not turned on, no action will be taken.
		 */
		if ((ret & ldata->bitsmask) == ldata->reg_val)
			reg_val = (u8)((ret & (~ldata->bitsmask))
					| FBSCM_CTL_LED_OFF);
		else
			reg_val = ret;

	ret = i2c_smbus_write_byte_data(ldata->client, ldata->reg_addr, reg_val);

exit:
	mutex_unlock(&leddata->idd_lock);

	return ret;
}

static int front_panel_leds_init(struct fbscm_ctl_leds_data *leddata)
{
	int i, ln, ret;
	int led_num = 0;
	struct i2c_client *client = leddata->client;
	struct device *dev = &client->dev;
	u8 reg_val;

	/*
	 * Note: "ln" (leds number).
	 */
	for (ln = 0; ln < SCM_CTL_LEDS_NUM; ln++) {
		u8 reg_addr = leds_table[ln].reg_addr;

		ret = i2c_smbus_read_byte_data(client, reg_addr);
		if (ret < 0)
			return ret;

		reg_val = (ret & (~leds_table[ln].bitsmask))
			| FBSCM_CTL_LED_BLUE;

		/*
		 * Control the LEDs by default blue color.
		 */
		ret = i2c_smbus_write_byte_data(client, reg_addr, reg_val);
		if (ret) {
			dev_err(dev,
				"fan%d_led: failed to set default value, ret=%d",
				ln, ret);
		}

		for (i = 0; i < FBSCM_LED_COLOR_NUM; i++) {
			struct fbscm_leds_data *ldata = &leddata->leds[led_num++];

			ldata->client = client;
			ldata->reg_addr = reg_addr;
			ldata->reg_val = led_color_info[i].reg_val;
			ldata->bitsmask = leds_table[ln].bitsmask;
			snprintf(ldata->name, sizeof(ldata->name),
				"%s:%s:status", leds_table[ln].name, led_color_info[i].color);

			ldata->cdev.name = ldata->name;
			ldata->cdev.brightness_get = brightness_get;
			ldata->cdev.brightness_set_blocking = brightness_set;
			ldata->cdev.max_brightness = 1;

			ret = devm_led_classdev_register(dev, &ldata->cdev);
			if (ret)
				return ret;
			ret = led_trigger_init(ldata->cdev.dev);
			if (ret)
				dev_err(dev, "fan%d_led: failed to init trigger, ret=%d", ln, ret);
		}
	}

	return 0;
}

static const struct i2c_device_id mp3_scmcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mp3_scmcpld_id);

#if KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE
static int scmcpld_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
#else
static int scmcpld_probe(struct i2c_client *client)
#endif
{
	int ret;
	struct fbscm_ctl_leds_data *leddata;

	leddata = devm_kzalloc(&client->dev, sizeof(*leddata), GFP_KERNEL);
	if (!leddata)
		return -ENOMEM;

	mutex_init(&leddata->idd_lock);

	i2c_set_clientdata(client, leddata);
	leddata->client = client;

	ret = front_panel_leds_init(leddata);
	if (ret) {
		dev_err(&client->dev,
			"failed to init led controllers, ret=%d", ret);
		mutex_destroy(&leddata->idd_lock);
		return ret;
	}

	return regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					ARRAY_SIZE(sysfs_files));
}

#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
static int scmcpld_remove(struct i2c_client *client)
#else
static void scmcpld_remove(struct i2c_client *client)
#endif
{
	struct fbscm_ctl_leds_data *data = i2c_get_clientdata(client);

	for (int i = 0; i < SCM_CTL_LEDS_NUM * FBSCM_LED_COLOR_NUM; i++) {
		struct fbscm_leds_data *ldata = &data->leds[i];

		led_trigger_deinit(ldata->cdev.dev);
	}

	mutex_destroy(&data->idd_lock);
#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
	return 0;
#endif
}

static struct i2c_driver mp3_scmcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = scmcpld_probe,
	.remove = scmcpld_remove,
	.id_table = mp3_scmcpld_id,
};
module_i2c_driver(mp3_scmcpld_driver);

MODULE_AUTHOR("Tao Ren <taoren@meta.com>");
MODULE_DESCRIPTION("Meta FBOSS SCM CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
