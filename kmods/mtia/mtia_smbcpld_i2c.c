// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * mtia_smbcpld1.c
 *
 * SMB CPLD1 I2C Slave Module @addr 0x35, mainly for system management.
 *
 * Refer to "scmcpld.c" SCM CPLD from fboss MP3.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/leds.h>

#include "regbit-sysfs.h"
#include "../fboss_iob_led_trigger.h"

#define DRIVER_NAME	"mtia_smbcpld"

//Led controller
#define FBSCM_FP_SYS_LED_CTRL 0x16
#define FBSCM_FP_STA_LED_CTRL 0x17

#define FBSCM_CTL_LED_RED		0x06
#define FBSCM_CTL_LED_GRN		0x05
#define FBSCM_CTL_LED_AMBER		0x04
#define FBSCM_CTL_LED_BLUE		0x03
#define FBSCM_CTL_LED_OFF		0x07
#define FBSCM_CTL_REG_LED_MASK		0b00000111

#define SCM_CTL_LEDS_NUM		2 /* total 2 leds on smb cpld1, system, sys_sta */
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
		.reg_addr = FBSCM_FP_SYS_LED_CTRL,
		.bitsmask = FBSCM_CTL_REG_LED_MASK,
	},
	{
		.name = "sta_led",
		.reg_addr = FBSCM_FP_STA_LED_CTRL,
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
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 0,
		.num_bits = 4,
	},
	{
		.name = "board_id",
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
};

/*
 * LEDs device operation functions.
 */
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
			reg_val = (u8)(ret & (~ldata->bitsmask))
					| FBSCM_CTL_LED_OFF;
		else
			reg_val = ret;

	ret = i2c_smbus_write_byte_data(ldata->client, ldata->reg_addr,
					reg_val);

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
				"failed to set %s default value, ret=%d",
				leds_table[ln].name, ret);
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

static const struct i2c_device_id mtia_smbcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mtia_smbcpld_id);

#if KERNEL_VERSION(6, 3, 0) > LINUX_VERSION_CODE
static int mtia_smbcpld_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
#else
static int mtia_smbcpld_probe(struct i2c_client *client)
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
static int mtia_smbcpld_remove(struct i2c_client *client)
#else
static void mtia_smbcpld_remove(struct i2c_client *client)
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

static struct i2c_driver mtia_smbcpld_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mtia_smbcpld_probe,
	.remove = mtia_smbcpld_remove,
	.id_table = mtia_smbcpld_id,
};
module_i2c_driver(mtia_smbcpld_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lucas Liu <lucli@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS MTIA Switch Blades SMB CPlD1 Driver");
MODULE_VERSION(BSP_VERSION);
