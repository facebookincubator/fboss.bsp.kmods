// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "fbcpld-cdev.h"
#include "../fboss_iob_led_trigger.h"

#define DRIVER_NAME	"m6060actl_scmcpld"

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

#define I2C_REG_VAL_BITS	8
#define I2C_REG_ADDR_BITS	8
#define I2C_REG_OFFSET_MAX	0xFF

static const struct regmap_config fbcpld_i2c_regmap_cfg = {
	.reg_bits = I2C_REG_ADDR_BITS,
	.val_bits = I2C_REG_VAL_BITS,
	.max_register = I2C_REG_OFFSET_MAX,
};

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

static const struct fbscm_leds_info {
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

static enum led_brightness brightness_get(struct led_classdev *led_cdev)
{
	int ret;
	u8 reg_val;
	struct fbscm_leds_data *ldata = container_of(led_cdev,
					struct fbscm_leds_data, cdev);
	struct fbscm_ctl_leds_data *leddata = i2c_get_clientdata(ldata->client);

	mutex_lock(&leddata->idd_lock);
	ret = i2c_smbus_read_byte_data(ldata->client, ldata->reg_addr);
	mutex_unlock(&leddata->idd_lock);
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

	if (value == LED_OFF) {
		/*
		 * Turn off the led of the specified color.
		 *
		 * All color classdevs of one LED share a single 3-bit color
		 * field in this register, so only one color can be displayed
		 * at a time. Guard the off: only blank the
		 * panel when THIS color is the one currently displayed.
		 * Otherwise leave the register untouched, so turning off a
		 * color that is not currently shown does not overwrite and
		 * turn off whatever other color is on.
		 */
		if ((ret & ldata->bitsmask) == ldata->reg_val)
			reg_val = (u8)((ret & (~ldata->bitsmask))
					| FBSCM_CTL_LED_OFF);
		else
			reg_val = (u8)ret;
	} else {
		/* Turn on this specified color led (single color wins). */
		reg_val = (u8)((ret & (~ldata->bitsmask))
				| ldata->reg_val);
	}

	/* Only write the register when the value actually changes. */
	if (reg_val != (u8)ret)
		ret = i2c_smbus_write_byte_data(ldata->client,
						ldata->reg_addr, reg_val);
	else
		ret = 0;

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
				"%s: failed to set default value, ret=%d",
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
				dev_err(dev, "%s: failed to init trigger, ret=%d",
					ldata->name, ret);
		}
	}

	return 0;
}

static const struct i2c_device_id m6060actl_scmcpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, m6060actl_scmcpld_id);

static int scmcpld_probe(struct i2c_client *client)
{
	int ret;
	struct regmap *regmap;
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

	regmap = devm_regmap_init_i2c(client, &fbcpld_i2c_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "failed to init regmap: %ld\n",
			PTR_ERR(regmap));
		mutex_destroy(&leddata->idd_lock);
		return PTR_ERR(regmap);
	}

	ret = fbcpld_cdev_init(client);
	if (ret) {
		dev_err(&client->dev, "failed to init cdev: %d\n", ret);
		mutex_destroy(&leddata->idd_lock);
		return ret;
	}

	return 0;
}

static void scmcpld_remove(struct i2c_client *client)
{
	struct fbscm_ctl_leds_data *data = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < SCM_CTL_LEDS_NUM * FBSCM_LED_COLOR_NUM; i++) {
		struct fbscm_leds_data *ldata = &data->leds[i];

		led_trigger_deinit(ldata->cdev.dev);
	}

	mutex_destroy(&data->idd_lock);
}

static struct i2c_driver m6060actl_scmcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = scmcpld_probe,
	.remove = scmcpld_remove,
	.id_table = m6060actl_scmcpld_id,
};
module_i2c_driver(m6060actl_scmcpld_driver);

MODULE_AUTHOR("Brandon Chuang <brandon_chuang@accton.com>");
MODULE_DESCRIPTION("Meta FBOSS m6060actl SCM CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
