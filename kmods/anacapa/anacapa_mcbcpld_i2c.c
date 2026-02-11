// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * MCBCPLD I2C Slave Module @addr 0x60, mainly for power control.
 * Refer to FANCPLD for MCB CPLD fan control component.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/version.h>

#include "regbit-sysfs.h"
#include "../fboss_iob_led_trigger.h"

#define DRIVER_NAME	"anacapa_mcbcpld"

/* Led controller register address */
#define FBMCB_QSFP1_LED_CTRL    0x18
#define FBMCB_QSFP2_LED_CTRL    0x19

/* Leds controller register values */
#define FBMCB_CTL_LED_RED       0x83
#define FBMCB_CTL_LED_GRN       0x85
#define FBMCB_CTL_LED_AMBER     0x81
#define FBMCB_CTL_LED_BLUE      0x86
#define FBMCB_CTL_LED_OFF       0x87
#define FBMCB_CTL_REG_LED_MASK  0b10010111

/*
 * Total 2 system level pannel leds controlled by mcb cpld: qsfp1, qsfp2
 */
#define MCB_CTL_LEDS_NUM        2

/* Led controller color type number supported */
#define FBMCB_LED_COLOR_NUM     4

static const struct {
	u8 reg_val;
	const char *color;
} led_colors_info[FBMCB_LED_COLOR_NUM] = {
	{
		.reg_val = FBMCB_CTL_LED_RED,
		.color = "red",
	},
	{
		.reg_val = FBMCB_CTL_LED_GRN,
		.color = "green",
	},
	{
		.reg_val = FBMCB_CTL_LED_BLUE,
		.color = "blue",
	},
	{
		.reg_val = FBMCB_CTL_LED_AMBER,
		.color = "amber",
	},
};

struct fbmcb_led_info {
	const char name[NAME_MAX];
	const u8 reg_addr;
	const u8 bitsmask;
} leds_table[MCB_CTL_LEDS_NUM] = {
	{
		.name = "port1_led1",
		.reg_addr = FBMCB_QSFP1_LED_CTRL,
		.bitsmask = FBMCB_CTL_REG_LED_MASK,
	},
	{
		.name = "port2_led1",
		.reg_addr = FBMCB_QSFP2_LED_CTRL,
		.bitsmask = FBMCB_CTL_REG_LED_MASK,
	},
};

struct fbmcb_led_data {
	char name[NAME_MAX];
	struct led_classdev cdev;
	struct i2c_client *client;
	u8 reg_addr;
	u8 reg_val;
	u8 bitsmask;
};

struct fbmcb_ctl_leds_data {
	struct device *dev;
	struct i2c_client *client;
	struct mutex idd_lock;
	struct fbmcb_led_data leds_data[MCB_CTL_LEDS_NUM * FBMCB_LED_COLOR_NUM];
};

static const struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "version_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
		.bit_offset = 4,
		.num_bits = 4,
	},
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0,
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
		.name = "oob_eeprom_wp",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = 6,
		.bit_offset = 2,
		.num_bits = 1,
	},
	{
		.name = "oob_eeprom_sel",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = 6,
		.bit_offset = 3,
		.num_bits = 1,
	},
	{
		.name = "smbl_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0A,
		.bit_offset = 1,
		.num_bits = 1,
	},
	{
		.name = "hs_48v_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0A,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "rtml_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0B,
		.bit_offset = 4,
		.num_bits = 1,
	},
	{
		.name = "rtmr_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0B,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "smbr_pg",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x0B,
		.bit_offset = 6,
		.num_bits = 1,
	},
	{
		.name = "smbl_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x10,
		.bit_offset = 5,
		.num_bits = 1,
	},
	{
		.name = "smbr_present",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = 0x12,
		.bit_offset = 3,
		.num_bits = 1,
	},
};

static enum led_brightness brightness_get(struct led_classdev *led_cdev)
{
	int ret;
	u8 reg_val;
	struct fbmcb_led_data *ldata = container_of(led_cdev,
					struct fbmcb_led_data, cdev);

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
	struct fbmcb_led_data *ldata = container_of(led_cdev,
					struct fbmcb_led_data, cdev);
	struct fbmcb_ctl_leds_data *leddata = i2c_get_clientdata(ldata->client);

	mutex_lock(&leddata->idd_lock);
	ret = i2c_smbus_read_byte_data(ldata->client, ldata->reg_addr);
	if (ret < 0)
		goto exit;

	/* turn on this specified color led. */
	if (value == LED_ON) {
		reg_val = (u8)((ret & (~ldata->bitsmask))
				| ldata->reg_val);
	} else {
		/*
		 * Turn off the led of the specified color.
		 * If the color is not turned on, no action will be taken.
		 */
		if ((ret & ldata->bitsmask) == ldata->reg_val) {
			reg_val = (u8)((ret & (~ldata->bitsmask))
					| FBMCB_CTL_LED_OFF);
		} else {
			reg_val = ret;
		}
	}
	ret = i2c_smbus_write_byte_data(ldata->client, ldata->reg_addr, reg_val);

exit:
	mutex_unlock(&leddata->idd_lock);
	return ret;
}

static int front_panel_qsfp_leds_init(struct fbmcb_ctl_leds_data *leddata)
{
	int i, ln, ret;
	int led_data_num = 0;
	struct i2c_client *client = leddata->client;
	struct device *dev = &client->dev;
	struct fbmcb_led_data *ldata;
	u8 reg_val;
	u8 reg_addr;

	/* Note: "ln" means led number. */
	for (ln = 0; ln < ARRAY_SIZE(leds_table); ln++) {
		reg_addr = leds_table[ln].reg_addr;

		ret = i2c_smbus_read_byte_data(client, reg_addr);
		if (ret < 0)
			return ret;

		reg_val = (ret & (~leds_table[ln].bitsmask))
			| FBMCB_CTL_LED_BLUE;

		/* Control the LEDs by default blue color. */
		ret = i2c_smbus_write_byte_data(client, reg_addr, reg_val);
		if (ret)
			dev_err(dev,
				"%s: failed to set default value, ret=%d",
				leds_table[ln].name, ret);

		/* Note: "i" means color type number */
		for (i = 0; i < ARRAY_SIZE(led_colors_info); i++) {
			ldata = &leddata->leds_data[led_data_num++];

			ldata->client = client;
			ldata->reg_addr = reg_addr;
			ldata->reg_val = led_colors_info[i].reg_val;
			ldata->bitsmask = leds_table[ln].bitsmask;
			snprintf(ldata->name, sizeof(ldata->name),
				"%s:%s:status", leds_table[ln].name, led_colors_info[i].color);

			/* Prepare parameters of the correspoding charactor device */
			ldata->cdev.name = ldata->name;
			ldata->cdev.brightness_get = brightness_get;
			ldata->cdev.brightness_set_blocking = brightness_set;
			ldata->cdev.max_brightness = 1;

			ret = devm_led_classdev_register(dev, &ldata->cdev);
			if (ret)
				return ret;

			ret = led_trigger_init(ldata->cdev.dev);
			if (ret)
				dev_err(dev, "%s:%s failed to init trigger, ret=%d",
					leds_table[ln].name, led_colors_info[i].color, ret);
		}
	}

	return 0;
}

static void front_panel_qsfp_leds_deinit(struct fbmcb_ctl_leds_data *leddata)
{
	int i;
	struct fbmcb_led_data *ldata;

	/* Note: "i" walks through all slots of leds data array. */
	for (i = 0; i <  ARRAY_SIZE(leddata->leds_data); i++) {
		ldata = &leddata->leds_data[i];
		led_trigger_deinit(ldata->cdev.dev);
	}
}

static int mcbcpld_probe(struct i2c_client *client)
{
	int ret;
	struct fbmcb_ctl_leds_data *leddata;

	leddata = devm_kzalloc(&client->dev, sizeof(*leddata), GFP_KERNEL);
	if (!leddata)
		return -ENOMEM;

	mutex_init(&leddata->idd_lock);

	i2c_set_clientdata(client, leddata);
	leddata->client = client;

	/* initialize LEDs of qsfp1 and qsfp2 */
	ret = front_panel_qsfp_leds_init(leddata);
	if (ret) {
		dev_err(&client->dev,
			"failed to init led controllers, ret=%d", ret);
		mutex_destroy(&leddata->idd_lock);
		return ret;
	}

	ret = regbit_sysfs_init_i2c(&client->dev, sysfs_files,
					 ARRAY_SIZE(sysfs_files));
	if (ret) {
		dev_err(&client->dev,
			"failed to init regbit sysfs, ret=%d", ret);
		front_panel_qsfp_leds_deinit(leddata);
		mutex_destroy(&leddata->idd_lock);
		return ret;
	}

	return 0;
}

static void mcbcpld_remove(struct i2c_client *client)
{
	struct fbmcb_ctl_leds_data *leddata = i2c_get_clientdata(client);

	front_panel_qsfp_leds_deinit(leddata);
	mutex_destroy(&leddata->idd_lock);
}

static const struct i2c_device_id anacapa_mcbcpld_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, anacapa_mcbcpld_id);

static struct i2c_driver anacapa_mcbcpld_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcbcpld_probe,
	.remove = mcbcpld_remove,
	.id_table = anacapa_mcbcpld_id,
};
module_i2c_driver(anacapa_mcbcpld_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yves Wang <yves.wang@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS MCB CPLD (PWR/LED) Driver");
MODULE_VERSION(BSP_VERSION);
