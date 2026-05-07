// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * Generic FBOSS fan CPLD driver.
 *
 * FANCPLD is a logical component (I2C Slave Module, address 0x33) of
 * MCBCPLD dedicated for fan control.
 *
 * FANCPLD doesn't have its own firmware revision because it's part of
 * MCB CPLD. MCB CPLD firmware revision is reported by mcbcpld.
 *
 * This is a parameterized replacement for per-platform fan CPLD drivers
 * (anacapa, icecube, icetea, mp3, mp3n, tahansb, w800). All share the
 * same register layout; only fan count, PWM range, speed multiplier,
 * tach topology, and LED presence differ. The configuration is supplied
 * from userspace via ioctl on a misc device, similar to fbcpld_generic.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/leds.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>

#include "fboss_iob_led_trigger.h"
#include "lib/fbfancpld-ioctl.h"

#define DRIVER_NAME		"fbfancpld"

#define MAX_FANS		8
#define FAN_LED_COLOR_NUM	2

/* Register layout (fn = 1-based fan number) */
#define REG_FAN_TACH_F(fn)	(0x30 + ((fn) - 1) * 0x10)
#define REG_FAN_TACH_R(fn)	(0x31 + ((fn) - 1) * 0x10)
#define REG_FAN_PWM(fn)		(0x32 + ((fn) - 1) * 0x10)
#define REG_FAN_LED(fn)		(0x34 + ((fn) - 1) * 0x10)
#define REG_FAN_STATUS(fn)	(0x38 + ((fn) - 1) * 0x10)

#define FAN_MODE_BIT_OFFSET	7
#define FAN_MODE_MASK		BIT(7)
#define FAN_PRESENT_MASK	0x01

/* LED register values */
#define FAN_LED_HW_CTRL		0x00
#define FAN_LED_BLUE		0x01
#define FAN_LED_AMBER		0x02
#define FAN_LED_OFF		0x03
#define FAN_LED_REG_MASK	0x03

/* Watchdog registers */
#define WDT_CTRL_0		0x1F
#define WDT_ENABLE		BIT(4)
#define WDT_PET			BIT(5)
#define WDT_CTRL_1		0x20
#define WDT_TIMEOUT_MAX		(0xFF * 5)
#define WDT_STA_REG		0x21
#define WDT_PWM_REG		0x22
#define WDT_DFT_TIMEOUT		(0x3F * 5)

static int wdt_timeout = WDT_DFT_TIMEOUT;
module_param(wdt_timeout, int, 0);
MODULE_PARM_DESC(wdt_timeout,
	"Watchdog timeout in seconds. Default = "
		__MODULE_STRING(WDT_DFT_TIMEOUT));

struct fbfancpld_config {
	int num_fans;
	int pwm_max;
	int speed_multiplier;
	bool has_rear_tach;
	bool has_leds;
};

struct fan_led_data {
	struct led_classdev cdev;
	char name[NAME_MAX];
	struct i2c_client *client;
	u8 reg_addr;
	u8 reg_val;
};

struct fbfancpld_data {
	struct miscdevice miscdev;
	char miscdev_name[NAME_MAX];
	struct mutex lock;
	struct i2c_client *client;
	bool configured;

	struct fbfancpld_config config;

	struct device *hwmon_dev;
	u32 fan_config[MAX_FANS * 2 + 1];
	u32 pwm_config[MAX_FANS + 1];
	struct hwmon_channel_info fan_channel;
	struct hwmon_channel_info pwm_channel;
	const struct hwmon_channel_info *channels[3];
	struct hwmon_chip_info chip_info;

	struct attribute *present_ptrs[MAX_FANS + 1];
	struct attribute_group present_group;
	const struct attribute_group *extra_groups[3];

	struct fan_led_data *leds;
	int num_leds;
};

/* --- Watchdog --- */

static int fan_wdt_start(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, WDT_CTRL_0, WDT_ENABLE);
}

static int fan_wdt_stop(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, WDT_CTRL_0, 0);
}

static int fan_wdt_set_timeout(struct watchdog_device *wdd,
			       unsigned int timeout)
{
	u8 reg;
	struct i2c_client *client = to_i2c_client(wdd->parent);

	wdd->timeout = min_t(unsigned int, timeout, WDT_TIMEOUT_MAX);
	reg = wdd->timeout / 5;

	return i2c_smbus_write_byte_data(client, WDT_CTRL_1, reg);
}

static unsigned int fan_wdt_get_timeleft(struct watchdog_device *wdd)
{
	int max_timeout;
	int counter;
	int ret;
	struct i2c_client *client = to_i2c_client(wdd->parent);

	max_timeout = i2c_smbus_read_byte_data(client, WDT_CTRL_1);
	counter = i2c_smbus_read_byte_data(client, WDT_STA_REG);
	ret = (max_timeout - counter) * 5;
	if (ret < 0)
		ret = 0;
	return ret;
}

static int fan_wdt_ping(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, WDT_CTRL_0,
					 WDT_ENABLE | WDT_PET);
}

static const struct watchdog_ops fan_wdt_ops = {
	.start = fan_wdt_start,
	.stop = fan_wdt_stop,
	.set_timeout = fan_wdt_set_timeout,
	.get_timeleft = fan_wdt_get_timeleft,
	.ping = fan_wdt_ping,
	.owner = THIS_MODULE,
};

static const struct watchdog_info fan_wdt_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT,
	.identity = KBUILD_MODNAME,
};

static int fan_wdt_init(struct i2c_client *client)
{
	struct watchdog_device *wdd;
	struct device *dev = &client->dev;
	int err;

	wdd = devm_kzalloc(dev, sizeof(*wdd), GFP_KERNEL);
	if (!wdd)
		return -ENOMEM;

	wdd->info = &fan_wdt_info;
	wdd->ops = &fan_wdt_ops;
	wdd->parent = dev;
	wdd->timeout = wdt_timeout;
	wdd->max_timeout = WDT_TIMEOUT_MAX;

	watchdog_init_timeout(wdd, wdt_timeout, dev);

	fan_wdt_stop(wdd);
	err = devm_watchdog_register_device(dev, wdd);
	if (err) {
		dev_err(dev, "watchdog_register_device failed, ret=%d\n", err);
		return err;
	}
	return 0;
}

/* --- hwmon --- */

static umode_t fbfancpld_is_visible(const void *_data,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		if (attr == hwmon_fan_input)
			return 0444;
		break;
	case hwmon_pwm:
		if (attr == hwmon_pwm_input || attr == hwmon_pwm_enable)
			return 0644;
		break;
	default:
		break;
	}

	return 0;
}

static int fbfancpld_fan_input_read(struct device *dev, int channel, long *val)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int fn;
	int reg_addr;
	int reg_data;

	if (data->config.has_rear_tach) {
		fn = channel / 2 + 1;
		reg_addr = (channel % 2) ? REG_FAN_TACH_R(fn) :
					   REG_FAN_TACH_F(fn);
	} else {
		fn = channel + 1;
		reg_addr = REG_FAN_TACH_F(fn);
	}

	mutex_lock(&data->lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&data->lock);

	if (reg_data < 0)
		return reg_data;

	*val = (long)reg_data * data->config.speed_multiplier;
	return 0;
}

static int fbfancpld_fan_read(struct device *dev, u32 attr,
			      int channel, long *val)
{
	switch (attr) {
	case hwmon_fan_input:
		return fbfancpld_fan_input_read(dev, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fbfancpld_pwm_input_read(struct device *dev, int channel, long *val)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int fn;
	int reg_addr;
	int reg_data;

	fn = channel + 1;
	reg_addr = REG_FAN_PWM(fn);

	mutex_lock(&data->lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&data->lock);

	if (reg_data < 0)
		return reg_data;

	*val = (reg_data > data->config.pwm_max) ?
		data->config.pwm_max : reg_data;
	return 0;
}

static int fbfancpld_pwm_enable_read(struct device *dev, int channel, long *val)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int fn;
	int reg_addr;
	int reg_data;

	fn = channel + 1;
	reg_addr = REG_FAN_PWM(fn);

	mutex_lock(&data->lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&data->lock);

	if (reg_data < 0)
		return reg_data;

	*val = (reg_data & FAN_MODE_MASK) ? 0 : 1;
	return 0;
}

static int fbfancpld_pwm_read(struct device *dev, u32 attr,
			      int channel, long *val)
{
	switch (attr) {
	case hwmon_pwm_input:
		return fbfancpld_pwm_input_read(dev, channel, val);
	case hwmon_pwm_enable:
		return fbfancpld_pwm_enable_read(dev, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fbfancpld_pwm_input_write(struct device *dev, int channel, long val)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int byte_val;
	int fn;
	int err;
	int reg_addr;
	int reg_data;
	int target_pwm;

	if (val < 0)
		return -EINVAL;

	target_pwm = (val > data->config.pwm_max) ?
		      data->config.pwm_max : val;

	fn = channel + 1;
	reg_addr = REG_FAN_PWM(fn);

	mutex_lock(&data->lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);

	if (reg_data < 0) {
		err = reg_data;
		goto unlock_out;
	}

	byte_val = (reg_data & FAN_MODE_MASK) | target_pwm;

	err = i2c_smbus_write_byte_data(client, reg_addr, byte_val);

unlock_out:
	mutex_unlock(&data->lock);

	return err;
}

static int fbfancpld_pwm_enable_write(struct device *dev, int channel, long val)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int byte_val;
	int fn;
	int err = 0;
	int reg_data;
	int reg_addr;

	if (val != 0 && val != 1)
		return -EINVAL;

	fn = channel + 1;
	reg_addr = REG_FAN_PWM(fn);

	mutex_lock(&data->lock);

	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	if (reg_data < 0) {
		err = reg_data;
		goto unlock_out;
	}

	byte_val = (val << FAN_MODE_BIT_OFFSET) | reg_data;

	err = i2c_smbus_write_byte_data(client, reg_addr, byte_val);

unlock_out:
	mutex_unlock(&data->lock);

	return err;
}

static int fbfancpld_pwm_write(struct device *dev, u32 attr,
			       int channel, long val)
{
	switch (attr) {
	case hwmon_pwm_input:
		return fbfancpld_pwm_input_write(dev, channel, val);
	case hwmon_pwm_enable:
		return fbfancpld_pwm_enable_write(dev, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fbfancpld_read(struct device *dev,
			  enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		return fbfancpld_fan_read(dev, attr, channel, val);
	case hwmon_pwm:
		return fbfancpld_pwm_read(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fbfancpld_write(struct device *dev,
			   enum hwmon_sensor_types type,
			   u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_pwm:
		return fbfancpld_pwm_write(dev, attr, channel, val);
	case hwmon_fan:
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_ops fbfancpld_hwmon_ops = {
	.is_visible = fbfancpld_is_visible,
	.read = fbfancpld_read,
	.write = fbfancpld_write,
};

/* --- Non-standard sysfs: fan present --- */

static ssize_t fan_present_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int fn = to_sensor_dev_attr(dev_attr)->index;
	int reg_addr = REG_FAN_STATUS(fn);
	int reg_data;

	mutex_lock(&data->lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n",
		       reg_data & FAN_PRESENT_MASK ? 0 : 1);
}

static SENSOR_DEVICE_ATTR_RO(fan1_present, fan_present, 1);
static SENSOR_DEVICE_ATTR_RO(fan2_present, fan_present, 2);
static SENSOR_DEVICE_ATTR_RO(fan3_present, fan_present, 3);
static SENSOR_DEVICE_ATTR_RO(fan4_present, fan_present, 4);
static SENSOR_DEVICE_ATTR_RO(fan5_present, fan_present, 5);
static SENSOR_DEVICE_ATTR_RO(fan6_present, fan_present, 6);
static SENSOR_DEVICE_ATTR_RO(fan7_present, fan_present, 7);
static SENSOR_DEVICE_ATTR_RO(fan8_present, fan_present, 8);

static struct attribute *all_present_attrs[MAX_FANS] = {
	&sensor_dev_attr_fan1_present.dev_attr.attr,
	&sensor_dev_attr_fan2_present.dev_attr.attr,
	&sensor_dev_attr_fan3_present.dev_attr.attr,
	&sensor_dev_attr_fan4_present.dev_attr.attr,
	&sensor_dev_attr_fan5_present.dev_attr.attr,
	&sensor_dev_attr_fan6_present.dev_attr.attr,
	&sensor_dev_attr_fan7_present.dev_attr.attr,
	&sensor_dev_attr_fan8_present.dev_attr.attr,
};

/* --- Non-standard sysfs: boost PWM --- */

static ssize_t fan_boost_pwm_show(struct device *dev,
				  struct device_attribute *dev_attr, char *buf)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;

	mutex_lock(&data->lock);
	ret = i2c_smbus_read_byte_data(client, WDT_PWM_REG);
	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", (ret > data->config.pwm_max) ?
			data->config.pwm_max : ret);
}

static ssize_t fan_boost_pwm_store(struct device *dev,
				   struct device_attribute *dev_attr,
				   const char *buf, size_t count)
{
	struct fbfancpld_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;

	val = (val > data->config.pwm_max) ? data->config.pwm_max : val;
	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(client, WDT_PWM_REG, val);
	mutex_unlock(&data->lock);

	if (ret == 0)
		ret = count;

	return ret;
}

static SENSOR_DEVICE_ATTR_RW(pwm_boost, fan_boost_pwm, 0);

static struct attribute *boost_pwm_attrs[] = {
	&sensor_dev_attr_pwm_boost.dev_attr.attr,
	NULL
};

static const struct attribute_group boost_pwm_group = {
	.attrs = boost_pwm_attrs,
};

/* --- LEDs --- */

static const struct {
	u8 reg_val;
	const char *color;
} led_color_info[FAN_LED_COLOR_NUM] = {
	{ .reg_val = FAN_LED_BLUE,  .color = "blue" },
	{ .reg_val = FAN_LED_AMBER, .color = "amber" },
};

static enum led_brightness fan_led_brightness_get(struct led_classdev *cdev)
{
	struct fan_led_data *ldata = container_of(cdev,
						  struct fan_led_data, cdev);
	int ret;
	u8 reg_val;

	ret = i2c_smbus_read_byte_data(ldata->client, ldata->reg_addr);
	if (ret < 0)
		return ret;

	reg_val = (u8)(ret & FAN_LED_REG_MASK);
	if (reg_val == ldata->reg_val)
		return LED_ON;

	return LED_OFF;
}

static int fan_led_brightness_set(struct led_classdev *cdev,
				  enum led_brightness value)
{
	struct fan_led_data *ldata = container_of(cdev,
						  struct fan_led_data, cdev);
	u8 reg_val;

	reg_val = (value == LED_OFF) ? FAN_LED_OFF : ldata->reg_val;
	return i2c_smbus_write_byte_data(ldata->client, ldata->reg_addr,
					 reg_val);
}

static int fan_leds_init(struct fbfancpld_data *fdata)
{
	int i, fn, ret;
	int led_slot = 0;
	struct i2c_client *client = fdata->client;
	struct device *dev = &client->dev;

	fdata->num_leds = fdata->config.num_fans * FAN_LED_COLOR_NUM;
	fdata->leds = devm_kcalloc(dev, fdata->num_leds,
				   sizeof(*fdata->leds), GFP_KERNEL);
	if (!fdata->leds)
		return -ENOMEM;

	for (fn = 1; fn <= fdata->config.num_fans; fn++) {
		u8 reg_addr = REG_FAN_LED(fn);

		ret = i2c_smbus_write_byte_data(client, reg_addr,
						FAN_LED_HW_CTRL);
		if (ret)
			dev_err(dev,
				"fan%d_led: failed to set hw_ctrl, ret=%d",
				fn, ret);

		for (i = 0; i < FAN_LED_COLOR_NUM; i++) {
			struct fan_led_data *ldata =
				&fdata->leds[led_slot++];

			ldata->client = client;
			ldata->reg_addr = reg_addr;
			ldata->reg_val = led_color_info[i].reg_val;
			snprintf(ldata->name, sizeof(ldata->name),
				 "fan%d:%s:status",
				 fn, led_color_info[i].color);

			ldata->cdev.name = ldata->name;
			ldata->cdev.brightness_get =
				fan_led_brightness_get;
			ldata->cdev.brightness_set_blocking =
				fan_led_brightness_set;
			ldata->cdev.max_brightness = 1;

			ret = devm_led_classdev_register(dev, &ldata->cdev);
			if (ret)
				return ret;
			ret = led_trigger_init(ldata->cdev.dev);
			if (ret)
				dev_err(dev,
					"fan%d_led: failed to init trigger, ret=%d",
					fn, ret);
		}
	}

	return 0;
}

/* --- ioctl: configure --- */

static int fbfancpld_validate_config(struct device *dev,
				     const struct fbfancpld_ioctl_config *ucfg)
{
	if (ucfg->num_fans == 0 || ucfg->num_fans > MAX_FANS) {
		dev_err(dev, "invalid num_fans %u (must be 1-%d)\n",
			ucfg->num_fans, MAX_FANS);
		return -EINVAL;
	}

	if (ucfg->pwm_max == 0 || ucfg->pwm_max > 255) {
		dev_err(dev, "invalid pwm_max %u\n", ucfg->pwm_max);
		return -EINVAL;
	}

	if (ucfg->speed_multiplier == 0) {
		dev_err(dev, "invalid speed_multiplier 0\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * Initialize hwmon, watchdog, and optionally LEDs from the validated config.
 * Must be called with data->lock held.
 */
static int fbfancpld_apply_config(struct fbfancpld_data *data)
{
	struct fbfancpld_config *cfg = &data->config;
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	int i, num_tach, err;

	/* Build dynamic hwmon fan channel info */
	num_tach = cfg->has_rear_tach ?
		   cfg->num_fans * 2 : cfg->num_fans;
	for (i = 0; i < num_tach; i++)
		data->fan_config[i] = HWMON_F_INPUT | HWMON_F_LABEL;
	data->fan_channel.type = hwmon_fan;
	data->fan_channel.config = data->fan_config;

	/* Build dynamic hwmon PWM channel info */
	for (i = 0; i < cfg->num_fans; i++)
		data->pwm_config[i] = HWMON_PWM_INPUT | HWMON_PWM_ENABLE;
	data->pwm_channel.type = hwmon_pwm;
	data->pwm_channel.config = data->pwm_config;

	data->channels[0] = &data->fan_channel;
	data->channels[1] = &data->pwm_channel;
	data->chip_info.ops = &fbfancpld_hwmon_ops;
	data->chip_info.info = data->channels;

	/* Build dynamic present sysfs attrs (first num_fans entries) */
	for (i = 0; i < cfg->num_fans; i++)
		data->present_ptrs[i] = all_present_attrs[i];
	data->present_group.attrs = data->present_ptrs;

	data->extra_groups[0] = &data->present_group;
	data->extra_groups[1] = &boost_pwm_group;

	data->hwmon_dev = devm_hwmon_device_register_with_info(dev,
				client->name, data, &data->chip_info,
				data->extra_groups);
	err = PTR_ERR_OR_ZERO(data->hwmon_dev);
	if (err)
		return err;

	err = fan_wdt_init(client);
	if (err)
		return err;

	if (cfg->has_leds) {
		err = fan_leds_init(data);
		if (err)
			return err;
	}

	return 0;
}

static long fbfancpld_ioctl_configure(struct fbfancpld_data *data,
				      void __user *userp)
{
	struct fbfancpld_ioctl_config ucfg;
	struct device *dev = &data->client->dev;
	int ret;

	if (copy_from_user(&ucfg, userp, sizeof(ucfg)))
		return -EFAULT;

	ret = fbfancpld_validate_config(dev, &ucfg);
	if (ret)
		return ret;

	mutex_lock(&data->lock);

	if (data->configured) {
		dev_err(dev, "already configured\n");
		mutex_unlock(&data->lock);
		return -EBUSY;
	}

	data->config.num_fans = ucfg.num_fans;
	data->config.pwm_max = ucfg.pwm_max;
	data->config.speed_multiplier = ucfg.speed_multiplier;
	data->config.has_rear_tach = !!ucfg.has_rear_tach;
	data->config.has_leds = !!ucfg.has_leds;

	ret = fbfancpld_apply_config(data);
	if (ret) {
		dev_err(dev, "failed to apply config: %d\n", ret);
		mutex_unlock(&data->lock);
		return ret;
	}

	data->configured = true;
	mutex_unlock(&data->lock);

	dev_info(dev, "configured: fans=%d pwm_max=%d speed_mult=%d rear_tach=%d leds=%d\n",
		 data->config.num_fans, data->config.pwm_max,
		 data->config.speed_multiplier, data->config.has_rear_tach,
		 data->config.has_leds);

	return 0;
}

static long fbfancpld_cdev_ioctl(struct file *file,
				 unsigned int cmd,
				 unsigned long param)
{
	struct fbfancpld_data *data = container_of(file->private_data,
						   struct fbfancpld_data,
						   miscdev);
	void __user *userp = (void __user *)param;

	switch (cmd) {
	case FBFANCPLD_IOC_CONFIGURE:
		return fbfancpld_ioctl_configure(data, userp);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations fbfancpld_cdev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fbfancpld_cdev_ioctl,
};

/* --- Probe / Remove --- */

static int fbfancpld_probe(struct i2c_client *client)
{
	struct fbfancpld_data *data;
	struct device *dev = &client->dev;
	int err;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);

	snprintf(data->miscdev_name, sizeof(data->miscdev_name),
		 "fbfancpld-%d-%04x", client->adapter->nr, client->addr);

	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = data->miscdev_name;
	data->miscdev.fops = &fbfancpld_cdev_fops;
	data->miscdev.parent = dev;

	err = misc_register(&data->miscdev);
	if (err)
		return err;

	dev_info(dev, "registered %s (awaiting configuration)\n",
		 data->miscdev_name);
	return 0;
}

static void fbfancpld_remove(struct i2c_client *client)
{
	struct fbfancpld_data *data = i2c_get_clientdata(client);

	if (data->configured && data->config.has_leds) {
		for (int i = 0; i < data->num_leds; i++)
			led_trigger_deinit(data->leds[i].cdev.dev);
	}

	misc_deregister(&data->miscdev);
	mutex_destroy(&data->lock);
}

static const struct i2c_device_id fbfancpld_id[] = {
	{ DRIVER_NAME, 0 },
	{ "fbg_fancpld", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fbfancpld_id);

static struct i2c_driver fbfancpld_driver = {
	.class    = I2C_CLASS_HWMON,
	.driver   = {
		.name  = DRIVER_NAME,
	},
	.probe    = fbfancpld_probe,
	.remove   = fbfancpld_remove,
	.id_table = fbfancpld_id,
};

module_i2c_driver(fbfancpld_driver);

MODULE_AUTHOR("Meta Platforms, Inc.");
MODULE_DESCRIPTION("Generic FBOSS Fan CPLD Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
