// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

/*
 * FANCPLD is a logical component (I2C Slave Module, address 0x33) of
 * MCBCPLD dedicated for fan control.
 *
 * FANCPLD doesn't have its own firmware revision because it's part of
 * MCB CPLD. MCB CPLD firmware revision is reported by mcbcpld_i2c.c.
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/watchdog.h>
#include <linux/version.h>

#define DRIVER_NAME	"anacapa_fancpld"

/* Fan PWM is defined as 0 to 64 mapped to 0 to 100%, multiplied by 1.5625% */
#define FAN_PWM_MAX		64

/* Fan control related registers, fn = 1...6 */
#define FBMCB_REG_FAN_TACH_F_N(fn)                (0x30 + (fn - 1) * 0x10)
#define FBMCB_REG_FAN_TACH_B_N(fn)                (0x31 + (fn - 1) * 0x10)
#define FBMCB_REG_FAN_PWM(fn)                     (0x32 + (fn - 1) * 0x10)
#define FBMCB_REG_FAN_STATUS(fn)                  (0x38 + (fn - 1) * 0x10)

#define FBMCB_REG_FAN_MODE_BIT_OFFSET             7
#define FBMCB_REG_FAN_MODE_MASK                   BIT(7)
#define FBMCB_REG_FAN_PRESENT_MASK                0b00000001

#define FBMCB_FAN_SPEED_MULTIPLIER                300

#define FAN_WDT_CTRL_0                            0x1F
#define FAN_WDT_ENABLE_BIT                        BIT(4)
#define FAN_WDT_PET                               BIT(5)
#define FAN_WDT_TIMEOUT                           BIT(6)

#define FAN_WDT_CTRL_1                            0x20
/* maximum reg val 0xff, increments of 5 sec */
#define FAN_WDT_TIMEOUT_MAX                       (0xFF * 5)

#define FAN_WDT_STA_REG                           0x21
#define FAN_WDT_PWM                               0x22

/* default timeout value, in seconds (increments of 5) */
#define FAN_WDT_DFT_TIMEOUT	(0x3F * 5)

static int wdt_timeout = FAN_WDT_DFT_TIMEOUT;
module_param(wdt_timeout, int, 0);
MODULE_PARM_DESC(wdt_timeout,
	"Watchdog timeout in seconds. Default = "
		__MODULE_STRING(FAN_WDT_DFT_TIMEOUT));

struct fbmcb_fan_data {
	struct device *hwmon_dev;
	struct watchdog_device *wdd;
	struct mutex idd_lock;
	struct i2c_client *client;
};

/*
 * MCB CPLD Fan WDT -- How does it work:
 *  - 0x1F - Control Reg 0, writing values to:
 *           Start: 0x10
 *             Pet: 0x30 (0x20 | 0x10)
 *            Stop: 0x00
 *  - 0x20 - Timeout: * 5 sec;
 *  - 0x21 - Counter: * 5 sec;
 *                    Times out when reaches the value of 0x20;
 *                    Will be set to 0 after petting;
 *  - 0x22 - Boost PWM: from 0 to 64 (0x40);
 */

static int fan_wdt_start(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, FAN_WDT_CTRL_0, FAN_WDT_ENABLE_BIT);
}

static int fan_wdt_stop(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, FAN_WDT_CTRL_0, 0);
}

static int fan_wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	/*
	 * Timeout length in seconds is 5 * register_value. Divide timeout by 5
	 * to determine value to set in register.
	 */
	u8 reg;
	struct i2c_client *client = to_i2c_client(wdd->parent);

	wdd->timeout = min_t(unsigned int, timeout, FAN_WDT_TIMEOUT_MAX);
	reg = wdd->timeout / 5;

	return i2c_smbus_write_byte_data(client, FAN_WDT_CTRL_1, reg);
}

static unsigned int fan_wdt_get_timeleft(struct watchdog_device *wdd)
{
	int max_timeout;
	int counter;
	int ret;

	struct i2c_client *client = to_i2c_client(wdd->parent);

	max_timeout = i2c_smbus_read_byte_data(client, FAN_WDT_CTRL_1);
	counter = i2c_smbus_read_byte_data(client, FAN_WDT_STA_REG);
	ret = (max_timeout - counter) * 5;
	if (ret < 0)
		ret = 0;
	return ret;
}

static int fan_wdt_ping(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, FAN_WDT_CTRL_0, FAN_WDT_ENABLE_BIT | FAN_WDT_PET);
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

static int fan_wdt_init(struct watchdog_device *wdd, struct i2c_client *client)
{
	int err;
	struct device *dev = &client->dev;

	wdd = devm_kzalloc(dev, sizeof(*wdd), GFP_KERNEL);
	if (!wdd)
		return -ENOMEM;

	wdd->info = &fan_wdt_info;
	wdd->ops = &fan_wdt_ops;
	wdd->parent = dev;
	wdd->timeout = wdt_timeout;
	wdd->max_timeout = FAN_WDT_TIMEOUT_MAX;

	watchdog_init_timeout(wdd, wdt_timeout, dev);

	fan_wdt_stop(wdd);
	err = devm_watchdog_register_device(dev, wdd);
	if (err) {
		dev_err(dev, "watchdog_register_device failed, ret=%d\n", err);
		return err;
	}
	return 0;
}

static umode_t mcbfan_is_visible(const void *_data,
				 enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	umode_t mode = 0;

	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
			mode = 0444;
			break;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
		case hwmon_pwm_enable:
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

static int mcbfan_fan_input_read(struct device *dev, int channel, long *val)
{
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int fn; /* fan number */
	int reg_addr;
	int reg_data;

	fn = channel / 2 + 1;
	reg_addr = (channel % 2) ? FBMCB_REG_FAN_TACH_B_N(fn) :
				   FBMCB_REG_FAN_TACH_F_N(fn);
	mutex_lock(&fan_data->idd_lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&fan_data->idd_lock);

	if (reg_data < 0)
		return reg_data;

	*val = reg_data * FBMCB_FAN_SPEED_MULTIPLIER;
	return 0;
}

static int mcbfan_fan_read(struct device *dev, u32 attr, int channel, long *val)
{
	switch (attr) {
	case hwmon_fan_input:
		return mcbfan_fan_input_read(dev, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int mcbfan_pwm_input_read(struct device *dev, int channel, long *val)
{
	/*
	 * PWM register value is from 0 to 64
	 * actual PWM is calculated by multiplying 1.5625%
	 * If the register value is over 64 then PWM is set to 100%
	 */
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int fn;
	int reg_addr;
	int reg_data;

	fn = channel + 1;
	reg_addr = FBMCB_REG_FAN_PWM(fn);

	mutex_lock(&fan_data->idd_lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&fan_data->idd_lock);

	if (reg_data < 0)
		return reg_data;

	*val = (reg_data > FAN_PWM_MAX) ? FAN_PWM_MAX : reg_data;
	return 0;
}

static int mcbfan_pwm_enable_read(struct device *dev, int channel, long *val)
{
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int fn;
	int reg_addr;
	int reg_data;

	fn = channel + 1;
	reg_addr = FBMCB_REG_FAN_PWM(fn);

	mutex_lock(&fan_data->idd_lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&fan_data->idd_lock);

	if (reg_data < 0)
		return reg_data;

	*val = (reg_data & FBMCB_REG_FAN_MODE_MASK) ? 0 : 1;
	return 0;
}

static int mcbfan_pwm_read(struct device *dev, u32 attr, int channel, long *val)
{
	switch (attr) {
	case hwmon_pwm_input:
		return mcbfan_pwm_input_read(dev, channel, val);
	case hwmon_pwm_enable:
		return mcbfan_pwm_enable_read(dev, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int mcbfan_pwm_input_write(struct device *dev, int channel, long val)
{
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int byte_val;
	int fn;
	int err;
	int reg_addr;
	int reg_data;
	int target_pwm;

	if (val < 0)
		return -EINVAL;

	target_pwm = (val > FAN_PWM_MAX) ? FAN_PWM_MAX : val;

	fn = channel + 1;
	reg_addr = FBMCB_REG_FAN_PWM(fn);

	mutex_lock(&fan_data->idd_lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);

	if (reg_data < 0) {
		err = reg_data;
		goto unlock_out;
	}

	byte_val = (reg_data & FBMCB_REG_FAN_MODE_MASK) | target_pwm;

	err = i2c_smbus_write_byte_data(client, reg_addr, byte_val);

unlock_out:
	mutex_unlock(&fan_data->idd_lock);

	return err;
}

static int mcbfan_pwm_enable_write(struct device *dev, int channel, long val)
{
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int byte_val;
	int fn;
	int err = 0;
	int reg_data;
	int reg_addr;

	if ((val != 0) && (val != 1))
		return -EINVAL;

	fn = channel + 1;
	reg_addr = FBMCB_REG_FAN_PWM(fn);

	mutex_lock(&fan_data->idd_lock);

	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	if (reg_data < 0) {
		err = reg_data;
		goto unlock_out;
	}

	byte_val = (val << FBMCB_REG_FAN_MODE_BIT_OFFSET) | reg_data;

	err = i2c_smbus_write_byte_data(client, reg_addr, byte_val);

unlock_out:
	mutex_unlock(&fan_data->idd_lock);

	return err;
}

static int mcbfan_pwm_write(struct device *dev, u32 attr, int channel, long val)
{
	switch (attr) {
	case hwmon_pwm_input:
		return mcbfan_pwm_input_write(dev, channel, val);
	case hwmon_pwm_enable:
		return mcbfan_pwm_enable_write(dev, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int mcbfan_read(struct device *dev,
		       enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		return mcbfan_fan_read(dev, attr, channel, val);
	case hwmon_pwm:
		return mcbfan_pwm_read(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int mcbfan_write(struct device *dev,
			enum hwmon_sensor_types type,
			u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_pwm:
		return mcbfan_pwm_write(dev, attr, channel, val);
	case hwmon_fan:
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_ops mcbfan_hwmon_ops = {
	.is_visible = mcbfan_is_visible,
	.read = mcbfan_read,
	.write = mcbfan_write,
};

static const struct hwmon_channel_info *mcbfan_info[] = {
	/* 6 fans x 2 tach (f&r) */
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL
};

static struct hwmon_chip_info mcbfan_chip_info = {
	.ops = &mcbfan_hwmon_ops,
	.info = mcbfan_info,
};

/* Non-standard hwmon sysfs: present and boost PWM */

static ssize_t fan_present_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int fn = to_sensor_dev_attr(dev_attr)->index;
	int reg_addr = FBMCB_REG_FAN_STATUS(fn);
	int reg_data;

	mutex_lock(&fan_data->idd_lock);
	reg_data = i2c_smbus_read_byte_data(client, reg_addr);
	mutex_unlock(&fan_data->idd_lock);

	return sprintf(buf, "%d\n", reg_data & FBMCB_REG_FAN_PRESENT_MASK ?
			0 : 1);
}

static SENSOR_DEVICE_ATTR_RO(fan1_present, fan_present, 1);
static SENSOR_DEVICE_ATTR_RO(fan2_present, fan_present, 2);
static SENSOR_DEVICE_ATTR_RO(fan3_present, fan_present, 3);
static SENSOR_DEVICE_ATTR_RO(fan4_present, fan_present, 4);
static SENSOR_DEVICE_ATTR_RO(fan5_present, fan_present, 5);
static SENSOR_DEVICE_ATTR_RO(fan6_present, fan_present, 6);

static struct attribute *mcbfan_present_attrs[] = {
	&sensor_dev_attr_fan1_present.dev_attr.attr,
	&sensor_dev_attr_fan2_present.dev_attr.attr,
	&sensor_dev_attr_fan3_present.dev_attr.attr,
	&sensor_dev_attr_fan4_present.dev_attr.attr,
	&sensor_dev_attr_fan5_present.dev_attr.attr,
	&sensor_dev_attr_fan6_present.dev_attr.attr,
	NULL
};

static const struct attribute_group mcbfan_present_group = {
	.attrs = mcbfan_present_attrs,
};

static ssize_t fan_boost_pwm_show(struct device *dev,
				  struct device_attribute *dev_attr, char *buf)
{
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int ret;

	mutex_lock(&fan_data->idd_lock);
	ret = i2c_smbus_read_byte_data(client, FAN_WDT_PWM);
	mutex_unlock(&fan_data->idd_lock);

	return sprintf(buf, "%d\n", (ret > FAN_PWM_MAX) ?
			FAN_PWM_MAX : ret);
}

static ssize_t fan_boost_pwm_store(struct device *dev,
				   struct device_attribute *dev_attr,
				   const char *buf, size_t count)
{
	struct fbmcb_fan_data *fan_data = dev_get_drvdata(dev);
	struct i2c_client *client = fan_data->client;
	int ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;

	val = (val > FAN_PWM_MAX) ? FAN_PWM_MAX : val;
	mutex_lock(&fan_data->idd_lock);
	ret = i2c_smbus_write_byte_data(client, FAN_WDT_PWM, val);
	mutex_unlock(&fan_data->idd_lock);

	if (ret == 0)
		ret = count;

	return ret;
}

static SENSOR_DEVICE_ATTR_RW(pwm_boost, fan_boost_pwm, 0);

static struct attribute *mcbfan_boost_pwm_attrs[] = {
	&sensor_dev_attr_pwm_boost.dev_attr.attr,
	NULL
};

static const struct attribute_group mcbfan_boost_pwm_group = {
	.attrs = mcbfan_boost_pwm_attrs,
};

static const struct attribute_group *mcbfan_extra_groups[] = {
	&mcbfan_present_group,
	&mcbfan_boost_pwm_group,
	NULL
};

static int fancpld_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fbmcb_fan_data *fan_data;
	int err;

	fan_data = devm_kzalloc(dev, sizeof(*fan_data), GFP_KERNEL);
	if (!fan_data)
		return -ENOMEM;

	i2c_set_clientdata(client, fan_data);
	fan_data->client = client;
	mutex_init(&fan_data->idd_lock);

	fan_data->hwmon_dev = devm_hwmon_device_register_with_info(dev,
				client->name, fan_data, &mcbfan_chip_info,
				mcbfan_extra_groups);

	err = PTR_ERR_OR_ZERO(fan_data->hwmon_dev);
	if (err)
		goto exit_cleanup;

	err = fan_wdt_init(fan_data->wdd, client);
	if (err)
		goto exit_cleanup;

	return 0;

exit_cleanup:
	mutex_destroy(&fan_data->idd_lock);

	return err;
}

static void fancpld_i2c_remove(struct i2c_client *client)
{
	struct fbmcb_fan_data *fan_data = i2c_get_clientdata(client);

	mutex_destroy(&fan_data->idd_lock);
}

/* MCBCPLD_FAN id */
static const struct i2c_device_id anacapa_fancpld_i2c_id[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, anacapa_fancpld_i2c_id);

static struct i2c_driver anacapa_fancpld_i2c_driver = {
	.class    = I2C_CLASS_HWMON,
	.driver   = {
		.name  = DRIVER_NAME,
	},
	.probe    = fancpld_i2c_probe,
	.remove   = fancpld_i2c_remove,
	.id_table = anacapa_fancpld_i2c_id,
};

module_i2c_driver(anacapa_fancpld_i2c_driver);

MODULE_AUTHOR("Yves Wang <yves.wang@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS MCB CPLD (FAN) Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
