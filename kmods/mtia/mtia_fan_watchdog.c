// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * This is the driver for the fan watchdog on minerva, the driver can
 * be used on the rack and the test fixture
 * Copyright (c) Celestica Inc.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/watchdog.h>

/* The fan watchdog controller is in the rack CPLD and test fixture CPLD */
#define RACK_FAN_WD_CMD_EN	0x00
#define RACK_FAN_WD_CMD_PWM	0x01
#define RACK_FAN_WD_CMD_COUNT	0x02
#define FAN_WD_START		0x01
#define FAN_WD_STOP		0x00
#define FAN_WD_TIMEOUT_MIN	5       /* in sec */
#define FAN_WD_TIMEOUT_MAX	1275	 /* in sec */
#define FAN_WD_TIMEOUT_DEFAULT	30      /* in sec */
#define FAN_TIMEOUT_STEP	5       /* fan watchdog timeout is the multiples of 5 */

#define DEVNAME	"mtia_fan_watchdog"

struct mtia_fan_wdt_data {
	struct watchdog_device wdd;
};

static int wdt_timeout;
module_param(wdt_timeout, int, 0);
MODULE_PARM_DESC(wdt_timeout, "Watchdog timeout in seconds");
static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started defaultly"
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int mtia_fan_wdt_start(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, RACK_FAN_WD_CMD_EN, FAN_WD_START);
}

static int mtia_fan_wdt_stop(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, RACK_FAN_WD_CMD_EN, FAN_WD_STOP);
}

static int mtia_fan_wdt_ping(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	return i2c_smbus_write_byte_data(client, RACK_FAN_WD_CMD_COUNT,
					 wdd->timeout/FAN_TIMEOUT_STEP);
}

static int mtia_fan_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	int ret;

	if (timeout > FAN_WD_TIMEOUT_MAX ||
	    timeout < FAN_WD_TIMEOUT_MIN || timeout%FAN_TIMEOUT_STEP != 0) {
		dev_err(&client->dev, "Failed to set timeout\n");
		return -EINVAL;
	}
	ret = i2c_smbus_write_byte_data(client, RACK_FAN_WD_CMD_COUNT, timeout/FAN_TIMEOUT_STEP);
	if (!ret)
		wdd->timeout = timeout;
	return ret;
}

static unsigned int mtia_fan_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct i2c_client *client = to_i2c_client(wdd->parent);

	int ret;

	ret = i2c_smbus_read_byte_data(client, RACK_FAN_WD_CMD_COUNT);
	if (ret < 0)
		ret = 0;
	return ret*FAN_TIMEOUT_STEP;
}

static const struct watchdog_info mtia_fan_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING |
		   WDIOS_DISABLECARD | WDIOS_ENABLECARD,
	.identity = "MTIA Fan Watchdog",
};

static const struct watchdog_ops mtia_fan_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= mtia_fan_wdt_start,
	.stop		= mtia_fan_wdt_stop,
	.ping		= mtia_fan_wdt_ping,
	.set_timeout	= mtia_fan_wdt_set_timeout,
	.get_timeleft	= mtia_fan_wdt_get_timeleft,
};

static int mtia_fan_wdt_probe(struct i2c_client *client)
{
	int ret;
	struct mtia_fan_wdt_data *w_priv;
	int val;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WRITE_BLOCK_DATA))
		return -ENODEV;

	w_priv = devm_kzalloc(&client->dev, sizeof(*w_priv), GFP_KERNEL);
	if (!w_priv)
		return -ENOMEM;

	w_priv->wdd.info = &mtia_fan_wdt_info;
	w_priv->wdd.ops = &mtia_fan_wdt_ops;
	w_priv->wdd.min_timeout = FAN_WD_TIMEOUT_MIN;
	w_priv->wdd.max_timeout = FAN_WD_TIMEOUT_MAX;
	w_priv->wdd.parent = &client->dev;
	watchdog_init_timeout(&w_priv->wdd, wdt_timeout, &client->dev);

	/*
	 * The default value set in the watchdog should be perfectly valid, so
	 * pass that in if we haven't provided one via the module parameter or
	 * of property.
	 */
	if (w_priv->wdd.timeout == 0) {
		val = i2c_smbus_read_byte_data(client, RACK_FAN_WD_CMD_COUNT) *
				FAN_TIMEOUT_STEP;
		if (val < 0) {
			dev_err(&client->dev, "Failed to read timeout\n");
			return val;
		}
		if (val > FAN_WD_TIMEOUT_MAX ||
		    val < FAN_WD_TIMEOUT_MIN)
			val = FAN_WD_TIMEOUT_DEFAULT;
		w_priv->wdd.timeout = val;
	}

	ret = mtia_fan_wdt_set_timeout(&w_priv->wdd, w_priv->wdd.timeout);
	if (ret)
		return ret;

	dev_info(&client->dev, "Timeout set to %ds\n", w_priv->wdd.timeout);
	watchdog_set_nowayout(&w_priv->wdd, nowayout);
	i2c_set_clientdata(client, w_priv);
	ret = devm_watchdog_register_device(&client->dev, &w_priv->wdd);
	return ret;
}

static const struct i2c_device_id mtia_fan_wdt_id[] = {
	{ DEVNAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mtia_fan_wdt_id);

static const struct of_device_id mtia_fan_wdt_of_match[] = {
	{ .compatible = DEVNAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, mtia_fan_wdt_of_match);

static struct i2c_driver mtia_fan_wdt_driver = {
	.driver = {
		.name = DEVNAME,
		.of_match_table = mtia_fan_wdt_of_match,
	},
	.probe = mtia_fan_wdt_probe,
	.id_table = mtia_fan_wdt_id,
};
module_i2c_driver(mtia_fan_wdt_driver);

MODULE_AUTHOR("Xiaoshen Chen <xiaoshen@celestica.com");
MODULE_DESCRIPTION("Minerva rack fan watchdog driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
