// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for MPS MPQ82D00
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"

#define MPQ82D00_VOUT_DIV	64

#define MPQ82D00_PAGE_NUM	1

#define MPQ82D00_RAIL1_FUNC	(PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | \
							PMBUS_HAVE_IOUT | PMBUS_HAVE_TEMP | \
							PMBUS_HAVE_POUT | PMBUS_HAVE_PIN | \
							PMBUS_HAVE_STATUS_VOUT | \
							PMBUS_HAVE_STATUS_IOUT | \
							PMBUS_HAVE_STATUS_TEMP | \
							PMBUS_HAVE_STATUS_INPUT)

struct mpq82d00_data {
	struct pmbus_driver_info info;
	int vout_scale;
};

#define to_mpq82d00_data(x) container_of(x, struct mpq82d00_data, info)

static int mpq82d00_read_byte_data(struct i2c_client *client, int page, int reg)
{
	int ret;

	switch (reg) {
	case PMBUS_VOUT_MODE:
		ret = PB_VOUT_MODE_DIRECT;
		break;
	case PMBUS_STATUS_WORD:
	case PMBUS_STATUS_VOUT:
	case PMBUS_STATUS_IOUT:
	case PMBUS_STATUS_INPUT:
		ret = -ENODATA;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mpq82d00_read_word_data(struct i2c_client *client, int page,
				 int phase, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct mpq82d00_data *data = to_mpq82d00_data(info);
	int ret;

	switch (reg) {
	case PMBUS_STATUS_WORD:
	case PMBUS_READ_VIN:
	case PMBUS_READ_IOUT:
	case PMBUS_READ_POUT:
	case PMBUS_READ_PIN:
	case PMBUS_READ_TEMPERATURE_1:
	case PMBUS_IOUT_OC_FAULT_LIMIT:
	case PMBUS_OT_FAULT_LIMIT:
		ret = pmbus_read_word_data(client, page, phase, reg);
		break;
	case PMBUS_READ_VOUT:
		ret = pmbus_read_word_data(client, page, phase, reg);
		if (ret < 0)
			return ret;

		ret = DIV_ROUND_CLOSEST((ret & GENMASK(11, 0)) * data->vout_scale,
									MPQ82D00_VOUT_DIV);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mpq82d00_identify(struct i2c_client *client, struct pmbus_driver_info *info)
{
	struct mpq82d00_data *data = to_mpq82d00_data(info);
	int ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte_data(client, PMBUS_VOUT_MODE);
	if (ret < 0)
		return ret;

	if (FIELD_GET(GENMASK(5, 5), ret)) {
		data->vout_scale = 320;
	} else {
		data->vout_scale = 125;
	}

	return 0;
}

static struct pmbus_driver_info mpq82d00_info = {
	.pages = MPQ82D00_PAGE_NUM,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_POWER] = linear,

	/* 1mV/LSB */
	.m[PSC_VOLTAGE_OUT] = 1,
	.R[PSC_VOLTAGE_OUT] = 3,
	.b[PSC_VOLTAGE_OUT] = 0,

	.func[0] = MPQ82D00_RAIL1_FUNC,
	.read_word_data = mpq82d00_read_word_data,
	.read_byte_data = mpq82d00_read_byte_data,
	.identify = mpq82d00_identify,
};

static int mpq82d00_probe(struct i2c_client *client)
{
	return pmbus_do_probe(client, &mpq82d00_info);
}

static const struct i2c_device_id mpq82d00_id[] = {
	{"mpq82d00"},
	{"mpq82b00"},
	{"mpq82600"},
	{}
};
MODULE_DEVICE_TABLE(i2c, mpq82d00_id);

static const struct of_device_id __maybe_unused mpq82d00_of_match[] = {
	{.compatible = "mps,mpq82d00"},
	{.compatible = "mps,mpq82b00"},
	{.compatible = "mps,mpq82600"},
	{}
};
MODULE_DEVICE_TABLE(of, mpq82d00_of_match);

static struct i2c_driver mpq82d00_driver = {
	.driver = {
		.name = "mpq82d00",
		.of_match_table = mpq82d00_of_match,
	},
	.probe = mpq82d00_probe,
	.id_table = mpq82d00_id,
};

module_i2c_driver(mpq82d00_driver);

MODULE_AUTHOR("Noah Wang <Noah.Wang@monolithicpower.com>");
MODULE_DESCRIPTION("PMBus driver for MPS MPQ82D00");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS_PMBUS;
MODULE_VERSION(BSP_VERSION);
