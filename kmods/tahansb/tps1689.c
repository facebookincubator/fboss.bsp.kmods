// SPDX-License-Identifier: GPL-2.0
//
// Author: Hameem Hamza <hameem@ti.com>
// Created for TPS1689.
//
// Modified for backward compatibility with kernels < 5.14

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>

#include "pmbus.h"

/* Common Manufacturer Specific Registers */
#define TPS_READ_VAUX			0xd0
#define TPS_READ_VIN_MIN		0xd1
#define TPS_READ_VIN_PEAK		0xd2
#define TPS_READ_IIN_PEAK		0xd4
#define TPS_READ_PIN_PEAK		0xd5
#define TPS_READ_TEMP_AVG		0xd6
#define TPS_READ_TEMP_PEAK		0xd7
#define TPS_READ_VOUT_MIN		0xda
#define TPS_READ_VIN_AVG		0xdc
#define TPS_READ_VOUT_AVG		0xdd
#define TPS_READ_IIN_AVG		0xde
#define TPS_READ_PIN_AVG		0xdf
#define TPS_VIREF			0xe0
#define TPS_PK_MIN_AVG			0xea
#define  PK_MIN_AVG_RST_PEAK		BIT(7)
#define  PK_MIN_AVG_RST_AVG		BIT(6)
#define  PK_MIN_AVG_RST_MIN		BIT(5)
#define  PK_MIN_AVG_AVG_CNT		GENMASK(2, 0)
#define TPS_MFR_WRITE_PROTECT		0xf8
#define  TPS_UNLOCKED			BIT(7)
#define  TPS_UNLOCK_CODE		0xa2
#define  TPS_LOCK_CODE			0x00

/*
 * Board RIMON value for TPS1689: 2.2kOhm (2,200,000000 uOhm)- only used if dts file is not loaded
 */
#define TPS1689_BOARD_RIMON		17400000000ULL

#define TPS1689_8B_SHIFT		2

#define TPS1689_VIN_OVF_NUM		1000
#define TPS1689_VIN_OVF_DIV		3984
#define TPS1689_VIN_OVF_OFF_IN		-63750
#define TPS1689_VIN_OVF_OFF_OUT		16000

#define TPS1689_VIREF_MIN_MV		300
#define TPS1689_VIREF_MAX_MV		1182
#define TPS1689_VIREF_STEPS		0x3F
#define TPS1689_G_IMON_NA		18250

#define PK_MIN_AVG_RST_MASK		(PK_MIN_AVG_RST_PEAK | \
					 PK_MIN_AVG_RST_AVG  | \
					 PK_MIN_AVG_RST_MIN)

struct tps1689_data {
	struct pmbus_driver_info info;
	u64 rimon_uohms;
};

/*
 * Scales the base 'm' coefficient from the datasheet to a new 'm' based on the
 * actual R_IMON value used.
 * new_m = base_m * (actual_rimon / base_rimon)
 */
static void tps1689_set_m_for_rimon(int *m, u64 rimon_uohms)
{
	u64 val = (u64)*m * rimon_uohms;

	*m = DIV_ROUND_CLOSEST_ULL(val, 2550000000ULL);
}

static int tps1689_mfr_write_protect_set(struct i2c_client *client, u8 protect)
{
	u8 val;

	switch (protect) {
	case 0:
		val = TPS_UNLOCK_CODE;
		break;
	case PB_WP_ALL:
		val = TPS_LOCK_CODE;
		break;
	default:
		return -EINVAL;
	}

	return pmbus_write_byte_data(client, 0, TPS_MFR_WRITE_PROTECT, val);
}

static int tps1689_mfr_write_protect_get(struct i2c_client *client)
{
	int ret = pmbus_read_byte_data(client, 0, TPS_MFR_WRITE_PROTECT);

	if (ret < 0)
		return ret;

	return (ret & TPS_UNLOCKED) ? 0 : PB_WP_ALL;
}

static int tps1689_read_byte_data(struct i2c_client *client, int page, int reg)
{
	switch (reg) {
	case PMBUS_WRITE_PROTECT:
		return tps1689_mfr_write_protect_get(client);
	default:
		return -ENODATA;
	}
}

static int tps1689_write_byte_data(struct i2c_client *client, int page, int reg, u8 byte)
{
	switch (reg) {
	case PMBUS_WRITE_PROTECT:
		return tps1689_mfr_write_protect_set(client, byte);
	default:
		return -ENODATA;
	}
}

static int tps1689_read_word_data(struct i2c_client *client, int page, int phase, int reg)
{
	struct tps1689_data *data = i2c_get_clientdata(client);
	int ret;

	switch (reg) {
	case PMBUS_VIRT_READ_VIN_MAX:
		return pmbus_read_word_data(client, page, phase, TPS_READ_VIN_PEAK);

	case PMBUS_VIRT_READ_VIN_MIN:
		return pmbus_read_word_data(client, page, phase, TPS_READ_VIN_MIN);

	case PMBUS_VIRT_READ_VIN_AVG:
		return pmbus_read_word_data(client, page, phase, TPS_READ_VIN_AVG);

	case PMBUS_VIRT_READ_VOUT_MIN:
		return pmbus_read_word_data(client, page, phase, TPS_READ_VOUT_MIN);

	case PMBUS_VIRT_READ_VOUT_AVG:
		return pmbus_read_word_data(client, page, phase, TPS_READ_VOUT_AVG);

	case PMBUS_VIRT_READ_IIN_AVG:
		return pmbus_read_word_data(client, page, phase, TPS_READ_IIN_AVG);

	case PMBUS_VIRT_READ_IIN_MAX:
		return pmbus_read_word_data(client, page, phase, TPS_READ_IIN_PEAK);

	case PMBUS_VIRT_READ_TEMP_AVG:
		return pmbus_read_word_data(client, page, phase, TPS_READ_TEMP_AVG);

	case PMBUS_VIRT_READ_TEMP_MAX:
		return pmbus_read_word_data(client, page, phase, TPS_READ_TEMP_PEAK);

	case PMBUS_VIRT_READ_PIN_AVG:
		return pmbus_read_word_data(client, page, phase, TPS_READ_PIN_AVG);

	case PMBUS_VIRT_READ_PIN_MAX:
		return pmbus_read_word_data(client, page, phase, TPS_READ_PIN_PEAK);

	case PMBUS_VIRT_READ_VMON:
		return pmbus_read_word_data(client, page, phase, TPS_READ_VAUX);

	case PMBUS_VIRT_SAMPLES: {
		ret = pmbus_read_byte_data(client, page, TPS_PK_MIN_AVG);
		if (ret < 0)
			return ret;
		return 1 << FIELD_GET(PK_MIN_AVG_AVG_CNT, ret);
	}
	case PMBUS_VIRT_RESET_TEMP_HISTORY:
	case PMBUS_VIRT_RESET_VIN_HISTORY:
	case PMBUS_VIRT_RESET_IIN_HISTORY:
	case PMBUS_VIRT_RESET_PIN_HISTORY:
	case PMBUS_VIRT_RESET_VOUT_HISTORY:
		return 0;

	case PMBUS_VIN_OV_FAULT_LIMIT:
		ret = pmbus_read_word_data(client, page, phase, reg);
		if (ret < 0)
			return ret;
		/* Convert from DIRECT format to linear value in mV */
		ret = ret * TPS1689_VIN_OVF_NUM + TPS1689_VIN_OVF_OFF_IN;
		return DIV_ROUND_CLOSEST(ret, TPS1689_VIN_OVF_DIV);

	case PMBUS_IIN_OC_FAULT_LIMIT: {
		u64 vref_mv;

		if (!data->rimon_uohms)
			return -EOPNOTSUPP;

		ret = pmbus_read_byte_data(client, page, TPS_VIREF);
		if (ret < 0)
			return ret;
		/* Convert raw VIREF value to millivolts */
		vref_mv = (u64)ret * (TPS1689_VIREF_MAX_MV - TPS1689_VIREF_MIN_MV);
		vref_mv = DIV_ROUND_CLOSEST(vref_mv, TPS1689_VIREF_STEPS) + TPS1689_VIREF_MIN_MV;

		/* I_ocp (mA) = (Vref (mV) * 1,000,000,000) / (G_imon (nA/A) * R_imon (uOhms)) */
		return DIV_ROUND_CLOSEST_ULL(vref_mv * 1000000000ULL,
						 (u64)TPS1689_G_IMON_NA * data->rimon_uohms);
	}
	case PMBUS_IIN_OC_WARN_LIMIT:
		ret = pmbus_read_word_data(client, page, phase, reg);
		if (ret < 0)
			return ret;
		/* Align 8-bit register to 10-bit scaling used by curr1_input */
		return ret << TPS1689_8B_SHIFT;

	default:
		/* All other limits handled by PMBus core with scaled coefficients */
		return pmbus_read_word_data(client, page, phase, reg);
	}
}

static int tps1689_write_word_data(struct i2c_client *client, int page, int reg, u16 value)
{
	struct tps1689_data *data = i2c_get_clientdata(client);

	switch (reg) {
	case PMBUS_VIRT_SAMPLES:
		value = clamp_val(value, 1, 1 << FIELD_GET(PK_MIN_AVG_AVG_CNT, 0x07));
		value = ilog2(value);
		return pmbus_update_byte_data(client, page, TPS_PK_MIN_AVG,
					  PK_MIN_AVG_AVG_CNT,
					  FIELD_PREP(PK_MIN_AVG_AVG_CNT, value));

	case PMBUS_VIRT_RESET_TEMP_HISTORY:
	case PMBUS_VIRT_RESET_VIN_HISTORY:
	case PMBUS_VIRT_RESET_IIN_HISTORY:
	case PMBUS_VIRT_RESET_PIN_HISTORY:
	case PMBUS_VIRT_RESET_VOUT_HISTORY:
		return pmbus_update_byte_data(client, page, TPS_PK_MIN_AVG,
					  PK_MIN_AVG_RST_MASK,
					  PK_MIN_AVG_RST_MASK);

	case PMBUS_VIN_OV_FAULT_LIMIT:
		/* Convert from linear value in mV to DIRECT format */
		value = (s16)value * TPS1689_VIN_OVF_DIV - TPS1689_VIN_OVF_OFF_OUT;
		value = DIV_ROUND_CLOSEST(value, TPS1689_VIN_OVF_NUM);
		value = clamp_val(value, 0, 0xff);
		return pmbus_write_word_data(client, page, reg, value);

	case PMBUS_IIN_OC_FAULT_LIMIT: {
		u64 vref_mv;
		u32 raw_vref;

		if (!data->rimon_uohms)
			return -EOPNOTSUPP;

		/* value is in mA. Vref(mV) = (I_ocp(mA) * G_imon(nA/A) * R_imon(uOhms)) / 1,000,000,000 */
		vref_mv = (u64)value * TPS1689_G_IMON_NA * data->rimon_uohms;
		vref_mv = DIV_ROUND_CLOSEST_ULL(vref_mv, 1000000000ULL);

		if (vref_mv < TPS1689_VIREF_MIN_MV || vref_mv > TPS1689_VIREF_MAX_MV)
			return -EINVAL;

		/* Convert millivolts to raw VIREF value */
		raw_vref = vref_mv - TPS1689_VIREF_MIN_MV;
		raw_vref = DIV_ROUND_CLOSEST(raw_vref * TPS1689_VIREF_STEPS,
						 TPS1689_VIREF_MAX_MV - TPS1689_VIREF_MIN_MV);

		return pmbus_write_byte_data(client, page, TPS_VIREF,
						 clamp_val(raw_vref, 0, 0x3f));
	}
	case PMBUS_IIN_OC_WARN_LIMIT:
		value >>= TPS1689_8B_SHIFT;
		value = clamp_val(value, 0, 0xff);
		return pmbus_write_word_data(client, page, reg, value);

	default:
		/* All other limits handled by PMBus core with scaled coefficients */
		return pmbus_write_word_data(client, page, reg, value);
	}
}

static const struct pmbus_driver_info tps1689_base_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.m[PSC_VOLTAGE_IN] = 1166,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = -2,
	.format[PSC_VOLTAGE_OUT] = direct,
	.m[PSC_VOLTAGE_OUT] = 1166,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = -2,
	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_TEMPERATURE] = 140,
	.b[PSC_TEMPERATURE] = 32103,
	.R[PSC_TEMPERATURE] = -2,
	/* Base m coefficients (for 2.55k RIMON); will be scaled at probe time */
	.format[PSC_CURRENT_IN] = direct,
	.m[PSC_CURRENT_IN] = 24347,
	.b[PSC_CURRENT_IN] = 0,
	.R[PSC_CURRENT_IN] = -3,
	.format[PSC_POWER] = direct,
	.m[PSC_POWER] = 2775,
	.b[PSC_POWER] = 0,
	.R[PSC_POWER] = -4,
	/*
	 * Statically declare capabilities for backward compatibility.
	 * The PMBUS_HAVE_STATUS_WORD flag was added in kernel 5.14.
	 * On older kernels, the core probes for STATUS_WORD automatically.
	 */
	.func[0] = (PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_VMON |
		    PMBUS_HAVE_IIN | PMBUS_HAVE_PIN | PMBUS_HAVE_TEMP |
		    PMBUS_HAVE_SAMPLES | PMBUS_HAVE_STATUS_VOUT |
		    PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
		    PMBUS_HAVE_STATUS_TEMP),
	.read_word_data = tps1689_read_word_data,
	.write_word_data = tps1689_write_word_data,
	.read_byte_data = tps1689_read_byte_data,
	.write_byte_data = tps1689_write_byte_data,
};

static const struct i2c_device_id tps1689_i2c_id[] = {
	{ "tps1689" },
	{}
};
MODULE_DEVICE_TABLE(i2c, tps1689_i2c_id);

static const struct of_device_id tps1689_of_match[] = {
	{ .compatible = "ti,tps1689" },
	{}
};
MODULE_DEVICE_TABLE(of, tps1689_of_match);

static int tps1689_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct tps1689_data *data;
	struct pmbus_driver_info *info;
	u64 rimon_val;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Start with a copy of the base device information */
	info = &data->info;
	memcpy(info, &tps1689_base_info, sizeof(*info));

	/* Set a default rimon value */
	rimon_val = TPS1689_BOARD_RIMON;

	/*
	 * Try to read the rimon property from the device tree.
	 * Ignore any errors and use the default value if not found.
	 */
	if (!device_property_read_u64(dev, "ti,rimon-micro-ohms", &rimon_val))
		dev_info(dev, "Using RIMON value of %llu micro-ohms from device tree\n", rimon_val);

	/* Store the rimon value in our private data for later use */
	data->rimon_uohms = rimon_val;

	/* Adapt the current and power scale for this instance */
	tps1689_set_m_for_rimon(&info->m[PSC_CURRENT_IN], data->rimon_uohms);
	tps1689_set_m_for_rimon(&info->m[PSC_POWER], data->rimon_uohms);

	i2c_set_clientdata(client, data);

	return pmbus_do_probe(client, info);
}

static struct i2c_driver tps1689_driver = {
	.driver = {
		.name = "tps1689",
		.of_match_table = tps1689_of_match,
	},
	.probe = tps1689_probe,
	.id_table = tps1689_i2c_id,
};
module_i2c_driver(tps1689_driver);

MODULE_AUTHOR("Hameem Hamza <hameem@ti.com>");
MODULE_DESCRIPTION("PMBUS driver for TPS1689 eFuse");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS_PMBUS;
MODULE_VERSION(BSP_VERSION);
