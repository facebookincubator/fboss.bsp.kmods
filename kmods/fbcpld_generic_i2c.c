// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "lib/fbcpld-cdev.h"

#define DRIVER_NAME	"fbcpld_generic"

#define I2C_REG_VAL_BITS	8
#define I2C_REG_ADDR_BITS	8
#define I2C_REG_OFFSET_MAX	0xFF

static const struct regmap_config fbcpld_i2c_regmap_cfg = {
	.reg_bits = I2C_REG_ADDR_BITS,
	.val_bits = I2C_REG_VAL_BITS,
	.max_register = I2C_REG_OFFSET_MAX,
};

static int fbcpld_generic_probe(struct i2c_client *client)
{
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client, &fbcpld_i2c_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "failed to init regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	ret = fbcpld_cdev_init(client);
	if (ret) {
		dev_err(&client->dev, "failed to init cdev: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct i2c_device_id fbcpld_generic_id[] = {
	{ DRIVER_NAME, 0 },
	{ "fbg_smbcpld", 0 },
	{ "fbg_scmcpld", 0 },
	{ "fbg_mcbcpld", 0 },
	{ "fbg_fancpld", 0 },
	{ "fbg_pwrcpld", 0 },
	{ "fbg_piccpld", 0 },
	{ "fbg_rtmcpld", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fbcpld_generic_id);

static struct i2c_driver fbcpld_generic_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = fbcpld_generic_probe,
	.id_table = fbcpld_generic_id,
};
module_i2c_driver(fbcpld_generic_driver);

MODULE_AUTHOR("Meta Platforms, Inc.");
MODULE_DESCRIPTION("Generic FBOSS CPLD Driver with Runtime Sysfs Support");
MODULE_LICENSE("GPL");
MODULE_VERSION(BSP_VERSION);
