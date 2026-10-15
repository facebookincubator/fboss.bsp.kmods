// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.
//
// Virtual I2C mux driver for the channel-select registers of the Nexthop
// FPGA I2C masters. Register-level support is added by a following change;
// this driver registers its boundary only.

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "nh_fpga_i2c_mux.h"

#define DRIVER_NAME "nh_fpga_mux"
#define DRIVER_VERSION "1.0"

int nh_fpga_mux_channel_xfer(struct pci_dev *pdev, u8 logical_master,
			     u8 channel, struct i2c_msg *msgs, int num)
{
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL_GPL(nh_fpga_mux_channel_xfer);

static int fpga_mux_probe(struct i2c_client *client)
{
	dev_info(&client->dev, "FPGA mux support not yet enabled\n");
	return -EOPNOTSUPP;
}

static void fpga_mux_remove(struct i2c_client *client)
{
}

static const struct i2c_device_id fpga_mux_id[] = { { "fpga-mux", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, fpga_mux_id);

static struct i2c_driver fpga_mux_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = fpga_mux_probe,
	.remove = fpga_mux_remove,
	.id_table = fpga_mux_id,
};

static int __init fpga_mux_init(void)
{
	return i2c_add_driver(&fpga_mux_driver);
}

static void __exit fpga_mux_exit(void)
{
	i2c_del_driver(&fpga_mux_driver);
}

module_init(fpga_mux_init);
module_exit(fpga_mux_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Arif Mohammad <marif@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA I2C Mux Driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_SOFTDEP("pre: i2c-mux");
