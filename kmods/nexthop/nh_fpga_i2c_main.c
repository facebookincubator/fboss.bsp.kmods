// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.
//
// FPGA I2C controller driver for the auxiliary devices the fboss_iob_pci
// parent creates. Register-level support is added by a following change;
// this driver registers its boundary only.

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "fbiob-auxdev.h"
#include "nh_fpga_i2c.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_fpga_i2c"
#define DRIVER_VERSION "1.0"

int nh_fpga_i2c_probe(struct auxiliary_device *aux_dev,
		      const struct auxiliary_device_id *id)
{
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	const struct nh_platform_cfg *cfg = nh_get_platform_dev(pdev);

	if (!cfg)
		return -ENODEV;
	dev_info(&aux_dev->dev, "%s: I2C controller support not yet enabled\n",
		 cfg->name);
	return -EOPNOTSUPP;
}

void nh_fpga_i2c_remove(struct auxiliary_device *aux_dev)
{
}

static const struct auxiliary_device_id nh_fpga_i2c_aux_id_table[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER ".nh_i2c_master" },
	{ .name = FBOSS_IOB_PCI_DRIVER ".i2c_master_ext" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_fpga_i2c_aux_id_table);

static struct auxiliary_driver nh_fpga_i2c_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_fpga_i2c_probe,
	.remove = nh_fpga_i2c_remove,
	.id_table = nh_fpga_i2c_aux_id_table,
};

static int __init nh_fpga_i2c_init_module(void)
{
	return auxiliary_driver_register(&nh_fpga_i2c_aux_driver);
}

static void __exit nh_fpga_i2c_exit_module(void)
{
	auxiliary_driver_unregister(&nh_fpga_i2c_aux_driver);
}

module_init(nh_fpga_i2c_init_module);
module_exit(nh_fpga_i2c_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Arif Mohammad <marif@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA I2C Controller Driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_SOFTDEP("pre: i2c-xiic nh_xiic nh_fpga_mux nh_irq");
