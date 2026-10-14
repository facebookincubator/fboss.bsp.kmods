// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.
//
// Transceiver control for the Nexthop FPGAs under the fboss_iob_pci parent:
// per-port reset, low-power mode and presence. Register-level support is
// added by a following change; this driver registers its boundary only.

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "fbiob-auxdev.h"
#include "nh_xcvr_ctrl.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_xcvr_ctrl"
#define DRIVER_VERSION "1.0"

u32 xcvr_ctrl_slot_for_port(const struct xcvr_ctrl_config *cfg, u32 port_num)
{
	return port_num;
}

static int nh_xcvr_ctrl_probe(struct auxiliary_device *aux_dev,
			      const struct auxiliary_device_id *id)
{
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	const struct nh_platform_cfg *cfg = nh_get_platform_dev(pdev);

	if (!cfg)
		return -ENODEV;
	dev_info(&aux_dev->dev, "%s: transceiver control not yet enabled\n", cfg->name);
	return -EOPNOTSUPP;
}

static void nh_xcvr_ctrl_remove(struct auxiliary_device *aux_dev)
{
}

static const struct auxiliary_device_id nh_xcvr_ctrl_aux_id_table[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER ".nh_xcvr_ctrl" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_xcvr_ctrl_aux_id_table);

static struct auxiliary_driver nh_xcvr_ctrl_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_xcvr_ctrl_probe,
	.remove = nh_xcvr_ctrl_remove,
	.id_table = nh_xcvr_ctrl_aux_id_table,
};

static int __init nh_xcvr_ctrl_init_module(void)
{
	return auxiliary_driver_register(&nh_xcvr_ctrl_aux_driver);
}

static void __exit nh_xcvr_ctrl_exit_module(void)
{
	auxiliary_driver_unregister(&nh_xcvr_ctrl_aux_driver);
}

module_init(nh_xcvr_ctrl_init_module);
module_exit(nh_xcvr_ctrl_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mike Zhang <mike@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA Transceiver Control Driver");
MODULE_VERSION(DRIVER_VERSION);
