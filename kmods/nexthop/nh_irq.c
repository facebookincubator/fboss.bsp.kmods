// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.
//
// MSI vectors and reference clock for the Nexthop FPGAs under the
// fboss_iob_pci parent, set up per FPGA on first use by nh_fpga_i2c and the
// LED drivers. Register-level support is added by a following change; this
// module exports its interface only.

#include <linux/clk-provider.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>

#include "nh_irq.h"

#define DRIVER_NAME "nh_irq"
#define DRIVER_VERSION "1.0"

int nh_irq_resolve(struct pci_dev *pdev, u32 csr_offset_rel, int *virq_out,
		   struct clk_hw **clk_hw_out)
{
	return -EOPNOTSUPP;
}
EXPORT_SYMBOL_GPL(nh_irq_resolve);

struct mutex *nh_irq_led_reg_lock(struct pci_dev *pdev)
{
	return NULL;
}
EXPORT_SYMBOL_GPL(nh_irq_led_reg_lock);

void __iomem *nh_irq_mmio_base(struct pci_dev *pdev)
{
	return NULL;
}
EXPORT_SYMBOL_GPL(nh_irq_mmio_base);

static int __init nh_irq_init(void)
{
	pr_info("%s loaded\n", DRIVER_NAME);
	return 0;
}

static void __exit nh_irq_exit(void)
{
	pr_info("%s unloaded\n", DRIVER_NAME);
}

module_init(nh_irq_init);
module_exit(nh_irq_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Spandan Chowdhury <spandan@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA IRQ/clock driver");
MODULE_VERSION(DRIVER_VERSION);
