/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Nexthop Systems Inc. */

#ifndef _NH_IRQ_H_
#define _NH_IRQ_H_

#include <linux/types.h>

struct pci_dev;
struct clk_hw;

/* Resolve an I2C master's BAR-relative CSR offset to its virq and reference
 * clock; sets up @pdev's FPGA on first call. -ENOENT when interrupt-driven
 * I2C is unavailable on this FPGA. */
int nh_irq_resolve(struct pci_dev *pdev, u32 csr_offset_rel, int *virq_out,
		   struct clk_hw **clk_hw_out);

/* Per-FPGA LED register RMW lock; NULL if @pdev is not a Nexthop FPGA. */
struct mutex *nh_irq_led_reg_lock(struct pci_dev *pdev);

/* The FPGA's BAR0 mapping; NULL if @pdev is not a Nexthop FPGA. */
void __iomem *nh_irq_mmio_base(struct pci_dev *pdev);

#endif /* _NH_IRQ_H_ */
