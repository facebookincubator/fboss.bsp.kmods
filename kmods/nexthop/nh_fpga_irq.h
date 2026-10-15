/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Nexthop Systems Inc. */

#ifndef _NH_FPGA_IRQ_H_
#define _NH_FPGA_IRQ_H_

#include <linux/types.h>

struct nh_fpga_pci_dev;
struct clk_hw;

/* FPGA version register at BAR0+0; the low 16 bits are the firmware version. */
#define NH_FPGA_VERSION_REG_OFFSET 0x0000
#define NH_FPGA_VERSION_MASK 0xFFFF

/* One MSI vector's demux info. A regmap_irq_chip over its ISR/IER pair
 * plus the i2c masters mapped onto it. */
struct nh_fpga_msi_domain_cfg {
	const char *chip_name;
	u32 ier_offset; /* Interrupt Enable Register BAR offset */
	u32 isr_offset; /* Interrupt Status Register BAR offset */

	int first_master;
	int last_master;
	u8 first_hw_irq; /* hw_irq of first_master */
};

/* Per-SKU interrupt topology, referenced from struct nh_platform_cfg. */
struct nh_fpga_irq_cfg {
	const struct nh_fpga_msi_domain_cfg *domains;
	int num_domains; /* must equal num_msi_vectors */

	/* master_index = (csr_offset - i2c_csr_base) / i2c_csr_channel_size */
	u32 i2c_csr_base;
	u32 i2c_csr_channel_size;

	u32 ref_clk_hz; /* AXI IIC reference clock */
	u16 min_fpga_version; /* min FPGA minor revision ([15:0] of the BAR0+0
			       * revision register) for interrupt-driven I2C;
			       * 0 disables the gate */
};

/* Alloc MSI, build the BAR0 regmap, register the irq chips + ref clock.
 * No-op (returns 0) when fbiob_dev->irq_cfg is NULL. */
int nh_fpga_irq_init(struct nh_fpga_pci_dev *fbiob_dev);

/* Resolve a controller's CSR offset to its Linux virq and ref clk_hw. */
int nh_fpga_resolve_master_irq_clk(struct nh_fpga_pci_dev *fbiob_dev,
				   u32 csr_offset, int *virq_out,
				   struct clk_hw **clk_hw_out);

#endif /* _NH_FPGA_IRQ_H_ */
