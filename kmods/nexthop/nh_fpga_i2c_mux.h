/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_FPGA_I2C_MUX_H_
#define _NH_FPGA_I2C_MUX_H_

#include <linux/types.h>

struct i2c_msg;
struct pci_dev;

/*
 * Platform data for FPGA mux devices
 * This is passed to the custom FPGA mux driver via platform_data
 */
struct fpga_mux_platform_data {
	void __iomem *mux_reg; /* FPGA mux control register */
	int num_channels; /* Number of mux channels */
	struct pci_dev *pdev; /* FPGA owning the parent master */
	u8 logical_master; /* mux_config.logical_master of the parent */
};

/*
 * Run a transfer on one mux channel, addressed by the owning FPGA and the
 * parent master's logical index.
 *
 * Deliberately not addressed by adapter name: the "#N" in the name is the
 * auxiliary device id, which platform_manager assigns dynamically, and the
 * rest of the name has already changed shape once. Neither is under this
 * driver's control.
 *
 * Returns the i2c_transfer() result, or -ENODEV if no such channel is
 * registered.
 */
int nh_fpga_mux_channel_xfer(struct pci_dev *pdev, u8 logical_master,
			     u8 channel, struct i2c_msg *msgs, int num);

#endif /* _NH_FPGA_I2C_MUX_H_ */
