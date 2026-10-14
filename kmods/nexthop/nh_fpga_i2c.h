/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_FPGA_I2C_H_
#define _NH_FPGA_I2C_H_

#include <linux/i2c.h>
#include <linux/auxiliary_bus.h>
#include <linux/platform_device.h>
#include <linux/clkdev.h>

#include "nh_fpga_i2c_mux.h"

/* I2C shim state; the adapter itself is owned by the child xiic-i2c pdev. */
struct nh_fpga_i2c {
	struct auxiliary_device *aux_dev;

	/* i2c mux support */
	struct i2c_client *mux_client; /* fpga-mux client on the master adapter */
	void __iomem *mux_reg; /* FPGA mux control register */
	int num_mux_channels; /* Number of mux channels (0 = no mux) */
	struct fpga_mux_platform_data mux_pdata; /* passed to nh_fpga_mux */

	/* xiic-i2c child platform_device + its ref-clock alias */
	struct platform_device *pdev;
	struct clk_lookup *cl;

	/* Master adapter bus number (set by the notifier); -1 if none. */
	int master_bus_num;
};

/* Function prototypes */
int nh_fpga_i2c_probe(struct auxiliary_device *aux_dev,
		      const struct auxiliary_device_id *id);
void nh_fpga_i2c_remove(struct auxiliary_device *aux_dev);

#endif /* _NH_FPGA_I2C_H_ */
