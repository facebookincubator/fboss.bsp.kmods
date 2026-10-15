/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_FPGA_I2C_MASTERS_H_
#define _NH_FPGA_I2C_MASTERS_H_

#include <linux/types.h>

/* Mux configuration for a single I2C master */
struct mux_config {
	int logical_master;
	int num_channels;
	u32 reg_offset;
	const char *description;
};

#endif /* _NH_FPGA_I2C_MASTERS_H_ */
