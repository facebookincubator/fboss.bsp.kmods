// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) 2026 Nexthop Systems Inc.
// Copyright (c) Meta Platforms, Inc. and affiliates.
/*
 * nh_fpga_asic_temp.h - NH FPGA ASIC Temperature Driver Header
 *
 * Type definitions for the ASIC temperature driver.
 * Per-platform data (register offsets, masks, shifts) lives in
 * platform/<sku>.c and is accessed via nh_get_platform().
 */

#ifndef _NH_FPGA_ASIC_TEMP_H_
#define _NH_FPGA_ASIC_TEMP_H_

#include <linux/types.h>

struct nh_platform_cfg; /* forward declaration */

/*
 * Per-platform layout of the ASIC Min/Max temperature status register.
 *
 * A single 32-bit register carries two 13-bit fields; the exact offsets,
 * masks, and shifts can vary by platform FPGA revision.
 */
struct asic_temp_config {
	u32 status_reg_offset;
	u32 max_mask;
	u32 max_shift;
	u32 min_mask;
	u32 min_shift;
};

#endif /* _NH_FPGA_ASIC_TEMP_H_ */
