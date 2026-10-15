// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.

#ifndef _NH_FPGA_PSU_H_
#define _NH_FPGA_PSU_H_

#include <linux/types.h>

struct nh_platform_cfg; /* forward declaration */

/*
 * Sentinel for a not-yet-characterized PSU-present register: the driver
 * stays idle (probe returns -ENODEV) rather than read an unknown register
 * and report bogus presence. Set the real offset to enable.
 */
#define NH_FPGA_PSU_OFFSET_UNSET 0xFFFFFFFFU

/*
 * Firmware-version read for PSUs sitting behind the FPGA I2C mux.
 *
 * The transfer goes through the mux child adapter registered by nh_fpga_mux,
 * so the channel select and bus locking are handled by the i2c.
 *
 * logical_master: index of the parent I2C master within the FPGA's I2C CSR
 *   block.
 * mux_channels: per-PSU mux channel, indexed by PSU slot. Must have
 *   psu_present_cfg.num_psus entries.
 * device_addr: PMBus slave address.
 * version_reg: PMBus FW revision register.
 * read_len: total bytes to read from version_reg.
 * payload_offset: index of the first version byte. Set to 1 for devices that
 *   answer with an SMBus-block-style leading byte count.
 */
struct psu_fw_cfg {
	u8 logical_master;
	const u8 *mux_channels;
	u16 device_addr;
	u8 version_reg;
	u8 read_len;
	u8 payload_offset;
};

/*
 * Per-platform layout of the PSU-present register.
 *
 * A single 32-bit register within the aux device's CSR block carries one
 * presence bit per PSU. The CSR block base is set by the fpgaIpBlockConfig
 * csrOffset in platform_manager.json; present_reg_offset is relative to
 * that base.
 *
 * num_psus: number of PSUs on this platform. Set to ARRAY_SIZE() of the
 * platform's present_masks array — no hardcoded cap needed.
 *
 * active_low: when true the hardware bit reads 0 for "present", so the driver
 * inverts it before reporting.
 */
struct psu_present_cfg {
	u32 present_reg_offset;
	u8 num_psus;
	bool active_low;
	const u32 *present_masks;
	const struct psu_fw_cfg *fw_cfg;
};

#endif /* _NH_FPGA_PSU_H_ */
