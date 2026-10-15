/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_XCVR_CTRL_H_
#define _NH_XCVR_CTRL_H_

#include <linux/auxiliary_bus.h>

struct nh_platform_cfg; /* forward declaration — full definition in platform/nh_platform.h */

/* One contiguous range of ports sharing a single 32-bit (reset, lp_mode,
 * present) register triple. Each group maps to one physical 32-bit
 * register per attribute; the port's bit position within the register is
 * (port_num - start_port). A platform with multiple register windows
 * (per port card / mezzanine / FPGA-internal half) declares one entry
 * per window, so end_port - start_port is at most 31.
 *
 * A dedicated single-port block (e.g. the management QSFP) whose three
 * signals do not share one bit index across the register triple sets
 * fixed_bits and the per-signal *_bit fields instead; the port-offset
 * rule above is then bypassed.
 */
struct xcvr_port_group {
	u32 start_port; /* inclusive, 1-based */
	u32 end_port; /* inclusive, 1-based; end_port - start_port < 32 */
	u32 reset_reg_offset;
	u32 lp_mode_reg_offset;
	u32 present_reg_offset;
	bool fixed_bits; /* use reset_bit/lp_mode_bit/present_bit verbatim */
	u32 reset_bit;
	u32 lp_mode_bit;
	u32 present_bit;
	/* If true, writing 1 to the register bit puts the module in reset;
	 * if false, writing 0 does. The polarity is platform dependent. On
	 * Nexthop platforms OSFP/QSFP ResetL is commonly active low (leave
	 * unset) and SFP TX_DISABLE is commonly active high (set true). */
	bool reset_active_high;
	/* No LPMode pin: xcvr_low_power is a software shadow. */
	bool no_lp_mode;
};

/* Per-device xcvr control structure */
struct nh_xcvr_ctrl {
	struct auxiliary_device *aux_dev;
	void __iomem *base;
	u32 port_num;
	/* Bit index within each signal's register. For OSFP groups all three
	 * equal (port_num - group->start_port); for a fixed_bits group they
	 * take the group's per-signal values. */
	u32 reset_bit;
	u32 lp_mode_bit;
	u32 present_bit;
	u8 lp_mode_shadow; /* no_lp_mode groups */
	const struct xcvr_port_group *group; /* cached group for this port */
	const struct nh_platform_cfg
		*platform_cfg; /* Per-device platform descriptor */
};

/* Per-platform xcvr register layout. @groups is keyed by slot (the register
 * window numbering). Set @id_to_slot only when transceiver id != slot
 * (m4062nhp): id_to_slot[id] gives the slot for @groups, while id names the
 * sysfs attrs. NULL means slot == id. */
struct xcvr_ctrl_config {
	const struct xcvr_port_group *groups;
	u32 num_groups;
	const u32 *id_to_slot; /* id_to_slot[id], 1-based, NULL == identity */
	u32 id_to_slot_len; /* number of valid entries, i.e. max id + 1 */
};

/* Look up the port group whose [start_port, end_port] range covers
 * @slot (physical slot space; see xcvr_ctrl_config::id_to_slot above).
 * Returns NULL if no group matches.
 */
const struct xcvr_port_group *
xcvr_ctrl_find_port_group(const struct xcvr_ctrl_config *cfg, u32 slot);

/* Resolve @port_num (transceiver id, as named in sysfs) to its physical
 * slot via @cfg's id_to_slot table, or return it unchanged if the platform
 * did not set one (identity mapping).
 */
u32 xcvr_ctrl_slot_for_port(const struct xcvr_ctrl_config *cfg, u32 port_num);

#endif /* _NH_XCVR_CTRL_H_ */
