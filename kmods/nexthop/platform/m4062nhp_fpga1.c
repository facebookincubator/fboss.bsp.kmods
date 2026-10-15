// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.

#include <linux/types.h>
#include "nh_platform.h"
#include "m4062nhp_common.h"

/* ============================================================================
 * Transceiver control
 *
 * FPGA1 carries reset / high-power / present signals for all 128 OSFP
 * ports in four 32-slot register windows, keyed by physical slot.
 *
 *   Window   Reset   Hi-Pwr   Present   Slots
 *   A        0x006C  0x0070   0x0074    1-32
 *   B        0x0030  0x0034   0x0038    33-64
 *   C        0x0044  0x0048   0x004C    65-96
 *   D        0x0058  0x005C   0x0060    97-128
 * ============================================================================ */

static const struct xcvr_port_group m4062nhp_fpga1_xcvr_groups[] = {
	{
		.start_port = 1,
		.end_port = 32,
		.reset_reg_offset = 0x006C,
		.lp_mode_reg_offset = 0x0070,
		.present_reg_offset = 0x0074,
	},
	{
		.start_port = 33,
		.end_port = 64,
		.reset_reg_offset = 0x0030,
		.lp_mode_reg_offset = 0x0034,
		.present_reg_offset = 0x0038,
	},
	{
		.start_port = 65,
		.end_port = 96,
		.reset_reg_offset = 0x0044,
		.lp_mode_reg_offset = 0x0048,
		.present_reg_offset = 0x004C,
	},
	{
		.start_port = 97,
		.end_port = 128,
		.reset_reg_offset = 0x0058,
		.lp_mode_reg_offset = 0x005C,
		.present_reg_offset = 0x0060,
	},
};

static const struct xcvr_ctrl_config m4062nhp_fpga1_xcvr = {
	.groups = m4062nhp_fpga1_xcvr_groups,
	.num_groups = ARRAY_SIZE(m4062nhp_fpga1_xcvr_groups),
	.id_to_slot = m4062nhp_xcvr_id_to_slot,
	.id_to_slot_len = ARRAY_SIZE(m4062nhp_xcvr_id_to_slot),
};

/* ============================================================================
 * I2C mux
 *
 * Two mezzanine i2c masters each get their own select register on FPGA1
 * (revision 0x0304 split). nh_fpga_mux writes the channel index into
 * bits [1:0] of the named register, matching the layout:
 *   master 7 (Bottom Mezz Card): reg 0x0010
 *   master 8 (Top Mezz Card)   : reg 0x0014
 * ============================================================================ */

static const struct mux_config m4062nhp_fpga1_mux[] = {
	{ 7, 4, 0x0010, "Bottom Mezz MUX control" },
	{ 8, 4, 0x0014, "Top Mezz MUX control" },
};

/* ============================================================================
 * Port LED
 *
 * FPGA1 drives the mezzanine front-panel port LEDs in eight 32-bit
 * control registers, packed 8 ports per register with a 4-bit field per
 * port (bit 0 = amber on, bit 1 = blue on, bits 3:2 = blink mode; the
 * blink bits are not exposed by the current driver).
 *
 *   Mezz 0 (ports 97-128): 0x80, 0x84, 0x88, 0x8C
 *   Mezz 1 (ports   1-32): 0x90, 0x94, 0x98, 0x9C
 * ============================================================================ */

/* { start_port, end_port, offset, bits_per_port, color_bit_offset } */
static const struct port_led_range m4062nhp_fpga1_amber_ranges[] = {
	{ 1, 8, 0x90, 4 },     { 9, 16, 0x94, 4 },    { 17, 24, 0x98, 4 },
	{ 25, 32, 0x9C, 4 },   { 97, 104, 0x80, 4 },  { 105, 112, 0x84, 4 },
	{ 113, 120, 0x88, 4 }, { 121, 128, 0x8C, 4 },
};

static const struct port_led_range m4062nhp_fpga1_blue_ranges[] = {
	{ 1, 8, 0x90, 4, 1 },	  { 9, 16, 0x94, 4, 1 },
	{ 17, 24, 0x98, 4, 1 },	  { 25, 32, 0x9C, 4, 1 },
	{ 97, 104, 0x80, 4, 1 },  { 105, 112, 0x84, 4, 1 },
	{ 113, 120, 0x88, 4, 1 }, { 121, 128, 0x8C, 4, 1 },
};

static const struct port_led_color_addr m4062nhp_fpga1_port_led[PORT_LED_NUM_COLORS] = {
	[PORT_LED_AMBER] = {
		.num_ranges = ARRAY_SIZE(m4062nhp_fpga1_amber_ranges),
		.ranges = m4062nhp_fpga1_amber_ranges,
	},
	[PORT_LED_BLUE] = {
		.num_ranges = ARRAY_SIZE(m4062nhp_fpga1_blue_ranges),
		.ranges = m4062nhp_fpga1_blue_ranges,
	},
};

/* ============================================================================
 * Fan
 *
 * Eight dual-rotor fans across two fan cards (chains), four fans each.
 *
 * Registers (all in the FPGA1 global window):
 *   Fan Card Control (LEDs)      : 0x00A0  (per-fan red/green bits, active-low)
 *   Fan Card 0 / 1 Status        : 0x00A4 / 0x00A8  (PG, module-id, card-type)
 *   State-change flags 0 / 1     : 0x01D0 / 0x01D4
 *   Per-fan Tach Status          : chain 0 0x00B4..0x00C0, chain 1 0x00C8..0x00D4
 *                                  (inner period [15:0], outer period [31:16])
 *   Per-fan PWM Control          : chain 0 0x0240..0x024C, chain 1 0x0250..0x025C
 *                                  (duty [7:0], max [23:16])
 *
 * PWM and tach live in distinct registers, so pwm_offset differs from
 * inner_tach_offset/outer_tach_offset. Inner and outer tach share one
 * register, distinguished by inner_tach_mask / outer_tach_mask.
 *
 * The status register has no per-fan tach-OK bits (only PG / module-id /
 * card-type), so .inner_tach / .outer_tach are left 0.
 *
 * LED bit layout in 0x00A0 (active-low, default fail-on), per chain-local fan:
 *   chain 0: fan1 red=0/grn=1, fan2 red=2/grn=3, fan3 red=4/grn=5, fan4 red=6/grn=7
 *   chain 1: fan1 red=16/grn=17, fan2 red=18/grn=19, fan3 red=20/grn=21, fan4 red=22/grn=23
 * Software fan1-4 map to chain 1 (top row), fan5-8 to chain 0 (bottom row).
 * ============================================================================ */

static const struct fan m4062nhp_fpga1_fans[] = {
	/*
	 * Software fan1-8 are numbered top row then bottom row to match the
	 * physical chassis. The top row is wired to chain 1 and the
	 * bottom row to chain 0, so fan1-4 use the chain-1 register set and
	 * fan5-8 use chain 0.
	 */
	/* ---- fan1-4 = top row (chain 1): status 0x00A8, flags 0x01D4 ---- */
	[0] = {
		.status_offset = 0x00A8,
		.status_change_flags_offset = 0x01D4,
		.pwrgood = BIT(8),
		.module_id_mask = GENMASK(7, 4),
		.module_id_shift = 0x4,
		.ctrl_fail_led_bit = BIT(16),
		.ctrl_good_led_bit = BIT(17),
		.pwm_offset = 0x0250,
		.inner_tach_offset = 0x00C8,
		.outer_tach_offset = 0x00C8,
	},
	[1] = {
		.status_offset = 0x00A8,
		.status_change_flags_offset = 0x01D4,
		.pwrgood = BIT(15),
		.module_id_mask = GENMASK(14, 11),
		.module_id_shift = 0xb,
		.ctrl_fail_led_bit = BIT(18),
		.ctrl_good_led_bit = BIT(19),
		.pwm_offset = 0x0254,
		.inner_tach_offset = 0x00CC,
		.outer_tach_offset = 0x00CC,
	},
	[2] = {
		.status_offset = 0x00A8,
		.status_change_flags_offset = 0x01D4,
		.pwrgood = BIT(22),
		.module_id_mask = GENMASK(21, 18),
		.module_id_shift = 0x12,
		.ctrl_fail_led_bit = BIT(20),
		.ctrl_good_led_bit = BIT(21),
		.pwm_offset = 0x0258,
		.inner_tach_offset = 0x00D0,
		.outer_tach_offset = 0x00D0,
	},
	[3] = {
		.status_offset = 0x00A8,
		.status_change_flags_offset = 0x01D4,
		.pwrgood = BIT(29),
		.module_id_mask = GENMASK(28, 25),
		.module_id_shift = 0x19,
		.ctrl_fail_led_bit = BIT(22),
		.ctrl_good_led_bit = BIT(23),
		.pwm_offset = 0x025C,
		.inner_tach_offset = 0x00D4,
		.outer_tach_offset = 0x00D4,
	},
	/* ---- fan5-8 = bottom row (chain 0): status 0x00A4, flags 0x01D0 ---- */
	[4] = {
		.status_offset = 0x00A4,
		.status_change_flags_offset = 0x01D0,
		.pwrgood = BIT(8),
		.module_id_mask = GENMASK(7, 4),
		.module_id_shift = 0x4,
		.ctrl_fail_led_bit = BIT(0),
		.ctrl_good_led_bit = BIT(1),
		.pwm_offset = 0x0240,
		.inner_tach_offset = 0x00B4,
		.outer_tach_offset = 0x00B4,
	},
	[5] = {
		.status_offset = 0x00A4,
		.status_change_flags_offset = 0x01D0,
		.pwrgood = BIT(15),
		.module_id_mask = GENMASK(14, 11),
		.module_id_shift = 0xb,
		.ctrl_fail_led_bit = BIT(2),
		.ctrl_good_led_bit = BIT(3),
		.pwm_offset = 0x0244,
		.inner_tach_offset = 0x00B8,
		.outer_tach_offset = 0x00B8,
	},
	[6] = {
		.status_offset = 0x00A4,
		.status_change_flags_offset = 0x01D0,
		.pwrgood = BIT(22),
		.module_id_mask = GENMASK(21, 18),
		.module_id_shift = 0x12,
		.ctrl_fail_led_bit = BIT(4),
		.ctrl_good_led_bit = BIT(5),
		.pwm_offset = 0x0248,
		.inner_tach_offset = 0x00BC,
		.outer_tach_offset = 0x00BC,
	},
	[7] = {
		.status_offset = 0x00A4,
		.status_change_flags_offset = 0x01D0,
		.pwrgood = BIT(29),
		.module_id_mask = GENMASK(28, 25),
		.module_id_shift = 0x19,
		.ctrl_fail_led_bit = BIT(6),
		.ctrl_good_led_bit = BIT(7),
		.pwm_offset = 0x024C,
		.inner_tach_offset = 0x00C0,
		.outer_tach_offset = 0x00C0,
	},
};

static const struct fan_card_status_addr m4062nhp_fpga1_fan = {
	.num_fan_controllers = ARRAY_SIZE(m4062nhp_fpga1_fans),
	.ctrl_offset = 0x00A0,
	.card_type_mask = GENMASK(2, 0),
	.pwm_mask = 0xFF, /* PWM duty in [7:0] */
	.inner_tach_mask = GENMASK(15, 0),
	.outer_tach_mask = GENMASK(31, 16),
	.num_rotors = 2,
	.fans = m4062nhp_fpga1_fans,
};

/* Interrupt topology: 3 MSI vectors, 74 masters */
static const struct nh_fpga_msi_domain_cfg m4062nhp_fpga1_msi_domains[] = {
	{ .chip_name = "m4062-fpga1-msi-0",
	  .ier_offset = 0x170,
	  .isr_offset = 0x174,
	  .first_master = 0,
	  .last_master = 9,
	  .first_hw_irq = 8 },
	{ .chip_name = "m4062-fpga1-msi-1",
	  .ier_offset = 0x178,
	  .isr_offset = 0x17c,
	  .first_master = 10,
	  .last_master = 41,
	  .first_hw_irq = 0 },
	{ .chip_name = "m4062-fpga1-msi-2",
	  .ier_offset = 0x180,
	  .isr_offset = 0x184,
	  .first_master = 42,
	  .last_master = 73,
	  .first_hw_irq = 0 },
};

static const struct nh_fpga_irq_cfg m4062nhp_fpga1_irq_cfg = {
	.domains = m4062nhp_fpga1_msi_domains,
	.num_domains = ARRAY_SIZE(m4062nhp_fpga1_msi_domains),
	.i2c_csr_base = 0x40000,
	.i2c_csr_channel_size = 0x200,
	.ref_clk_hz = 25000000,
	.min_fpga_version = 0x00,
};

const struct nh_platform_cfg nh_platform_m4062nhp_fpga1 = {
	.device_id = PCI_DEVICE_ID_NEXTHOP_M4062NHP_SMB1,
	.name = "M4062NHP_FPGA1",
	.fan_cfg = &m4062nhp_fpga1_fan,
	.xcvr_cfg = &m4062nhp_fpga1_xcvr,
	.mux_cfg = m4062nhp_fpga1_mux,
	.mux_count = ARRAY_SIZE(m4062nhp_fpga1_mux),
	.port_led_cfg = m4062nhp_fpga1_port_led,
	.num_ports_per_led = 128,
	.irq_cfg = &m4062nhp_fpga1_irq_cfg,
	.dual_color_per_led = true,
	.led_to_id = m4062nhp_led_to_id,
	.led_to_id_len = ARRAY_SIZE(m4062nhp_led_to_id),
	/* led_core_cfg: NULL/0. */
};
