// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.

#include <linux/bits.h>
#include <linux/types.h>
#include "nh_platform.h"

/* ============================================================================
 * LED — PSU / FAN / SYS status LEDs
 *
 * The "QSFP Management Card LED Control" register at offset 0x009C packs
 * 4-bit LED control fields:
 *   bits [3:0]   PSU LED  ([0]=good, [1]=bad, [2]=blink, [3]=rsvd)
 *   bits [7:4]   FAN LED  (same field shape)
 *   bits [11:8]  SYS LED  (same field shape)
 *   bits [23:20] MGMT QSFP2 LED — driven as port LED 65.
 * The LEDs are blue (good) / yellow (bad), so they are named blue/amber.
 * ============================================================================
 */

static const char *const m4061nhp_led_color_names[NH_LED_BICOLOR_NUM_COLORS] = {
	[NH_LED_BICOLOR_FAIL] = "amber",
	[NH_LED_BICOLOR_GOOD] = "blue",
};

static const struct nh_led_core_info m4061nhp_led_info[NH_LED_TYPE_MAX][NH_LED_BICOLOR_NUM_COLORS] = {
	[NH_LED_TYPE_PSU] = {
		[NH_LED_BICOLOR_FAIL] = {
			.name = "psu_led",
			.color_offset = 1,
			.color_field_width = 1,
			.blink_bit = 2,
			.colors = {NH_LED_BICOLOR_FAIL},
		},
		[NH_LED_BICOLOR_GOOD] = {
			.name = "psu_led",
			.color_offset = 0,
			.color_field_width = 1,
			.blink_bit = 2,
			.colors = {NH_LED_BICOLOR_GOOD},
		}
	},
	[NH_LED_TYPE_FAN] = {
		[NH_LED_BICOLOR_FAIL] = {
			.name = "fan_led",
			.color_offset = 5,
			.color_field_width = 1,
			.blink_bit = 6,
			.colors = {NH_LED_BICOLOR_FAIL},
		},
		[NH_LED_BICOLOR_GOOD] = {
			.name = "fan_led",
			.color_offset = 4,
			.color_field_width = 1,
			.blink_bit = 6,
			.colors = {NH_LED_BICOLOR_GOOD},
		}
	},
	[NH_LED_TYPE_SYS] = {
		[NH_LED_BICOLOR_FAIL] = {
			.name = "sys_led",
			.color_offset = 9,
			.color_field_width = 1,
			.blink_bit = 10,
			.colors = {NH_LED_BICOLOR_FAIL},
		},
		[NH_LED_BICOLOR_GOOD] = {
			.name = "sys_led",
			.color_offset = 8,
			.color_field_width = 1,
			.blink_bit = 10,
			.colors = {NH_LED_BICOLOR_GOOD},
		}
	},
};

static const struct nh_led_core_config m4061nhp_led_config = {
	.led_info = (const struct nh_led_core_info *)m4061nhp_led_info,
	.color_names = m4061nhp_led_color_names,
	.num_colors_per_led = 1,
	.num_leds = 2,
	.control_offset = 0x9c,
};

/* ============================================================================
 * Fan
 *
 * M4340NHP-style fan card: five dual-rotor trays. Per-tray PWM (0x0240..0x0250)
 * and tach are separate registers; tach period is [15:0] inner / [31:16]
 * outer. Tach for trays 1-4 is at 0x00B4..0x00C0 and for tray 5 at 0x00C8;
 * unlike M4340NHP, 0x00C4 reads 0. Trays 1-3 report ID/power-good in status
 * register 0x00A4, trays 4-5 in 0x00A8. The status registers carry no
 * tach-ok bits, so inner_tach/outer_tach are 0. Fan Card Control @ 0x00A0
 * holds a fail/good LED bit pair per tray at bits 2*(tray-1) / 2*(tray-1)+1.
 * ============================================================================
 */

static const struct fan m4061nhp_fans[] = {
	[0] = {
		.status_offset = 0x00A4,
		.status_change_flags_offset = 0x0194,
		.pwrgood = BIT(12),
		.module_id_mask = GENMASK(11, 8),
		.module_id_shift = 8,
		.ctrl_fail_led_bit = BIT(0),
		.ctrl_good_led_bit = BIT(1),
		.pwm_offset = 0x0240,
		.inner_tach_offset = 0x00B4,
		.outer_tach_offset = 0x00B4,
	},
	[1] = {
		.status_offset = 0x00A4,
		.status_change_flags_offset = 0x0194,
		.pwrgood = BIT(20),
		.module_id_mask = GENMASK(19, 16),
		.module_id_shift = 16,
		.ctrl_fail_led_bit = BIT(2),
		.ctrl_good_led_bit = BIT(3),
		.pwm_offset = 0x0244,
		.inner_tach_offset = 0x00B8,
		.outer_tach_offset = 0x00B8,
	},
	[2] = {
		.status_offset = 0x00A4,
		.status_change_flags_offset = 0x0194,
		.pwrgood = BIT(28),
		.module_id_mask = GENMASK(27, 24),
		.module_id_shift = 24,
		.ctrl_fail_led_bit = BIT(4),
		.ctrl_good_led_bit = BIT(5),
		.pwm_offset = 0x0248,
		.inner_tach_offset = 0x00BC,
		.outer_tach_offset = 0x00BC,
	},
	[3] = {
		.status_offset = 0x00A8,
		.status_change_flags_offset = 0x0198,
		.pwrgood = BIT(4),
		.module_id_mask = GENMASK(3, 0),
		.module_id_shift = 0,
		.ctrl_fail_led_bit = BIT(6),
		.ctrl_good_led_bit = BIT(7),
		.pwm_offset = 0x024C,
		.inner_tach_offset = 0x00C0,
		.outer_tach_offset = 0x00C0,
	},
	[4] = {
		.status_offset = 0x00A8,
		.status_change_flags_offset = 0x0198,
		.pwrgood = BIT(12),
		.module_id_mask = GENMASK(11, 8),
		.module_id_shift = 8,
		.ctrl_fail_led_bit = BIT(8),
		.ctrl_good_led_bit = BIT(9),
		.pwm_offset = 0x0250,
		.inner_tach_offset = 0x00C8,
		.outer_tach_offset = 0x00C8,
	},
};

static const struct fan_card_status_addr m4061nhp_fan = {
	.num_fan_controllers = ARRAY_SIZE(m4061nhp_fans),
	.ctrl_offset = 0x00A0,
	.card_type_mask = GENMASK(2, 0),
	.pwm_mask = 0xFF,
	.inner_tach_mask = 0x0000FFFF,
	.outer_tach_mask = 0xFFFF0000,
	/* fan shift chain sampled at 171.232 kHz. one tick = 5.84us. */
	.tach_time_unit_ns = 5840,
	.num_rotors = 2,
	.fans = m4061nhp_fans,
};

/* ============================================================================
 * Port LED
 *
 * 64 front-panel port LEDs at 0x60..0x7C, 8 ports per 32-bit register
 * with a 4-bit field per port: bit 0 = amber, bit 1 = blue, bits 3:2 =
 * blink mode (blink not exposed by this driver). Both colors share the
 * same register set with color_bit_offset 0 / 1.
 *
 * The mgmt QSFP28 cage (transceiver 65) is the card's QSFP2 position; its
 * LED is QSFP2 [23:20] of the QSFP Management Card LED Control register
 * 0x9C, shared with the psu/fan/sys LEDs.
 * ============================================================================
 */

/* { start_port, end_port, offset, bits_per_port, color_bit_offset } */
static const struct port_led_range m4061nhp_amber_ranges[] = {
	{ 1, 8, 0x60, 4 },   { 9, 16, 0x64, 4 },  { 17, 24, 0x68, 4 },
	{ 25, 32, 0x6C, 4 }, { 33, 40, 0x70, 4 }, { 41, 48, 0x74, 4 },
	{ 49, 56, 0x78, 4 }, { 57, 64, 0x7C, 4 }, { 65, 65, 0x9C, 4, 20 },
};

static const struct port_led_range m4061nhp_blue_ranges[] = {
	{ 1, 8, 0x60, 4, 1 },	{ 9, 16, 0x64, 4, 1 },	{ 17, 24, 0x68, 4, 1 },
	{ 25, 32, 0x6C, 4, 1 }, { 33, 40, 0x70, 4, 1 }, { 41, 48, 0x74, 4, 1 },
	{ 49, 56, 0x78, 4, 1 }, { 57, 64, 0x7C, 4, 1 }, { 65, 65, 0x9C, 4, 21 },
};

static const struct port_led_color_addr m4061nhp_port_led[PORT_LED_NUM_COLORS] = {
	[PORT_LED_AMBER] = {
		.num_ranges = ARRAY_SIZE(m4061nhp_amber_ranges),
		.ranges = m4061nhp_amber_ranges,
	},
	[PORT_LED_BLUE] = {
		.num_ranges = ARRAY_SIZE(m4061nhp_blue_ranges),
		.ranges = m4061nhp_blue_ranges,
	},
};

/* ============================================================================
 * Transceiver control
 *
 * OSFP 1-32 and 33-64 each have a 32-bit register window (0x0030 and
 * 0x0040) with reset / high-power / present at +0x0 / +0x4 / +0x8; the
 * high-power register stands in for lp_mode. Each port's aux device base
 * is set by the platform manager to the start of its window.
 *
 * The mgmt QSFP28 port 65 is a dedicated block with fixed bits (the
 * card's QSFP2 position; the QSFP1 bits are reserved):
 *   QSFP Management Card Control @ 0x0088: QSFP2 rst_l bit 8 / lpmode bit 10
 *   QSFP Management Card Status  @ 0x008C: QSFP2 prsnt_l bit 17
 * Its aux device uses csrOffset 0, so these offsets are absolute.
 * ============================================================================
 */

static const struct xcvr_port_group m4061nhp_xcvr_groups[] = {
	{
		.start_port = 1,
		.end_port = 32,
		.reset_reg_offset = 0x0,
		.lp_mode_reg_offset = 0x4,
		.present_reg_offset = 0x8,
	},
	{
		.start_port = 33,
		.end_port = 64,
		.reset_reg_offset = 0x0,
		.lp_mode_reg_offset = 0x4,
		.present_reg_offset = 0x8,
	},
	{
		.start_port = 65,
		.end_port = 65,
		.reset_reg_offset = 0x0088,
		.lp_mode_reg_offset = 0x0088,
		.present_reg_offset = 0x008C,
		.fixed_bits = true,
		.reset_bit = 8,
		.lp_mode_bit = 10,
		.present_bit = 17,
	},
};

static const struct xcvr_ctrl_config m4061nhp_xcvr = {
	.groups = m4061nhp_xcvr_groups,
	.num_groups = ARRAY_SIZE(m4061nhp_xcvr_groups),
};

/* ============================================================================
 * I2C mux
 *
 * Masters 0, 1, 3 and 4 each drive a 1-of-4 mux, select registers at
 * 0x0010 / 0x0014 / 0x0018 / 0x001C (bits 1:0). Master 2 is the TH6 BSC
 * and has no mux. The spec's register text calls 0x0018/0x001C masters
 * 2/3 (M4340NHP wording); its pinout has I2C{0,1,3,4}_BUS_SEL only.
 * ============================================================================
 */

static const struct mux_config m4061nhp_mux[] = {
	{ 0, 4, 0x0010, "MUX0 control" },
	{ 1, 4, 0x0014, "MUX1 control" },
	{ 3, 4, 0x0018, "MUX3 control" },
	{ 4, 4, 0x001C, "MUX4 control" },
};

/* ============================================================================
 * ASIC temperature
 * ============================================================================
 */

static const struct asic_temp_config m4061nhp_asic_temp = {
	.status_reg_offset = 0x0,
	.max_mask = 0x1FFF0000,
	.max_shift = 16,
	.min_mask = 0x00001FFF,
	.min_shift = 0,
};

/* ============================================================================
 * PSU presence
 *
 * PDC Status (the aux device's csrOffset, 0x002C) carries four bits per
 * PSU slot 0-3 — VINOK, PWOK, PRES_L, SMBALERT_L — so PRES_L sits at bit
 * 4*slot + 2. No fw_cfg: the PSU PMBus bus is not reachable through an
 * FPGA I2C master.
 * ============================================================================
 */

static const u32 m4061nhp_psu_masks[] = {
	BIT(2), /* PSU1 */
	BIT(6), /* PSU2 */
	BIT(10), /* PSU3 */
	BIT(14), /* PSU4 */
};

static const struct psu_present_cfg m4061nhp_psu = {
	.present_reg_offset = 0x00,
	.num_psus = ARRAY_SIZE(m4061nhp_psu_masks),
	.active_low = true,
	.present_masks = m4061nhp_psu_masks,
};

/* ============================================================================
 * Platform descriptor
 * ============================================================================
 */

/*
 * Interrupt topology (interrupt-driven i2c-xiic): the first three of the
 * FPGA's six MSI vectors. Vector 0 covers the 5 onboard masters (IER bits
 * 12:8), vectors 1/2 the per-port OSFP masters 5-36 / 37-68. Vectors 3-5
 * (module-present and PSU-presence change) have no consumer and stay
 * masked.
 */
static const struct nh_fpga_msi_domain_cfg m4061nhp_msi_domains[] = {
	{ .chip_name = "m4061-msi-0",
	  .ier_offset = 0x208,
	  .isr_offset = 0x20c,
	  .first_master = 0,
	  .last_master = 4,
	  .first_hw_irq = 8 },
	{ .chip_name = "m4061-msi-1",
	  .ier_offset = 0x210,
	  .isr_offset = 0x214,
	  .first_master = 5,
	  .last_master = 36,
	  .first_hw_irq = 0 },
	{ .chip_name = "m4061-msi-2",
	  .ier_offset = 0x218,
	  .isr_offset = 0x21c,
	  .first_master = 37,
	  .last_master = 68,
	  .first_hw_irq = 0 },
};

static const struct nh_fpga_irq_cfg m4061nhp_irq_cfg = {
	.domains = m4061nhp_msi_domains,
	.num_domains = ARRAY_SIZE(m4061nhp_msi_domains),
	.i2c_csr_base = 0x40000,
	.i2c_csr_channel_size = 0x200,
	.ref_clk_hz = 25000000,
	.min_fpga_version = 0x0401,
};

const struct nh_platform_cfg nh_platform_m4061nhp = {
	.device_id = PCI_DEVICE_ID_NEXTHOP_M4061NHP_SMB,
	.name = "M4061NHP",
	.led_core_cfg = &m4061nhp_led_config,
	.fan_cfg = &m4061nhp_fan,
	.port_led_cfg = m4061nhp_port_led,
	.num_ports_per_led = 65,
	.dual_color_per_led = true,
	.xcvr_cfg = &m4061nhp_xcvr,
	.mux_cfg = m4061nhp_mux,
	.mux_count = ARRAY_SIZE(m4061nhp_mux),
	.asic_temp_cfg = &m4061nhp_asic_temp,
	.psu_present_cfg = &m4061nhp_psu,
	.irq_cfg = &m4061nhp_irq_cfg,
};
