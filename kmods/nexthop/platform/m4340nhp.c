// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/types.h>
#include "nh_platform.h"

/* ============================================================================
 * LED — PSU / FAN / SYS status LEDs
 *
 * The "QSFP Management Card LED Control" register at offset 0x009C packs
 * four 4-bit LED control fields:
 *   bits [3:0]   PSU LED  ([0]=good, [1]=fail, [2]=blink, [3]=rsvd)
 *   bits [7:4]   FAN LED  (same field shape)
 *   bits [11:8]  SYS LED  (same field shape)
 *   bits [19:16] / [23:20] MGMT QSFP 1/2 LEDs — driven as port LEDs 65/66.
 * ============================================================================ */

static const struct nh_led_core_info m4340nhp_led_info[NH_LED_TYPE_MAX][NH_LED_BICOLOR_NUM_COLORS] = {
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

static const struct nh_led_core_config m4340nhp_led_config = {
	.led_info = (const struct nh_led_core_info *)m4340nhp_led_info,
	.color_names = nh_led_rg_color_names,
	.num_colors_per_led = 1,
	.num_leds = 2,
	.control_offset = 0x9c,
};

/* ============================================================================
 * Fan
 *
 * One fan card with five dual-rotor trays. Per-tray PWM (0x0240..0x0250)
 * and tach are separate registers; tach period is [15:0] inner / [31:16]
 * outer. Tach is at 0x00B4..0x00C0 for trays 1-4 and at 0x00C8 for tray 5:
 * 0x00C4 is reserved and reads 0, although the FPGA spec's register table
 * lists tray 5 there. Trays 1-3 report ID/power-good in status register
 * 0x00A4, trays 4-5 in 0x00A8. The status registers carry no tach-ok bits,
 * so inner_tach/outer_tach are 0. Fan Card Control @ 0x00A0 holds a
 * fail/good LED bit pair per tray at bits 2*(tray-1) / 2*(tray-1)+1.
 * ============================================================================ */

static const struct fan m4340nhp_fans[] = {
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

static const struct fan_card_status_addr m4340nhp_fan = {
	.num_fan_controllers = ARRAY_SIZE(m4340nhp_fans),
	.ctrl_offset = 0x00A0,
	.card_type_mask = GENMASK(2, 0),
	.pwm_mask = 0xFF,
	.inner_tach_mask = 0x0000FFFF,
	.outer_tach_mask = 0xFFFF0000,
	/* Fan shift chain sampled at 171.232 kHz. one tick = 5.84us. */
	.tach_time_unit_ns = 5840,
	.num_rotors = 2,
	.fans = m4340nhp_fans,
};

/* ============================================================================
 * Port LED
 *
 * 64 front-panel port LEDs at 0x60..0x7C, 8 ports per 32-bit register
 * with a 4-bit field per port: bit 0 = amber, bit 1 = blue, bits 3:2 =
 * blink mode (blink not exposed by this driver). Both colors share the
 * same register set with color_bit_offset 0 / 1.
 *
 * Mgmt QSFP28 cages (transceivers 65, 66) use the QSFP Management Card
 * LED Control register 0x9C, shared with the psu/fan/sys LEDs: QSFP1
 * [19:16], QSFP2 [23:20] (yellow, green, blink). Exposed as amber/blue
 * like every port.
 * ============================================================================ */

/* { start_port, end_port, offset, bits_per_port, color_bit_offset } */
static const struct port_led_range m4340nhp_amber_ranges[] = {
	{ 1, 8, 0x60, 4 },   { 9, 16, 0x64, 4 },  { 17, 24, 0x68, 4 },
	{ 25, 32, 0x6C, 4 }, { 33, 40, 0x70, 4 }, { 41, 48, 0x74, 4 },
	{ 49, 56, 0x78, 4 }, { 57, 64, 0x7C, 4 }, { 65, 66, 0x9C, 4, 16 },
};

static const struct port_led_range m4340nhp_blue_ranges[] = {
	{ 1, 8, 0x60, 4, 1 },	{ 9, 16, 0x64, 4, 1 },	{ 17, 24, 0x68, 4, 1 },
	{ 25, 32, 0x6C, 4, 1 }, { 33, 40, 0x70, 4, 1 }, { 41, 48, 0x74, 4, 1 },
	{ 49, 56, 0x78, 4, 1 }, { 57, 64, 0x7C, 4, 1 }, { 65, 66, 0x9C, 4, 17 },
};

static const struct port_led_color_addr m4340nhp_port_led[PORT_LED_NUM_COLORS] = {
	[PORT_LED_AMBER] = {
		.num_ranges = ARRAY_SIZE(m4340nhp_amber_ranges),
		.ranges = m4340nhp_amber_ranges,
	},
	[PORT_LED_BLUE] = {
		.num_ranges = ARRAY_SIZE(m4340nhp_blue_ranges),
		.ranges = m4340nhp_blue_ranges,
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
 * Mgmt QSFP28 ports 65/66 are a dedicated block with fixed bits:
 *   QSFP Management Card Control @ 0x0088: QSFP1 rst_l bit 12 / lpmode
 *   bit 14, QSFP2 rst_l bit 8 / lpmode bit 10
 *   QSFP Management Card Status  @ 0x008C: prsnt_l bits 22 / 17
 * Their aux devices use csrOffset 0, so these offsets are absolute.
 * ============================================================================ */

static const struct xcvr_port_group m4340nhp_xcvr_groups[] = {
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
		.reset_bit = 12,
		.lp_mode_bit = 14,
		.present_bit = 22,
	},
	{
		.start_port = 66,
		.end_port = 66,
		.reset_reg_offset = 0x0088,
		.lp_mode_reg_offset = 0x0088,
		.present_reg_offset = 0x008C,
		.fixed_bits = true,
		.reset_bit = 8,
		.lp_mode_bit = 10,
		.present_bit = 17,
	},
};

static const struct xcvr_ctrl_config m4340nhp_xcvr = {
	.groups = m4340nhp_xcvr_groups,
	.num_groups = ARRAY_SIZE(m4340nhp_xcvr_groups),
};

/* ============================================================================
 * I2C mux
 *
 * I2C masters 0-3 each drive a 1-of-4 mux, select register at
 * 0x0010 + 4*N (bits 1:0), per the M4340NHP I2C topology:
 *   0: PDC mux U1 — PSU slots 1-4 on channels 0-3
 *   1: mux U10 — West / Core / fixed-supply DC-DCs, temp sensors (ch 3)
 *   2: MGMT mux U18 — TMP451 + IDPROM, QSFP 1/2, SFP
 *   3: mux U11 — East DC-DCs, clock gens 0-2
 * ============================================================================ */

static const struct mux_config m4340nhp_mux[] = {
	{ 0, 4, 0x0010, "MUX0 control" },
	{ 1, 4, 0x0014, "MUX1 control" },
	{ 2, 4, 0x0018, "MUX2 control" },
	{ 3, 4, 0x001C, "MUX3 control" },
};

/* ============================================================================
 * PSU presence
 *
 * PDC Status (the aux device's csrOffset, 0x002C) carries four bits per
 * PSU slot 0-3 — VINOK, PWOK, PRES_L, SMBALERT_L — so PRES_L sits at bit
 * 4*slot + 2.
 * ============================================================================ */

static const u32 m4340nhp_psu_masks[] = {
	BIT(2), /* PSU1 */
	BIT(6), /* PSU2 */
	BIT(10), /* PSU3 */
	BIT(14), /* PSU4 */
};

/*
 * PSU firmware version. Each PSU hangs off its own channel of the PDC mux
 * on I2C master 0. The PDC straps A0 low and leaves A1 pulled up inside the
 * PSU, so the PMBus address is 0x5A, not the CRPS default 0x58.
 *
 * MFR_FW_REVISION is block-formatted: a leading byte count, then
 * [minor_secondary, minor_primary, major, ...]. Only the first three
 * payload bytes carry the version.
 */
static const u8 m4340nhp_psu_fw_mux_channels[] = {
	0, /* PSU1 */
	1, /* PSU2 */
	2, /* PSU3 */
	3, /* PSU4 */
};

static_assert(ARRAY_SIZE(m4340nhp_psu_fw_mux_channels) ==
		      ARRAY_SIZE(m4340nhp_psu_masks),
	      "one mux channel per PSU slot");

static const struct psu_fw_cfg m4340nhp_psu_fw = {
	.logical_master = 0,
	.mux_channels = m4340nhp_psu_fw_mux_channels,
	.device_addr = 0x5A,
	.version_reg = 0xD9,
	.read_len = 8,
	.payload_offset = 1,
};

static const struct psu_present_cfg m4340nhp_psu = {
	.present_reg_offset = 0x00,
	.num_psus = ARRAY_SIZE(m4340nhp_psu_masks),
	.active_low = true,
	.present_masks = m4340nhp_psu_masks,
	.fw_cfg = &m4340nhp_psu_fw,
};

/* ============================================================================
 * Platform descriptor
 *
 * No asic_temp_cfg: the FPGA has no ASIC temperature register; Q4D
 * temperatures are read from I2C sensors.
 * ============================================================================ */

/*
 * Interrupt topology as documented: the first three of the FPGA's six MSI
 * vectors. Vector 0 covers the 7 onboard masters (IER bits 14:8), vectors
 * 1/2 the per-port OSFP masters 7-38 / 39-70. Vectors 3-5 (module-present
 * and PSU-presence change) have no consumer and stay masked. With i2c_poll
 * set below, no I2C interrupt is requested; this still supplies the CSR
 * layout and reference clock.
 */
static const struct nh_fpga_msi_domain_cfg m4340nhp_msi_domains[] = {
	{ .chip_name = "m4340-msi-0",
	  .ier_offset = 0x208,
	  .isr_offset = 0x20c,
	  .first_master = 0,
	  .last_master = 6,
	  .first_hw_irq = 8 },
	{ .chip_name = "m4340-msi-1",
	  .ier_offset = 0x210,
	  .isr_offset = 0x214,
	  .first_master = 7,
	  .last_master = 38,
	  .first_hw_irq = 0 },
	{ .chip_name = "m4340-msi-2",
	  .ier_offset = 0x218,
	  .isr_offset = 0x21c,
	  .first_master = 39,
	  .last_master = 70,
	  .first_hw_irq = 0 },
};

static const struct nh_fpga_irq_cfg m4340nhp_irq_cfg = {
	.domains = m4340nhp_msi_domains,
	.num_domains = ARRAY_SIZE(m4340nhp_msi_domains),
	.i2c_csr_base = 0x40000,
	.i2c_csr_channel_size = 0x200,
	.ref_clk_hz = 25000000,
	.min_fpga_version = 0x0301,
};

const struct nh_platform_cfg nh_platform_m4340nhp = {
	.device_id = PCI_DEVICE_ID_NEXTHOP_M4340NHP_SMB,
	.name = "M4340NHP",
	.led_core_cfg = &m4340nhp_led_config,
	.fan_cfg = &m4340nhp_fan,
	.port_led_cfg = m4340nhp_port_led,
	.num_ports_per_led = 66,
	.dual_color_per_led = true,
	.xcvr_cfg = &m4340nhp_xcvr,
	.mux_cfg = m4340nhp_mux,
	.mux_count = ARRAY_SIZE(m4340nhp_mux),
	.psu_present_cfg = &m4340nhp_psu,
	.irq_cfg = &m4340nhp_irq_cfg,
	/*
	 * Bitstream 0xf301 builds its 192-bit interrupt-source vector from 193
	 * bits (a 2-bit SPI field in a 1-bit slot), so every I2C master's
	 * interrupt lands one status bit high and the masters' handlers never
	 * see their own completions. Poll until the bitstream is fixed.
	 */
	.i2c_poll = true,
};
