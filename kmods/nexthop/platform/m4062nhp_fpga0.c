// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/types.h>
#include "nh_platform.h"
#include "m4062nhp_common.h"

/* ============================================================================
 * LED — PSU / FAN / SYS status LEDs
 *
 * The "QSFP Management Card LED Control" register at offset 0x009C packs
 * four 4-bit LED control fields:
 *   bits [3:0]   PSU LED  ([0]=amber, [1]=blue, [2]=blink, [3]=rsvd)
 *   bits [7:4]   FAN LED  (same field shape)
 *   bits [11:8]  SYS LED  (same field shape)
 *   bits [19:16] MGMT QSFP LED (same field shape)
 * ============================================================================ */

enum m4062nhp_fpga0_led_color {
	M4062NHP_LED_COLOR_FAIL = 0,
	M4062NHP_LED_COLOR_GOOD = 1,
	M4062NHP_LED_NUM_COLORS
};

static const char *const
	m4062nhp_fpga0_led_color_names[M4062NHP_LED_NUM_COLORS] = {
		[M4062NHP_LED_COLOR_FAIL] = "amber",
		[M4062NHP_LED_COLOR_GOOD] = "blue",
	};

static const struct nh_led_core_info
m4062nhp_fpga0_led_info[NH_LED_TYPE_MAX][M4062NHP_LED_NUM_COLORS] = {
	[NH_LED_TYPE_PSU] = {
		[M4062NHP_LED_COLOR_FAIL] = {
			.name = "psu_led",
			.color_offset = 0,
			.color_field_width = 1,
			.blink_bit = 2,
			.colors = {M4062NHP_LED_COLOR_FAIL},
		},
		[M4062NHP_LED_COLOR_GOOD] = {
			.name = "psu_led",
			.color_offset = 1,
			.color_field_width = 1,
			.blink_bit = 2,
			.colors = {M4062NHP_LED_COLOR_GOOD},
		},
	},
	[NH_LED_TYPE_FAN] = {
		[M4062NHP_LED_COLOR_FAIL] = {
			.name = "fan_led",
			.color_offset = 4,
			.color_field_width = 1,
			.blink_bit = 6,
			.colors = {M4062NHP_LED_COLOR_FAIL},
		},
		[M4062NHP_LED_COLOR_GOOD] = {
			.name = "fan_led",
			.color_offset = 5,
			.color_field_width = 1,
			.blink_bit = 6,
			.colors = {M4062NHP_LED_COLOR_GOOD},
		},
	},
	[NH_LED_TYPE_SYS] = {
		[M4062NHP_LED_COLOR_FAIL] = {
			.name = "sys_led",
			.color_offset = 8,
			.color_field_width = 1,
			.blink_bit = 10,
			.colors = {M4062NHP_LED_COLOR_FAIL},
		},
		[M4062NHP_LED_COLOR_GOOD] = {
			.name = "sys_led",
			.color_offset = 9,
			.color_field_width = 1,
			.blink_bit = 10,
			.colors = {M4062NHP_LED_COLOR_GOOD},
		},
	},
	[NH_LED_TYPE_MGMT_QSFP] = {
		/* Both colors share one name so XcvrLib sees one bicolor LED
		 * (port129_led1:{blue,amber}:status). */
		[M4062NHP_LED_COLOR_FAIL] = {
			.name = "port129_led1",
			.color_offset = 16,
			.color_field_width = 1,
			.blink_bit = 18,
			.colors = {M4062NHP_LED_COLOR_FAIL},
		},
		[M4062NHP_LED_COLOR_GOOD] = {
			.name = "port129_led1",
			.color_offset = 17,
			.color_field_width = 1,
			.blink_bit = 18,
			.colors = {M4062NHP_LED_COLOR_GOOD},
		},
	},
};

static const struct nh_led_core_config m4062nhp_fpga0_led_config = {
	.led_info = (const struct nh_led_core_info *)m4062nhp_fpga0_led_info,
	.color_names = m4062nhp_fpga0_led_color_names,
	.num_colors_per_led = 1,
	.num_leds = 2,
	.control_offset = 0x009c,
};

/* ============================================================================
 * I2C mux
 *
 * FPGA0 has a single "I2C Mux Select" register at offset 0x0010 whose
 * bits [1:0] select the channel for i2c master 0 (4 outgoing buses to
 * PSU1..PSU4 EEPROM/MCU pairs). nh_fpga_mux writes the channel index
 * into bits [1:0] of this register, matching the layout.
 * ============================================================================ */

static const struct mux_config m4062nhp_fpga0_mux[] = {
	{ 0, 4, 0x0010, "PSU MUX control" },
};

/* ============================================================================
 * Port LED
 *
 * FPGA0 drives the switch-card front-panel port LEDs for ports 33-96 in
 * eight 32-bit control registers at 0x60..0x7C, packed 8 ports per
 * register with a 4-bit field per port:
 *   bit 0 = amber LED on, bit 1 = blue LED on, bits 3:2 = blink mode.
 * The blink bits are not exposed by the current driver.
 *
 * Fans and OSFP transceiver control signals are owned by FPGA1; see
 * m4062nhp_fpga1.c.
 * ============================================================================ */

/* { start_port, end_port, offset, bits_per_port, color_bit_offset } */
static const struct port_led_range m4062nhp_fpga0_amber_ranges[] = {
	{ 33, 40, 0x60, 4 }, { 41, 48, 0x64, 4 }, { 49, 56, 0x68, 4 },
	{ 57, 64, 0x6C, 4 }, { 65, 72, 0x70, 4 }, { 73, 80, 0x74, 4 },
	{ 81, 88, 0x78, 4 }, { 89, 96, 0x7C, 4 },
};

static const struct port_led_range m4062nhp_fpga0_blue_ranges[] = {
	{ 33, 40, 0x60, 4, 1 }, { 41, 48, 0x64, 4, 1 }, { 49, 56, 0x68, 4, 1 },
	{ 57, 64, 0x6C, 4, 1 }, { 65, 72, 0x70, 4, 1 }, { 73, 80, 0x74, 4, 1 },
	{ 81, 88, 0x78, 4, 1 }, { 89, 96, 0x7C, 4, 1 },
};

static const struct port_led_color_addr m4062nhp_fpga0_port_led[PORT_LED_NUM_COLORS] = {
	[PORT_LED_AMBER] = {
		.num_ranges = ARRAY_SIZE(m4062nhp_fpga0_amber_ranges),
		.ranges = m4062nhp_fpga0_amber_ranges,
	},
	[PORT_LED_BLUE] = {
		.num_ranges = ARRAY_SIZE(m4062nhp_fpga0_blue_ranges),
		.ranges = m4062nhp_fpga0_blue_ranges,
	},
};

/* ============================================================================
 * ASIC temperature — owned by FPGA0
 * ============================================================================ */

static const struct asic_temp_config m4062nhp_fpga0_asic_temp = {
	.status_reg_offset = 0x0,
	.max_mask = 0x1FFF0000,
	.max_shift = 16,
	.min_mask = 0x00001FFF,
	.min_shift = 0,
};

/* ============================================================================
 * PSU presence — FPGA0
 *
 * Register 0x002c contains one active-low presence bit per PSU
 * (0 = present). present_reg_offset is relative to the psu_present
 * aux device's CSR base from fpgaIpBlockConfig.csrOffset in
 * platform_manager.json.
 * ============================================================================ */
static const u32 m4062nhp_fpga0_psu_masks[] = {
	BIT(9), /* PSU1 */
	BIT(13), /* PSU2 */
	BIT(1), /* PSU3 */
	BIT(5), /* PSU4 */
};

/*
 * PSU firmware version. Each PSU hangs off its own channel of the PSU mux.
 *
 * MFR_FW_REVISION is block-formatted: a leading byte count, then
 * [minor_secondary, minor_primary, major]. Read the whole block and skip the
 * count rather than truncating mid-block.
 */
static const u8 m4062nhp_fpga0_psu_fw_mux_channels[] = {
	0, /* PSU1 */
	1, /* PSU2 */
	2, /* PSU3 */
	3, /* PSU4 */
};

static_assert(ARRAY_SIZE(m4062nhp_fpga0_psu_fw_mux_channels) ==
		      ARRAY_SIZE(m4062nhp_fpga0_psu_masks),
	      "one mux channel per PSU slot");

static const struct psu_fw_cfg m4062nhp_fpga0_psu_fw = {
	.logical_master = 0,
	.mux_channels = m4062nhp_fpga0_psu_fw_mux_channels,
	.device_addr = 0x58,
	.version_reg = 0xD9,
	.read_len = 8,
	.payload_offset = 1,
};

static const struct psu_present_cfg m4062nhp_fpga0_psu = {
	.present_reg_offset = 0x00,
	.num_psus = ARRAY_SIZE(m4062nhp_fpga0_psu_masks),
	.active_low = true,
	.present_masks = m4062nhp_fpga0_psu_masks,
	.fw_cfg = &m4062nhp_fpga0_psu_fw,
};

/* ============================================================================
 * Transceiver control — management QSFP (port 129)
 *
 * The mgmt QSFP is a dedicated block on FPGA0, not part of the FPGA1
 * front-panel windows. Its three signals live at fixed bits rather than a
 * shared per-port offset:
 *   QSFP Management Card Control @ 0x0088: bit1 reset_l, bit2 lpmode
 *   QSFP Management Card Status  @ 0x008C: bit4 prsnt_l
 * (reset_l / prsnt_l are active-low; polarity is handled in the BSP
 * platform mapping, not here.)
 * ============================================================================ */

static const struct xcvr_port_group m4062nhp_fpga0_xcvr_groups[] = {
	{
		.start_port = 129,
		.end_port = 129,
		.reset_reg_offset = 0x0088,
		.lp_mode_reg_offset = 0x0088,
		.present_reg_offset = 0x008C,
		.fixed_bits = true,
		.reset_bit = 1,
		.lp_mode_bit = 2,
		.present_bit = 4,
	},
};

static const struct xcvr_ctrl_config m4062nhp_fpga0_xcvr = {
	.groups = m4062nhp_fpga0_xcvr_groups,
	.num_groups = ARRAY_SIZE(m4062nhp_fpga0_xcvr_groups),
};

/* Interrupt topology: 3 MSI vectors, 68 masters */
static const struct nh_fpga_msi_domain_cfg m4062nhp_fpga0_msi_domains[] = {
	{ .chip_name = "m4062-fpga0-msi-0",
	  .ier_offset = 0x170,
	  .isr_offset = 0x174,
	  .first_master = 0,
	  .last_master = 3,
	  .first_hw_irq = 8 },
	{ .chip_name = "m4062-fpga0-msi-1",
	  .ier_offset = 0x178,
	  .isr_offset = 0x17c,
	  .first_master = 4,
	  .last_master = 35,
	  .first_hw_irq = 0 },
	{ .chip_name = "m4062-fpga0-msi-2",
	  .ier_offset = 0x180,
	  .isr_offset = 0x184,
	  .first_master = 36,
	  .last_master = 67,
	  .first_hw_irq = 0 },
};

static const struct nh_fpga_irq_cfg m4062nhp_fpga0_irq_cfg = {
	.domains = m4062nhp_fpga0_msi_domains,
	.num_domains = ARRAY_SIZE(m4062nhp_fpga0_msi_domains),
	.i2c_csr_base = 0x40000,
	.i2c_csr_channel_size = 0x200,
	.ref_clk_hz = 25000000,
	.min_fpga_version = 0x00,
};

const struct nh_platform_cfg nh_platform_m4062nhp_fpga0 = {
	.device_id = PCI_DEVICE_ID_NEXTHOP_M4062NHP_SMB0,
	.name = "M4062NHP_FPGA0",
	.led_core_cfg = &m4062nhp_fpga0_led_config,
	.port_led_cfg = m4062nhp_fpga0_port_led,
	.num_ports_per_led = 128,
	.dual_color_per_led = true,
	.led_to_id = m4062nhp_led_to_id,
	.led_to_id_len = ARRAY_SIZE(m4062nhp_led_to_id),
	.mux_cfg = m4062nhp_fpga0_mux,
	.mux_count = ARRAY_SIZE(m4062nhp_fpga0_mux),
	.asic_temp_cfg = &m4062nhp_fpga0_asic_temp,
	.psu_present_cfg = &m4062nhp_fpga0_psu,
	.xcvr_cfg = &m4062nhp_fpga0_xcvr,
	/* fan_cfg: NULL/0. */
	.irq_cfg = &m4062nhp_fpga0_irq_cfg,
};
