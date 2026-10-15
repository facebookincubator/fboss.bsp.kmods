/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_FPGA_FAN_H_
#define _NH_FPGA_FAN_H_

#include <linux/hwmon.h>
#include <linux/auxiliary_bus.h>
#include <linux/leds.h>
#include <linux/types.h>
#include <linux/bits.h>

/* Fan module ID values */
#define FAN_MODULE_ID_ABSENT 0xF
#define FAN_MODULE_ID_DUAL_ROTOR 0x7

/* Fan card type values */
#define FAN_CARD_TYPE_INVALID 0x0
#define FAN_CARD_TYPE_RAPTOR_2RU_12V 0x1

/* Tachometer time units (in nanoseconds for integer arithmetic) */
#define FAN_TACH_TIME_UNIT_NS 3920 /* default tick: 3.92us = 3920ns */

/* PWM scaling constants */
#define FAN_PWM_MIN 0
#define FAN_PWM_MAX 255
#define FAN_PWM_DEFAULT 0x80 /* 50% duty cycle */

/* FBOSS LED control values */
#define FBOSS_LED_GOOD 1 /* Green LED on, Red LED off */
#define FBOSS_LED_FAIL 0 /* Red LED on, Green LED off */
#define FBOSS_LED_OFF 2 /* Both LEDs off */

struct nh_platform_cfg; /* forward declaration — full definition in platform/nh_platform.h */

/* LED device structure for each fan LED */
struct nh_fpga_fan_led {
	struct led_classdev led_cdev;
	struct nh_fpga_fan_controller *fan_ctrl;
	int fan_index; /* 0-based fan index */
	bool is_good_led; /* true for green LED, false for red LED */
};

/* Fan controller structure */
struct nh_fpga_fan_controller {
	struct auxiliary_device *aux_dev;
	struct device *hwmon_dev;
	void __iomem *base;
	int num_fan_trays;
	struct mutex lock; /* Protect register access */
	struct nh_fpga_fan_led *good_leds;
	struct nh_fpga_fan_led *fail_leds;
	u32 *fan_pwm_offsets;
	u32 *fan_inner_tach_offsets;
	u32 *fan_outer_tach_offsets;
	/* fanN_label strings, one per hwmon fan channel */
	const char **fan_labels;
	const struct nh_platform_cfg
		*platform_cfg; /* Per-device platform descriptor */
	const struct attribute_group
		*attr_group; /* Dynamically-built per-fan attrs */
};

/* Fan register structure for easy access */
struct fan_regs {
	u32 pwm_inner_tach; /* PWM control and inner tach status */
	u32 outer_tach; /* Outer tach status */
};

/* Fan status structure */
struct fan_status {
	bool present; /* Fan module present */
	bool power_good; /* Fan power good */
	u8 module_id; /* Fan module ID */
	bool inner_tach_ok; /* Inner tachometer status */
	bool outer_tach_ok; /* Outer tachometer status */
};

struct fan {
	u32 status_offset;
	u32 status_change_flags_offset;
	u32 inner_tach;
	u32 outer_tach;
	u32 pwrgood;
	u32 module_id_mask;
	u32 module_id_shift;
	u32 ctrl_good_led_bit;
	u32 ctrl_fail_led_bit;
	/* PWM duty, inner-tach and outer-tach register offsets. These may be
	 * the same register (read with different masks) or distinct registers,
	 * depending on the FPGA layout.
	 */
	u32 pwm_offset;
	u32 inner_tach_offset;
	u32 outer_tach_offset;
};

struct fan_card_status_addr {
	u32 num_fan_controllers;
	u32 ctrl_offset;
	u32 card_type_mask;
	/* pwm_mask / inner_tach_mask / outer_tach_mask are register-position
	 * masks (the bits as they sit in the FPGA register). The driver
	 * derives the field shift from __ffs(mask), so a field at a non-zero
	 * offset just declares e.g. pwm_mask = 0xFF00 — no separate shift.
	 * read = (reg & mask) >> __ffs(mask),
	 * write = (value << __ffs(mask)) & mask.
	 */
	u32 pwm_mask;
	u32 inner_tach_mask;
	u32 outer_tach_mask;
	/* Tach tick period in ns; 0 selects FAN_TACH_TIME_UNIT_NS. */
	u32 tach_time_unit_ns;

	/* Tachometers per fan tray: 1 (inner only) or 2 (inner + outer). A
	 * tray's rotors occupy consecutive hwmon fan channels, inner first,
	 * so this governs the fan channel count, the channel-to-tray mapping
	 * and the fanN_label text together.
	 */
	u32 num_rotors;

	const struct fan *fans;
};

/* Function prototypes */
int nh_fpga_fan_get_status(struct nh_fpga_fan_controller *controller,
			   int channel, struct fan_status *status);
int nh_fpga_fan_get_card_type(struct nh_fpga_fan_controller *controller);
u32 nh_fpga_fan_tach_to_rpm(u32 tach_cycles, u32 tach_time_unit_ns);

#endif /* _NH_FPGA_FAN_H_ */
