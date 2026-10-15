/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_FPGA_LED_CORE_H_
#define _NH_FPGA_LED_CORE_H_

#include <linux/leds.h>
#include <linux/limits.h>

#define NH_LED_MAX_COLORS 8

enum nh_led_type {
	NH_LED_TYPE_PSU = 0,
	NH_LED_TYPE_FAN = 1,
	NH_LED_TYPE_SYS = 2,
	NH_LED_TYPE_MGMT_QSFP = 3,
	NH_LED_TYPE_MAX
};

/* LED information for one LED type on one platform */
struct nh_led_core_info {
	const char *name; /* LED name (psu_led, fan_led, sys_led) */
	u8 color_offset; /* Bit offset in control register for color field */
	u8 color_field_width; /* Number of bits for color field */
	u8 blink_bit; /* Bit position for blink enable */
	u8 colors[NH_LED_MAX_COLORS]; /* Array of color indices for this LED */
};

/* Individual LED device structure */
struct nh_led_core_dev {
	struct led_classdev cdev; /* LED class device */
	int color; /* LED color index */
	struct nh_led_core_controller *ctrl; /* Parent controller */
	bool trigger_inited; /* led_trigger_init succeeded for this LED */
};

struct nh_platform_cfg; /* forward declaration — full definition in platform/nh_platform.h */

/* LED controller structure - one per auxiliary device (LED type) */
struct nh_led_core_controller {
	struct auxiliary_device *aux_dev; /* Auxiliary device */
	void __iomem *base; /* MMIO base address */
	struct mutex *reg_lock; /* Parent FPGA's LED register RMW lock */
	enum nh_led_type led_type; /* LED type (PSU/FAN/SYS) */
	struct nh_led_core_dev *leds; /* Pointer to LED devices array */
	const struct nh_platform_cfg
		*platform_cfg; /* Per-device platform descriptor */
};

/* Per-platform LED configuration (one entry in nh_platform.led_core) */
struct nh_led_core_config {
	const void *
		led_info; /* Pointer to 2D array [LED_TYPE][num_leds] of nh_led_core_info */
	const char *const *color_names; /* Pointer to color name array */
	u8 num_colors_per_led; /* Number of colors per LED */
	u8 num_leds; /* Number of LED devices per LED type */
	u32 control_offset;
};

#endif /* _NH_FPGA_LED_CORE_H_ */
