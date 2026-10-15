/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_FPGA_PORT_LED_H_
#define _NH_FPGA_PORT_LED_H_

#include <linux/auxiliary_bus.h>
#include <linux/leds.h>
#include <linux/types.h>

#define PORT_LED_DRIVER_NAME "nh_fpga_port_led"

#define PORT_LED_NUM_COLORS 2

enum port_led_color {
	PORT_LED_BLUE = 0,
	PORT_LED_AMBER = 1,
};

static const char *const color_names[PORT_LED_NUM_COLORS] = {
	[PORT_LED_BLUE] = "blue",
	[PORT_LED_AMBER] = "amber",
};

struct nh_platform_cfg; /* forward declaration — full definition in platform/nh_platform.h */

struct port_led_ctrl;

struct port_led_dev {
	struct led_classdev cdev;
	struct port_led_ctrl *ctrl;
	u32 port_num;
	enum port_led_color color;
	bool trigger_inited; /* led_trigger_init succeeded for this LED */
};

struct port_led_ctrl {
	struct auxiliary_device *aux_dev;
	void __iomem *base;
	struct mutex *reg_lock; /* Parent FPGA's LED register RMW lock */
	struct port_led_dev *leds;
	u32 num_leds;
	const struct nh_platform_cfg
		*platform_cfg; /* Per-device platform descriptor */
};

/*
 * One range of contiguous ports backed by a single 32-bit LED control
 * register. Each port occupies @bits_per_port consecutive bits inside the
 * register, starting at bit `(port_num - start_port) * bits_per_port`.
 * Within those per-port bits, the color whose entry this range belongs to
 * lives at sub-offset @color_bit_offset.
 *
 * Single-bit-per-port designs set `bits_per_port = 1` and
 * `color_bit_offset = 0`, which collapses to the simple math
 * `bit = port_num - start_port`.
 *
 * Multi-bit-per-port designs (e.g., 8 ports × 4 bits per port) set
 * `bits_per_port` to the field width and place each color at a different
 * `color_bit_offset` within each port's field. Both colors then point at
 * the same register offset.
 */
struct port_led_range {
	u32 start_port;
	u32 end_port;
	u32 offset;
	u8 bits_per_port;
	u8 color_bit_offset;
};

struct port_led_color_addr {
	u32 num_ranges;
	const struct port_led_range *ranges;
};

#endif /* _NH_FPGA_PORT_LED_H_ */
