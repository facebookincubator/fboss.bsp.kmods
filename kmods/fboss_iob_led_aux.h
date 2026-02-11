/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __MFD_FBIOB_LED_AUX_H__
#define __MFD_FBIOB_LED_AUX_H__

#include <linux/auxiliary_bus.h>
#include <linux/limits.h>

/*
 * Only 3 colors are exported to user space for qsfp_service.
 * Set the last element to 'green' for setting default color to green.
 */
static const char *const pled_colors[] = {"amber", "blue", "green"};

/*
 * Structure for each LED color
 */
struct port_led_dev {
	// struct port_led_attribute attr[PORT_LED_ATTR_COUNT];
	struct led_classdev cdev;
	char name[NAME_MAX];
	u32 led_on_mask;

	struct mutex *lock;
	void __iomem *mmio_csr;
	struct port_led_priv *priv;
};

/*
 * Structure for the entire LED block (supporting multiple colors).
 */
struct port_led_priv {
	struct auxiliary_device *auxdev;
	u32 csr_bus_addr;
	void __iomem *mmio_csr;
	int port_num;
	int led_idx;
	int num_leds;

	struct mutex lock;
	struct port_led_dev leds[ARRAY_SIZE(pled_colors)];
};

#endif /* __MFD_FBIOB_LED_AUX_H__ */
