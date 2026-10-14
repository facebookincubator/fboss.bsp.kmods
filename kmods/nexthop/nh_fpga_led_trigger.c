// SPDX-License-Identifier: GPL-2.0+
/*
 * nh_fpga_led_trigger.c - NH FPGA LED Trigger Support
 *
 * LED trigger support for blinking and timer functionality. The trigger
 * implementation is added by a following change; these entry points are
 * the boundary the LED drivers call.
 *
 * Copyright (c) 2026 Nexthop Systems Inc.
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 */

#include <linux/device.h>

#include "nh_fpga_led_trigger.h"

int led_trigger_init(struct device *dev)
{
	return 0;
}

int led_trigger_deinit(struct device *dev)
{
	return 0;
}
