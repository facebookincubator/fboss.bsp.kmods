/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * nh_fpga_led_trigger.h - NH FPGA LED Trigger Support
 *
 * LED trigger support for blinking and timer functionality.
 * Adapted from fboss_iob_led_trigger.h
 *
 * Copyright (c) 2026 Nexthop Systems Inc.
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 */

#ifndef _NH_FPGA_LED_TRIGGER_H_
#define _NH_FPGA_LED_TRIGGER_H_

int led_trigger_init(struct device *dev);
int led_trigger_deinit(struct device *dev);

#endif /* _NH_FPGA_LED_TRIGGER_H_ */
