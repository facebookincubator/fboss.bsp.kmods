/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __FBIOB_LED_TRIGGER_H__
#define __FBIOB_LED_TRIGGER_H__

int led_trigger_init(struct device *dev);
int led_trigger_deinit(struct device *dev);

#endif /* __FBIOB_LED_TRIGGER_H__ */
