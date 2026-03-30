#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += anacapa_fancpld.o
anacapa_fancpld-objs := anacapa/anacapa_fancpld_i2c.o
obj-m += anacapa_mcbcpld.o
anacapa_mcbcpld-objs := anacapa/anacapa_mcbcpld_i2c.o fboss_iob_led_trigger.o lib/regbit-sysfs.o
obj-m += anacapa_scmcpld.o
anacapa_scmcpld-objs := anacapa/anacapa_scmcpld_i2c.o fboss_iob_led_trigger.o lib/regbit-sysfs.o
