#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += anacapa_fancpld.o
anacapa_fancpld-objs := anacapa_fancpld_i2c.o
obj-m += anacapa_mcbcpld.o
anacapa_mcbcpld-objs := anacapa_mcbcpld_i2c.o ../fboss_iob_led_trigger.o ../lib/regbit-sysfs.o
obj-m += anacapa_scmcpld.o
anacapa_scmcpld-objs := anacapa_scmcpld_i2c.o ../fboss_iob_led_trigger.o ../lib/regbit-sysfs.o
obj-m += anacapa_rtmcpld.o
anacapa_rtmcpld-objs := anacapa_rtmcpld_i2c.o ../lib/regbit-sysfs.o
