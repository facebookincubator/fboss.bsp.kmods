#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += tahansb_fancpld.o
tahansb_fancpld-objs := tahansb_fancpld_i2c.o
obj-m += tahansb_mcbcpld.o
tahansb_mcbcpld-objs := tahansb_mcbcpld_i2c.o ../lib/regbit-sysfs.o
obj-m += tahansb_scmcpld.o
tahansb_scmcpld-objs := tahansb_scmcpld_i2c.o ../fboss_iob_led_trigger.o ../lib/regbit-sysfs.o

# TPS16890 driver
obj-m += tps1689.o
