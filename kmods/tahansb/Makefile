#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += tahansb_fancpld.o
tahansb_fancpld-objs := tahansb/tahansb_fancpld_i2c.o
obj-m += tahansb_scmcpld.o
tahansb_scmcpld-objs := tahansb/tahansb_scmcpld_i2c.o fboss_iob_led_trigger.o lib/regbit-sysfs.o

# TPS16890 driver
obj-m += tps1689.o
tps1689-objs := tahansb/tps1689.o
