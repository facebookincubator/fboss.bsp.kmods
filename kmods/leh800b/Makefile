#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += leh800b_pdbcpld.o
leh800b_pdbcpld-objs := leh800b/leh800b_pdbcpld_i2c.o lib/regbit-sysfs.o fboss_iob_led_trigger.o
obj-m += leh800b_scmcpld.o
leh800b_scmcpld-objs := leh800b/leh800b_scmcpld_i2c.o lib/regbit-sysfs.o fboss_iob_led_trigger.o