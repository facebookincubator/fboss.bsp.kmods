#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += w800_fancpld.o
w800_fancpld-objs := wedge800/w800_fancpld_i2c.o fboss_iob_led_trigger.o
obj-m += w800_scmcpld.o
w800_scmcpld-objs := wedge800/w800_scmcpld_i2c.o fboss_iob_led_trigger.o lib/regbit-sysfs.o
obj-m += w800b_smbcpld.o
w800b_smbcpld-objs := wedge800/w800b_smbcpld_i2c.o lib/regbit-sysfs.o
obj-m += w800c_smbcpld.o
w800c_smbcpld-objs := wedge800/w800c_smbcpld_i2c.o lib/regbit-sysfs.o
