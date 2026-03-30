#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += janga_smbcpld.o
janga_smbcpld-objs := mtia/janga_smbcpld_i2c.o lib/regbit-sysfs.o
obj-m += mtia_fan_watchdog.o
mtia_fan_watchdog-objs := mtia/mtia_fan_watchdog.o
obj-m += mtia_smbcpld.o
mtia_smbcpld-objs := mtia/mtia_smbcpld_i2c.o lib/regbit-sysfs.o fboss_iob_led_trigger.o
obj-m += tahan_smbcpld.o
tahan_smbcpld-objs := mtia/tahan_smbcpld_i2c.o lib/regbit-sysfs.o
