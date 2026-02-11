#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += mp3_fancpld.o
mp3_fancpld-objs := mp3_fancpld_i2c.o ../fboss_iob_led_trigger.o
obj-m += mp3_mcbcpld.o
mp3_mcbcpld-objs := mp3_mcbcpld_i2c.o ../lib/regbit-sysfs.o
obj-m += mp3_scmcpld.o
mp3_scmcpld-objs := mp3_scmcpld_i2c.o ../fboss_iob_led_trigger.o ../lib/regbit-sysfs.o
obj-m += mp3_smbcpld.o
mp3_smbcpld-objs := mp3_smbcpld_i2c.o ../lib/regbit-sysfs.o
