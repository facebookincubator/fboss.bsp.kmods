#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += mp3n_fancpld.o
mp3n_fancpld-objs := mp3n_fancpld_i2c.o ../fboss_iob_led_trigger.o
obj-m += mp3n_mcbcpld.o
mp3n_mcbcpld-objs := mp3n_mcbcpld_i2c.o ../lib/regbit-sysfs.o
obj-m += mp3n_scmcpld.o
mp3n_scmcpld-objs := mp3n_scmcpld_i2c.o ../fboss_iob_led_trigger.o ../lib/regbit-sysfs.o
obj-m += mp3n_smbcpld.o
mp3n_smbcpld-objs := mp3n_smbcpld_i2c.o ../lib/regbit-sysfs.o
