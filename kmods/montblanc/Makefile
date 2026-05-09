#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += mp3_fancpld.o
mp3_fancpld-objs := montblanc/mp3_fancpld_i2c.o fboss_iob_led_trigger.o
obj-m += mp3_scmcpld.o
mp3_scmcpld-objs := montblanc/mp3_scmcpld_i2c.o fboss_iob_led_trigger.o lib/regbit-sysfs.o
obj-m += mp3_smbcpld.o
mp3_smbcpld-objs := montblanc/mp3_smbcpld_i2c.o lib/regbit-sysfs.o
