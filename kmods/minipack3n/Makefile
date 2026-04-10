#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += mp3n_fancpld.o
mp3n_fancpld-objs := minipack3n/mp3n_fancpld_i2c.o fboss_iob_led_trigger.o
obj-m += mp3n_scmcpld.o
mp3n_scmcpld-objs := minipack3n/mp3n_scmcpld_i2c.o fboss_iob_led_trigger.o lib/regbit-sysfs.o
