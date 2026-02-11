#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += icetea_scmcpld.o
icetea_scmcpld-objs := icetea_scmcpld_i2c.o ../fboss_iob_led_trigger.o ../lib/regbit-sysfs.o
obj-m += icetea_mcbcpld.o
icetea_mcbcpld-objs := icetea_mcbcpld_i2c.o ../lib/regbit-sysfs.o
obj-m += icetea_fancpld.o
icetea_fancpld-objs := icetea_fancpld_i2c.o
obj-m += icetea_piccpld.o
icetea_piccpld-objs := icetea_piccpld_i2c.o ../lib/regbit-sysfs.o