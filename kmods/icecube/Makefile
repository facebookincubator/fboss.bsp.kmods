#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += icecube_scmcpld.o
icecube_scmcpld-objs := icecube_scmcpld_i2c.o ../fboss_iob_led_trigger.o ../lib/regbit-sysfs.o
obj-m += icecube_mcbcpld.o
icecube_mcbcpld-objs := icecube_mcbcpld_i2c.o ../lib/regbit-sysfs.o
obj-m += icecube_fancpld.o
icecube_fancpld-objs := icecube_fancpld_i2c.o
obj-m += icecube_piccpld.o
icecube_piccpld-objs := icecube_piccpld_i2c.o ../lib/regbit-sysfs.o