#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

subdir-ccflags-y += -I${M}/lib -I${M}/backports
subdir-ccflags-y += -DBSP_VERSION=\"${BSP_VERSION}\"

# FBOSS IOB FPGA controller drivers
obj-m += fboss_iob_gpio.o
obj-m += fboss_iob_i2c.o
obj-m += fboss_iob_info.o
fboss_iob_info-objs := fboss_iob_info_aux.o lib/regbit-sysfs.o
obj-m += fboss_iob_led.o
fboss_iob_led-objs := fboss_iob_led_aux.o fboss_iob_led_trigger.o
obj-m += fboss_iob_mdio.o
fboss_iob_mdio-objs := fboss_iob_mdio_aux.o lib/fboss-mdio.o
obj-m += fboss_iob_pci.o
fboss_iob_pci-objs := fboss_iob_pci_main.o lib/fbiob-auxdev.o lib/fbiob-cdev.o
obj-m += fboss_iob_spi.o
obj-m += fboss_iob_xcvr.o
fboss_iob_xcvr-objs := fboss_iob_xcvr_aux.o lib/regbit-sysfs.o
obj-m += fboss_iob_rtm.o
fboss_iob_rtm-objs := fboss_iob_rtm_aux.o lib/regbit-sysfs.o

# Generic CPLD driver with runtime sysfs support
obj-m += fbcpld_generic.o
fbcpld_generic-objs := fbcpld_generic_i2c.o lib/fbcpld-cdev.o

# Common TH6 SMB CPLD driver
obj-m += th6_smbcpld.o
th6_smbcpld-objs := th6_smbcpld_i2c.o lib/regbit-sysfs.o

# Backported drivers.
obj-m += backports/

KDIR ?= /lib/modules/${BUILD_KERNEL}/build
M ?= $(CURDIR)
PWD := $(shell pwd)
JOBS ?= $(shell nproc)

# Platform specific device drivers.
# Please sort platforms in alphabetical order.
include $(M)/anacapa/Makefile
include $(M)/icecube/Makefile
include $(M)/icetea/Makefile
include $(M)/minipack3n/Makefile
include $(M)/montblanc/Makefile
include $(M)/mtia/Makefile
include $(M)/tahansb/Makefile
include $(M)/wedge800/Makefile

kmods:
	$(MAKE) -j $(JOBS) -C $(KDIR) M=$(PWD) modules

install: kmods
	$(MAKE) INSTALL_MOD_DIR=fboss-kmods -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
