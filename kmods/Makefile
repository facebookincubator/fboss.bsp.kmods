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

# Common TH6 SMB CPLD driver
obj-m += th6_smbcpld.o
th6_smbcpld-objs := th6_smbcpld_i2c.o lib/regbit-sysfs.o

# Backported drivers.
obj-m += backports/

# Platform specific device drivers.
# Please sort platforms in alphabetical order.
obj-m += anacapa/
obj-m += icecube/
obj-m += icetea/
obj-m += minipack3n/
obj-m += montblanc/
obj-m += mtia/
obj-m += tahansb/
obj-m += wedge800/

KDIR ?= /lib/modules/${BUILD_KERNEL}/build

PWD := $(shell pwd)

kmods:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: kmods
	$(MAKE) INSTALL_MOD_DIR=fboss-kmods -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
