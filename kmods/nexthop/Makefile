# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2025 Nexthop Systems Inc.
#
# Nexthop FPGA drivers, included by the top-level kmods/Makefile. Object paths
# carry the nexthop/ prefix because the build runs with M=kmods.

obj-m += nh_fpga_i2c.o
obj-m += nh_fpga_mux.o
obj-m += nh_fpga_info.o
obj-m += nh_fpga_asic_temp.o
obj-m += nh_fpga_fan.o
obj-m += nh_xcvr_ctrl.o
obj-m += nh_fpga_led.o
obj-m += nh_fpga_port_led.o
obj-m += nh_fpga_psu.o
obj-m += nh_irq.o

# Shared platform objects linked into each module that uses per-platform data.
nh_platform_objs := nexthop/platform/nh_platform.o \
                    nexthop/platform/m4062nhp_common.o \
                    nexthop/platform/m4062nhp_scm.o \
                    nexthop/platform/m4062nhp_fpga0.o \
                    nexthop/platform/m4062nhp_fpga1.o

nh_fpga_i2c-objs := nexthop/nh_fpga_i2c_main.o $(nh_platform_objs)

nh_fpga_mux-objs := nexthop/nh_fpga_mux.o

nh_fpga_info-objs := nexthop/nh_fpga_info.o $(nh_platform_objs)

nh_fpga_asic_temp-objs := nexthop/nh_fpga_asic_temp_main.o $(nh_platform_objs)

nh_fpga_fan-objs := nexthop/nh_fpga_fan_main.o $(nh_platform_objs)

nh_xcvr_ctrl-objs := nexthop/nh_xcvr_ctrl_main.o $(nh_platform_objs)

nh_fpga_led-objs := nexthop/nh_fpga_led_core_main.o \
                    nexthop/nh_fpga_led_trigger.o $(nh_platform_objs)

nh_fpga_port_led-objs := nexthop/nh_fpga_port_led_main.o \
                         nexthop/nh_fpga_led_trigger.o $(nh_platform_objs)

nh_fpga_psu-objs := nexthop/nh_fpga_psu_main.o $(nh_platform_objs)

nh_irq-objs := nexthop/nh_irq.o $(nh_platform_objs)
