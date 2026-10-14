/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2025 Nexthop Systems Inc. */

#ifndef _NH_PLATFORM_H_
#define _NH_PLATFORM_H_

#include <linux/types.h>
#include <linux/pci.h>

#include "fboss-pci-ids.h"

/* Pull in subsystem struct definitions. These headers contain only
 * type/enum/macro definitions — no static data. */
#include "../nh_fpga_led_core.h"
#include "../nh_fpga_fan.h"
#include "../nh_fpga_port_led.h"
#include "../nh_xcvr_ctrl.h"
#include "../nh_fpga_i2c_masters.h"
#include "../nh_fpga_asic_temp.h"
#include "../nh_fpga_psu.h"
#include "../nh_fpga_irq.h"

/* Common LED color indices for standard bicolor (red/green) system LEDs */
enum nh_led_bicolor {
	NH_LED_BICOLOR_FAIL = 0, /* Red - failure/fault state */
	NH_LED_BICOLOR_GOOD = 1, /* Green - good/operational state */
	NH_LED_BICOLOR_NUM_COLORS
};

/* Shared color names for red/green bicolor LEDs */
static const char *const nh_led_rg_color_names[NH_LED_BICOLOR_NUM_COLORS] = {
	[NH_LED_BICOLOR_FAIL] = "red",
	[NH_LED_BICOLOR_GOOD] = "green",
};

/**
 * struct nh_platform_cfg - All per-SKU data aggregated in one place.
 *
 * Fields are NULL/0 for subsystems not present on a given platform.
 * Drivers must check for NULL before dereferencing subsystem pointers.
 */
struct nh_platform_cfg {
	u16 device_id;
	const char *name;

	/* LED subsystem (PSU/FAN/SYS LEDs) */
	const struct nh_led_core_config
		*led_core_cfg; /* Core LED driver (all types) */

	/* Fan/PWM subsystem */
	const struct fan_card_status_addr *fan_cfg;

	/* Port LED subsystem */
	const struct port_led_color_addr
		*port_led_cfg; /* array[PORT_LED_NUM_COLORS] */

	u32 num_ports_per_led;

	/* When true, one aux device registers both color channels of a port's
	 * LED (named port<id>_led1). When false, led_idx selects a single color
	 * (1 = blue, 2 = amber). */
	bool dual_color_per_led;

	/* Optional physical-LED-number -> transceiver id remap for the port LED
	 * sysfs name (port<id>_led). NULL names the node by port_num directly. */
	const u32 *led_to_id;
	u32 led_to_id_len;

	/* Transceiver control subsystem */
	const struct xcvr_ctrl_config *xcvr_cfg;

	/* I2C mux subsystem */
	const struct mux_config *mux_cfg;
	int mux_count;

	/* ASIC temperature subsystem */
	const struct asic_temp_config *asic_temp_cfg;

	/* PSU presence subsystem */
	const struct psu_present_cfg *psu_present_cfg;
	/* Interrupt topology (i2c-xiic); NULL if unsupported. */
	const struct nh_fpga_irq_cfg *irq_cfg;

	/*
	 * Run the I2C masters on the polled nh-xiic-i2c driver instead of the
	 * interrupt-driven xiic-i2c. irq_cfg still supplies the CSR layout and
	 * reference clock; the I2C interrupt lines stay masked.
	 */
	bool i2c_poll;
};

/**
 * nh_get_platform - Look up the platform descriptor for a PCI device ID.
 * @device_id: PCI device ID from pdev->device
 *
 * Returns a pointer to the read-only platform descriptor, or NULL if the
 * device ID is not recognised. Callers must check for NULL.
 *
 * This function is compiled into each module that uses it (not exported
 * via EXPORT_SYMBOL).
 */
const struct nh_platform_cfg *nh_get_platform(u16 device_id);

/* As above, for a PCI device; NULL unless it is a Nexthop FPGA. */
const struct nh_platform_cfg *nh_get_platform_dev(struct pci_dev *pdev);

/* fbiob_aux_data.csr_offset is a bus address; make it BAR-relative. */
static inline u32 nh_csr_rel(struct pci_dev *pdev, u32 csr_bus_addr)
{
	return csr_bus_addr - (u32)pci_resource_start(pdev, 0);
}

#endif /* _NH_PLATFORM_H_ */
