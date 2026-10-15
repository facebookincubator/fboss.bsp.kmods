// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.

#include <linux/kernel.h>
#include "nh_platform.h"

extern const struct nh_platform_cfg nh_platform_m4062nhp_scm;
extern const struct nh_platform_cfg nh_platform_m4062nhp_fpga0;
extern const struct nh_platform_cfg nh_platform_m4062nhp_fpga1;

static const struct nh_platform_cfg *const nh_platforms[] = {
	&nh_platform_m4062nhp_scm,
	&nh_platform_m4062nhp_fpga0,
	&nh_platform_m4062nhp_fpga1,
};

const struct nh_platform_cfg *nh_get_platform(u16 device_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nh_platforms); i++)
		if (nh_platforms[i]->device_id == device_id)
			return nh_platforms[i];
	return NULL;
}

const struct nh_platform_cfg *nh_get_platform_dev(struct pci_dev *pdev)
{
	if (pdev->vendor != PCI_VENDOR_ID_XILINX)
		return NULL;
	return nh_get_platform(pdev->device);
}
