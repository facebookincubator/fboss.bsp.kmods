/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __MFD_FBIOB_INT_H__
#define __MFD_FBIOB_INT_H__

#include <linux/pci.h>

#include "fbiob-auxdev.h"
#include "fbiob-cdev.h"

/*
 * IOB and DOM FPGA Global registers.
 */
#define FBIOB_GLOBAL_REG_START	0
#define FBIOB_GLOBAL_REG_SIZE	0x200
#define FBDOM1_GLOBAL_REG_START	0x40000
#define FBDOM2_GLOBAL_REG_START	0x48000
#define FBDOM_GLOBAL_REG_SIZE	0x80

struct fbiob_priv {
	struct pci_dev *pdev;

	/* memory access */
	resource_size_t bar0_bus_addr;
	resource_size_t bar0_size;

	/* Container of all the child auxiliary devices. */
	struct fbiob_aux_bus aux_bus;

	/* Character device for auxdev handling from user space. */
	struct fbiob_cdev_desc cdev_desc;

	/* Cached values */
	u8 device_id;
	u8 board_id;
	u8 board_rev;
	u8 fw_rev_major;
	u8 fw_rev_minor;
};

#endif /* __MFD_FBIOB_INT_H__ */
