/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __MFD_FBIOB_CDEV_H__
#define __MFD_FBIOB_CDEV_H__

#include <linux/miscdevice.h>
#include <linux/pci.h>

#include "fbiob-auxdev.h"

struct fbiob_cdev_desc {
	struct miscdevice miscdev;
	char name[NAME_MAX];

	/*
	 * Store the parent's data to add/remove auxdev easily.
	 */
	struct fbiob_aux_bus *aux_bus;
	resource_size_t base_bus_addr;
	resource_size_t max_offset;
};

int fbiob_cdev_init(struct fbiob_cdev_desc *cdesc,
		    struct pci_dev *pcidev,
		    struct fbiob_aux_bus *aux_bus,
		    resource_size_t base_bus_addr,
		    resource_size_t max_offset);
void fbiob_cdev_destroy(struct fbiob_cdev_desc *cdesc);

#endif /* __MFD_FBIOB_CDEV_H__ */
