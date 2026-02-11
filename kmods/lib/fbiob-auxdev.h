/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __MFD_FBIOB_AUXDEV_H__
#define __MFD_FBIOB_AUXDEV_H__

#include <linux/auxiliary_bus.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include "fbiob-ioctl.h"

#define FBOSS_IOB_PCI_DRIVER	"fboss_iob_pci"

/*
 * Register block size of the Meta IOB FPGA I/O controllers.
 * The size is used by the I/O controller drivers to request and map
 * MMIO.
 */
#define FBIOB_I2C_BLK_SIZE	0x100
#define FBIOB_GPIO_BLK_SIZE	0x400
#define FBIOB_SPI_CSR_BLK_SIZE	0x80
#define FBIOB_SPI_IOBUF_SIZE	0x400
#define FBIOB_LED_BLK_SIZE	0x4
#define FBIOB_XCVR_BLK_SIZE	0x4
#define FBIOB_MDIO_BLK_SIZE	0x20

/*
 * The structure representing a single instance of IOB FPGA I/O controller.
 */
struct fbiob_aux_adapter {
	struct list_head list;
	struct auxiliary_device auxdev;
	struct fbiob_aux_data data;
};

/*
 * The structure contains all the auxiliary devices belongin to the same
 * parent (FPGA).
 */
struct fbiob_aux_bus {
	struct mutex mutex;
	struct device *parent;
	struct list_head devices;
};

int fbiob_auxdev_remove(struct fbiob_aux_bus *bus, struct fbiob_aux_id *id);
int fbiob_auxdev_add(struct fbiob_aux_bus *bus, struct fbiob_aux_data *data);
void fbiob_auxbus_destroy(struct fbiob_aux_bus *bus);
int fbiob_auxbus_init(struct fbiob_aux_bus *bus, struct device *parent);

#endif /* __MFD_FBIOB_AUXDEV_H__ */
