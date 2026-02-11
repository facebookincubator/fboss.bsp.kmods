// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/auxiliary_bus.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>

#include "fbiob-auxdev.h"
#include "regbit-sysfs.h"

#define DRIVER_NAME "fboss_iob_xcvr"

static int fbiob_xcvr_sysfs_init(struct device *dev,
				 void __iomem *mmio_base,
				 u32 port_num)
{
	int i, nleft;
	char suffix[32];
	struct regbit_sysfs_config sysfs_files[] = {
		{
			.name = "xcvr_reset",
			.mode = REGBIT_FMODE_RW,
			.reg_addr = 0,
			.bit_offset = 0,	/* xcvr reset @ bit 0 */
			.num_bits = 1,
		},
		{
			.name = "xcvr_low_power",
			.mode = REGBIT_FMODE_RW,
			.reg_addr = 0,
			.bit_offset = 1,	/* xcvr low_power @ bit 1 */
			.num_bits = 1,
		},
		{
			.name = "xcvr_present",
			.mode = REGBIT_FMODE_RO,
			.reg_addr = 0,
			.bit_offset = 4,	/* xcvr present @ bit 4 */
			.num_bits = 1,
		},
	};

	/* Append port_number to the sysfs filenames. */
	snprintf(suffix, sizeof(suffix), "_%u", port_num);
	for (i = 0; i < ARRAY_SIZE(sysfs_files); i++) {
		nleft = sizeof(sysfs_files[i].name) -
				strlen(sysfs_files[i].name);
		strncat(sysfs_files[i].name, suffix, nleft);
	}

	return regbit_sysfs_init_mmio(dev, mmio_base, sysfs_files,
					ARRAY_SIZE(sysfs_files));
}

static int fbiob_xcvr_probe(struct auxiliary_device *auxdev,
			    const struct auxiliary_device_id *id)
{
	int ret;
	struct resource *res;
	void __iomem *mmio_base;
	struct device *dev = &auxdev->dev;
	struct fbiob_aux_adapter *aux_adap =
		(struct fbiob_aux_adapter *)container_of(auxdev,
				struct fbiob_aux_adapter, auxdev);
	u32 bus_addr = aux_adap->data.csr_offset;
	u32 port_num = aux_adap->data.xcvr_data.port_num;

	if (port_num == 0)
		return -EINVAL;

	res = devm_request_mem_region(dev, bus_addr, FBIOB_XCVR_BLK_SIZE,
					auxdev->name);
	if (!res)
		return -EBUSY;

	mmio_base = devm_ioremap(dev, bus_addr, FBIOB_XCVR_BLK_SIZE);
	if (!mmio_base)
		return -ENOMEM;

	ret = fbiob_xcvr_sysfs_init(dev, mmio_base, port_num);
	if (ret)
		return ret;

	dev_info(dev, "xcvr_ctrl (port=%u, csr_addr=0x%x) registered",
		port_num, bus_addr);
	return 0;
}

static const struct auxiliary_device_id fboss_iob_xcvr_ids[] = {
	{.name = FBOSS_IOB_PCI_DRIVER".xcvr_ctrl"},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_xcvr_ids);

static struct auxiliary_driver fboss_iob_xcvr_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = fbiob_xcvr_probe,
	.id_table = fboss_iob_xcvr_ids,
};
module_auxiliary_driver(fboss_iob_xcvr_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lucas Liu <lucli@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS IOB_DOM XCVR Driver");
MODULE_VERSION(BSP_VERSION);
