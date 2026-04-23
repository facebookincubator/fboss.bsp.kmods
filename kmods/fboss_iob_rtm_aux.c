// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/auxiliary_bus.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>

#include "fbiob-auxdev.h"
#include "regbit-sysfs.h"

#define DRIVER_NAME "fboss_iob_rtm"

static const struct regbit_sysfs_config fbiob_rtm_sysfs_template[] = {
	{
		.name = "rtm_reset",
		.mode = REGBIT_FMODE_RW,
		.reg_addr = 0,
		.bit_offset = 0,	/* rtm reset @ bit 0 */
		.num_bits = 1,
	},
};

static int fbiob_rtm_sysfs_init(struct device *dev,
				 void __iomem *mmio_base,
				 u32 port_num)
{
	int i;
	struct regbit_sysfs_config *sysfs_files;

	sysfs_files = devm_kmemdup(dev, fbiob_rtm_sysfs_template,
				   sizeof(fbiob_rtm_sysfs_template), GFP_KERNEL);
	if (!sysfs_files)
		return -ENOMEM;

	/* Append port_number to the sysfs filenames. */
	for (i = 0; i < ARRAY_SIZE(fbiob_rtm_sysfs_template); i++)
		snprintf(sysfs_files[i].name, sizeof(sysfs_files[i].name),
			 "%s_%u", fbiob_rtm_sysfs_template[i].name, port_num);

	return regbit_sysfs_init_mmio(dev, mmio_base, sysfs_files,
					ARRAY_SIZE(fbiob_rtm_sysfs_template));
}

static int fbiob_rtm_probe(struct auxiliary_device *auxdev,
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
	u32 port_num = aux_adap->data.rtm_data.port_num;

	if (port_num == 0)
		return -EINVAL;

	res = devm_request_mem_region(dev, bus_addr, FBIOB_RTM_BLK_SIZE,
					auxdev->name);
	if (!res)
		return -EBUSY;

	mmio_base = devm_ioremap(dev, bus_addr, FBIOB_RTM_BLK_SIZE);
	if (!mmio_base)
		return -ENOMEM;

	ret = fbiob_rtm_sysfs_init(dev, mmio_base, port_num);
	if (ret)
		return ret;

	dev_info(dev, "rtm_ctrl (port=%u, csr_addr=0x%x) registered",
		port_num, bus_addr);
	return 0;
}

static const struct auxiliary_device_id fboss_iob_rtm_ids[] = {
	{.name = FBOSS_IOB_PCI_DRIVER".rtm_ctrl"},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_rtm_ids);

static struct auxiliary_driver fboss_iob_rtm_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = fbiob_rtm_probe,
	.id_table = fboss_iob_rtm_ids,
};
module_auxiliary_driver(fboss_iob_rtm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Evan Zong <ezong@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS IOB_DOM Retimer Management Driver");
MODULE_VERSION(BSP_VERSION);
