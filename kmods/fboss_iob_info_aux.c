// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/regmap.h>

#include "fbiob-auxdev.h"
#include "regbit-sysfs.h"

#define FPGA_INFO_MEM_SIZE	4

#define FPGA_VER_INFO_REG	0
#define FPGA_SUBVER_BIT_OFFSET	8
#define FPGA_VER_BIT_OFFSET	16
#define	FPGA_VER_BIT_MASK	0xFF

static ssize_t fpga_fw_ver_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ret;
	u32 val;
	struct regmap *regmap;
	u8 fpga_ver, fpga_sub_ver;

	regmap = dev_get_regmap(dev, NULL);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = regmap_read(regmap, FPGA_VER_INFO_REG, &val);
	if (ret)
		return ret;

	fpga_ver = (u8)((val >> FPGA_VER_BIT_OFFSET) & FPGA_VER_BIT_MASK);
	fpga_sub_ver = (u8)((val >> FPGA_SUBVER_BIT_OFFSET) &
			    FPGA_VER_BIT_MASK);
	return sprintf(buf, "%u.%u\n", fpga_ver, fpga_sub_ver);
}

struct regbit_sysfs_config sysfs_files[] = {
	{
		.name = "board_rev",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FPGA_VER_INFO_REG,
		.bit_offset = 0,
		.num_bits = 4,
	},
	{
		.name = "board_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FPGA_VER_INFO_REG,
		.bit_offset = 4,
		.num_bits = 4,
	},
	{
		.name = "fw_ver",
		.mode = REGBIT_FMODE_RO,
		.show_func = fpga_fw_ver_show,
	},
	{
		.name = "fpga_sub_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FPGA_VER_INFO_REG,
		.bit_offset = FPGA_SUBVER_BIT_OFFSET,
		.num_bits = 8,
	},
	{
		.name = "fpga_ver",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FPGA_VER_INFO_REG,
		.bit_offset = FPGA_VER_BIT_OFFSET,
		.num_bits = 8,
	},
	{
		.name = "device_id",
		.mode = REGBIT_FMODE_RO,
		.reg_addr = FPGA_VER_INFO_REG,
		.bit_offset = 24,
		.num_bits = 8,
	},
};

static int fpga_info_probe(struct auxiliary_device *auxdev,
			   const struct auxiliary_device_id *id)
{
	struct resource *res;
	void __iomem *mmio_base;
	struct device *dev = &auxdev->dev;
	struct fbiob_aux_adapter *aux_adap =
		(struct fbiob_aux_adapter *)container_of(auxdev,
				struct fbiob_aux_adapter, auxdev);
	u32 bus_addr = aux_adap->data.csr_offset;

	res = devm_request_mem_region(dev, bus_addr, FPGA_INFO_MEM_SIZE,
					auxdev->name);
	if (!res)
		return -EBUSY;

	mmio_base = devm_ioremap(dev, bus_addr, FPGA_INFO_MEM_SIZE);
	if (!mmio_base)
		return -ENOMEM;

	return regbit_sysfs_init_mmio(dev, mmio_base, sysfs_files,
				      ARRAY_SIZE(sysfs_files));
}

static const struct auxiliary_device_id fboss_iob_info_ids[] = {
	{.name = FBOSS_IOB_PCI_DRIVER".fpga_info_iob"},
	{.name = FBOSS_IOB_PCI_DRIVER".fpga_info_dom"},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_info_ids);

static struct auxiliary_driver fboss_iob_info_driver = {
	.driver = {
		.name = "fboss_iob_info",
	},
	.probe = fpga_info_probe,
	.id_table = fboss_iob_info_ids,
};
module_auxiliary_driver(fboss_iob_info_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tao Ren <taoren@meta.com>");
MODULE_DESCRIPTION("Meta FBOSS FPGA Info Driver");
MODULE_VERSION(BSP_VERSION);
