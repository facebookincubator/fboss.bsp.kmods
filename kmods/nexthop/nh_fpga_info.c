// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.

#include <linux/module.h>
#include <linux/auxiliary_bus.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/pci.h>
#include "fbiob-auxdev.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_fpga_info"
#define DRIVER_VERSION "1.0"

/* FPGA version register offsets */
#define FPGA_VERSION_REG_OFFSET 0x0000
#define FPGA_VERSION_MAJOR_MASK 0x00000F00
#define FPGA_VERSION_MAJOR_SHIFT 8
#define FPGA_VERSION_MINOR_MASK 0x000000FF
#define FPGA_VERSION_MINOR_SHIFT 0

/* Private single-register window; an exclusive request is safe. */
#define NH_FPGA_INFO_BLK_SIZE 0x4

/* FPGA info device structure */
struct nh_fpga_info {
	struct auxiliary_device *aux_dev;
	void __iomem *base;
	u32 fpga_ver;
	u32 fpga_sub_ver;
};

/* Sysfs attribute: fw_ver — "major.minor" per the BSP API spec */
static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct nh_fpga_info *info = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u.%u\n", info->fpga_ver, info->fpga_sub_ver);
}
static DEVICE_ATTR_RO(fw_ver);

/* Sysfs attribute: fpga_ver */
static ssize_t fpga_ver_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct nh_fpga_info *info = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", info->fpga_ver);
}
static DEVICE_ATTR_RO(fpga_ver);

/* Sysfs attribute: fpga_sub_ver */
static ssize_t fpga_sub_ver_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct nh_fpga_info *info = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", info->fpga_sub_ver);
}
static DEVICE_ATTR_RO(fpga_sub_ver);

/* Sysfs attributes array */
static struct attribute *fpga_info_attrs[] = {
	&dev_attr_fw_ver.attr,
	&dev_attr_fpga_ver.attr,
	&dev_attr_fpga_sub_ver.attr,
	NULL,
};

/* Sysfs attribute group */
static const struct attribute_group fpga_info_group = {
	.attrs = fpga_info_attrs,
};

/* Read FPGA version information */
static int nh_fpga_info_read_version(struct nh_fpga_info *info)
{
	u32 version_reg;

	/* Read version register */
	version_reg = ioread32(info->base + FPGA_VERSION_REG_OFFSET);

	/* Extract major and minor version */
	info->fpga_ver = (version_reg & FPGA_VERSION_MAJOR_MASK) >>
			 FPGA_VERSION_MAJOR_SHIFT;
	info->fpga_sub_ver = (version_reg & FPGA_VERSION_MINOR_MASK) >>
			     FPGA_VERSION_MINOR_SHIFT;

	dev_info(&info->aux_dev->dev, "FPGA Version: %u.%u\n", info->fpga_ver,
		 info->fpga_sub_ver);

	return 0;
}

/* Auxiliary device probe function */
static int nh_fpga_info_probe(struct auxiliary_device *aux_dev,
			      const struct auxiliary_device_id *id)
{
	struct fbiob_aux_adapter *adap =
		container_of(aux_dev, struct fbiob_aux_adapter, auxdev);
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	struct nh_fpga_info *info;
	int ret;

	if (!nh_get_platform_dev(pdev))
		return -ENODEV;

	/* Allocate info structure */
	info = devm_kzalloc(&aux_dev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->aux_dev = aux_dev;

	/* csr_offset is a bus address; the parent maps nothing. */
	if (!devm_request_mem_region(&aux_dev->dev, adap->data.csr_offset,
				     NH_FPGA_INFO_BLK_SIZE, aux_dev->name))
		return -EBUSY;
	info->base = devm_ioremap(&aux_dev->dev, adap->data.csr_offset,
				  NH_FPGA_INFO_BLK_SIZE);
	if (!info->base)
		return -ENOMEM;

	/* Read FPGA version information */
	ret = nh_fpga_info_read_version(info);
	if (ret) {
		dev_err(&aux_dev->dev, "Failed to read FPGA version: %d\n",
			ret);
		return ret;
	}

	/* Create sysfs attributes */
	ret = devm_device_add_group(&aux_dev->dev, &fpga_info_group);
	if (ret) {
		dev_err(&aux_dev->dev,
			"Failed to create sysfs attributes: %d\n", ret);
		return ret;
	}

	/* Store info in auxiliary device data */
	auxiliary_set_drvdata(aux_dev, info);

	dev_info(&aux_dev->dev, "NH FPGA info driver registered\n");
	return 0;
}

/* Auxiliary device remove function */
static void nh_fpga_info_remove(struct auxiliary_device *aux_dev)
{
	/* Clear auxiliary device data to prevent further access */
	auxiliary_set_drvdata(aux_dev, NULL);

	/* With devm_device_add_groups(), attributes are automatically removed */
	dev_info(&aux_dev->dev, "NH FPGA info driver removed\n");
}

/* Auxiliary device ID table */
static const struct auxiliary_device_id nh_fpga_info_aux_id_table[] = {
	{
		.name = FBOSS_IOB_PCI_DRIVER".nh_fpga_info_iob",
	},
	{
		.name = FBOSS_IOB_PCI_DRIVER".nh_fpga_info_dom",
	},
	{
		.name = FBOSS_IOB_PCI_DRIVER".fpga_info",
	}, /* Generic fallback */
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_fpga_info_aux_id_table);

/* Auxiliary driver structure */
static struct auxiliary_driver nh_fpga_info_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_fpga_info_probe,
	.remove = nh_fpga_info_remove,
	.id_table = nh_fpga_info_aux_id_table,
};

/* Module initialization */
static int __init nh_fpga_info_init_module(void)
{
	int ret;

	pr_info("NH FPGA Info driver v%s loading\n", DRIVER_VERSION);

	ret = auxiliary_driver_register(&nh_fpga_info_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		return ret;
	}

	pr_info("NH FPGA Info driver loaded successfully\n");
	return 0;
}

/* Module cleanup */
static void __exit nh_fpga_info_exit_module(void)
{
	pr_info("NH FPGA Info driver unloading\n");
	auxiliary_driver_unregister(&nh_fpga_info_aux_driver);
	pr_info("NH FPGA Info driver unloaded\n");
}

module_init(nh_fpga_info_init_module);
module_exit(nh_fpga_info_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Arif Mohammad <marif@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA Information Driver");
MODULE_VERSION(DRIVER_VERSION);
