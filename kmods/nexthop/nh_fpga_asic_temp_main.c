// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) 2026 Nexthop Systems Inc.
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/module.h>
#include <linux/auxiliary_bus.h>
#include <linux/hwmon.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/pci.h>
#include "fbiob-auxdev.h"
#include "nh_fpga_asic_temp.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_fpga_asic_temp"
#define DRIVER_VERSION "1.0"

/* Private status-register window; an exclusive request is safe. */
#define NH_ASIC_TEMP_BLK_SIZE 0x8

/* ASIC Min/Max temperature status register.
 *
 * Single 32-bit register with two 13-bit fields:
 *
 *   bits[28:16]  Max temperature (13-bit unsigned)
 *   bits[12:0]   Min temperature (13-bit unsigned)
 *
 * The encoding from the 13-bit raw value to degC is not handled here;
 * the raw fields are surfaced as-is and any conversion is left to
 * userspace.
 *
 * base points at the device CSR block (the aux device is created with
 * the CSR offset already applied); the status register lives at
 * cfg->status_reg_offset within that block.
 */

struct nh_fpga_asic_temp {
	struct auxiliary_device *aux_dev;
	struct device *hwmon_dev;
	void __iomem *base;
	const struct asic_temp_config *cfg;
};

/* Sysfs attribute: asic_temp_max_raw — bits[28:16] of the status register. */
static ssize_t asic_temp_max_raw_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct nh_fpga_asic_temp *t = dev_get_drvdata(dev);
	u32 reg = ioread32(t->base + t->cfg->status_reg_offset);

	return sysfs_emit(buf, "%u\n",
			  (reg & t->cfg->max_mask) >> t->cfg->max_shift);
}
static DEVICE_ATTR_RO(asic_temp_max_raw);

/* Sysfs attribute: asic_temp_min_raw — bits[12:0] of the status register. */
static ssize_t asic_temp_min_raw_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct nh_fpga_asic_temp *t = dev_get_drvdata(dev);
	u32 reg = ioread32(t->base + t->cfg->status_reg_offset);

	return sysfs_emit(buf, "%u\n",
			  (reg & t->cfg->min_mask) >> t->cfg->min_shift);
}
static DEVICE_ATTR_RO(asic_temp_min_raw);

static struct attribute *asic_temp_attrs[] = {
	&dev_attr_asic_temp_max_raw.attr,
	&dev_attr_asic_temp_min_raw.attr,
	NULL,
};

static const struct attribute_group asic_temp_group = {
	.attrs = asic_temp_attrs,
};

/* Register the attributes via the hwmon framework so they appear
 * under a hwmon[N] subdirectory of the aux device, which is the
 * convention userspace sensor-symlink machinery looks for.
 */
static const struct attribute_group *asic_temp_groups[] = {
	&asic_temp_group,
	NULL,
};

static int nh_fpga_asic_temp_probe(struct auxiliary_device *aux_dev,
				   const struct auxiliary_device_id *id)
{
	struct fbiob_aux_adapter *adap;
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	const struct nh_platform_cfg *platform_cfg;
	struct nh_fpga_asic_temp *t;
	void __iomem *base;
	u16 device_id;

	adap = container_of(aux_dev, struct fbiob_aux_adapter, auxdev);
	device_id = pdev->device;

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg || !platform_cfg->asic_temp_cfg) {
		dev_err(&aux_dev->dev,
			"No ASIC temp config for device ID: 0x%04x\n",
			device_id);
		return -ENODEV;
	}

	t = devm_kzalloc(&aux_dev->dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	/* csr_offset is a bus address; the parent maps nothing. */
	if (!devm_request_mem_region(&aux_dev->dev, adap->data.csr_offset,
				     NH_ASIC_TEMP_BLK_SIZE, aux_dev->name))
		return -EBUSY;
	base = devm_ioremap(&aux_dev->dev, adap->data.csr_offset,
			    NH_ASIC_TEMP_BLK_SIZE);
	if (!base)
		return -ENOMEM;

	t->aux_dev = aux_dev;
	t->base = base;
	t->cfg = platform_cfg->asic_temp_cfg;

	t->hwmon_dev = devm_hwmon_device_register_with_groups(
		&aux_dev->dev, "nh_fpga_asic", t, asic_temp_groups);
	if (IS_ERR(t->hwmon_dev)) {
		dev_err(&aux_dev->dev, "Failed to register hwmon device: %ld\n",
			PTR_ERR(t->hwmon_dev));
		return PTR_ERR(t->hwmon_dev);
	}

	auxiliary_set_drvdata(aux_dev, t);

	dev_info(&aux_dev->dev, "NH FPGA ASIC temperature driver registered\n");
	return 0;
}

static void nh_fpga_asic_temp_remove(struct auxiliary_device *aux_dev)
{
	auxiliary_set_drvdata(aux_dev, NULL);
	dev_info(&aux_dev->dev, "NH FPGA ASIC temperature driver removed\n");
}

static const struct auxiliary_device_id nh_fpga_asic_temp_aux_id_table[] = {
	{
		.name = FBOSS_IOB_PCI_DRIVER ".asic_temp",
	},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_fpga_asic_temp_aux_id_table);

static struct auxiliary_driver nh_fpga_asic_temp_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_fpga_asic_temp_probe,
	.remove = nh_fpga_asic_temp_remove,
	.id_table = nh_fpga_asic_temp_aux_id_table,
};

static int __init nh_fpga_asic_temp_init_module(void)
{
	int ret;

	pr_info("NH FPGA ASIC temp driver v%s loading\n", DRIVER_VERSION);

	ret = auxiliary_driver_register(&nh_fpga_asic_temp_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		return ret;
	}

	pr_info("NH FPGA ASIC temp driver loaded successfully\n");
	return 0;
}

static void __exit nh_fpga_asic_temp_exit_module(void)
{
	pr_info("NH FPGA ASIC temp driver unloading\n");
	auxiliary_driver_unregister(&nh_fpga_asic_temp_aux_driver);
}

module_init(nh_fpga_asic_temp_init_module);
module_exit(nh_fpga_asic_temp_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joy Wu <joy.wu@celestica.com>");
MODULE_AUTHOR("Anna Komarova <anna@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA ASIC Temperature Driver");
MODULE_VERSION(DRIVER_VERSION);
