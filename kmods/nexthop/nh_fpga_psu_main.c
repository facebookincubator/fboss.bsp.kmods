// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.

#include <linux/module.h>
#include <linux/auxiliary_bus.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/hwmon.h>
#include "fbiob-auxdev.h"
#include "nh_fpga_i2c_mux.h"
#include "nh_fpga_psu.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_fpga_psu"
#define DRIVER_VERSION "0.1"

/* Bounds the on-stack read buffer; platform read_len is checked against it. */
#define PSU_FW_MAX_READ_LEN 8

/* Version is major.minor_primary.minor_secondary, one byte each. */
#define PSU_FW_VERSION_BYTES 3

struct nh_fpga_psu {
	struct auxiliary_device *aux_dev;
	struct device *hwmon_dev;
	void __iomem *base;
	struct pci_dev *pdev;
	const struct psu_present_cfg *cfg;
	/* Per-PSU attrs are allocated via devm at probe time, sized to
	 * cfg->num_psus.
	 * No static cap array needed.
	 */
	struct attribute_group group;
	const struct attribute_group *groups[2];
};

static int psu_attr_to_index(const char *name)
{
	int psu_num;

	if (sscanf(name, "psu%d", &psu_num) != 1)
		return -EINVAL;
	return psu_num - 1;
}

static bool nh_fpga_psu_is_present(struct nh_fpga_psu *p, int idx)
{
	u32 reg = ioread32(p->base + p->cfg->present_reg_offset);
	bool present = !!(reg & p->cfg->present_masks[idx]);

	return p->cfg->active_low ? !present : present;
}

/*
 * The PSU lives on a mux child adapter registered by nh_fpga_mux, which handles
 * the channel select and bus locking.
 */
static int nh_fpga_psu_read_fw_raw(struct nh_fpga_psu *p, int idx, u8 *raw)
{
	const struct psu_fw_cfg *fw = p->cfg->fw_cfg;
	u8 cmd = fw->version_reg;
	struct i2c_msg msgs[] = {
		{
			.addr = fw->device_addr,
			.flags = 0,
			.len = 1,
			.buf = &cmd,
		},
		{
			.addr = fw->device_addr,
			.flags = I2C_M_RD,
			.len = fw->read_len,
			.buf = raw,
		},
	};
	int ret;

	ret = nh_fpga_mux_channel_xfer(p->pdev, fw->logical_master,
				       fw->mux_channels[idx], msgs,
				       ARRAY_SIZE(msgs));

	return ret == ARRAY_SIZE(msgs) ? 0 : (ret < 0 ? ret : -EIO);
}

static ssize_t psu_present_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nh_fpga_psu *p = dev_get_drvdata(dev);
	int idx;

	idx = psu_attr_to_index(attr->attr.name);
	if (idx < 0 || idx >= p->cfg->num_psus)
		return -EINVAL;

	/* 1 for present, 0 for absent (FBOSS convention) */
	return sysfs_emit(buf, "%d\n", nh_fpga_psu_is_present(p, idx) ? 1 : 0);
}

static ssize_t psu_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct nh_fpga_psu *p = dev_get_drvdata(dev);
	u8 raw[PSU_FW_MAX_READ_LEN] = { 0 };
	const u8 *ver;
	int idx, ret;

	idx = psu_attr_to_index(attr->attr.name);
	if (idx < 0 || idx >= p->cfg->num_psus)
		return -EINVAL;

	/*
	 * An empty slot is normal, and fw_util turns any failed version read
	 * into a failure of the whole run, so report a sentinel rather than
	 * an errno.
	 */
	if (!nh_fpga_psu_is_present(p, idx))
		return sysfs_emit(buf, "n/a\n");

	ret = nh_fpga_psu_read_fw_raw(p, idx, raw);
	if (ret)
		return ret;

	ver = raw + p->cfg->fw_cfg->payload_offset;
	return sysfs_emit(buf, "%u.%u.%u\n", ver[2] & 0x7f, ver[1], ver[0]);
}

static int nh_fpga_psu_check_fw_cfg(struct device *dev,
				    const struct psu_fw_cfg *fw)
{
	if (!fw->mux_channels) {
		dev_err(dev, "PSU FW config has no mux channel map\n");
		return -EINVAL;
	}

	if (fw->read_len > PSU_FW_MAX_READ_LEN ||
	    fw->read_len < fw->payload_offset + PSU_FW_VERSION_BYTES) {
		dev_err(dev, "PSU FW config read_len %u out of range\n",
			fw->read_len);
		return -EINVAL;
	}

	return 0;
}

static int nh_fpga_psu_probe(struct auxiliary_device *aux_dev,
			     const struct auxiliary_device_id *id)
{
	struct fbiob_aux_adapter *adap =
		container_of(aux_dev, struct fbiob_aux_adapter, auxdev);
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	const struct nh_platform_cfg *platform_cfg;
	struct nh_fpga_psu *p;
	u16 device_id;
	u32 blk;
	int i, ret;

	device_id = pdev->device;

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg || !platform_cfg->psu_present_cfg) {
		dev_err(&aux_dev->dev,
			"No PSU present config for device ID: 0x%04x\n",
			device_id);
		return -ENODEV;
	}

	if (platform_cfg->psu_present_cfg->present_reg_offset ==
	    NH_FPGA_PSU_OFFSET_UNSET) {
		dev_warn(
			&aux_dev->dev,
			"PSU-present register not yet characterized for 0x%04x; driver idle\n",
			device_id);
		return -ENODEV;
	}

	p = devm_kzalloc(&aux_dev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->aux_dev = aux_dev;
	p->pdev = pdev;
	p->cfg = platform_cfg->psu_present_cfg;

	/* PSU-present register; csr_offset is a bus address, the parent maps
	 * nothing. */
	blk = p->cfg->present_reg_offset + sizeof(u32);
	if (!devm_request_mem_region(&aux_dev->dev, adap->data.csr_offset, blk,
				     aux_dev->name))
		return -EBUSY;
	p->base = devm_ioremap(&aux_dev->dev, adap->data.csr_offset, blk);
	if (!p->base)
		return -ENOMEM;

	if (p->cfg->num_psus == 0) {
		dev_err(&aux_dev->dev, "num_psus is zero in platform config\n");
		return -EINVAL;
	}

	if (p->cfg->fw_cfg) {
		ret = nh_fpga_psu_check_fw_cfg(&aux_dev->dev, p->cfg->fw_cfg);
		if (ret)
			return ret;
	}

	/* Build per-PSU attrs dynamically, no static cap array needed.
	 * All allocations are devm. */
	{
		struct device_attribute *attrs;
		struct attribute **attr_ptrs;
		int attrs_per_psu = p->cfg->fw_cfg ? 2 : 1;
		int num_attrs = p->cfg->num_psus * attrs_per_psu;
		int attr_idx = 0;

		attrs = devm_kcalloc(&aux_dev->dev, num_attrs, sizeof(*attrs),
				     GFP_KERNEL);
		attr_ptrs = devm_kcalloc(&aux_dev->dev, num_attrs + 1,
					 sizeof(*attr_ptrs), GFP_KERNEL);
		if (!attrs || !attr_ptrs)
			return -ENOMEM;

		for (i = 0; i < p->cfg->num_psus; i++) {
			attrs[attr_idx].attr.name =
				devm_kasprintf(&aux_dev->dev, GFP_KERNEL,
					       "psu%d_present", i + 1);
			if (!attrs[attr_idx].attr.name)
				return -ENOMEM;
			sysfs_attr_init(&attrs[attr_idx].attr);
			attrs[attr_idx].attr.mode = 0444;
			attrs[attr_idx].show = psu_present_show;
			attr_ptrs[attr_idx] = &attrs[attr_idx].attr;
			attr_idx++;

			if (!p->cfg->fw_cfg)
				continue;

			attrs[attr_idx].attr.name =
				devm_kasprintf(&aux_dev->dev, GFP_KERNEL,
					       "psu%d_fw_version", i + 1);
			if (!attrs[attr_idx].attr.name)
				return -ENOMEM;
			sysfs_attr_init(&attrs[attr_idx].attr);
			attrs[attr_idx].attr.mode = 0444;
			attrs[attr_idx].show = psu_fw_version_show;
			attr_ptrs[attr_idx] = &attrs[attr_idx].attr;
			attr_idx++;
		}
		attr_ptrs[attr_idx] = NULL;
		p->group.attrs = attr_ptrs;
	}
	p->groups[0] = &p->group;
	p->groups[1] = NULL;

	auxiliary_set_drvdata(aux_dev, p);

	/* Register via hwmon so the psu<N>_present attrs land under a hwmon[N]
	 * subdir, which is where platform_manager's /run/devmap/sensors
	 * resolver (resolveSensorPath) looks.
	 */
	p->hwmon_dev = devm_hwmon_device_register_with_groups(
		&aux_dev->dev, "nh_fpga_psu", p, p->groups);
	if (IS_ERR(p->hwmon_dev)) {
		dev_err(&aux_dev->dev, "Failed to register hwmon device: %ld\n",
			PTR_ERR(p->hwmon_dev));
		return PTR_ERR(p->hwmon_dev);
	}

	dev_info(&aux_dev->dev,
		 "NH FPGA PSU driver registered (%u PSUs, fw version %s)\n",
		 p->cfg->num_psus, p->cfg->fw_cfg ? "enabled" : "disabled");
	return 0;
}

static void nh_fpga_psu_remove(struct auxiliary_device *aux_dev)
{
	auxiliary_set_drvdata(aux_dev, NULL);
	dev_info(&aux_dev->dev, "NH FPGA PSU presence driver removed\n");
}

static const struct auxiliary_device_id nh_fpga_psu_aux_id_table[] = {
	{
		.name = FBOSS_IOB_PCI_DRIVER".psu_present",
	},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_fpga_psu_aux_id_table);

static struct auxiliary_driver nh_fpga_psu_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_fpga_psu_probe,
	.remove = nh_fpga_psu_remove,
	.id_table = nh_fpga_psu_aux_id_table,
};

static int __init nh_fpga_psu_init_module(void)
{
	int ret;

	pr_info("NH FPGA PSU presence driver v%s loading\n", DRIVER_VERSION);

	ret = auxiliary_driver_register(&nh_fpga_psu_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		return ret;
	}

	pr_info("NH FPGA PSU presence driver loaded successfully\n");
	return 0;
}

static void __exit nh_fpga_psu_exit_module(void)
{
	pr_info("NH FPGA PSU presence driver unloading\n");
	auxiliary_driver_unregister(&nh_fpga_psu_aux_driver);
}

module_init(nh_fpga_psu_init_module);
module_exit(nh_fpga_psu_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Maisey Perelonia <maisey@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA PSU Presence Driver");
MODULE_VERSION(DRIVER_VERSION);
