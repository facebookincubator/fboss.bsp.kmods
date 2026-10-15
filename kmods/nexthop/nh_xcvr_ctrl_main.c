// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.

#include <linux/module.h>
#include <linux/auxiliary_bus.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include "fbiob-auxdev.h"
#include "nh_xcvr_ctrl.h"
#include "platform/nh_platform.h"
#include "nh_irq.h"

#define DRIVER_NAME "nh_xcvr_ctrl"
#define DRIVER_VERSION "1.0"

/* Multiple ports share each control register; one lock serializes RMW
 * across all xcvr port devices. */
static DEFINE_MUTEX(xcvr_reg_lock);

/* Look up the port group covering @slot. */
const struct xcvr_port_group *
xcvr_ctrl_find_port_group(const struct xcvr_ctrl_config *cfg, u32 slot)
{
	u32 i;

	if (!cfg)
		return NULL;

	for (i = 0; i < cfg->num_groups; i++) {
		const struct xcvr_port_group *g = &cfg->groups[i];

		if (slot >= g->start_port && slot <= g->end_port)
			return g;
	}
	return NULL;
}

/* Resolve a transceiver id to its physical slot; identity if the platform
 * has no id_to_slot table. */
u32 xcvr_ctrl_slot_for_port(const struct xcvr_ctrl_config *cfg, u32 port_num)
{
	if (!cfg || !cfg->id_to_slot || port_num >= cfg->id_to_slot_len)
		return port_num;
	return cfg->id_to_slot[port_num];
}

/* Read transceiver control register */
static int nh_xcvr_ctrl_read_reg(struct nh_xcvr_ctrl *ctrl, u32 offset, u32 bit)
{
	u32 reg_value;
	u32 bit_mask;

	/* Read register */
	reg_value = ioread32(ctrl->base + offset);

	/* Extract bit for this port/signal */
	bit_mask = 1U << bit;

	return (reg_value & bit_mask) ? 1 : 0;
}

/* Write transceiver control register */
static void nh_xcvr_ctrl_write_reg(struct nh_xcvr_ctrl *ctrl, u32 offset,
				   u32 bit, int value)
{
	u32 reg_value;
	u32 bit_mask;

	mutex_lock(&xcvr_reg_lock);

	/* Read current register value */
	reg_value = ioread32(ctrl->base + offset);

	/* Modify bit for this port/signal */
	bit_mask = 1U << bit;

	if (value)
		reg_value |= bit_mask;
	else
		reg_value &= ~bit_mask;

	/* Write back */
	iowrite32(reg_value, ctrl->base + offset);

	mutex_unlock(&xcvr_reg_lock);
}

/* Sysfs attribute: xcvr_reset_%d */
static ssize_t xcvr_reset_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct nh_xcvr_ctrl *ctrl = dev_get_drvdata(dev);
	int val = nh_xcvr_ctrl_read_reg(ctrl, ctrl->group->reset_reg_offset,
					ctrl->reset_bit);

	if (!ctrl->group->reset_active_high)
		val = !val;

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t xcvr_reset_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct nh_xcvr_ctrl *ctrl = dev_get_drvdata(dev);
	int value, ret;

	ret = kstrtoint(buf, 10, &value);
	if (ret)
		return ret;

	value = !!value;
	if (!ctrl->group->reset_active_high)
		value = !value;

	nh_xcvr_ctrl_write_reg(ctrl, ctrl->group->reset_reg_offset,
			       ctrl->reset_bit, value);
	return count;
}

/* Sysfs attribute: xcvr_low_power_%d */
static ssize_t xcvr_low_power_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct nh_xcvr_ctrl *ctrl = dev_get_drvdata(dev);

	if (ctrl->group->no_lp_mode)
		return sysfs_emit(buf, "%u\n", ctrl->lp_mode_shadow);

	return sysfs_emit(buf, "%u\n",
			  nh_xcvr_ctrl_read_reg(ctrl,
						ctrl->group->lp_mode_reg_offset,
						ctrl->lp_mode_bit));
}

static ssize_t xcvr_low_power_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nh_xcvr_ctrl *ctrl = dev_get_drvdata(dev);
	int value, ret;

	ret = kstrtoint(buf, 10, &value);
	if (ret)
		return ret;

	if (ctrl->group->no_lp_mode) {
		ctrl->lp_mode_shadow = !!value;
		return count;
	}

	nh_xcvr_ctrl_write_reg(ctrl, ctrl->group->lp_mode_reg_offset,
			       ctrl->lp_mode_bit, value);
	return count;
}

/* Sysfs attribute: xcvr_present_%d */
static ssize_t xcvr_present_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct nh_xcvr_ctrl *ctrl = dev_get_drvdata(dev);

	/* We could reverse the presence bit with !nh_xcvr_ctrl_read_reg
	so we do not need to modify presentHoldHi but it would not match the raw hardware value */
	return sysfs_emit(buf, "%u\n",
			  nh_xcvr_ctrl_read_reg(ctrl,
						ctrl->group->present_reg_offset,
						ctrl->present_bit));
}

/* Create sysfs attributes for transceiver control */
static int nh_xcvr_ctrl_create_attrs(struct device *dev, u32 port_num)
{
	struct device_attribute *reset_attr, *lpmode_attr, *present_attr;
	char *reset_name, *lpmode_name, *present_name;
	struct attribute_group *group;
	struct attribute **attrs;

	/* Create attribute names */
	reset_name = devm_kasprintf(dev, GFP_KERNEL, "xcvr_reset_%u", port_num);
	lpmode_name =
		devm_kasprintf(dev, GFP_KERNEL, "xcvr_low_power_%u", port_num);
	present_name =
		devm_kasprintf(dev, GFP_KERNEL, "xcvr_present_%u", port_num);
	if (!reset_name || !lpmode_name || !present_name)
		return -ENOMEM;

	/* Allocate attributes */
	reset_attr = devm_kzalloc(dev, sizeof(*reset_attr), GFP_KERNEL);
	lpmode_attr = devm_kzalloc(dev, sizeof(*lpmode_attr), GFP_KERNEL);
	present_attr = devm_kzalloc(dev, sizeof(*present_attr), GFP_KERNEL);
	attrs = devm_kcalloc(dev, 4, sizeof(*attrs), GFP_KERNEL);
	group = devm_kzalloc(dev, sizeof(*group), GFP_KERNEL);
	if (!reset_attr || !lpmode_attr || !present_attr || !attrs || !group)
		return -ENOMEM;

	/* Setup reset attribute */
	reset_attr->attr.name = reset_name;
	reset_attr->attr.mode = 0644;
	reset_attr->show = xcvr_reset_show;
	reset_attr->store = xcvr_reset_store;
	sysfs_attr_init(&reset_attr->attr);

	/* Setup low power attribute */
	lpmode_attr->attr.name = lpmode_name;
	lpmode_attr->attr.mode = 0644;
	lpmode_attr->show = xcvr_low_power_show;
	lpmode_attr->store = xcvr_low_power_store;
	sysfs_attr_init(&lpmode_attr->attr);

	/* Setup present attribute */
	present_attr->attr.name = present_name;
	present_attr->attr.mode = 0444;
	present_attr->show = xcvr_present_show;
	present_attr->store = NULL;
	sysfs_attr_init(&present_attr->attr);

	/* Register as a devm-managed group so the sysfs files are removed
	 * on unbind before devres frees the attribute memory above. */
	attrs[0] = &reset_attr->attr;
	attrs[1] = &lpmode_attr->attr;
	attrs[2] = &present_attr->attr;
	attrs[3] = NULL;
	group->attrs = attrs;

	return devm_device_add_group(dev, group);
}

/* Auxiliary device probe function */
static int nh_xcvr_ctrl_probe(struct auxiliary_device *aux_dev,
			      const struct auxiliary_device_id *id)
{
	struct fbiob_aux_adapter *adap =
		container_of(aux_dev, struct fbiob_aux_adapter, auxdev);
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	const struct nh_platform_cfg *platform_cfg;
	struct nh_xcvr_ctrl *ctrl;
	u16 device_id;
	u32 slot;
	int ret;

	device_id = pdev->device;

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg || !platform_cfg->xcvr_cfg ||
	    !platform_cfg->xcvr_cfg->groups ||
	    platform_cfg->xcvr_cfg->num_groups == 0) {
		dev_err(&aux_dev->dev, "No xcvr config for device ID: 0x%04x\n",
			device_id);
		return -ENODEV;
	}

	/* Validate every group: start <= end, and the range fits in one
	 * 32-bit register (end - start < 32). Reject the platform if any
	 * group is malformed — getting bit_pos wrong silently is worse than
	 * refusing to probe.
	 */
	{
		const struct xcvr_ctrl_config *cfg = platform_cfg->xcvr_cfg;
		u32 i;

		for (i = 0; i < cfg->num_groups; i++) {
			const struct xcvr_port_group *g = &cfg->groups[i];

			if (g->end_port < g->start_port ||
			    g->end_port - g->start_port >= 32) {
				dev_err(&aux_dev->dev,
					"xcvr_port_group[%u] {start=%u end=%u} is malformed on device 0x%04x\n",
					i, g->start_port, g->end_port,
					device_id);
				return -EINVAL;
			}
		}
	}

	/* Allocate ctrl structure */
	ctrl = devm_kzalloc(&aux_dev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->platform_cfg = platform_cfg;
	ctrl->aux_dev = aux_dev;
	ctrl->base = nh_irq_mmio_base(pdev);
	if (!ctrl->base)
		return -ENODEV;
	ctrl->base += nh_csr_rel(pdev, adap->data.csr_offset);
	/* port_num is the transceiver id (names the sysfs attrs); the register
	 * window is looked up by slot - see xcvr_ctrl_config::id_to_slot. */
	ctrl->port_num = adap->data.xcvr_data.port_num;

	slot = xcvr_ctrl_slot_for_port(platform_cfg->xcvr_cfg, ctrl->port_num);

	ctrl->group = xcvr_ctrl_find_port_group(platform_cfg->xcvr_cfg, slot);
	if (!ctrl->group) {
		dev_err(&aux_dev->dev,
			"port %u (slot %u) not covered by any xcvr_port_group on device 0x%04x\n",
			ctrl->port_num, slot, device_id);
		return -ENODEV;
	}
	/* Each OSFP group covers at most 32 slots; the slot's bit
	 * position within its 32-bit register is its 0-based offset
	 * within the group, shared across reset/lp/present. A
	 * fixed_bits group (e.g. the mgmt QSFP) instead pins each
	 * signal to its own bit. */
	if (ctrl->group->fixed_bits) {
		ctrl->reset_bit = ctrl->group->reset_bit;
		ctrl->lp_mode_bit = ctrl->group->lp_mode_bit;
		ctrl->present_bit = ctrl->group->present_bit;
	} else {
		u32 bit_pos = slot - ctrl->group->start_port;

		ctrl->reset_bit = bit_pos;
		ctrl->lp_mode_bit = bit_pos;
		ctrl->present_bit = bit_pos;
	}

	/* Create sysfs attributes */
	ret = nh_xcvr_ctrl_create_attrs(&aux_dev->dev, ctrl->port_num);
	if (ret) {
		dev_err(&aux_dev->dev,
			"Failed to create sysfs attributes: %d\n", ret);
		return ret;
	}

	/* Store ctrl in auxiliary device data */
	auxiliary_set_drvdata(aux_dev, ctrl);

	dev_info(&aux_dev->dev, "NH xcvr ctrl driver registered\n");
	return 0;
}

/* Auxiliary device remove function */
static void nh_xcvr_ctrl_remove(struct auxiliary_device *aux_dev)
{
	/* Attributes were added with devm_device_add_group() and are
	 * removed automatically on unbind. */
	dev_info(&aux_dev->dev, "NH xcvr ctrl driver removed\n");
}

/* Auxiliary device ID table */
static const struct auxiliary_device_id nh_xcvr_ctrl_aux_id_table[] = {
	{
		.name = FBOSS_IOB_PCI_DRIVER".nh_xcvr_ctrl",
	},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_xcvr_ctrl_aux_id_table);

/* Auxiliary driver structure */
static struct auxiliary_driver nh_xcvr_ctrl_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_xcvr_ctrl_probe,
	.remove = nh_xcvr_ctrl_remove,
	.id_table = nh_xcvr_ctrl_aux_id_table,
};

/* Module initialization */
static int __init nh_xcvr_ctrl_init_module(void)
{
	int ret;

	pr_info("NH XCVR Ctrl driver v%s loading\n", DRIVER_VERSION);

	ret = auxiliary_driver_register(&nh_xcvr_ctrl_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		return ret;
	}

	pr_info("NH XCVR Ctrl driver loaded successfully\n");
	return 0;
}

/* Module cleanup */
static void __exit nh_xcvr_ctrl_exit_module(void)
{
	pr_info("NH XCVR Ctrl driver unloading\n");
	auxiliary_driver_unregister(&nh_xcvr_ctrl_aux_driver);
	pr_info("NH XCVR Ctrl driver unloaded\n");
}

module_init(nh_xcvr_ctrl_init_module);
module_exit(nh_xcvr_ctrl_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mike Zhang <mike@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA Transceiver Control Driver");
MODULE_VERSION(DRIVER_VERSION);
