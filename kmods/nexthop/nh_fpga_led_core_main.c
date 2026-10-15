// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.

#include <linux/module.h>
#include <linux/auxiliary_bus.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include "fbiob-auxdev.h"
#include "nh_fpga_led_core.h"
#include "nh_fpga_led_trigger.h"
#include "nh_irq.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_fpga_led"
#define DRIVER_VERSION "1.0.0"

/* PSU, FAN and SYS LEDs share one control register, possibly with
 * nh_fpga_port_led: RMW is serialized by the parent FPGA's led_reg_lock. */

/* ============================================================================
 * LED Control Functions
 * ============================================================================ */

/* Get LED brightness */
static enum led_brightness nh_led_brightness_get(struct led_classdev *led_cdev)
{
	struct nh_led_core_dev *led_dev =
		container_of(led_cdev, struct nh_led_core_dev, cdev);
	struct nh_led_core_controller *ctrl = led_dev->ctrl;
	const struct nh_led_core_info *led_info_base =
		ctrl->platform_cfg->led_core_cfg->led_info;
	int num_leds = ctrl->platform_cfg->led_core_cfg->num_leds;
	const struct nh_led_core_info *info =
		&led_info_base[ctrl->led_type * num_leds + led_dev->color];
	u32 reg_val, mask;

	mask = (1 << info->color_field_width) - 1;

	mutex_lock(ctrl->reg_lock);
	reg_val = ioread32(ctrl->base +
			   ctrl->platform_cfg->led_core_cfg->control_offset);
	mutex_unlock(ctrl->reg_lock);

	return (enum led_brightness)((reg_val >> info->color_offset) & mask);
}

/* Set LED brightness */
static int nh_led_brightness_set(struct led_classdev *led_cdev,
				 enum led_brightness brightness)
{
	struct nh_led_core_dev *led_dev =
		container_of(led_cdev, struct nh_led_core_dev, cdev);
	struct nh_led_core_controller *ctrl = led_dev->ctrl;
	const struct nh_led_core_info *led_info_base =
		ctrl->platform_cfg->led_core_cfg->led_info;
	int num_leds = ctrl->platform_cfg->led_core_cfg->num_leds;
	const struct nh_led_core_info *info =
		&led_info_base[ctrl->led_type * num_leds + led_dev->color];
	u32 reg_val, new_reg_val, mask;

	mask = (1 << info->color_field_width) - 1;

	mutex_lock(ctrl->reg_lock);

	reg_val = ioread32(ctrl->base +
			   ctrl->platform_cfg->led_core_cfg->control_offset);
	new_reg_val = reg_val;

	new_reg_val &= ~(mask << info->color_offset);
	new_reg_val |= ((brightness & mask) << info->color_offset);

	iowrite32(new_reg_val,
		  ctrl->base +
			  ctrl->platform_cfg->led_core_cfg->control_offset);

	mutex_unlock(ctrl->reg_lock);

	dev_dbg(&ctrl->aux_dev->dev,
		"LED %s: brightness=%d, reg=0x%08x->0x%08x\n", led_cdev->name,
		brightness, reg_val, new_reg_val);

	return 0;
}

/* Set LED blink */
static int nh_led_blink_set(struct led_classdev *led_cdev,
			    unsigned long *delay_on, unsigned long *delay_off)
{
	struct nh_led_core_dev *led_dev =
		container_of(led_cdev, struct nh_led_core_dev, cdev);
	struct nh_led_core_controller *ctrl = led_dev->ctrl;
	const struct nh_led_core_info *led_info_base =
		ctrl->platform_cfg->led_core_cfg->led_info;
	int num_leds = ctrl->platform_cfg->led_core_cfg->num_leds;
	const struct nh_led_core_info *info =
		&led_info_base[ctrl->led_type * num_leds + led_dev->color];
	u32 reg_val, new_reg_val;
	bool blink_enable = (*delay_on != 0 || *delay_off != 0);

	mutex_lock(ctrl->reg_lock);

	reg_val = ioread32(ctrl->base +
			   ctrl->platform_cfg->led_core_cfg->control_offset);
	new_reg_val = reg_val;

	if (blink_enable)
		new_reg_val |= BIT(info->blink_bit);
	else
		new_reg_val &= ~BIT(info->blink_bit);

	iowrite32(new_reg_val,
		  ctrl->base +
			  ctrl->platform_cfg->led_core_cfg->control_offset);

	mutex_unlock(ctrl->reg_lock);

	dev_dbg(&ctrl->aux_dev->dev, "LED %s: blink=%s, reg=0x%08x->0x%08x\n",
		led_cdev->name, blink_enable ? "on" : "off", reg_val,
		new_reg_val);

	return 0;
}

/* ============================================================================
 * LED Registration
 * ============================================================================ */

static int nh_led_register_leds(struct nh_led_core_controller *ctrl)
{
	int led_entry, ret, color_slot;
	char led_name[NAME_MAX];
	int num_leds = ctrl->platform_cfg->led_core_cfg->num_leds;

	const struct nh_led_core_info *led_info_base =
		ctrl->platform_cfg->led_core_cfg->led_info;

	for (led_entry = 0; led_entry < num_leds; led_entry++) {
		for (color_slot = 0;
		     color_slot <
		     ctrl->platform_cfg->led_core_cfg->num_colors_per_led;
		     color_slot++) {
			struct nh_led_core_dev *led_dev =
				&ctrl->leds[led_entry *
						    ctrl->platform_cfg
							    ->led_core_cfg
							    ->num_colors_per_led +
					    color_slot];
			const struct nh_led_core_info *info =
				&led_info_base[ctrl->led_type * num_leds +
					       led_entry];

			led_dev->ctrl = ctrl;
			led_dev->color = led_entry;

			snprintf(
				led_name, sizeof(led_name), "%s:%s:status",
				info->name,
				ctrl->platform_cfg->led_core_cfg
					->color_names[info->colors[color_slot]]);
			led_dev->cdev.name = devm_kstrdup(&ctrl->aux_dev->dev,
							  led_name, GFP_KERNEL);
			if (!led_dev->cdev.name)
				return -ENOMEM;

			led_dev->cdev.brightness_get = nh_led_brightness_get;
			led_dev->cdev.brightness_set_blocking =
				nh_led_brightness_set;
			led_dev->cdev.blink_set = nh_led_blink_set;
			led_dev->cdev.max_brightness = 1;
			led_dev->cdev.brightness = 0;

			ret = devm_led_classdev_register(&ctrl->aux_dev->dev,
							 &led_dev->cdev);
			if (ret) {
				dev_err(&ctrl->aux_dev->dev,
					"Failed to register LED %s: %d\n",
					led_name, ret);
				return ret;
			}

			ret = led_trigger_init(led_dev->cdev.dev);
			if (ret) {
				dev_warn(
					&ctrl->aux_dev->dev,
					"Failed to initialize LED trigger for %s: %d\n",
					led_name, ret);
			}
			led_dev->trigger_inited = !ret;

			dev_dbg(&ctrl->aux_dev->dev, "Registered LED: %s\n",
				led_name);
		}
	}

	dev_info(&ctrl->aux_dev->dev, "Registered %d LED devices\n",
		 num_leds *
			 ctrl->platform_cfg->led_core_cfg->num_colors_per_led);
	return 0;
}

/* ============================================================================
 * Driver Infrastructure
 * ============================================================================ */

static int nh_led_probe(struct auxiliary_device *aux_dev,
			const struct auxiliary_device_id *id)
{
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	struct nh_led_core_controller *ctrl;
	const char *led_name;
	u16 device_id;
	int ret;

	device_id = pdev->device;

	ctrl = devm_kzalloc(&aux_dev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->platform_cfg = nh_get_platform_dev(pdev);
	if (!ctrl->platform_cfg || !ctrl->platform_cfg->led_core_cfg) {
		dev_err(&aux_dev->dev, "No LED config for device 0x%04x\n",
			device_id);
		return -ENODEV;
	}

	/* Allocate LED devices array based on platform config */
	ctrl->leds = devm_kzalloc(
		&aux_dev->dev,
		sizeof(*ctrl->leds) *
			ctrl->platform_cfg->led_core_cfg->num_leds *
			ctrl->platform_cfg->led_core_cfg->num_colors_per_led,
		GFP_KERNEL);
	if (!ctrl->leds)
		return -ENOMEM;

	ctrl->aux_dev = aux_dev;
	ctrl->base = nh_irq_mmio_base(pdev);
	if (!ctrl->base)
		return -ENODEV;
	ctrl->reg_lock = nh_irq_led_reg_lock(pdev);
	if (!ctrl->reg_lock)
		return -ENODEV;

	/* Determine LED type from auxiliary device name */
	led_name = strrchr(id->name, '.');
	if (!led_name) {
		dev_err(&aux_dev->dev, "Invalid auxiliary device name: %s\n",
			id->name);
		return -EINVAL;
	}
	led_name++; /* Skip the '.' */

	if (strcmp(led_name, "psu_led") == 0)
		ctrl->led_type = NH_LED_TYPE_PSU;
	else if (strcmp(led_name, "fan_led") == 0)
		ctrl->led_type = NH_LED_TYPE_FAN;
	else if (strcmp(led_name, "sys_led") == 0)
		ctrl->led_type = NH_LED_TYPE_SYS;
	else if (strcmp(led_name, "mgmt_qsfp_led") == 0)
		ctrl->led_type = NH_LED_TYPE_MGMT_QSFP;
	else {
		dev_err(&aux_dev->dev, "Unknown LED type: %s\n", led_name);
		return -EINVAL;
	}

	dev_set_drvdata(&aux_dev->dev, ctrl);

	const struct nh_led_core_info *led_info_base =
		ctrl->platform_cfg->led_core_cfg->led_info;
	int num_leds = ctrl->platform_cfg->led_core_cfg->num_leds;

	dev_info(&aux_dev->dev,
		 "Initializing %s LED Controller at offset 0x%x\n",
		 led_info_base[ctrl->led_type * num_leds].name,
		 ctrl->platform_cfg->led_core_cfg->control_offset);

	ret = nh_led_register_leds(ctrl);
	if (ret) {
		dev_err(&aux_dev->dev, "Failed to register LEDs: %d\n", ret);
		return ret;
	}

	dev_info(&aux_dev->dev, "%s LED Controller initialized successfully\n",
		 led_info_base[ctrl->led_type * num_leds].name);
	return 0;
}

static void nh_led_remove(struct auxiliary_device *aux_dev)
{
	struct nh_led_core_controller *ctrl = dev_get_drvdata(&aux_dev->dev);
	int i;

	if (ctrl && ctrl->leds) {
		int total_leds =
			ctrl->platform_cfg->led_core_cfg->num_leds *
			ctrl->platform_cfg->led_core_cfg->num_colors_per_led;
		for (i = 0; i < total_leds; i++) {
			if (ctrl->leds[i].trigger_inited)
				led_trigger_deinit(ctrl->leds[i].cdev.dev);
		}
	}
	dev_info(&aux_dev->dev, "LED Controller removed\n");
}

static const struct auxiliary_device_id nh_led_aux_id_table[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER".psu_led" },
	{ .name = FBOSS_IOB_PCI_DRIVER".fan_led" },
	{ .name = FBOSS_IOB_PCI_DRIVER".sys_led" },
	{ .name = FBOSS_IOB_PCI_DRIVER".mgmt_qsfp_led" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_led_aux_id_table);

static struct auxiliary_driver nh_led_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_led_probe,
	.remove = nh_led_remove,
	.id_table = nh_led_aux_id_table,
};

/* ============================================================================
 * Module Infrastructure
 * ============================================================================ */

static int __init nh_led_init_module(void)
{
	int ret;

	pr_info("NH FPGA LED driver v%s loading\n", DRIVER_VERSION);

	ret = auxiliary_driver_register(&nh_led_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		return ret;
	}

	pr_info("NH FPGA LED driver loaded successfully\n");
	return 0;
}

static void __exit nh_led_exit_module(void)
{
	pr_info("NH FPGA LED driver unloading\n");
	auxiliary_driver_unregister(&nh_led_aux_driver);
	pr_info("NH FPGA LED driver unloaded\n");
}

module_init(nh_led_init_module);
module_exit(nh_led_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Arif Mohammad <marif@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA LED Core Controller Driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_SOFTDEP("pre: nh_irq");
