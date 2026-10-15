// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.

#include <linux/module.h>
#include <linux/auxiliary_bus.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <dt-bindings/leds/common.h>

#include "fbiob-auxdev.h"
#include "nh_fpga_port_led.h"
#include "nh_fpga_led_trigger.h"
#include "nh_irq.h"
#include "platform/nh_platform.h"

#define DRIVER_VERSION "1.0.0"

/* Port LED registers may be shared with nh_fpga_led: RMW is serialized by
 * the parent FPGA's led_reg_lock. */

/* Resolve a physical LED number to the transceiver id its sysfs node is
 * named after; identity if the platform has no led_to_id table. */
static u32 port_led_id_for_num(const struct nh_platform_cfg *cfg, u32 num)
{
	if (!cfg->led_to_id || num >= cfg->led_to_id_len)
		return num;
	return cfg->led_to_id[num];
}

/* Get bit position and offset for port */
static int
nh_port_led_get_bit_and_offset(const struct port_led_color_addr *color_table,
			       enum port_led_color color, u32 port_num,
			       u32 *offset, u32 *bit)
{
	const struct port_led_color_addr *c = &color_table[color];
	int i;

	for (i = 0; i < c->num_ranges; i++) {
		const struct port_led_range *r = &c->ranges[i];
		if (port_num >= r->start_port && port_num <= r->end_port) {
			*bit = (port_num - r->start_port) * r->bits_per_port +
			       r->color_bit_offset;
			*offset = r->offset;
			return 0;
		}
	}

	return -EINVAL;
}

static enum led_brightness
nh_port_led_brightness_get(struct led_classdev *led_cdev)
{
	struct port_led_dev *led_dev =
		container_of(led_cdev, struct port_led_dev, cdev);
	const struct port_led_color_addr *color_table =
		led_dev->ctrl->platform_cfg->port_led_cfg;
	u32 offset, bit, reg_val;
	int ret;

	ret = nh_port_led_get_bit_and_offset(color_table, led_dev->color,
					     led_dev->port_num, &offset, &bit);
	if (ret)
		return 0;

	reg_val = ioread32(led_dev->ctrl->base + offset);
	return (reg_val & BIT(bit)) ? 1 : 0;
}

static int nh_port_led_brightness_set(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct port_led_dev *led_dev =
		container_of(led_cdev, struct port_led_dev, cdev);
	struct port_led_ctrl *ctrl = led_dev->ctrl;
	const struct port_led_color_addr *color_table =
		ctrl->platform_cfg->port_led_cfg;
	u32 offset, bit, reg_val;
	int ret;

	ret = nh_port_led_get_bit_and_offset(color_table, led_dev->color,
					     led_dev->port_num, &offset, &bit);
	if (ret)
		return ret;

	mutex_lock(ctrl->reg_lock);
	if (brightness) {
		/* Turn off other color first, LEDs should be mutually exclusive */
		u32 off_offset, off_bit;

		ret = nh_port_led_get_bit_and_offset(color_table,
						     !led_dev->color,
						     led_dev->port_num,
						     &off_offset, &off_bit);
		if (ret) {
			mutex_unlock(ctrl->reg_lock);
			return ret;
		}
		if (off_offset == offset) {
			/* Both colors live in one register: clear and set in
			 * a single RMW so the cleared off color bit is not
			 * written back stale. */
			reg_val = ioread32(ctrl->base + offset);
			reg_val &= ~BIT(off_bit);
			reg_val |= BIT(bit);
		} else {
			u32 off_reg = ioread32(ctrl->base + off_offset) &
				      ~BIT(off_bit);

			iowrite32(off_reg, ctrl->base + off_offset);
			reg_val = ioread32(ctrl->base + offset) | BIT(bit);
		}
	} else {
		reg_val = ioread32(ctrl->base + offset) & ~BIT(bit);
	}
	iowrite32(reg_val, ctrl->base + offset);
	mutex_unlock(ctrl->reg_lock);

	return 0;
}

static int nh_port_led_probe(struct auxiliary_device *aux_dev,
			     const struct auxiliary_device_id *id)
{
	struct fbiob_aux_adapter *adap =
		container_of(aux_dev, struct fbiob_aux_adapter, auxdev);
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	const struct nh_platform_cfg *platform_cfg;
	struct port_led_ctrl *ctrl;
	int port = adap->data.led_data.port_num;
	int led_idx = adap->data.led_data.led_idx;
	u16 device_id = pdev->device;

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg || !platform_cfg->port_led_cfg) {
		dev_err(&aux_dev->dev,
			"No port LED config for device ID: 0x%04x\n",
			device_id);
		return -ENODEV;
	}

	if (port < 1 || port > (int)platform_cfg->num_ports_per_led ||
	    led_idx < 1 || led_idx > 2)
		return -EINVAL;

	if (platform_cfg->dual_color_per_led && led_idx != 1) {
		dev_err(&aux_dev->dev,
			"led_idx %d invalid with dual_color_per_led on device 0x%04x\n",
			led_idx, device_id);
		return -EINVAL;
	}

	/* Reject ports this FPGA's range tables do not cover, otherwise we would
	 * register an LED whose set path always fails. */
	{
		u32 offset, bit;
		int color;

		for (color = 0; color < PORT_LED_NUM_COLORS; color++) {
			if (nh_port_led_get_bit_and_offset(
				    platform_cfg->port_led_cfg, color, port,
				    &offset, &bit)) {
				dev_err(&aux_dev->dev,
					"Port %d not covered by %s LED ranges on device 0x%04x\n",
					port, color_names[color], device_id);
				return -ENODEV;
			}
		}
	}

	ctrl = devm_kzalloc(&aux_dev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->platform_cfg = platform_cfg;
	ctrl->aux_dev = aux_dev;
	ctrl->base = nh_irq_mmio_base(pdev);
	if (!ctrl->base)
		return -ENODEV;
	ctrl->reg_lock = nh_irq_led_reg_lock(pdev);
	if (!ctrl->reg_lock)
		return -ENODEV;
	ctrl->num_leds =
		platform_cfg->dual_color_per_led ? PORT_LED_NUM_COLORS : 1;

	ctrl->leds = devm_kcalloc(&aux_dev->dev, ctrl->num_leds,
				  sizeof(*ctrl->leds), GFP_KERNEL);
	if (!ctrl->leds)
		return -ENOMEM;

	/* Name by transceiver id, not the physical LED number used for register
	 * access. In dual-color mode both colors share one physical LED at index
	 * 1; otherwise led_idx is the index and selects the color. */
	u32 tcvr_id = port_led_id_for_num(platform_cfg, port);
	int name_idx = platform_cfg->dual_color_per_led ? 1 : led_idx;

	for (int i = 0; i < ctrl->num_leds; i++) {
		struct port_led_dev *led = &ctrl->leds[i];
		char name[64];
		int ret;

		led->ctrl = ctrl;
		led->port_num = port;
		led->color =
			platform_cfg->dual_color_per_led ?
				i :
				(led_idx == 1 ? PORT_LED_BLUE : PORT_LED_AMBER);

		snprintf(name, sizeof(name),
			 "port%d_led%d:%s:" LED_FUNCTION_STATUS, tcvr_id,
			 name_idx, color_names[led->color]);
		led->cdev.name = devm_kstrdup(&aux_dev->dev, name, GFP_KERNEL);
		if (!led->cdev.name)
			return -ENOMEM;

		led->cdev.brightness_get = nh_port_led_brightness_get;
		led->cdev.brightness_set_blocking = nh_port_led_brightness_set;
		led->cdev.max_brightness = 1;

		ret = devm_led_classdev_register(&aux_dev->dev, &led->cdev);
		if (ret)
			return ret;

		/* Trigger init failure is non-fatal: keep the LED without it. */
		ret = led_trigger_init(led->cdev.dev);
		if (ret)
			dev_warn(&aux_dev->dev,
				 "Failed to initialize LED trigger: %d\n", ret);
		led->trigger_inited = !ret;
	}

	auxiliary_set_drvdata(aux_dev, ctrl);

	return 0;
}

/* Auxiliary device remove function */
static void nh_port_led_remove(struct auxiliary_device *aux_dev)
{
	struct port_led_ctrl *ctrl = auxiliary_get_drvdata(aux_dev);

	if (ctrl && ctrl->leds) {
		for (int i = 0; i < ctrl->num_leds; i++)
			if (ctrl->leds[i].trigger_inited)
				led_trigger_deinit(ctrl->leds[i].cdev.dev);
	}

	dev_info(&aux_dev->dev, "NH port LED driver removed\n");
}

static const struct auxiliary_device_id nh_port_led_aux_id_table[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER".nh_port_led" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_port_led_aux_id_table);

static struct auxiliary_driver nh_port_led_aux_driver = {
	.name = PORT_LED_DRIVER_NAME,
	.probe = nh_port_led_probe,
	.remove = nh_port_led_remove,
	.id_table = nh_port_led_aux_id_table,
};

static int __init nh_port_led_init_module(void)
{
	int ret;

	pr_info("NH Port LED driver v%s loading\n", DRIVER_VERSION);

	ret = auxiliary_driver_register(&nh_port_led_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		return ret;
	}

	pr_info("NH Port LED driver loaded successfully\n");
	return 0;
}

static void __exit nh_port_led_exit_module(void)
{
	pr_info("NH Port LED driver unloading\n");
	auxiliary_driver_unregister(&nh_port_led_aux_driver);
	pr_info("NH Port LED driver unloaded\n");
}

module_init(nh_port_led_init_module);
module_exit(nh_port_led_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mike Zhang <mike@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA Port LED Controller Driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_SOFTDEP("pre: nh_irq");
