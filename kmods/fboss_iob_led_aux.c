// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/limits.h>
#include <linux/module.h>

#include "fbiob-auxdev.h"
#include "fboss_iob_led_aux.h"
#include "fboss_iob_led_trigger.h"

#define DRIVER_NAME "fboss_iob_led"

#define FBIOB_PORT_LED_FUNC			"status"

#define PORT_LED_COLOR_PROF_MASK(profile)	(profile << 2)
#define PORT_LED_FLASH_CTRL			BIT(1)
#define PORT_LED_ON_OFF_CTRL			BIT(0)
#define PORT_LED_ON_MASK(profile) \
	(PORT_LED_COLOR_PROF_MASK(profile) | PORT_LED_ON_OFF_CTRL)

#define PORT_LED_MASK_ALL ((7 << 2) | PORT_LED_FLASH_CTRL | PORT_LED_ON_OFF_CTRL)

enum color_profile {WHITE, CYAN, BLUE, PINK, RED, ORANGE, AMBER, GREEN};

static enum led_brightness brightness_get(struct led_classdev *led_cdev)
{
	u32 data;
	struct port_led_dev *ldev = container_of(led_cdev,
					struct port_led_dev, cdev);

	mutex_lock(ldev->lock);
	data = readl(ldev->mmio_csr);
	mutex_unlock(ldev->lock);

	return (data == ldev->led_on_mask) ? LED_ON : LED_OFF;
}

static int brightness_set(struct led_classdev *led_cdev,
			  enum led_brightness value)
{

	u32 data;
	struct port_led_dev *ldev = container_of(led_cdev,
					struct port_led_dev, cdev);

	mutex_lock(ldev->lock);

	data = readl(ldev->mmio_csr);
	if (value == 0) {
		data &= ~(ldev->led_on_mask);
	} else {
		// turn off control bits before turning specific color on
		data &= ~PORT_LED_MASK_ALL;
		data |= ldev->led_on_mask;
	}
	writel(data, ldev->mmio_csr);

	mutex_unlock(ldev->lock);
	return 0;
}

static int port_led_init(struct port_led_dev *ldev,
			 struct port_led_priv *priv,
			 const char *color)
{
	int ret;
	u32 val;

	/* initial port led device color */
	if (!strcmp(color, "white"))
		ldev->led_on_mask = PORT_LED_ON_MASK(WHITE);
	else if (!strcmp(color, "cyan"))
		ldev->led_on_mask = PORT_LED_ON_MASK(CYAN);
	else if (!strcmp(color, "blue"))
		ldev->led_on_mask = PORT_LED_ON_MASK(BLUE);
	else if (!strcmp(color, "pink"))
		ldev->led_on_mask = PORT_LED_ON_MASK(PINK);
	else if (!strcmp(color, "red"))
		ldev->led_on_mask = PORT_LED_ON_MASK(RED);
	else if (!strcmp(color, "orange"))
		ldev->led_on_mask = PORT_LED_ON_MASK(ORANGE);
	else if (!strcmp(color, "amber"))
		ldev->led_on_mask = PORT_LED_ON_MASK(AMBER);
	else if (!strcmp(color, "green"))
		ldev->led_on_mask = PORT_LED_ON_MASK(GREEN);
	else
		return -EINVAL;

	ldev->lock = &priv->lock;
	ldev->mmio_csr = priv->mmio_csr;

	/*
	 * NOTE: make sure "qsfp_service" config is updated whenever the
	 * led naming format are changed.
	 */
	if (priv->led_idx > 0)
		snprintf(ldev->name, sizeof(ldev->name), "port%d_led%d:%s:%s",
			 priv->port_num, priv->led_idx, color,
			 FBIOB_PORT_LED_FUNC);
	else
		snprintf(ldev->name, sizeof(ldev->name), "port%d_led:%s:%s",
			 priv->port_num, color, FBIOB_PORT_LED_FUNC);

	/*
	 * Set default color to green, no blinking.
	 */
	val = readl(ldev->mmio_csr);
	val &= ~PORT_LED_MASK_ALL;
	val |= ldev->led_on_mask;
	writel(val, ldev->mmio_csr);

	ldev->priv = priv;
	ldev->cdev.name = ldev->name;
	ldev->cdev.brightness_get = brightness_get;
	ldev->cdev.brightness_set_blocking = brightness_set;
	ldev->cdev.max_brightness = 1;

	ret = devm_led_classdev_register(&ldev->priv->auxdev->dev, &ldev->cdev);
	if (ret) {
		dev_err(&ldev->priv->auxdev->dev, "could not register led: %d", ret);
		return ret;
	}
	return led_trigger_init(ldev->cdev.dev);
}

static void port_led_remove(struct auxiliary_device *auxdev)
{
	struct port_led_priv *priv = dev_get_drvdata(&auxdev->dev);

	for (int i = 0; i < priv->num_leds; ++i)
		led_trigger_deinit(priv->leds[i].cdev.dev);

	mutex_destroy(&priv->lock);
}


static int port_led_probe(struct auxiliary_device *auxdev,
			  const struct auxiliary_device_id *id)
{
	int i, ret = 0;
	struct resource *res;
	struct port_led_priv *priv;
	struct device *dev = &auxdev->dev;
	struct fbiob_aux_adapter *aux_adap =
			(struct fbiob_aux_adapter *)container_of(auxdev,
					struct fbiob_aux_adapter, auxdev);
	struct fbiob_aux_data *pdata = &aux_adap->data;

	/*
	 * Port number (1-based integer) must be supplied.
	 */
	if (pdata->led_data.port_num <= 0)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	dev_set_drvdata(dev, priv);

	priv->csr_bus_addr = aux_adap->data.csr_offset;
	res = devm_request_mem_region(dev, priv->csr_bus_addr,
				FBIOB_LED_BLK_SIZE, auxdev->name);
	if (!res)
		return -EBUSY;

	priv->mmio_csr = devm_ioremap(dev, priv->csr_bus_addr,
					FBIOB_LED_BLK_SIZE);
	if (!priv->mmio_csr)
		return -ENOMEM;

	mutex_init(&priv->lock);
	priv->auxdev = auxdev;
	priv->port_num = pdata->led_data.port_num;
	priv->led_idx = pdata->led_data.led_idx;
	priv->num_leds = ARRAY_SIZE(pled_colors);

	for (i = 0; i < priv->num_leds; i++) {
		ret = port_led_init(&priv->leds[i], priv, pled_colors[i]);
		if (ret < 0)
			goto error;
	}

	dev_info(dev, "port%d_led_%d (csr_addr=0x%x) registered",
		 priv->port_num, priv->led_idx, priv->csr_bus_addr);
	return 0;

error:
	mutex_destroy(&priv->lock);
	return ret;
}

static const struct auxiliary_device_id fboss_iob_led_ids[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER".port_led" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_led_ids);

static struct auxiliary_driver fboss_iob_led_driver = {
	.driver = {
		.name = DRIVER_NAME
	},
	.probe = port_led_probe,
	.remove = port_led_remove,
	.id_table = fboss_iob_led_ids,
};
module_auxiliary_driver(fboss_iob_led_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Scott Smith <smithscott@meta.com>");
MODULE_DESCRIPTION("Meta FBOSS SFP Port LED Driver");
MODULE_VERSION(BSP_VERSION);
