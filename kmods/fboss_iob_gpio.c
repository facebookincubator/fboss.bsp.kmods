// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/auxiliary_bus.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/gpio/driver.h>

#include "fbiob-auxdev.h"

#define DRIVER_NAME	"fboss_iob_gpio"

#define FBIOB_GPIO_INPUT		BIT(2)
#define FBIOB_GPIO_OUTPUT		BIT(1)
#define FBIOB_GPIO_TRISTATE_CTRL	BIT(0)

#define FBIOB_GPIO_UNIT_SIZE		4
#define FBIOB_GPIO_NUM_LINES		256

#define GPIO_LINE_CSR(_l)		((_l) * FBIOB_GPIO_UNIT_SIZE)

struct fbiob_gpio_priv {
	struct gpio_chip	gc;
	struct device		*dev;
	struct auxiliary_device *auxdev;

	u32 csr_bus_addr;
	void __iomem *mmio_csr;
};

static u32 gpio_csr_read(struct fbiob_gpio_priv *priv, u32 offset)
{
	return readl(priv->mmio_csr + offset);
}

static void gpio_csr_write(struct fbiob_gpio_priv *priv, u32 offset, u32 val)
{
	writel(val, priv->mmio_csr + offset);
}

static int fbiob_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	u32 val;
	struct fbiob_gpio_priv *priv = gpiochip_get_data(gc);

	val = gpio_csr_read(priv, GPIO_LINE_CSR(offset));
	return ((val & FBIOB_GPIO_INPUT) > 0);
}

static int fbiob_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	u32 val;
	struct fbiob_gpio_priv *priv = gpiochip_get_data(gc);

	val = gpio_csr_read(priv, GPIO_LINE_CSR(offset));
	return (val | FBIOB_GPIO_TRISTATE_CTRL) ?
		GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int fbiob_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	u32 val;
	struct fbiob_gpio_priv *priv = gpiochip_get_data(gc);

	val = gpio_csr_read(priv, GPIO_LINE_CSR(offset));
	val |= FBIOB_GPIO_TRISTATE_CTRL;
	gpio_csr_write(priv, GPIO_LINE_CSR(offset), val);

	return 0;
}

static int fbiob_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int value)
{
	struct fbiob_gpio_priv *priv = gpiochip_get_data(gc);
	int direction = fbiob_gpio_get_direction(gc, offset);
	u32 val = gpio_csr_read(priv, GPIO_LINE_CSR(offset));

	if (direction == GPIO_LINE_DIRECTION_IN) {
		val &= ~FBIOB_GPIO_TRISTATE_CTRL;
		if (value)
			val |= FBIOB_GPIO_OUTPUT;
		else
			val &= ~FBIOB_GPIO_OUTPUT;
	} else {
		if (value)
			val |= FBIOB_GPIO_OUTPUT;
		else
			val &= ~FBIOB_GPIO_OUTPUT;
	}

	gpio_csr_write(priv, GPIO_LINE_CSR(offset), val);
	return 0;
}


static int fbiob_gpio_probe(struct auxiliary_device *auxdev,
			    const struct auxiliary_device_id *id)
{
	int ret;
	struct resource *res;
	struct fbiob_gpio_priv *priv;
	struct device *dev = &auxdev->dev;
	struct fbiob_aux_adapter *aux_adap =
			(struct fbiob_aux_adapter *)container_of(auxdev,
					struct fbiob_aux_adapter, auxdev);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	dev_set_drvdata(dev, priv);

	priv->csr_bus_addr = aux_adap->data.csr_offset;
	res = devm_request_mem_region(dev, priv->csr_bus_addr,
				FBIOB_GPIO_BLK_SIZE, auxdev->name);
	if (!res)
		return -EBUSY;

	priv->mmio_csr = devm_ioremap(dev, priv->csr_bus_addr,
					FBIOB_GPIO_BLK_SIZE);
	if (!priv->mmio_csr)
		return -ENOMEM;

	priv->dev = dev;
	priv->auxdev = auxdev;
	priv->gc.base = -1,
	priv->gc.parent = dev;
	priv->gc.label = dev_name(dev);
	priv->gc.get = fbiob_gpio_get,
	priv->gc.get_direction = fbiob_gpio_get_direction,
	priv->gc.direction_input = fbiob_gpio_direction_input,
	priv->gc.direction_output = fbiob_gpio_direction_output,
	priv->gc.can_sleep = true,
	priv->gc.ngpio = FBIOB_GPIO_NUM_LINES,

	ret = devm_gpiochip_add_data(dev, &priv->gc, priv);
	if (ret < 0)
		return ret;

	dev_info(dev, "gpiochip at %x registered", priv->csr_bus_addr);
	return 0;
}

static const struct auxiliary_device_id fboss_iob_gpio_ids[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER".gpiochip" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_gpio_ids);

static struct auxiliary_driver fboss_iob_gpio_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = fbiob_gpio_probe,
	.id_table = fboss_iob_gpio_ids,
};
module_auxiliary_driver(fboss_iob_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lucas Liu <lucli@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS IOB_FPGA GPIO Controller Driver");
MODULE_VERSION(BSP_VERSION);
