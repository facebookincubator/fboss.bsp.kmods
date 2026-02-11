// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "fboss_iob_pci.h"

#define PCI_VENDOR_ID_META	0x1d9b
#define PCI_DEVICE_ID_FBIOB	0x0011
#define FBIOB_BAR_CSR		0

static int fbiob_init_pci(struct fbiob_priv *priv)
{
	int ret;
	struct pci_dev *pdev = priv->pdev;

	ret = pcim_enable_device(pdev);
	if (ret)
		return -ENODEV;

	priv->bar0_bus_addr = pci_resource_start(pdev, FBIOB_BAR_CSR);
	priv->bar0_size = pci_resource_len(pdev, FBIOB_BAR_CSR);

	return 0;
}

static void fbiob_pci_remove(struct pci_dev *pdev)
{
	struct fbiob_priv *priv = pci_get_drvdata(pdev);

	fbiob_cdev_destroy(&priv->cdev_desc);
	fbiob_auxbus_destroy(&priv->aux_bus);
}

static int fbiob_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *id)
{
	int ret;
	struct fbiob_priv *priv;
	struct device *dev = &pdev->dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->pdev = pdev;
	pci_set_drvdata(pdev, priv);

	ret = fbiob_init_pci(priv);
	if (ret)
		return ret;

	ret = fbiob_auxbus_init(&priv->aux_bus, dev);
	if (ret)
		return ret;

	ret = fbiob_cdev_init(&priv->cdev_desc, pdev, &priv->aux_bus,
				priv->bar0_bus_addr, priv->bar0_size);
	if (ret)
		goto error;

	return 0;

error:
	fbiob_auxbus_destroy(&priv->aux_bus);
	return ret;
}

static struct pci_device_id fboss_iob_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_META, PCI_DEVICE_ID_FBIOB) },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, fboss_iob_pci_ids);

static struct pci_driver fboss_iob_pci_driver = {
	.name = FBOSS_IOB_PCI_DRIVER,
	.id_table = fboss_iob_pci_ids,
	.probe = fbiob_pci_probe,
	.remove = fbiob_pci_remove,
};

module_pci_driver(fboss_iob_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tao Ren <taoren@meta.com>");
MODULE_DESCRIPTION("Meta FBOSS IOB_FPGA PCIe Driver");
MODULE_VERSION(BSP_VERSION);
