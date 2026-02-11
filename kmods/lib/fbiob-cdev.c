// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include "fbiob-cdev.h"

static int aux_data_validate(struct fbiob_cdev_desc *cdesc,
			     struct fbiob_aux_data *aux_data)
{
	/*
	 * csr_offset is mandatory when creating I/O controllers.
	 */
	if ((aux_data->csr_offset == FBIOB_INVALID_OFFSET) ||
	    (aux_data->csr_offset >= cdesc->max_offset))
		return -EINVAL;

	/* Translate csr_offset to csr_bus_addr. */
	aux_data->csr_offset += cdesc->base_bus_addr;

	/* Translate iobuf_offset to iobuf_bus_addr if needed. */
	if (aux_data->iobuf_offset != FBIOB_INVALID_OFFSET) {
		if (aux_data->iobuf_offset >= cdesc->max_offset)
			return -EINVAL;

		aux_data->iobuf_offset += cdesc->base_bus_addr;
	}

	return 0;
}

static long fbiob_cdev_ioctl(struct file *file,
			     unsigned int cmd,
			     unsigned long param)
{
	int ret = -EINVAL;
	struct fbiob_aux_data aux_data;
	void __user *userp = (void __user *)param;
	struct fbiob_cdev_desc *cdesc = container_of(file->private_data,
					struct fbiob_cdev_desc, miscdev);

	if (copy_from_user(&aux_data, userp, sizeof(aux_data)))
		return -EFAULT;

	switch (cmd) {
	case FBIOB_IOC_NEW_DEVICE:
		ret = aux_data_validate(cdesc, &aux_data);
		if (!ret)
			ret = fbiob_auxdev_add(cdesc->aux_bus, &aux_data);
		break;

	case FBIOB_IOC_DEL_DEVICE:
		ret = fbiob_auxdev_remove(cdesc->aux_bus, &aux_data.id);
		break;
	}

	return ret;
}

static const struct file_operations fbiob_cdev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fbiob_cdev_ioctl,
};

void fbiob_cdev_destroy(struct fbiob_cdev_desc *cdesc)
{
	misc_deregister(&cdesc->miscdev);
}

int fbiob_cdev_init(struct fbiob_cdev_desc *cdesc,
		    struct pci_dev *pcidev,
		    struct fbiob_aux_bus *aux_bus,
		    resource_size_t base_bus_addr,
		    resource_size_t max_offset)
{
	cdesc->aux_bus = aux_bus;
	cdesc->base_bus_addr = base_bus_addr;
	cdesc->max_offset = max_offset;
	snprintf(cdesc->name, sizeof(cdesc->name), "fbiob_%04x.%04x.%04x.%04x",
		 pcidev->vendor, pcidev->device, pcidev->subsystem_vendor,
		 pcidev->subsystem_device);

	cdesc->miscdev.minor = MISC_DYNAMIC_MINOR;
	cdesc->miscdev.name = cdesc->name;
	cdesc->miscdev.fops = &fbiob_cdev_fops;
	cdesc->miscdev.parent = &pcidev->dev;

	return misc_register(&cdesc->miscdev);
}
