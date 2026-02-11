// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/slab.h>

#include "fbiob-auxdev.h"

static void fbiob_auxdev_release(struct device *dev)
{
	struct fbiob_aux_adapter *adap =
		(struct fbiob_aux_adapter *)container_of(dev,
			struct fbiob_aux_adapter, auxdev.dev);

	kfree(adap);
}

static bool fbiob_auxdev_match(struct fbiob_aux_id *id1,
				struct fbiob_aux_id *id2)
{
	return (id1->id == id2->id) && (!strcmp(id1->name, id2->name));
}

int fbiob_auxdev_remove(struct fbiob_aux_bus *bus, struct fbiob_aux_id *id)
{
	int ret = -ENXIO;
	struct fbiob_aux_adapter *adap, *tmp;

	mutex_lock(&bus->mutex);
	list_for_each_entry_safe(adap, tmp, &bus->devices, list) {
		if (fbiob_auxdev_match(&adap->data.id, id)) {
			list_del(&adap->list);
			auxiliary_device_delete(&adap->auxdev);
			auxiliary_device_uninit(&adap->auxdev);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&bus->mutex);

	return ret;
}

int fbiob_auxdev_add(struct fbiob_aux_bus *bus, struct fbiob_aux_data *data)
{
	int ret;
	struct fbiob_aux_adapter *adap;

	/*
	 * No need to call kfree() explicitly once auxiliary_device_init()
	 * returns success: auxiliary_device_uninit() will trigger the
	 * device release method which releases the memory.
	 */
	adap = kzalloc(sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;

	adap->data = *data;
	adap->auxdev.name = adap->data.id.name;
	adap->auxdev.id = adap->data.id.id;
	adap->auxdev.dev.parent = bus->parent;
	adap->auxdev.dev.release = fbiob_auxdev_release;

	ret = auxiliary_device_init(&adap->auxdev);
	if (ret < 0) {
		kfree(adap);
		return ret;
	}

	ret = auxiliary_device_add(&adap->auxdev);
	if (ret < 0) {
		auxiliary_device_uninit(&adap->auxdev);
		return ret;
	}

	mutex_lock(&bus->mutex);
	list_add(&adap->list, &bus->devices);
	mutex_unlock(&bus->mutex);

	return 0;
}

void fbiob_auxbus_destroy(struct fbiob_aux_bus *bus)
{
	struct fbiob_aux_adapter *adap, *tmp;

	mutex_lock(&bus->mutex);
	list_for_each_entry_safe(adap, tmp, &bus->devices, list) {
		list_del(&adap->list);
		auxiliary_device_delete(&adap->auxdev);
		auxiliary_device_uninit(&adap->auxdev);
	}
	mutex_unlock(&bus->mutex);

	mutex_destroy(&bus->mutex);
}

int fbiob_auxbus_init(struct fbiob_aux_bus *bus, struct device *parent)
{
	bus->parent = parent;
	mutex_init(&bus->mutex);
	INIT_LIST_HEAD(&bus->devices);

	return 0;
}
