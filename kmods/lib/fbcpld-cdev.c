// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#include "fbcpld-cdev.h"

#define ATTR_TO_ENTRY(_a)	\
		container_of(_a, struct fbcpld_dyn_entry, dev_attr)

/*
 * Firmware version registers are consistent across all FBOSS CPLDs.
 */
#define CPLD_MAJOR_VER_REG	1
#define CPLD_MINOR_VER_REG	2
#define CPLD_SUB_VER_REG	3
#define CPLD_MAJOR_VER_MASK	0x7F

/*
 * Static fw_ver sysfs attribute - available on all FBOSS CPLDs.
 */
static ssize_t fw_ver_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	int ret;
	struct regmap *regmap;
	u32 major_ver, minor_ver, sub_ver;

	regmap = dev_get_regmap(dev, NULL);
	if (IS_ERR_OR_NULL(regmap))
		return -ENODEV;

	ret = regmap_read(regmap, CPLD_MAJOR_VER_REG, &major_ver);
	if (ret)
		return ret;

	ret = regmap_read(regmap, CPLD_MINOR_VER_REG, &minor_ver);
	if (ret)
		return ret;

	ret = regmap_read(regmap, CPLD_SUB_VER_REG, &sub_ver);
	if (ret)
		return ret;

	return sprintf(buf, "%u.%u.%u\n", major_ver & CPLD_MAJOR_VER_MASK,
		       (u8)minor_ver, (u8)sub_ver);
}
static DEVICE_ATTR_RO(fw_ver);

static struct attribute *fbcpld_static_attrs[] = {
	&dev_attr_fw_ver.attr,
	NULL,
};

static const struct attribute_group fbcpld_static_group = {
	.attrs = fbcpld_static_attrs,
};

/*
 * Check if an entry with the given name already exists.
 * Must be called with entries_lock held.
 */
static bool fbcpld_entry_exists(struct fbcpld_cdev_desc *cdesc, const char *name)
{
	size_t i;

	for (i = 0; i < cdesc->num_entries; i++) {
		if (strncmp(cdesc->entries[i].attr_cfg.name, name, NAME_MAX) == 0)
			return true;
	}
	return false;
}

/*
 * sysfs show callback for dynamically created attributes.
 */
static ssize_t fbcpld_dyn_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;
	char data[32] = {};
	u32 val, mask, reg_val;
	struct fbcpld_dyn_entry *entry = ATTR_TO_ENTRY(attr);
	struct fbcpld_sysfs_attr *cfg = &entry->attr_cfg;
	struct regmap *regmap = dev_get_regmap(dev, NULL);

	if (IS_ERR_OR_NULL(regmap))
		return -ENODEV;

	ret = regmap_read(regmap, cfg->reg_addr, &reg_val);
	if (ret)
		return ret;

	mask = GENMASK(cfg->num_bits - 1, 0);
	if (cfg->flags & FBCPLD_FLAG_VAL_NEGATE)
		val = (~reg_val >> cfg->bit_offset) & mask;
	else
		val = (reg_val >> cfg->bit_offset) & mask;

	if (cfg->flags & FBCPLD_FLAG_SHOW_DEC)
		snprintf(data, sizeof(data), "%u", val);
	else
		snprintf(data, sizeof(data), "0x%x", val);

	if (cfg->flags & FBCPLD_FLAG_SHOW_NOTES)
		return sprintf(buf, "%s\n\n"
			       "Note:\nRegister 0x%x, bit[%u:%u]\n%s\n",
			       data, cfg->reg_addr,
			       cfg->bit_offset + cfg->num_bits - 1,
			       cfg->bit_offset,
			       cfg->help_str[0] ? cfg->help_str : "");
	else
		return sprintf(buf, "%s\n", data);
}

/*
 * sysfs store callback for dynamically created attributes.
 */
static ssize_t fbcpld_dyn_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	u32 input, mask;
	struct fbcpld_dyn_entry *entry = ATTR_TO_ENTRY(attr);
	struct fbcpld_sysfs_attr *cfg = &entry->attr_cfg;
	struct regmap *regmap = dev_get_regmap(dev, NULL);

	if (IS_ERR_OR_NULL(regmap))
		return -ENODEV;

	if (kstrtou32(buf, 0, &input))
		return -EINVAL;

	input &= GENMASK(cfg->num_bits - 1, 0);

	if (!(cfg->mode & S_IRUSR)) {
		ret = regmap_write(regmap, cfg->reg_addr,
				   input << cfg->bit_offset);
	} else {
		mask = GENMASK(cfg->bit_offset + cfg->num_bits - 1,
			       cfg->bit_offset);
		ret = regmap_update_bits(regmap, cfg->reg_addr, mask,
					 input << cfg->bit_offset);
	}
	if (ret)
		return ret;

	if (cfg->flags & FBCPLD_FLAG_LOG_WRITE)
		dev_info(dev, "write %#x to %s by pid %ld (cmd=%s)\n",
			 input, cfg->name, (long)current->pid, current->comm);

	return count;
}

/*
 * Initialize a dynamic entry from the userspace config.
 */
static void fbcpld_init_entry(struct fbcpld_dyn_entry *entry,
			      const struct fbcpld_sysfs_attr *cfg)
{
	entry->attr_cfg = *cfg;

	sysfs_attr_init(&entry->dev_attr.attr);
	entry->dev_attr.attr.name = entry->attr_cfg.name;
	entry->dev_attr.attr.mode = cfg->mode;

	if (cfg->mode & S_IRUSR)
		entry->dev_attr.show = fbcpld_dyn_show;
	if (cfg->mode & S_IWUSR)
		entry->dev_attr.store = fbcpld_dyn_store;
}

/*
 * Validate an attribute configuration.
 */
static int fbcpld_validate_attr(const struct fbcpld_sysfs_attr *cfg)
{
	mode_t mode_mask = S_IRUSR | S_IWUSR;

	if (cfg->name[0] == '\0')
		return -EINVAL;

	if ((cfg->mode & mode_mask) == 0)
		return -EINVAL;

	if (cfg->reg_addr > 0xFF)
		return -EINVAL;

	if (cfg->bit_offset >= 8)
		return -EINVAL;

	if (cfg->num_bits == 0 || cfg->num_bits > 8)
		return -EINVAL;

	if ((cfg->bit_offset + cfg->num_bits) > 8)
		return -EINVAL;

	return 0;
}

/*
 * FBCPLD_IOC_SYSFS_CREATE handler.
 */
static long fbcpld_sysfs_create(struct fbcpld_cdev_desc *cdesc,
				void __user *userp)
{
	struct fbcpld_ioctl_create_request *req;
	int i, ret = 0;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	if (copy_from_user(req, userp, sizeof(*req))) {
		kfree(req);
		return -EFAULT;
	}

	if (req->num_attrs == 0 || req->num_attrs > FBCPLD_MAX_ATTRS_PER_IOCTL) {
		kfree(req);
		return -EINVAL;
	}

	mutex_lock(&cdesc->entries_lock);

	for (i = 0; i < req->num_attrs; i++) {
		struct fbcpld_sysfs_attr *cfg = &req->attrs[i];
		struct fbcpld_dyn_entry *entry;

		/* Validate configuration */
		ret = fbcpld_validate_attr(cfg);
		if (ret) {
			dev_warn(cdesc->dev, "invalid attr config: %s\n",
				 cfg->name);
			continue;
		}

		/* Skip if name already exists */
		if (fbcpld_entry_exists(cdesc, cfg->name)) {
			dev_err(cdesc->dev, "attr %s already exists\n",
				cfg->name);
			continue;
		}

		/* Check if array is full */
		if (cdesc->num_entries >= FBCPLD_MAX_ATTRS) {
			dev_err(cdesc->dev, "max attrs (%d) reached\n",
				FBCPLD_MAX_ATTRS);
			ret = -ENOSPC;
			break;
		}

		entry = &cdesc->entries[cdesc->num_entries];
		fbcpld_init_entry(entry, cfg);

		ret = sysfs_create_file(&cdesc->dev->kobj,
					&entry->dev_attr.attr);
		if (ret) {
			dev_err(cdesc->dev, "failed to create sysfs file: %s\n",
				cfg->name);
			break;
		}

		cdesc->num_entries++;
	}

	mutex_unlock(&cdesc->entries_lock);
	kfree(req);

	return ret;
}

/*
 * FBCPLD_IOC_SYSFS_DESTROY_ALL handler.
 */
static long fbcpld_sysfs_destroy_all(struct fbcpld_cdev_desc *cdesc)
{
	size_t i, count;

	mutex_lock(&cdesc->entries_lock);
	count = cdesc->num_entries;
	cdesc->num_entries = 0;
	mutex_unlock(&cdesc->entries_lock);

	/*
	 * Remove files without holding the lock to avoid deadlock.
	 * sysfs_remove_file() waits for active sysfs operations,
	 * which may be waiting on entries_lock.
	 */
	for (i = 0; i < count; i++)
		sysfs_remove_file(&cdesc->dev->kobj,
				  &cdesc->entries[i].dev_attr.attr);

	return 0;
}

static long fbcpld_cdev_ioctl(struct file *file,
			      unsigned int cmd,
			      unsigned long param)
{
	struct fbcpld_cdev_desc *cdesc = container_of(file->private_data,
						      struct fbcpld_cdev_desc,
						      miscdev);
	void __user *userp = (void __user *)param;

	switch (cmd) {
	case FBCPLD_IOC_SYSFS_CREATE:
		dev_info(cdesc->dev, "SYSFS_CREATE: pid=%d (%s)\n",
			 current->pid, current->comm);
		return fbcpld_sysfs_create(cdesc, userp);

	case FBCPLD_IOC_SYSFS_DESTROY_ALL:
		dev_info(cdesc->dev, "SYSFS_DESTROY_ALL: pid=%d (%s)\n",
			 current->pid, current->comm);
		return fbcpld_sysfs_destroy_all(cdesc);

	default:
		return -ENOTTY;
	}
}

static const struct file_operations fbcpld_cdev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fbcpld_cdev_ioctl,
};

static void fbcpld_cdev_cleanup(void *data)
{
	struct fbcpld_cdev_desc *cdesc = data;

	fbcpld_sysfs_destroy_all(cdesc);
	misc_deregister(&cdesc->miscdev);
}

int fbcpld_cdev_init(struct i2c_client *client)
{
	struct fbcpld_cdev_desc *cdesc;
	int ret;

	cdesc = devm_kzalloc(&client->dev, sizeof(*cdesc), GFP_KERNEL);
	if (!cdesc)
		return -ENOMEM;

	cdesc->client = client;
	cdesc->dev = &client->dev;

	mutex_init(&cdesc->entries_lock);
	cdesc->num_entries = 0;

	/* Device name: fbcpld-<bus>-<addr> */
	snprintf(cdesc->name, sizeof(cdesc->name), "fbcpld-%d-%04x",
		 client->adapter->nr, client->addr);

	cdesc->miscdev.minor = MISC_DYNAMIC_MINOR;
	cdesc->miscdev.name = cdesc->name;
	cdesc->miscdev.fops = &fbcpld_cdev_fops;
	cdesc->miscdev.parent = cdesc->dev;

	i2c_set_clientdata(client, cdesc);

	ret = misc_register(&cdesc->miscdev);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(cdesc->dev, fbcpld_cdev_cleanup, cdesc);
	if (ret)
		return ret;

	/* Create static sysfs attributes (fw_ver) */
	ret = devm_device_add_group(cdesc->dev, &fbcpld_static_group);
	if (ret)
		return ret;

	dev_info(cdesc->dev, "registered %s\n", cdesc->name);
	return 0;
}
