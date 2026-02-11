// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "regbit-sysfs.h"

#define MMIO_REG_VAL_BITS	32
#define MMIO_REG_ADDR_BITS	32
#define MMIO_REG_OFFSET_MAX	0x7FFFF
#define I2C_REG_VAL_BITS	8
#define I2C_REG_ADDR_BITS	8
#define I2C_REG_OFFSET_MAX	0xFF

/*
 * The firmware version registers are consistent in all the FBOSS CPLDs.
 */
#define CPLD_MAJOR_VER_REG	1
#define CPLD_MINOR_VER_REG	2
#define CPLD_SUB_VER_REG	3
#define CPLD_MAJOR_VER_MASK	0x7F

#define ATTR_TO_ENTRY(_a)	\
		container_of(_a, struct regbit_sysfs_entry, dev_attr)

struct regbit_sysfs_entry {
	struct mutex lock;
	struct device_attribute dev_attr;
	struct regbit_sysfs_config config;
};

struct regbit_sysfs_handle {
	struct device *dev;
	size_t num_entries;
	const struct regmap_config *regmap_cfg;
	struct regbit_sysfs_entry *entries;
	struct attribute_group attr_group;
};

/*
 * Below are the default regmap_config for FBOSS (MMIO) FPGAs and (I2C)
 * CPLDs.
 * TODO: we may allow callers to customize "regmap_config" if it becomes
 * necessary in the future.
 */
static const struct regmap_config mmio_regmap_cfg = {
	.reg_bits = MMIO_REG_ADDR_BITS,
	.reg_stride = 4,
	.val_bits = MMIO_REG_VAL_BITS,
	.max_register = MMIO_REG_OFFSET_MAX,
};

static const struct regmap_config i2c_regmap_cfg = {
	.reg_bits = I2C_REG_ADDR_BITS,
	.val_bits = I2C_REG_VAL_BITS,
	.max_register = I2C_REG_OFFSET_MAX,
};

/*
 * Helper function to parse/print CPLD firmware version.
 */
ssize_t cpld_fw_ver_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	int ret;
	struct regmap *regmap;
	u32 major_ver, minor_ver, sub_ver;

	regmap = dev_get_regmap(dev, NULL);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

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

static ssize_t regbit_sysfs_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int ret;
	char data[32] = {};
	struct regmap *regmap;
	u32 val, mask, reg_val;
	struct regbit_sysfs_entry *entry = ATTR_TO_ENTRY(attr);
	struct regbit_sysfs_config *config = &entry->config;

	regmap = dev_get_regmap(dev, NULL);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = regmap_read(regmap, config->reg_addr, &reg_val);
	if (ret)
		return ret;

	mask = GENMASK(config->num_bits - 1, 0);
	if (config->flags & RBS_FLAG_VAL_NEGATE)
		val = (~reg_val >> config->bit_offset) & mask;
	else
		val = (reg_val >> config->bit_offset) & mask;

	if (config->flags & RBS_FLAG_SHOW_DEC)
		snprintf(data, sizeof(data), "%u", val);
	else
		snprintf(data, sizeof(data), "0x%x", val);

	if (config->flags & RBS_FLAG_SHOW_NOTES)
		return sprintf(buf, "%s\n\n"
				"Note:\nRegister 0x%x, bit[%u:%u]\n%s\n",
				data, config->reg_addr,
				config->bit_offset + config->num_bits - 1,
				config->bit_offset,
				(config->help_str) ? config->help_str : "");
	else
		return sprintf(buf, "%s\n", data);
}

static ssize_t regbit_sysfs_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret = 0;
	u32 input, val, mask;
	struct regmap *regmap;
	struct regbit_sysfs_entry *entry = ATTR_TO_ENTRY(attr);
	struct regbit_sysfs_config *config = &entry->config;

	regmap = dev_get_regmap(dev, NULL);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* parse the buffer */
	if (kstrtou32(buf, 0, &input))
		return -EINVAL;

	mutex_lock(&entry->lock);

	mask = GENMASK(config->num_bits - 1, 0);
	input &= mask;

	ret = regmap_read(regmap, config->reg_addr, &val);
	if (ret)
		goto exit;

	/* Clear the original bits before setting the new value. */
	mask = GENMASK(config->bit_offset + config->num_bits - 1,
			config->bit_offset);
	val &= ~mask;
	val |= (input << config->bit_offset);

	ret = regmap_write(regmap, config->reg_addr, val);

exit:
	mutex_unlock(&entry->lock);
	if (ret)
		return ret;

	if (config->flags & RBS_FLAG_LOG_WRITE)
		dev_info(dev, "write %#x to %s by pid %ld (cmd=%s)\n",
			input, config->name, (long)current->pid,
			current->comm);

	return count;
}

static int regbit_config_check(const struct regbit_sysfs_config *config,
				const struct regmap_config *regmap_cfg)
{
	mode_t mode_mask = S_IRUSR | S_IWUSR;

	if ((config->mode & mode_mask) == 0)
		return -EINVAL;

	if (config->reg_addr > regmap_cfg->max_register)
		return -EINVAL;

	if ((config->bit_offset >= regmap_cfg->val_bits) ||
	    ((config->bit_offset + config->num_bits) > regmap_cfg->val_bits))
		return -EINVAL;

	return 0;
}

static void regbit_handle_free(struct regbit_sysfs_handle *handle)
{
	if (handle->entries) {
		kfree(handle->entries);
		handle->entries = NULL;
	}
	if (handle->attr_group.attrs) {
		kfree(handle->attr_group.attrs);
		handle->attr_group.attrs = NULL;
	}
	kfree(handle);
}

static struct regbit_sysfs_handle *regbit_handle_alloc(struct device *dev,
							size_t num_entries)
{
	struct attribute **attrs;
	struct regbit_sysfs_entry *entries;
	struct regbit_sysfs_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return NULL;

	/* Allocate one more entry for the last NULL <attr>. */
	attrs = kzalloc((num_entries + 1) * sizeof(*attrs), GFP_KERNEL);
	if (!attrs) {
		kfree(handle);
		return NULL;
	}

	entries = kzalloc(num_entries * sizeof(*entries), GFP_KERNEL);
	if (!entries) {
		kfree(attrs);
		kfree(handle);
		return NULL;
	}

	handle->dev = dev;
	handle->num_entries = num_entries;
	handle->entries = entries;
	handle->attr_group.attrs = attrs;
	return handle;
}

static void regbit_attr_uninit(struct regbit_sysfs_handle *handle)
{
	int i;
	struct regbit_sysfs_entry *entry;

	for (i = 0; i < handle->num_entries; i++) {
		entry = &handle->entries[i];
		if (entry->dev_attr.attr.name != NULL)
			mutex_destroy(&entry->lock);
	}
}

static int regbit_attr_init(struct regbit_sysfs_handle *handle,
			    const struct regbit_sysfs_config *configs)
{
	int i;
	int ret = 0;
	struct regbit_sysfs_entry *entry;
	const struct regbit_sysfs_config *config;

	for (i = 0; i < handle->num_entries; i++) {
		config = &configs[i];
		entry = &handle->entries[i];

		ret = regbit_config_check(config, handle->regmap_cfg);
		if (ret) {
			regbit_attr_uninit(handle);
			break;
		}

		mutex_init(&entry->lock);
		entry->config = *config;
		sysfs_attr_init(&entry->dev_attr.attr);
		entry->dev_attr.attr.name = entry->config.name;
		entry->dev_attr.attr.mode = entry->config.mode;
		if (config->mode & S_IRUSR) {
			if (config->show_func)
				entry->dev_attr.show = config->show_func;
			else
				entry->dev_attr.show = regbit_sysfs_show;
		}
		if (config->mode & S_IWUSR) {
			if (config->store_func)
				entry->dev_attr.store = config->store_func;
			else
				entry->dev_attr.store = regbit_sysfs_store;
		}

		handle->attr_group.attrs[i] = &entry->dev_attr.attr;
	}

	return ret;
}

static void regbit_sysfs_uninit(void *data)
{
	struct regbit_sysfs_handle *handle = data;

	sysfs_remove_group(&handle->dev->kobj, &handle->attr_group);
	regbit_attr_uninit(handle);
	regbit_handle_free(handle);
}

static int regbit_sysfs_init_common(struct device *dev,
				const struct regbit_sysfs_config *configs,
				size_t num_configs,
				const struct regmap_config *regmap_cfg)
{
	int ret;
	struct regbit_sysfs_handle *handle;

	handle = regbit_handle_alloc(dev, num_configs);
	if (!handle)
		return -ENOMEM;
	handle->regmap_cfg = regmap_cfg;

	ret = regbit_attr_init(handle, configs);
	if (ret)
		goto err_attr_init;

	ret = sysfs_create_group(&dev->kobj, &handle->attr_group);
	if (ret)
		goto err_exit;

	return devm_add_action_or_reset(dev, regbit_sysfs_uninit, handle);

err_exit:
	regbit_attr_uninit(handle);
err_attr_init:
	regbit_handle_free(handle);
	return ret;
}

int regbit_sysfs_init_mmio(struct device *dev,
			void __iomem *mmio_base,
			const struct regbit_sysfs_config *configs,
			size_t num_configs)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_mmio(dev, mmio_base, &mmio_regmap_cfg);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return regbit_sysfs_init_common(dev, configs, num_configs,
					&mmio_regmap_cfg);
}

int regbit_sysfs_init_i2c(struct device *dev,
			const struct regbit_sysfs_config *configs,
			size_t num_configs)
{
	struct regmap *regmap;
	struct i2c_client *client = to_i2c_client(dev);

	regmap = devm_regmap_init_i2c(client, &i2c_regmap_cfg);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return regbit_sysfs_init_common(dev, configs, num_configs,
					&i2c_regmap_cfg);
}

MODULE_AUTHOR("Tao Ren <taoren@meta.com>");
MODULE_DESCRIPTION("Library for mapping register bits to sysfs files");
MODULE_LICENSE("GPL");
