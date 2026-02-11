/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __REGBIT_SYSFS_H
#define __REGBIT_SYSFS_H

#include <linux/device.h>
#include <linux/limits.h>
#include <linux/stat.h>
#include <linux/types.h>

#define REGBIT_FMODE_RO	0444
#define REGBIT_FMODE_RW	0644

struct regbit_sysfs_config {
	/*
	 * Mandatory fields.
	 */
	char name[NAME_MAX];
	mode_t mode;	/* REGBIT_FMODE_RO or REGBIT_FMODE_RW */
	u32 reg_addr;
	u32 bit_offset;
	u32 num_bits;

	/*
	 * The default show/store methods will be applied if either
	 * one is set to NULL.
	 */
	ssize_t (*show_func)(struct device *dev,
				struct device_attribute *attr,
				char *buf);
	ssize_t (*store_func)(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count);

	/*
	 * Optional help string to include more notes about the file.
	 */
	const char *help_str;

	/*
	 * Optional flags.
	 */
	unsigned int flags;
#define RBS_FLAG_LOG_WRITE	BIT(0)
#define RBS_FLAG_SHOW_NOTES	BIT(1)
#define RBS_FLAG_SHOW_DEC	BIT(2)
/*
 * Optional flags, Negate the bit value
 */
#define RBS_FLAG_VAL_NEGATE	BIT(3)
};

ssize_t cpld_fw_ver_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf);

int regbit_sysfs_init_mmio(struct device *dev,
			void __iomem *mmio_base,
			const struct regbit_sysfs_config *configs,
			size_t num_configs);
int regbit_sysfs_init_i2c(struct device *dev,
			const struct regbit_sysfs_config *configs,
			size_t num_configs);

#endif /* __REGBIT_SYSFS_H */
