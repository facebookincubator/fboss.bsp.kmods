/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __FBOSS_DUMMY_KMOD_H__
#define __FBOSS_DUMMY_KMOD_H__

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#define create_dummy_kmod(name)				\
static int __init name##_dummy_init(void)		\
{							\
	return 0;					\
}							\
static void __exit name##_dummy_exit(void)		\
{							\
}							\
module_init(name##_dummy_init);				\
module_exit(name##_dummy_exit);				\
MODULE_AUTHOR("Tao Ren <taoren@meta.com>");		\
MODULE_DESCRIPTION("Dummy (empty) " #name " module");	\
MODULE_LICENSE("GPL v2")

#endif /* __FBOSS_DUMMY_KMOD_H__ */
