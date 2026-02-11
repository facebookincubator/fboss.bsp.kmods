/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef __FBIOB_MMIO_H__
#define __FBIOB_MMIO_H__

#include <linux/io.h>

/*
 * The FBOSS IOB/DOM FPGAs only expect 32-bit (4-byte aligned)
 * register access: byte addressing is not supported as of now.
 */
#define FBIOB_IO_SIZE		4

/*
 * Helper functions for bulk I/O.
 * NOTE: only 32-bits reads/writes are expected for the IOB and DOM FPGAs.
 */
static inline void fbiob_bulk_read(void __iomem *mmio, void *buf, size_t size)
{
	u32 val;
	u32 *wptr = buf;
	u32 words = size / FBIOB_IO_SIZE;
	u32 remainder = size % FBIOB_IO_SIZE;

	while (words--) {
		*wptr = readl(mmio);
		wptr++;
		mmio += sizeof(*wptr);
	}

	if (remainder > 0) {
		val = readl(mmio);
		memcpy(wptr, &val, remainder);
	}
}

static inline void fbiob_bulk_write(void __iomem *mmio,
				    const void *buf,
				    size_t size)
{
	u32 val = 0;
	const u32 *wptr = buf;
	u32 words = size / FBIOB_IO_SIZE;
	u32 remainder = size % FBIOB_IO_SIZE;

	while (words--) {
		writel(*wptr, mmio);
		wptr++;
		mmio += sizeof(*wptr);
	}

	if (remainder > 0) {
		memcpy(&val, wptr, remainder);
		writel(val, mmio);
	}
}

#endif /* __FBIOB_MMIO_H__ */
