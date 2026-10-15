/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (c) 2026 Nexthop Systems Inc. */

#ifndef _FBOSS_PCI_IDS_H_
#define _FBOSS_PCI_IDS_H_

#include <linux/pci.h>
#include <linux/pci_ids.h>

#define PCI_VENDOR_ID_META	0x1d9b
#define PCI_DEVICE_ID_FBIOB	0x0011

#define FBOSS_PCI_ID_ENTRY(vendor, device)	{ PCI_DEVICE((vendor), (device)) }

/* One header per vendor; each defines <VENDOR>_FBOSS_IOB_PCI_IDS(MACRO__). */
#include "pci-ids/nexthop.h"

#define FBOSS_IOB_PCI_IDS(MACRO__)					\
	MACRO__(PCI_VENDOR_ID_META, PCI_DEVICE_ID_FBIOB),		\
	NEXTHOP_FBOSS_IOB_PCI_IDS(MACRO__)

#endif /* _FBOSS_PCI_IDS_H_ */
