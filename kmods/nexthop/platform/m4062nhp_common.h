/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 Nexthop Systems Inc. */

#ifndef _M4062NHP_COMMON_H_
#define _M4062NHP_COMMON_H_

#include <linux/types.h>

/* led_to_id[physical LED number] = transceiver id (1..128). One table serves
 * both FPGAs; they drive disjoint LED numbers. Index 0 is unused padding. */
extern const u32 m4062nhp_led_to_id[129];

/* xcvr_id_to_slot[transceiver id] = physical slot (1..128), for FPGA1's
 * xcvr_ctrl_config.id_to_slot. Index 0 is unused padding. */
extern const u32 m4062nhp_xcvr_id_to_slot[129];

#endif /* _M4062NHP_COMMON_H_ */
