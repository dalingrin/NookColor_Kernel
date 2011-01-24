/*
 * include tmg120.h - platform data structure for TPK sensor
 *
 * Copyright (C) 2010 Inventec, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_TMG120_H
#define _LINUX_TMG120_H

#define CYPRESS_CY8CTMG120_NAME "Cypress-CY8CTMG120"
#define CYPRESS_CY8CTMG120_IRQ 	99

enum {
	TMG120_FLIP_X = 1UL << 0,
	TMG120_FLIP_Y = 1UL << 1,
	TMG120_SWAP_XY = 1UL << 2,
	TMG120_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

struct tmg120_platform_data {
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	int (*power)(int on);	/* Only valid in first array entry */
	uint32_t flags;
	uint32_t irqflags;
	uint32_t inactive_left; /* 0x10000 = screen width */
	uint32_t inactive_right; /* 0x10000 = screen width */
	uint32_t inactive_top; /* 0x10000 = screen height */
	uint32_t inactive_bottom; /* 0x10000 = screen height */
	uint32_t snap_left_on; /* 0x10000 = screen width */
	uint32_t snap_left_off; /* 0x10000 = screen width */
	uint32_t snap_right_on; /* 0x10000 = screen width */
	uint32_t snap_right_off; /* 0x10000 = screen width */
	uint32_t snap_top_on; /* 0x10000 = screen height */
	uint32_t snap_top_off; /* 0x10000 = screen height */
	uint32_t snap_bottom_on; /* 0x10000 = screen height */
	uint32_t snap_bottom_off; /* 0x10000 = screen height */
	uint32_t fuzz_x; /* 0x10000 = screen width */
	uint32_t fuzz_y; /* 0x10000 = screen height */
	int fuzz_p;
	int fuzz_w;
	uint32_t invoked_by;
};


//#define TMG120_DEBUG
#ifdef TMG120_DEBUG
#define tmg120_dump(fmt, args...)		printk(fmt, ##args)
#else
#define tmg120_dump(fmt, args...)		/* not debugging: nothing */
#endif
#endif /* _LINUX_TMG120 */
