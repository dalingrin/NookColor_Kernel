/*
 * _tiomap_mmu.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Definitions and types for the DSP MMU modules.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _TIOMAP_MMU_
#define _TIOMAP_MMU_

#include "_tiomap.h"

/*
 *  ======== configureDspMmu ========
 *
 *  Make DSP MMu page table entries.
 *  Note: Not utilizing Coarse / Fine page tables.
 *  SECTION = 1MB, LARGE_PAGE = 64KB, SMALL_PAGE = 4KB, TINY_PAGE = 1KB.
 *  DSP Byte address 0x40_0000 is word addr 0x20_0000.
 */
extern void configureDspMmu(struct WMD_DEV_CONTEXT *pDevContext,
			    u32 dataBasePhys,
			    u32 dspBaseVirt,
			    u32 sizeInBytes,
			    s32 nEntryStart,
			    enum HW_Endianism_t endianism,
			    enum HW_ElementSize_t elemSize,
			    enum HW_MMUMixedSize_t mixedSize);

#endif				/* _TIOMAP_MMU_ */
