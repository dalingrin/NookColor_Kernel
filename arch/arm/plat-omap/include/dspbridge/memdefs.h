/*
 * memdefs.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Global MEM constants and types, shared between WCD and WMD.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef MEMDEFS_
#define MEMDEFS_

/* Memory Pool Attributes: */
	enum MEM_POOLATTRS {
		MEM_PAGED = 0,
		MEM_NONPAGED = 1,
		MEM_LARGEVIRTMEM = 2
	} ;

/*
 *  MEM_VIRTUALSEGID is used by Node & Strm to access virtual address space in
 *  the correct client process context.
 */
#define MEM_SETVIRTUALSEGID     0x10000000
#define MEM_GETVIRTUALSEGID     0x20000000
#define MEM_MASKVIRTUALSEGID    (MEM_SETVIRTUALSEGID | MEM_GETVIRTUALSEGID)

#endif				/* MEMDEFS_ */
