/*
 * dispdefs.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Global DISP constants and types, shared by PROCESSOR, NODE, and DISP.
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

#ifndef DISPDEFS_
#define DISPDEFS_

	struct DISP_OBJECT;

/* Node Dispatcher attributes */
	struct DISP_ATTRS {
		u32 ulChnlOffset; /* Offset of channel ids reserved for RMS */
		/* Size of buffer for sending data to RMS */
		u32 ulChnlBufSize;
		int procFamily;		/* eg, 5000 */
		int procType;		/* eg, 5510 */
		HANDLE hReserved1;	/* Reserved for future use. */
		u32 hReserved2;	/* Reserved for future use. */
	} ;

#endif				/* DISPDEFS_ */
