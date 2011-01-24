/*
 * _deh.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Private header for DEH module.
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

#ifndef _DEH_
#define _DEH_

#include <dspbridge/ntfy.h>
#include <dspbridge/wmd.h>

#define SIGNATURE       0x5f484544	/* "DEH_" backwards */

/* DEH Manager: only one created per board: */
struct DEH_MGR {
	u32 dwSignature;	/* Used for object validation.  */
	struct WMD_DEV_CONTEXT *hWmdContext;	/* WMD device context. */
	struct NTFY_OBJECT *hNtfy;	/* NTFY object                  */
	struct DSP_ERRORINFO errInfo;	/* DSP exception info.          */

	/* MMU Fault DPC */
	struct tasklet_struct dpc_tasklet;
} ;

#endif				/* _DEH_ */
