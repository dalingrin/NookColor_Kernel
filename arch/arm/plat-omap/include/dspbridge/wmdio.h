/*
 * wmdio.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Declares the upper edge IO functions required by all WMD / WCD
 * driver interface tables.
 *
 * Notes:
 *   Function comment headers reside in wmd.h.
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

#ifndef WMDIO_
#define WMDIO_

#include <dspbridge/devdefs.h>
#include <dspbridge/iodefs.h>

	extern DSP_STATUS WMD_IO_Create(OUT struct IO_MGR **phIOMgr,
					struct DEV_OBJECT *hDevObject,
					IN CONST struct IO_ATTRS *pMgrAttrs);

	extern DSP_STATUS WMD_IO_Destroy(struct IO_MGR *hIOMgr);

	extern DSP_STATUS WMD_IO_OnLoaded(struct IO_MGR *hIOMgr);

	extern DSP_STATUS IVA_IO_OnLoaded(struct IO_MGR *hIOMgr);
	extern DSP_STATUS WMD_IO_GetProcLoad(IN struct IO_MGR *hIOMgr,
				OUT struct DSP_PROCLOADSTAT *pProcStat);

#endif				/* WMDIO_ */
