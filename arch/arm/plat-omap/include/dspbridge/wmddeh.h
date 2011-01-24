/*
 * wmddeh.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Defines upper edge DEH functions required by all WMD/WCD driver
 * interface tables.
 *
 * Notes:
 *   Function comment headers reside with the function typedefs in wmd.h.
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

#ifndef WMDDEH_
#define WMDDEH_

#include <dspbridge/devdefs.h>

#include <dspbridge/dehdefs.h>

	extern DSP_STATUS WMD_DEH_Create(OUT struct DEH_MGR **phDehMgr,
					 struct DEV_OBJECT *hDevObject);

	extern DSP_STATUS WMD_DEH_Destroy(struct DEH_MGR *hDehMgr);

	extern DSP_STATUS WMD_DEH_GetInfo(struct DEH_MGR *hDehMgr,
					  struct DSP_ERRORINFO *pErrInfo);

	extern DSP_STATUS WMD_DEH_RegisterNotify(struct DEH_MGR *hDehMgr,
						 u32 uEventMask,
						 u32 uNotifyType,
						 struct DSP_NOTIFICATION
						 *hNotification);

	extern void WMD_DEH_Notify(struct DEH_MGR *hDehMgr,
				   u32 ulEventMask, u32 dwErrInfo);

	extern void WMD_DEH_ReleaseDummyMem(void);
#endif				/* WMDDEH_ */
