/*
 * io.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * IO manager interface: Manages IO between CHNL and MSG.
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

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/mem.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/dev.h>

/*  ----------------------------------- This */
#include <ioobj.h>
#include <dspbridge/iodefs.h>
#include <dspbridge/io.h>

/*  ----------------------------------- Globals */
static u32 cRefs;

#if GT_TRACE
static struct GT_Mask IO_DebugMask = { NULL, NULL };	/* WCD IO Mask */
#endif

/*
 *  ======== IO_Create ========
 *  Purpose:
 *      Create an IO manager object, responsible for managing IO between
 *      CHNL and MSG
 */
DSP_STATUS IO_Create(OUT struct IO_MGR **phIOMgr, struct DEV_OBJECT *hDevObject,
		    IN CONST struct IO_ATTRS *pMgrAttrs)
{
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct IO_MGR *hIOMgr = NULL;
	struct IO_MGR_ *pIOMgr = NULL;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	DBC_Require(phIOMgr != NULL);
	DBC_Require(pMgrAttrs != NULL);

	*phIOMgr = NULL;

	/* A memory base of 0 implies no memory base:  */
	if ((pMgrAttrs->dwSMBase != 0) && (pMgrAttrs->uSMLength == 0))
		status = CHNL_E_INVALIDMEMBASE;

	if (pMgrAttrs->uWordSize == 0)
		status = CHNL_E_INVALIDWORDSIZE;

	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetIntfFxns(hDevObject, &pIntfFxns);

		if (pIntfFxns) {
			/* Let WMD channel module finish the create */
			status = (*pIntfFxns->pfnIOCreate)(&hIOMgr, hDevObject,
					pMgrAttrs);
		}

		if (DSP_SUCCEEDED(status)) {
			pIOMgr = (struct IO_MGR_ *) hIOMgr;
			pIOMgr->pIntfFxns = pIntfFxns;
			pIOMgr->hDevObject = hDevObject;

			/* Return the new channel manager handle: */
			*phIOMgr = hIOMgr;
		}
	}

	return status;
}

/*
 *  ======== IO_Destroy ========
 *  Purpose:
 *      Delete IO manager.
 */
DSP_STATUS IO_Destroy(struct IO_MGR *hIOMgr)
{
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct IO_MGR_ *pIOMgr = (struct IO_MGR_ *)hIOMgr;
	DSP_STATUS status;

	DBC_Require(cRefs > 0);

	pIntfFxns = pIOMgr->pIntfFxns;

	/* Let WMD channel module destroy the IO_MGR: */
	status = (*pIntfFxns->pfnIODestroy) (hIOMgr);

	return status;
}

/*
 *  ======== IO_Exit ========
 *  Purpose:
 *      Discontinue usage of the IO module.
 */
void IO_Exit(void)
{
	DBC_Require(cRefs > 0);

	cRefs--;

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== IO_Init ========
 *  Purpose:
 *      Initialize the IO module's private state.
 */
bool IO_Init(void)
{
	bool fRetval = true;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		DBC_Assert(!IO_DebugMask.flags);
		GT_create(&IO_DebugMask, "IO");	/* "IO" for IO */
	}

	if (fRetval)
		cRefs++;

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	return fRetval;
}
