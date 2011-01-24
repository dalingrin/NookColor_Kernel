/*
 * chnl.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * WCD channel interface: multiplexes data streams through the single
 * physical link managed by a Bridge mini-driver.
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
#include <dspbridge/sync.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/proc.h>
#include <dspbridge/dev.h>

/*  ----------------------------------- Others */
#include <dspbridge/chnlpriv.h>
#include <chnlobj.h>

/*  ----------------------------------- This */
#include <dspbridge/chnl.h>

/*  ----------------------------------- Globals */
static u32 cRefs;
#if GT_TRACE
static struct GT_Mask CHNL_DebugMask = { NULL, NULL };	/* WCD CHNL Mask */
#endif



/*
 *  ======== CHNL_Create ========
 *  Purpose:
 *      Create a channel manager object, responsible for opening new channels
 *      and closing old ones for a given 'Bridge board.
 */
DSP_STATUS CHNL_Create(OUT struct CHNL_MGR **phChnlMgr,
		       struct DEV_OBJECT *hDevObject,
		       IN CONST struct CHNL_MGRATTRS *pMgrAttrs)
{
	DSP_STATUS status;
	struct CHNL_MGR *hChnlMgr;
	struct CHNL_MGR_ *pChnlMgr = NULL;

	DBC_Require(cRefs > 0);
	DBC_Require(phChnlMgr != NULL);
	DBC_Require(pMgrAttrs != NULL);

	*phChnlMgr = NULL;

	/* Validate args: */
	if ((0 < pMgrAttrs->cChannels) &&
	   (pMgrAttrs->cChannels <= CHNL_MAXCHANNELS))
		status = DSP_SOK;
	else if (pMgrAttrs->cChannels == 0)
		status = DSP_EINVALIDARG;
	else
		status = CHNL_E_MAXCHANNELS;

	if (pMgrAttrs->uWordSize == 0)
		status = CHNL_E_INVALIDWORDSIZE;

	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetChnlMgr(hDevObject, &hChnlMgr);
		if (DSP_SUCCEEDED(status) && hChnlMgr != NULL)
			status = CHNL_E_MGREXISTS;

	}

	if (DSP_SUCCEEDED(status)) {
		struct WMD_DRV_INTERFACE *pIntfFxns;
		status = DEV_GetIntfFxns(hDevObject, &pIntfFxns);
		if (pIntfFxns) {
			/* Let WMD channel module finish the create */
			status = (*pIntfFxns->pfnChnlCreate)(&hChnlMgr,
						hDevObject, pMgrAttrs);
		}
		if (DSP_SUCCEEDED(status)) {
			/* Fill in WCD channel module's fields of the
			 * CHNL_MGR structure */
			pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
			pChnlMgr->pIntfFxns = pIntfFxns;
			/* Finally, return the new channel manager handle: */
			*phChnlMgr = hChnlMgr;
		}
	}

	DBC_Ensure(DSP_FAILED(status) || CHNL_IsValidMgr(pChnlMgr));

	return status;
}

/*
 *  ======== CHNL_Destroy ========
 *  Purpose:
 *      Close all open channels, and destroy the channel manager.
 */
DSP_STATUS CHNL_Destroy(struct CHNL_MGR *hChnlMgr)
{
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	DSP_STATUS status;

	DBC_Require(cRefs > 0);

	if (CHNL_IsValidMgr(pChnlMgr)) {
		pIntfFxns = pChnlMgr->pIntfFxns;
		/* Let WMD channel module destroy the CHNL_MGR: */
		status = (*pIntfFxns->pfnChnlDestroy)(hChnlMgr);
	} else {
		status = DSP_EHANDLE;
	}

	DBC_Ensure(DSP_FAILED(status) || !CHNL_IsValidMgr(pChnlMgr));

	return status;
}

/*
 *  ======== CHNL_Exit ========
 *  Purpose:
 *      Discontinue usage of the CHNL module.
 */
void CHNL_Exit(void)
{
	DBC_Require(cRefs > 0);

	cRefs--;

	DBC_Ensure(cRefs >= 0);
}


/*
 *  ======== CHNL_Init ========
 *  Purpose:
 *      Initialize the CHNL module's private state.
 */
bool CHNL_Init(void)
{
	bool fRetval = true;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		DBC_Assert(!CHNL_DebugMask.flags);
		GT_create(&CHNL_DebugMask, "CH");   /* "CH" for CHannel */
	}

	if (fRetval)
		cRefs++;

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	return fRetval;
}


