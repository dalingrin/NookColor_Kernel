/*
 * drv.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * DSP/BIOS Bridge resource allocation module.
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
#include <dspbridge/list.h>
#include <dspbridge/mem.h>
#include <dspbridge/reg.h>

/*  ----------------------------------- This */
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>

#include <dspbridge/node.h>
#include <dspbridge/proc.h>
#include <dspbridge/strm.h>
#include <dspbridge/nodepriv.h>
#include <dspbridge/wmdchnl.h>
#include <dspbridge/resourcecleanup.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define SIGNATURE   0x5f52474d	/* "DRV_" (in reverse) */

struct DRV_OBJECT {
	u32 dwSignature;
	struct LST_LIST *devList;
	struct LST_LIST *devNodeString;
};

/*
 *  This is the Device Extension. Named with the Prefix
 *  DRV_ since it is living in this module
 */
struct DRV_EXT {
	struct list_head link;
	char szString[MAXREGPATHLENGTH];
};

/*  ----------------------------------- Globals */
static s32 cRefs;

#if GT_TRACE
extern struct GT_Mask curTrace;
#endif

/*  ----------------------------------- Function Prototypes */
static DSP_STATUS RequestBridgeResources(u32 dwContext, s32 fRequest);
static DSP_STATUS RequestBridgeResourcesDSP(u32 dwContext, s32 fRequest);

/* GPP PROCESS CLEANUP CODE */

static DSP_STATUS DRV_ProcFreeNodeRes(HANDLE hPCtxt);
extern enum NODE_STATE NODE_GetState(HANDLE hNode);

/* Allocate and add a node resource element
* This function is called from .Node_Allocate.  */
DSP_STATUS DRV_InsertNodeResElement(HANDLE hNode, HANDLE hNodeRes,
					HANDLE hPCtxt)
{
	struct NODE_RES_OBJECT **pNodeRes = (struct NODE_RES_OBJECT **)hNodeRes;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct NODE_RES_OBJECT   *pTempNodeRes = NULL;

	*pNodeRes = (struct NODE_RES_OBJECT *)MEM_Calloc
		    (1 * sizeof(struct NODE_RES_OBJECT), MEM_PAGED);
	if (*pNodeRes == NULL)
		status = DSP_EHANDLE;

	if (DSP_SUCCEEDED(status)) {
		if (mutex_lock_interruptible(&pCtxt->node_mutex)) {
			kfree(*pNodeRes);
			return DSP_EFAIL;
		}
		(*pNodeRes)->hNode = hNode;
		if (pCtxt->pNodeList != NULL) {
			pTempNodeRes = pCtxt->pNodeList;
			while (pTempNodeRes->next != NULL)
				pTempNodeRes = pTempNodeRes->next;

			pTempNodeRes->next = *pNodeRes;
		} else {
			pCtxt->pNodeList = *pNodeRes;
		}
		mutex_unlock(&pCtxt->node_mutex);
	}

	return status;
}

/* Release all Node resources and its context
* This is called from .Node_Delete.  */
DSP_STATUS DRV_RemoveNodeResElement(HANDLE hNodeRes, HANDLE hPCtxt)
{
	struct NODE_RES_OBJECT *pNodeRes = (struct NODE_RES_OBJECT *)hNodeRes;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	struct NODE_RES_OBJECT *pTempNode;
	DSP_STATUS status = DSP_SOK;

	if (mutex_lock_interruptible(&pCtxt->node_mutex))
		return DSP_EFAIL;
	pTempNode = pCtxt->pNodeList;
	if (pTempNode == pNodeRes) {
		pCtxt->pNodeList = pNodeRes->next;
	} else {
		while (pTempNode && pTempNode->next != pNodeRes)
			pTempNode = pTempNode->next;
		if (!pTempNode)
			status = DSP_ENOTFOUND;
		else
			pTempNode->next = pNodeRes->next;
	}
	mutex_unlock(&pCtxt->node_mutex);
	kfree(pNodeRes);
	return status;
}

/* Actual Node De-Allocation */
static DSP_STATUS DRV_ProcFreeNodeRes(HANDLE hPCtxt)
{
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct NODE_RES_OBJECT *pNodeList = NULL;
	struct NODE_RES_OBJECT *pNodeRes = NULL;
	u32  nState;

	pNodeList = pCtxt->pNodeList;
	while (pNodeList != NULL) {
		pNodeRes = pNodeList;
		pNodeList = pNodeList->next;
		if (pNodeRes->nodeAllocated) {
			nState = NODE_GetState(pNodeRes->hNode) ;
			if (nState <= NODE_DELETING) {
				if ((nState == NODE_RUNNING) ||
					(nState == NODE_PAUSED) ||
					(nState == NODE_TERMINATING)) {
					status = NODE_Terminate
						(pNodeRes->hNode, &status);
					status = NODE_Delete(pNodeRes->hNode,
							pCtxt);
				} else if ((nState == NODE_ALLOCATED)
					|| (nState == NODE_CREATED))
					status = NODE_Delete(pNodeRes->hNode,
							pCtxt);
			}
		}
	}
	return status;
}

/* Release all Mapped and Reserved DMM resources */
DSP_STATUS DRV_RemoveAllDMMResElements(HANDLE hPCtxt)
{
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct DMM_MAP_OBJECT *temp_map, *map_obj;
	struct DMM_RSV_OBJECT *temp_rsv, *rsv_obj;

	/* Free DMM mapped memory resources */
	list_for_each_entry_safe(map_obj, temp_map, &pCtxt->dmm_map_list,
			link) {
		status = PROC_UnMap(pCtxt->hProcessor,
				(void *)map_obj->dsp_addr, pCtxt);
		if (DSP_FAILED(status))
			pr_err("%s: PROC_UnMap failed!"
					" status = 0x%xn", __func__, status);
	}

	/* Free DMM reserved memory resources */
	list_for_each_entry_safe(rsv_obj, temp_rsv, &pCtxt->dmm_rsv_list,
			link) {
		status = PROC_UnReserveMemory(pCtxt->hProcessor,
				(void *)rsv_obj->dsp_reserved_addr, pCtxt);
		if (DSP_FAILED(status))
			pr_err("%s: PROC_UnReserveMemory failed!"
					" status = 0x%xn", __func__, status);
	}
	return status;
}

/* Update Node allocation status */
void DRV_ProcNodeUpdateStatus(HANDLE hNodeRes, s32 status)
{
	struct NODE_RES_OBJECT *pNodeRes = (struct NODE_RES_OBJECT *)hNodeRes;
	DBC_Assert(hNodeRes != NULL);
	pNodeRes->nodeAllocated = status;
}

/* Update Node Heap status */
void DRV_ProcNodeUpdateHeapStatus(HANDLE hNodeRes, s32 status)
{
	struct NODE_RES_OBJECT *pNodeRes = (struct NODE_RES_OBJECT *)hNodeRes;
	DBC_Assert(hNodeRes != NULL);
	pNodeRes->heapAllocated = status;
}

/* Release all Node resources and its context
* This is called from .bridge_release.
*/
DSP_STATUS 	DRV_RemoveAllNodeResElements(HANDLE hPCtxt)
{
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct NODE_RES_OBJECT *pTempNode2 = NULL;
	struct NODE_RES_OBJECT *pTempNode = NULL;

	DRV_ProcFreeNodeRes(pCtxt);
	pTempNode = pCtxt->pNodeList;
	while (pTempNode != NULL) {
		pTempNode2 = pTempNode;
		pTempNode = pTempNode->next;
		kfree(pTempNode2);
	}
	pCtxt->pNodeList = NULL;
	return status;
}

/* Getting the node resource element */
DSP_STATUS DRV_GetNodeResElement(HANDLE hNode, HANDLE hNodeRes, HANDLE hPCtxt)
{
	struct NODE_RES_OBJECT **nodeRes = (struct NODE_RES_OBJECT **)hNodeRes;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct NODE_RES_OBJECT *pTempNode2 = NULL;
	struct NODE_RES_OBJECT *pTempNode = NULL;

	if (mutex_lock_interruptible(&pCtxt->node_mutex))
		return DSP_EFAIL;

	pTempNode = pCtxt->pNodeList;
	while ((pTempNode != NULL) && (pTempNode->hNode != hNode)) {
		pTempNode2 = pTempNode;
		pTempNode = pTempNode->next;
	}

	mutex_unlock(&pCtxt->node_mutex);

	if (pTempNode != NULL)
		*nodeRes = pTempNode;
	else
		status = DSP_ENOTFOUND;

	return status;
}

/* Allocate the STRM resource element
* This is called after the actual resource is allocated
*/
DSP_STATUS DRV_ProcInsertSTRMResElement(HANDLE hStreamHandle, HANDLE hSTRMRes,
					HANDLE hPCtxt)
{
	struct STRM_RES_OBJECT **pSTRMRes = (struct STRM_RES_OBJECT **)hSTRMRes;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct STRM_RES_OBJECT *pTempSTRMRes = NULL;

	*pSTRMRes = (struct STRM_RES_OBJECT *)
		    MEM_Calloc(1 * sizeof(struct STRM_RES_OBJECT), MEM_PAGED);
	if (*pSTRMRes == NULL)
		status = DSP_EHANDLE;

	if (DSP_SUCCEEDED(status)) {
		if (mutex_lock_interruptible(&pCtxt->strm_mutex)) {
			kfree(*pSTRMRes);
			return DSP_EFAIL;
		}
		(*pSTRMRes)->hStream = hStreamHandle;
		if (pCtxt->pSTRMList != NULL) {
			pTempSTRMRes = pCtxt->pSTRMList;
			while (pTempSTRMRes->next != NULL)
				pTempSTRMRes = pTempSTRMRes->next;

			pTempSTRMRes->next = *pSTRMRes;
		} else {
			pCtxt->pSTRMList = *pSTRMRes;
		}
		mutex_unlock(&pCtxt->strm_mutex);
	}
	return status;
}

/* Release Stream resource element context
* This function called after the actual resource is freed
*/
DSP_STATUS 	DRV_ProcRemoveSTRMResElement(HANDLE hSTRMRes, HANDLE hPCtxt)
{
	struct STRM_RES_OBJECT *pSTRMRes = (struct STRM_RES_OBJECT *)hSTRMRes;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	struct STRM_RES_OBJECT *pTempSTRMRes;
	DSP_STATUS status = DSP_SOK;

	if (mutex_lock_interruptible(&pCtxt->strm_mutex))
		return DSP_EFAIL;
	pTempSTRMRes = pCtxt->pSTRMList;

	if (pCtxt->pSTRMList == pSTRMRes) {
		pCtxt->pSTRMList = pSTRMRes->next;
	} else {
		while (pTempSTRMRes && pTempSTRMRes->next != pSTRMRes)
			pTempSTRMRes = pTempSTRMRes->next;
		if (pTempSTRMRes == NULL)
			status = DSP_ENOTFOUND;
		else
			pTempSTRMRes->next = pSTRMRes->next;
	}
	mutex_unlock(&pCtxt->strm_mutex);
	kfree(pSTRMRes);
	return status;
}

/* Release all Stream resources and its context
 * This is called from .bridge_release.
 */
DSP_STATUS DRV_RemoveAllSTRMResElements(HANDLE hPCtxt)
{
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct STRM_RES_OBJECT *strm_res = NULL;
	struct STRM_RES_OBJECT *strm_tmp = NULL;
	struct STRM_INFO strm_info;
	struct DSP_STREAMINFO user;
	u8 **apBuffer = NULL;
	u8 *pBufPtr;
	u32 ulBytes;
	u32 dwArg;
	s32 ulBufSize;

	strm_tmp = pCtxt->pSTRMList;
	while (strm_tmp) {
		strm_res = strm_tmp;
		strm_tmp = strm_tmp->next;
		if (strm_res->uNumBufs) {
			apBuffer = MEM_Alloc((strm_res->uNumBufs *
						sizeof(u8 *)), MEM_NONPAGED);

			if (apBuffer) {
				status = STRM_FreeBuffer(strm_res->hStream,
					apBuffer, strm_res->uNumBufs, pCtxt);
				kfree(apBuffer);
			}
		}
		strm_info.pUser = &user;
		user.uNumberBufsInStream = 0;
		STRM_GetInfo(strm_res->hStream, &strm_info, sizeof(strm_info));
		while (user.uNumberBufsInStream--)
			STRM_Reclaim(strm_res->hStream, &pBufPtr, &ulBytes,
					     (u32 *)&ulBufSize, &dwArg);
		status = STRM_Close(strm_res->hStream, pCtxt);
	}
	return status;
}

/* Getting the stream resource element */
DSP_STATUS DRV_GetSTRMResElement(HANDLE hStrm, HANDLE hSTRMRes, HANDLE hPCtxt)
{
	struct STRM_RES_OBJECT **STRMRes = (struct STRM_RES_OBJECT **)hSTRMRes;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	DSP_STATUS status = DSP_SOK;
	struct STRM_RES_OBJECT *pTempSTRM2 = NULL;
	struct STRM_RES_OBJECT *pTempSTRM;

	if (mutex_lock_interruptible(&pCtxt->strm_mutex))
		return DSP_EFAIL;

	pTempSTRM = pCtxt->pSTRMList;
	while ((pTempSTRM != NULL) && (pTempSTRM->hStream != hStrm)) {
		pTempSTRM2 = pTempSTRM;
		pTempSTRM = pTempSTRM->next;
	}

	mutex_unlock(&pCtxt->strm_mutex);


	if (pTempSTRM != NULL)
		*STRMRes = pTempSTRM;
	else
		status = DSP_ENOTFOUND;

	return status;
}

/* Updating the stream resource element */
DSP_STATUS DRV_ProcUpdateSTRMRes(u32 uNumBufs, HANDLE hSTRMRes)
{
	DSP_STATUS status = DSP_SOK;
	struct STRM_RES_OBJECT **STRMRes = (struct STRM_RES_OBJECT **)hSTRMRes;

	(*STRMRes)->uNumBufs = uNumBufs;
	return status;
}

/* GPP PROCESS CLEANUP CODE END */

/*
 *  ======== = DRV_Create ======== =
 *  Purpose:
 *      DRV Object gets created only once during Driver Loading.
 */
DSP_STATUS DRV_Create(OUT struct DRV_OBJECT **phDRVObject)
{
	DSP_STATUS status = DSP_SOK;
	struct DRV_OBJECT *pDRVObject = NULL;

	DBC_Require(phDRVObject != NULL);
	DBC_Require(cRefs > 0);

	MEM_AllocObject(pDRVObject, struct DRV_OBJECT, SIGNATURE);
	if (pDRVObject) {
		/* Create and Initialize List of device objects */
		pDRVObject->devList = MEM_Calloc(sizeof(struct LST_LIST),
			MEM_NONPAGED);
		if (pDRVObject->devList) {
			/* Create and Initialize List of device Extension */
			pDRVObject->devNodeString = MEM_Calloc(sizeof(struct
				LST_LIST), MEM_NONPAGED);
			if (!(pDRVObject->devNodeString)) {
				status = DSP_EFAIL;
			} else {
				INIT_LIST_HEAD(&pDRVObject->devNodeString->
					head);
				INIT_LIST_HEAD(&pDRVObject->devList->head);
			}
		} else {
			status = DSP_EMEMORY;
		}
	} else {
		status = DSP_EMEMORY;
	}
	/* Store the DRV Object in the Registry */
	if (DSP_SUCCEEDED(status))
		status = CFG_SetObject((u32) pDRVObject, REG_DRV_OBJECT);
	if (DSP_SUCCEEDED(status)) {
		*phDRVObject = pDRVObject;
	} else {
		kfree(pDRVObject->devList);
		kfree(pDRVObject->devNodeString);
		/* Free the DRV Object */
		kfree(pDRVObject);
	}

	DBC_Ensure(DSP_FAILED(status) ||
		  MEM_IsValidHandle(pDRVObject, SIGNATURE));
	return status;
}

/*
 *  ======== DRV_Exit ========
 *  Purpose:
 *      Discontinue usage of the DRV module.
 */
void DRV_Exit(void)
{
	DBC_Require(cRefs > 0);

	cRefs--;

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== = DRV_Destroy ======== =
 *  purpose:
 *      Invoked during bridge de-initialization
 */
DSP_STATUS DRV_Destroy(struct DRV_OBJECT *hDRVObject)
{
	DSP_STATUS status = DSP_SOK;
	struct DRV_OBJECT *pDRVObject = (struct DRV_OBJECT *)hDRVObject;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(pDRVObject, SIGNATURE));

	/*
	 *  Delete the List if it exists.Should not come here
	 *  as the DRV_RemoveDevObject and the Last DRV_RequestResources
	 *  removes the list if the lists are empty.
	 */
	kfree(pDRVObject->devList);
	kfree(pDRVObject->devNodeString);
	MEM_FreeObject(pDRVObject);
	/* Update the DRV Object in Registry to be 0 */
	(void)CFG_SetObject(0, REG_DRV_OBJECT);
	DBC_Ensure(!MEM_IsValidHandle(pDRVObject, SIGNATURE));
	return status;
}

/*
 *  ======== DRV_GetDevObject ========
 *  Purpose:
 *      Given a index, returns a handle to DevObject from the list.
 */
DSP_STATUS DRV_GetDevObject(u32 uIndex, struct DRV_OBJECT *hDrvObject,
			   struct DEV_OBJECT **phDevObject)
{
	DSP_STATUS status = DSP_SOK;
#if GT_TRACE	/* pDrvObject is used only for Assertions and debug messages.*/
	struct DRV_OBJECT *pDrvObject = (struct DRV_OBJECT *)hDrvObject;
#endif
	struct DEV_OBJECT *pDevObject;
	u32 i;
	DBC_Require(MEM_IsValidHandle(pDrvObject, SIGNATURE));
	DBC_Require(phDevObject != NULL);
	DBC_Require(uIndex >= 0);
	DBC_Require(cRefs > 0);
	DBC_Assert(!(LST_IsEmpty(pDrvObject->devList)));

	pDevObject = (struct DEV_OBJECT *)DRV_GetFirstDevObject();
	for (i = 0; i < uIndex; i++) {
		pDevObject =
		   (struct DEV_OBJECT *)DRV_GetNextDevObject((u32)pDevObject);
	}
	if (pDevObject) {
		*phDevObject = (struct DEV_OBJECT *) pDevObject;
	} else {
		*phDevObject = NULL;
		status = DSP_EFAIL;
	}

	return status;
}

/*
 *  ======== DRV_GetFirstDevObject ========
 *  Purpose:
 *      Retrieve the first Device Object handle from an internal linked list of
 *      of DEV_OBJECTs maintained by DRV.
 */
u32 DRV_GetFirstDevObject(void)
{
	u32 dwDevObject = 0;
	struct DRV_OBJECT *pDrvObject;

	if (DSP_SUCCEEDED
	    (CFG_GetObject((u32 *)&pDrvObject, REG_DRV_OBJECT))) {
		if ((pDrvObject->devList != NULL) &&
		   !LST_IsEmpty(pDrvObject->devList))
			dwDevObject = (u32) LST_First(pDrvObject->devList);
	}

	return dwDevObject;
}

/*
 *  ======== DRV_GetFirstDevNodeString ========
 *  Purpose:
 *      Retrieve the first Device Extension from an internal linked list of
 *      of Pointer to DevNode Strings maintained by DRV.
 */
u32 DRV_GetFirstDevExtension(void)
{
	u32 dwDevExtension = 0;
	struct DRV_OBJECT *pDrvObject;

	if (DSP_SUCCEEDED
	    (CFG_GetObject((u32 *)&pDrvObject, REG_DRV_OBJECT))) {

		if ((pDrvObject->devNodeString != NULL) &&
		   !LST_IsEmpty(pDrvObject->devNodeString)) {
			dwDevExtension = (u32)LST_First(pDrvObject->
							devNodeString);
		}
	}

	return dwDevExtension;
}

/*
 *  ======== DRV_GetNextDevObject ========
 *  Purpose:
 *      Retrieve the next Device Object handle from an internal linked list of
 *      of DEV_OBJECTs maintained by DRV, after having previously called
 *      DRV_GetFirstDevObject() and zero or more DRV_GetNext.
 */
u32 DRV_GetNextDevObject(u32 hDevObject)
{
	u32 dwNextDevObject = 0;
	struct DRV_OBJECT *pDrvObject;

	DBC_Require(hDevObject != 0);

	if (DSP_SUCCEEDED
	    (CFG_GetObject((u32 *)&pDrvObject, REG_DRV_OBJECT))) {

		if ((pDrvObject->devList != NULL) &&
		   !LST_IsEmpty(pDrvObject->devList)) {
			dwNextDevObject = (u32)LST_Next(pDrvObject->devList,
					  (struct list_head *)hDevObject);
		}
	}
	return dwNextDevObject;
}

/*
 *  ======== DRV_GetNextDevExtension ========
 *  Purpose:
 *      Retrieve the next Device Extension from an internal linked list of
 *      of pointer to DevNodeString maintained by DRV, after having previously
 *      called DRV_GetFirstDevExtension() and zero or more
 *      DRV_GetNextDevExtension().
 */
u32 DRV_GetNextDevExtension(u32 hDevExtension)
{
	u32 dwDevExtension = 0;
	struct DRV_OBJECT *pDrvObject;

	DBC_Require(hDevExtension != 0);

	if (DSP_SUCCEEDED(CFG_GetObject((u32 *)&pDrvObject,
	   REG_DRV_OBJECT))) {
		if ((pDrvObject->devNodeString != NULL) &&
		   !LST_IsEmpty(pDrvObject->devNodeString)) {
			dwDevExtension = (u32)LST_Next(pDrvObject->
				devNodeString,
				(struct list_head *)hDevExtension);
		}
	}

	return dwDevExtension;
}

/*
 *  ======== DRV_Init ========
 *  Purpose:
 *      Initialize DRV module private state.
 */
DSP_STATUS DRV_Init(void)
{
	s32 fRetval = 1;	/* function return value */

	DBC_Require(cRefs >= 0);

	if (fRetval)
		cRefs++;

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	return fRetval;
}

/*
 *  ======== DRV_InsertDevObject ========
 *  Purpose:
 *      Insert a DevObject into the list of Manager object.
 */
DSP_STATUS DRV_InsertDevObject(struct DRV_OBJECT *hDRVObject,
				struct DEV_OBJECT *hDevObject)
{
	DSP_STATUS status = DSP_SOK;
	struct DRV_OBJECT *pDRVObject = (struct DRV_OBJECT *)hDRVObject;

	DBC_Require(cRefs > 0);
	DBC_Require(hDevObject != NULL);
	DBC_Require(MEM_IsValidHandle(pDRVObject, SIGNATURE));
	DBC_Assert(pDRVObject->devList);

	LST_PutTail(pDRVObject->devList, (struct list_head *)hDevObject);

	DBC_Ensure(DSP_SUCCEEDED(status) && !LST_IsEmpty(pDRVObject->devList));

	return status;
}

/*
 *  ======== DRV_RemoveDevObject ========
 *  Purpose:
 *      Search for and remove a DeviceObject from the given list of DRV
 *      objects.
 */
DSP_STATUS DRV_RemoveDevObject(struct DRV_OBJECT *hDRVObject,
				struct DEV_OBJECT *hDevObject)
{
	DSP_STATUS status = DSP_EFAIL;
	struct DRV_OBJECT *pDRVObject = (struct DRV_OBJECT *)hDRVObject;
	struct list_head *pCurElem;

	DBC_Require(cRefs > 0);
	DBC_Require(MEM_IsValidHandle(pDRVObject, SIGNATURE));
	DBC_Require(hDevObject != NULL);

	DBC_Require(pDRVObject->devList != NULL);
	DBC_Require(!LST_IsEmpty(pDRVObject->devList));

	/* Search list for pProcObject: */
	for (pCurElem = LST_First(pDRVObject->devList); pCurElem != NULL;
	    pCurElem = LST_Next(pDRVObject->devList, pCurElem)) {
		/* If found, remove it. */
		if ((struct DEV_OBJECT *) pCurElem == hDevObject) {
			LST_RemoveElem(pDRVObject->devList, pCurElem);
			status = DSP_SOK;
			break;
		}
	}
	/* Remove list if empty. */
	if (LST_IsEmpty(pDRVObject->devList)) {
		kfree(pDRVObject->devList);
		pDRVObject->devList = NULL;
	}
	DBC_Ensure((pDRVObject->devList == NULL) ||
		  !LST_IsEmpty(pDRVObject->devList));

	return status;
}

/*
 *  ======== DRV_RequestResources ========
 *  Purpose:
 *      Requests  resources from the OS.
 */
DSP_STATUS DRV_RequestResources(u32 dwContext, u32 *pDevNodeString)
{
	DSP_STATUS status = DSP_SOK;
	struct DRV_OBJECT *pDRVObject;
	struct DRV_EXT *pszdevNode;

	DBC_Require(dwContext != 0);
	DBC_Require(pDevNodeString != NULL);

	/*
	 *  Allocate memory to hold the string. This will live untill
	 *  it is freed in the Release resources. Update the driver object
	 *  list.
	 */

	status = CFG_GetObject((u32 *)&pDRVObject, REG_DRV_OBJECT);
	if (DSP_SUCCEEDED(status)) {
		pszdevNode = MEM_Calloc(sizeof(struct DRV_EXT), MEM_NONPAGED);
		if (pszdevNode) {
			LST_InitElem(&pszdevNode->link);
			strncpy(pszdevNode->szString,
				 (char *)dwContext, MAXREGPATHLENGTH - 1);
			pszdevNode->szString[MAXREGPATHLENGTH - 1] = '\0';
			/* Update the Driver Object List */
			*pDevNodeString = (u32)pszdevNode->szString;
			LST_PutTail(pDRVObject->devNodeString,
					(struct list_head *)pszdevNode);
		} else {
			status = DSP_EMEMORY;
			*pDevNodeString = 0;
		}
	} else {
		GT_0trace(curTrace, GT_7CLASS,
			 "Failed to get Driver Object from Registry");
		*pDevNodeString = 0;
	}

	if (!(strcmp((char *) dwContext, "TIOMAP1510"))) {
		GT_0trace(curTrace, GT_1CLASS,
			  " Allocating resources for UMA \n");
		status = RequestBridgeResourcesDSP(dwContext, DRV_ASSIGN);
	} else {
		status = DSP_EFAIL;
		GT_0trace(curTrace, GT_7CLASS, "Unknown Device ");
	}

	if (DSP_FAILED(status)) {
		GT_0trace(curTrace, GT_7CLASS,
			 "Failed to reserve bridge resources ");
	}
	DBC_Ensure((DSP_SUCCEEDED(status) && pDevNodeString != NULL &&
		  !LST_IsEmpty(pDRVObject->devNodeString)) ||
		  (DSP_FAILED(status) && *pDevNodeString == 0));

	return status;
}

/*
 *  ======== DRV_ReleaseResources ========
 *  Purpose:
 *      Releases  resources from the OS.
 */
DSP_STATUS DRV_ReleaseResources(u32 dwContext, struct DRV_OBJECT *hDrvObject)
{
	DSP_STATUS status = DSP_SOK;
	struct DRV_OBJECT *pDRVObject = (struct DRV_OBJECT *)hDrvObject;
	struct DRV_EXT *pszdevNode;

	if (!(strcmp((char *)((struct DRV_EXT *)dwContext)->szString,
	   "TIOMAP1510")))
		status = RequestBridgeResources(dwContext, DRV_RELEASE);
	else
		GT_0trace(curTrace, GT_1CLASS, " Unknown device\n");

	if (DSP_FAILED(status))
		GT_0trace(curTrace, GT_1CLASS,
			 "Failed to relese bridge resources\n");

	/*
	 *  Irrespective of the status go ahead and clean it
	 *  The following will over write the status.
	 */
	for (pszdevNode = (struct DRV_EXT *)DRV_GetFirstDevExtension();
	    pszdevNode != NULL; pszdevNode = (struct DRV_EXT *)
	    DRV_GetNextDevExtension((u32)pszdevNode)) {
		if (!pDRVObject->devNodeString) {
			/* When this could happen? */
			continue;
		}
		if ((u32)pszdevNode == dwContext) {
			/* Found it */
			/* Delete from the Driver object list */
			LST_RemoveElem(pDRVObject->devNodeString,
				      (struct list_head *)pszdevNode);
			kfree((void *) pszdevNode);
			break;
		}
		/* Delete the List if it is empty */
		if (LST_IsEmpty(pDRVObject->devNodeString)) {
			kfree(pDRVObject->devNodeString);
			pDRVObject->devNodeString = NULL;
		}
	}
	return status;
}

/*
 *  ======== RequestBridgeResources ========
 *  Purpose:
 *      Reserves shared memory for bridge.
 */
static DSP_STATUS RequestBridgeResources(u32 dwContext, s32 bRequest)
{
	DSP_STATUS status = DSP_SOK;
	struct CFG_HOSTRES *pResources;
	u32 dwBuffSize;

	struct DRV_EXT *driverExt;
	u32 shm_size;

	DBC_Require(dwContext != 0);

	if (!bRequest) {
		driverExt = (struct DRV_EXT *)dwContext;
		/* Releasing resources by deleting the registry key  */
		dwBuffSize = sizeof(struct CFG_HOSTRES);
		pResources = MEM_Calloc(dwBuffSize, MEM_NONPAGED);
		if (pResources != NULL) {
			if (DSP_FAILED(REG_GetValue(CURRENTCONFIG,
					(u8 *)pResources, &dwBuffSize))) {
				status = CFG_E_RESOURCENOTAVAIL;
			}

			dwBuffSize = sizeof(shm_size);
			status = REG_GetValue(SHMSIZE, (u8 *)&shm_size,
					      &dwBuffSize);
			if (DSP_SUCCEEDED(status)) {
				if ((pResources->dwMemBase[1]) &&
				   (pResources->dwMemPhys[1])) {
					MEM_FreePhysMem((void *)pResources->
					dwMemBase[1], pResources->dwMemPhys[1],
					shm_size);
				}
			} else {
				GT_1trace(curTrace, GT_7CLASS,
					"Error getting SHM size from registry: "
					"%x. Not calling MEM_FreePhysMem\n",
					status);
			}
			pResources->dwMemBase[1] = 0;
			pResources->dwMemPhys[1] = 0;

			if (pResources->dwPrmBase)
				iounmap(pResources->dwPrmBase);
			if (pResources->dwCmBase)
				iounmap(pResources->dwCmBase);
			if (pResources->dwMemBase[0])
				iounmap((void *)pResources->dwMemBase[0]);
			if (pResources->dwMemBase[2])
				iounmap((void *)pResources->dwMemBase[2]);
			if (pResources->dwMemBase[3])
				iounmap((void *)pResources->dwMemBase[3]);
			if (pResources->dwMemBase[4])
				iounmap((void *)pResources->dwMemBase[4]);
			if (pResources->dwWdTimerDspBase)
				iounmap(pResources->dwWdTimerDspBase);
			if (pResources->dwDmmuBase)
				iounmap(pResources->dwDmmuBase);
			if (pResources->dwPerBase)
				iounmap(pResources->dwPerBase);
			if (pResources->dwPerPmBase)
				iounmap((void *)pResources->dwPerPmBase);
			if (pResources->dwCorePmBase)
				iounmap((void *)pResources->dwCorePmBase);
			if (pResources->dwSysCtrlBase)
				iounmap(pResources->dwSysCtrlBase);

			pResources->dwPrmBase = NULL;
			pResources->dwCmBase = NULL;
			pResources->dwMemBase[0] = (u32) NULL;
			pResources->dwMemBase[2] = (u32) NULL;
			pResources->dwMemBase[3] = (u32) NULL;
			pResources->dwMemBase[4] = (u32) NULL;
			pResources->dwWdTimerDspBase = NULL;
			pResources->dwDmmuBase = NULL;
			pResources->dwSysCtrlBase = NULL;

			dwBuffSize = sizeof(struct CFG_HOSTRES);
			status = REG_SetValue(CURRENTCONFIG, (u8 *)pResources,
				 (u32)dwBuffSize);
			/*  Set all the other entries to NULL */
			kfree(pResources);
		} else {
			status = DSP_EMEMORY;
		}
		return status;
	}
	dwBuffSize = sizeof(struct CFG_HOSTRES);
	pResources = MEM_Calloc(dwBuffSize, MEM_NONPAGED);
	if (pResources != NULL) {
		/* wNumMemWindows must not be more than CFG_MAXMEMREGISTERS */
		pResources->wNumMemWindows = 2;
		/* First window is for DSP internal memory */

		pResources->dwPrmBase = ioremap(OMAP_IVA2_PRM_BASE,
							OMAP_IVA2_PRM_SIZE);
		pResources->dwCmBase = ioremap(OMAP_IVA2_CM_BASE,
							OMAP_IVA2_CM_SIZE);
		pResources->dwSysCtrlBase = ioremap(OMAP_SYSC_BASE,
							OMAP_SYSC_SIZE);
		GT_1trace(curTrace, GT_2CLASS, "dwMemBase[0] 0x%x\n",
			 pResources->dwMemBase[0]);
		GT_1trace(curTrace, GT_2CLASS, "dwMemBase[3] 0x%x\n",
			 pResources->dwMemBase[3]);
		GT_1trace(curTrace, GT_2CLASS, "dwPrmBase 0x%x\n",
							pResources->dwPrmBase);
		GT_1trace(curTrace, GT_2CLASS, "dwCmBase 0x%x\n",
							pResources->dwCmBase);
		GT_1trace(curTrace, GT_2CLASS, "dwWdTimerDspBase 0x%x\n",
						pResources->dwWdTimerDspBase);
		GT_1trace(curTrace, GT_2CLASS, "dwDmmuBase 0x%x\n",
						pResources->dwDmmuBase);

		/* for 24xx base port is not mapping the mamory for DSP
		 * internal memory TODO Do a ioremap here */
		/* Second window is for DSP external memory shared with MPU */
		/* for Linux, these are hard-coded values */
		pResources->bIRQRegisters = 0;
		pResources->bIRQAttrib = 0;
		pResources->dwOffsetForMonitor = 0;
		pResources->dwChnlOffset = 0;
		/* CHNL_MAXCHANNELS */
		pResources->dwNumChnls = CHNL_MAXCHANNELS;
		pResources->dwChnlBufSize = 0x400;
		dwBuffSize = sizeof(struct CFG_HOSTRES);
		status = REG_SetValue(CURRENTCONFIG, (u8 *)pResources,
						sizeof(struct CFG_HOSTRES));
		if (DSP_FAILED(status)) {
			GT_0trace(curTrace, GT_7CLASS,
				 " Failed to set the registry "
				 "value for CURRENTCONFIG\n");
		}
		kfree(pResources);
	}
	/* End Mem alloc */
	return status;
}

/*
 *  ======== RequestBridgeResourcesDSP ========
 *  Purpose:
 *      Reserves shared memory for bridge.
 */
static DSP_STATUS RequestBridgeResourcesDSP(u32 dwContext, s32 bRequest)
{
	DSP_STATUS status = DSP_SOK;
	struct CFG_HOSTRES *pResources;
	u32 dwBuffSize;
	u32 dmaAddr;
	u32 shm_size;

	DBC_Require(dwContext != 0);

	dwBuffSize = sizeof(struct CFG_HOSTRES);

	pResources = MEM_Calloc(dwBuffSize, MEM_NONPAGED);

	if (pResources != NULL) {
		if (DSP_FAILED(CFG_GetHostResources((struct CFG_DEVNODE *)
		   dwContext, pResources))) {
			/* Call CFG_GetHostResources to get reserve resouces */
			status = RequestBridgeResources(dwContext, bRequest);
			if (DSP_SUCCEEDED(status)) {
				status = CFG_GetHostResources
					((struct CFG_DEVNODE *) dwContext,
					pResources);
			}
		}
		/* wNumMemWindows must not be more than CFG_MAXMEMREGISTERS */
		pResources->wNumMemWindows = 4;

		pResources->dwMemBase[0] = 0;
		pResources->dwMemBase[2] = (u32)ioremap(OMAP_DSP_MEM1_BASE,
							OMAP_DSP_MEM1_SIZE);
		pResources->dwMemBase[3] = (u32)ioremap(OMAP_DSP_MEM2_BASE,
							OMAP_DSP_MEM2_SIZE);
		pResources->dwMemBase[4] = (u32)ioremap(OMAP_DSP_MEM3_BASE,
							OMAP_DSP_MEM3_SIZE);
		pResources->dwPerBase = ioremap(OMAP_PER_CM_BASE,
							OMAP_PER_CM_SIZE);
		pResources->dwPerPmBase = ioremap(OMAP_PER_PRM_BASE,
							OMAP_PER_PRM_SIZE);
		pResources->dwCorePmBase = (u32)ioremap(OMAP_CORE_PRM_BASE,
							OMAP_CORE_PRM_SIZE);
		pResources->dwDmmuBase = ioremap(OMAP_DMMU_BASE,
							OMAP_DMMU_SIZE);
		pResources->dwWdTimerDspBase = ioremap(OMAP_WDT3_BASE,
							OMAP_WDT3_BASE);

		GT_1trace(curTrace, GT_2CLASS, "dwMemBase[0] 0x%x\n",
						pResources->dwMemBase[0]);
		GT_1trace(curTrace, GT_2CLASS, "dwMemBase[1] 0x%x\n",
						pResources->dwMemBase[1]);
		GT_1trace(curTrace, GT_2CLASS, "dwMemBase[2] 0x%x\n",
						pResources->dwMemBase[2]);
		GT_1trace(curTrace, GT_2CLASS, "dwMemBase[3] 0x%x\n",
						pResources->dwMemBase[3]);
		GT_1trace(curTrace, GT_2CLASS, "dwMemBase[4] 0x%x\n",
						pResources->dwMemBase[4]);
		GT_1trace(curTrace, GT_2CLASS, "dwPrmBase 0x%x\n",
						pResources->dwPrmBase);
		GT_1trace(curTrace, GT_2CLASS, "dwCmBase 0x%x\n",
						pResources->dwCmBase);
		GT_1trace(curTrace, GT_2CLASS, "dwWdTimerDspBase 0x%x\n",
						pResources->dwWdTimerDspBase);
		GT_1trace(curTrace, GT_2CLASS, "dwDmmuBase 0x%x\n",
						pResources->dwDmmuBase);
		dwBuffSize = sizeof(shm_size);
		status = REG_GetValue(SHMSIZE, (u8 *)&shm_size, &dwBuffSize);
		if (DSP_SUCCEEDED(status)) {
			/* Allocate Physically contiguous,
			 * non-cacheable  memory */
			pResources->dwMemBase[1] =
				(u32)MEM_AllocPhysMem(shm_size, 0x100000,
							&dmaAddr);
			if (pResources->dwMemBase[1] == 0) {
				status = DSP_EMEMORY;
				pr_err("SHM reservation Failed\n");
			} else {
				pResources->dwMemLength[1] = shm_size;
				pResources->dwMemPhys[1] = dmaAddr;

				GT_3trace(curTrace, GT_1CLASS,
					 "Bridge SHM address 0x%x dmaAddr"
					 " %x size %x\n",
					 pResources->dwMemBase[1],
					 dmaAddr, shm_size);
			}
		}
		if (DSP_SUCCEEDED(status)) {
			/* for Linux, these are hard-coded values */
			pResources->bIRQRegisters = 0;
			pResources->bIRQAttrib = 0;
			pResources->dwOffsetForMonitor = 0;
			pResources->dwChnlOffset = 0;
			/* CHNL_MAXCHANNELS */
			pResources->dwNumChnls = CHNL_MAXCHANNELS;
			pResources->dwChnlBufSize = 0x400;
			dwBuffSize = sizeof(struct CFG_HOSTRES);
			status = REG_SetValue(CURRENTCONFIG, (u8 *)pResources,
					     sizeof(struct CFG_HOSTRES));
			if (DSP_FAILED(status)) {
				GT_0trace(curTrace, GT_7CLASS,
					 " Failed to set the registry value"
					 " for CURRENTCONFIG\n");
			}
		}
		kfree(pResources);
	}
	/* End Mem alloc */
	return status;
}
