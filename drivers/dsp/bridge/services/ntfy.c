/*
 * ntfy.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Manage lists of notification events.
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
#include <dspbridge/list.h>
#include <dspbridge/mem.h>
#include <dspbridge/sync.h>

/*  ----------------------------------- This */
#include <dspbridge/ntfy.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define NTFY_SIGNATURE      0x5946544e	/* "YFTN" */

/*
 *  ======== NTFY_OBJECT ========
 */
struct NTFY_OBJECT {
	u32 dwSignature;	/* For object validation */
	struct LST_LIST *notifyList;	/* List of NOTIFICATION objects */
	struct SYNC_CSOBJECT *hSync;	/* For critical sections */
};

/*
 *  ======== NOTIFICATION ========
 *  This object will be created when a client registers for events.
 */
struct NOTIFICATION {
	struct list_head listElem;
	u32 uEventMask;	/* Events to be notified about */
	u32 uNotifyType;	/* Type of notification to be sent */

	/*
	 *  We keep a copy of the event name to check if the event has
	 *  already been registered. (SYNC also keeps a copy of the name).
	 */
	char *pstrName;		/* Name of event */
	HANDLE hEvent;		/* Handle for notification */
	struct SYNC_OBJECT *hSync;
};

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask NTFY_debugMask = { NULL, NULL };  /* GT trace variable */
#endif

/*  ----------------------------------- Function Prototypes */
static void DeleteNotify(struct NOTIFICATION *pNotify);

/*
 *  ======== NTFY_Create ========
 *  Purpose:
 *      Create an empty list of notifications.
 */
DSP_STATUS NTFY_Create(struct NTFY_OBJECT **phNtfy)
{
	struct NTFY_OBJECT *pNtfy;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(phNtfy != NULL);

	*phNtfy = NULL;
	MEM_AllocObject(pNtfy, struct NTFY_OBJECT, NTFY_SIGNATURE);

	if (pNtfy) {

		status = SYNC_InitializeDPCCS(&pNtfy->hSync);
		if (DSP_SUCCEEDED(status)) {
			pNtfy->notifyList = MEM_Calloc(sizeof(struct LST_LIST),
							MEM_NONPAGED);
			if (pNtfy->notifyList == NULL) {
				(void) SYNC_DeleteCS(pNtfy->hSync);
				MEM_FreeObject(pNtfy);
				status = DSP_EMEMORY;
			} else {
				INIT_LIST_HEAD(&pNtfy->notifyList->head);
				*phNtfy = pNtfy;
			}
		}
	} else {
		status = DSP_EMEMORY;
	}

	DBC_Ensure((DSP_FAILED(status) && *phNtfy == NULL) ||
		  (DSP_SUCCEEDED(status) && MEM_IsValidHandle((*phNtfy),
		  NTFY_SIGNATURE)));

	return status;
}

/*
 *  ======== NTFY_Delete ========
 *  Purpose:
 *      Free resources allocated in NTFY_Create.
 */
void NTFY_Delete(struct NTFY_OBJECT *hNtfy)
{
	struct NOTIFICATION *pNotify;

	DBC_Require(MEM_IsValidHandle(hNtfy, NTFY_SIGNATURE));

	/* Remove any elements remaining in list */
	if (hNtfy->notifyList) {

		(void) SYNC_EnterCS(hNtfy->hSync);
	
		while ((pNotify = (struct NOTIFICATION *)LST_GetHead(hNtfy->
								notifyList))) {
			DeleteNotify(pNotify);
		}
		DBC_Assert(LST_IsEmpty(hNtfy->notifyList));
		kfree(hNtfy->notifyList);

		(void) SYNC_LeaveCS(hNtfy->hSync);
	}

	
	if (hNtfy->hSync)
		(void)SYNC_DeleteCS(hNtfy->hSync);

	MEM_FreeObject(hNtfy);
}

/*
 *  ======== NTFY_Exit ========
 *  Purpose:
 *      Discontinue usage of NTFY module.
 */
void NTFY_Exit(void)
{
	/* Do nothing */
}

/*
 *  ======== NTFY_Init ========
 *  Purpose:
 *      Initialize the NTFY module.
 */
bool NTFY_Init(void)
{
	GT_create(&NTFY_debugMask, "NY");	/* "NY" for NtfY */

	return true;
}

/*
 *  ======== NTFY_Notify ========
 *  Purpose:
 *      Execute notify function (signal event) for every
 *      element in the notification list that is to be notified about the
 *      event specified in uEventMask.
 */
void NTFY_Notify(struct NTFY_OBJECT *hNtfy, u32 uEventMask)
{
	struct NOTIFICATION *pNotify;

	DBC_Require(MEM_IsValidHandle(hNtfy, NTFY_SIGNATURE));

	/*
	 *  Go through notifyList and notify all clients registered for
	 *  uEventMask events.
	 */

	(void) SYNC_EnterCS(hNtfy->hSync);

	pNotify = (struct NOTIFICATION *)LST_First(hNtfy->notifyList);
	while (pNotify != NULL) {
		if (pNotify->uEventMask & uEventMask) {
			/* Notify */
			if (pNotify->uNotifyType == DSP_SIGNALEVENT)
				(void)SYNC_SetEvent(pNotify->hSync);

		}
		pNotify = (struct NOTIFICATION *)LST_Next(hNtfy->notifyList,
			  (struct list_head *)pNotify);
	}

	(void) SYNC_LeaveCS(hNtfy->hSync);
}

/*
 *  ======== NTFY_Register ========
 *  Purpose:
 *      Add a notification element to the list. If the notification is already
 *      registered, and uEventMask != 0, the notification will get posted for
 *      events specified in the new event mask. If the notification is already
 *      registered and uEventMask == 0, the notification will be unregistered.
 */
DSP_STATUS NTFY_Register(struct NTFY_OBJECT *hNtfy,
			 struct DSP_NOTIFICATION *hNotification,
			 u32 uEventMask, u32 uNotifyType)
{
	struct NOTIFICATION *pNotify;
	struct SYNC_ATTRS syncAttrs;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(MEM_IsValidHandle(hNtfy, NTFY_SIGNATURE));

	if (hNotification == NULL)
		status = DSP_EHANDLE;

	/* Return DSP_ENOTIMPL if uNotifyType is not supported */
	if (DSP_SUCCEEDED(status)) {
		if (!IsValidNotifyMask(uNotifyType))
			status = DSP_ENOTIMPL;

	}

	if (DSP_FAILED(status))
		return status;

	(void)SYNC_EnterCS(hNtfy->hSync);

	pNotify = (struct NOTIFICATION *)LST_First(hNtfy->notifyList);
	while (pNotify != NULL) {
		/* If there is more than one notification type, each
		 * type may require its own handler code.  */

		if (hNotification->handle == pNotify->hSync) {
			/* found */
			break;
		}
		pNotify = (struct NOTIFICATION *)LST_Next(hNtfy->notifyList,
			  (struct list_head *)pNotify);
	}
	if (pNotify == NULL) {
		/* Not registered */
		if (uEventMask == 0) {
			status = DSP_EVALUE;
		} else {
			/* Allocate NOTIFICATION object, add to list */
			pNotify = MEM_Calloc(sizeof(struct NOTIFICATION),
					     MEM_PAGED);
			if (pNotify == NULL)
				status = DSP_EMEMORY;

		}
		if (DSP_SUCCEEDED(status)) {
			LST_InitElem((struct list_head *)pNotify);
			 /* If there is more than one notification type, each
			 * type may require its own handler code. */
			status = SYNC_OpenEvent(&pNotify->hSync, &syncAttrs);
			hNotification->handle = pNotify->hSync;

			if (DSP_SUCCEEDED(status)) {
				pNotify->uEventMask = uEventMask;
				pNotify->uNotifyType = uNotifyType;
				LST_PutTail(hNtfy->notifyList,
					   (struct list_head *)pNotify);
			} else {
				DeleteNotify(pNotify);
			}
		}
	} else {
		/* Found in list */
		if (uEventMask == 0) {
			/* Remove from list and free */
			LST_RemoveElem(hNtfy->notifyList,
				      (struct list_head *)pNotify);
			DeleteNotify(pNotify);
		} else {
			/* Update notification mask (type shouldn't change) */
			pNotify->uEventMask = uEventMask;
		}
	}
	(void)SYNC_LeaveCS(hNtfy->hSync);
	return status;
}

/*
 *  ======== DeleteNotify ========
 *  Purpose:
 *      Free the notification object.
 */
static void DeleteNotify(struct NOTIFICATION *pNotify)
{
	if (pNotify->hSync)
		(void) SYNC_CloseEvent(pNotify->hSync);

	kfree(pNotify->pstrName);

	kfree(pNotify);
}

