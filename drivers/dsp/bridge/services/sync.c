/*
 * sync.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Synchronization services.
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
#include <dspbridge/mem.h>

/*  ----------------------------------- This */
#include <dspbridge/sync.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define SIGNATURE       0x434e5953	/* "SYNC" (in reverse) */

enum wait_state {
	wo_waiting,
	wo_signalled
} ;

enum sync_state {
	so_reset,
	so_signalled
} ;

struct WAIT_OBJECT {
	enum wait_state state;
	struct SYNC_OBJECT *signalling_event;
	struct semaphore sem;
};

/* Generic SYNC object: */
struct SYNC_OBJECT {
	u32 dwSignature;	/* Used for object validation. */
	enum sync_state state;
	spinlock_t sync_lock;
	struct WAIT_OBJECT *pWaitObj;
};

struct SYNC_DPCCSOBJECT {
	u32 dwSignature;	/* used for object validation */
	spinlock_t sync_dpccs_lock;
	s32 count;
} ;

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask SYNC_debugMask = { NULL, NULL };  /* GT trace variable */
#endif

static int test_and_set(volatile void *ptr, int val)
{
	int ret = val;
	asm volatile (" swp %0, %0, [%1]" : "+r" (ret) : "r"(ptr) : "memory");
	return ret;
}

static void timeout_callback(unsigned long hWaitObj);

/*
 *  ======== SYNC_CloseEvent ========
 *  Purpose:
 *      Close an existing SYNC event object.
 */
DSP_STATUS SYNC_CloseEvent(struct SYNC_OBJECT *hEvent)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_OBJECT *pEvent = (struct SYNC_OBJECT *)hEvent;

	DBC_Require(pEvent != NULL && pEvent->pWaitObj == NULL);

	if (MEM_IsValidHandle(hEvent, SIGNATURE)) {
		if (pEvent->pWaitObj)
			status = DSP_EFAIL;

		MEM_FreeObject(pEvent);

	} else {
		status = DSP_EHANDLE;
	}

	return status;
}

/*
 *  ======== SYNC_Exit ========
 *  Purpose:
 *      Cleanup SYNC module.
 */
void SYNC_Exit(void)
{
	/* Do nothing */
}

/*
 *  ======== SYNC_Init ========
 *  Purpose:
 *      Initialize SYNC module.
 */
bool SYNC_Init(void)
{
	GT_create(&SYNC_debugMask, "SY");	/* SY for SYnc */

	return true;
}

/*
 *  ======== SYNC_OpenEvent ========
 *  Purpose:
 *      Open a new synchronization event object.
 */
DSP_STATUS SYNC_OpenEvent(OUT struct SYNC_OBJECT **phEvent,
			  IN OPTIONAL struct SYNC_ATTRS *pAttrs)
{
	struct SYNC_OBJECT *pEvent = NULL;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(phEvent != NULL);

	/* Allocate memory for sync object */
	MEM_AllocObject(pEvent, struct SYNC_OBJECT, SIGNATURE);
	if (pEvent != NULL) {
		pEvent->state = so_reset;
		pEvent->pWaitObj = NULL;
		spin_lock_init(&pEvent->sync_lock);
	} else {
		status = DSP_EMEMORY;
	}

	*phEvent = pEvent;

	return status;
}

/*
 *  ======== SYNC_ResetEvent ========
 *  Purpose:
 *      Reset an event to non-signalled.
 */
DSP_STATUS SYNC_ResetEvent(struct SYNC_OBJECT *hEvent)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_OBJECT *pEvent = (struct SYNC_OBJECT *)hEvent;

	if (MEM_IsValidHandle(hEvent, SIGNATURE))
		pEvent->state = so_reset;
	else
		status = DSP_EHANDLE;

	return status;
}

/*
 *  ======== SYNC_SetEvent ========
 *  Purpose:
 *      Set an event to signaled and unblock one waiting thread.
 *
 *  This function is called from ISR, DPC and user context. Hence interrupts
 *  are disabled to ensure atomicity.
 */

DSP_STATUS SYNC_SetEvent(struct SYNC_OBJECT *hEvent)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_OBJECT *pEvent = (struct SYNC_OBJECT *)hEvent;
	unsigned long flags;

	if (MEM_IsValidHandle(hEvent, SIGNATURE)) {
		spin_lock_irqsave(&hEvent->sync_lock, flags);

		if (pEvent->pWaitObj != NULL &&
		   test_and_set(&pEvent->pWaitObj->state,
		   wo_signalled) == wo_waiting) {
			pEvent->state = so_reset;
			pEvent->pWaitObj->signalling_event = pEvent;
			up(&pEvent->pWaitObj->sem);
		} else {
			pEvent->state = so_signalled;
		}
		spin_unlock_irqrestore(&hEvent->sync_lock, flags);
	} else {
		status = DSP_EHANDLE;
	}
	return status;
}

/*
 *  ======== SYNC_WaitOnEvent ========
 *  Purpose:
 *      Wait for an event to be signalled, up to the specified timeout.
 *      Note: dwTimeOut must be 0xffffffff to signal infinite wait.
 */
DSP_STATUS SYNC_WaitOnEvent(struct SYNC_OBJECT *hEvent, u32 dwTimeout)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_OBJECT *pEvent = (struct SYNC_OBJECT *)hEvent;
	u32 temp;

	if (MEM_IsValidHandle(hEvent, SIGNATURE))
		status = SYNC_WaitOnMultipleEvents(&pEvent, 1, dwTimeout,
						  &temp);
	else
		status = DSP_EHANDLE;

	return status;
}

/*
 *  ======== SYNC_WaitOnMultipleEvents ========
 *  Purpose:
 *      Wait for any of an array of events to be signalled, up to the
 *      specified timeout.
 */
DSP_STATUS SYNC_WaitOnMultipleEvents(struct SYNC_OBJECT **hSyncEvents,
				     u32 uCount, u32 dwTimeout,
				     OUT u32 *puIndex)
{
	u32 i;
	DSP_STATUS status = DSP_SOK;
	u32 curr;
	struct WAIT_OBJECT *Wp;

	DBC_Require(uCount > 0);
	DBC_Require(hSyncEvents != NULL);
	DBC_Require(puIndex != NULL);

	for (i = 0; i < uCount; i++)
		DBC_Require(MEM_IsValidHandle(hSyncEvents[i], SIGNATURE));

	Wp = MEM_Calloc(sizeof(struct WAIT_OBJECT), MEM_NONPAGED);
	if (Wp == NULL)
		return DSP_EMEMORY;

	Wp->state = wo_waiting;
	Wp->signalling_event = NULL;
	init_MUTEX_LOCKED(&(Wp->sem));

	for (curr = 0; curr < uCount; curr++) {
		hSyncEvents[curr]->pWaitObj = Wp;
		if (hSyncEvents[curr]->state == so_signalled) {
			if (test_and_set(&(Wp->state), wo_signalled) ==
			   wo_waiting) {
				hSyncEvents[curr]->state = so_reset;
				Wp->signalling_event = hSyncEvents[curr];
			}
		curr++;	/* Will try optimizing later */
		break;
		}
	}

	curr--;			/* Will try optimizing later */
	if (Wp->state != wo_signalled && dwTimeout > 0) {
		struct timer_list timeout;
		if (dwTimeout != SYNC_INFINITE) {
			init_timer_on_stack(&timeout);
			timeout.function = timeout_callback;
			timeout.data = (unsigned long)Wp;
			timeout.expires = jiffies + dwTimeout * HZ / 1000;
			add_timer(&timeout);
		}
		if (down_interruptible(&(Wp->sem)))
			status = DSP_EFAIL;

		if (dwTimeout != SYNC_INFINITE) {
			if (in_interrupt())
				del_timer(&timeout);
			else
				del_timer_sync(&timeout);
		}
	}
	for (i = 0; i <= curr; i++) {
		if (MEM_IsValidHandle(hSyncEvents[i], SIGNATURE)) {
			/*  Memory corruption here if hSyncEvents[i] is
			 *  freed before following statememt. */
			hSyncEvents[i]->pWaitObj = NULL;
		}
		if (hSyncEvents[i] == Wp->signalling_event)
			*puIndex = i;

	}
	if (Wp->signalling_event == NULL && DSP_SUCCEEDED(status))
		status = DSP_ETIMEOUT;
	kfree(Wp);
	return status;
}

static void timeout_callback(unsigned long hWaitObj)
{
	struct WAIT_OBJECT *pWaitObj = (struct WAIT_OBJECT *)hWaitObj;
	if (test_and_set(&pWaitObj->state, wo_signalled) == wo_waiting)
		up(&pWaitObj->sem);

}

/*
 *  ======== SYNC_DeleteCS ========
 */
DSP_STATUS SYNC_DeleteCS(struct SYNC_CSOBJECT *hCSObj)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_CSOBJECT *pCSObj = (struct SYNC_CSOBJECT *)hCSObj;

	if (MEM_IsValidHandle(hCSObj, SIGNATURECS)) {
		if (down_trylock(&pCSObj->sem) != 0)
			DBC_Assert(0);

		MEM_FreeObject(hCSObj);
	} else if (MEM_IsValidHandle(hCSObj, SIGNATUREDPCCS)) {
		struct SYNC_DPCCSOBJECT *pDPCCSObj =
					 (struct SYNC_DPCCSOBJECT *)hCSObj;
		if (pDPCCSObj->count != 1)
			DBC_Assert(0);

		MEM_FreeObject(pDPCCSObj);
	} else {
		status = DSP_EHANDLE;
	}

	return status;
}

/*
 *  ======== SYNC_EnterCS ========
 */
DSP_STATUS SYNC_EnterCS(struct SYNC_CSOBJECT *hCSObj)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_CSOBJECT *pCSObj = (struct SYNC_CSOBJECT *)hCSObj;

	if (MEM_IsValidHandle(hCSObj, SIGNATURECS)) {
		if (in_interrupt()) {
			status = DSP_EFAIL;
			DBC_Assert(0);
		} else if (down_interruptible(&pCSObj->sem)) {
			status = DSP_EFAIL;
		}
	} else if (MEM_IsValidHandle(hCSObj, SIGNATUREDPCCS)) {
		struct SYNC_DPCCSOBJECT *pDPCCSObj =
					(struct SYNC_DPCCSOBJECT *)hCSObj;
		spin_lock_bh(&pDPCCSObj->sync_dpccs_lock);
		pDPCCSObj->count--;
		if (pDPCCSObj->count != 0) {
			/* FATAL ERROR : Failed to acquire DPC CS */
			spin_unlock_bh(&pDPCCSObj->sync_dpccs_lock);
			DBC_Assert(0);
		}
	} else {
		status = DSP_EHANDLE;
	}

	return status;
}

/*
 *  ======== SYNC_InitializeCS ========
 */
DSP_STATUS SYNC_InitializeCS(OUT struct SYNC_CSOBJECT **phCSObj)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_CSOBJECT *pCSObj = NULL;

	/* Allocate memory for sync CS object */
	MEM_AllocObject(pCSObj, struct SYNC_CSOBJECT, SIGNATURECS);
	if (pCSObj != NULL)
		init_MUTEX(&pCSObj->sem);
	else
		status = DSP_EMEMORY;

	/* return CS object */
	*phCSObj = pCSObj;
	DBC_Assert(DSP_FAILED(status) || (pCSObj));
	return status;
}

DSP_STATUS SYNC_InitializeDPCCS(OUT struct SYNC_CSOBJECT **phCSObj)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_DPCCSOBJECT *pCSObj = NULL;

	DBC_Require(phCSObj);

	if (phCSObj) {
		/* Allocate memory for sync CS object */
		MEM_AllocObject(pCSObj, struct SYNC_DPCCSOBJECT,
				SIGNATUREDPCCS);
		if (pCSObj != NULL) {
			pCSObj->count = 1;
			spin_lock_init(&pCSObj->sync_dpccs_lock);
		} else {
			status = DSP_EMEMORY;
		}

		/* return CS object */
		*phCSObj = (struct SYNC_CSOBJECT *)pCSObj;
	} else {
		status = DSP_EPOINTER;
	}

	DBC_Assert(DSP_FAILED(status) || (pCSObj));

	return status;
}

/*
 *  ======== SYNC_LeaveCS ========
 */
DSP_STATUS SYNC_LeaveCS(struct SYNC_CSOBJECT *hCSObj)
{
	DSP_STATUS status = DSP_SOK;
	struct SYNC_CSOBJECT *pCSObj = (struct SYNC_CSOBJECT *)hCSObj;

	if (MEM_IsValidHandle(hCSObj, SIGNATURECS)) {
		up(&pCSObj->sem);
	} else if (MEM_IsValidHandle(hCSObj, SIGNATUREDPCCS)) {
		struct SYNC_DPCCSOBJECT *pDPCCSObj =
					(struct SYNC_DPCCSOBJECT *)hCSObj;
		pDPCCSObj->count++;
		if (pDPCCSObj->count != 1) {
			/* FATAL ERROR : Invalid DPC CS count */
			spin_unlock_bh(&pDPCCSObj->sync_dpccs_lock);
			DBC_Assert(0);
			spin_lock_bh(&pDPCCSObj->sync_dpccs_lock);
		}
		spin_unlock_bh(&pDPCCSObj->sync_dpccs_lock);
	} else {
		status = DSP_EHANDLE;
	}

	return status;
}
