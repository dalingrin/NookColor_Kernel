/*
 * sync.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Provide synchronization services.
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

#ifndef _SYNC_H
#define _SYNC_H

#define SIGNATURECS     0x53435953	/* "SYCS" (in reverse) */
#define SIGNATUREDPCCS  0x53445953	/* "SYDS" (in reverse) */

/* Special timeout value indicating an infinite wait: */
#define SYNC_INFINITE  0xffffffff

/* Maximum string length of a named event */
#define SYNC_MAXNAMELENGTH 32

/* Generic SYNC object: */
	struct SYNC_OBJECT;

/* Generic SYNC CS object: */
struct SYNC_CSOBJECT {
	u32 dwSignature;	/* used for object validation */
	struct semaphore sem;
} ;

/* SYNC object attributes: */
	struct SYNC_ATTRS {
		HANDLE hUserEvent;    /* Platform's User Mode synch. object. */
		HANDLE hKernelEvent;  /* Platform's Kernel Mode sync. object. */
		u32 dwReserved1;	/* For future expansion.   */
		u32 dwReserved2;	/* For future expansion.   */
	} ;

/*
 *  ======== SYNC_CloseEvent ========
 *  Purpose:
 *      Close this event handle, freeing resources allocated in SYNC_OpenEvent
 *      if necessary.
 *  Parameters:
 *      hEvent: Handle to a synchronization event, created/opened in
 *              SYNC_OpenEvent.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EFAIL:      Failed to close event handle.
 *      DSP_EHANDLE:    Invalid handle.
 *  Requires:
 *      SYNC initialized.
 *  Ensures:
 *      Any subsequent usage of hEvent would be invalid.
 */
	extern DSP_STATUS SYNC_CloseEvent(IN struct SYNC_OBJECT *hEvent);

/*
 *  ======== SYNC_DeleteCS ========
 *  Purpose:
 *      Delete a critical section.
 *  Parameters:
 *      hCSObj: critical section handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Invalid handle.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS SYNC_DeleteCS(IN struct SYNC_CSOBJECT *hCSObj);

/*
 *  ======== SYNC_EnterCS ========
 *  Purpose:
 *      Enter the critical section.
 *  Parameters:
 *      hCSObj: critical section handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Invalid handle.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS SYNC_EnterCS(IN struct SYNC_CSOBJECT *hCSObj);

/*
 *  ======== SYNC_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      SYNC initialized.
 *  Ensures:
 *      Resources used by module are freed when cRef reaches zero.
 */
	extern void SYNC_Exit(void);

/*
 *  ======== SYNC_Init ========
 *  Purpose:
 *      Initializes private state of SYNC module.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      SYNC initialized.
 */
	extern bool SYNC_Init(void);

/*
 *  ======== SYNC_InitializeCS ========
 *  Purpose:
 *      Initialize the critical section.
 *  Parameters:
 *      hCSObj: critical section handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Out of memory.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS SYNC_InitializeCS(OUT struct SYNC_CSOBJECT **phCSObj);

/*
 *  ======== SYNC_InitializeDPCCS ========
 *  Purpose:
 *      Initialize the critical section between process context and DPC.
 *  Parameters:
 *      hCSObj: critical section handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Out of memory.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS SYNC_InitializeDPCCS(OUT struct SYNC_CSOBJECT
					       **phCSObj);

/*
 *  ======== SYNC_LeaveCS ========
 *  Purpose:
 *      Leave the critical section.
 *  Parameters:
 *      hCSObj: critical section handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Invalid handle.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS SYNC_LeaveCS(IN struct SYNC_CSOBJECT *hCSObj);

/*
 *  ======== SYNC_OpenEvent ========
 *  Purpose:
 *      Create/open and initialize an event object for thread synchronization,
 *      which is initially in the non-signalled state.
 *  Parameters:
 *      phEvent:    Pointer to location to receive the event object handle.
 *      pAttrs:     Pointer to SYNC_ATTRS object containing initial SYNC
 *                  SYNC_OBJECT attributes.  If this pointer is NULL, then
 *                  SYNC_OpenEvent will create and manage an OS specific
 *                  syncronization object.
 *          pAttrs->hUserEvent:  Platform's User Mode synchronization object.
 *
 *      The behaviour of the SYNC methods depend on the value of
 *      the hUserEvent attr:
 *
 *      1. (hUserEvent == NULL):
 *          A user mode event is created.
 *      2. (hUserEvent != NULL):
 *          A user mode event is supplied by the caller of SYNC_OpenEvent().
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Unable to create user mode event.
 *      DSP_EMEMORY:    Insufficient memory.
 *      DSP_EINVALIDARG SYNC_ATTRS values are invalid.
 *  Requires:
 *      - SYNC initialized.
 *      - phEvent != NULL.
 *  Ensures:
 *      If function succeeded, pEvent->hEvent must be a valid event handle.
 */
	extern DSP_STATUS SYNC_OpenEvent(OUT struct SYNC_OBJECT **phEvent,
					 IN OPTIONAL struct SYNC_ATTRS
					 *pAttrs);

/*
 * ========= SYNC_PostMessage ========
 *  Purpose:
 *      To post a windows message
 *  Parameters:
 *      hWindow:    Handle to the window
 *      uMsg:       Message to be posted
 *  Returns:
 *      DSP_SOK:        Success
 *      DSP_EFAIL:      Post message failed
 *      DSP_EHANDLE:    Invalid Window handle
 *  Requires:
 *      SYNC initialized
 *  Ensures
 */
	extern DSP_STATUS SYNC_PostMessage(IN HANDLE hWindow, IN u32 uMsg);

/*
 *  ======== SYNC_ResetEvent ========
 *  Purpose:
 *      Reset a syncronization event object state to non-signalled.
 *  Parameters:
 *      hEvent:         Handle to a sync event.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EFAIL:      Failed to reset event.
 *      DSP_EHANDLE:    Invalid handle.
 *  Requires:
 *      SYNC initialized.
 *  Ensures:
 */
	extern DSP_STATUS SYNC_ResetEvent(IN struct SYNC_OBJECT *hEvent);

/*
 *  ======== SYNC_SetEvent ========
 *  Purpose:
 *      Signal the event.  Will unblock one waiting thread.
 *  Parameters:
 *      hEvent:         Handle to an event object.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Failed to signal event.
 *      DSP_EHANDLE:    Invalid handle.
 *  Requires:
 *      SYNC initialized.
 *  Ensures:
 */
	extern DSP_STATUS SYNC_SetEvent(IN struct SYNC_OBJECT *hEvent);

/*
 *  ======== SYNC_WaitOnEvent ========
 *  Purpose:
 *      Wait for an event to be signalled, up to the specified timeout.
 *  Parameters:
 *      hEvent:         Handle to an event object.
 *      dwTimeOut:      The time-out interval, in milliseconds.
 *                      The function returns if the interval elapses, even if
 *                      the object's state is nonsignaled.
 *                      If zero, the function tests the object's state and
 *                      returns immediately.
 *                      If SYNC_INFINITE, the function's time-out interval
 *                      never elapses.
 *  Returns:
 *      DSP_SOK:        The object was signalled.
 *      DSP_EHANDLE:    Invalid handle.
 *      SYNC_E_FAIL:    Wait failed, possibly because the process terminated.
 *      SYNC_E_TIMEOUT: Timeout expired while waiting for event to be signalled.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS SYNC_WaitOnEvent(IN struct SYNC_OBJECT *hEvent,
					   IN u32 dwTimeOut);

/*
 *  ======== SYNC_WaitOnMultipleEvents ========
 *  Purpose:
 *      Wait for any of an array of events to be signalled, up to the
 *      specified timeout.
 *      Note: dwTimeOut must be SYNC_INFINITE to signal infinite wait.
 *  Parameters:
 *      hSyncEvents:    Array of handles to event objects.
 *      uCount:         Number of event handles.
 *      dwTimeOut:      The time-out interval, in milliseconds.
 *                      The function returns if the interval elapses, even if
 *                      no event is signalled.
 *                      If zero, the function tests the object's state and
 *                      returns immediately.
 *                      If SYNC_INFINITE, the function's time-out interval
 *                      never elapses.
 *      puIndex:        Location to store index of event that was signalled.
 *  Returns:
 *      DSP_SOK:        The object was signalled.
 *      SYNC_E_FAIL:    Wait failed, possibly because the process terminated.
 *      SYNC_E_TIMEOUT: Timeout expired before event was signalled.
 *      DSP_EMEMORY:    Memory allocation failed.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS SYNC_WaitOnMultipleEvents(IN struct SYNC_OBJECT
						    **hSyncEvents,
						    IN u32 uCount,
						    IN u32 dwTimeout,
						    OUT u32 *puIndex);

#endif				/* _SYNC_H */
