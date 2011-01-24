/*
 * _dcd.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Includes the wrapper functions called directly by the
 * DeviceIOControl interface.
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

#ifndef _WCD_
#define _WCD_

#include <dspbridge/wcdioctl.h>

/*
 *  ======== WCD_CallDevIOCtl ========
 *  Purpose:
 *      Call the (wrapper) function for the corresponding WCD IOCTL.
 *  Parameters:
 *      cmd:        IOCTL id, base 0.
 *      args:       Argument structure.
 *      pResult:
 *  Returns:
 *      DSP_SOK if command called; DSP_EINVALIDARG if command not in IOCTL
 *      table.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS WCD_CallDevIOCtl(unsigned int cmd,
					   union Trapped_Args *args,
					   u32 *pResult, void *pr_ctxt);

/*
 *  ======== WCD_Init ========
 *  Purpose:
 *      Initialize WCD modules, and export WCD services to WMD's.
 *      This procedure is called when the class driver is loaded.
 *  Parameters:
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 */
	extern bool WCD_Init(void);

/*
 *  ======== WCD_InitComplete2 ========
 *  Purpose:
 *      Perform any required WCD, and WMD initialization which
 *      cannot not be performed in WCD_Init(void) or DEV_StartDevice() due
 *      to the fact that some services are not yet
 *      completely initialized.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:        Allow this device to load
 *      DSP_EFAIL:      Failure.
 *  Requires:
 *      WCD initialized.
 *  Ensures:
 */
	extern DSP_STATUS WCD_InitComplete2(void);

/*
 *  ======== WCD_Exit ========
 *  Purpose:
 *      Exit all modules initialized in WCD_Init(void).
 *      This procedure is called when the class driver is unloaded.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      WCD_Init(void) was previously called.
 *  Ensures:
 *      Resources acquired in WCD_Init(void) are freed.
 */
	extern void WCD_Exit(void);

/* MGR wrapper functions */
	extern u32 MGRWRAP_EnumNode_Info(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 MGRWRAP_EnumProc_Info(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 MGRWRAP_RegisterObject(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 MGRWRAP_UnregisterObject(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 MGRWRAP_WaitForBridgeEvents(union Trapped_Args *args,
			void *pr_ctxt);

	extern u32 MGRWRAP_GetProcessResourcesInfo(union Trapped_Args *args,
			void *pr_ctxt);

/* CPRC (Processor) wrapper Functions */
	extern u32 PROCWRAP_Attach(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_Ctrl(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_Detach(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_EnumNode_Info(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 PROCWRAP_EnumResources(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 PROCWRAP_GetState(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_GetTrace(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_Load(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_RegisterNotify(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 PROCWRAP_Start(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_ReserveMemory(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 PROCWRAP_UnReserveMemory(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 PROCWRAP_Map(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_UnMap(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_FlushMemory(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 PROCWRAP_Stop(union Trapped_Args *args, void *pr_ctxt);
	extern u32 PROCWRAP_InvalidateMemory(union Trapped_Args *args,
			void *pr_ctxt);

/* NODE wrapper functions */
	extern u32 NODEWRAP_Allocate(union Trapped_Args *args, void *pr_ctxt);
	extern u32 NODEWRAP_AllocMsgBuf(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 NODEWRAP_ChangePriority(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 NODEWRAP_Connect(union Trapped_Args *args, void *pr_ctxt);
	extern u32 NODEWRAP_Create(union Trapped_Args *args, void *pr_ctxt);
	extern u32 NODEWRAP_Delete(union Trapped_Args *args, void *pr_ctxt);
	extern u32 NODEWRAP_FreeMsgBuf(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 NODEWRAP_GetAttr(union Trapped_Args *args, void *pr_ctxt);
	extern u32 NODEWRAP_GetMessage(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 NODEWRAP_Pause(union Trapped_Args *args, void *pr_ctxt);
	extern u32 NODEWRAP_PutMessage(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 NODEWRAP_RegisterNotify(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 NODEWRAP_Run(union Trapped_Args *args, void *pr_ctxt);
	extern u32 NODEWRAP_Terminate(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 NODEWRAP_GetUUIDProps(union Trapped_Args *args,
			void *pr_ctxt);

/* STRM wrapper functions */
	extern u32 STRMWRAP_AllocateBuffer(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 STRMWRAP_Close(union Trapped_Args *args, void *pr_ctxt);
	extern u32 STRMWRAP_FreeBuffer(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 STRMWRAP_GetEventHandle(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 STRMWRAP_GetInfo(union Trapped_Args *args, void *pr_ctxt);
	extern u32 STRMWRAP_Idle(union Trapped_Args *args, void *pr_ctxt);
	extern u32 STRMWRAP_Issue(union Trapped_Args *args, void *pr_ctxt);
	extern u32 STRMWRAP_Open(union Trapped_Args *args, void *pr_ctxt);
	extern u32 STRMWRAP_Reclaim(union Trapped_Args *args, void *pr_ctxt);
	extern u32 STRMWRAP_RegisterNotify(union Trapped_Args *args,
			void *pr_ctxt);
	extern u32 STRMWRAP_Select(union Trapped_Args *args, void *pr_ctxt);

	extern u32 CMMWRAP_CallocBuf(union Trapped_Args *args, void *pr_ctxt);
	extern u32 CMMWRAP_FreeBuf(union Trapped_Args *args, void *pr_ctxt);
	extern u32 CMMWRAP_GetHandle(union Trapped_Args *args, void *pr_ctxt);
	extern u32 CMMWRAP_GetInfo(union Trapped_Args *args, void *pr_ctxt);

#endif				/* _WCD_ */
