/*
 * io_sm.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * IO dispatcher for a shared memory channel driver.
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

/*
 * Channel Invariant:
 * There is an important invariant condition which must be maintained per
 * channel outside of WMD_CHNL_GetIOC() and IO_Dispatch(), violation of
 * which may cause timeouts and/or failure of the SYNC_WaitOnEvent
 * function.
 */

/* Host OS */
#include <dspbridge/host_os.h>
#include <linux/workqueue.h>

#ifdef CONFIG_BRIDGE_DVFS
#include <mach/omap-pm.h>
#endif

/* DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/* Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/dbg.h>

/* Services Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/mem.h>
#include <dspbridge/ntfy.h>
#include <dspbridge/sync.h>
#include <dspbridge/reg.h>

/* Hardware Abstraction Layer */
#include <hw_defs.h>
#include <hw_mmu.h>

/* Mini Driver */
#include <dspbridge/wmddeh.h>
#include <dspbridge/wmdio.h>
#include <dspbridge/wmdioctl.h>
#include <_tiomap.h>
#include <tiomap_io.h>
#include <_tiomap_pwr.h>

/* Platform Manager */
#include <dspbridge/cod.h>
#include <dspbridge/node.h>
#include <dspbridge/dev.h>

/* Others */
#include <dspbridge/rms_sh.h>
#include <dspbridge/mgr.h>
#include <dspbridge/drv.h>
#include "_cmm.h"

/* This */
#include <dspbridge/io_sm.h>
#include "_msg_sm.h"
#include <dspbridge/gt.h>
#include "module_list.h"

/* Defines, Data Structures, Typedefs */
#define OUTPUTNOTREADY  0xffff
#define NOTENABLED      0xffff	/* Channel(s) not enabled */

#define EXTEND      "_EXT_END"

#define SwapWord(x)     (x)
#define ulPageAlignSize 0x10000   /* Page Align Size */

#define MAX_PM_REQS 32

#define MMU_FAULT_HEAD1 0xa5a5a5a5
#define MMU_FAULT_HEAD2 0x96969696
#define POLL_MAX 1000
#define MAX_MMU_DBGBUFF 10240

/* IO Manager: only one created per board */
struct IO_MGR {
	/* These four fields must be the first fields in a IO_MGR_ struct */
	u32 dwSignature; 	/* Used for object validation */
	struct WMD_DEV_CONTEXT *hWmdContext; 	/* WMD device context */
	struct WMD_DRV_INTERFACE *pIntfFxns; 	/* Function interface to WMD */
	struct DEV_OBJECT *hDevObject; 	/* Device this board represents */

	/* These fields initialized in WMD_IO_Create() */
	struct CHNL_MGR *hChnlMgr;
	struct SHM *pSharedMem; 	/* Shared Memory control */
	u8 *pInput; 		/* Address of input channel */
	u8 *pOutput; 		/* Address of output channel */
	struct MSG_MGR *hMsgMgr; 	/* Message manager */
	struct MSG *pMsgInputCtrl; 	/* Msg control for from DSP messages */
	struct MSG *pMsgOutputCtrl; 	/* Msg control for to DSP messages */
	u8 *pMsgInput; 	/* Address of input messages */
	u8 *pMsgOutput; 	/* Address of output messages */
	u32 uSMBufSize; 	/* Size of a shared memory I/O channel */
	bool fSharedIRQ; 	/* Is this IRQ shared? */
	struct SYNC_CSOBJECT *hCSObj; 	/* Critical section object handle */
	u32 uWordSize; 	/* Size in bytes of DSP word */
	u16 wIntrVal; 		/* Interrupt value */
	/* Private extnd proc info; mmu setup */
	struct MGR_PROCESSOREXTINFO extProcInfo;
	struct CMM_OBJECT *hCmmMgr; 	/* Shared Mem Mngr */
	struct work_struct io_workq;     /* workqueue */
#ifndef DSP_TRACEBUF_DISABLED
	u32 ulTraceBufferBegin; 	/* Trace message start address */
	u32 ulTraceBufferEnd; 	/* Trace message end address */
	u32 ulTraceBufferCurrent; 	/* Trace message current address */
	u32 ulGPPReadPointer; 	/* GPP Read pointer to Trace buffer */
	u8 *pMsg;
	u32 ulGppVa;
	u32 ulDspVa;
#endif
	/* IO Dpc */
	u32 dpc_req;				/* Number of requested DPC's. */
	u32 dpc_sched;				/* Number of executed DPC's. */
	struct tasklet_struct dpc_tasklet;
#ifdef CONFIG_BRIDGE_WDT3
	struct tasklet_struct wdt3_tasklet;
#endif
	spinlock_t dpc_lock;

} ;

/* Function Prototypes */
static void IO_DispatchChnl(IN struct IO_MGR *pIOMgr,
			   IN OUT struct CHNL_OBJECT *pChnl, u32 iMode);
static void IO_DispatchMsg(IN struct IO_MGR *pIOMgr, struct MSG_MGR *hMsgMgr);
static void IO_DispatchPM(struct IO_MGR *pIOMgr);
static void NotifyChnlComplete(struct CHNL_OBJECT *pChnl,
				struct CHNL_IRP *pChirp);
static void InputChnl(struct IO_MGR *pIOMgr, struct CHNL_OBJECT *pChnl,
			u32 iMode);
static void OutputChnl(struct IO_MGR *pIOMgr, struct CHNL_OBJECT *pChnl,
			u32 iMode);
static void InputMsg(struct IO_MGR *pIOMgr, struct MSG_MGR *hMsgMgr);
static void OutputMsg(struct IO_MGR *pIOMgr, struct MSG_MGR *hMsgMgr);
static u32 FindReadyOutput(struct CHNL_MGR *pChnlMgr,
			     struct CHNL_OBJECT *pChnl, u32 dwMask);
static u32 ReadData(struct WMD_DEV_CONTEXT *hDevContext, void *pDest,
			void *pSrc, u32 uSize);
static u32 WriteData(struct WMD_DEV_CONTEXT *hDevContext, void *pDest,
			void *pSrc, u32 uSize);

#ifndef DSP_TRACEBUF_DISABLED
void PrintDSPDebugTrace(struct IO_MGR *hIOMgr);
#endif

#ifdef CONFIG_BRIDGE_WDT3
static bool wdt3_enable = true;
static void io_wdt3_ovf(unsigned long);
#endif

/* Bus Addr (cached kernel) */
static DSP_STATUS registerSHMSegs(struct IO_MGR *hIOMgr,
				  struct COD_MANAGER *hCodMan,
				  u32 dwGPPBasePA);

#if GT_TRACE
static struct GT_Mask dsp_trace_mask = { NULL, NULL }; /* GT trace variable */
#endif

/*
 *  ======== WMD_IO_Create ========
 *      Create an IO manager object.
 */
DSP_STATUS WMD_IO_Create(OUT struct IO_MGR **phIOMgr,
			 struct DEV_OBJECT *hDevObject,
			 IN CONST struct IO_ATTRS *pMgrAttrs)
{
	DSP_STATUS status = DSP_SOK;
	struct IO_MGR *pIOMgr = NULL;
	struct SHM *pSharedMem = NULL;
	struct WMD_DEV_CONTEXT *hWmdContext = NULL;
	struct CFG_HOSTRES hostRes;
	struct CFG_DEVNODE *hDevNode;
	struct CHNL_MGR *hChnlMgr;
	u32 devType;
	/* Check requirements */
	if (!phIOMgr || !pMgrAttrs || pMgrAttrs->uWordSize == 0) {
		status = DSP_EHANDLE;
		goto func_end;
	}
	DEV_GetChnlMgr(hDevObject, &hChnlMgr);
	if (!hChnlMgr || hChnlMgr->hIOMgr) {
		status = DSP_EHANDLE;
		goto func_end;
	}
	/*
	 * Message manager will be created when a file is loaded, since
	 * size of message buffer in shared memory is configurable in
	 * the base image.
	 */
	DEV_GetWMDContext(hDevObject, &hWmdContext);
	if (!hWmdContext) {
		status = DSP_EHANDLE;
		goto func_end;
	}
	DEV_GetDevType(hDevObject, &devType);
	/*
	 * DSP shared memory area will get set properly when
	 * a program is loaded. They are unknown until a COFF file is
	 * loaded. I chose the value -1 because it was less likely to be
	 * a valid address than 0.
	 */
	pSharedMem = (struct SHM *) -1;

	/* Allocate IO manager object */
	MEM_AllocObject(pIOMgr, struct IO_MGR, IO_MGRSIGNATURE);
	if (pIOMgr == NULL) {
		status = DSP_EMEMORY;
		goto func_end;
	}


	/* Initialize CHNL_MGR object */
#ifndef DSP_TRACEBUF_DISABLED
	pIOMgr->pMsg = NULL;
#endif
	pIOMgr->hChnlMgr = hChnlMgr;
	pIOMgr->uWordSize = pMgrAttrs->uWordSize;
	pIOMgr->pSharedMem = pSharedMem;
	if (DSP_SUCCEEDED(status))
		status = SYNC_InitializeCS(&pIOMgr->hCSObj);

	if (devType == DSP_UNIT) {
		/* Create an IO DPC */
		tasklet_init(&pIOMgr->dpc_tasklet, IO_DPC, (u32)pIOMgr);
#ifdef CONFIG_BRIDGE_WDT3
		tasklet_init(&pIOMgr->wdt3_tasklet, io_wdt3_ovf, (u32)pIOMgr);
#endif
		/* Initialize DPC counters */
		pIOMgr->dpc_req = 0;
		pIOMgr->dpc_sched = 0;

		spin_lock_init(&pIOMgr->dpc_lock);

		if (DSP_SUCCEEDED(status))
			status = DEV_GetDevNode(hDevObject, &hDevNode);

	}
	if (DSP_SUCCEEDED(status)) {
		status = CFG_GetHostResources((struct CFG_DEVNODE *)
				DRV_GetFirstDevExtension() , &hostRes);
	}
	if (DSP_SUCCEEDED(status)) {
		pIOMgr->hWmdContext = hWmdContext;
		pIOMgr->fSharedIRQ = pMgrAttrs->fShared;

	} else {
		status = CHNL_E_ISR;
	}
#ifdef CONFIG_BRIDGE_WDT3
	if (DSP_SUCCEEDED(status)) {
		if ((request_irq(INT_34XX_WDT3_IRQ, io_isr_wdt3, 0,
			  "dsp_wdt", (void *)pIOMgr)) != 0)
			status = DSP_EFAIL;
		else
		/* Disable at this moment I will enable when DSP starts */
			disable_irq(INT_34XX_WDT3_IRQ);
	}
#endif
func_end:
	if (DSP_FAILED(status)) {
		/* Cleanup */
		WMD_IO_Destroy(pIOMgr);
		if (phIOMgr)
			*phIOMgr = NULL;
	} else {
		/* Return IO manager object to caller... */
		hChnlMgr->hIOMgr = pIOMgr;
		*phIOMgr = pIOMgr;
	}
	return status;
}

/*
 *  ======== WMD_IO_Destroy ========
 *  Purpose:
 *      Disable interrupts, destroy the IO manager.
 */
DSP_STATUS WMD_IO_Destroy(struct IO_MGR *hIOMgr)
{
	DSP_STATUS status = DSP_SOK;
	struct WMD_DEV_CONTEXT *hWmdContext;
	if (MEM_IsValidHandle(hIOMgr, IO_MGRSIGNATURE)) {
		/* Disable interrupts from the board */
		status = DEV_GetWMDContext(hIOMgr->hDevObject, &hWmdContext);
#ifdef CONFIG_BRIDGE_WDT3
		free_irq(INT_34XX_WDT3_IRQ, (void *)hIOMgr);
#endif
		/* Free IO DPC object */
		tasklet_kill(&hIOMgr->dpc_tasklet);
#ifdef CONFIG_BRIDGE_WDT3
		tasklet_kill(&hIOMgr->wdt3_tasklet);
#endif
#ifndef DSP_TRACEBUF_DISABLED
		kfree(hIOMgr->pMsg);
#endif
		SYNC_DeleteCS(hIOMgr->hCSObj); 	/* Leak Fix. */
		/* Free this IO manager object */
		MEM_FreeObject(hIOMgr);
	} else {
		status = DSP_EHANDLE;
	}

	return status;
}

/*
 *  ======== WMD_IO_OnLoaded ========
 *  Purpose:
 *      Called when a new program is loaded to get shared memory buffer
 *      parameters from COFF file. ulSharedBufferBase and ulSharedBufferLimit
 *      are in DSP address units.
 */
DSP_STATUS WMD_IO_OnLoaded(struct IO_MGR *hIOMgr)
{
	struct COD_MANAGER *hCodMan;
	struct CHNL_MGR *hChnlMgr;
	struct MSG_MGR *hMsgMgr;
	u32 ulShmBase;
	u32 ulShmBaseOffset;
	u32 ulShmLimit;
	u32 ulShmLength = -1;
	u32 ulMemLength = -1;
	u32 ulMsgBase;
	u32 ulMsgLimit;
	u32 ulMsgLength = -1;
	u32 ulExtEnd;
	u32 ulGppPa = 0;
	u32 ulGppVa = 0;
	u32 ulDspVa = 0;
	u32 ulSegSize = 0;
	u32 ulPadSize = 0;
	u32 i;
	DSP_STATUS status = DSP_SOK;
	u32 uNumProcs = 0;
	s32 ndx = 0;
	/* DSP MMU setup table */
	struct WMDIOCTL_EXTPROC aEProc[WMDIOCTL_NUMOFMMUTLB];
	struct CFG_HOSTRES hostRes;
	u32 mapAttrs;
	u32 ulShm0End;
	u32 ulDynExtBase;
	u32 ulSeg1Size = 0;
	u32 paCurr = 0;
	u32 vaCurr = 0;
	u32 gppVaCurr = 0;
	u32 numBytes = 0;
	u32 allBits = 0;
	u32 pgSize[] = { HW_PAGE_SIZE_16MB, HW_PAGE_SIZE_1MB,
			   HW_PAGE_SIZE_64KB, HW_PAGE_SIZE_4KB };

	status = DEV_GetCodMgr(hIOMgr->hDevObject, &hCodMan);
	if (!hCodMan) {
		status = DSP_EHANDLE;
		goto func_end;
	}
	hChnlMgr = hIOMgr->hChnlMgr;
	/* The message manager is destroyed when the board is stopped. */
	DEV_GetMsgMgr(hIOMgr->hDevObject, &hIOMgr->hMsgMgr);
	hMsgMgr = hIOMgr->hMsgMgr;
	if (!MEM_IsValidHandle(hChnlMgr, CHNL_MGRSIGNATURE) ||
	   !MEM_IsValidHandle(hMsgMgr, MSGMGR_SIGNATURE)) {
		status = DSP_EHANDLE;
		goto func_end;
	}
	if (hIOMgr->pSharedMem)
		hIOMgr->pSharedMem = NULL;

	/* Get start and length of channel part of shared memory */
	status = COD_GetSymValue(hCodMan, CHNL_SHARED_BUFFER_BASE_SYM,
				 &ulShmBase);
	if (DSP_FAILED(status)) {
		status = CHNL_E_NOMEMMAP;
		goto func_end;
	}
	status = COD_GetSymValue(hCodMan, CHNL_SHARED_BUFFER_LIMIT_SYM,
				&ulShmLimit);
	if (DSP_FAILED(status)) {
		status = CHNL_E_NOMEMMAP;
		goto func_end;
	}
	if (ulShmLimit <= ulShmBase) {
		status = CHNL_E_INVALIDMEMBASE;
		goto func_end;
	}
	/* Get total length in bytes */
	ulShmLength = (ulShmLimit - ulShmBase + 1) * hIOMgr->uWordSize;
	/* Calculate size of a PROCCOPY shared memory region */
	DBG_Trace(DBG_LEVEL7,
		 "**(proc)PROCCOPY SHMMEM SIZE: 0x%x bytes\n",
		  (ulShmLength - sizeof(struct SHM)));

	if (DSP_SUCCEEDED(status)) {
		/* Get start and length of message part of shared memory */
		status = COD_GetSymValue(hCodMan, MSG_SHARED_BUFFER_BASE_SYM,
					&ulMsgBase);
	}
	if (DSP_SUCCEEDED(status)) {
		status = COD_GetSymValue(hCodMan, MSG_SHARED_BUFFER_LIMIT_SYM,
					&ulMsgLimit);
		if (DSP_SUCCEEDED(status)) {
			if (ulMsgLimit <= ulMsgBase) {
				status = CHNL_E_INVALIDMEMBASE;
			} else {
				/*
				 * Length (bytes) of messaging part of shared
				 * memory.
				 */
				ulMsgLength = (ulMsgLimit - ulMsgBase + 1) *
					      hIOMgr->uWordSize;
				/*
				 * Total length (bytes) of shared memory:
				 * chnl + msg.
				 */
				ulMemLength = ulShmLength + ulMsgLength;
			}
		} else {
			status = CHNL_E_NOMEMMAP;
		}
	} else {
		status = CHNL_E_NOMEMMAP;
	}
	if (DSP_SUCCEEDED(status)) {
#ifndef DSP_TRACEBUF_DISABLED
		status = COD_GetSymValue(hCodMan, DSP_TRACESEC_END, &ulShm0End);
#else
		status = COD_GetSymValue(hCodMan, SHM0_SHARED_END_SYM,
					 &ulShm0End);
#endif
		if (DSP_FAILED(status))
			status = CHNL_E_NOMEMMAP;
	}
	if (DSP_SUCCEEDED(status)) {
		status = COD_GetSymValue(hCodMan, DYNEXTBASE, &ulDynExtBase);
		if (DSP_FAILED(status))
			status = CHNL_E_NOMEMMAP;
	}
	if (DSP_SUCCEEDED(status)) {
		status = COD_GetSymValue(hCodMan, EXTEND, &ulExtEnd);
		if (DSP_FAILED(status))
			status = CHNL_E_NOMEMMAP;
	}
	if (DSP_SUCCEEDED(status)) {
		/* Get memory reserved in host resources */
		(void)MGR_EnumProcessorInfo(0,
			(struct DSP_PROCESSORINFO *)&hIOMgr->extProcInfo,
			sizeof(struct MGR_PROCESSOREXTINFO), &uNumProcs);
		CFG_GetHostResources((
			struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&hostRes);
		/* The first MMU TLB entry(TLB_0) in DCD is ShmBase. */
		ndx = 0;
		ulGppPa = hostRes.dwMemPhys[1];
		ulGppVa = hostRes.dwMemBase[1];
		/* This is the virtual uncached ioremapped address!!! */
		/* Why can't we directly take the DSPVA from the symbols? */
		ulDspVa = hIOMgr->extProcInfo.tyTlb[0].ulDspVirt;
		ulSegSize = (ulShm0End - ulDspVa) * hIOMgr->uWordSize;
		ulSeg1Size = (ulExtEnd - ulDynExtBase) * hIOMgr->uWordSize;
		ulSeg1Size = (ulSeg1Size + 0xFFF) & (~0xFFFUL); /* 4K align*/
		ulSegSize = (ulSegSize + 0xFFFF) & (~0xFFFFUL); /* 64K align*/
		ulPadSize = ulPageAlignSize - ((ulGppPa + ulSeg1Size) %
			     ulPageAlignSize);
			if (ulPadSize == ulPageAlignSize)
				ulPadSize = 0x0;

		 DBG_Trace(DBG_LEVEL7, "ulGppPa %x, ulGppVa %x, ulDspVa %x, "
			  "ulShm0End %x, ulDynExtBase %x, ulExtEnd %x, "
			  "ulSegSize %x ulSeg1Size %x \n", ulGppPa, ulGppVa,
			  ulDspVa, ulShm0End, ulDynExtBase, ulExtEnd, ulSegSize,
			  ulSeg1Size);

		if ((ulSegSize + ulSeg1Size + ulPadSize) >
		   hostRes.dwMemLength[1]) {
			pr_err("%s: SHM Error, reserved 0x%x required 0x%x\n",
					__func__, hostRes.dwMemLength[1],
					ulSegSize + ulSeg1Size + ulPadSize);
			status = DSP_EMEMORY;
		}
	}
	if (DSP_FAILED(status))
		goto func_end;

	paCurr = ulGppPa;
	vaCurr = ulDynExtBase * hIOMgr->uWordSize;
	gppVaCurr = ulGppVa;
	numBytes = ulSeg1Size;

	/*
	 * Try to fit into TLB entries. If not possible, push them to page
	 * tables. It is quite possible that if sections are not on
	 * bigger page boundary, we may end up making several small pages.
	 * So, push them onto page tables, if that is the case.
	 */
	mapAttrs = 0x00000000;
	mapAttrs = DSP_MAPLITTLEENDIAN;
	mapAttrs |= DSP_MAPPHYSICALADDR;
	mapAttrs |= DSP_MAPELEMSIZE32;
	mapAttrs |= DSP_MAPDONOTLOCK;

	while (numBytes) {
		/*
		 * To find the max. page size with which both PA & VA are
		 * aligned.
		 */
		allBits = paCurr | vaCurr;
		DBG_Trace(DBG_LEVEL1, "allBits %x, paCurr %x, vaCurr %x, "
			 "numBytes %x\n", allBits, paCurr, vaCurr, numBytes);
		for (i = 0; i < 4; i++) {
			if ((numBytes >= pgSize[i]) && ((allBits &
			   (pgSize[i] - 1)) == 0)) {
				status = hIOMgr->pIntfFxns->pfnBrdMemMap
					(hIOMgr->hWmdContext, paCurr, vaCurr,
					pgSize[i], mapAttrs);
				if (DSP_FAILED(status))
					goto func_end;
				paCurr += pgSize[i];
				vaCurr += pgSize[i];
				gppVaCurr += pgSize[i];
				numBytes -= pgSize[i];
				/*
				 * Don't try smaller sizes. Hopefully we have
				 * reached an address aligned to a bigger page
				 * size.
				 */
				break;
			}
		}
	}
	paCurr += ulPadSize;
	vaCurr += ulPadSize;
	gppVaCurr += ulPadSize;

	/* Configure the TLB entries for the next cacheable segment */
	numBytes = ulSegSize;
	vaCurr = ulDspVa * hIOMgr->uWordSize;
	while (numBytes) {
		/*
		 * To find the max. page size with which both PA & VA are
		 * aligned.
		 */
		allBits = paCurr | vaCurr;
		DBG_Trace(DBG_LEVEL1, "allBits for Seg1 %x, paCurr %x, "
			 "vaCurr %x, numBytes %x\n", allBits, paCurr, vaCurr,
			 numBytes);
		for (i = 0; i < 4; i++) {
			if (!(numBytes >= pgSize[i]) ||
			   !((allBits & (pgSize[i]-1)) == 0))
				continue;
			if (ndx < MAX_LOCK_TLB_ENTRIES) {
				/*
				 * This is the physical address written to
				 * DSP MMU.
				 */
				aEProc[ndx].ulGppPa = paCurr;
				/*
				 * This is the virtual uncached ioremapped
				 * address!!!
				 */
				aEProc[ndx].ulGppVa = gppVaCurr;
				aEProc[ndx].ulDspVa = vaCurr / hIOMgr->
						      uWordSize;
				aEProc[ndx].ulSize = pgSize[i];
				aEProc[ndx].endianism = HW_LITTLE_ENDIAN;
				aEProc[ndx].elemSize = HW_ELEM_SIZE_16BIT;
				aEProc[ndx].mixedMode = HW_MMU_CPUES;
				DBG_Trace(DBG_LEVEL1, "SHM MMU TLB entry PA %lx"
					 " VA %lx DSP_VA %lx Size %lx\n",
					 aEProc[ndx].ulGppPa,
					 aEProc[ndx].ulGppVa,
					 aEProc[ndx].ulDspVa *
					 hIOMgr->uWordSize, pgSize[i]);
				ndx++;
			} else {
				status = hIOMgr->pIntfFxns->pfnBrdMemMap(
				hIOMgr->hWmdContext, paCurr, vaCurr, pgSize[i],
					mapAttrs);
				DBG_Trace(DBG_LEVEL1, "SHM MMU PTE entry PA %lx"
					 " VA %lx DSP_VA %lx Size %lx\n",
					 aEProc[ndx].ulGppPa,
					 aEProc[ndx].ulGppVa,
					 aEProc[ndx].ulDspVa *
					 hIOMgr->uWordSize, pgSize[i]);
				if (DSP_FAILED(status))
					goto func_end;
			}
			paCurr += pgSize[i];
			vaCurr += pgSize[i];
			gppVaCurr += pgSize[i];
			numBytes -= pgSize[i];
			/*
			 * Don't try smaller sizes. Hopefully we have reached
			 * an address aligned to a bigger page size.
			 */
			break;
		}
	}

	/*
	 * Copy remaining entries from CDB. All entries are 1 MB and
	 * should not conflict with SHM entries on MPU or DSP side.
	 */
	for (i = 3; i < 7 && ndx < WMDIOCTL_NUMOFMMUTLB; i++) {
		if (hIOMgr->extProcInfo.tyTlb[i].ulGppPhys == 0)
			continue;

		if ((hIOMgr->extProcInfo.tyTlb[i].ulGppPhys > ulGppPa - 0x100000
			&& hIOMgr->extProcInfo.tyTlb[i].ulGppPhys <=
				ulGppPa + ulSegSize)
			|| (hIOMgr->extProcInfo.tyTlb[i].ulDspVirt > ulDspVa -
				0x100000 / hIOMgr->uWordSize && hIOMgr->
				extProcInfo.tyTlb[i].ulDspVirt
				<= ulDspVa + ulSegSize / hIOMgr->uWordSize)) {
			DBG_Trace(DBG_LEVEL7, "CDB MMU entry %d conflicts with "
				 "SHM.\n\tCDB: GppPa %x, DspVa %x.\n\tSHM: "
				 "GppPa %x, DspVa %x, Bytes %x.\n", i,
				 hIOMgr->extProcInfo.tyTlb[i].ulGppPhys,
				 hIOMgr->extProcInfo.tyTlb[i].ulDspVirt,
				 ulGppPa, ulDspVa, ulSegSize);
			status = DSP_EFAIL;
		} else {
			if (ndx < MAX_LOCK_TLB_ENTRIES) {
				aEProc[ndx].ulDspVa = hIOMgr->extProcInfo.
					tyTlb[i].ulDspVirt;
				aEProc[ndx].ulGppPa = hIOMgr->extProcInfo.
					tyTlb[i].ulGppPhys;
				aEProc[ndx].ulGppVa = 0;
				/* Can't convert, so set to zero */
				aEProc[ndx].ulSize = 0x100000; 	/* 1 MB */
				DBG_Trace(DBG_LEVEL1, "SHM MMU entry PA %x "
					 "DSP_VA 0x%x\n", aEProc[ndx].ulGppPa,
					aEProc[ndx].ulDspVa);
				ndx++;
			} else {
				status = hIOMgr->pIntfFxns->pfnBrdMemMap
					(hIOMgr->hWmdContext,
					hIOMgr->extProcInfo.tyTlb[i].ulGppPhys,
					hIOMgr->extProcInfo.tyTlb[i].ulDspVirt,
					0x100000, mapAttrs);
			}
		}
		if (DSP_FAILED(status))
			goto func_end;
	}

	mapAttrs = 0x00000000;
	mapAttrs = DSP_MAPLITTLEENDIAN;
	mapAttrs |= DSP_MAPPHYSICALADDR;
	mapAttrs |= DSP_MAPELEMSIZE32;
	mapAttrs |= DSP_MAPDONOTLOCK;

	/* Map the L4 peripherals */
	i = 0;
	while (L4PeripheralTable[i].physAddr) {
		status = hIOMgr->pIntfFxns->pfnBrdMemMap
			(hIOMgr->hWmdContext, L4PeripheralTable[i].physAddr,
			L4PeripheralTable[i].dspVirtAddr, HW_PAGE_SIZE_4KB,
			mapAttrs);
		if (DSP_FAILED(status))
			goto func_end;
		i++;
	}


	for (i = ndx; i < WMDIOCTL_NUMOFMMUTLB; i++) {
		aEProc[i].ulDspVa = 0;
		aEProc[i].ulGppPa = 0;
		aEProc[i].ulGppVa = 0;
		aEProc[i].ulSize = 0;
	}
	/*
	 * Set the SHM physical address entry (grayed out in CDB file)
	 * to the virtual uncached ioremapped address of SHM reserved
	 * on MPU.
	 */
	hIOMgr->extProcInfo.tyTlb[0].ulGppPhys = (ulGppVa + ulSeg1Size +
						 ulPadSize);

	/*
	 * Need SHM Phys addr. IO supports only one DSP for now:
	 * uNumProcs = 1.
	 */
	if (!hIOMgr->extProcInfo.tyTlb[0].ulGppPhys || uNumProcs != 1) {
		status = CHNL_E_NOMEMMAP;
		goto func_end;
	} else {
		if (aEProc[0].ulDspVa > ulShmBase) {
			status = DSP_EFAIL;
			goto func_end;
		}
		/* ulShmBase may not be at ulDspVa address */
		ulShmBaseOffset = (ulShmBase - aEProc[0].ulDspVa) *
				hIOMgr->uWordSize;
		/*
		 * WMD_BRD_Ctrl() will set dev context dsp-mmu info. In
		 * _BRD_Start() the MMU will be re-programed with MMU
		 * DSPVa-GPPPa pair info while DSP is in a known
		 * (reset) state.
		 */

		status = hIOMgr->pIntfFxns->pfnDevCntrl(hIOMgr->hWmdContext,
						WMDIOCTL_SETMMUCONFIG, aEProc);
		if (DSP_FAILED(status))
			goto func_end;
		ulShmBase = hIOMgr->extProcInfo.tyTlb[0].ulGppPhys;
		ulShmBase += ulShmBaseOffset;
		ulShmBase = (u32)MEM_LinearAddress((void *)ulShmBase,
				    ulMemLength);
		if (ulShmBase == 0) {
			status = DSP_EPOINTER;
			goto func_end;
		}
		/* Register SM */
		status = registerSHMSegs(hIOMgr, hCodMan, aEProc[0].ulGppPa);
	}

	hIOMgr->pSharedMem = (struct SHM *)ulShmBase;
	hIOMgr->pInput = (u8 *)hIOMgr->pSharedMem + sizeof(struct SHM);
	hIOMgr->pOutput = hIOMgr->pInput + (ulShmLength -
						sizeof(struct SHM)) / 2;
	hIOMgr->uSMBufSize = hIOMgr->pOutput - hIOMgr->pInput;

	/* Set up Shared memory addresses for messaging. */
	hIOMgr->pMsgInputCtrl = (struct MSG *)((u8 *)hIOMgr->pSharedMem
							+ ulShmLength);
	hIOMgr->pMsgInput = (u8 *)hIOMgr->pMsgInputCtrl + sizeof(struct MSG);
	hIOMgr->pMsgOutputCtrl = (struct MSG *)((u8 *)hIOMgr->pMsgInputCtrl
							+ ulMsgLength / 2);
	hIOMgr->pMsgOutput = (u8 *)hIOMgr->pMsgOutputCtrl + sizeof(struct MSG);
	hMsgMgr->uMaxMsgs = ((u8 *)hIOMgr->pMsgOutputCtrl - hIOMgr->pMsgInput)
						/ sizeof(struct MSG_DSPMSG);
	DBG_Trace(DBG_LEVEL7, "IO MGR SHM details : pSharedMem 0x%x, "
		"pInput 0x%x, pOutput 0x%x, pMsgInputCtrl 0x%x, "
		"pMsgInput 0x%x, pMsgOutputCtrl 0x%x, pMsgOutput "
		"0x%x \n", (u8 *)hIOMgr->pSharedMem, (u8 *)hIOMgr->pInput,
		(u8 *)hIOMgr->pOutput, (u8 *)hIOMgr->pMsgInputCtrl,
		(u8 *)hIOMgr->pMsgInput, (u8 *)hIOMgr->pMsgOutputCtrl,
		(u8 *)hIOMgr->pMsgOutput);
	DBG_Trace(DBG_LEVEL7, "** (proc) MAX MSGS IN SHARED MEMORY: "
					"0x%x\n", hMsgMgr->uMaxMsgs);
	memset((void *) hIOMgr->pSharedMem, 0, sizeof(struct SHM));

#ifndef DSP_TRACEBUF_DISABLED
	/* Get the start address of trace buffer */
	status = COD_GetSymValue(hCodMan, SYS_PUTCBEG,
				 &hIOMgr->ulTraceBufferBegin);
	if (DSP_FAILED(status)) {
		status = CHNL_E_NOMEMMAP;
		goto func_end;
	}

	hIOMgr->ulGPPReadPointer = hIOMgr->ulTraceBufferBegin =
		(ulGppVa + ulSeg1Size + ulPadSize) +
		(hIOMgr->ulTraceBufferBegin - ulDspVa);
	/* Get the end address of trace buffer */

	status = COD_GetSymValue(hCodMan, SYS_PUTCEND,
						&hIOMgr->ulTraceBufferEnd);
	if (DSP_FAILED(status)) {
		status = CHNL_E_NOMEMMAP;
		goto func_end;
	}
	hIOMgr->ulTraceBufferEnd = (ulGppVa + ulSeg1Size + ulPadSize) +
				   (hIOMgr->ulTraceBufferEnd - ulDspVa);
	/* Get the current address of DSP write pointer */
	status = COD_GetSymValue(hCodMan, BRIDGE_SYS_PUTC_current,
					 &hIOMgr->ulTraceBufferCurrent);
	if (DSP_FAILED(status)) {
		status = CHNL_E_NOMEMMAP;
		goto func_end;
	}
	hIOMgr->ulTraceBufferCurrent = (ulGppVa + ulSeg1Size + ulPadSize) +
				(hIOMgr->ulTraceBufferCurrent - ulDspVa);
	/* Calculate the size of trace buffer */
	kfree(hIOMgr->pMsg);
	hIOMgr->pMsg = MEM_Alloc(((hIOMgr->ulTraceBufferEnd -
				hIOMgr->ulTraceBufferBegin) *
				hIOMgr->uWordSize) + 2, MEM_NONPAGED);
	if (!hIOMgr->pMsg)
		status = DSP_EMEMORY;

	hIOMgr->ulDspVa = ulDspVa;
	hIOMgr->ulGppVa = (ulGppVa + ulSeg1Size + ulPadSize);

#endif
func_end:
	return status;
}

/*
 *  ======== IO_BufSize ========
 *      Size of shared memory I/O channel.
 */
u32 IO_BufSize(struct IO_MGR *hIOMgr)
{
	if (MEM_IsValidHandle(hIOMgr, IO_MGRSIGNATURE))
		return hIOMgr->uSMBufSize;
	else
		return 0;
}

/*
 *  ======== IO_CancelChnl ========
 *      Cancel IO on a given PCPY channel.
 */
void IO_CancelChnl(struct IO_MGR *hIOMgr, u32 ulChnl)
{
	struct IO_MGR *pIOMgr = (struct IO_MGR *)hIOMgr;
	struct SHM *sm;

	if (!MEM_IsValidHandle(hIOMgr, IO_MGRSIGNATURE))
		goto func_end;
	sm = hIOMgr->pSharedMem;

	/* Inform DSP that we have no more buffers on this channel */
	IO_AndValue(pIOMgr->hWmdContext, struct SHM, sm, hostFreeMask,
		   (~(1 << ulChnl)));

	sm_interrupt_dsp(pIOMgr->hWmdContext, MBX_PCPY_CLASS);
func_end:
	return;
}

/*
 *  ======== IO_DispatchChnl ========
 *      Proc-copy chanl dispatch.
 */
static void IO_DispatchChnl(IN struct IO_MGR *pIOMgr,
			   IN OUT struct CHNL_OBJECT *pChnl, u32 iMode)
{
	if (!MEM_IsValidHandle(pIOMgr, IO_MGRSIGNATURE))
		goto func_end;

	/* See if there is any data available for transfer */
	if (iMode != IO_SERVICE)
		goto func_end;

	/* Any channel will do for this mode */
	InputChnl(pIOMgr, pChnl, iMode);
	OutputChnl(pIOMgr, pChnl, iMode);
func_end:
	return;
}

/*
 *  ======== IO_DispatchMsg ========
 *      Performs I/O dispatch on message queues.
 */
static void IO_DispatchMsg(IN struct IO_MGR *pIOMgr, struct MSG_MGR *hMsgMgr)
{
	if (!MEM_IsValidHandle(pIOMgr, IO_MGRSIGNATURE))
		goto func_end;

	/* We are performing both input and output processing. */
	InputMsg(pIOMgr, hMsgMgr);
	OutputMsg(pIOMgr, hMsgMgr);
func_end:
	return;
}

/*
 *  ======== IO_DispatchPM ========
 *      Performs I/O dispatch on PM related messages from DSP
 */
static void IO_DispatchPM(struct IO_MGR *pIOMgr)
{
	DSP_STATUS status;
	u32 pArg[2];

	/* Perform Power message processing here */
	pArg[0] = pIOMgr->wIntrVal;

	/* Send the command to the WMD clk/pwr manager to handle */
	if (pArg[0] ==  MBX_PM_HIBERNATE_EN) {
		DBG_Trace(DBG_LEVEL7, "IO_DispatchPM : Hibernate "
			 "command\n");
		status = pIOMgr->pIntfFxns->pfnDevCntrl(pIOMgr->
			 hWmdContext, WMDIOCTL_PWR_HIBERNATE, pArg);
		if (DSP_FAILED(status))
			pr_err("%s: hibernate cmd failed 0x%x\n",
						__func__, status);
	} else if (pArg[0] == MBX_PM_OPP_REQ) {
		pArg[1] = pIOMgr->pSharedMem->oppRequest.rqstDspFreq;
		DBG_Trace(DBG_LEVEL7, "IO_DispatchPM : Value of OPP "
			 "value =0x%x \n", pArg[1]);
		status = pIOMgr->pIntfFxns->pfnDevCntrl(pIOMgr->
			 hWmdContext, WMDIOCTL_CONSTRAINT_REQUEST,
			 pArg);
		if (DSP_FAILED(status)) {
			DBG_Trace(DBG_LEVEL7, "IO_DispatchPM : Failed "
				 "to set constraint = 0x%x \n",
				 pArg[1]);
		}
	} else {
		DBG_Trace(DBG_LEVEL7, "IO_DispatchPM - clock control - "
			 "value of msg = 0x%x: \n", pArg[0]);
		status = pIOMgr->pIntfFxns->pfnDevCntrl(pIOMgr->
			 hWmdContext, WMDIOCTL_CLK_CTRL, pArg);
		if (DSP_FAILED(status)) {
			DBG_Trace(DBG_LEVEL7, "IO_DispatchPM : Failed "
				 "to control the DSP clk = 0x%x \n",
				 *pArg);
		}
	}
}

/*
 *  ======== IO_DPC ========
 *      Deferred procedure call for shared memory channel driver ISR.  Carries
 *      out the dispatch of I/O as a non-preemptible event.It can only be
 *      pre-empted      by an ISR.
 */
void IO_DPC(IN OUT unsigned long pRefData)
{
	struct IO_MGR *pIOMgr = (struct IO_MGR *)pRefData;
	struct CHNL_MGR *pChnlMgr;
	struct MSG_MGR *pMsgMgr;
	struct DEH_MGR *hDehMgr;
	u32 requested;
	u32 serviced;

	if (!MEM_IsValidHandle(pIOMgr, IO_MGRSIGNATURE))
		goto func_end;
	pChnlMgr = pIOMgr->hChnlMgr;
	DEV_GetMsgMgr(pIOMgr->hDevObject, &pMsgMgr);
	DEV_GetDehMgr(pIOMgr->hDevObject, &hDehMgr);
	if (!MEM_IsValidHandle(pChnlMgr, CHNL_MGRSIGNATURE))
		goto func_end;


	requested = pIOMgr->dpc_req;
	serviced = pIOMgr->dpc_sched;

	if (serviced == requested)
		goto func_end;

	/* Process pending DPC's */
	do {
		/* Check value of interrupt reg to ensure it's a valid error */
		if ((pIOMgr->wIntrVal > DEH_BASE) &&
		   (pIOMgr->wIntrVal < DEH_LIMIT)) {
			/* Notify DSP/BIOS exception */
			if (hDehMgr) {
#ifndef DSP_TRACEBUF_DISABLED
				PrintDSPDebugTrace(pIOMgr);
#endif
				WMD_DEH_Notify(hDehMgr, DSP_SYSERROR,
						pIOMgr->wIntrVal);
			}
		}
		IO_DispatchChnl(pIOMgr, NULL, IO_SERVICE);
#ifdef CHNL_MESSAGES
		if (MEM_IsValidHandle(pMsgMgr, MSGMGR_SIGNATURE))
			IO_DispatchMsg(pIOMgr, pMsgMgr);
#endif
#ifndef DSP_TRACEBUF_DISABLED
		if (pIOMgr->wIntrVal & MBX_DBG_SYSPRINTF) {
			/* Notify DSP Trace message */
			PrintDSPDebugTrace(pIOMgr);
		}
#endif
		serviced++;
	} while (serviced != requested);
	pIOMgr->dpc_sched = requested;
func_end:
	return;
}

/*
 *  ======== io_mbox_msg ========
 *      Main interrupt handler for the shared memory IO manager.
 *      Calls the WMD's CHNL_ISR to determine if this interrupt is ours, then
 *      schedules a DPC to dispatch I/O.
 */
void io_mbox_msg(u32 msg)
{
	struct IO_MGR *io_mgr;
	struct DEV_OBJECT *dev_obj;
	unsigned long flags;


	dev_obj = DEV_GetFirst();
	DEV_GetIOMgr(dev_obj, &io_mgr);

	if (!io_mgr)
		return;

	io_mgr->wIntrVal = (u16)msg;
	if (io_mgr->wIntrVal & MBX_PM_CLASS)
		IO_DispatchPM(io_mgr);

	if (io_mgr->wIntrVal == MBX_DEH_RESET) {
		io_mgr->wIntrVal = 0;
	} else {
		spin_lock_irqsave(&io_mgr->dpc_lock, flags);
		io_mgr->dpc_req++;
		spin_unlock_irqrestore(&io_mgr->dpc_lock, flags);
		tasklet_schedule(&io_mgr->dpc_tasklet);
	}
	return;
}

/*
 *  ======== IO_RequestChnl ========
 *  Purpose:
 *      Request chanenel I/O from the DSP. Sets flags in shared memory, then
 *      interrupts the DSP.
 */
void IO_RequestChnl(struct IO_MGR *pIOMgr, struct CHNL_OBJECT *pChnl,
		   u32 iMode, OUT u16 *pwMbVal)
{
	struct CHNL_MGR *pChnlMgr;
	struct SHM *sm;

	if (!pChnl || !pwMbVal)
		goto func_end;
	pChnlMgr = pIOMgr->hChnlMgr;
	sm = pIOMgr->pSharedMem;
	if (iMode == IO_INPUT) {
		/*
		 * Assertion fires if CHNL_AddIOReq() called on a stream
		 * which was cancelled, or attached to a dead board.
		 */
		DBC_Assert((pChnl->dwState == CHNL_STATEREADY) ||
			  (pChnl->dwState == CHNL_STATEEOS));
		/* Indicate to the DSP we have a buffer available for input */
		IO_OrValue(pIOMgr->hWmdContext, struct SHM, sm, hostFreeMask,
			  (1 << pChnl->uId));
		*pwMbVal = MBX_PCPY_CLASS;
	} else if (iMode == IO_OUTPUT) {
		/*
		 * This assertion fails if CHNL_AddIOReq() was called on a
		 * stream which was cancelled, or attached to a dead board.
		 */
		DBC_Assert((pChnl->dwState & ~CHNL_STATEEOS) ==
			  CHNL_STATEREADY);
		/*
		 * Record the fact that we have a buffer available for
		 * output.
		 */
		pChnlMgr->dwOutputMask |= (1 << pChnl->uId);
	} else {
		DBC_Assert(iMode); 	/* Shouldn't get here. */
	}
func_end:
	return;
}

/*
 *  ======== IO_Schedule ========
 *      Schedule DPC for IO.
 */
void IO_Schedule(struct IO_MGR *pIOMgr)
{
	unsigned long flags;

	if (!MEM_IsValidHandle(pIOMgr, IO_MGRSIGNATURE))
		return;

	/* Increment count of DPC's pending. */
	spin_lock_irqsave(&pIOMgr->dpc_lock, flags);
	pIOMgr->dpc_req++;
	spin_unlock_irqrestore(&pIOMgr->dpc_lock, flags);

	/* Schedule DPC */
	tasklet_schedule(&pIOMgr->dpc_tasklet);
}

/*
 *  ======== FindReadyOutput ========
 *      Search for a host output channel which is ready to send.  If this is
 *      called as a result of servicing the DPC, then implement a round
 *      robin search; otherwise, this was called by a client thread (via
 *      IO_Dispatch()), so just start searching from the current channel id.
 */
static u32 FindReadyOutput(struct CHNL_MGR *pChnlMgr,
			     struct CHNL_OBJECT *pChnl, u32 dwMask)
{
	u32 uRetval = OUTPUTNOTREADY;
	u32 id, startId;
	u32 shift;

	id = (pChnl != NULL ? pChnl->uId : (pChnlMgr->dwLastOutput + 1));
	id = ((id == CHNL_MAXCHANNELS) ? 0 : id);
	if (id >= CHNL_MAXCHANNELS)
		goto func_end;
	if (dwMask) {
		shift = (1 << id);
		startId = id;
		do {
			if (dwMask & shift) {
				uRetval = id;
				if (pChnl == NULL)
					pChnlMgr->dwLastOutput = id;
				break;
			}
			id = id + 1;
			id = ((id == CHNL_MAXCHANNELS) ? 0 : id);
			shift = (1 << id);
		} while (id != startId);
	}
func_end:
	return uRetval;
}

/*
 *  ======== InputChnl ========
 *      Dispatch a buffer on an input channel.
 */
static void InputChnl(struct IO_MGR *pIOMgr, struct CHNL_OBJECT *pChnl,
		      u32 iMode)
{
	struct CHNL_MGR *pChnlMgr;
	struct SHM *sm;
	u32 chnlId;
	u32 uBytes;
	struct CHNL_IRP *pChirp = NULL;
	u32 dwArg;
	bool fClearChnl = false;
	bool fNotifyClient = false;

	sm = pIOMgr->pSharedMem;
	pChnlMgr = pIOMgr->hChnlMgr;

	/* Attempt to perform input */
	if (!IO_GetValue(pIOMgr->hWmdContext, struct SHM, sm, inputFull))
		goto func_end;

	uBytes = IO_GetValue(pIOMgr->hWmdContext, struct SHM, sm, inputSize) *
			    pChnlMgr->uWordSize;
	chnlId = IO_GetValue(pIOMgr->hWmdContext, struct SHM, sm, inputId);
	dwArg = IO_GetLong(pIOMgr->hWmdContext, struct SHM, sm, arg);
	if (chnlId >= CHNL_MAXCHANNELS) {
		/* Shouldn't be here: would indicate corrupted SHM. */
		DBC_Assert(chnlId);
		goto func_end;
	}
	pChnl = pChnlMgr->apChannel[chnlId];
	if ((pChnl != NULL) && CHNL_IsInput(pChnl->uMode)) {
		if ((pChnl->dwState & ~CHNL_STATEEOS) == CHNL_STATEREADY) {
			if (!pChnl->pIORequests)
				goto func_end;
			/* Get the I/O request, and attempt a transfer */
			pChirp = (struct CHNL_IRP *)LST_GetHead(pChnl->
				 pIORequests);
			if (pChirp) {
				pChnl->cIOReqs--;
				if (pChnl->cIOReqs < 0)
					goto func_end;
				/*
				 * Ensure we don't overflow the client's
				 * buffer.
				 */
				uBytes = min(uBytes, pChirp->cBytes);
				/* Transfer buffer from DSP side */
				uBytes = ReadData(pIOMgr->hWmdContext,
						pChirp->pHostSysBuf,
						pIOMgr->pInput, uBytes);
				pChnl->cBytesMoved += uBytes;
				pChirp->cBytes = uBytes;
				pChirp->dwArg = dwArg;
				pChirp->status = CHNL_IOCSTATCOMPLETE;

				if (uBytes == 0) {
					/*
					 * This assertion fails if the DSP
					 * sends EOS more than once on this
					 * channel.
					 */
					if (pChnl->dwState & CHNL_STATEEOS)
						goto func_end;
					/*
					 * Zero bytes indicates EOS. Update
					 * IOC status for this chirp, and also
					 * the channel state.
					 */
					pChirp->status |= CHNL_IOCSTATEOS;
					pChnl->dwState |= CHNL_STATEEOS;
					/*
					 * Notify that end of stream has
					 * occurred.
					 */
					NTFY_Notify(pChnl->hNtfy,
						   DSP_STREAMDONE);
				}
				/* Tell DSP if no more I/O buffers available */
				if (!pChnl->pIORequests)
					goto func_end;
				if (LST_IsEmpty(pChnl->pIORequests)) {
					IO_AndValue(pIOMgr->hWmdContext,
						   struct SHM, sm, hostFreeMask,
						   ~(1 << pChnl->uId));
				}
				fClearChnl = true;
				fNotifyClient = true;
			} else {
				/*
				 * Input full for this channel, but we have no
				 * buffers available.  The channel must be
				 * "idling". Clear out the physical input
				 * channel.
				 */
				fClearChnl = true;
			}
		} else {
			/* Input channel cancelled: clear input channel */
			fClearChnl = true;
		}
	} else {
		/* DPC fired after host closed channel: clear input channel */
		fClearChnl = true;
	}
	if (fClearChnl) {
		/* Indicate to the DSP we have read the input */
		IO_SetValue(pIOMgr->hWmdContext, struct SHM, sm, inputFull, 0);
		sm_interrupt_dsp(pIOMgr->hWmdContext, MBX_PCPY_CLASS);
	}
	if (fNotifyClient) {
		/* Notify client with IO completion record */
		NotifyChnlComplete(pChnl, pChirp);
	}
func_end:
	return;
}

/*
 *  ======== InputMsg ========
 *      Copies messages from shared memory to the message queues.
 */
static void InputMsg(struct IO_MGR *pIOMgr, struct MSG_MGR *hMsgMgr)
{
	u32 uMsgs;
	u32 i;
	u8 *pMsgInput;
	struct MSG_QUEUE *hMsgQueue;
	struct MSG_FRAME *pMsg;
	struct MSG_DSPMSG msg;
	struct MSG *pCtrl;
	u32 fInputEmpty;
	u32 addr;

	pCtrl = pIOMgr->pMsgInputCtrl;
	/* Get the number of input messages to be read */
	fInputEmpty = IO_GetValue(pIOMgr->hWmdContext, struct MSG, pCtrl,
				 bufEmpty);
	uMsgs = IO_GetValue(pIOMgr->hWmdContext, struct MSG, pCtrl, size);
	if (fInputEmpty)
		goto func_end;

	pMsgInput = pIOMgr->pMsgInput;
	for (i = 0; i < uMsgs; i++) {
		/* Read the next message */
		addr = (u32)&(((struct MSG_DSPMSG *)pMsgInput)->msg.dwCmd);
		msg.msg.dwCmd = ReadExt32BitDspData(pIOMgr->hWmdContext, addr);
		addr = (u32)&(((struct MSG_DSPMSG *)pMsgInput)->msg.dwArg1);
		msg.msg.dwArg1 = ReadExt32BitDspData(pIOMgr->hWmdContext, addr);
		addr = (u32)&(((struct MSG_DSPMSG *)pMsgInput)->msg.dwArg2);
		msg.msg.dwArg2 = ReadExt32BitDspData(pIOMgr->hWmdContext, addr);
		addr = (u32)&(((struct MSG_DSPMSG *)pMsgInput)->dwId);
		msg.dwId = ReadExt32BitDspData(pIOMgr->hWmdContext, addr);
		pMsgInput += sizeof(struct MSG_DSPMSG);
		if (!hMsgMgr->queueList)
			goto func_end;

		/* Determine which queue to put the message in */
		hMsgQueue = (struct MSG_QUEUE *)LST_First(hMsgMgr->queueList);
		DBG_Trace(DBG_LEVEL7, "InputMsg RECVD: dwCmd=0x%x dwArg1=0x%x "
			 "dwArg2=0x%x dwId=0x%x \n", msg.msg.dwCmd,
			 msg.msg.dwArg1, msg.msg.dwArg2, msg.dwId);
		/*
		 * Interrupt may occur before shared memory and message
		 * input locations have been set up. If all nodes were
		 * cleaned up, hMsgMgr->uMaxMsgs should be 0.
		 */
		while (hMsgQueue != NULL) {
			if (msg.dwId == hMsgQueue->dwId) {
				/* Found it */
				if (msg.msg.dwCmd == RMS_EXITACK) {
					/*
					 * Call the node exit notification.
					 * The exit message does not get
					 * queued.
					 */
					(*hMsgMgr->onExit)((HANDLE)hMsgQueue->
						hArg, msg.msg.dwArg1);
				} else {
					/*
					 * Not an exit acknowledgement, queue
					 * the message.
					 */
					if (!hMsgQueue->msgFreeList)
						goto func_end;
					pMsg = (struct MSG_FRAME *)LST_GetHead
						(hMsgQueue->msgFreeList);
					if (hMsgQueue->msgUsedList && pMsg) {
						pMsg->msgData = msg;
						LST_PutTail(hMsgQueue->
						    msgUsedList,
						    (struct list_head *)pMsg);
						NTFY_Notify(hMsgQueue->hNtfy,
							DSP_NODEMESSAGEREADY);
						SYNC_SetEvent(hMsgQueue->
							hSyncEvent);
					} else {
						/*
						 * No free frame to copy the
						 * message into.
						 */
						pr_err("%s: no free msg frames,"
							" discarding msg\n",
							__func__);
					}
				}
				break;
			}

			if (!hMsgMgr->queueList || !hMsgQueue)
				goto func_end;
			hMsgQueue = (struct MSG_QUEUE *)LST_Next(hMsgMgr->
				    queueList, (struct list_head *)hMsgQueue);
		}
	}
	/* Set the post SWI flag */
	if (uMsgs > 0) {
		/* Tell the DSP we've read the messages */
		IO_SetValue(pIOMgr->hWmdContext, struct MSG, pCtrl, bufEmpty,
			   true);
		IO_SetValue(pIOMgr->hWmdContext, struct MSG, pCtrl, postSWI,
			   true);
		sm_interrupt_dsp(pIOMgr->hWmdContext, MBX_PCPY_CLASS);
	}
func_end:
	return;
}

/*
 *  ======== NotifyChnlComplete ========
 *  Purpose:
 *      Signal the channel event, notifying the client that I/O has completed.
 */
static void NotifyChnlComplete(struct CHNL_OBJECT *pChnl,
			      struct CHNL_IRP *pChirp)
{
	bool fSignalEvent;

	if (!MEM_IsValidHandle(pChnl, CHNL_SIGNATURE) || !pChnl->hSyncEvent ||
	   !pChnl->pIOCompletions || !pChirp)
		goto func_end;

	/*
	 * Note: we signal the channel event only if the queue of IO
	 * completions is empty.  If it is not empty, the event is sure to be
	 * signalled by the only IO completion list consumer:
	 * WMD_CHNL_GetIOC().
	 */
	fSignalEvent = LST_IsEmpty(pChnl->pIOCompletions);
	/* Enqueue the IO completion info for the client */
	LST_PutTail(pChnl->pIOCompletions, (struct list_head *)pChirp);
	pChnl->cIOCs++;

	if (pChnl->cIOCs > pChnl->cChirps)
		goto func_end;
	/* Signal the channel event (if not already set) that IO is complete */
	if (fSignalEvent)
		SYNC_SetEvent(pChnl->hSyncEvent);

	/* Notify that IO is complete */
	NTFY_Notify(pChnl->hNtfy, DSP_STREAMIOCOMPLETION);
func_end:
	return;
}

/*
 *  ======== OutputChnl ========
 *  Purpose:
 *      Dispatch a buffer on an output channel.
 */
static void OutputChnl(struct IO_MGR *pIOMgr, struct CHNL_OBJECT *pChnl,
			u32 iMode)
{
	struct CHNL_MGR *pChnlMgr;
	struct SHM *sm;
	u32 chnlId;
	struct CHNL_IRP *pChirp;
	u32 dwDspFMask;

	pChnlMgr = pIOMgr->hChnlMgr;
	sm = pIOMgr->pSharedMem;
	/* Attempt to perform output */
	if (IO_GetValue(pIOMgr->hWmdContext, struct SHM, sm, outputFull))
		goto func_end;

	if (pChnl && !((pChnl->dwState & ~CHNL_STATEEOS) == CHNL_STATEREADY))
		goto func_end;

	/* Look to see if both a PC and DSP output channel are ready */
	dwDspFMask = IO_GetValue(pIOMgr->hWmdContext, struct SHM, sm,
				 dspFreeMask);
	chnlId = FindReadyOutput(pChnlMgr, pChnl, (pChnlMgr->dwOutputMask &
				 dwDspFMask));
	if (chnlId == OUTPUTNOTREADY)
		goto func_end;

	pChnl = pChnlMgr->apChannel[chnlId];
	if (!pChnl || !pChnl->pIORequests) {
		/* Shouldn't get here */
		goto func_end;
	}
	/* Get the I/O request, and attempt a transfer */
	pChirp = (struct CHNL_IRP *)LST_GetHead(pChnl->pIORequests);
	if (!pChirp)
		goto func_end;

	pChnl->cIOReqs--;
	if (pChnl->cIOReqs < 0 || !pChnl->pIORequests)
		goto func_end;

	/* Record fact that no more I/O buffers available */
	if (LST_IsEmpty(pChnl->pIORequests))
		pChnlMgr->dwOutputMask &= ~(1 << chnlId);

	/* Transfer buffer to DSP side */
	pChirp->cBytes = WriteData(pIOMgr->hWmdContext, pIOMgr->pOutput,
			pChirp->pHostSysBuf, min(pIOMgr->uSMBufSize, pChirp->
			cBytes));
	pChnl->cBytesMoved += pChirp->cBytes;
	/* Write all 32 bits of arg */
	IO_SetLong(pIOMgr->hWmdContext, struct SHM, sm, arg, pChirp->dwArg);
#if _CHNL_WORDSIZE == 2
	IO_SetValue(pIOMgr->hWmdContext, struct SHM, sm, outputId,
		   (u16)chnlId);
	IO_SetValue(pIOMgr->hWmdContext, struct SHM, sm, outputSize,
		   (u16)(pChirp->cBytes + (pChnlMgr->uWordSize-1)) /
		   (u16)pChnlMgr->uWordSize);
#else
	IO_SetValue(pIOMgr->hWmdContext, struct SHM, sm, outputId, chnlId);
	IO_SetValue(pIOMgr->hWmdContext, struct SHM, sm, outputSize,
		   (pChirp->cBytes + (pChnlMgr->uWordSize - 1)) / pChnlMgr->
		   uWordSize);
#endif
	IO_SetValue(pIOMgr->hWmdContext, struct SHM, sm, outputFull, 1);
	/* Indicate to the DSP we have written the output */
	sm_interrupt_dsp(pIOMgr->hWmdContext, MBX_PCPY_CLASS);
	/* Notify client with IO completion record (keep EOS) */
	pChirp->status &= CHNL_IOCSTATEOS;
	NotifyChnlComplete(pChnl, pChirp);
	/* Notify if stream is done. */
	if (pChirp->status & CHNL_IOCSTATEOS)
		NTFY_Notify(pChnl->hNtfy, DSP_STREAMDONE);

func_end:
	return;
}
/*
 *  ======== OutputMsg ========
 *      Copies messages from the message queues to the shared memory.
 */
static void OutputMsg(struct IO_MGR *pIOMgr, struct MSG_MGR *hMsgMgr)
{
	u32 uMsgs = 0;
	u32 i;
	u8 *pMsgOutput;
	struct MSG_FRAME *pMsg;
	struct MSG *pCtrl;
	u32 fOutputEmpty;
	u32 val;
	u32 addr;

	pCtrl = pIOMgr->pMsgOutputCtrl;

	/* Check if output has been cleared */
	fOutputEmpty = IO_GetValue(pIOMgr->hWmdContext, struct MSG, pCtrl,
				  bufEmpty);
	if (fOutputEmpty) {
		uMsgs = (hMsgMgr->uMsgsPending > hMsgMgr->uMaxMsgs) ?
			 hMsgMgr->uMaxMsgs : hMsgMgr->uMsgsPending;
		pMsgOutput = pIOMgr->pMsgOutput;
		/* Copy uMsgs messages into shared memory */
		for (i = 0; i < uMsgs; i++) {
			if (!hMsgMgr->msgUsedList) {
				pMsg = NULL;
				goto func_end;
			} else {
				pMsg = (struct MSG_FRAME *)LST_GetHead(
					hMsgMgr->msgUsedList);
			}
			if (pMsg != NULL) {
				val = (pMsg->msgData).dwId;
				addr = (u32)&(((struct MSG_DSPMSG *)
					pMsgOutput)->dwId);
				WriteExt32BitDspData(pIOMgr->hWmdContext, addr,
						     val);
				val = (pMsg->msgData).msg.dwCmd;
				addr = (u32)&((((struct MSG_DSPMSG *)
					pMsgOutput)->msg).dwCmd);
				WriteExt32BitDspData(pIOMgr->hWmdContext, addr,
						     val);
				val = (pMsg->msgData).msg.dwArg1;
				addr =
					(u32)&((((struct MSG_DSPMSG *)
					pMsgOutput)->msg).dwArg1);
				WriteExt32BitDspData(pIOMgr->hWmdContext, addr,
						    val);
				val = (pMsg->msgData).msg.dwArg2;
				addr =
					(u32)&((((struct MSG_DSPMSG *)
					pMsgOutput)->msg).dwArg2);
				WriteExt32BitDspData(pIOMgr->hWmdContext, addr,
						    val);
				pMsgOutput += sizeof(struct MSG_DSPMSG);
				if (!hMsgMgr->msgFreeList)
					goto func_end;
				LST_PutTail(hMsgMgr->msgFreeList,
					   (struct list_head *)pMsg);
				SYNC_SetEvent(hMsgMgr->hSyncEvent);
			}
		}

		if (uMsgs > 0) {
			hMsgMgr->uMsgsPending -= uMsgs;
#if _CHNL_WORDSIZE == 2
			IO_SetValue(pIOMgr->hWmdContext, struct MSG, pCtrl,
				   size, (u16)uMsgs);
#else
			IO_SetValue(pIOMgr->hWmdContext, struct MSG, pCtrl,
				   size, uMsgs);
#endif
			IO_SetValue(pIOMgr->hWmdContext, struct MSG, pCtrl,
				   bufEmpty, false);
			/* Set the post SWI flag */
			IO_SetValue(pIOMgr->hWmdContext, struct MSG, pCtrl,
				   postSWI, true);
			/* Tell the DSP we have written the output. */
			sm_interrupt_dsp(pIOMgr->hWmdContext,
						MBX_PCPY_CLASS);
		}
	}
func_end:
	return;
}

/*
 *  ======== registerSHMSegs ========
 *  purpose:
 *      Registers GPP SM segment with CMM.
 */
static DSP_STATUS registerSHMSegs(struct IO_MGR *hIOMgr,
				 struct COD_MANAGER *hCodMan,
				 u32 dwGPPBasePA)
{
	DSP_STATUS status = DSP_SOK;
	u32 ulShm0_Base = 0;
	u32 ulShm0_End = 0;
	u32 ulShm0_RsrvdStart = 0;
	u32 ulRsrvdSize = 0;
	u32 ulGppPhys;
	u32 ulDspVirt;
	u32 ulShmSegId0 = 0;
	u32 dwOffset, dwGPPBaseVA, ulDSPSize;

	/*
	 * Read address and size info for first SM region.
	 * Get start of 1st SM Heap region.
	 */
	status = COD_GetSymValue(hCodMan, SHM0_SHARED_BASE_SYM, &ulShm0_Base);
	if (ulShm0_Base == 0) {
		status = DSP_EFAIL;
		goto func_end;
	}
	/* Get end of 1st SM Heap region */
	if (DSP_SUCCEEDED(status)) {
		/* Get start and length of message part of shared memory */
		status = COD_GetSymValue(hCodMan, SHM0_SHARED_END_SYM,
					 &ulShm0_End);
		if (ulShm0_End == 0) {
			status = DSP_EFAIL;
			goto func_end;
		}
	}
	/* Start of Gpp reserved region */
	if (DSP_SUCCEEDED(status)) {
		/* Get start and length of message part of shared memory */
		status = COD_GetSymValue(hCodMan, SHM0_SHARED_RESERVED_BASE_SYM,
					&ulShm0_RsrvdStart);
		if (ulShm0_RsrvdStart == 0) {
			status = DSP_EFAIL;
			goto func_end;
		}
	}
	/* Register with CMM */
	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetCmmMgr(hIOMgr->hDevObject, &hIOMgr->hCmmMgr);
		if (DSP_SUCCEEDED(status)) {
			status = CMM_UnRegisterGPPSMSeg(hIOMgr->hCmmMgr,
				 CMM_ALLSEGMENTS);
		}
	}
	/* Register new SM region(s) */
	if (DSP_SUCCEEDED(status) && (ulShm0_End - ulShm0_Base) > 0) {
		/* Calc size (bytes) of SM the GPP can alloc from */
		ulRsrvdSize = (ulShm0_End - ulShm0_RsrvdStart + 1) * hIOMgr->
			      uWordSize;
		if (ulRsrvdSize <= 0) {
			status = DSP_EFAIL;
			goto func_end;
		}
		/* Calc size of SM DSP can alloc from */
		ulDSPSize = (ulShm0_RsrvdStart - ulShm0_Base) * hIOMgr->
			uWordSize;
		if (ulDSPSize <= 0) {
			status = DSP_EFAIL;
			goto func_end;
		}
		/* First TLB entry reserved for Bridge SM use.*/
		ulGppPhys = hIOMgr->extProcInfo.tyTlb[0].ulGppPhys;
		/* Get size in bytes */
		ulDspVirt = hIOMgr->extProcInfo.tyTlb[0].ulDspVirt * hIOMgr->
			uWordSize;
		/*
		 * Calc byte offset used to convert GPP phys <-> DSP byte
		 * address.
		 */
		if (dwGPPBasePA > ulDspVirt)
			dwOffset = dwGPPBasePA - ulDspVirt;
		else
			dwOffset = ulDspVirt - dwGPPBasePA;

		if (ulShm0_RsrvdStart * hIOMgr->uWordSize < ulDspVirt) {
			status = DSP_EFAIL;
			goto func_end;
		}
		/*
		 * Calc Gpp phys base of SM region.
		 * This is actually uncached kernel virtual address.
		 */
		dwGPPBaseVA = ulGppPhys + ulShm0_RsrvdStart * hIOMgr->uWordSize
				- ulDspVirt;
		/*
		 * Calc Gpp phys base of SM region.
		 * This is the physical address.
		 */
		dwGPPBasePA = dwGPPBasePA + ulShm0_RsrvdStart * hIOMgr->
			      uWordSize - ulDspVirt;
		/* Register SM Segment 0.*/
		status = CMM_RegisterGPPSMSeg(hIOMgr->hCmmMgr, dwGPPBasePA,
			 ulRsrvdSize, dwOffset, (dwGPPBasePA > ulDspVirt) ?
			 CMM_ADDTODSPPA : CMM_SUBFROMDSPPA,
			 (u32)(ulShm0_Base * hIOMgr->uWordSize),
			 ulDSPSize, &ulShmSegId0, dwGPPBaseVA);
		/* First SM region is segId = 1 */
		if (ulShmSegId0 != 1)
			status = DSP_EFAIL;
	}
func_end:
	return status;
}

/*
 *  ======== ReadData ========
 *      Copies buffers from the shared memory to the host buffer.
 */
static u32 ReadData(struct WMD_DEV_CONTEXT *hDevContext, void *pDest,
		     void *pSrc, u32 uSize)
{
	memcpy(pDest, pSrc, uSize);
	return uSize;
}

/*
 *  ======== WriteData ========
 *      Copies buffers from the host side buffer to the shared memory.
 */
static u32 WriteData(struct WMD_DEV_CONTEXT *hDevContext, void *pDest,
			void *pSrc, u32 uSize)
{
	memcpy(pDest, pSrc, uSize);
	return uSize;
}

/* ZCPY IO routines. */
void IO_IntrDSP2(IN struct IO_MGR *pIOMgr, IN u16 wMbVal)
{
	sm_interrupt_dsp(pIOMgr->hWmdContext, wMbVal);
}

/*
 *  ======== IO_SHMcontrol ========
 *      Sets the requested SHM setting.
 */
DSP_STATUS IO_SHMsetting(struct IO_MGR *hIOMgr, u8 desc, void *pArgs)
{
#ifdef CONFIG_BRIDGE_DVFS
	u32 i;
	struct dspbridge_platform_data *pdata =
				omap_dspbridge_dev->dev.platform_data;

	switch (desc) {
	case SHM_CURROPP:
		/* Update the shared memory with requested OPP information */
		if (pArgs != NULL)
			hIOMgr->pSharedMem->oppTableStruct.currOppPt =
				*(u32 *)pArgs;
		else
			return DSP_EFAIL;
		break;
	case SHM_OPPINFO:
		/*
		 * Update the shared memory with the voltage, frequency,
		 * min and max frequency values for an OPP.
		 */
		for (i = 0; i <= pdata->dsp_num_speeds; i++) {
			hIOMgr->pSharedMem->oppTableStruct.oppPoint[i].voltage =
				pdata->dsp_freq_table[i].u_volts;
			DBG_Trace(DBG_LEVEL5, "OPP shared memory -voltage: "
				 "%d\n", hIOMgr->pSharedMem->oppTableStruct.
				 oppPoint[i].voltage);
			hIOMgr->pSharedMem->oppTableStruct.oppPoint[i].
				frequency = pdata->dsp_freq_table[i].dsp_freq;
			DBG_Trace(DBG_LEVEL5, "OPP shared memory -frequency: "
				 "%d\n", hIOMgr->pSharedMem->oppTableStruct.
				 oppPoint[i].frequency);
			hIOMgr->pSharedMem->oppTableStruct.oppPoint[i].minFreq =
				pdata->dsp_freq_table[i].thresh_min_freq;
			DBG_Trace(DBG_LEVEL5, "OPP shared memory -min value: "
				 "%d\n", hIOMgr->pSharedMem->oppTableStruct.
				  oppPoint[i].minFreq);
			hIOMgr->pSharedMem->oppTableStruct.oppPoint[i].maxFreq =
				pdata->dsp_freq_table[i].thresh_max_freq;
			DBG_Trace(DBG_LEVEL5, "OPP shared memory -max value: "
				 "%d\n", hIOMgr->pSharedMem->oppTableStruct.
				 oppPoint[i].maxFreq);
		}
		hIOMgr->pSharedMem->oppTableStruct.numOppPts =
			pdata->dsp_num_speeds;
		DBG_Trace(DBG_LEVEL5, "OPP shared memory - max OPP number: "
			 "%d\n", hIOMgr->pSharedMem->oppTableStruct.numOppPts);
		/* Update the current OPP number */
		if (pdata->dsp_get_opp)
			i = (*pdata->dsp_get_opp)();
		hIOMgr->pSharedMem->oppTableStruct.currOppPt = i;
		DBG_Trace(DBG_LEVEL7, "OPP value programmed to shared memory: "
			 "%d\n", i);
		break;
	case SHM_GETOPP:
		/* Get the OPP that DSP has requested */
		*(u32 *)pArgs = hIOMgr->pSharedMem->oppRequest.rqstOppPt;
		break;
	default:
		break;
	}
#endif
	return DSP_SOK;
}

/*
 *  ======== WMD_IO_GetProcLoad ========
 *      Gets the Processor's Load information
 */
DSP_STATUS WMD_IO_GetProcLoad(IN struct IO_MGR *hIOMgr,
			     OUT struct DSP_PROCLOADSTAT *pProcStat)
{
	pProcStat->uCurrLoad = hIOMgr->pSharedMem->loadMonInfo.currDspLoad;
	pProcStat->uPredictedLoad = hIOMgr->pSharedMem->loadMonInfo.predDspLoad;
	pProcStat->uCurrDspFreq = hIOMgr->pSharedMem->loadMonInfo.currDspFreq;
	pProcStat->uPredictedFreq = hIOMgr->pSharedMem->loadMonInfo.predDspFreq;

	DBG_Trace(DBG_LEVEL4, "Curr Load =%d, Pred Load = %d, Curr Freq = %d, "
			     "Pred Freq = %d\n", pProcStat->uCurrLoad,
			     pProcStat->uPredictedLoad, pProcStat->uCurrDspFreq,
			     pProcStat->uPredictedFreq);
	return DSP_SOK;
}

#ifndef DSP_TRACEBUF_DISABLED
void PrintDSPDebugTrace(struct IO_MGR *hIOMgr)
{
	u32 ulNewMessageLength = 0, ulGPPCurPointer;

	while (true) {
		/* Get the DSP current pointer */
		ulGPPCurPointer = *(u32 *) (hIOMgr->ulTraceBufferCurrent);
		ulGPPCurPointer = hIOMgr->ulGppVa + (ulGPPCurPointer -
				  hIOMgr->ulDspVa);

		/* No new debug messages available yet */
		if (ulGPPCurPointer == hIOMgr->ulGPPReadPointer) {
			break;
		} else if (ulGPPCurPointer > hIOMgr->ulGPPReadPointer) {
			/* Continuous data */
			ulNewMessageLength = ulGPPCurPointer - hIOMgr->
					     ulGPPReadPointer;

			memcpy(hIOMgr->pMsg, (char *)hIOMgr->ulGPPReadPointer,
				ulNewMessageLength);
			hIOMgr->pMsg[ulNewMessageLength] = '\0';
			/*
			 * Advance the GPP trace pointer to DSP current
			 * pointer.
			 */
			hIOMgr->ulGPPReadPointer += ulNewMessageLength;
			/* Print the trace messages */
			pr_info("DSPTrace:%s", hIOMgr->pMsg);
		} else if (ulGPPCurPointer < hIOMgr->ulGPPReadPointer) {
		/* Handle trace buffer wraparound */
			memcpy(hIOMgr->pMsg, (char *)hIOMgr->ulGPPReadPointer,
				hIOMgr->ulTraceBufferEnd -
				hIOMgr->ulGPPReadPointer);
			ulNewMessageLength = ulGPPCurPointer -
				hIOMgr->ulTraceBufferBegin;
			memcpy(&hIOMgr->pMsg[hIOMgr->ulTraceBufferEnd -
				hIOMgr->ulGPPReadPointer],
				(char *)hIOMgr->ulTraceBufferBegin,
				ulNewMessageLength);
			hIOMgr->pMsg[hIOMgr->ulTraceBufferEnd -
				hIOMgr->ulGPPReadPointer +
				ulNewMessageLength] = '\0';
			/*
			 * Advance the GPP trace pointer to DSP current
			 * pointer.
			 */
			hIOMgr->ulGPPReadPointer = hIOMgr->ulTraceBufferBegin +
						   ulNewMessageLength;
			/* Print the trace messages */
			pr_info("DSPTrace:%s", hIOMgr->pMsg);
		}
	}
}
#endif

/*
 *  ======== PrintDspTraceBuffer ========
 *      Prints the trace buffer returned from the DSP (if DBG_Trace is enabled).
 *  Parameters:
 *    hDehMgr:          Handle to DEH manager object
 *                      number of extra carriage returns to generate.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Unable to allocate memory.
 *  Requires:
 *      hDehMgr muse be valid. Checked in WMD_DEH_Notify.
 */
DSP_STATUS PrintDspTraceBuffer(struct WMD_DEV_CONTEXT *hWmdContext)
{
	DSP_STATUS status = DSP_SOK;

#if (defined(CONFIG_BRIDGE_DEBUG) || defined(DDSP_DEBUG_PRODUCT)) && GT_TRACE
	struct COD_MANAGER *hCodMgr;
	u32 ulTraceEnd;
	u32 ulTraceBegin;
	u32 trace_cur_pos;
	u32 ulNumBytes = 0;
	u32 ulNumWords = 0;
	u32 ulWordSize = 2;
	char *pszBuf;
	char *str_beg;
	char *trace_end;
	char *buf_end;
	char *new_line;


	struct WMD_DEV_CONTEXT *pWmdContext = (struct WMD_DEV_CONTEXT *)
						     hWmdContext;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct DEV_OBJECT *pDevObject = (struct DEV_OBJECT *)
					    pWmdContext->hDevObject;

	status = DEV_GetCodMgr(pDevObject, &hCodMgr);
	if (!hCodMgr)
		status = DSP_EHANDLE;

	if (DSP_SUCCEEDED(status))
		/* Look for SYS_PUTCBEG/SYS_PUTCEND */
		status = COD_GetSymValue(hCodMgr, COD_TRACEBEG, &ulTraceBegin);

	if (DSP_SUCCEEDED(status))
		status = COD_GetSymValue(hCodMgr, COD_TRACEEND, &ulTraceEnd);

	if (DSP_SUCCEEDED(status))
		/* trace_cur_pos will hold the address of a DSP pointer */
		status = COD_GetSymValue(hCodMgr, COD_TRACECURPOS,
							&trace_cur_pos);
	if (DSP_FAILED(status))
		goto func_end;

	ulNumBytes = (ulTraceEnd - ulTraceBegin);

	ulNumWords = ulNumBytes * ulWordSize;
	status = DEV_GetIntfFxns(pDevObject, &pIntfFxns);

	if (DSP_FAILED(status))
		goto func_end;

	pszBuf = MEM_Calloc(ulNumBytes+2, MEM_NONPAGED);
	if (pszBuf != NULL) {
		/* Read trace buffer data */
		status = (*pIntfFxns->pfnBrdRead)(hWmdContext,
			(u8 *)pszBuf, (u32)ulTraceBegin,
			ulNumBytes, 0);

		if (DSP_FAILED(status))
			goto func_end;

		/* Read the value at the DSP address in trace_cur_pos. */
		status = (*pIntfFxns->pfnBrdRead)(hWmdContext,
				(u8 *)&trace_cur_pos, (u32)trace_cur_pos,
					 4, 0);
		if (DSP_FAILED(status))
			goto func_end;
		/* Pack and do newline conversion */
		pr_info("%s: DSP Trace Buffer Begin:\n"
			 "=======================\n%s\n", __func__, pszBuf);
		/* convert to offset */
			trace_cur_pos = trace_cur_pos - ulTraceBegin;
		if (ulNumBytes) {
			/*
			 * The buffer is not full, find the end of the
			 * data -- buf_end will be >= pszBuf after
			 * while.
			 */
			buf_end = &pszBuf[ulNumBytes+1];
			/* DSP print position */
			trace_end = &pszBuf[trace_cur_pos];
			/*
			 * Search buffer for a new_line and replace it
			 * with '\0', then print as string.
			 * Continue until end of buffer is reached.
			 */
			str_beg = trace_end;
			ulNumBytes = buf_end - str_beg;

			while (str_beg < buf_end) {
				new_line = strnchr(str_beg, ulNumBytes, '\n');
				if (new_line && new_line < buf_end) {
					*new_line = 0;
					pr_debug("%s\n", str_beg);
					str_beg = ++new_line;
					ulNumBytes = buf_end - str_beg;
				} else {
					/*
					 * Assume buffer empty if it contains
					 * a zero
					 */
					if (*str_beg != '\0') {
						str_beg[ulNumBytes] = 0;
						pr_debug("%s\n", str_beg);
					}
					str_beg = buf_end;
					ulNumBytes = 0;
				}
			}
			/*
			 * Search buffer for a nNewLine and replace it
			 * with '\0', then print as string.
			 * Continue until buffer is exhausted.
			 */
			str_beg = pszBuf;
			ulNumBytes = trace_end - str_beg;

			while (str_beg < trace_end) {
				new_line = strnchr(str_beg, ulNumBytes, '\n');
				if (new_line != NULL && new_line < trace_end) {
					*new_line = 0;
					pr_debug("%s\n", str_beg);
					str_beg = ++new_line;
					ulNumBytes = trace_end - str_beg;
				} else {
					/*
					 * Assume buffer empty if it contains
					 * a zero
					 */
					if (*str_beg != '\0') {
						str_beg[ulNumBytes] = 0;
						pr_debug("%s\n", str_beg);
					}
				str_beg = trace_end;
				ulNumBytes = 0;
				}
			}
		}
			pr_info("\n=======================\n"
					"DSP Trace Buffer End:\n");
			kfree(pszBuf);
} 	else {
		status = DSP_EMEMORY;
	}
func_end:
#endif
	if (DSP_FAILED(status))
		pr_err("%s: Failed, status 0x%x\n", __func__, status);
		return status;
}

void IO_SM_init(void)
{
	GT_create(&dsp_trace_mask, "DT"); /* DSP Trace Mask */
}

#ifdef CONFIG_BRIDGE_WDT3
/*
 *  ======== io_wdt3_ovf ========
 *      Deferred procedure call WDT overflow ISR.  Carries
 *      out the dispatch of I/O as a non-preemptible event.It can only be
 *      pre-empted  by an ISR.
 */
void io_wdt3_ovf(unsigned long data)
{
	struct DEH_MGR *deh_mgr;
	struct IO_MGR *io_mgr = (struct IO_MGR *)data;
	DEV_GetDehMgr(io_mgr->hDevObject, &deh_mgr);
	if (deh_mgr)
		WMD_DEH_Notify(deh_mgr, DSP_WDTOVERFLOW, (u32)io_mgr);
}

/*
 *  ======== io_isr_wdt3 ========

 */
irqreturn_t io_isr_wdt3(int irq, IN void *data)
{
	u32 value;
	struct IO_MGR *io_mgr = (struct IO_MGR *)data;
	/* The pending interrupt event is cleared when the set status bit is
	 * overwritten by a value of 1 by a write command in the WTDi.WISR
	 * register. Reading the WTDi.WISR register and writing the value
	 * back allows a fast acknowledge interrupt process. */
	if (CLK_Get_UseCnt(SERVICESCLK_wdt3_fck)) {
		value = __raw_readl(io_mgr->hWmdContext->wdt3_base
							+ WDT_ISR_OFFSET);
		__raw_writel(value, io_mgr->hWmdContext->wdt3_base
							+ WDT_ISR_OFFSET);
	}
	tasklet_schedule(&io_mgr->wdt3_tasklet);
	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_BRIDGE_WDT3
/*
 *  ======== dsp_wdt_config ========
 *      Enables/disables WDT.
 */
void dsp_wdt_enable(bool enable)
{
	u32 tmp;
	struct WMD_DEV_CONTEXT *dev_ctxt;
	struct IO_MGR *io_mgr;
	static bool already_enabled;

	if (!wdt3_enable || already_enabled == enable)
		return;

	DEV_GetWMDContext(DEV_GetFirst(), &dev_ctxt);
	DEV_GetIOMgr(DEV_GetFirst(), &io_mgr);
	if (!dev_ctxt || !io_mgr)
		return;
	already_enabled = enable;
	if (enable) {
		CLK_Enable(SERVICESCLK_wdt3_fck);
		CLK_Enable(SERVICESCLK_wdt3_ick);
		io_mgr->pSharedMem->wdt_setclocks = 1;
		tmp = __raw_readl(dev_ctxt->wdt3_base + WDT_ISR_OFFSET);
		__raw_writel(tmp, dev_ctxt->wdt3_base + WDT_ISR_OFFSET);
		enable_irq(INT_34XX_WDT3_IRQ);
	} else {
		disable_irq(INT_34XX_WDT3_IRQ);
		io_mgr->pSharedMem->wdt_setclocks = 0;
		CLK_Disable(SERVICESCLK_wdt3_ick);
		CLK_Disable(SERVICESCLK_wdt3_fck);
	}
}

void dsp_wdt_set_timeout(unsigned timeout)
{
	struct IO_MGR *io_mgr;
	DEV_GetIOMgr(DEV_GetFirst(), &io_mgr);
	if (io_mgr && io_mgr->pSharedMem != (void *)-1)
		io_mgr->pSharedMem->wdt_overflow = timeout;
	else
		pr_err("%s: DSP image not loaded\n", __func__);
}

unsigned dsp_wdt_get_timeout(void)
{
	struct IO_MGR *io_mgr;
	DEV_GetIOMgr(DEV_GetFirst(), &io_mgr);
	return (io_mgr && io_mgr->pSharedMem != (void *)-1) ?
		io_mgr->pSharedMem->wdt_overflow :
		(pr_err("%s: DSP image not loaded\n", __func__), 0);
}

bool dsp_wdt_get_enable(void)
{
	return wdt3_enable;
}

void dsp_wdt_set_enable(bool enable)
{
	wdt3_enable = enable;
}
#endif

DSP_STATUS dump_dsp_stack(struct WMD_DEV_CONTEXT *wmd_context)
{
	DSP_STATUS status = DSP_SOK;

	struct COD_MANAGER *code_mgr;
	struct NODE_MGR *node_mgr;
	u32 trace_begin;
	char name[256];
	struct {
		u32 head[2];
		u32 size;
	} mmu_fault_dbg_info;
	u32 *buffer;
	u32 *buffer_beg;
	u32 *buffer_end;
	u32 exc_type;
	u32 i;
	u32 offset_output;
	u32 total_size;
	u32 poll_cnt;
	const char *dsp_regs[] = {"EFR", "IERR", "ITSR", "NTSR",
				"IRP", "NRP", "AMR", "SSR",
				"ILC", "RILC", "IER", "CSR"};
	const char *exec_ctxt[] = {"Task", "SWI", "HWI", "Unknown"};

	struct WMD_DRV_INTERFACE *intf_fxns;
	struct DEV_OBJECT *dev_object = wmd_context->hDevObject;

	status = DEV_GetCodMgr(dev_object, &code_mgr);
	if (!code_mgr) {
		pr_debug("%s: Failed on DEV_GetCodMgr.\n", __func__);
		status = DSP_EHANDLE;
	}

	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetNodeManager(dev_object, &node_mgr);
		if (!node_mgr) {
			pr_debug("%s: Failed on DEV_GetNodeManager.\n",
								__func__);
			status = DSP_EHANDLE;
		}
	}

	if (DSP_SUCCEEDED(status)) {
		/* Look for SYS_PUTCBEG/SYS_PUTCEND: */
		status = COD_GetSymValue(code_mgr, COD_TRACEBEG, &trace_begin);
		pr_debug("%s: trace_begin Value 0x%x\n",
			__func__, trace_begin);
		if (DSP_FAILED(status))
			pr_debug("%s: Failed on COD_GetSymValue.\n", __func__);
	}
	if (DSP_SUCCEEDED(status))
		status = DEV_GetIntfFxns(dev_object, &intf_fxns);

	/* Check for the "magic number" in the trace buffer.  If it has   */
	/* yet to appear then poll the trace buffer to wait for it.  Its  */
	/* appearance signals that the DSP has finished dumping its state.*/
	mmu_fault_dbg_info.head[0] = 0;
	mmu_fault_dbg_info.head[1] = 0;
	if (DSP_SUCCEEDED(status)) {
		poll_cnt = 0;
		while ((mmu_fault_dbg_info.head[0] != MMU_FAULT_HEAD1 ||
			mmu_fault_dbg_info.head[1] != MMU_FAULT_HEAD2) &&
			poll_cnt < POLL_MAX) {

			/* Read DSP dump size from the DSP trace buffer... */
			status = (*intf_fxns->pfnBrdRead)(wmd_context,
				(u8 *)&mmu_fault_dbg_info, (u32)trace_begin,
				sizeof(mmu_fault_dbg_info), 0);

			if (DSP_FAILED(status))
				break;

			poll_cnt++;
		}

		if (mmu_fault_dbg_info.head[0] != MMU_FAULT_HEAD1 &&
			mmu_fault_dbg_info.head[1] != MMU_FAULT_HEAD2) {
			status = DSP_ETIMEOUT;
			pr_err("%s:No DSP MMU-Fault information available.\n",
							__func__);
		}
	}

	if (DSP_SUCCEEDED(status)) {
		total_size = mmu_fault_dbg_info.size;
		/* Limit the size in case DSP went crazy */
		if (total_size > MAX_MMU_DBGBUFF)
			total_size = MAX_MMU_DBGBUFF;

		buffer = MEM_Calloc(total_size, MEM_NONPAGED);
		buffer_beg = buffer;
		buffer_end =  buffer + total_size / 4;

		if (!buffer) {
			status = DSP_EMEMORY;
			pr_debug("%s: Failed to "
				"allocate stack dump buffer.\n", __func__);
			goto func_end;
		}

		/* Read bytes from the DSP trace buffer... */
		status = (*intf_fxns->pfnBrdRead)(wmd_context,
				(u8 *)buffer, (u32)trace_begin,
				total_size, 0);
		if (DSP_FAILED(status)) {
			pr_debug("%s: Failed to Read Trace Buffer.\n",
						__func__);
			goto func_end;
		}

		pr_err("\nAproximate Crash Position:\n");
		pr_err("--------------------------\n");

		exc_type = buffer[3];
		if (!exc_type)
			i = buffer[79];		/* IRP */
		else
			i = buffer[80];		/* NRP */

		if ((*buffer > 0x01000000) && (node_find_addr(node_mgr, i,
			0x1000, &offset_output, name) == DSP_SOK))
			pr_err("0x%-8x [\"%s\" + 0x%x]\n", i, name,
				i - offset_output);
		else
			pr_err("0x%-8x [Unable to match to a symbol.]\n", i);


		buffer += 4;

		pr_err("\nExecution Info:\n");
		pr_err("---------------\n");

		if (*buffer < ARRAY_SIZE(exec_ctxt)) {
			pr_err("Execution context \t%s\n",
				exec_ctxt[*buffer++]);
		} else {
			pr_err("Execution context corrupt\n");
			kfree(buffer_beg);
			return -EFAULT;
		}
		pr_err("Task Handle\t\t0x%x\n", *buffer++);
		pr_err("Stack Pointer\t\t0x%x\n", *buffer++);
		pr_err("Stack Top\t\t0x%x\n", *buffer++);
		pr_err("Stack Bottom\t\t0x%x\n", *buffer++);
		pr_err("Stack Size\t\t0x%x\n", *buffer++);
		pr_err("Stack Size In Use\t0x%x\n", *buffer++);

		pr_err("\nCPU Registers\n");
		pr_err("---------------\n");

		for (i = 0; i < 32; i++) {
			if (i == 4 || i == 6 || i == 8)
				pr_err("A%d 0x%-8x [Function Argument %d]\n",
						i, *buffer++, i-3);
			else if (i == 15)
				pr_err("A15 0x%-8x [Frame Pointer]\n",
							*buffer++);
			else
				pr_err("A%d 0x%x\n", i, *buffer++);
		}

		pr_err("\nB0 0x%x\n", *buffer++);
		pr_err("B1 0x%x\n", *buffer++);
		pr_err("B2 0x%x\n", *buffer++);

		if ((*buffer > 0x01000000) && (node_find_addr(node_mgr, *buffer,
			0x1000, &offset_output, name) == DSP_SOK))

			pr_err("B3 0x%-8x [Function Return Pointer:"
				" \"%s\" + 0x%x]\n", *buffer, name,
				*buffer - offset_output);
		else
			pr_err("B3 0x%-8x [Function Return Pointer:"
				"Unable to match to a symbol.]\n", *buffer);

		buffer++;

		for (i = 4; i < 32; i++) {
			if (i == 4 || i == 6 || i == 8)
				pr_err("B%d 0x%-8x [Function Argument %d]\n",
						i, *buffer++, i-2);
			else if (i == 14)
				pr_err("B14 0x%-8x [Data Page Pointer]\n",
							*buffer++);
			else
				pr_err("B%d 0x%x\n", i, *buffer++);

		}

		pr_err("\n");

		for (i = 0; i < ARRAY_SIZE(dsp_regs); i++)
			pr_err("%s 0x%x\n", dsp_regs[i], *buffer++);

		pr_err("\nStack:\n");
		pr_err("------\n");

		for (i = 0; buffer < buffer_end; i++, buffer++) {
			if ((*buffer > 0x01000000) && (node_find_addr(node_mgr,
				*buffer , 0x600, &offset_output, name) ==
				DSP_SOK))
				pr_err("[%d] 0x%-8x [\"%s\" + 0x%x]\n",
					i, *buffer, name,
					*buffer - offset_output);
			else
				pr_err("[%d] 0x%x\n", i, *buffer);
		}

		kfree(buffer_beg);
	}
func_end:
	return status;
}



void dump_dl_modules(struct WMD_DEV_CONTEXT *wmd_context)
{
	struct COD_MANAGER *code_mgr;
	struct WMD_DRV_INTERFACE *intf_fxns;
	struct WMD_DEV_CONTEXT *wmd_ctxt =
				(struct WMD_DEV_CONTEXT *) wmd_context;
	struct DEV_OBJECT *dev_object =
				(struct DEV_OBJECT *) wmd_ctxt->hDevObject;

	struct modules_header modules_hdr;
	struct dll_module *module_struct = NULL;
	u32 module_dsp_addr;
	u32 module_size;
	u32 module_struct_size = 0;
	u32 sect_ndx;
	char *sect_str ;
	DSP_STATUS status = DSP_SOK;

	status = DEV_GetIntfFxns(dev_object, &intf_fxns);
	if (DSP_FAILED(status)) {
		pr_debug("%s: Failed on DEV_GetIntfFxns.\n", __func__);
		goto func_end;
	}

	status = DEV_GetCodMgr(dev_object, &code_mgr);
	if (!code_mgr) {
		pr_debug("%s: Failed on DEV_GetCodMgr.\n", __func__);
		status = DSP_EHANDLE;
		goto func_end;
	}

	/* Lookup  the address of the modules_header structure */
	status = COD_GetSymValue(code_mgr, "_DLModules", &module_dsp_addr);
	if (DSP_FAILED(status)) {
		pr_debug("%s: Failed on COD_GetSymValue for _DLModules.\n",
				__func__);
		goto func_end;
	}

	pr_debug("%s: _DLModules at 0x%x\n", __func__, module_dsp_addr);

	/* Copy the modules_header structure from DSP memory. */
	status = (*intf_fxns->pfnBrdRead)(wmd_context, (u8 *) &modules_hdr,
				(u32) module_dsp_addr, sizeof(modules_hdr), 0);

	if (DSP_FAILED(status)) {
		pr_debug("%s: Failed failed to read modules header.\n",
			__func__);
		goto func_end;
	}

	module_dsp_addr = modules_hdr.first_module;
	module_size = modules_hdr.first_module_size;

	pr_debug("%s: dll_module_header 0x%x %d\n", __func__, module_dsp_addr,
								module_size);

	pr_err("\nDynamically Loaded Modules:\n"
		"---------------------------\n");

	/* For each dll_module structure in the list... */
	while (module_size) {

		/*
		 * Allocate/re-allocate memory to hold the dll_module
		 * structure. The memory is re-allocated only if the existing
		 * allocation is too small.
		 */
		if (module_size > module_struct_size) {

			kfree(module_struct);

			module_struct = MEM_Calloc(module_size+128,
							MEM_NONPAGED);
			module_struct_size = module_size+128;
			pr_debug("%s: allocated module struct %p %d\n",
				__func__, module_struct, module_struct_size);
			if (!module_struct)
				goto func_end;
		}
		/* Copy the dll_module structure from DSP memory */
		status = (*intf_fxns->pfnBrdRead)(wmd_context,
			(u8 *)module_struct, module_dsp_addr, module_size, 0);

		if (DSP_FAILED(status)) {
			pr_debug(
			"%s: Failed to read dll_module stuct for 0x%x.\n",
			__func__, module_dsp_addr);
			break;
		}

		/* Update info regarding the _next_ module in the list. */
		module_dsp_addr = module_struct->next_module;
		module_size = module_struct->next_module_size;

		pr_debug("%s: next module 0x%x %d, this module num sects %d\n",
			__func__, module_dsp_addr, module_size,
			module_struct->num_sects);

		/*
		 * The section name strings start immedialty following
		 * the array of dll_sect structures.
		 */
		sect_str = (char *) &module_struct->
					sects[module_struct->num_sects];

		pr_err("%s\n", sect_str);

		/*
		 * Advance to the first section name string.
		 * Each string follows the one before.
		 */
		sect_str += strlen(sect_str) + 1;

		/* Access each dll_sect structure and its name string. */
		for (sect_ndx = 0;
			sect_ndx < module_struct->num_sects; sect_ndx++) {
			pr_err("    Section: 0x%x ",
				module_struct->sects[sect_ndx].sect_load_adr);

			if (((u32) sect_str - (u32) module_struct) <
				module_struct_size) {
				pr_err("%s\n", sect_str);
				/* Each string follows the one before. */
				sect_str += strlen(sect_str)+1;
			} else {
				pr_err("<string error>\n");
				pr_debug("%s: section name sting address "
					"is invalid %p\n", __func__, sect_str);
			}
		}
	}
func_end:
	kfree(module_struct);

}
