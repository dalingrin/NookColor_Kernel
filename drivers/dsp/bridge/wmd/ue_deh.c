/*
 * ue_deh.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Implements upper edge DSP exception handling (DEH) functions.
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
#include <dspbridge/dbg.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/mem.h>
#include <dspbridge/ntfy.h>
#include <dspbridge/drv.h>

/*  ----------------------------------- Link Driver */
#include <dspbridge/wmddeh.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/dev.h>
#include <dspbridge/wcd.h>

/* ------------------------------------ Hardware Abstraction Layer */
#include <hw_defs.h>
#include <hw_mmu.h>

/*  ----------------------------------- This */
#include "mmu_fault.h"
#include "_tiomap.h"
#include "_deh.h"
#include "_tiomap_mmu.h"
#include "_tiomap_pwr.h"
#include <dspbridge/io_sm.h>

#include <mach/dmtimer.h>

/* GP Timer number to trigger interrupt for MMU-fault ISR on DSP */
#define GPTIMER_FOR_DSP_MMU_FAULT	7
/* Bit mask to enable overflow interrupt */
#define GPTIMER_IRQ_OVERFLOW		2
/* Max time to check for GP Timer IRQ */
#define GPTIMER_IRQ_WAIT_MAX_CNT	1000

static struct HW_MMUMapAttrs_t  mapAttrs = { HW_LITTLE_ENDIAN,
					HW_ELEM_SIZE_16BIT,
					HW_MMU_CPUES} ;
#define VirtToPhys(x)       ((x) - PAGE_OFFSET + PHYS_OFFSET)

static u32 dummyVaAddr;

static 	struct omap_dm_timer *timer;
/*
 *  ======== WMD_DEH_Create ========
 *      Creates DEH manager object.
 */
DSP_STATUS WMD_DEH_Create(OUT struct DEH_MGR **phDehMgr,
			 struct DEV_OBJECT *hDevObject)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = NULL;
	struct CFG_HOSTRES cfgHostRes;
	struct CFG_DEVNODE *hDevNode;
	struct WMD_DEV_CONTEXT *hWmdContext = NULL;

	 /*  Message manager will be created when a file is loaded, since
	 *  size of message buffer in shared memory is configurable in
	 *  the base image.  */
	/* Get WMD context info. */
	DEV_GetWMDContext(hDevObject, &hWmdContext);
	DBC_Assert(hWmdContext);
	dummyVaAddr = 0;
	/* Allocate IO manager object: */
	MEM_AllocObject(pDehMgr, struct DEH_MGR, SIGNATURE);
	if (pDehMgr == NULL) {
		status = DSP_EMEMORY;
	} else {
		/* Create an NTFY object to manage notifications */
		status = NTFY_Create(&pDehMgr->hNtfy);

		/* Create a MMUfault DPC */
		tasklet_init(&pDehMgr->dpc_tasklet, MMU_FaultDpc, (u32)pDehMgr);

		if (DSP_SUCCEEDED(status))
			status = DEV_GetDevNode(hDevObject, &hDevNode);

		if (DSP_SUCCEEDED(status))
			status = CFG_GetHostResources(hDevNode, &cfgHostRes);

		if (DSP_SUCCEEDED(status)) {
			/* Fill in context structure */
			pDehMgr->hWmdContext = hWmdContext;
			pDehMgr->errInfo.dwErrMask = 0L;
			pDehMgr->errInfo.dwVal1 = 0L;
			pDehMgr->errInfo.dwVal2 = 0L;
			pDehMgr->errInfo.dwVal3 = 0L;
			/* Install ISR function for DSP MMU fault */
			if ((request_irq(INT_DSP_MMU_IRQ, MMU_FaultIsr, 0,
			   "DspBridge\tiommu fault", (void *)pDehMgr)) == 0)
				status = DSP_SOK;
			else
				status = DSP_EFAIL;
		}
	}
	if (DSP_FAILED(status)) {
		/* If create failed, cleanup */
		WMD_DEH_Destroy((struct DEH_MGR *)pDehMgr);
		*phDehMgr = NULL;
	} else {
		timer = omap_dm_timer_request_specific(
					GPTIMER_FOR_DSP_MMU_FAULT);
		if (timer)
			omap_dm_timer_disable(timer);
		else {
			pr_err("%s:GPTimer not available\n", __func__);
			return -ENODEV;
		}
		*phDehMgr = (struct DEH_MGR *)pDehMgr;
	}

	return status;
}

/*
 *  ======== WMD_DEH_Destroy ========
 *      Destroys DEH manager object.
 */
DSP_STATUS WMD_DEH_Destroy(struct DEH_MGR *hDehMgr)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;

	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		/* Release dummy VA buffer */
		WMD_DEH_ReleaseDummyMem();
		/* If notification object exists, delete it */
		if (pDehMgr->hNtfy)
			(void)NTFY_Delete(pDehMgr->hNtfy);
		/* Disable DSP MMU fault */
		free_irq(INT_DSP_MMU_IRQ, pDehMgr);

		/* Free DPC object */
		tasklet_kill(&pDehMgr->dpc_tasklet);

		/* Deallocate the DEH manager object */
		MEM_FreeObject(pDehMgr);
		/* The GPTimer is no longer needed */
		if (timer) {
			omap_dm_timer_free(timer);
			timer = NULL;
		}
	}

	return status;
}

/*
 *  ======== WMD_DEH_RegisterNotify ========
 *      Registers for DEH notifications.
 */
DSP_STATUS WMD_DEH_RegisterNotify(struct DEH_MGR *hDehMgr, u32 uEventMask,
				 u32 uNotifyType,
				 struct DSP_NOTIFICATION *hNotification)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;

	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		status = NTFY_Register(pDehMgr->hNtfy, hNotification,
			 uEventMask, uNotifyType);
	}

	return status;
}


/*
 *  ======== WMD_DEH_Notify ========
 *      DEH error notification function. Informs user about the error.
 */
void WMD_DEH_Notify(struct DEH_MGR *hDehMgr, u32 ulEventMask,
			 u32 dwErrInfo)
{
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;
	struct WMD_DEV_CONTEXT *pDevContext;
	u32 memPhysical = 0;
	u32 HW_MMU_MAX_TLB_COUNT = 31;
	extern u32 faultAddr;
	u32 cnt = 0;

	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		printk(KERN_INFO "WMD_DEH_Notify: ********** DEVICE EXCEPTION "
			"**********\n");
		pDevContext = (struct WMD_DEV_CONTEXT *)pDehMgr->hWmdContext;

		switch (ulEventMask) {
		case DSP_SYSERROR:
			/* reset errInfo structure before use */
			pDehMgr->errInfo.dwErrMask = DSP_SYSERROR;
			pDehMgr->errInfo.dwVal1 = 0L;
			pDehMgr->errInfo.dwVal2 = 0L;
			pDehMgr->errInfo.dwVal3 = 0L;
			pDehMgr->errInfo.dwVal1 = dwErrInfo;
			printk(KERN_ERR "WMD_DEH_Notify: DSP_SYSERROR, errInfo "
				"= 0x%x\n", dwErrInfo);

			dump_dl_modules(pDevContext);
			dump_dsp_stack(pDevContext);

			break;
		case DSP_MMUFAULT:
			/* MMU fault routine should have set err info
			 * structure */
			pDehMgr->errInfo.dwErrMask = DSP_MMUFAULT;
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT,"
				"errInfo = 0x%x\n", dwErrInfo);
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT, High "
				"Address = 0x%x\n",
				(unsigned int)pDehMgr->errInfo.dwVal1);
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT, Low "
				"Address = 0x%x\n",
				(unsigned int)pDehMgr->errInfo.dwVal2);
			printk(KERN_INFO "WMD_DEH_Notify: DSP_MMUFAULT, fault "
				"address = 0x%x\n", (unsigned int)faultAddr);

			PrintDspTraceBuffer(pDevContext);
			dump_dl_modules(pDevContext);

			dummyVaAddr = (u32)MEM_Calloc(sizeof(char) * 0x1000,
					MEM_PAGED);
			memPhysical  = VirtToPhys(PG_ALIGN_LOW((u32)dummyVaAddr,
								PG_SIZE_4K));
			pDevContext = (struct WMD_DEV_CONTEXT *)
						pDehMgr->hWmdContext;
			/* Reset the dynamic mmu index to fixed count if it
			 * exceeds 31. So that the dynmmuindex is always
			 * between the range of standard/fixed entries
			 * and 31.  */
			if (pDevContext->numTLBEntries >
			   HW_MMU_MAX_TLB_COUNT) {
				pDevContext->numTLBEntries = pDevContext->
					fixedTLBEntries;
			}

			HW_MMU_TLBAdd(pDevContext->dwDSPMmuBase,
				memPhysical, faultAddr, HW_PAGE_SIZE_4KB, 1,
				&mapAttrs, HW_SET, HW_SET);
			/*
			 * Send a GP Timer interrupt to DSP
			 * The DSP expects a GP timer interrupt after an
			 * MMU-Fault Request GPTimer
			 */
			if (timer) {
				omap_dm_timer_enable(timer);
				/* Enable overflow interrupt */
				omap_dm_timer_set_int_enable(timer,
						GPTIMER_IRQ_OVERFLOW);
				/*
				 * Set counter value to overflow counter after
				 * one tick and start timer
				 */
				omap_dm_timer_set_load_start(timer, 0,
							0xfffffffe);

				/* Wait 80us for timer to overflow */
				udelay(80);

				/* Check interrupt status and */
				/* wait for interrupt */
				cnt = 0;
				while (!(omap_dm_timer_read_status(timer) &
					GPTIMER_IRQ_OVERFLOW)) {
					if (cnt++ >=
						GPTIMER_IRQ_WAIT_MAX_CNT) {
						pr_err("%s: GPTimer interrupt"
							" failed\n", __func__);
						break;
					}
				}
			}

			/* Clear MMU interrupt */
			HW_MMU_EventAck(pDevContext->dwDSPMmuBase,
					 HW_MMU_TRANSLATION_FAULT);

			dump_dsp_stack(hDehMgr->hWmdContext);
			if (timer)
				omap_dm_timer_disable(timer);
			break;
#ifdef CONFIG_BRIDGE_NTFY_PWRERR
		case DSP_PWRERROR:
			/* reset errInfo structure before use */
			pDehMgr->errInfo.dwErrMask = DSP_PWRERROR;
			pDehMgr->errInfo.dwVal1 = 0L;
			pDehMgr->errInfo.dwVal2 = 0L;
			pDehMgr->errInfo.dwVal3 = 0L;
			pDehMgr->errInfo.dwVal1 = dwErrInfo;
			printk(KERN_ERR "WMD_DEH_Notify: DSP_PWRERROR, errInfo "
					"= 0x%x\n", dwErrInfo);
			break;
#endif /* CONFIG_BRIDGE_NTFY_PWRERR */
#ifdef CONFIG_BRIDGE_WDT3
		case DSP_WDTOVERFLOW:
			pDehMgr->errInfo.dwErrMask = DSP_WDTOVERFLOW;
			pDehMgr->errInfo.dwVal1 = 0L;
			pDehMgr->errInfo.dwVal2 = 0L;
			pDehMgr->errInfo.dwVal3 = 0L;
			pr_err("WMD_DEH_Notify: DSP_WDTOVERFLOW \n ");
			break;
#endif
		default:
			DBG_Trace(DBG_LEVEL6,
				 "WMD_DEH_Notify: Unknown Error, errInfo = "
				 "0x%x\n", dwErrInfo);
			break;
		}

		/* Filter subsequent notifications when an error occurs */
		if (pDevContext->dwBrdState != BRD_ERROR) {
			NTFY_Notify(pDehMgr->hNtfy, ulEventMask);
#ifdef CONFIG_BRIDGE_RECOVERY
			bridge_recover_schedule();
#endif
		}

		/* Set the Board state as ERROR */
		pDevContext->dwBrdState = BRD_ERROR;
		/* Disable all the clocks that were enabled by DSP */
		(void)DSP_PeripheralClocks_Disable(pDevContext, NULL);
#ifdef CONFIG_BRIDGE_WDT3
		/*
		 * Avoid the subsequent WDT if it happens once,
		 * also If MMU fault occurs
		 */
		dsp_wdt_enable(false);
#endif

	}
}

/*
 *  ======== WMD_DEH_GetInfo ========
 *      Retrieves error information.
 */
DSP_STATUS WMD_DEH_GetInfo(struct DEH_MGR *hDehMgr,
			  struct DSP_ERRORINFO *pErrInfo)
{
	DSP_STATUS status = DSP_SOK;
	struct DEH_MGR *pDehMgr = (struct DEH_MGR *)hDehMgr;

	DBC_Require(pDehMgr);
	DBC_Require(pErrInfo);

	if (MEM_IsValidHandle(pDehMgr, SIGNATURE)) {
		/* Copy DEH error info structure to PROC error info
		 * structure. */
		pErrInfo->dwErrMask = pDehMgr->errInfo.dwErrMask;
		pErrInfo->dwVal1 = pDehMgr->errInfo.dwVal1;
		pErrInfo->dwVal2 = pDehMgr->errInfo.dwVal2;
		pErrInfo->dwVal3 = pDehMgr->errInfo.dwVal3;
	} else {
		status = DSP_EHANDLE;
	}

	return status;
}


/*
 *  ======== WMD_DEH_ReleaseDummyMem ========
 *      Releases memory allocated for dummy page
 */
void WMD_DEH_ReleaseDummyMem(void)
{
	kfree((void *)dummyVaAddr);
	dummyVaAddr = 0;
}

