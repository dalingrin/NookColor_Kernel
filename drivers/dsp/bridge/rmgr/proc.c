/*
 * proc.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Processor interface at the driver level.
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

/* ------------------------------------ Host OS */
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
#include <dspbridge/ntfy.h>
#include <dspbridge/sync.h>
/*  ----------------------------------- Mini Driver */
#include <dspbridge/wmd.h>
#include <dspbridge/wmddeh.h>
/*  ----------------------------------- Platform Manager */
#include <dspbridge/cod.h>
#include <dspbridge/dev.h>
#include <dspbridge/procpriv.h>
#include <dspbridge/dmm.h>

/*  ----------------------------------- Resource Manager */
#include <dspbridge/mgr.h>
#include <dspbridge/node.h>
#include <dspbridge/nldr.h>
#include <dspbridge/rmm.h>

/*  ----------------------------------- Others */
#include <dspbridge/dbdcd.h>
#include <dspbridge/msg.h>
#include <dspbridge/wmdioctl.h>
#include <dspbridge/drv.h>

/*  ----------------------------------- This */
#include <dspbridge/proc.h>
#include <dspbridge/pwr.h>

#include <dspbridge/resourcecleanup.h>
/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define PROC_SIGNATURE	   0x434F5250	/* "PROC" (in reverse). */
#define MAXCMDLINELEN       255
#define PROC_ENVPROCID      "PROC_ID=%d"
#define MAXPROCIDLEN	(8 + 5)
#define PROC_DFLT_TIMEOUT   10000	/* Time out in milliseconds  */
#define PWR_TIMEOUT	 500	/* Sleep/wake timout in msec */
#define EXTEND	      "_EXT_END"	/* Extmem end addr in DSP binary */

#define DSP_CACHE_LINE 128

#define BUFMODE_MASK	(3 << 14)

/* Buffer modes from DSP perspective */
#define RBUF		0x4000		/* Input buffer */
#define WBUF		0x8000		/* Output Buffer */

extern char *iva_img;

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask PROC_DebugMask = { NULL, NULL };	/* WCD MGR Mask */
#endif

/* The PROC_OBJECT structure.   */
struct PROC_OBJECT {
	struct list_head link;		/* Link to next PROC_OBJECT */
	u32 dwSignature;		/* Used for object validation */
	struct DEV_OBJECT *hDevObject;	/* Device this PROC represents */
	u32 hProcess;			/* Process owning this Processor */
	struct MGR_OBJECT *hMgrObject;	/* Manager Object Handle */
	u32 uAttachCount;		/* Processor attach count */
	u32 uProcessor;			/* Processor number */
	u32 uTimeout;			/* Time out count */
	enum DSP_PROCSTATE sState;	/* Processor state */
	u32 ulUnit;			/* DDSP unit number */
	bool bIsAlreadyAttached;	/*
					 * True if the Device below has
					 * GPP Client attached
					 */
	struct NTFY_OBJECT *hNtfy;	/* Manages  notifications */
	struct WMD_DEV_CONTEXT *hWmdContext;	/* WMD Context Handle */
	struct WMD_DRV_INTERFACE *pIntfFxns;	/* Function interface to WMD */
	char *g_pszLastCoff;
	struct list_head proc_object;
};

static u32 cRefs;

struct SYNC_CSOBJECT *hProcLock;	/* For critical sections */

/*  ----------------------------------- Function Prototypes */
static DSP_STATUS PROC_Monitor(struct PROC_OBJECT *hProcessor);
static s32 GetEnvpCount(char **envp);
static char **PrependEnvp(char **newEnvp, char **envp, s32 cEnvp, s32 cNewEnvp,
			 char *szVar);

/*
 *  ======== PROC_Attach ========
 *  Purpose:
 *      Prepare for communication with a particular DSP processor, and return
 *      a handle to the processor object.
 */
DSP_STATUS
PROC_Attach(u32 uProcessor, OPTIONAL CONST struct DSP_PROCESSORATTRIN *pAttrIn,
       void **phProcessor, struct PROCESS_CONTEXT *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DEV_OBJECT *hDevObject;
	struct PROC_OBJECT *pProcObject = NULL;
	struct MGR_OBJECT *hMgrObject = NULL;
	struct DRV_OBJECT *hDrvObject = NULL;
	u32 devType;

	DBC_Require(cRefs > 0);
	DBC_Require(phProcessor != NULL);

	if (pr_ctxt->hProcessor) {
		*phProcessor = pr_ctxt->hProcessor;
		return status;
	}

	/* Get the Driver and Manager Object Handles */
	status = CFG_GetObject((u32 *)&hDrvObject, REG_DRV_OBJECT);
	if (DSP_SUCCEEDED(status))
		status = CFG_GetObject((u32 *)&hMgrObject, REG_MGR_OBJECT);

	if (DSP_SUCCEEDED(status)) {
		/* Get the Device Object */
		status = DRV_GetDevObject(uProcessor, hDrvObject, &hDevObject);
	}
	if (DSP_SUCCEEDED(status))
		status = DEV_GetDevType(hDevObject, &devType);

	if (DSP_FAILED(status))
		goto func_end;

	/* If we made it this far, create the Proceesor object: */
	MEM_AllocObject(pProcObject, struct PROC_OBJECT, PROC_SIGNATURE);
	/* Fill out the Processor Object: */
	if (pProcObject == NULL) {
		status = DSP_EMEMORY;
		goto func_end;
	}
	pProcObject->hDevObject = hDevObject;
	pProcObject->hMgrObject = hMgrObject;
	pProcObject->uProcessor = devType;
	/* Store TGID of Caller Process */
	pProcObject->hProcess = current->tgid;

	INIT_LIST_HEAD(&pProcObject->proc_object);

	if (pAttrIn)
		pProcObject->uTimeout = pAttrIn->uTimeout;
	else
		pProcObject->uTimeout = PROC_DFLT_TIMEOUT;

	status = DEV_GetIntfFxns(hDevObject, &pProcObject->pIntfFxns);
	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetWMDContext(hDevObject,
					 &pProcObject->hWmdContext);
		if (DSP_FAILED(status))
			MEM_FreeObject(pProcObject);
	} else
		MEM_FreeObject(pProcObject);

	if (DSP_FAILED(status))
		goto func_end;

	/* Create the Notification Object */
	/* This is created with no event mask, no notify mask
	 * and no valid handle to the notification. They all get
	 * filled up when PROC_RegisterNotify is called */
	status = NTFY_Create(&pProcObject->hNtfy);
	if (DSP_SUCCEEDED(status)) {
		/* Insert the Processor Object into the DEV List.
		 * Return handle to this Processor Object:
		 * Find out if the Device is already attached to a
		 * Processor. If so, return AlreadyAttached status */
		LST_InitElem(&pProcObject->link);
		status = DEV_InsertProcObject(pProcObject->hDevObject,
					     (u32)pProcObject,
					     &pProcObject->bIsAlreadyAttached);
		if (DSP_SUCCEEDED(status)) {
			if (pProcObject->bIsAlreadyAttached)
				status = DSP_SALREADYATTACHED;
		} else {
			if (pProcObject->hNtfy)
				NTFY_Delete(pProcObject->hNtfy);

			MEM_FreeObject(pProcObject);
		}
		if (DSP_SUCCEEDED(status)) {
			*phProcessor = (void *)pProcObject;
			pr_ctxt->hProcessor = *phProcessor;
			(void)PROC_NotifyClients(pProcObject,
						 DSP_PROCESSORATTACH);
		}
	} else {
		/* Don't leak memory if DSP_FAILED */
		MEM_FreeObject(pProcObject);
		goto func_end;
	}
func_end:
	DBC_Ensure((status == DSP_EFAIL && *phProcessor == NULL) ||
		  (DSP_SUCCEEDED(status) &&
		  MEM_IsValidHandle(pProcObject, PROC_SIGNATURE)) ||
		  (status == DSP_SALREADYATTACHED &&
		  MEM_IsValidHandle(pProcObject, PROC_SIGNATURE)));

	return status;
}

static DSP_STATUS GetExecFile(struct CFG_DEVNODE *hDevNode,
			     struct DEV_OBJECT *hDevObject,
			     u32 size, char *execFile)
{
	s32 devType;
	s32 len;

	DEV_GetDevType(hDevObject, (u32 *) &devType);
	if (devType == DSP_UNIT) {
		return CFG_GetExecFile(hDevNode, size, execFile);
	} else if (devType == IVA_UNIT) {
		if (iva_img) {
			len = strlen(iva_img);
			strncpy(execFile, iva_img, len + 1);
			return DSP_SOK;
		}
	}
	return DSP_EFILE;
}

/*
 *  ======== PROC_AutoStart ======== =
 *  Purpose:
 *      A Particular device gets loaded with the default image
 *      if the AutoStart flag is set.
 *  Parameters:
 *      hDevObject:     Handle to the Device
 *  Returns:
 *      DSP_SOK:   On Successful Loading
 *      DSP_EFAIL  General Failure
 *  Requires:
 *      hDevObject != NULL
 *  Ensures:
 */
DSP_STATUS PROC_AutoStart(struct CFG_DEVNODE *hDevNode,
			 struct DEV_OBJECT *hDevObject)
{
	DSP_STATUS status = DSP_EFAIL;
	u32 dwAutoStart = 0;	/* autostart flag */
	struct PROC_OBJECT *pProcObject;
	char szExecFile[MAXCMDLINELEN];
	char *argv[2];
	struct MGR_OBJECT *hMgrObject = NULL;
	s32 devType;

	DBC_Require(cRefs > 0);
	DBC_Require(hDevNode != NULL);
	DBC_Require(hDevObject != NULL);

	/* Create a Dummy PROC Object */
	status = CFG_GetObject((u32 *)&hMgrObject, REG_MGR_OBJECT);
	if (DSP_FAILED(status))
		goto func_end;

	MEM_AllocObject(pProcObject, struct PROC_OBJECT, PROC_SIGNATURE);
	if (pProcObject == NULL) {
		status = DSP_EMEMORY;
		goto func_end;
	}
	pProcObject->hDevObject = hDevObject;
	pProcObject->hMgrObject = hMgrObject;
	status = DEV_GetIntfFxns(hDevObject, &pProcObject->pIntfFxns);
	if (DSP_SUCCEEDED(status))
		status = DEV_GetWMDContext(hDevObject,
					&pProcObject->hWmdContext);
	if (DSP_FAILED(status))
		goto func_cont;

	/* Stop the Device, put it into standby mode */
	status = PROC_Stop(pProcObject);

	if (DSP_FAILED(status))
		goto func_cont;

	status = CFG_GetAutoStart(hDevNode, &dwAutoStart);
	if (DSP_FAILED(status) || !dwAutoStart) {
		status = DSP_EFAIL;
		goto func_cont;
	}

	/* paranoid - must be able to kfree this on remaining error paths */
	pProcObject->g_pszLastCoff = NULL;

	/* Get the default executable for this board... */
	DEV_GetDevType(hDevObject, (u32 *)&devType);
	pProcObject->uProcessor = devType;
	status = GetExecFile(hDevNode, hDevObject, sizeof(szExecFile),
							szExecFile);
	if (DSP_SUCCEEDED(status)) {
		argv[0] = szExecFile;
		argv[1] = NULL;
		/* ...and try to load it: */
		status = PROC_Load(pProcObject, 1, (CONST char **)argv, NULL);
		if (DSP_SUCCEEDED(status))
			status = PROC_Start(pProcObject);
	}
	kfree(pProcObject->g_pszLastCoff);
	pProcObject->g_pszLastCoff = NULL;
func_cont:
	MEM_FreeObject(pProcObject);
func_end:
	return status;
}

/*
 *  ======== PROC_Ctrl ========
 *  Purpose:
 *      Pass control information to the GPP device driver managing the
 *      DSP processor.
 *
 *      This will be an OEM-only function, and not part of the DSP/BIOS Bridge
 *      application developer's API.
 *      Call the WMD_ICOTL Fxn with the Argument This is a Synchronous
 *      Operation. arg can be null.
 */
DSP_STATUS PROC_Ctrl(void *hProcessor, u32 dwCmd,
		    IN struct DSP_CBDATA *arg)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = hProcessor;
	u32 timeout = 0;

	DBC_Require(cRefs > 0);


	/* intercept PWR deep sleep command */
	if (dwCmd == WMDIOCTL_DEEPSLEEP) {
		timeout = arg->cbData;
		status = PWR_SleepDSP(PWR_DEEPSLEEP, timeout);
	}
	/* intercept PWR emergency sleep command */
	else if (dwCmd == WMDIOCTL_EMERGENCYSLEEP) {
		timeout = arg->cbData;
		status = PWR_SleepDSP(PWR_EMERGENCYDEEPSLEEP, timeout);
	} else if (dwCmd == PWR_DEEPSLEEP) {
		/* timeout = arg->cbData; */
		status = PWR_SleepDSP(PWR_DEEPSLEEP, timeout);
	}
	/* intercept PWR wake commands */
	else if (dwCmd == WMDIOCTL_WAKEUP) {
		timeout = arg->cbData;
		status = PWR_WakeDSP(timeout);
	} else if (dwCmd == PWR_WAKEUP) {
		/* timeout = arg->cbData; */
		status = PWR_WakeDSP(timeout);
	} else
	    if (DSP_SUCCEEDED
		((*pProcObject->pIntfFxns->pfnDevCntrl)
			(pProcObject->hWmdContext, dwCmd, arg))) {
		status = DSP_SOK;
	} else {
		status = DSP_EFAIL;
	}

	return status;
}

/*
 *  ======== PROC_Detach ========
 *  Purpose:
 *      Destroys the  Processor Object. Removes the notification from the Dev
 *      List.
 */
DSP_STATUS PROC_Detach(struct PROCESS_CONTEXT *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = NULL;

	DBC_Require(cRefs > 0);

	pProcObject = (struct PROC_OBJECT *)pr_ctxt->hProcessor;

	if (MEM_IsValidHandle(pProcObject, PROC_SIGNATURE)) {
		if (pProcObject->hNtfy) {
			/* Notify the Client */
			NTFY_Notify(pProcObject->hNtfy, DSP_PROCESSORDETACH);
			/* Remove the notification memory */
			NTFY_Delete(pProcObject->hNtfy);
		}
		kfree(pProcObject->g_pszLastCoff);
		pProcObject->g_pszLastCoff = NULL;
		/* Remove the Proc from the DEV List */
		(void)DEV_RemoveProcObject(pProcObject->hDevObject,
			(u32)pProcObject);
		/* Free the Processor Object */
		MEM_FreeObject(pProcObject);
		pr_ctxt->hProcessor = NULL;
	} else {
		status = DSP_EHANDLE;
	}

	return status;
}

/*
 *  ======== PROC_EnumNodes ========
 *  Purpose:
 *      Enumerate and get configuration information about nodes allocated
 *      on a DSP processor.
 */
DSP_STATUS PROC_EnumNodes(void *hProcessor, void **aNodeTab,
		IN u32 uNodeTabSize, OUT u32 *puNumNodes,
		OUT u32 *puAllocated)
{
	DSP_STATUS status = DSP_EFAIL;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct NODE_MGR *hNodeMgr = NULL;

	DBC_Require(cRefs > 0);
	DBC_Require(aNodeTab != NULL || uNodeTabSize == 0);
	DBC_Require(puNumNodes != NULL);
	DBC_Require(puAllocated != NULL);


	if (DSP_SUCCEEDED(DEV_GetNodeManager(pProcObject->hDevObject,
			 &hNodeMgr))) {
		if (hNodeMgr) {
			status = NODE_EnumNodes(hNodeMgr, aNodeTab,
						uNodeTabSize,
						puNumNodes,
						puAllocated);
		}
	}


	return status;
}

/* Cache operation against kernel address instead of users */
static int memory_sync_page(struct vm_area_struct *vma, unsigned long start,
			    ssize_t len, u32 ulFlags)
{
	struct page *page;
	void *kaddr;
	unsigned long offset;
	ssize_t rest;

	while (len) {
		page = follow_page(vma, start, FOLL_GET);
		if (!page) {
			pr_err("%s: no page for %08lx\n", __func__, start);
			return -EINVAL;
		} else if (IS_ERR(page)) {
			pr_err("%s: err page for %08lx(%lu)\n", __func__, start,
			       IS_ERR(page));
			return IS_ERR(page);
		}

		offset = start & ~PAGE_MASK;
		kaddr = kmap(page) + offset;
		rest = min_t(ssize_t, PAGE_SIZE - offset, len);

		MEM_FlushCache(kaddr, rest, ulFlags);

		kunmap(page);
		put_page(page);
		len -= rest;
		start += rest;
	}

	return 0;
}

/* Check if the given area blongs to process virtul memory address space */
static int memory_sync_vma(unsigned long start, u32 len, u32 ulFlags)
{
	int err = 0;
	unsigned long end;
	struct vm_area_struct *vma;

	end = start + len;
	if (end <= start)
		return -EINVAL;

	while ((vma = find_vma(current->mm, start)) != NULL) {
		ssize_t size;

		if (vma->vm_flags & (VM_IO | VM_PFNMAP))
			return -EINVAL;

		if (vma->vm_start > start)
			return -EINVAL;

		size = min_t(ssize_t, vma->vm_end - start, len);
		err = memory_sync_page(vma, start, size, ulFlags);
		if (err)
			break;

		if (end <= vma->vm_end)
			break;

		start = vma->vm_end;
	}

	if (!vma)
		err = -EINVAL;

	return err;
}

static DSP_STATUS proc_memory_sync(void *hProcessor, void *pMpuAddr,
				   u32 ulSize, u32 ulFlags)
{
	/* Keep STATUS here for future additions to this function */
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);

#ifdef CONFIG_BRIDGE_CHECK_ALIGN_128
	if (!IS_ALIGNED((u32)pMpuAddr, DSP_CACHE_SIZE) ||
	    !IS_ALIGNED(ulSize, DSP_CACHE_SIZE)) {
		pr_err("%s: Invalid alignment %p %x\n",
			__func__, pMpuAddr, ulSize);
		return DSP_EALIGNMENT;
	}
#endif /* CONFIG_BRIDGE_CHECK_ALIGN_128 */

	if (ulFlags == 3)
		__cpuc_flush_kern_all();
	else {
		down_read(&current->mm->mmap_sem);
		if (memory_sync_vma((u32)pMpuAddr, ulSize, ulFlags)) {
				pr_err("%s: InValid address parameters %p %x\n",
				__func__, pMpuAddr, ulSize);
				status = DSP_EHANDLE;
		}
		up_read(&current->mm->mmap_sem);
	}

	return status;
}

/*
 *  ======== PROC_FlushMemory ========
 *  Purpose:
 *     Flush cache
 */
DSP_STATUS PROC_FlushMemory(void *hProcessor, void *pMpuAddr,
			    u32 ulSize, u32 ulFlags)
{
	return proc_memory_sync(hProcessor, pMpuAddr, ulSize, ulFlags);
}

/*
 *  ======== PROC_InvalidateMemory ========
 *  Purpose:
 *     Invalidates the memory specified
 */
DSP_STATUS PROC_InvalidateMemory(void *hProcessor, void *pMpuAddr,
				 u32 ulSize)
{
	enum DSP_FLUSHTYPE mtype = PROC_INVALIDATE_MEM;

	return proc_memory_sync(hProcessor, pMpuAddr, ulSize, mtype);
}

/*
 *  ======== PROC_GetResourceInfo ========
 *  Purpose:
 *      Enumerate the resources currently available on a processor.
 */
DSP_STATUS PROC_GetResourceInfo(void *hProcessor, u32 uResourceType,
				OUT struct DSP_RESOURCEINFO *pResourceInfo,
				u32 uResourceInfoSize)
{
	DSP_STATUS status = DSP_EFAIL;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct NODE_MGR *hNodeMgr = NULL;
	struct NLDR_OBJECT *hNldr = NULL;
	struct RMM_TargetObj *rmm = NULL;
	struct IO_MGR *hIOMgr = NULL;		/* IO manager handle */

	DBC_Require(cRefs > 0);
	DBC_Require(pResourceInfo != NULL);
	DBC_Require(uResourceInfoSize >= sizeof(struct DSP_RESOURCEINFO));


	switch (uResourceType) {
	case DSP_RESOURCE_DYNDARAM:
	case DSP_RESOURCE_DYNSARAM:
	case DSP_RESOURCE_DYNEXTERNAL:
	case DSP_RESOURCE_DYNSRAM:
		status = DEV_GetNodeManager(pProcObject->hDevObject,
								&hNodeMgr);
		if (!hNodeMgr) {
			status = DSP_EHANDLE;
			goto func_end;
		}

		status = NODE_GetNldrObj(hNodeMgr, &hNldr);
		if (DSP_SUCCEEDED(status)) {
			status = NLDR_GetRmmManager(hNldr, &rmm);
			if (rmm) {
				if (!RMM_stat(rmm,
				   (enum DSP_MEMTYPE)uResourceType,
				   (struct DSP_MEMSTAT *)&(pResourceInfo->
				   result.memStat)))
					status = DSP_EVALUE;
			} else {
				status = DSP_EHANDLE;
			}
		}
		break;
	case DSP_RESOURCE_PROCLOAD:
		status = DEV_GetIOMgr(pProcObject->hDevObject, &hIOMgr);
		if (hIOMgr)
			status = pProcObject->pIntfFxns->pfnIOGetProcLoad(
				hIOMgr, (struct DSP_PROCLOADSTAT *)&
				(pResourceInfo->result.procLoadStat));
		else
			status = DSP_EHANDLE;
		break;
	default:
		status = DSP_EFAIL;
		break;
	}
func_end:
	return status;
}

/*
 *  ======== PROC_Exit ========
 *  Purpose:
 *      Decrement reference count, and free resources when reference count is
 *      0.
 */
void PROC_Exit(void)
{
	DBC_Require(cRefs > 0);

	if (hProcLock)
		(void)SYNC_DeleteCS(hProcLock);

	cRefs--;

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== PROC_GetDevObject ========
 *  Purpose:
 *      Return the Dev Object handle for a given Processor.
 *
 */
DSP_STATUS PROC_GetDevObject(void *hProcessor,
			     struct DEV_OBJECT **phDevObject)
{
	DSP_STATUS status = DSP_EFAIL;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;

	DBC_Require(cRefs > 0);
	DBC_Require(phDevObject != NULL);

	*phDevObject = pProcObject->hDevObject;
	status = DSP_SOK;

	DBC_Ensure((DSP_SUCCEEDED(status) && *phDevObject != NULL) ||
		   (DSP_FAILED(status) && *phDevObject == NULL));

	return status;
}

/*
 *  ======== PROC_GetState ========
 *  Purpose:
 *      Report the state of the specified DSP processor.
 */
DSP_STATUS PROC_GetState(void *hProcessor,
			OUT struct DSP_PROCESSORSTATE *pProcStatus,
			u32 uStateInfoSize)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	int brdStatus;
	struct DEH_MGR *hDehMgr;

	DBC_Require(cRefs > 0);
	DBC_Require(pProcStatus != NULL);
	DBC_Require(uStateInfoSize >= sizeof(struct DSP_PROCESSORSTATE));


	/* First, retrieve BRD state information */
	status = (*pProcObject->pIntfFxns->pfnBrdStatus)
			   (pProcObject->hWmdContext, &brdStatus);
	if (DSP_SUCCEEDED(status)) {
		switch (brdStatus) {
		case BRD_STOPPED:
			pProcStatus->iState = PROC_STOPPED;
			break;
		case BRD_SLEEP_TRANSITION:
		case BRD_DSP_HIBERNATION:
			/* Fall through */
		case BRD_RUNNING:
			pProcStatus->iState = PROC_RUNNING;
			break;
		case BRD_LOADED:
			pProcStatus->iState = PROC_LOADED;
			break;
		case BRD_ERROR:
			pProcStatus->iState = PROC_ERROR;
			break;
		default:
			pProcStatus->iState = 0xFF;
			status = DSP_EFAIL;
		}
	}
	/* Next, retrieve error information, if any */
	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetDehMgr(pProcObject->hDevObject, &hDehMgr);
		if (DSP_SUCCEEDED(status) && hDehMgr)
			status = (*pProcObject->pIntfFxns->pfnDehGetInfo)
				 (hDehMgr, &(pProcStatus->errInfo));
	}
	GT_2trace(PROC_DebugMask, GT_ENTER,
		 "Exiting PROC_GetState, results:\n\t"
		 "status:  0x%x\n\tpProcStatus: 0x%x\n", status,
		 pProcStatus->iState);
	return status;
}

/*
 *  ======== PROC_GetTrace ========
 *  Purpose:
 *      Retrieve the current contents of the trace buffer, located on the
 *      Processor.  Predefined symbols for the trace buffer must have been
 *      configured into the DSP executable.
 *  Details:
 *      We support using the symbols SYS_PUTCBEG and SYS_PUTCEND to define a
 *      trace buffer, only.  Treat it as an undocumented feature.
 *      This call is destructive, meaning the processor is placed in the monitor
 *      state as a result of this function.
 */
DSP_STATUS PROC_GetTrace(void *hProcessor, u8 *pBuf, u32 uMaxSize)
{
	DSP_STATUS status;
	status = DSP_ENOTIMPL;
	return status;
}

/*
 *  ======== PROC_Init ========
 *  Purpose:
 *      Initialize PROC's private state, keeping a reference count on each call
 */
bool PROC_Init(void)
{
	bool fRetval = true;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		/* Set the Trace mask */
		DBC_Assert(!PROC_DebugMask.flags);
		GT_create(&PROC_DebugMask, "PR");  /* "PR" for Processor */

		(void)SYNC_InitializeCS(&hProcLock);
	}

	if (fRetval)
		cRefs++;

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	return fRetval;
}

/*
 *  ======== PROC_Load ========
 *  Purpose:
 *      Reset a processor and load a new base program image.
 *      This will be an OEM-only function, and not part of the DSP/BIOS Bridge
 *      application developer's API.
 */
DSP_STATUS PROC_Load(void *hProcessor, IN CONST s32 iArgc,
		    IN CONST char **aArgv, IN CONST char **aEnvp)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct IO_MGR *hIOMgr;		/* IO manager handle */
	struct MSG_MGR *hMsgMgr;
	struct COD_MANAGER *hCodMgr;	/* Code manager handle */
	char *pargv0;		/* temp argv[0] ptr */
	char **newEnvp;		/* Updated envp[] array. */
	char szProcID[MAXPROCIDLEN];	/* Size of "PROC_ID=<n>" */
	s32 cEnvp;		/* Num elements in envp[]. */
	s32 cNewEnvp;		/* "  " in newEnvp[]     */
	s32 nProcID = 0;	/* Anticipate MP version. */
	struct DCD_MANAGER *hDCDHandle;
	struct DMM_OBJECT *hDmmMgr;
	u32 dwExtEnd;
	u32 uProcId;
#ifdef CONFIG_BRIDGE_DEBUG
	int uBrdState;
#endif

#ifdef OPT_LOAD_TIME_INSTRUMENTATION
	struct timeval tv1;
	struct timeval tv2;
#endif

#if defined(CONFIG_BRIDGE_DVFS) && !defined(CONFIG_CPU_FREQ)
	struct dspbridge_platform_data *pdata =
				omap_dspbridge_dev->dev.platform_data;
#endif

	DBC_Require(cRefs > 0);
	DBC_Require(iArgc > 0);
	DBC_Require(aArgv != NULL);

#ifdef OPT_LOAD_TIME_INSTRUMENTATION
	do_gettimeofday(&tv1);
#endif

	DEV_GetCodMgr(pProcObject->hDevObject, &hCodMgr);
	if (!hCodMgr) {
		status = DSP_EFAIL;
		goto func_end;
	}
	status = PROC_Stop(hProcessor);
	if (DSP_FAILED(status))
		goto func_end;

	/* Place the board in the monitor state. */
	status = PROC_Monitor(hProcessor);
	if (DSP_FAILED(status))
		goto func_end;

	/* Save ptr to  original argv[0]. */
	pargv0 = (char *)aArgv[0];
	/*Prepend "PROC_ID=<nProcID>"to envp array for target.*/
	cEnvp = GetEnvpCount((char **)aEnvp);
	cNewEnvp = (cEnvp ? (cEnvp + 1) : (cEnvp + 2));
	newEnvp = MEM_Calloc(cNewEnvp * sizeof(char **), MEM_PAGED);
	if (newEnvp) {
		status = snprintf(szProcID, MAXPROCIDLEN, PROC_ENVPROCID,
				    nProcID);
		if (status == -1) {
			GT_0trace(PROC_DebugMask, GT_7CLASS, "PROC_Load: "
				 "Proc ID string overflow \n");
			status = DSP_EFAIL;
		} else {
			newEnvp = PrependEnvp(newEnvp, (char **)aEnvp, cEnvp,
					     cNewEnvp, szProcID);
			/* Get the DCD Handle */
			status = MGR_GetDCDHandle(pProcObject->hMgrObject,
						 (u32 *)&hDCDHandle);
			if (DSP_SUCCEEDED(status)) {
				/*  Before proceeding with new load,
				 *  check if a previously registered COFF
				 *  exists.
				 *  If yes, unregister nodes in previously
				 *  registered COFF.  If any error occurred,
				 *  set previously registered COFF to NULL.  */
				if (pProcObject->g_pszLastCoff != NULL) {
					status = DCD_AutoUnregister(hDCDHandle,
						 pProcObject->g_pszLastCoff);
					/* Regardless of auto unregister status,
					 *  free previously allocated
					 *  memory.  */
					kfree(pProcObject->g_pszLastCoff);
					pProcObject->g_pszLastCoff = NULL;
				}
			}
			/* On success, do COD_OpenBase() */
			status = COD_OpenBase(hCodMgr, (char *)aArgv[0],
					     COD_SYMB);
		}
	} else {
		status = DSP_EMEMORY;
	}
	if (DSP_SUCCEEDED(status)) {
		/* Auto-register data base */
		/* Get the DCD Handle */
		status = MGR_GetDCDHandle(pProcObject->hMgrObject,
					 (u32 *)&hDCDHandle);
		if (DSP_SUCCEEDED(status)) {
			/*  Auto register nodes in specified COFF
			 *  file.  If registration did not fail,
			 *  (status = DSP_SOK or DSP_EDCDNOAUTOREGISTER)
			 *  save the name of the COFF file for
			 *  de-registration in the future.  */
			status = DCD_AutoRegister(hDCDHandle, (char *)aArgv[0]);
			if (status == DSP_EDCDNOAUTOREGISTER)
				status = DSP_SOK;

			if (DSP_FAILED(status)) {
				status = DSP_EFAIL;
			} else {
				DBC_Assert(pProcObject->g_pszLastCoff == NULL);
				/* Allocate memory for pszLastCoff */
				pProcObject->g_pszLastCoff = MEM_Calloc(
					(strlen((char *)aArgv[0]) + 1),
					MEM_PAGED);
				/* If memory allocated, save COFF file name*/
				if (pProcObject->g_pszLastCoff) {
					strncpy(pProcObject->g_pszLastCoff,
						(char *)aArgv[0],
						(strlen((char *)aArgv[0]) + 1));
				}
			}
		}
	}
	/* Update shared memory address and size */
	if (DSP_SUCCEEDED(status)) {
		/*  Create the message manager. This must be done
		 *  before calling the IOOnLoaded function.  */
		DEV_GetMsgMgr(pProcObject->hDevObject, &hMsgMgr);
		if (!hMsgMgr) {
			status = MSG_Create(&hMsgMgr, pProcObject->hDevObject,
					   (MSG_ONEXIT)NODE_OnExit);
			DBC_Assert(DSP_SUCCEEDED(status));
			DEV_SetMsgMgr(pProcObject->hDevObject, hMsgMgr);
		}
	}
	if (DSP_SUCCEEDED(status)) {
		/* Set the Device object's message manager */
		status = DEV_GetIOMgr(pProcObject->hDevObject, &hIOMgr);
		if (hIOMgr)
			status = (*pProcObject->pIntfFxns->pfnIOOnLoaded)
								(hIOMgr);
		else
			status = DSP_EHANDLE;
	}
	if (DSP_SUCCEEDED(status)) {
		/* Now, attempt to load an exec: */

	/* Boost the OPP level to Maximum level supported by baseport*/
#if defined(CONFIG_BRIDGE_DVFS) && !defined(CONFIG_CPU_FREQ)
	if (pdata->cpu_set_freq)
		(*pdata->cpu_set_freq)(pdata->mpu_max_speed);
#endif
		status = COD_LoadBase(hCodMgr, iArgc, (char **)aArgv,
				     DEV_BrdWriteFxn,
				     pProcObject->hDevObject, NULL);
		if (DSP_FAILED(status)) {
			if (status == COD_E_OPENFAILED) {
				GT_0trace(PROC_DebugMask, GT_7CLASS,
					"PROC_Load:Failure to Load the EXE\n");
			}
			if (status == COD_E_SYMBOLNOTFOUND) {
				pr_err("%s: Couldn't parse the file\n",
								__func__);
			}
		}
	/* Requesting the lowest opp supported*/
#if defined(CONFIG_BRIDGE_DVFS) && !defined(CONFIG_CPU_FREQ)
	if (pdata->cpu_set_freq)
		(*pdata->cpu_set_freq)(pdata->mpu_min_speed);
#endif

	}
	if (DSP_SUCCEEDED(status)) {
		/* Update the Processor status to loaded */
		status = (*pProcObject->pIntfFxns->pfnBrdSetState)
			 (pProcObject->hWmdContext, BRD_LOADED);
		if (DSP_SUCCEEDED(status)) {
			pProcObject->sState = PROC_LOADED;
			if (pProcObject->hNtfy)
				PROC_NotifyClients(pProcObject,
						 DSP_PROCESSORSTATECHANGE);
		}
	}
	if (DSP_SUCCEEDED(status)) {
		status = PROC_GetProcessorId(hProcessor, &uProcId);
		if (uProcId == DSP_UNIT) {
			/* Use all available DSP address space after EXTMEM
			 * for DMM */
			if (DSP_SUCCEEDED(status))
				status = COD_GetSymValue(hCodMgr, EXTEND,
								&dwExtEnd);

			/* Reset DMM structs and add an initial free chunk*/
			if (DSP_SUCCEEDED(status)) {
				status = DEV_GetDmmMgr(pProcObject->hDevObject,
						      &hDmmMgr);
				if (hDmmMgr) {
					/* Set dwExtEnd to DMM START u8
					  * address */
					dwExtEnd = (dwExtEnd + 1) * DSPWORDSIZE;
					 /* DMM memory is from EXT_END */
					status = DMM_CreateTables(hDmmMgr,
						dwExtEnd, DMMPOOLSIZE);
				} else {
					status = DSP_EHANDLE;
				}
			}
		}
	}
	/* Restore the original argv[0] */
	kfree(newEnvp);
	aArgv[0] = pargv0;
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status)) {
		if (DSP_SUCCEEDED((*pProcObject->pIntfFxns->pfnBrdStatus)
		   (pProcObject->hWmdContext, &uBrdState))) {
			pr_info("%s: Processor Loaded %s\n", __func__, pargv0);
			DBC_Assert(uBrdState == BRD_LOADED);
		}
	}
#endif
func_end:
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_FAILED(status))
		pr_err("%s: Processor failed to load\n", __func__);
#endif
	DBC_Ensure((DSP_SUCCEEDED(status) && pProcObject->sState == PROC_LOADED)
		   || DSP_FAILED(status));
#ifdef OPT_LOAD_TIME_INSTRUMENTATION
	do_gettimeofday(&tv2);
	if (tv2.tv_usec < tv1.tv_usec) {
		tv2.tv_usec += 1000000;
		tv2.tv_sec--;
	}
	GT_2trace(PROC_DebugMask, GT_1CLASS,
			"Proc_Load: time to load %d sec and %d usec \n",
		    tv2.tv_sec - tv1.tv_sec, tv2.tv_usec - tv1.tv_usec);
#endif
	return status;
}

/*
 *  ======== PROC_Map ========
 *  Purpose:
 *      Maps a MPU buffer to DSP address space.
 */
DSP_STATUS PROC_Map(void *hProcessor, void *pMpuAddr, u32 ulSize,
		   void *pReqAddr, void **ppMapAddr, u32 ulMapAttr,
		   struct PROCESS_CONTEXT *pr_ctxt)
{
	u32 vaAlign;
	u32 paAlign;
	struct DMM_OBJECT *hDmmMgr;
	u32 sizeAlign;
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct DMM_MAP_OBJECT *map_obj;

	GT_6trace(PROC_DebugMask, GT_ENTER, "Entered PROC_Map, args:\n\t"
		 "hProcessor %x, pMpuAddr %x, ulSize %x, pReqAddr %x, "
		 "ulMapAttr %x, ppMapAddr %x\n", hProcessor, pMpuAddr, ulSize,
		 pReqAddr, ulMapAttr, ppMapAddr);

#ifdef CONFIG_BRIDGE_CACHE_LINE_CHECK
	if ((ulMapAttr & BUFMODE_MASK) != RBUF) {
		if (!IS_ALIGNED((u32)pMpuAddr, DSP_CACHE_LINE) ||
		    !IS_ALIGNED(ulSize, DSP_CACHE_LINE)) {
			pr_err("%s: not aligned: 0x%x (%d)\n", __func__,
						(u32)pMpuAddr, ulSize);
			return -EFAULT;
		}
	}
#endif

	/* Calculate the page-aligned PA, VA and size */
	vaAlign = PG_ALIGN_LOW((u32) pReqAddr, PG_SIZE_4K);
	paAlign = PG_ALIGN_LOW((u32) pMpuAddr, PG_SIZE_4K);
	sizeAlign = PG_ALIGN_HIGH(ulSize + (u32)pMpuAddr - paAlign,
				 PG_SIZE_4K);

	GT_3trace(PROC_DebugMask, GT_ENTER, "PROC_Map: vaAlign %x, paAlign %x, "
		 "sizeAlign %x\n", vaAlign, paAlign, sizeAlign);

	/* Critical section */
	(void)SYNC_EnterCS(hProcLock);
	status = DMM_GetHandle(pProcObject, &hDmmMgr);
	if (hDmmMgr)
		status = DMM_MapMemory(hDmmMgr, vaAlign, sizeAlign);
	else
		status = DSP_EHANDLE;

	/* Add mapping to the page tables. */
	if (DSP_SUCCEEDED(status)) {

		status = (*pProcObject->pIntfFxns->pfnBrdMemMap)
			(pProcObject->hWmdContext, paAlign, vaAlign, sizeAlign,
			ulMapAttr);
	}
	if (DSP_SUCCEEDED(status)) {
		/* Mapped address = MSB of VA | LSB of PA */
		*ppMapAddr = (void *) (vaAlign | ((u32) pMpuAddr &
			     (PG_SIZE_4K - 1)));
	} else {
		DMM_UnMapMemory(hDmmMgr, vaAlign, &sizeAlign);
	}
	(void)SYNC_LeaveCS(hProcLock);

	if (DSP_FAILED(status))
		goto func_end;

	/*
	 * A successful map should be followed by insertion of map_obj
	 * into dmm_map_list, so that mapped memory resource tracking
	 * remains uptodate
	 */
	map_obj = kmalloc(sizeof(struct DMM_MAP_OBJECT), GFP_KERNEL);
	if (map_obj) {
		map_obj->dsp_addr = (u32)*ppMapAddr;
		spin_lock(&pr_ctxt->dmm_map_lock);
		list_add(&map_obj->link, &pr_ctxt->dmm_map_list);
		spin_unlock(&pr_ctxt->dmm_map_lock);
	}

func_end:
	GT_1trace(PROC_DebugMask, GT_ENTER, "Leaving PROC_Map [0x%x]", status);
	return status;
}

/*
 *  ======== PROC_RegisterNotify ========
 *  Purpose:
 *      Register to be notified of specific processor events.
 */
DSP_STATUS PROC_RegisterNotify(void *hProcessor, u32 uEventMask,
			      u32 uNotifyType, struct DSP_NOTIFICATION
			      *hNotification)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct DEH_MGR *hDehMgr;

	DBC_Require(hNotification != NULL);
	DBC_Require(cRefs > 0);

	/* Check if event mask is a valid processor related event */
	if (uEventMask & ~(DSP_PROCESSORSTATECHANGE | DSP_PROCESSORATTACH |
	   DSP_PROCESSORDETACH | DSP_PROCESSORRESTART | DSP_MMUFAULT |
	   DSP_SYSERROR | DSP_PWRERROR | DSP_WDTOVERFLOW))
		status = DSP_EVALUE;

	/* Check if notify type is valid */
	if (uNotifyType != DSP_SIGNALEVENT)
		status = DSP_EVALUE;

	if (DSP_SUCCEEDED(status)) {
		/* If event mask is not DSP_SYSERROR, DSP_MMUFAULT,
		 * or DSP_PWRERROR then register event immediately. */
		if (uEventMask &
		    ~(DSP_SYSERROR | DSP_MMUFAULT | DSP_PWRERROR |
				DSP_WDTOVERFLOW)) {
			status = NTFY_Register(pProcObject->hNtfy,
				 hNotification,	uEventMask, uNotifyType);
			/* Special case alert, special case alert!
			 * If we're trying to *deregister* (i.e. uEventMask
			 * is 0), a DSP_SYSERROR or DSP_MMUFAULT notification,
			 * we have to deregister with the DEH manager.
			 * There's no way to know, based on uEventMask which
			 * manager the notification event was registered with,
			 * so if we're trying to deregister and NTFY_Register
			 * failed, we'll give the deh manager a shot.
			 */
			if ((uEventMask == 0) && DSP_FAILED(status)) {
				status = DEV_GetDehMgr(pProcObject->hDevObject,
					 &hDehMgr);
				DBC_Assert(pProcObject->pIntfFxns->
					   pfnDehRegisterNotify);
				status = (*pProcObject->pIntfFxns->
					 pfnDehRegisterNotify)
					 (hDehMgr, uEventMask, uNotifyType,
					 hNotification);
			}
		} else {
			status = DEV_GetDehMgr(pProcObject->hDevObject,
					      &hDehMgr);
			DBC_Assert(pProcObject->pIntfFxns->
				  pfnDehRegisterNotify);
			status = (*pProcObject->pIntfFxns->pfnDehRegisterNotify)
				 (hDehMgr, uEventMask, uNotifyType,
				 hNotification);

		}
	}
	return status;
}

/*
 *  ======== PROC_ReserveMemory ========
 *  Purpose:
 *      Reserve a virtually contiguous region of DSP address space.
 */
DSP_STATUS PROC_ReserveMemory(void *hProcessor, u32 ulSize,
		     void **ppRsvAddr, struct PROCESS_CONTEXT *pr_ctxt)
{
	struct DMM_OBJECT *hDmmMgr;
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct DMM_RSV_OBJECT *rsv_obj;

	GT_3trace(PROC_DebugMask, GT_ENTER,
		 "Entered PROC_ReserveMemory, args:\n\t"
		 "hProcessor: 0x%x ulSize: 0x%x ppRsvAddr: 0x%x\n", hProcessor,
		 ulSize, ppRsvAddr);

	status = DMM_GetHandle(pProcObject, &hDmmMgr);
	if (hDmmMgr)
		status = DMM_ReserveMemory(hDmmMgr, ulSize, (u32 *)ppRsvAddr);
	else
		status = DSP_EHANDLE;

	if (status != DSP_SOK)
		goto func_end;

	/*
	 * A successful reserve should be followed by insertion of rsv_obj
	 * into dmm_rsv_list, so that reserved memory resource tracking
	 * remains uptodate
	 */
	rsv_obj = kmalloc(sizeof(struct DMM_RSV_OBJECT), GFP_KERNEL);
	if (rsv_obj) {
		rsv_obj->dsp_reserved_addr = (u32) *ppRsvAddr;
		spin_lock(&pr_ctxt->dmm_rsv_lock);
		list_add(&rsv_obj->link, &pr_ctxt->dmm_rsv_list);
		spin_unlock(&pr_ctxt->dmm_rsv_lock);
	}

func_end:
	GT_1trace(PROC_DebugMask, GT_ENTER, "Leaving PROC_ReserveMemory [0x%x]",
		 status);
	return status;
}

/*
 *  ======== PROC_Start ========
 *  Purpose:
 *      Start a processor running.
 */
DSP_STATUS PROC_Start(void *hProcessor)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct COD_MANAGER *hCodMgr;	/* Code manager handle    */
	u32 dwDspAddr;	/* Loaded code's entry point.    */
#ifdef CONFIG_BRIDGE_DEBUG
	int uBrdState;
#endif
	DBC_Require(cRefs > 0);

	/* Call the WMD_BRD_Start */
	if (pProcObject->sState != PROC_LOADED) {
		status = DSP_EWRONGSTATE;
		goto func_end;
	}
	status = DEV_GetCodMgr(pProcObject->hDevObject, &hCodMgr);
	if (!hCodMgr) {
		status = DSP_EHANDLE;
		goto func_cont;
	}

	status = COD_GetEntry(hCodMgr, &dwDspAddr);
	if (DSP_FAILED(status))
		goto func_cont;

	status = (*pProcObject->pIntfFxns->pfnBrdStart)
		 (pProcObject->hWmdContext, dwDspAddr);
	if (DSP_FAILED(status))
		goto func_cont;

	/* Call DEV_Create2 */
	status = DEV_Create2(pProcObject->hDevObject);
	if (DSP_SUCCEEDED(status)) {
		pProcObject->sState = PROC_RUNNING;
		/* Deep sleep switces off the peripheral clocks.
		 * we just put the DSP CPU in idle in the idle loop.
		 * so there is no need to send a command to DSP */

		if (pProcObject->hNtfy) {
			PROC_NotifyClients(pProcObject,
					  DSP_PROCESSORSTATECHANGE);
		}
	} else {
		/* Failed to Create Node Manager and DISP Object
		 * Stop the Processor from running. Put it in STOPPED State */
		(void)(*pProcObject->pIntfFxns->pfnBrdStop)(pProcObject->
			hWmdContext);
		pProcObject->sState = PROC_STOPPED;
	}
func_cont:
#ifdef CONFIG_BRIDGE_DEBUG
	if (DSP_SUCCEEDED(status)) {
		if (DSP_SUCCEEDED((*pProcObject->pIntfFxns->pfnBrdStatus)
		   (pProcObject->hWmdContext, &uBrdState))) {
			pr_info("%s: dsp in running state\n", __func__);
			DBC_Assert(uBrdState != BRD_HIBERNATION);
		}
	} else {
		pr_err("%s: Failed to start the dsp\n", __func__);
	}
#endif
func_end:
	DBC_Ensure((DSP_SUCCEEDED(status) && pProcObject->sState ==
		  PROC_RUNNING)	|| DSP_FAILED(status));
	return status;
}

/*
 *  ======== PROC_Stop ========
 *  Purpose:
 *      Stop a processor running.
 */
DSP_STATUS PROC_Stop(void *hProcessor)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct MSG_MGR *hMsgMgr;
	struct NODE_MGR *hNodeMgr;
	void *hNode;
	u32 uNodeTabSize = 1;
	u32 uNumNodes = 0;
	u32 uNodesAllocated = 0;
	int uBrdState;

	DBC_Require(cRefs > 0);

	if (DSP_SUCCEEDED((*pProcObject->pIntfFxns->pfnBrdStatus)
	   (pProcObject->hWmdContext, &uBrdState))) {
		if (uBrdState == BRD_ERROR)
			WMD_DEH_ReleaseDummyMem();
	}
	/* check if there are any running nodes */
	status = DEV_GetNodeManager(pProcObject->hDevObject, &hNodeMgr);
	if (DSP_SUCCEEDED(status) && hNodeMgr) {
		status = NODE_EnumNodes(hNodeMgr, &hNode, uNodeTabSize,
					&uNumNodes, &uNodesAllocated);
		if ((status == DSP_ESIZE) || (uNodesAllocated > 0)) {
			pr_err("%s: Can't stop device, active nodes = %d \n",
						__func__, uNodesAllocated);
			return DSP_EWRONGSTATE;
		}
	}
	/* Call the WMD_BRD_Stop */
	/* It is OK to stop a device that does n't have nodes OR not started */
	status = (*pProcObject->pIntfFxns->pfnBrdStop)(pProcObject->
		 hWmdContext);
	if (DSP_SUCCEEDED(status)) {
		GT_0trace(PROC_DebugMask, GT_1CLASS,
			 "PROC_Stop: Processor Stopped, "
			 "i.e in standby mode \n");
		pProcObject->sState = PROC_STOPPED;
		/* Destory the Node Manager, MSG Manager */
		if (DSP_SUCCEEDED(DEV_Destroy2(pProcObject->hDevObject))) {
			/* Destroy the MSG by calling MSG_Delete */
			DEV_GetMsgMgr(pProcObject->hDevObject, &hMsgMgr);
			if (hMsgMgr) {
				MSG_Delete(hMsgMgr);
				DEV_SetMsgMgr(pProcObject->hDevObject, NULL);
			}
#ifdef CONFIG_BRIDGE_DEBUG
			if (DSP_SUCCEEDED((*pProcObject->pIntfFxns->
			   pfnBrdStatus)(pProcObject->hWmdContext,
			   &uBrdState)))
				DBC_Assert(uBrdState == BRD_STOPPED);
#endif
		}
	} else {
		pr_err("%s: Failed to stop the processor\n", __func__);
	}


	return status;
}

/*
 *  ======== PROC_UnMap ========
 *  Purpose:
 *      Removes a MPU buffer mapping from the DSP address space.
 */
DSP_STATUS PROC_UnMap(void *hProcessor, void *pMapAddr,
		struct PROCESS_CONTEXT *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct DMM_OBJECT *hDmmMgr;
	u32 vaAlign;
	u32 sizeAlign;
	struct DMM_MAP_OBJECT *map_obj;

	GT_2trace(PROC_DebugMask, GT_ENTER,
		 "Entered PROC_UnMap, args:\n\thProcessor:"
		 "0x%x pMapAddr: 0x%x\n", hProcessor, pMapAddr);

	vaAlign = PG_ALIGN_LOW((u32) pMapAddr, PG_SIZE_4K);

	status = DMM_GetHandle(hProcessor, &hDmmMgr);
	if (!hDmmMgr) {
		status = DSP_EHANDLE;
		goto func_end;
	}

	/* Critical section */
	(void)SYNC_EnterCS(hProcLock);
	/*
	 * Update DMM structures. Get the size to unmap.
	 * This function returns error if the VA is not mapped
	 */
	status = DMM_UnMapMemory(hDmmMgr, (u32) vaAlign, &sizeAlign);
	/* Remove mapping from the page tables. */
	if (DSP_SUCCEEDED(status)) {
		status = (*pProcObject->pIntfFxns->pfnBrdMemUnMap)
			 (pProcObject->hWmdContext, vaAlign, sizeAlign);
	}
	(void)SYNC_LeaveCS(hProcLock);
	if (DSP_FAILED(status))
		goto func_end;

	/*
	 * A successful unmap should be followed by removal of map_obj
	 * from dmm_map_list, so that mapped memory resource tracking
	 * remains uptodate
	 */
	spin_lock(&pr_ctxt->dmm_map_lock);
	list_for_each_entry(map_obj, &pr_ctxt->dmm_map_list, link) {
		if (map_obj->dsp_addr == (u32)pMapAddr) {
			list_del(&map_obj->link);
			kfree(map_obj);
			break;
		}
	}
	spin_unlock(&pr_ctxt->dmm_map_lock);

func_end:
	GT_1trace(PROC_DebugMask, GT_ENTER,
		 "Leaving PROC_UnMap [0x%x]", status);
	return status;
}

/*
 *  ======== PROC_UnReserveMemory ========
 *  Purpose:
 *      Frees a previously reserved region of DSP address space.
 */
DSP_STATUS PROC_UnReserveMemory(void *hProcessor, void *pRsvAddr,
		struct PROCESS_CONTEXT *pr_ctxt)
{
	struct DMM_OBJECT *hDmmMgr;
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProcessor;
	struct DMM_RSV_OBJECT *rsv_obj;

	GT_2trace(PROC_DebugMask, GT_ENTER,
		 "Entered PROC_UnReserveMemory, args:\n\t"
		 "hProcessor: 0x%x pRsvAddr: 0x%x\n", hProcessor, pRsvAddr);

	status = DMM_GetHandle(pProcObject, &hDmmMgr);
	if (hDmmMgr)
		status = DMM_UnReserveMemory(hDmmMgr, (u32) pRsvAddr);
	else
		status = DSP_EHANDLE;

	if (status != DSP_SOK)
		goto func_end;

	/*
	 * A successful unreserve should be followed by removal of rsv_obj
	 * from dmm_rsv_list, so that reserved memory resource tracking
	 * remains uptodate
	 */
	spin_lock(&pr_ctxt->dmm_rsv_lock);
	list_for_each_entry(rsv_obj, &pr_ctxt->dmm_rsv_list, link) {
		if (rsv_obj->dsp_reserved_addr == (u32)pRsvAddr) {
			list_del(&rsv_obj->link);
			kfree(rsv_obj);
			break;
		}
	}
	spin_unlock(&pr_ctxt->dmm_rsv_lock);
func_end:

	GT_1trace(PROC_DebugMask, GT_ENTER,
		 "Leaving PROC_UnReserveMemory [0x%x]",
		 status);

	return status;
}

/*
 *  ======== = PROC_Monitor ======== ==
 *  Purpose:
 *      Place the Processor in Monitor State. This is an internal
 *      function and a requirement before Processor is loaded.
 *      This does a WMD_BRD_Stop, DEV_Destroy2 and WMD_BRD_Monitor.
 *      In DEV_Destroy2 we delete the node manager.
 *  Parameters:
 *      pProcObject:    Pointer to Processor Object
 *  Returns:
 *      DSP_SOK:	Processor placed in monitor mode.
 *      !DSP_SOK:       Failed to place processor in monitor mode.
 *  Requires:
 *      Valid Processor Handle
 *  Ensures:
 *      Success:	ProcObject state is PROC_IDLE
 */
static DSP_STATUS PROC_Monitor(struct PROC_OBJECT *pProcObject)
{
	DSP_STATUS status = DSP_EFAIL;
	struct MSG_MGR *hMsgMgr;
#ifdef CONFIG_BRIDGE_DEBUG
	int uBrdState;
#endif

	DBC_Require(cRefs > 0);

	/* This is needed only when Device is loaded when it is
	 * already 'ACTIVE' */
	/* Destory the Node Manager, MSG Manager */
	if (DSP_SUCCEEDED(DEV_Destroy2(pProcObject->hDevObject))) {
		/* Destroy the MSG by calling MSG_Delete */
		DEV_GetMsgMgr(pProcObject->hDevObject, &hMsgMgr);
		if (hMsgMgr) {
			MSG_Delete(hMsgMgr);
			DEV_SetMsgMgr(pProcObject->hDevObject, NULL);
		}
	}
	/* Place the Board in the Monitor State */
	if (DSP_SUCCEEDED((*pProcObject->pIntfFxns->pfnBrdMonitor)
	   (pProcObject->hWmdContext))) {
		status = DSP_SOK;
#ifdef CONFIG_BRIDGE_DEBUG
		if (DSP_SUCCEEDED((*pProcObject->pIntfFxns->pfnBrdStatus)
		   (pProcObject->hWmdContext, &uBrdState)))
			DBC_Assert(uBrdState == BRD_IDLE);
#endif
	}

#ifdef CONFIG_BRIDGE_DEBUG
	DBC_Ensure((DSP_SUCCEEDED(status) && uBrdState == BRD_IDLE) ||
		  DSP_FAILED(status));
#endif
	return status;
}

/*
 *  ======== GetEnvpCount ========
 *  Purpose:
 *      Return the number of elements in the envp array, including the
 *      terminating NULL element.
 */
static s32 GetEnvpCount(char **envp)
{
	s32 cRetval = 0;
	if (envp) {
		while (*envp++)
			cRetval++;

		cRetval += 1;	/* Include the terminating NULL in the count. */
	}

	return cRetval;
}

/*
 *  ======== PrependEnvp ========
 *  Purpose:
 *      Prepend an environment variable=value pair to the new envp array, and
 *      copy in the existing var=value pairs in the old envp array.
 */
static char **PrependEnvp(char **newEnvp, char **envp, s32 cEnvp, s32 cNewEnvp,
			 char *szVar)
{
	char **ppEnvp = newEnvp;

	DBC_Require(newEnvp);

	/* Prepend new environ var=value string */
	*newEnvp++ = szVar;

	/* Copy user's environment into our own. */
	while (cEnvp--)
		*newEnvp++ = *envp++;

	/* Ensure NULL terminates the new environment strings array. */
	if (cEnvp == 0)
		*newEnvp = NULL;

	return ppEnvp;
}

/*
 *  ======== PROC_NotifyClients ========
 *  Purpose:
 *      Notify the processor the events.
 */
DSP_STATUS PROC_NotifyClients(void *hProc, u32 uEvents)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProc;

	DBC_Require(IsValidProcEvent(uEvents));
	DBC_Require(cRefs > 0);

	NTFY_Notify(pProcObject->hNtfy, uEvents);

	return status;
}

/*
 *  ======== PROC_NotifyAllClients ========
 *  Purpose:
 *      Notify the processor the events. This includes notifying all clients
 *      attached to a particulat DSP.
 */
DSP_STATUS PROC_NotifyAllClients(void *hProc, u32 uEvents)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProc;

	DBC_Require(IsValidProcEvent(uEvents));
	DBC_Require(cRefs > 0);

	if (!MEM_IsValidHandle(pProcObject, PROC_SIGNATURE)) {
		status = DSP_EHANDLE;
		goto func_end;
	}

	DEV_NotifyClients(pProcObject->hDevObject, uEvents);

func_end:
	return status;
}

/*
 *  ======== PROC_GetProcessorId ========
 *  Purpose:
 *      Retrieves the processor ID.
 */
DSP_STATUS PROC_GetProcessorId(void *hProc, u32 *procID)
{
	DSP_STATUS status = DSP_SOK;
	struct PROC_OBJECT *pProcObject = (struct PROC_OBJECT *)hProc;

	if (MEM_IsValidHandle(pProcObject, PROC_SIGNATURE))
		*procID = pProcObject->uProcessor;
	else
		status = DSP_EHANDLE;

	return status;
}

