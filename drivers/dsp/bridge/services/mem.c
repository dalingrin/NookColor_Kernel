/*
 * mem.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Implementation of platform specific memory services.
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

/*  ----------------------------------- This */
#include <dspbridge/mem.h>
#include <dspbridge/list.h>

/*  ----------------------------------- Defines */
#define MEM_512MB   0x1fffffff

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask MEM_debugMask = { NULL, NULL };	/* GT trace variable */
#endif

static bool extPhysMemPoolEnabled;

struct extPhysMemPool {
	u32 physMemBase;
	u32 physMemSize;
	u32 virtMemBase;
	u32 nextPhysAllocPtr;
};

static struct extPhysMemPool extMemPool;

void MEM_ExtPhysPoolInit(u32 poolPhysBase, u32 poolSize)
{
	u32 poolVirtBase;

	/* get the virtual address for the physical memory pool passed */
	poolVirtBase = (u32)ioremap(poolPhysBase, poolSize);

	if ((void **)poolVirtBase == NULL) {
		pr_err("%s: external physical memory map failed\n", __func__);
		extPhysMemPoolEnabled = false;
	} else {
		extMemPool.physMemBase = poolPhysBase;
		extMemPool.physMemSize = poolSize;
		extMemPool.virtMemBase = poolVirtBase;
		extMemPool.nextPhysAllocPtr = poolPhysBase;
		extPhysMemPoolEnabled = true;
	}
}

void MEM_ExtPhysPoolRelease(void)
{
	if (extPhysMemPoolEnabled) {
		iounmap((void *)(extMemPool.virtMemBase));
		extPhysMemPoolEnabled = false;
	}
}

/*
 *  ======== MEM_ExtPhysMemAlloc ========
 *  Purpose:
 *     Allocate physically contiguous, uncached memory from external memory pool
 */

static void *MEM_ExtPhysMemAlloc(u32 bytes, u32 align, OUT u32 *pPhysAddr)
{
	u32 newAllocPtr;
	u32 offset;
	u32 virtAddr;

	if (align == 0)
		align = 1;

	if (bytes > ((extMemPool.physMemBase + extMemPool.physMemSize)
	    - extMemPool.nextPhysAllocPtr)) {
		pPhysAddr = NULL;
		return NULL;
	} else {
		offset = (extMemPool.nextPhysAllocPtr & (align - 1));
		if (offset == 0)
			newAllocPtr = extMemPool.nextPhysAllocPtr;
		else
			newAllocPtr = (extMemPool.nextPhysAllocPtr) +
				      (align - offset);
		if ((newAllocPtr + bytes) <=
		    (extMemPool.physMemBase + extMemPool.physMemSize)) {
			/* we can allocate */
			*pPhysAddr = newAllocPtr;
			extMemPool.nextPhysAllocPtr = newAllocPtr + bytes;
			virtAddr = extMemPool.virtMemBase + (newAllocPtr -
				   extMemPool.physMemBase);
			return (void *)virtAddr;
		} else {
			*pPhysAddr = 0;
			return NULL;
		}
	}
}

/*
 *  ======== MEM_Alloc ========
 *  Purpose:
 *      Allocate memory from the paged or non-paged pools.
 */
void *MEM_Alloc(u32 cBytes, enum MEM_POOLATTRS type)
{
	void *pMem = NULL;

	if (cBytes > 0) {
		switch (type) {
		case MEM_NONPAGED:
		/* If non-paged memory required, see note at top of file. */
		case MEM_PAGED:
			pMem = kmalloc(cBytes,
				(in_atomic()) ? GFP_ATOMIC : GFP_KERNEL);
			break;
		case MEM_LARGEVIRTMEM:
			pMem = vmalloc(cBytes);
			break;

		default:
			break;
		}
	}

	return pMem;
}

/*
 *  ======== MEM_AllocPhysMem ========
 *  Purpose:
 *      Allocate physically contiguous, uncached memory
 */
void *MEM_AllocPhysMem(u32 cBytes, u32 ulAlign, OUT u32 *pPhysicalAddress)
{
	void *pVaMem = NULL;
	dma_addr_t paMem;

	if (cBytes > 0) {
		if (extPhysMemPoolEnabled) {
			pVaMem = MEM_ExtPhysMemAlloc(cBytes, ulAlign,
						    (u32 *)&paMem);
		} else
			pVaMem = dma_alloc_coherent(NULL, cBytes, &paMem,
				(in_atomic()) ? GFP_ATOMIC : GFP_KERNEL);
		if (pVaMem == NULL) {
			*pPhysicalAddress = 0;
		} else {
			*pPhysicalAddress = paMem;
		}
	}
	return pVaMem;
}

/*
 *  ======== MEM_Calloc ========
 *  Purpose:
 *      Allocate zero-initialized memory from the paged or non-paged pools.
 */
void *MEM_Calloc(u32 cBytes, enum MEM_POOLATTRS type)
{
	void *pMem = NULL;

	if (cBytes > 0) {
		switch (type) {
		case MEM_NONPAGED:
		/* If non-paged memory required, see note at top of file. */
		case MEM_PAGED:
			pMem = kzalloc(cBytes,
				(in_atomic()) ? GFP_ATOMIC : GFP_KERNEL);
			break;
		case MEM_LARGEVIRTMEM:
			pMem = __vmalloc(cBytes,
				GFP_KERNEL | __GFP_HIGHMEM | __GFP_ZERO,
				PAGE_KERNEL);
			break;
		default:
			break;
		}
	}

	return pMem;
}

/*
 *  ======== MEM_Exit ========
 *  Purpose:
 *      Discontinue usage of the MEM module.
 */
void MEM_Exit(void)
{
}

/*
 *  ======== MEM_FlushCache ========
 *  Purpose:
 *      Flush cache
 */
void MEM_FlushCache(void *pMemBuf, u32 cBytes, u32 FlushType)
{
	if (!pMemBuf)
		return;

	switch (FlushType) {
	/* invalidate only */
	case PROC_INVALIDATE_MEM:
		dmac_inv_range(pMemBuf, pMemBuf + cBytes);
		outer_inv_range(__pa((u32)pMemBuf), __pa((u32)pMemBuf +
				cBytes));
	break;
	/* writeback only */
	case PROC_WRITEBACK_MEM:
		dmac_clean_range(pMemBuf, pMemBuf + cBytes);
		outer_clean_range(__pa((u32)pMemBuf), __pa((u32)pMemBuf +
				  cBytes));
	break;
	/* writeback and invalidate */
	case PROC_WRITEBACK_INVALIDATE_MEM:
		dmac_flush_range(pMemBuf, pMemBuf + cBytes);
		outer_flush_range(__pa((u32)pMemBuf), __pa((u32)pMemBuf +
				  cBytes));
	break;
	/* Writeback and Invalidate all */
	case PROC_WRBK_INV_ALL:
		__cpuc_flush_kern_all();
		break;
	}

}

/*
 *  ======== MEM_FreePhysMem ========
 *  Purpose:
 *      Free the given block of physically contiguous memory.
 */
void MEM_FreePhysMem(void *pVirtualAddress, u32 pPhysicalAddress,
		     u32 cBytes)
{
	DBC_Require(pVirtualAddress != NULL);

	if (!extPhysMemPoolEnabled)
		dma_free_coherent(NULL, cBytes, pVirtualAddress,
				 pPhysicalAddress);
}

/*
 *  ======== MEM_Init ========
 *  Purpose:
 *      Initialize MEM module private state.
 */
bool MEM_Init(void)
{
	GT_create(&MEM_debugMask, "MM");	/* MM for MeM module */
	return true;
}
