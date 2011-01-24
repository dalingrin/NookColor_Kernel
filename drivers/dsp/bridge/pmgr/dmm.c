/*
 * dmm.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * The Dynamic Memory Manager (DMM) module manages the DSP Virtual address
 * space that can be directly mapped to any MPU buffer or memory region
 *
 * Notes:
 *   Region: Generic memory entitiy having a start address and a size
 *   Chunk:  Reserved region
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

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/errbase.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/mem.h>
#include <dspbridge/sync.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/dev.h>
#include <dspbridge/proc.h>

/*  ----------------------------------- This */
#include <dspbridge/dmm.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
/* Object signatures */
#define DMMSIGNATURE       0x004d4d44	/* "DMM"   (in reverse) */

#define DMM_ADDR_VIRTUAL(a) \
	(((struct MapPage *)(a) - pVirtualMappingTable) * PG_SIZE_4K +\
	dynMemMapBeg)
#define DMM_ADDR_TO_INDEX(a) (((a) - dynMemMapBeg) / PG_SIZE_4K)

/* DMM Mgr */
struct DMM_OBJECT {
	u32 dwSignature;	/* Used for object validation */
	/* Dmm Lock is used to serialize access mem manager for
	 * multi-threads. */
	struct SYNC_CSOBJECT *hDmmLock;	/* Lock to access dmm mgr */
};


/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask DMM_debugMask = { NULL, NULL };	/* GT trace variable */
#endif

static u32 cRefs;		/* module reference count */
struct MapPage {
	u64   RegionSize:31;
	u64   MappedSize:31;
	u64   bReserved:1;
	u64   bMapped:1;
};

/*  Create the free list */
static struct MapPage *pVirtualMappingTable;
static u32  iFreeRegion;	/* The index of free region */
static u32  iFreeSize;
static u32  dynMemMapBeg;	/* The Beginning of dynamic memory mapping */
static u32  TableSize;/* The size of virtual and physical pages tables */

/*  ----------------------------------- Function Prototypes */
static struct MapPage *GetRegion(u32 addr);
static struct MapPage *GetFreeRegion(u32 aSize);
static struct MapPage *GetMappedRegion(u32 aAddr);
#ifdef DSP_DMM_DEBUG
u32 DMM_MemMapDump(struct DMM_OBJECT *hDmmMgr);
#endif

/*  ======== DMM_CreateTables ========
 *  Purpose:
 *      Create table to hold the information of physical address
 *      the buffer pages that is passed by the user, and the table
 *      to hold the information of the virtual memory that is reserved
 *      for DSP.
 */
DSP_STATUS DMM_CreateTables(struct DMM_OBJECT *hDmmMgr, u32 addr, u32 size)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	DSP_STATUS status = DSP_SOK;

	status = DMM_DeleteTables(pDmmObj);
	if (DSP_SUCCEEDED(status)) {
		SYNC_EnterCS(pDmmObj->hDmmLock);
		dynMemMapBeg = addr;
		TableSize = PG_ALIGN_HIGH(size, PG_SIZE_4K)/PG_SIZE_4K;
		/*  Create the free list */
		pVirtualMappingTable = (struct MapPage *) MEM_Calloc
			(TableSize * sizeof(struct MapPage), MEM_LARGEVIRTMEM);
		if (pVirtualMappingTable == NULL)
			status = DSP_EMEMORY;
		else {
			/* On successful allocation,
			* all entries are zero ('free') */
			iFreeRegion = 0;
			iFreeSize = TableSize*PG_SIZE_4K;
			pVirtualMappingTable[0].RegionSize = TableSize;
		}
		SYNC_LeaveCS(pDmmObj->hDmmLock);
	}

	if (DSP_FAILED(status))
		pr_err("%s: failure, status 0x%x\n", __func__, status);

	return status;
}

/*
 *  ======== DMM_Create ========
 *  Purpose:
 *      Create a dynamic memory manager object.
 */
DSP_STATUS DMM_Create(OUT struct DMM_OBJECT **phDmmMgr,
		     struct DEV_OBJECT *hDevObject,
		     IN CONST struct DMM_MGRATTRS *pMgrAttrs)
{
	struct DMM_OBJECT *pDmmObject = NULL;
	DSP_STATUS status = DSP_SOK;
	DBC_Require(cRefs > 0);
	DBC_Require(phDmmMgr != NULL);

	*phDmmMgr = NULL;
	/* create, zero, and tag a cmm mgr object */
	MEM_AllocObject(pDmmObject, struct DMM_OBJECT, DMMSIGNATURE);
	if (pDmmObject != NULL) {
		status = SYNC_InitializeCS(&pDmmObject->hDmmLock);
		if (DSP_SUCCEEDED(status))
			*phDmmMgr = pDmmObject;
		else
			DMM_Destroy(pDmmObject);
	} else {
		status = DSP_EMEMORY;
	}

	return status;
}

/*
 *  ======== DMM_Destroy ========
 *  Purpose:
 *      Release the communication memory manager resources.
 */
DSP_STATUS DMM_Destroy(struct DMM_OBJECT *hDmmMgr)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	if (MEM_IsValidHandle(hDmmMgr, DMMSIGNATURE)) {
		status = DMM_DeleteTables(pDmmObj);
		if (DSP_SUCCEEDED(status)) {
			/* Delete CS & dmm mgr object */
			SYNC_DeleteCS(pDmmObj->hDmmLock);
			MEM_FreeObject(pDmmObj);
		}
	} else
		status = DSP_EHANDLE;

	return status;
}


/*
 *  ======== DMM_DeleteTables ========
 *  Purpose:
 *      Delete DMM Tables.
 */
DSP_STATUS DMM_DeleteTables(struct DMM_OBJECT *hDmmMgr)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	DSP_STATUS status = DSP_SOK;

	DBC_Require(cRefs > 0);
	if (MEM_IsValidHandle(hDmmMgr, DMMSIGNATURE)) {
		/* Delete all DMM tables */
		SYNC_EnterCS(pDmmObj->hDmmLock);

		vfree(pVirtualMappingTable);

		SYNC_LeaveCS(pDmmObj->hDmmLock);
	} else
		status = DSP_EHANDLE;
	return status;
}




/*
 *  ======== DMM_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 */
void DMM_Exit(void)
{
	DBC_Require(cRefs > 0);

	cRefs--;
}

/*
 *  ======== DMM_GetHandle ========
 *  Purpose:
 *      Return the dynamic memory manager object for this device.
 *      This is typically called from the client process.
 */
DSP_STATUS DMM_GetHandle(void *hProcessor,
			OUT struct DMM_OBJECT **phDmmMgr)
{
	DSP_STATUS status = DSP_SOK;
	struct DEV_OBJECT *hDevObject;

	DBC_Require(cRefs > 0);
	DBC_Require(phDmmMgr != NULL);
	if (hProcessor != NULL)
		status = PROC_GetDevObject(hProcessor, &hDevObject);
	else
		hDevObject = DEV_GetFirst();	/* default */

	if (DSP_SUCCEEDED(status))
		status = DEV_GetDmmMgr(hDevObject, phDmmMgr);

	return status;
}

/*
 *  ======== DMM_Init ========
 *  Purpose:
 *      Initializes private state of DMM module.
 */
bool DMM_Init(void)
{
	bool fRetval = true;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		/* Set the Trace mask */
		/*"DM" for Dymanic Memory Manager */
		GT_create(&DMM_debugMask, "DM");
	}

	if (fRetval)
		cRefs++;

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	pVirtualMappingTable = NULL ;
	TableSize = 0;

	return fRetval;
}

/*
 *  ======== DMM_MapMemory ========
 *  Purpose:
 *      Add a mapping block to the reserved chunk. DMM assumes that this block
 *  will be mapped in the DSP/IVA's address space. DMM returns an error if a
 *  mapping overlaps another one. This function stores the info that will be
 *  required later while unmapping the block.
 */
DSP_STATUS DMM_MapMemory(struct DMM_OBJECT *hDmmMgr, u32 addr, u32 size)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct MapPage *chunk;
	DSP_STATUS status = DSP_SOK;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_MapMemory () hDmmMgr %x, "
		 "addr %x, size %x\n", hDmmMgr, addr, size);
	SYNC_EnterCS(pDmmObj->hDmmLock);
	/* Find the Reserved memory chunk containing the DSP block to
	 * be mapped */
	chunk = (struct MapPage *)GetRegion(addr);
	if (chunk != NULL) {
		/* Mark the region 'mapped', leave the 'reserved' info as-is */
		chunk->bMapped = true;
		chunk->MappedSize = (size/PG_SIZE_4K);
	} else
		status = DSP_ENOTFOUND;
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_2trace(DMM_debugMask, GT_4CLASS,
		 "Leaving DMM_MapMemory status %x, chunk %x\n",
		status, chunk);
	return status;
}

/*
 *  ======== DMM_ReserveMemory ========
 *  Purpose:
 *      Reserve a chunk of virtually contiguous DSP/IVA address space.
 */
DSP_STATUS DMM_ReserveMemory(struct DMM_OBJECT *hDmmMgr, u32 size,
							u32 *pRsvAddr)
{
	DSP_STATUS status = DSP_SOK;
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct MapPage *node;
	u32 rsvAddr = 0;
	u32 rsvSize = 0;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_ReserveMemory () hDmmMgr %x, "
		 "size %x, pRsvAddr %x\n", hDmmMgr, size, pRsvAddr);
	SYNC_EnterCS(pDmmObj->hDmmLock);

	/* Try to get a DSP chunk from the free list */
	node = GetFreeRegion(size);
	if (node != NULL) {
		/*  DSP chunk of given size is available. */
		rsvAddr = DMM_ADDR_VIRTUAL(node);
		/* Calculate the number entries to use */
		rsvSize = size/PG_SIZE_4K;
		if (rsvSize < node->RegionSize) {
			/* Mark remainder of free region */
			node[rsvSize].bMapped = false;
			node[rsvSize].bReserved = false;
			node[rsvSize].RegionSize = node->RegionSize - rsvSize;
			node[rsvSize].MappedSize = 0;
		}
		/*  GetRegion will return first fit chunk. But we only use what
			is requested. */
		node->bMapped = false;
		node->bReserved = true;
		node->RegionSize = rsvSize;
		node->MappedSize = 0;
		/* Return the chunk's starting address */
		*pRsvAddr = rsvAddr;
	} else
		/*dSP chunk of given size is not available */
		status = DSP_EMEMORY;

	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_3trace(DMM_debugMask, GT_4CLASS,
		 "Leaving ReserveMemory status %x, rsvAddr"
		 " %x, rsvSize %x\n", status, rsvAddr, rsvSize);
	return status;
}


/*
 *  ======== DMM_UnMapMemory ========
 *  Purpose:
 *      Remove the mapped block from the reserved chunk.
 */
DSP_STATUS DMM_UnMapMemory(struct DMM_OBJECT *hDmmMgr, u32 addr, u32 *pSize)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct MapPage *chunk;
	DSP_STATUS status = DSP_SOK;

	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_UnMapMemory () hDmmMgr %x, "
		 "addr %x, pSize %x\n", hDmmMgr, addr, pSize);
	SYNC_EnterCS(pDmmObj->hDmmLock);
	chunk = GetMappedRegion(addr) ;
	if (chunk == NULL)
		status = DSP_ENOTFOUND ;

	if (DSP_SUCCEEDED(status)) {
		/* Unmap the region */
		*pSize = chunk->MappedSize * PG_SIZE_4K;
		chunk->bMapped = false;
		chunk->MappedSize = 0;
	}
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_3trace(DMM_debugMask, GT_ENTER,
		 "Leaving DMM_UnMapMemory status %x, chunk"
		 " %x,  *pSize %x\n", status, chunk, *pSize);

	return status;
}

/*
 *  ======== DMM_UnReserveMemory ========
 *  Purpose:
 *      Free a chunk of reserved DSP/IVA address space.
 */
DSP_STATUS DMM_UnReserveMemory(struct DMM_OBJECT *hDmmMgr, u32 rsvAddr)
{
	struct DMM_OBJECT *pDmmObj = (struct DMM_OBJECT *)hDmmMgr;
	struct MapPage *chunk;
	u32 i;
	DSP_STATUS status = DSP_SOK;
	u32 chunkSize;

	GT_2trace(DMM_debugMask, GT_ENTER,
		 "Entered DMM_UnReserveMemory () hDmmMgr "
		 "%x, rsvAddr %x\n", hDmmMgr, rsvAddr);

	SYNC_EnterCS(pDmmObj->hDmmLock);

	/* Find the chunk containing the reserved address */
	chunk = GetMappedRegion(rsvAddr);
	if (chunk == NULL)
		status = DSP_ENOTFOUND;

	if (DSP_SUCCEEDED(status)) {
		/* Free all the mapped pages for this reserved region */
		i = 0;
		while (i < chunk->RegionSize) {
			if (chunk[i].bMapped) {
				/* Remove mapping from the page tables. */
				chunkSize = chunk[i].MappedSize;
				/* Clear the mapping flags */
				chunk[i].bMapped = false;
				chunk[i].MappedSize = 0;
				i += chunkSize;
			} else
				i++;
		}
		/* Clear the flags (mark the region 'free') */
		chunk->bReserved = false;
		/* NOTE: We do NOT coalesce free regions here.
		 * Free regions are coalesced in GetRegion(), as it traverses
		 *the whole mapping table
		 */
	}
	SYNC_LeaveCS(pDmmObj->hDmmLock);
	GT_2trace(DMM_debugMask, GT_ENTER,
		 "Leaving DMM_UnReserveMemory status %x"
		 " chunk %x\n", status, chunk);
	return status;
}


/*
 *  ======== GetRegion ========
 *  Purpose:
 *      Returns a region containing the specified memory region
 */
static struct MapPage *GetRegion(u32 aAddr)
{
	struct MapPage *currRegion = NULL;
	u32   i = 0;

	if (pVirtualMappingTable != NULL) {
		/* find page mapped by this address */
		i = DMM_ADDR_TO_INDEX(aAddr);
		if (i < TableSize)
			currRegion = pVirtualMappingTable + i;
	}
	GT_3trace(DMM_debugMask, GT_4CLASS,
	       "Leaving GetRegion currRegion %x, iFreeRegion %d\n,"
	       "iFreeSize %d\n", currRegion, iFreeRegion, iFreeSize) ;
	return currRegion;
}

/*
 *  ======== GetFreeRegion ========
 *  Purpose:
 *  Returns the requested free region
 */
static struct MapPage *GetFreeRegion(u32 aSize)
{
	struct MapPage *currRegion = NULL;
	u32   i = 0;
	u32   RegionSize = 0;
	u32   nextI = 0;

	if (pVirtualMappingTable == NULL)
		return currRegion;
	if (aSize > iFreeSize) {
		/* Find the largest free region
		* (coalesce during the traversal) */
		while (i < TableSize) {
			RegionSize = pVirtualMappingTable[i].RegionSize;
			nextI = i+RegionSize;
			if (pVirtualMappingTable[i].bReserved == false) {
				/* Coalesce, if possible */
				if (nextI < TableSize &&
				pVirtualMappingTable[nextI].bReserved
							== false) {
					pVirtualMappingTable[i].RegionSize +=
					pVirtualMappingTable[nextI].RegionSize;
					continue;
				}
				RegionSize *= PG_SIZE_4K;
				if (RegionSize > iFreeSize) 	{
					iFreeRegion = i;
					iFreeSize = RegionSize;
				}
			}
			i = nextI;
		}
	}
	if (aSize <= iFreeSize) {
		currRegion = pVirtualMappingTable + iFreeRegion;
		iFreeRegion += (aSize / PG_SIZE_4K);
		iFreeSize -= aSize;
	}
	return currRegion;
}

/*
 *  ======== GetMappedRegion ========
 *  Purpose:
 *  Returns the requestedmapped region
 */
static struct MapPage *GetMappedRegion(u32 aAddr)
{
	u32   i = 0;
	struct MapPage *currRegion = NULL;

	if (pVirtualMappingTable == NULL)
		return currRegion;

	i = DMM_ADDR_TO_INDEX(aAddr);
	if (i < TableSize && (pVirtualMappingTable[i].bMapped ||
			pVirtualMappingTable[i].bReserved))
		currRegion = pVirtualMappingTable + i;
	return currRegion;
}

#ifdef DSP_DMM_DEBUG
u32 DMM_MemMapDump(struct DMM_OBJECT *hDmmMgr)
{
	struct MapPage *curNode = NULL;
	u32 i;
	u32 freemem = 0;
	u32 bigsize = 0;

	SYNC_EnterCS(hDmmMgr->hDmmLock);

	if (pVirtualMappingTable != NULL) {
		for (i = 0; i < TableSize; i +=
				pVirtualMappingTable[i].RegionSize) {
			curNode = pVirtualMappingTable + i;
			if (curNode->bReserved == TRUE)	{
				/*printk("RESERVED size = 0x%x, "
					"Map size = 0x%x\n",
					(curNode->RegionSize * PG_SIZE_4K),
					(curNode->bMapped == false) ? 0 :
					(curNode->MappedSize * PG_SIZE_4K));
*/
			} else {
/*				printk("UNRESERVED size = 0x%x\n",
					(curNode->RegionSize * PG_SIZE_4K));
*/
				freemem += (curNode->RegionSize * PG_SIZE_4K);
				if (curNode->RegionSize > bigsize)
					bigsize = curNode->RegionSize;
			}
		}
	}
	printk(KERN_INFO "Total DSP VA FREE memory = %d Mbytes\n",
			freemem/(1024*1024));
	printk(KERN_INFO "Total DSP VA USED memory= %d Mbytes \n",
			(((TableSize * PG_SIZE_4K)-freemem))/(1024*1024));
	printk(KERN_INFO "DSP VA - Biggest FREE block = %d Mbytes \n\n",
			(bigsize*PG_SIZE_4K/(1024*1024)));
	SYNC_LeaveCS(hDmmMgr->hDmmLock);

	return 0;
}
#endif
