/*
 * cmm.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * The Communication Memory Management(CMM) module provides shared memory
 * management services for DSP/BIOS Bridge data streaming and messaging.
 * Multiple shared memory segments can be registered with CMM. Memory is
 * coelesced back to the appropriate pool when a buffer is freed.
 *
 * The CMM_Xlator[xxx] functions are used for node messaging and data
 * streaming address translation to perform zero-copy inter-processor
 * data transfer(GPP<->DSP). A "translator" object is created for a node or
 * stream object that contains per thread virtual address information. This
 * translator info is used at runtime to perform SM address translation
 * to/from the DSP address space.
 *
 * Notes:
 *   CMM_XlatorAllocBuf - Used by Node and Stream modules for SM address
 *			  translation.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef CMM_
#define CMM_

#include <dspbridge/devdefs.h>

#include <dspbridge/cmmdefs.h>
#include <dspbridge/host_os.h>

/*
 *  ======== CMM_CallocBuf ========
 *  Purpose:
 *      Allocate memory buffers that can be used for data streaming or
 *      messaging.
 *  Parameters:
 *      hCmmMgr:   Cmm Mgr handle.
 *      uSize:     Number of bytes to allocate.
 *      pAttr:     Attributes of memory to allocate.
 *      ppBufVA:   Address of where to place VA.
 *  Returns:
 *      Pointer to a zero'd block of SM memory;
 *      NULL if memory couldn't be allocated,
 *      or if cBytes == 0,
 *  Requires:
 *      Valid hCmmMgr.
 *      CMM initialized.
 *  Ensures:
 *      The returned pointer, if not NULL, points to a valid memory block of
 *      the size requested.
 *
 */
	extern void *CMM_CallocBuf(struct CMM_OBJECT *hCmmMgr,
				   u32 uSize, struct CMM_ATTRS *pAttrs,
				   OUT void **ppBufVA);

/*
 *  ======== CMM_Create ========
 *  Purpose:
 *      Create a communication memory manager object.
 *  Parameters:
 *      phCmmMgr:   Location to store a communication manager handle on output.
 *      hDevObject: Handle to a device object.
 *      pMgrAttrs:  Comm mem manager attributes.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EMEMORY:    Insufficient memory for requested resources.
 *      DSP_EFAIL:      Failed to initialize critical sect sync object.
 *
 *  Requires:
 *      CMM_Init(void) called.
 *      phCmmMgr != NULL.
 *      pMgrAttrs->ulMinBlockSize >= 4 bytes.
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_Create(OUT struct CMM_OBJECT **phCmmMgr,
				     struct DEV_OBJECT *hDevObject,
				     IN CONST struct CMM_MGRATTRS *pMgrAttrs);

/*
 *  ======== CMM_Destroy ========
 *  Purpose:
 *      Destroy the communication memory manager object.
 *  Parameters:
 *      hCmmMgr:   Cmm Mgr handle.
 *      bForce:    Force deallocation of all cmm memory immediately if set TRUE.
 *                 If FALSE, and outstanding allocations will return DSP_EFAIL
 *                 status.
 *  Returns:
 *      DSP_SOK:        CMM object & resources deleted.
 *      DSP_EFAIL:      Unable to free CMM object due to outstanding allocation.
 *      DSP_EHANDLE:    Unable to free CMM due to bad handle.
 *  Requires:
 *      CMM is initialized.
 *      hCmmMgr != NULL.
 *  Ensures:
 *      Memory resources used by Cmm Mgr are freed.
 */
	extern DSP_STATUS CMM_Destroy(struct CMM_OBJECT *hCmmMgr, bool bForce);

/*
 *  ======== CMM_Exit ========
 *  Purpose:
 *     Discontinue usage of module. Cleanup CMM module if CMM cRef reaches zero.
 *  Parameters:
 *     n/a
 *  Returns:
 *     n/a
 *  Requires:
 *     CMM is initialized.
 *  Ensures:
 */
	extern void CMM_Exit(void);

/*
 *  ======== CMM_FreeBuf ========
 *  Purpose:
 *      Free the given buffer.
 *  Parameters:
 *      hCmmMgr:    Cmm Mgr handle.
 *      pBuf:       Pointer to memory allocated by CMM_CallocBuf().
 *      ulSegId:    SM segment Id used in CMM_Calloc() attrs.
 *                  Set to 0 to use default segment.
 *  Returns:
 *      DSP_SOK
 *      DSP_EFAIL
 *  Requires:
 *      CMM initialized.
 *      pBufPA != NULL
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_FreeBuf(struct CMM_OBJECT *hCmmMgr,
				      void *pBufPA, u32 ulSegId);

/*
 *  ======== CMM_GetHandle ========
 *  Purpose:
 *      Return the handle to the cmm mgr for the given device obj.
 *  Parameters:
 *      hProcessor:   Handle to a Processor.
 *      phCmmMgr:     Location to store the shared memory mgr handle on output.
 *
 *  Returns:
 *      DSP_SOK:        Cmm Mgr opaque handle returned.
 *      DSP_EHANDLE:    Invalid handle.
 *  Requires:
 *      phCmmMgr != NULL
 *      hDevObject != NULL
 *  Ensures:
 */
	extern DSP_STATUS CMM_GetHandle(void *hProcessor,
					OUT struct CMM_OBJECT **phCmmMgr);

/*
 *  ======== CMM_GetInfo ========
 *  Purpose:
 *      Return the current SM and VM utilization information.
 *  Parameters:
 *      hCmmMgr:     Handle to a Cmm Mgr.
 *      pCmmInfo:    Location to store the Cmm information on output.
 *
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Invalid handle.
 *      DSP_EINVALIDARG Invalid input argument.
 *  Requires:
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_GetInfo(struct CMM_OBJECT *hCmmMgr,
				      OUT struct CMM_INFO *pCmmInfo);

/*
 *  ======== CMM_Init ========
 *  Purpose:
 *      Initializes private state of CMM module.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      CMM initialized.
 */
	extern bool CMM_Init(void);

/*
 *  ======== CMM_RegisterGPPSMSeg ========
 *  Purpose:
 *      Register a block of SM with the CMM.
 *  Parameters:
 *      hCmmMgr:         Handle to a Cmm Mgr.
 *      lpGPPBasePA:     GPP Base Physical address.
 *      ulSize:          Size in GPP bytes.
 *      dwDSPAddrOffset  GPP PA to DSP PA Offset.
 *      cFactor:         Add offset if CMM_ADDTODSPPA, sub if CMM_SUBFROMDSPPA.
 *      dwDSPBase:       DSP virtual base byte address.
 *      ulDSPSize:       Size of DSP segment in bytes.
 *      pulSegId:        Address to store segment Id.
 *
 *  Returns:
 *      DSP_SOK:         Success.
 *      DSP_EHANDLE:     Invalid hCmmMgr handle.
 *      DSP_EINVALIDARG: Invalid input argument.
 *      DSP_EFAIL:       Unable to register.
 *      - On success *pulSegId is a valid SM segment ID.
 *  Requires:
 *      ulSize > 0
 *      pulSegId != NULL
 *      dwGPPBasePA != 0
 *      cFactor = CMM_ADDTODSPPA || cFactor = CMM_SUBFROMDSPPA
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_RegisterGPPSMSeg(struct CMM_OBJECT *hCmmMgr,
					       unsigned int dwGPPBasePA,
					       u32 ulSize,
					       u32 dwDSPAddrOffset,
					       s8  cFactor,
					       unsigned int dwDSPBase,
					       u32 ulDSPSize,
					       u32 *pulSegId,
					       u32 dwGPPBaseBA);

/*
 *  ======== CMM_UnRegisterGPPSMSeg ========
 *  Purpose:
 *      Unregister the given memory segment that was previously registered
 *      by CMM_RegisterGPPSMSeg.
 *  Parameters:
 *      hCmmMgr:    Handle to a Cmm Mgr.
 *      ulSegId     Segment identifier returned by CMM_RegisterGPPSMSeg.
 *  Returns:
 *       DSP_SOK:         Success.
 *       DSP_EHANDLE:     Invalid handle.
 *       DSP_EINVALIDARG: Invalid ulSegId.
 *       DSP_EFAIL:       Unable to unregister for unknown reason.
 *  Requires:
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_UnRegisterGPPSMSeg(struct CMM_OBJECT *hCmmMgr,
						 u32 ulSegId);

/*
 *  ======== CMM_XlatorAllocBuf ========
 *  Purpose:
 *      Allocate the specified SM buffer and create a local memory descriptor.
 *      Place on the descriptor on the translator's HaQ (Host Alloc'd Queue).
 *  Parameters:
 *      hXlator:    Handle to a Xlator object.
 *      pVaBuf:     Virtual address ptr(client context)
 *      uPaSize:    Size of SM memory to allocate.
 *  Returns:
 *      Ptr to valid physical address(Pa) of uPaSize bytes, NULL if failed.
 *  Requires:
 *      pVaBuf != 0.
 *      uPaSize != 0.
 *  Ensures:
 *
 */
	extern void *CMM_XlatorAllocBuf(struct CMM_XLATOROBJECT *hXlator,
					void *pVaBuf, u32 uPaSize);

/*
 *  ======== CMM_XlatorCreate ========
 *  Purpose:
 *     Create a translator(xlator) object used for process specific Va<->Pa
 *     address translation. Node messaging and streams use this to perform
 *     inter-processor(GPP<->DSP) zero-copy data transfer.
 *  Parameters:
 *     phXlator:       Address to place handle to a new Xlator handle.
 *     hCmmMgr:        Handle to Cmm Mgr associated with this translator.
 *     pXlatorAttrs:   Translator attributes used for the client NODE or STREAM.
 *  Returns:
 *     DSP_SOK:            Success.
 *     DSP_EINVALIDARG:    Bad input Attrs.
 *     DSP_EMEMORY:   Insufficient memory(local) for requested resources.
 *  Requires:
 *     phXlator != NULL
 *     hCmmMgr != NULL
 *     pXlatorAttrs != NULL
 *  Ensures:
 *
 */
      extern DSP_STATUS CMM_XlatorCreate(OUT struct CMM_XLATOROBJECT **phXlator,
					 struct CMM_OBJECT *hCmmMgr,
					 struct CMM_XLATORATTRS *pXlatorAttrs);

/*
 *  ======== CMM_XlatorDelete ========
 *  Purpose:
 *      Delete translator resources
 *  Parameters:
 *      hXlator:    handle to translator.
 *      bForce:     bForce = TRUE will free XLators SM buffers/dscriptrs.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Bad translator handle.
 *      DSP_EFAIL:      Unable to free translator resources.
 *  Requires:
 *      cRefs > 0
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_XlatorDelete(struct CMM_XLATOROBJECT *hXlator,
					   bool bForce);

/*
 *  ======== CMM_XlatorFreeBuf ========
 *  Purpose:
 *      Free SM buffer and descriptor.
 *      Does not free client process VM.
 *  Parameters:
 *      hXlator:    handle to translator.
 *      pBufVa      Virtual address of PA to free.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Bad translator handle.
 *  Requires:
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_XlatorFreeBuf(struct CMM_XLATOROBJECT *hXlator,
					    void *pBufVa);

/*
 *  ======== CMM_XlatorInfo ========
 *  Purpose:
 *      Set/Get process specific "translator" address info.
 *      This is used to perform fast virtaul address translation
 *      for shared memory buffers between the GPP and DSP.
 *  Parameters:
 *     hXlator:     handle to translator.
 *     pAddr:       Virtual base address of segment.
 *     ulSize:      Size in bytes.
 *     uSegId:      Segment identifier of SM segment(s)
 *     bSetInfo     Set xlator fields if TRUE, else return base addr
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Bad translator handle.
 *  Requires:
 *      (cRefs > 0)
 *      (pAddr != NULL)
 *      (ulSize > 0)
 *  Ensures:
 *
 */
	extern DSP_STATUS CMM_XlatorInfo(struct CMM_XLATOROBJECT *hXlator,
					 IN OUT u8 **pAddr,
					 u32 ulSize, u32 uSegId,
					 bool bSetInfo);

/*
 *  ======== CMM_XlatorTranslate ========
 *  Purpose:
 *      Perform address translation VA<->PA for the specified stream or
 *      message shared memory buffer.
 *  Parameters:
 *     hXlator: handle to translator.
 *     pAddr    address of buffer to translate.
 *     xType    Type of address xlation. CMM_PA2VA or CMM_VA2PA.
 *  Returns:
 *     Valid address on success, else NULL.
 *  Requires:
 *      cRefs > 0
 *      pAddr != NULL
 *      xType >= CMM_VA2PA) && (xType <= CMM_DSPPA2PA)
 *  Ensures:
 *
 */
	extern void *CMM_XlatorTranslate(struct CMM_XLATOROBJECT *hXlator,
					 void *pAddr, enum CMM_XLATETYPE xType);

#endif				/* CMM_ */
