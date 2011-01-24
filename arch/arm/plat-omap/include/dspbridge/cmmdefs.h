/*
 * cmmdefs.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Global MEM constants and types.
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

#ifndef CMMDEFS_
#define CMMDEFS_

#include <dspbridge/list.h>

/* Cmm attributes used in CMM_Create() */
	struct CMM_MGRATTRS {
		/* Minimum SM allocation; default 32 bytes.  */
		u32 ulMinBlockSize;
	} ;

/* Attributes for CMM_AllocBuf() & CMM_AllocDesc() */
	struct CMM_ATTRS {
		u32 ulSegId;	/*  1,2... are SM segments. 0 is not. */
		u32 ulAlignment;	/*  0,1,2,4....ulMinBlockSize */
	} ;

/*
 *  DSPPa to GPPPa Conversion Factor.
 *
 *  For typical platforms:
 *      converted Address = PaDSP + ( cFactor * addressToConvert).
 */
#define CMM_SUBFROMDSPPA	-1
#define CMM_ADDTODSPPA		1

#define CMM_ALLSEGMENTS         0xFFFFFF	/* All SegIds */
#define CMM_MAXGPPSEGS          1	/* Maximum # of SM segs */

/*
 *  SMSEGs are SM segments the DSP allocates from.
 *
 *  This info is used by the GPP to xlate DSP allocated PAs.
 */

	struct CMM_SEGINFO {
		u32 dwSegBasePa;	/* Start Phys address of SM segment */
		/* Total size in bytes of segment: DSP+GPP */
		u32 ulTotalSegSize;
		u32 dwGPPBasePA;	/* Start Phys addr of Gpp SM seg */
		u32 ulGPPSize;	/* Size of Gpp SM seg in bytes */
		u32 dwDSPBaseVA;	/* DSP virt base byte address */
		u32 ulDSPSize;	/* DSP seg size in bytes */
		/* # of current GPP allocations from this segment */
		u32 ulInUseCnt;
		u32 dwSegBaseVa;	/* Start Virt address of SM seg */

	} ;

/* CMM useful information */
	struct CMM_INFO {
		/* # of SM segments registered with this Cmm. */
		u32 ulNumGPPSMSegs;
		/* Total # of allocations outstanding for CMM */
		u32 ulTotalInUseCnt;
		/* Min SM block size allocation from CMM_Create() */
		u32 ulMinBlockSize;
		/* Info per registered SM segment. */
		struct CMM_SEGINFO segInfo[CMM_MAXGPPSEGS];
	} ;

/* XlatorCreate attributes */
	struct CMM_XLATORATTRS {
		u32 ulSegId;	/* segment Id used for SM allocations */
		u32 dwDSPBufs;	/* # of DSP-side bufs */
		u32 dwDSPBufSize;	/* size of DSP-side bufs in GPP bytes */
		/* Vm base address alloc'd in client process context */
		void *pVmBase;
		/* dwVmSize must be >= (dwMaxNumBufs * dwMaxSize) */
		u32 dwVmSize;
	} ;

/*
 * Cmm translation types. Use to map SM addresses to process context.
 */
	enum CMM_XLATETYPE {
		CMM_VA2PA = 0,	/* Virtual to GPP physical address xlation */
		CMM_PA2VA = 1,	/* GPP Physical to virtual  */
		CMM_VA2DSPPA = 2,	/* Va to DSP Pa  */
		CMM_PA2DSPPA = 3,	/* GPP Pa to DSP Pa */
		CMM_DSPPA2PA = 4,	/* DSP Pa to GPP Pa */
	} ;

	struct CMM_OBJECT;
	struct CMM_XLATOROBJECT;

#endif				/* CMMDEFS_ */
