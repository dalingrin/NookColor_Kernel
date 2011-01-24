/*
 * dbdcddef.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * DCD (DSP/BIOS Bridge Configuration Database) constants and types.
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

#ifndef DBDCDDEF_
#define DBDCDDEF_

#include <dspbridge/dbdefs.h>
#include <dspbridge/mgrpriv.h>		/* for MGR_PROCESSOREXTINFO */

/*
 *  The following defines are critical elements for the DCD module:
 *
 * - DCD_REGKEY enables DCD functions to locate registered DCD objects.
 * - DCD_REGISTER_SECTION identifies the COFF section where the UUID of
 *   registered DCD objects are stored.
 */
#define DCD_REGKEY              "Software\\TexasInstruments\\DspBridge\\DCD"
#define DCD_REGISTER_SECTION    ".dcd_register"

/* DCD Manager Object */
	struct DCD_MANAGER;

/* DCD Node Properties */
	struct DCD_NODEPROPS {
		struct DSP_NDBPROPS ndbProps;
		u32 uMsgSegid;
		u32 uMsgNotifyType;
		char *pstrCreatePhaseFxn;
		char *pstrDeletePhaseFxn;
		char *pstrExecutePhaseFxn;
		char *pstrIAlgName;

		/* Dynamic load properties */
		u16 usLoadType;	/* Static, dynamic, overlay */
		u32 ulDataMemSegMask;	/* Data memory requirements */
		u32 ulCodeMemSegMask;	/* Code memory requirements */
	} ;

/* DCD Generic Object Type */
	struct DCD_GENERICOBJ {
		union dcdObjUnion {
			struct DCD_NODEPROPS nodeObj;	/* node object. */
			/* processor object. */
			struct DSP_PROCESSORINFO procObj;
			/* extended proc object (private) */
			struct MGR_PROCESSOREXTINFO extProcObj;
		} objData;
	} ;

/* DCD Internal Callback Type */
       typedef DSP_STATUS(*DCD_REGISTERFXN) (IN struct DSP_UUID *pUuid,
						IN enum DSP_DCDOBJTYPE objType,
						IN void *handle);

#endif				/* DBDCDDEF_ */

