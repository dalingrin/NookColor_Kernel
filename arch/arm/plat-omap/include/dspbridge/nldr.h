/*
 * nldr.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * DSP/BIOS Bridge dynamic loader interface.
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

#include <dspbridge/dbdefs.h>
#include <dspbridge/dbdcddef.h>
#include <dspbridge/dev.h>
#include <dspbridge/rmm.h>
#include <dspbridge/nldrdefs.h>

#ifndef NLDR_
#define NLDR_

	extern DSP_STATUS NLDR_Allocate(struct NLDR_OBJECT *hNldr,
					void *pPrivRef,
					IN CONST struct DCD_NODEPROPS
					*pNodeProps,
					OUT struct NLDR_NODEOBJECT **phNldrNode,
					IN bool *pfPhaseSplit);

	extern DSP_STATUS NLDR_Create(OUT struct NLDR_OBJECT **phNldr,
				      struct DEV_OBJECT *hDevObject,
				      IN CONST struct NLDR_ATTRS *pAttrs);

	extern void NLDR_Delete(struct NLDR_OBJECT *hNldr);
	extern void NLDR_Exit(void);

	extern DSP_STATUS NLDR_GetFxnAddr(struct NLDR_NODEOBJECT *hNldrNode,
					  char *pstrFxn, u32 *pulAddr);

	extern DSP_STATUS NLDR_GetRmmManager(struct NLDR_OBJECT *hNldrObject,
					     OUT struct RMM_TargetObj
					     **phRmmMgr);

	extern bool NLDR_Init(void);
	extern DSP_STATUS NLDR_Load(struct NLDR_NODEOBJECT *hNldrNode,
				    enum NLDR_PHASE phase);
	extern DSP_STATUS NLDR_Unload(struct NLDR_NODEOBJECT *hNldrNode,
				    enum NLDR_PHASE phase);
	DSP_STATUS nldr_find_addr(
			struct NLDR_NODEOBJECT *nldr_node, u32 sym_addr,
			u32 offset_range, void *offset_output, char *sym_name);

#endif				/* NLDR_ */
