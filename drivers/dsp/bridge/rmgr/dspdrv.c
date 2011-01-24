/*
 * dspdrv.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Interface to allocate and free bridge resources.
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

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/mem.h>
#include <dspbridge/reg.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>
#include <dspbridge/_dcd.h>

/*  ----------------------------------- Resource Manager */
#include <dspbridge/mgr.h>

/*  ----------------------------------- This */
#include <dspbridge/dspdrv.h>

/*  ----------------------------------- Globals */
struct GT_Mask curTrace;

/*
 *  ======== DSP_Init ========
 *  	Allocates bridge resources. Loads a base image onto DSP, if specified.
 */
u32 DSP_Init(OUT u32 *initStatus)
{
	char devNode[MAXREGPATHLENGTH] = "TIOMAP1510";
	DSP_STATUS status = DSP_EFAIL;
	struct DRV_OBJECT *drvObject = NULL;
	u32 deviceNode;
	u32 deviceNodeString;

	GT_create(&curTrace, "DD");

	if (!WCD_Init())
		goto func_cont;

	status = DRV_Create(&drvObject);
	if (DSP_FAILED(status)) {
		WCD_Exit();
		goto func_cont;
	}		/* End DRV_Create */

	/* Request Resources */
	status = DRV_RequestResources((u32)&devNode, &deviceNodeString);
	if (DSP_SUCCEEDED(status)) {
		/* Attempt to Start the Device */
		status = DEV_StartDevice((struct CFG_DEVNODE *)
							deviceNodeString);
		if (DSP_FAILED(status))
			(void)DRV_ReleaseResources
				((u32) deviceNodeString, drvObject);
	} else {
		GT_0trace(curTrace, GT_7CLASS,
			 "DSP_Init:DRV_RequestResources Failed \r\n");
		status = DSP_EFAIL;
	}

	/* Unwind whatever was loaded */
	if (DSP_FAILED(status)) {
		/* irrespective of the status of DEV_RemoveDevice we conitinue
		 * unloading. Get the Driver Object iterate through and remove.
		 * Reset the status to E_FAIL to avoid going through
		 * WCD_InitComplete2. */
		for (deviceNode = DRV_GetFirstDevExtension(); deviceNode != 0;
		    deviceNode = DRV_GetNextDevExtension(deviceNode)) {
			(void)DEV_RemoveDevice
				((struct CFG_DEVNODE *)deviceNode);
			(void)DRV_ReleaseResources((u32)deviceNode,
				drvObject);
		}
		/* Remove the Driver Object */
		(void)DRV_Destroy(drvObject);
		drvObject = NULL;
		WCD_Exit();
		GT_0trace(curTrace, GT_7CLASS,
			 "DSP_Init:Logical device Failed to Load\n");
	}	/* Unwinding the loaded drivers */
func_cont:
	/* Attempt to Start the Board */
	if (DSP_SUCCEEDED(status)) {
		/* BRD_AutoStart could fail if the dsp execuetable is not the
		 * correct one. We should not propagate that error
		 * into the device loader. */
		(void)WCD_InitComplete2();
	} else {
		GT_0trace(curTrace, GT_7CLASS, "DSP_Init Failed\n");
	}			/* End WCD_InitComplete2 */
	DBC_Ensure((DSP_SUCCEEDED(status) && drvObject != NULL) ||
		  (DSP_FAILED(status) && drvObject == NULL));
	*initStatus = status;
	/* Return the Driver Object */
	return (u32)drvObject;
}

/*
 *  ======== DSP_Deinit ========
 *  	Frees the resources allocated for bridge.
 */
bool DSP_Deinit(u32 deviceContext)
{
	bool retVal = true;
	u32 deviceNode;
	struct MGR_OBJECT *mgrObject = NULL;

	while ((deviceNode = DRV_GetFirstDevExtension()) != 0) {
		(void)DEV_RemoveDevice((struct CFG_DEVNODE *)deviceNode);

		(void)DRV_ReleaseResources((u32)deviceNode,
			 (struct DRV_OBJECT *)deviceContext);
	}

	(void) DRV_Destroy((struct DRV_OBJECT *) deviceContext);

	/* Get the Manager Object from Registry
	 * MGR Destroy will unload the DCD dll */
	if (DSP_SUCCEEDED(CFG_GetObject((u32 *)&mgrObject, REG_MGR_OBJECT)))
		(void)MGR_Destroy(mgrObject);

	WCD_Exit();

	return retVal;
}
