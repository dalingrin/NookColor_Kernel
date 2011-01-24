/*
 * services.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Provide SERVICES loading.
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

#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/dbg.h>
#include <dspbridge/mem.h>
#include <dspbridge/ntfy.h>
#include <dspbridge/reg.h>
#include <dspbridge/sync.h>
#include <dspbridge/clk.h>

/*  ----------------------------------- This */
#include <dspbridge/services.h>

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask SERVICES_debugMask = { NULL, NULL };  /* GT trace var. */
#endif

/*
 *  ======== SERVICES_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 */
void SERVICES_Exit(void)
{
	/* Uninitialize all SERVICES modules here */
	NTFY_Exit();
	SYNC_Exit();
	CLK_Exit();
	REG_Exit();
	DBG_Exit();
	CFG_Exit();
	MEM_Exit();

	GT_exit();
}

/*
 *  ======== SERVICES_Init ========
 *  Purpose:
 *      Initializes SERVICES modules.
 */
bool SERVICES_Init(void)
{
	bool fInit = true;
	bool fCFG, fDBG, fMEM;
	bool fREG, fSYNC, fCLK, fNTFY;

	GT_init();
	GT_create(&SERVICES_debugMask, "OS");	/* OS for OSal */

	/* Perform required initialization of SERVICES modules. */
	fMEM = MEM_Init();
	fSYNC = SYNC_Init();
	fREG = REG_Init();
	fCFG = CFG_Init();
	fDBG = DBG_Init();
	fCLK  = CLK_Init();
	fNTFY = NTFY_Init();

	fInit = fCFG && fDBG && fMEM && fREG && fSYNC && fCLK;

	if (!fInit) {
		if (fNTFY)
			NTFY_Exit();

		if (fSYNC)
			SYNC_Exit();

		if (fCLK)
			CLK_Exit();

		if (fREG)
			REG_Exit();

		if (fDBG)
			DBG_Exit();

		if (fCFG)
			CFG_Exit();

		if (fMEM)
			MEM_Exit();

	}

	return fInit;
}

