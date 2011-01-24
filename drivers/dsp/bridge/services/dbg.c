/*
 * dbg.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Provide debugging services for DSP/BIOS Bridge Mini Driver.
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

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- This */
#include <dspbridge/dbg.h>

/*  ----------------------------------- Globals */
#if GT_TRACE
static struct GT_Mask DBG_debugMask = { NULL, NULL };	/* GT trace var. */
#endif

#if (defined(CONFIG_BRIDGE_DEBUG) || defined(DDSP_DEBUG_PRODUCT)) && GT_TRACE

/*
 *  ======== DBG_Init ========
 *  Purpose:
 *      Ensures trace capability is set up for link drivers.
 */
bool DBG_Init(void)
{
	GT_create(&DBG_debugMask, "WD");     /* for WmD (link driver) debug */

	return true;
}

/*
 *  ======== DBG_Trace ========
 *  Purpose:
 *      Output a trace message to the debugger, if the given trace level
 *      is unmasked.
 */
DSP_STATUS DBG_Trace(u8 bLevel, char *pstrFormat, ...)
{
	s32 arg1, arg2, arg3, arg4, arg5, arg6;
	va_list va;

	va_start(va, pstrFormat);

	arg1 = va_arg(va, s32);
	arg2 = va_arg(va, s32);
	arg3 = va_arg(va, s32);
	arg4 = va_arg(va, s32);
	arg5 = va_arg(va, s32);
	arg6 = va_arg(va, s32);

	va_end(va);

	if (bLevel & *(DBG_debugMask).flags)
		printk(pstrFormat, arg1, arg2, arg3, arg4, arg5, arg6);

	return DSP_SOK;
}

/*
 *  ======== DBG_Exit ========
 *  Purpose:
 *      Discontinue usage of the DBG module.
 */
void DBG_Exit(void)
{
	/* Nothing to do */
}

#endif	/* (defined(CONFIG_BRIDGE_DEBUG) || defined(DDSP_DEBUG_PRODUCT)) && GT_TRACE */
