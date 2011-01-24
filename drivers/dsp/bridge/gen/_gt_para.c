/*
 * _gt_para.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
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

/*
 * Configuration parameters for GT.  This file is separated from
 * gt.c so that GT_assert() can reference the error function without
 * forcing the linker to include all the code for GT_set(), GT_init(),
 * etc. into a fully bound image.  Thus, GT_assert() can be retained in
 * a program for which GT_?trace() has been compiled out.
 */

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/gt.h>

/*  ----------------------------------- Function Prototypes */
static void error(char *msg, ...);
static s32 GT_nop(void);

/*  ----------------------------------- Defines, Data Structures, Typedefs */

struct GT_Config _GT_params = {
	(Fxn) printk,		/* printf */
	(Fxn) NULL,		/* procid */
	(Fxn) GT_nop,		/* taskid */
	(Fxn) error,		/* error */
};

/*  ----------------------------------- Globals */
struct GT_Config *GT = &_GT_params;

/*
 *  ======== GT_nop ========
 */
static s32 GT_nop(void)
{
	return 0;
}

/*
 * ======== error ========
 *  purpose:
 *      Prints error onto the standard output.
 */
static void error(char *fmt, ...)
{
	s32 arg1, arg2, arg3, arg4, arg5, arg6;

	va_list va;

	va_start(va, fmt);

	arg1 = va_arg(va, s32);
	arg2 = va_arg(va, s32);
	arg3 = va_arg(va, s32);
	arg4 = va_arg(va, s32);
	arg5 = va_arg(va, s32);
	arg6 = va_arg(va, s32);

	va_end(va);

	pr_err("ERROR: ");
	printk(fmt, arg1, arg2, arg3, arg4, arg5, arg6);

#if defined(CONFIG_BRIDGE_DEBUG) || defined(DDSP_DEBUG_PRODUCT)
	if (in_interrupt()) {
		printk(KERN_INFO "Not stopping after error since ISR/DPC "
			"are disabled\n");
	} else {
		set_current_state(TASK_INTERRUPTIBLE);
		flush_signals(current);
		schedule();
		flush_signals(current);
		printk(KERN_INFO "Signaled in error function\n");
	}
#endif
}
