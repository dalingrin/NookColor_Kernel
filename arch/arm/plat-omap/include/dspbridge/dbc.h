/*
 * dbc.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * "Design by Contract" programming macros.
 *
 * Notes:
 *   Requires that the GT->ERROR function has been defaulted to a valid
 *   error handler for the given execution environment.
 *
 *   Does not require that GT_init() be called.
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

#ifndef DBC_
#define DBC_

#ifndef GT_TRACE
#define GT_TRACE 0	    /* 0 = "trace compiled out"; 1 = "trace active" */
#endif

/* Assertion Macros: */
#if GT_TRACE

#include <dspbridge/gt.h>

#define DBC_Assert(exp) 						       \
do {									       \
	if (!(exp))							       \
		pr_err("%s, line %d: Assertion (" #exp ") failed.\n", \
		__FILE__, __LINE__);					       \
} while (0)
#define DBC_Require DBC_Assert	/* Function Precondition.  */
#define DBC_Ensure  DBC_Assert	/* Function Postcondition. */

#else

#define DBC_Assert(exp) {}
#define DBC_Require(exp) {}
#define DBC_Ensure(exp) {}

#endif				/* DEBUG */

#endif				/* DBC_ */
