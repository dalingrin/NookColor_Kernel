/*
 * gs.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * General storage memory allocator services.
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
#include <linux/types.h>
/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/mem.h>

/*  ----------------------------------- This */
#include <dspbridge/gs.h>

/*  ----------------------------------- Globals */
static u32 cumsize;

/*
 *  ======== GS_alloc ========
 *  purpose:
 *      Allocates memory of the specified size.
 */
void *GS_alloc(u32 size)
{
	void *p;

	p = MEM_Calloc(size, MEM_PAGED);
	if (p == NULL)
		return NULL;
	cumsize += size;
	return p;
}

/*
 *  ======== GS_exit ========
 *  purpose:
 *      Discontinue the usage of the GS module.
 */
void GS_exit(void)
{
	/* Do nothing */
}

/*
 *  ======== GS_free ========
 *  purpose:
 *      Frees the memory.
 */
void GS_free(void *ptr)
{
	kfree(ptr);
	/* ack! no size info */
	/* cumsize -= size; */
}

/*
 *  ======== GS_frees ========
 *  purpose:
 *      Frees the memory.
 */
void GS_frees(void *ptr, u32 size)
{
	kfree(ptr);
	cumsize -= size;
}

/*
 *  ======== GS_init ========
 *  purpose:
 *      Initializes the GS module.
 */
void GS_init(void)
{
	/* Do nothing */
}
