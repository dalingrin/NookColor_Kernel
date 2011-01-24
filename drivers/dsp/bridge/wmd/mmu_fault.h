/*
 * mmu_fault.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Defines DSP MMU fault handling functions.
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

#ifndef MMU_FAULT_
#define MMU_FAULT_

/*
 *  ======== MMU_FaultDpc ========
 *      Deferred procedure call to handle DSP MMU fault.
 */
	void MMU_FaultDpc(IN unsigned long pRefData);

/*
 *  ======== MMU_FaultIsr ========
 *      ISR to be triggered by a DSP MMU fault interrupt.
 */
irqreturn_t  MMU_FaultIsr(int irq, IN void *pRefData);

#endif				/* MMU_FAULT_ */

