/*
 * SDRC register values for the Micron ELPIDA_ECK100ACBCN
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Copyright (C) 2008 Nokia Corporation
 *
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_ELPIDA_EDD20323ABH
#define ARCH_ARM_MACH_OMAP2_SDRAM_ELPIDA_EDD20323ABH

#include <mach/sdrc.h>

/* ELPIDA EDD20323ABH */
/* XXX Using ARE = 0x1 (no autorefresh burst) -- can this be changed? */
/* ELPIDA mDDR Pascal project (166MHz optimized) 6.02ns
* Pascal : EDD20323ABH
*     ACTIMA
*        -TDAL = Twr/Tck + Trp/tck = 15/6 + 15/6 = 2.5 + 2.5 = 5
*        -TDPL (Twr) = 15/6       = 2.5 -> 3
*        -TRRD = 12/6     = 2
*        -TRCD = 18/6     = 3
*        -TRP = 18/6      = 3
*        -TRAS = 42/6     = 7
*        -TRC = 60/6      = 10
*        -TRFC = 78/6     = 13
*     ACTIMB
*        -TWTR = 1
*        -TCKE = 2
*        -TXP  = 1
*        -XSR  = 20
*/
#define TDAL_165  5
#define TDPL_165  3
#define TRRD_165  2
#define TRCD_165  3
#define TRP_165   3
#define TRAS_165  7
#define TRC_165   10
#define TRFC_165  13
#define V_ACTIMA_165 ((TRFC_165 << 27) | (TRC_165 << 22) | (TRAS_165 << 18) |\
          (TRP_165 << 15) | (TRCD_165 << 12) | (TRRD_165 << 9) | \
          (TDPL_165 << 6) | (TDAL_165))

#define TWTR_165  1
#define TCKE_165  2
#define TXP_165   1
#define XSR_165   20
#define V_ACTIMB_165 (((TCKE_165 << 12) | (XSR_165 << 0)) | \
          (TXP_165 << 8) | (TWTR_165 << 16))


/* ELPIDA mDDR Pascal project (83MHz optimized) 12.04ns
* Pascal : EDD20323ABH
*     ACTIMA
*        -TDAL = Twr/Tck + Trp/tck = 15/12 + 18/12 = 2.75 -> 3
*        -TDPL (Twr) = 15/12       = 1.1 -> 2
*        -TRRD = 12/12    = 1
*        -TRCD = 18/12    = 1.5 -> 2
*        -TRP = 18/12     = 1.5 -> 2
*        -TRAS = 42/12    = 3.5 -> 4
*        -TRC = 60/12     = 5
*        -TRFC = 78/12    = 6.5 -> 7
*     ACTIMB
*        -TWTR = 1
*        -TCKE = 2
*        -TXP  = 1
*        -XSR  = 10
*/
#define TDAL_83  3
#define TDPL_83  2
#define TRRD_83  1
#define TRCD_83  2
#define TRP_83   2
#define TRAS_83  4
#define TRC_83   5
#define TRFC_83  7
#define V_ACTIMA_83 ((TRFC_83 << 27) | (TRC_83 << 22) | (TRAS_83 << 18) |\
	(TRP_83 << 15) | (TRCD_83 << 12) | (TRRD_83 << 9) | \
	(TDPL_83 << 6) | (TDAL_83))
#define TWTR_83  1
#define TCKE_83  2
#define TXP_83   1
#define XSR_83   10
#define V_ACTIMB_83 (((TCKE_83 << 12) | (XSR_83 << 0)) | \
	(TXP_83 << 8) | (TWTR_83 << 16))


static struct omap_sdrc_params edd20323abh_sdrc_params[] = {
	[0] = {
		.rate	     = 166000000,
		.actim_ctrla = V_ACTIMA_165,
		.actim_ctrlb = V_ACTIMB_165,
		.rfr_ctrl    = 0x0004e201,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 165941176,
		.actim_ctrla = V_ACTIMA_165,
		.actim_ctrlb = V_ACTIMB_165,
		.rfr_ctrl    = 0x0004e201,
		.mr	     = 0x00000032,
	},
#if 1
	[2] = {
		.rate	     = 83000000,
		.actim_ctrla = V_ACTIMA_83,
		.actim_ctrlb = V_ACTIMB_83,
		.rfr_ctrl    = 0x00025501,
		.mr	     = 0x00000032,
	},
	[3] = {
		.rate	     = 82970588,
		.actim_ctrla = V_ACTIMA_83,
		.actim_ctrlb = V_ACTIMB_83,
		.rfr_ctrl    = 0x00025501,
		.mr	     = 0x00000032,
	},
	[4] = {
		.rate	     = 0
	},
#else
	[2] = {
		.rate	     = 0
	},
#endif
};

#endif
