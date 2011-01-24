/*
 * SDRC register values for the Samsung K4X4G303PB
 *
 * Copyright (C) 2010 MM Solutions AD
 *
 * Dimitar Dimitrov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_SAMSUNG_K4X4G303PB
#define ARCH_ARM_MACH_OMAP2_SDRAM_SAMSUNG_K4X4G303PB

#include <mach/sdrc.h>

/*--------------------------------------------------------------------------*/

/* Samsung K4X4G303PB mDDR (166MHz optimized) 6.02ns
 *
 *     ACTIMA
 *        -TDAL = Twr/Tck + Trp/tck = 12/6 + 15/6 = 2 + 2.5 = 4.5 -> 5
 *        -TDPL (Twr) = 12/6 = 2 -> 2
 *        -TRRD = 10/6     = 1.66 -> 2
 *        -TRCD = 15/6     = 2.5 -> 3
 *        -TRP = 15/6      = 2.5 -> 3
 *        -TRAS = 40/6     = 6.7 -> 7
 *        -TRC = 55/6      = 9.16 -> 10
 *        -TRFC = 120/6     = 20
 *     ACTIMB
 *        -TWTR = 2 (TCDLR in the datasheet)
 *        -TCKE = 2
 *        -TXP  = 1
 *        -XSR  = 120/6 = 20
 */
#define TDAL_165  5
#define TDPL_165  2
#define TRRD_165  2
#define TRCD_165  3
#define TRP_165   3
#define TRAS_165  7
#define TRC_165   10
#define TRFC_165  20
#define V_ACTIMA_165 ((TRFC_165 << 27) | (TRC_165 << 22) | (TRAS_165 << 18) |\
          (TRP_165 << 15) | (TRCD_165 << 12) | (TRRD_165 << 9) | \
          (TDPL_165 << 6) | (TDAL_165))

#define TWTR_165  2
#define TCKE_165  2
#define TXP_165   1
#define XSR_165   20
#define V_ACTIMB_165 (((TCKE_165 << 12) | (XSR_165 << 0)) | \
          (TXP_165 << 8) | (TWTR_165 << 16))

#define SAMSUNG_RFR_CTRL_165MHz   0x0004e201 /* 7.8us/6ns - 50=0x4e2 */

/*--------------------------------------------------------------------------*/

/* Samsung K4X4G303PB mDDR (83MHz optimized) 12.05ns
 *
 *     ACTIMA
 *        -TDAL = Twr/Tck + Trp/tck = 12/12 + 15/12 = 1 + 1.25 = 2.25 -> 3
 *        -TDPL (Twr) = 12/12 = 1 -> 1
 *        -TRRD = 10/12     = 0.83 -> 1
 *        -TRCD = 15/12     = 1.25 -> 2
 *        -TRP = 15/12      = 1.25 -> 2
 *        -TRAS = 40/12     = 3.33 -> 4
 *        -TRC = 55/12      = 4.58 -> 5
 *        -TRFC = 120/12     = 10
 *     ACTIMB
 *        -TWTR = 2 (TCDLR in the datasheet)
 *        -TCKE = 2
 *        -TXP  = 1
 *        -XSR  = 120/12 = 10
 */
#define TDAL_83  3
#define TDPL_83  1
#define TRRD_83  1
#define TRCD_83  2
#define TRP_83   2
#define TRAS_83  4
#define TRC_83   5
#define TRFC_83  10
#define V_ACTIMA_83 ((TRFC_83 << 27) | (TRC_83 << 22) | (TRAS_83 << 18) |\
          (TRP_83 << 15) | (TRCD_83 << 12) | (TRRD_83 << 9) | \
          (TDPL_83 << 6) | (TDAL_83))

#define TWTR_83  2
#define TCKE_83  2
#define TXP_83   1
#define XSR_83   10
#define V_ACTIMB_83 (((TCKE_83 << 12) | (XSR_83 << 0)) | \
          (TXP_83 << 8) | (TWTR_83 << 16))

#define SAMSUNG_RFR_CTRL_83MHz  0x00025501 /* 7.8us/12ns - 50=597.33 -> 0x255 */

/*--------------------------------------------------------------------------*/

/* XXX Using ARE = 0x1 (no autorefresh burst) -- can this be changed? */
static struct omap_sdrc_params samsung_k4x4g303pb_sdrc_params[] = {
	[0] = {
		.rate	     = 166000000,
		.actim_ctrla = V_ACTIMA_165,
		.actim_ctrlb = V_ACTIMB_165,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_165MHz,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 165941176,
		.actim_ctrla = V_ACTIMA_165,
		.actim_ctrlb = V_ACTIMB_165,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_165MHz,
		.mr	     = 0x00000032,
	},
	[2] = {
		.rate	     = 83000000,
		.actim_ctrla = V_ACTIMA_83,
		.actim_ctrlb = V_ACTIMB_83,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_83MHz,
		.mr	     = 0x00000032,	/* Only CAS_lat=3 supported! */
	},
	[3] = {
		.rate	     = 82970588,
		.actim_ctrla = V_ACTIMA_83,
		.actim_ctrlb = V_ACTIMB_83,
		.rfr_ctrl    = SAMSUNG_RFR_CTRL_83MHz,
		.mr	     = 0x00000032,	/* Only CAS_lat=3 supported! */
	},
	[4] = {
		.rate	     = 0
	},
};

#endif
