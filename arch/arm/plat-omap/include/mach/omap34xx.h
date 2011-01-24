/*
 * arch/arm/plat-omap/include/mach/omap34xx.h
 *
 * This file contains the processor specific definitions of the TI OMAP34XX.
 *
 * Copyright (C) 2007 Texas Instruments.
 * Copyright (C) 2007 Nokia Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_OMAP34XX_H
#define __ASM_ARCH_OMAP34XX_H
/*
 * Please place only base defines here and put the rest in device
 * specific headers.
 */
#define L4_34XX_BASE		0x48000000
#define L4_WK_34XX_BASE		0x48300000
#define L4_WK_OMAP_BASE		L4_WK_34XX_BASE
#define L4_PER_34XX_BASE	0x49000000
#define L4_PER_OMAP_BASE	L4_PER_34XX_BASE
#define L4_EMU_34XX_BASE	0x54000000
#define L4_EMU_BASE		L4_EMU_34XX_BASE
#define L3_34XX_BASE		0x68000000
#define L3_OMAP_BASE		L3_34XX_BASE

#define OMAP3430_32KSYNCT_BASE	0x48320000
#define OMAP3430_CM_BASE	0x48004800
#define OMAP3430_PRM_BASE	0x48306800
#define OMAP343X_SMS_BASE	0x6C000000
#define OMAP343X_SDRC_BASE	0x6D000000
#define OMAP34XX_GPMC_BASE	0x6E000000
#define OMAP343X_SCM_BASE	0x48002000
#define OMAP343X_CTRL_BASE	OMAP343X_SCM_BASE

#define OMAP34XX_IC_BASE	0x48200000

#define OMAP3430_ISP_BASE		(L4_34XX_BASE + 0xBC000)
#define OMAP3430_ISP_CBUFF_BASE		(OMAP3430_ISP_BASE + 0x0100)
#define OMAP3430_ISP_CCP2_BASE		(OMAP3430_ISP_BASE + 0x0400)
#define OMAP3430_ISP_CCDC_BASE		(OMAP3430_ISP_BASE + 0x0600)
#define OMAP3430_ISP_HIST_BASE		(OMAP3430_ISP_BASE + 0x0A00)
#define OMAP3430_ISP_H3A_BASE		(OMAP3430_ISP_BASE + 0x0C00)
#define OMAP3430_ISP_PREV_BASE		(OMAP3430_ISP_BASE + 0x0E00)
#define OMAP3430_ISP_RESZ_BASE		(OMAP3430_ISP_BASE + 0x1000)
#define OMAP3430_ISP_SBL_BASE		(OMAP3430_ISP_BASE + 0x1200)
#define OMAP3430_ISP_MMU_BASE		(OMAP3430_ISP_BASE + 0x1400)
#define OMAP3430_ISP_CSI2A_BASE		(OMAP3430_ISP_BASE + 0x1800)
#define OMAP3430_ISP_CSI2PHY_BASE	(OMAP3430_ISP_BASE + 0x1970)
#define OMAP3430_ISP_CSI2PHY2_BASE      (OMAP3430_ISP_BASE + 0x1D70)

#define OMAP3430_ISP_END		(OMAP3430_ISP_BASE         + 0x06F)
#define OMAP3430_ISP_CBUFF_END		(OMAP3430_ISP_CBUFF_BASE   + 0x077)
#define OMAP3430_ISP_CCP2_END		(OMAP3430_ISP_CCP2_BASE    + 0x1EF)
#define OMAP3430_ISP_CCDC_END		(OMAP3430_ISP_CCDC_BASE    + 0x0A7)
#define OMAP3430_ISP_HIST_END		(OMAP3430_ISP_HIST_BASE    + 0x047)
#define OMAP3430_ISP_H3A_END		(OMAP3430_ISP_H3A_BASE     + 0x05F)
#define OMAP3430_ISP_PREV_END		(OMAP3430_ISP_PREV_BASE    + 0x09F)
#define OMAP3430_ISP_RESZ_END		(OMAP3430_ISP_RESZ_BASE    + 0x0AB)
#define OMAP3430_ISP_SBL_END		(OMAP3430_ISP_SBL_BASE     + 0x0FB)
#define OMAP3430_ISP_MMU_END		(OMAP3430_ISP_MMU_BASE     + 0x06F)
#define OMAP3430_ISP_CSI2A_END		(OMAP3430_ISP_CSI2A_BASE   + 0x16F)
#define OMAP3430_ISP_CSI2PHY_END	(OMAP3430_ISP_CSI2PHY_BASE + 0x007)
#define OMAP3430_ISP_CSI2PHY2_END       (OMAP3430_ISP_CSI2PHY2_BASE + 0x007)

#define OMAP34XX_IVA_INTC_BASE	0x40000000
#define OMAP34XX_HSUSB_OTG_BASE	(L4_34XX_BASE + 0xAB000)
#define OMAP34XX_HSUSB_HOST_BASE	(L4_34XX_BASE + 0x64000)
#define OMAP34XX_USBTLL_BASE	(L4_34XX_BASE + 0x62000)
#define OMAP34XX_SR1_BASE	0x480C9000
#define OMAP34XX_SR2_BASE	0x480CB000

#define OMAP34XX_MAILBOX_BASE		(L4_34XX_BASE + 0x94000)


#if defined(CONFIG_ARCH_OMAP3430)

#define OMAP2_32KSYNCT_BASE		OMAP3430_32KSYNCT_BASE
#define OMAP2_CM_BASE			OMAP3430_CM_BASE
#define OMAP2_PRM_BASE			OMAP3430_PRM_BASE
#define OMAP2_VA_IC_BASE		IO_ADDRESS(OMAP34XX_IC_BASE)

#endif

#define OMAP34XX_DSP_BASE	0x58000000
#define OMAP34XX_DSP_MEM_BASE	(OMAP34XX_DSP_BASE + 0x0)
#define OMAP34XX_DSP_IPI_BASE	(OMAP34XX_DSP_BASE + 0x1000000)
#define OMAP34XX_DSP_MMU_BASE	(OMAP34XX_DSP_BASE + 0x2000000)

/* VDD OPP identifiers */
#define VDD1_OPP	0x1
#define VDD2_OPP	0x2

/* VDD1 OPPS */
#define VDD1_OPP1	0x1
#define VDD1_OPP2	0x2
#define VDD1_OPP3	0x3
#define VDD1_OPP4	0x4
#define VDD1_OPP5	0x5
#define VDD1_OPP6	0x6

/* VDD2 OPPS */
#define VDD2_OPP1	0x1
#define VDD2_OPP2	0x2
#define VDD2_OPP3	0x3
#define VDD2_OPP4	0x4

#define MIN_VDD1_OPP	(omap_pm_get_min_vdd1_opp())
#define MAX_VDD1_OPP	(omap_pm_get_max_vdd1_opp())
#define MIN_VDD2_OPP	(omap_pm_get_min_vdd2_opp())
#define MAX_VDD2_OPP	(omap_pm_get_max_vdd2_opp())
/*
 * scale VDD2 OPP when VDD1 is above OPP3
 * REVISIT: Need to check with h/w; for what VDD1 freq
 * the L3 bandwith needs to be bumped to 2. Power
 * measurement results shows scaling VDD2 to OPP2 when
 * VDD1 is above 3 is optimal.
 */
#define VDD1_THRESHOLD	VDD1_OPP3

/* MPU speeds */
#define S1200M  1200000000
#define S1000M  1000000000
#define S800M   800000000
#define S720M   720000000
#define S600M   600000000
#define S550M   550000000
#define S520M   520000000
#define S500M   500000000
#define S300M   300000000
#define S250M   250000000
#define S150M   150000000
#define S125M   125000000

/* DSP speeds */
#define S875M   875000000
#define S760M   760000000
#define S660M   660000000
#define S520M   520000000
#define S430M   430000000
#define S400M   400000000
#define S360M   360000000
#define S260M   260000000
#define S180M   180000000
#define S130M   130000000
#define S90M    90000000
#define S65M	65000000

/* L3 speeds */
#define S50M    50000000
#define S83M    83000000
#define S100M   100000000
#define S166M   166000000
#define S200M   200000000

#endif /* __ASM_ARCH_OMAP34XX_H */

