#ifndef __ARCH_ARM_MACH_OMAP3_SMARTREFLEX_H
#define __ARCH_ARM_MACH_OMAP3_SMARTREFLEX_H
/*
 * linux/arch/arm/mach-omap2/smartreflex.h
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define PHY_TO_OFF_PM_MASTER(p)		(p - 0x36)
#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)
#define PHY_TO_OFF_PM_INT(p)		(p - 0x2e)

/* SMART REFLEX REG ADDRESS OFFSET */
#define SRCONFIG	0x00
#define SRSTATUS	0x04
#define SENVAL		0x08
#define SENMIN		0x0C
#define SENMAX		0x10
#define SENAVG		0x14
#define AVGWEIGHT	0x18
#define NVALUERECIPROCAL	0x1C
#define SENERROR	0x20
#define ERRCONFIG	0x24
#define SENERROR_36XX   0x34
#define ERRCONFIG_36XX  0x38

/* SR Modules */
#define SR1		1
#define SR2		2

#define SR_FAIL		1
#define SR_PASS		0

#define SR_TRUE		1
#define SR_FALSE	0

#define GAIN_MAXLIMIT	16
#define R_MAXLIMIT	256

#define SR1_CLK_ENABLE	BIT(6)
#define SR2_CLK_ENABLE	BIT(7)

/* PRM_VP1_CONFIG */
#define PRM_VP1_CONFIG_ERROROFFSET	(0x00 << 24)
#define PRM_VP1_CONFIG_ERRORGAIN_OPPLOW		(0x0C << 16)
#define PRM_VP1_CONFIG_ERRORGAIN_OPPHIGH	(0x27 << 16)

#define PRM_VP1_CONFIG_INITVOLTAGE	(0x30 << 8) /* 1.2 volt */

/* PRM_VP1_VSTEPMIN */
#define PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN	(0x3C << 8)
#define PRM_VP1_VSTEPMIN_VSTEPMIN		BIT(0)

/* PRM_VP1_VSTEPMAX */
#define PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX	(0x3C << 8)
#define PRM_VP1_VSTEPMAX_VSTEPMAX		(0x04 << 0)

/* PRM_VP1_VLIMITTO */
#define PRM_VP1_VLIMITTO_VDDMAX		(0x44 << 24)
#define PRM_VP1_VLIMITTO_VDDMIN		(0x14 << 16)
#define PRM_VP1_VLIMITTO_TIMEOUT	(0xFFFF << 0)

/* PRM_VP2_CONFIG */
#define PRM_VP2_CONFIG_ERROROFFSET	(0x00 << 24)
#define PRM_VP2_CONFIG_ERRORGAIN_OPPLOW         (0x0C << 16)
#define PRM_VP2_CONFIG_ERRORGAIN_OPPHIGH        (0x18 << 16)

#define PRM_VP2_CONFIG_INITVOLTAGE	(0x30 << 8) /* 1.2 volt */

/* PRM_VP2_VSTEPMIN */
#define PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN	(0x3C << 8)
#define PRM_VP2_VSTEPMIN_VSTEPMIN		BIT(0)

/* PRM_VP2_VSTEPMAX */
#define PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX	(0x3C << 8)
#define PRM_VP2_VSTEPMAX_VSTEPMAX		(0x04 << 0)

/* PRM_VP2_VLIMITTO */
#define PRM_VP2_VLIMITTO_VDDMAX		(0x42 << 24)
#define PRM_VP2_VLIMITTO_VDDMIN		(0x18 << 16)
#define PRM_VP2_VLIMITTO_TIMEOUT	(0xFFFF << 0)

/* SRCONFIG */
#define SR1_SRCONFIG_ACCUMDATA		(0x1F4 << 22)
#define SR2_SRCONFIG_ACCUMDATA		(0x1F4 << 22)

#define SRCLKLENGTH_12MHZ_SYSCLK	0x3C
#define SRCLKLENGTH_13MHZ_SYSCLK	0x41
#define SRCLKLENGTH_19MHZ_SYSCLK	0x60
#define SRCLKLENGTH_26MHZ_SYSCLK	0x82
#define SRCLKLENGTH_38MHZ_SYSCLK	0xC0

#define SRCONFIG_SRCLKLENGTH_SHIFT	12
#define SRCONFIG_SENNENABLE_SHIFT	5
#define SRCONFIG_SENPENABLE_SHIFT	3
#define SRCONFIG_SENNENABLE_SHIFT_36XX	1
#define SRCONFIG_SENPENABLE_SHIFT_36XX	0

#define SRCONFIG_SRENABLE		BIT(11)
#define SRCONFIG_SENENABLE		BIT(10)
#define SRCONFIG_ERRGEN_EN		BIT(9)
#define SRCONFIG_MINMAXAVG_EN		BIT(8)

#define SRCONFIG_DELAYCTRL		BIT(2)
#define SRCONFIG_CLKCTRL		(0x00 << 0)

/* AVGWEIGHT */
#define SR1_AVGWEIGHT_SENPAVGWEIGHT	(0x03 << 2)
#define SR1_AVGWEIGHT_SENNAVGWEIGHT	(0x03 << 0)

#define SR2_AVGWEIGHT_SENPAVGWEIGHT	BIT(2)
#define SR2_AVGWEIGHT_SENNAVGWEIGHT	BIT(0)

/* NVALUERECIPROCAL */
#define NVALUERECIPROCAL_SENPGAIN_SHIFT	20
#define NVALUERECIPROCAL_SENNGAIN_SHIFT	16
#define NVALUERECIPROCAL_RNSENP_SHIFT	8
#define NVALUERECIPROCAL_RNSENN_SHIFT	0

/* ERRCONFIG */
#define SR_CLKACTIVITY_MASK		(0x03 << 20)
#define SR_ERRWEIGHT_MASK		(0x07 << 16)
#define SR_ERRMAXLIMIT_MASK		(0xFF << 8)
#define SR_ERRMINLIMIT_MASK		(0xFF << 0)

#define SR_CLKACTIVITY_IOFF_FOFF	(0x00 << 20)
#define SR_CLKACTIVITY_IOFF_FON		(0x02 << 20)

/* IDLEMODE SETTINGS for OMAP3630 */
#define SR_IDLEMODE_MASK		(0x3 << 24)
#define SR_FORCE_IDLE			(0x0 << 24)
#define SR_NO_IDLE			(0x1 << 24)
#define SR_SMART_IDLE			(0x2 << 24)
#define SR_SMART_IDLE_WKUP		(0x3 << 24)

#define ERRCONFIG_VPBOUNDINTEN		BIT(31)
#define ERRCONFIG_VPBOUNDINTST		BIT(30)
#define ERRCONFIG_VPBOUNDINTEN_36XX	BIT(22)
#define ERRCONFIG_VPBOUNDINTST_36XX	BIT(23)
#define ERRCONFIG_MCUBOUNDINTEN		BIT(25)
#define ERRCONFIG_MCUBOUNDINTST		BIT(24)
#define ERRCONFIG_MCUDISACKINTEN	BIT(23)
#define ERRCONFIG_MCUDISACKINTST	BIT(22)

#define SR1_ERRWEIGHT			(0x04 << 16)
#define SR1_ERRMAXLIMIT			(0x02 << 8)
#define SR1_ERRMINLIMIT_OPPLOW		(0xF4 << 0)
#define SR1_ERRMINLIMIT_OPPHIGH		(0xF9 << 0)
#define SR1_IRQ				18

#define SR2_ERRWEIGHT			(0x04 << 16)
#define SR2_ERRMAXLIMIT			(0x02 << 8)
#define SR2_ERRMINLIMIT_OPPLOW		(0xF4 << 0)
#define SR2_ERRMINLIMIT_OPPHIGH		(0xF9 << 0)
#define SR2_IRQ				19

/* T2 SMART REFLEX */
#define R_SRI2C_SLAVE_ADDR		0x12
#define R_VDD1_SR_CONTROL		0x00
#define R_VDD2_SR_CONTROL		0x01
#define T2_SMPS_UPDATE_DELAY		360	/* In uSec */

/* Vmode control */
#define R_DCDC_GLOBAL_CFG	PHY_TO_OFF_PM_RECIEVER(0x61)

#define R_VDD1_VSEL		PHY_TO_OFF_PM_RECIEVER(0xb9)
#define R_VDD1_VMODE_CFG	PHY_TO_OFF_PM_RECIEVER(0xba)
#define R_VDD1_VFLOOR		PHY_TO_OFF_PM_RECIEVER(0xbb)
#define R_VDD1_VROOF		PHY_TO_OFF_PM_RECIEVER(0xbc)
#define R_VDD1_STEP		PHY_TO_OFF_PM_RECIEVER(0xbd)

#define R_VDD2_VSEL		PHY_TO_OFF_PM_RECIEVER(0xc7)
#define R_VDD2_VMODE_CFG	PHY_TO_OFF_PM_RECIEVER(0xc8)
#define R_VDD2_VFLOOR		PHY_TO_OFF_PM_RECIEVER(0xc9)
#define R_VDD2_VROOF		PHY_TO_OFF_PM_RECIEVER(0xca)
#define R_VDD2_STEP		PHY_TO_OFF_PM_RECIEVER(0xcb)

/* R_DCDC_GLOBAL_CFG register, SMARTREFLEX_ENABLE values */
#define DCDC_GLOBAL_CFG_ENABLE_SRFLX	0x08

#define PRCM_MAX_SYSC_REGS 30

/*
 * XXX: These should be removed/moved from here once we have a working DVFS
 * implementation in place
 */
#define AT_3430		1	/*3430 ES 1.0 */
#define AT_3430_ES2	2	/*3430 ES 2.0 */

#define ID_OPP			0xE2 	/*OPP*/

/* DEVICE ID/DPLL ID/CLOCK ID: bits 28-31 for OMAP type */
#define OMAP_TYPE_SHIFT		28
#define OMAP_TYPE_MASK		0xF
/* OPP ID: bits: 0-4 for OPP number */
#define OPP_NO_POS		0
#define OPP_NO_MASK		0x1F
/* OPP ID: bits: 5-6 for VDD */
#define VDD_NO_POS		5
#define VDD_NO_MASK		0x3
/* Other IDs: bits 20-27 for ID type */
/* These IDs have bits 25,26,27 as 1 */
#define OTHER_ID_TYPE_SHIFT		20
#define OTHER_ID_TYPE_MASK		0xFF

#define OTHER_ID_TYPE(X) ((X & OTHER_ID_TYPE_MASK) << OTHER_ID_TYPE_SHIFT)
#define ID_OPP_NO(X)	 ((X & OPP_NO_MASK) << OPP_NO_POS)
#define ID_VDD(X)	 ((X & VDD_NO_MASK) << VDD_NO_POS)
#define OMAP(X)		 ((X >> OMAP_TYPE_SHIFT) & OMAP_TYPE_MASK)
#define get_opp_no(X)	 ((X >> OPP_NO_POS) & OPP_NO_MASK)
#define get_vdd(X)	 ((X >> VDD_NO_POS) & VDD_NO_MASK)

/* VDD1 OPPs */
#define PRCM_VDD1_OPP1		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD1) | ID_OPP_NO(0x1))
#define PRCM_VDD1_OPP2		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD1) | ID_OPP_NO(0x2))
#define PRCM_VDD1_OPP3		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD1) | ID_OPP_NO(0x3))
#define PRCM_VDD1_OPP4		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD1) | ID_OPP_NO(0x4))
#define PRCM_VDD1_OPP5		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD1) | ID_OPP_NO(0x5))
#define PRCM_NO_VDD1_OPPS	5


/* VDD2 OPPs */
#define PRCM_VDD2_OPP1		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD2) | ID_OPP_NO(0x1))
#define PRCM_VDD2_OPP2		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD2) | ID_OPP_NO(0x2))
#define PRCM_VDD2_OPP3		(OMAP(AT_3430_ES2) | OTHER_ID_TYPE(ID_OPP) | \
					ID_VDD(PRCM_VDD2) | ID_OPP_NO(0x3))
#define PRCM_NO_VDD2_OPPS	3
/* XXX: end remove/move */

/* XXX: find more appropriate place for these once DVFS is in place */
extern u32 current_vdd1_opp;
extern u32 current_vdd2_opp;

#ifdef CONFIG_OMAP_SMARTREFLEX_TESTING
#define SR_TESTING_NVALUES 	1
#else
#define SR_TESTING_NVALUES 	0
#endif

/*
 * Smartreflex module enable/disable interface.
 * NOTE: if smartreflex is not enabled from sysfs, these functions will not
 * do anything.
 */
#ifdef CONFIG_OMAP_SMARTREFLEX
void enable_smartreflex(int srid);
void disable_smartreflex(int srid);
int sr_voltagescale_vcbypass(u32 t_opp, u32 c_opp, u8 t_vsel, u8 c_vsel);
void sr_start_vddautocomap(int srid, u32 target_opp_no);
int sr_stop_vddautocomap(int srid);
typedef int (*omap3_voltagescale_vcbypass_t)(u32 t_opp, u32 c_opp,
						u8 t_vsel, u8 c_vsel);
void omap3_voltagescale_vcbypass_setup(omap3_voltagescale_vcbypass_t fun);
int sr_recalibrate(int srid, u32 target_opp, u32 current_opp);

#else
static inline void enable_smartreflex(int srid) {}
static inline void disable_smartreflex(int srid) {}
int sr_recalibrate(int srid, u32 target_opp, u32 current_opp)
{
	return -EINVAL;
}
#define omap3_voltagescale_vcbypass_setup(fun) do {} while (0);
#endif

#endif
