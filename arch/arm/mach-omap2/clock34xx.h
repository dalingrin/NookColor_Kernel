/*
 * OMAP3 clock framework
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2008 Nokia Corporation
 *
 * Written by Paul Walmsley
 * With many device clock fixes by Kevin Hilman and Jouni Högander
 * DPLL bypass clock support added by Roman Tereshonkov
 *
 */

/*
 * Virtual clocks are introduced as convenient tools.
 * They are sources for other clocks and not supposed
 * to be requested from drivers directly.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCK34XX_H
#define __ARCH_ARM_MACH_OMAP2_CLOCK34XX_H

#include <mach/control.h>
#include <mach/omap-pm.h>

#include "clock.h"
#include "cm.h"
#include "cm-regbits-34xx.h"
#include "prm.h"
#include "prm-regbits-34xx.h"

static void omap3_dpll_recalc(struct clk *clk, unsigned long parent_rate,
			      u8 rate_storage);
static void omap3_clkoutx2_recalc(struct clk *clk, unsigned long parent_rate,
			      u8 rate_storage);
static void omap3_dpll_allow_idle(struct clk *clk);
static void omap3_dpll_deny_idle(struct clk *clk);
static u32 omap3_dpll_autoidle_read(struct clk *clk);
static int omap3_noncore_dpll_enable(struct clk *clk);
static void omap3_noncore_dpll_disable(struct clk *clk);
static int omap3_noncore_dpll_set_rate(struct clk *clk, unsigned long rate);
static int omap3_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate);

/* Maximum DPLL multiplier, divider values for OMAP3 */
#define OMAP3_MAX_DPLL_MULT		2048
#define OMAP3_MAX_DPLL_DIV		128

/*
 * DPLL1 supplies clock to the MPU.
 * DPLL2 supplies clock to the IVA2.
 * DPLL3 supplies CORE domain clocks.
 * DPLL4 supplies peripheral clocks.
 * DPLL5 supplies other peripheral clocks (USBHOST, USIM).
 */

/* Forward declarations for DPLL bypass clocks */
static struct clk dpll1_fck;
static struct clk dpll2_fck;

/* CM_CLKEN_PLL*.EN* bit values - not all are available for every DPLL */
#define DPLL_LOW_POWER_STOP		0x1
#define DPLL_LOW_POWER_BYPASS		0x5
#define DPLL_LOCKED			0x7

/* PRM CLOCKS */

/* According to timer32k.c, this is a 32768Hz clock, not a 32000Hz clock. */
static struct clk omap_32k_fck = {
	.name		= "omap_32k_fck",
	.rate		= 32768,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

static struct clk secure_32k_fck = {
	.name		= "secure_32k_fck",
	.rate		= 32768,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

/* Virtual source clocks for osc_sys_ck */
static struct clk virt_12m_ck = {
	.name		= "virt_12m_ck",
	.rate		= 12000000,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

static struct clk virt_13m_ck = {
	.name		= "virt_13m_ck",
	.rate		= 13000000,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

static struct clk virt_16_8m_ck = {
	.name		= "virt_16_8m_ck",
	.rate		= 16800000,
	.flags		= CLOCK_IN_OMAP3430ES2 | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

static struct clk virt_19_2m_ck = {
	.name		= "virt_19_2m_ck",
	.rate		= 19200000,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

static struct clk virt_26m_ck = {
	.name		= "virt_26m_ck",
	.rate		= 26000000,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

static struct clk virt_38_4m_ck = {
	.name		= "virt_38_4m_ck",
	.rate		= 38400000,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

static const struct clksel_rate osc_sys_12m_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate osc_sys_13m_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate osc_sys_16_8m_rates[] = {
	{ .div = 1, .val = 5, .flags = RATE_IN_3430ES2 | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate osc_sys_19_2m_rates[] = {
	{ .div = 1, .val = 2, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate osc_sys_26m_rates[] = {
	{ .div = 1, .val = 3, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate osc_sys_38_4m_rates[] = {
	{ .div = 1, .val = 4, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel osc_sys_clksel[] = {
	{ .parent = &virt_12m_ck,   .rates = osc_sys_12m_rates },
	{ .parent = &virt_13m_ck,   .rates = osc_sys_13m_rates },
	{ .parent = &virt_16_8m_ck, .rates = osc_sys_16_8m_rates },
	{ .parent = &virt_19_2m_ck, .rates = osc_sys_19_2m_rates },
	{ .parent = &virt_26m_ck,   .rates = osc_sys_26m_rates },
	{ .parent = &virt_38_4m_ck, .rates = osc_sys_38_4m_rates },
	{ .parent = NULL },
};

/* Oscillator clock */
/* 12, 13, 16.8, 19.2, 26, or 38.4 MHz */
static struct clk osc_sys_ck = {
	.name		= "osc_sys_ck",
	.prcm_mod	= OMAP3430_CCR_MOD | CLK_REG_IN_PRM,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3_PRM_CLKSEL_OFFSET,
	.clksel_mask	= OMAP3430_SYS_CLKIN_SEL_MASK,
	.clksel		= osc_sys_clksel,
	/* REVISIT: deal with autoextclkmode? */
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate div2_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 2, .flags = RATE_IN_343X },
	{ .div = 0 }
};

static const struct clksel sys_clksel[] = {
	{ .parent = &osc_sys_ck, .rates = div2_rates },
	{ .parent = NULL }
};

/* Latency: this clock is only enabled after PRM_CLKSETUP.SETUP_TIME */
/* Feeds DPLLs - divided first by PRM_CLKSRC_CTRL.SYSCLKDIV? */
static struct clk sys_ck = {
	.name		= "sys_ck",
	.parent		= &osc_sys_ck,
	.prcm_mod	= OMAP3430_GR_MOD | CLK_REG_IN_PRM,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3_PRM_CLKSRC_CTRL_OFFSET,
	.clksel_mask	= OMAP_SYSCLKDIV_MASK,
	.clksel		= sys_clksel,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk sys_altclk = {
	.name		= "sys_altclk",
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "cm_clkdm" },
};

/*
 * Optional external clock input for some McBSPs
 * Apparently this is not really in prm_clkdm, but rather is fed into
 * both CORE and PER separately.
 */
static struct clk mcbsp_clks = {
	.name		= "mcbsp_clks",
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
};

/* PRM EXTERNAL CLOCK OUTPUT */

static struct clk sys_clkout1 = {
	.name		= "sys_clkout1",
	.parent		= &osc_sys_ck,
	.prcm_mod	= OMAP3430_CCR_MOD | CLK_REG_IN_PRM,
	.enable_reg	= OMAP3_PRM_CLKOUT_CTRL_OFFSET,
	.enable_bit	= OMAP3430_CLKOUT_EN_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

/* DPLLS */

/* CM CLOCKS */

static const struct clksel_rate div16_dpll_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 2, .flags = RATE_IN_343X },
	{ .div = 3, .val = 3, .flags = RATE_IN_343X },
	{ .div = 4, .val = 4, .flags = RATE_IN_343X },
	{ .div = 5, .val = 5, .flags = RATE_IN_343X },
	{ .div = 6, .val = 6, .flags = RATE_IN_343X },
	{ .div = 7, .val = 7, .flags = RATE_IN_343X },
	{ .div = 8, .val = 8, .flags = RATE_IN_343X },
	{ .div = 9, .val = 9, .flags = RATE_IN_343X },
	{ .div = 10, .val = 10, .flags = RATE_IN_343X },
	{ .div = 11, .val = 11, .flags = RATE_IN_343X },
	{ .div = 12, .val = 12, .flags = RATE_IN_343X },
	{ .div = 13, .val = 13, .flags = RATE_IN_343X },
	{ .div = 14, .val = 14, .flags = RATE_IN_343X },
	{ .div = 15, .val = 15, .flags = RATE_IN_343X },
	{ .div = 16, .val = 16, .flags = RATE_IN_343X },
	{ .div = 17, .val = 17, .flags = RATE_IN_343X },
	{ .div = 18, .val = 18, .flags = RATE_IN_343X },
	{ .div = 19, .val = 19, .flags = RATE_IN_343X },
	{ .div = 20, .val = 20, .flags = RATE_IN_343X },
	{ .div = 21, .val = 21, .flags = RATE_IN_343X },
	{ .div = 22, .val = 22, .flags = RATE_IN_343X },
	{ .div = 23, .val = 23, .flags = RATE_IN_343X },
	{ .div = 24, .val = 24, .flags = RATE_IN_343X },
	{ .div = 25, .val = 25, .flags = RATE_IN_343X },
	{ .div = 26, .val = 26, .flags = RATE_IN_343X },
	{ .div = 27, .val = 27, .flags = RATE_IN_343X },
	{ .div = 28, .val = 28, .flags = RATE_IN_343X },
	{ .div = 29, .val = 29, .flags = RATE_IN_343X },
	{ .div = 30, .val = 30, .flags = RATE_IN_343X },
	{ .div = 31, .val = 31, .flags = RATE_IN_343X },
	{ .div = 32, .val = 32, .flags = RATE_IN_343X },
	{ .div = 0 }
};

/* DPLL1 */
/* MPU clock source */
/* Type: DPLL */
static struct dpll_data dpll1_dd = {
	.mult_div1_reg	= OMAP3430_CM_CLKSEL1_PLL,
	.mult_mask	= OMAP3430_MPU_DPLL_MULT_MASK,
	.div1_mask	= OMAP3430_MPU_DPLL_DIV_MASK,
	.freqsel_mask	= OMAP3430_MPU_DPLL_FREQSEL_MASK,
	.control_reg	= OMAP3430_CM_CLKEN_PLL,
	.enable_mask	= OMAP3430_EN_MPU_DPLL_MASK,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.auto_recal_bit	= OMAP3430_EN_MPU_DPLL_DRIFTGUARD_SHIFT,
	.recal_en_bit	= OMAP3430_MPU_DPLL_RECAL_EN_SHIFT,
	.recal_st_bit	= OMAP3430_MPU_DPLL_ST_SHIFT,
	.autoidle_reg	= OMAP3430_CM_AUTOIDLE_PLL,
	.autoidle_mask	= OMAP3430_AUTO_MPU_DPLL_MASK,
	.idlest_reg	= OMAP3430_CM_IDLEST_PLL,
	.idlest_mask	= OMAP3430_ST_MPU_CLK_MASK,
	.bypass_clk	= &dpll1_fck,
	.max_multiplier = OMAP3_MAX_DPLL_MULT,
	.min_divider	= 1,
	.max_divider	= OMAP3_MAX_DPLL_DIV,
	.rate_tolerance = DEFAULT_DPLL_RATE_TOLERANCE
};

static struct clk dpll1_ck = {
	.name		= "dpll1_ck",
	.parent		= &sys_ck,
	.prcm_mod	= MPU_MOD,
	.dpll_data	= &dpll1_dd,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED | RECALC_ON_ENABLE,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.clkdm		= { .name = "dpll1_clkdm" },
	.recalc		= &omap3_dpll_recalc,
};

/*
 * This virtual clock provides the CLKOUTX2 output from the DPLL if the
 * DPLL isn't bypassed.
 */
static struct clk dpll1_x2_ck = {
	.name		= "dpll1_x2_ck",
	.parent		= &dpll1_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll1_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

/* On DPLL1, unlike other DPLLs, the divider is downstream from CLKOUTX2 */
static const struct clksel div16_dpll1_x2m2_clksel[] = {
	{ .parent = &dpll1_x2_ck, .rates = div16_dpll_rates },
	{ .parent = NULL }
};

/*
 * Does not exist in the TRM - needed to separate the M2 divider from
 * bypass selection in mpu_ck
 */
static struct clk dpll1_x2m2_ck = {
	.name		= "dpll1_x2m2_ck",
	.parent		= &dpll1_x2_ck,
	.prcm_mod	= MPU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430_CM_CLKSEL2_PLL,
	.clksel_mask	= OMAP3430_MPU_DPLL_CLKOUT_DIV_MASK,
	.clksel		= div16_dpll1_x2m2_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll1_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* DPLL2 */
/* IVA2 clock source */
/* Type: DPLL */

static struct dpll_data dpll2_dd = {
	.mult_div1_reg	= OMAP3430_CM_CLKSEL1_PLL,
	.mult_mask	= OMAP3430_IVA2_DPLL_MULT_MASK,
	.div1_mask	= OMAP3430_IVA2_DPLL_DIV_MASK,
	.freqsel_mask	= OMAP3430_IVA2_DPLL_FREQSEL_MASK,
	.control_reg	= OMAP3430_CM_CLKEN_PLL,
	.enable_mask	= OMAP3430_EN_IVA2_DPLL_MASK,
	.modes		= (1 << DPLL_LOW_POWER_STOP) | (1 << DPLL_LOCKED) |
				(1 << DPLL_LOW_POWER_BYPASS),
	.auto_recal_bit	= OMAP3430_EN_IVA2_DPLL_DRIFTGUARD_SHIFT,
	.recal_en_bit	= OMAP3430_PRM_IRQENABLE_MPU_IVA2_DPLL_RECAL_EN_SHIFT,
	.recal_st_bit	= OMAP3430_PRM_IRQSTATUS_MPU_IVA2_DPLL_ST_SHIFT,
	.autoidle_reg	= OMAP3430_CM_AUTOIDLE_PLL,
	.autoidle_mask	= OMAP3430_AUTO_IVA2_DPLL_MASK,
	.idlest_reg	= OMAP3430_CM_IDLEST_PLL,
	.idlest_mask	= OMAP3430_ST_IVA2_CLK_MASK,
	.bypass_clk	= &dpll2_fck,
	.max_multiplier = OMAP3_MAX_DPLL_MULT,
	.min_divider	= 1,
	.max_divider	= OMAP3_MAX_DPLL_DIV,
	.rate_tolerance = DEFAULT_DPLL_RATE_TOLERANCE
};

static struct clk dpll2_ck = {
	.name		= "dpll2_ck",
	.parent		= &sys_ck,
	.prcm_mod	= OMAP3430_IVA2_MOD,
	.dpll_data	= &dpll2_dd,
	.flags		= CLOCK_IN_OMAP343X | RECALC_ON_ENABLE,
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.clkdm		= { .name = "dpll2_clkdm" },
	.recalc		= &omap3_dpll_recalc,
};

static const struct clksel div16_dpll2_m2x2_clksel[] = {
	{ .parent = &dpll2_ck, .rates = div16_dpll_rates },
	{ .parent = NULL }
};

/*
 * The TRM is conflicted on whether IVA2 clock comes from DPLL2 CLKOUT
 * or CLKOUTX2. CLKOUT seems most plausible.
 */
static struct clk dpll2_m2_ck = {
	.name		= "dpll2_m2_ck",
	.parent		= &dpll2_ck,
	.prcm_mod	= OMAP3430_IVA2_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430_CM_CLKSEL2_PLL,
	.clksel_mask	= OMAP3430_IVA2_DPLL_CLKOUT_DIV_MASK,
	.clksel		= div16_dpll2_m2x2_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll2_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/*
 * DPLL3
 * Source clock for all interfaces and for some device fclks
 * REVISIT: Also supports fast relock bypass - not included below
 */
static struct dpll_data dpll3_dd = {
	.mult_div1_reg	= CM_CLKSEL1,
	.mult_mask	= OMAP3430_CORE_DPLL_MULT_MASK,
	.div1_mask	= OMAP3430_CORE_DPLL_DIV_MASK,
	.freqsel_mask	= OMAP3430_CORE_DPLL_FREQSEL_MASK,
	.control_reg	= CM_CLKEN,
	.enable_mask	= OMAP3430_EN_CORE_DPLL_MASK,
	.auto_recal_bit	= OMAP3430_EN_CORE_DPLL_DRIFTGUARD_SHIFT,
	.recal_en_bit	= OMAP3430_CORE_DPLL_RECAL_EN_SHIFT,
	.recal_st_bit	= OMAP3430_CORE_DPLL_ST_SHIFT,
	.autoidle_reg	= CM_AUTOIDLE,
	.autoidle_mask	= OMAP3430_AUTO_CORE_DPLL_MASK,
	.idlest_reg	= CM_IDLEST,
	.idlest_mask	= OMAP3430_ST_CORE_CLK_MASK,
	.bypass_clk	= &sys_ck,
	.max_multiplier = OMAP3_MAX_DPLL_MULT,
	.min_divider	= 1,
	.max_divider	= OMAP3_MAX_DPLL_DIV,
	.rate_tolerance = DEFAULT_DPLL_RATE_TOLERANCE
};

static struct clk dpll3_ck = {
	.name		= "dpll3_ck",
	.parent		= &sys_ck,
	.prcm_mod	= PLL_MOD,
	.dpll_data	= &dpll3_dd,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED | RECALC_ON_ENABLE,
	.round_rate	= &omap2_dpll_round_rate,
	.clkdm		= { .name = "dpll3_clkdm" },
	.recalc		= &omap3_dpll_recalc,
};

static const struct clksel_rate div31_dpll3_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 2, .flags = RATE_IN_343X },
	{ .div = 3, .val = 3, .flags = RATE_IN_3430ES2 },
	{ .div = 4, .val = 4, .flags = RATE_IN_3430ES2 },
	{ .div = 5, .val = 5, .flags = RATE_IN_3430ES2 },
	{ .div = 6, .val = 6, .flags = RATE_IN_3430ES2 },
	{ .div = 7, .val = 7, .flags = RATE_IN_3430ES2 },
	{ .div = 8, .val = 8, .flags = RATE_IN_3430ES2 },
	{ .div = 9, .val = 9, .flags = RATE_IN_3430ES2 },
	{ .div = 10, .val = 10, .flags = RATE_IN_3430ES2 },
	{ .div = 11, .val = 11, .flags = RATE_IN_3430ES2 },
	{ .div = 12, .val = 12, .flags = RATE_IN_3430ES2 },
	{ .div = 13, .val = 13, .flags = RATE_IN_3430ES2 },
	{ .div = 14, .val = 14, .flags = RATE_IN_3430ES2 },
	{ .div = 15, .val = 15, .flags = RATE_IN_3430ES2 },
	{ .div = 16, .val = 16, .flags = RATE_IN_3430ES2 },
	{ .div = 17, .val = 17, .flags = RATE_IN_3430ES2 },
	{ .div = 18, .val = 18, .flags = RATE_IN_3430ES2 },
	{ .div = 19, .val = 19, .flags = RATE_IN_3430ES2 },
	{ .div = 20, .val = 20, .flags = RATE_IN_3430ES2 },
	{ .div = 21, .val = 21, .flags = RATE_IN_3430ES2 },
	{ .div = 22, .val = 22, .flags = RATE_IN_3430ES2 },
	{ .div = 23, .val = 23, .flags = RATE_IN_3430ES2 },
	{ .div = 24, .val = 24, .flags = RATE_IN_3430ES2 },
	{ .div = 25, .val = 25, .flags = RATE_IN_3430ES2 },
	{ .div = 26, .val = 26, .flags = RATE_IN_3430ES2 },
	{ .div = 27, .val = 27, .flags = RATE_IN_3430ES2 },
	{ .div = 28, .val = 28, .flags = RATE_IN_3430ES2 },
	{ .div = 29, .val = 29, .flags = RATE_IN_3430ES2 },
	{ .div = 30, .val = 30, .flags = RATE_IN_3430ES2 },
	{ .div = 31, .val = 31, .flags = RATE_IN_3430ES2 },
	{ .div = 0 },
};

static const struct clksel div31_dpll3m2_clksel[] = {
	{ .parent = &dpll3_ck, .rates = div31_dpll3_rates },
	{ .parent = NULL }
};

/* DPLL3 output M2 - primary control point for CORE speed */
static struct clk dpll3_m2_ck = {
	.name		= "dpll3_m2_ck",
	.parent		= &dpll3_ck,
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_CORE_DPLL_CLKOUT_DIV_MASK,
	.clksel		= div31_dpll3m2_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll3_clkdm" },
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap3_core_dpll_m2_set_rate,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk core_ck = {
	.name		= "core_ck",
	.parent		= &dpll3_m2_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk dpll3_m2x2_ck = {
	.name		= "dpll3_m2x2_ck",
	.parent		= &dpll3_m2_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll3_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

/* The PWRDN bit is apparently only available on 3430ES2 and above */
static const struct clksel div16_dpll3_clksel[] = {
	{ .parent = &dpll3_ck, .rates = div16_dpll_rates },
	{ .parent = NULL }
};

/* This virtual clock is the source for dpll3_m3x2_ck */
static struct clk dpll3_m3_ck = {
	.name		= "dpll3_m3_ck",
	.parent		= &dpll3_ck,
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_DIV_DPLL3_MASK,
	.clksel_shift	= OMAP3430_DIV_DPLL3_SHIFT,
	.clksel		= div16_dpll3_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll3_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* The PWRDN bit is apparently only available on 3430ES2 and above */
static struct clk dpll3_m3x2_ck = {
	.name		= "dpll3_m3x2_ck",
	.parent		= &dpll3_m3_ck,
	.prcm_mod	= PLL_MOD,
	.enable_reg	= CM_CLKEN,
	.enable_bit	= OMAP3430_PWRDN_EMU_CORE_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | INVERT_ENABLE,
	.clkdm		= { .name = "dpll3_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

static struct clk emu_core_alwon_ck = {
	.name		= "emu_core_alwon_ck",
	.parent		= &dpll3_m3x2_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll3_clkdm" },
	.recalc		= &followparent_recalc,
};

/* DPLL4 */
/* Supplies 96MHz, 54Mhz TV DAC, DSS fclk, CAM sensor clock, emul trace clk */
/* Type: DPLL */
static struct dpll_data dpll4_dd = {
	.mult_div1_reg	= CM_CLKSEL2,
	.mult_mask	= OMAP3430_PERIPH_DPLL_MULT_MASK,
	.div1_mask	= OMAP3430_PERIPH_DPLL_DIV_MASK,
	.freqsel_mask	= OMAP3430_PERIPH_DPLL_FREQSEL_MASK,
	.control_reg	= CM_CLKEN,
	.enable_mask	= OMAP3430_EN_PERIPH_DPLL_MASK,
	.modes		= (1 << DPLL_LOW_POWER_STOP) | (1 << DPLL_LOCKED),
	.auto_recal_bit	= OMAP3430_EN_PERIPH_DPLL_DRIFTGUARD_SHIFT,
	.recal_en_bit	= OMAP3430_PERIPH_DPLL_RECAL_EN_SHIFT,
	.recal_st_bit	= OMAP3430_PERIPH_DPLL_ST_SHIFT,
	.autoidle_reg	= CM_AUTOIDLE,
	.autoidle_mask	= OMAP3430_AUTO_PERIPH_DPLL_MASK,
	.idlest_reg	= CM_IDLEST,
	.idlest_mask	= OMAP3430_ST_PERIPH_CLK_MASK,
	.bypass_clk	= &sys_ck,
	.max_multiplier = OMAP3_MAX_DPLL_MULT,
	.min_divider	= 1,
	.max_divider	= OMAP3_MAX_DPLL_DIV,
	.rate_tolerance = DEFAULT_DPLL_RATE_TOLERANCE
};

static struct clk dpll4_ck = {
	.name		= "dpll4_ck",
	.parent		= &sys_ck,
	.prcm_mod	= PLL_MOD,
	.dpll_data	= &dpll4_dd,
	.flags		= CLOCK_IN_OMAP343X | RECALC_ON_ENABLE,
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap3_dpll_recalc,
};

static const struct clksel div16_dpll4_clksel[] = {
	{ .parent = &dpll4_ck, .rates = div16_dpll_rates },
	{ .parent = NULL }
};

/* This virtual clock is the source for dpll4_m2x2_ck */
static struct clk dpll4_m2_ck = {
	.name		= "dpll4_m2_ck",
	.parent		= &dpll4_ck,
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430_CM_CLKSEL3,
	.clksel_mask	= OMAP3430_DIV_96M_MASK,
	.clksel_shift	= OMAP3430_DIV_96M_SHIFT,
	.clksel		= div16_dpll4_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* The PWRDN bit is apparently only available on 3430ES2 and above */
static struct clk dpll4_m2x2_ck = {
	.name		= "dpll4_m2x2_ck",
	.parent		= &dpll4_m2_ck,
	.prcm_mod	= PLL_MOD,
	.enable_reg	= CM_CLKEN,
	.enable_bit	= OMAP3430_PWRDN_96M_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | INVERT_ENABLE,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

/*
 * DPLL4 generates DPLL4_M2X2_CLK which is then routed into the PRM as
 * PRM_96M_ALWON_(F)CLK.  Two clocks then emerge from the PRM:
 * 96M_ALWON_FCLK (called "omap_96m_alwon_fck" below) and
 * CM_96K_(F)CLK.
 */

static struct clk omap_192m_alwon_ck = {
	.name		= "omap_192m_alwon_ck",
	.parent		= &dpll4_m2x2_ck,
	.flags		= CLOCK_IN_OMAP363X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
   };

static const struct clksel_rate omap_96m_alwon_fck_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_363X },
	{ .div = 2, .val = 2, .flags = RATE_IN_363X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel omap_96m_alwon_fck_clksel[] = {
	{ .parent = &omap_192m_alwon_ck, .rates = omap_96m_alwon_fck_rates },
	{ .parent = NULL }
};


static struct clk omap_96m_alwon_fck = {
	.name		= "omap_96m_alwon_fck",
	.parent		= &dpll4_m2x2_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk cm_96m_fck = {
	.name		= "cm_96m_fck",
	.parent		= &omap_96m_alwon_fck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &followparent_recalc,
};

static const struct clksel_rate omap_96m_dpll_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate omap_96m_sys_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel omap_96m_fck_clksel[] = {
	{ .parent = &cm_96m_fck, .rates = omap_96m_dpll_rates },
	{ .parent = &sys_ck,	 .rates = omap_96m_sys_rates },
	{ .parent = NULL }
};

static struct clk omap_96m_fck = {
	.name		= "omap_96m_fck",
	.parent		= &sys_ck,
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_SOURCE_96M_MASK,
	.clksel		= omap_96m_fck_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* This virtual clock is the source for dpll4_m3x2_ck */
static struct clk dpll4_m3_ck = {
	.name		= "dpll4_m3_ck",
	.parent		= &dpll4_ck,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_TV_MASK,
	.clksel_mask2 = OMAP3630_CLKSEL_TV_MASK,
	.clksel_shift = OMAP3430_CLKSEL_TV_SHIFT,
	.clksel		= div16_dpll4_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK
					| CLOCK_IN_OMAP363X,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* The PWRDN bit is apparently only available on 3430ES2 and above */
static struct clk dpll4_m3x2_ck = {
	.name		= "dpll4_m3x2_ck",
	.parent		= &dpll4_m3_ck,
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_CLKEN,
	.enable_bit	= OMAP3430_PWRDN_TV_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | INVERT_ENABLE,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

static const struct clksel_rate omap_54m_d4m3x2_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate omap_54m_alt_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel omap_54m_clksel[] = {
	{ .parent = &dpll4_m3x2_ck, .rates = omap_54m_d4m3x2_rates },
	{ .parent = &sys_altclk,    .rates = omap_54m_alt_rates },
	{ .parent = NULL }
};

static struct clk omap_54m_fck = {
	.name		= "omap_54m_fck",
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_SOURCE_54M_MASK,
	.clksel		= omap_54m_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate omap_48m_cm96m_rates[] = {
	{ .div = 2, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate omap_48m_alt_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel omap_48m_clksel[] = {
	{ .parent = &cm_96m_fck, .rates = omap_48m_cm96m_rates },
	{ .parent = &sys_altclk, .rates = omap_48m_alt_rates },
	{ .parent = NULL }
};

static struct clk omap_48m_fck = {
	.name		= "omap_48m_fck",
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_SOURCE_48M_MASK,
	.clksel		= omap_48m_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk omap_12m_fck = {
	.name		= "omap_12m_fck",
	.parent		= &omap_48m_fck,
	.fixed_div	= 4,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_fixed_divisor_recalc,
};

/* This virstual clock is the source for dpll4_m4x2_ck */
static struct clk dpll4_m4_ck = {
	.name		= "dpll4_m4_ck",
	.parent		= &dpll4_ck,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_DSS1_MASK,
	.clksel_mask2	= OMAP3630_CLKSEL_DSS1_MASK,
	.clksel_shift	= OMAP3430_CLKSEL_DSS1_SHIFT,
	.clksel		= div16_dpll4_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK
					| CLOCK_IN_OMAP363X,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
	.set_rate	= &omap2_clksel_set_rate,
	.round_rate	= &omap2_clksel_round_rate,
};

/* The PWRDN bit is apparently only available on 3430ES2 and above */
static struct clk dpll4_m4x2_ck = {
	.name		= "dpll4_m4x2_ck",
	.parent		= &dpll4_m4_ck,
	.prcm_mod	= PLL_MOD,
	.enable_reg	= CM_CLKEN,
	.enable_bit	= OMAP3430_PWRDN_DSS1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | INVERT_ENABLE,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

/* This virtual clock is the source for dpll4_m5x2_ck */
static struct clk dpll4_m5_ck = {
	.name		= "dpll4_m5_ck",
	.parent		= &dpll4_ck,
	.prcm_mod	= OMAP3430_CAM_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_CAM_MASK,
	.clksel_mask2	= OMAP3630_CLKSEL_CAM_MASK,
	.clksel_shift	= OMAP3430_CLKSEL_CAM_SHIFT,
	.clksel		= div16_dpll4_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK
					| CLOCK_IN_OMAP363X,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
	.set_rate	= &omap2_clksel_set_rate,
	.round_rate	= &omap2_clksel_round_rate,
};

/* The PWRDN bit is apparently only available on 3430ES2 and above */
static struct clk dpll4_m5x2_ck = {
	.name		= "dpll4_m5x2_ck",
	.parent		= &dpll4_m5_ck,
	.prcm_mod	= PLL_MOD,
	.enable_reg	= CM_CLKEN,
	.enable_bit	= OMAP3430_PWRDN_CAM_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | INVERT_ENABLE,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

/* This virtual clock is the source for dpll4_m6x2_ck */
static struct clk dpll4_m6_ck = {
	.name		= "dpll4_m6_ck",
	.parent		= &dpll4_ck,
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_DIV_DPLL4_MASK,
	.clksel_mask2	= OMAP3630_DIV_DPLL4_MASK,
	.clksel_shift	= OMAP3430_DIV_DPLL4_SHIFT,
	.clksel		= div16_dpll4_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK
					| CLOCK_IN_OMAP363X,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* The PWRDN bit is apparently only available on 3430ES2 and above */
static struct clk dpll4_m6x2_ck = {
	.name		= "dpll4_m6x2_ck",
	.parent		= &dpll4_m6_ck,
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_CLKEN,
	.enable_bit	= OMAP3430_PWRDN_EMU_PERIPH_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | INVERT_ENABLE,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &omap3_clkoutx2_recalc,
};

static struct clk emu_per_alwon_ck = {
	.name		= "emu_per_alwon_ck",
	.parent		= &dpll4_m6x2_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* DPLL5 */
/* Supplies 120MHz clock, USIM source clock */
/* Type: DPLL */
/* 3430ES2 only */
static struct dpll_data dpll5_dd = {
	.mult_div1_reg	= OMAP3430ES2_CM_CLKSEL4,
	.mult_mask	= OMAP3430ES2_PERIPH2_DPLL_MULT_MASK,
	.div1_mask	= OMAP3430ES2_PERIPH2_DPLL_DIV_MASK,
	.freqsel_mask	= OMAP3430ES2_PERIPH2_DPLL_FREQSEL_MASK,
	.control_reg	= OMAP3430ES2_CM_CLKEN2,
	.enable_mask	= OMAP3430ES2_EN_PERIPH2_DPLL_MASK,
	.modes		= (1 << DPLL_LOW_POWER_STOP) | (1 << DPLL_LOCKED),
	.auto_recal_bit	= OMAP3430ES2_EN_PERIPH2_DPLL_DRIFTGUARD_SHIFT,
	.recal_en_bit	= OMAP3430ES2_SND_PERIPH_DPLL_RECAL_EN_SHIFT,
	.recal_st_bit	= OMAP3430ES2_SND_PERIPH_DPLL_ST_SHIFT,
	.autoidle_reg	= OMAP3430ES2_CM_AUTOIDLE2_PLL,
	.autoidle_mask	= OMAP3430ES2_AUTO_PERIPH2_DPLL_MASK,
	.idlest_reg	= CM_IDLEST2,
	.idlest_mask	= OMAP3430ES2_ST_PERIPH2_CLK_MASK,
	.bypass_clk	= &sys_ck,
	.max_multiplier = OMAP3_MAX_DPLL_MULT,
	.min_divider	= 1,
	.max_divider	= OMAP3_MAX_DPLL_DIV,
	.rate_tolerance = DEFAULT_DPLL_RATE_TOLERANCE
};

static struct clk dpll5_ck = {
	.name		= "dpll5_ck",
	.parent		= &sys_ck,
	.prcm_mod	= PLL_MOD,
	.dpll_data	= &dpll5_dd,
	.flags		= CLOCK_IN_OMAP3430ES2 | RECALC_ON_ENABLE,
	.enable		= &omap3_noncore_dpll_enable,
	.disable	= &omap3_noncore_dpll_disable,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
	.clkdm		= { .name = "dpll5_clkdm" },
	.recalc		= &omap3_dpll_recalc,
};

static const struct clksel div16_dpll5_clksel[] = {
	{ .parent = &dpll5_ck, .rates = div16_dpll_rates },
	{ .parent = NULL }
};

static struct clk dpll5_m2_ck = {
	.name		= "dpll5_m2_ck",
	.parent		= &dpll5_ck,
	.prcm_mod	= PLL_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430ES2_CM_CLKSEL5,
	.clksel_mask	= OMAP3430ES2_DIV_120M_MASK,
	.clksel		= div16_dpll5_clksel,
	.flags		= CLOCK_IN_OMAP3430ES2 | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "dpll5_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* CM EXTERNAL CLOCK OUTPUTS */

static const struct clksel_rate clkout2_src_core_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate clkout2_src_sys_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate clkout2_src_96m_rates[] = {
	{ .div = 1, .val = 2, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate clkout2_src_54m_rates[] = {
	{ .div = 1, .val = 3, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel clkout2_src_clksel[] = {
	{ .parent = &core_ck,		.rates = clkout2_src_core_rates },
	{ .parent = &sys_ck,		.rates = clkout2_src_sys_rates },
	{ .parent = &cm_96m_fck,	.rates = clkout2_src_96m_rates },
	{ .parent = &omap_54m_fck,	.rates = clkout2_src_54m_rates },
	{ .parent = NULL }
};

static struct clk clkout2_src_ck = {
	.name		= "clkout2_src_ck",
	.prcm_mod	= OMAP3430_CCR_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= OMAP3430_CM_CLKOUT_CTRL_OFFSET,
	.enable_bit	= OMAP3430_CLKOUT2_EN_SHIFT,
	.clksel_reg	= OMAP3430_CM_CLKOUT_CTRL_OFFSET,
	.clksel_mask	= OMAP3430_CLKOUT2SOURCE_MASK,
	.clksel		= clkout2_src_clksel,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate sys_clkout2_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 1, .flags = RATE_IN_343X },
	{ .div = 4, .val = 2, .flags = RATE_IN_343X },
	{ .div = 8, .val = 3, .flags = RATE_IN_343X },
	{ .div = 16, .val = 4, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel sys_clkout2_clksel[] = {
	{ .parent = &clkout2_src_ck, .rates = sys_clkout2_rates },
	{ .parent = NULL },
};

static struct clk sys_clkout2 = {
	.name		= "sys_clkout2",
	.prcm_mod	= OMAP3430_CCR_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430_CM_CLKOUT_CTRL_OFFSET,
	.clksel_mask	= OMAP3430_CLKOUT2_DIV_MASK,
	.clksel		= sys_clkout2_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
	.set_rate	= &omap2_clksel_set_rate,
	.round_rate	= &omap2_clksel_round_rate,
};

/* CM OUTPUT CLOCKS */

static struct clk corex2_fck = {
	.name		= "corex2_fck",
	.parent		= &dpll3_m2x2_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &followparent_recalc,
};

/* DPLL power domain clock controls */

static const struct clksel_rate div4_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 2, .flags = RATE_IN_343X },
	{ .div = 4, .val = 4, .flags = RATE_IN_343X },
	{ .div = 0 }
};

static const struct clksel div4_core_clksel[] = {
	{ .parent = &core_ck, .rates = div4_rates },
	{ .parent = NULL }
};

static struct clk dpll1_fck = {
	.name		= "dpll1_fck",
	.parent		= &core_ck,
	.prcm_mod	= MPU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430_CM_CLKSEL1_PLL,
	.clksel_mask	= OMAP3430_MPU_CLK_SRC_MASK,
	.clksel		= div4_core_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mpu_ck = {
	.name		= "mpu_ck",
	.parent		= &dpll1_x2m2_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "mpu_clkdm" },
	.recalc		= &followparent_recalc,
};

/* arm_fck is divided by two when DPLL1 locked; otherwise, passthrough mpu_ck */
static const struct clksel_rate arm_fck_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 1, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel arm_fck_clksel[] = {
	{ .parent = &mpu_ck, .rates = arm_fck_rates },
	{ .parent = NULL }
};

static struct clk arm_fck = {
	.name		= "arm_fck",
	.parent		= &mpu_ck,
	.prcm_mod	= MPU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430_CM_IDLEST_PLL,
	.clksel_mask	= OMAP3430_ST_MPU_CLK_MASK,
	.clksel		= arm_fck_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "mpu_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* XXX What about neon_clkdm ? */

/*
 * REVISIT: This clock is never specifically defined in the 3430 TRM,
 * although it is referenced - so this is a guess
 */
static struct clk emu_mpu_alwon_ck = {
	.name		= "emu_mpu_alwon_ck",
	.parent		= &mpu_ck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "mpu_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk dpll2_fck = {
	.name		= "dpll2_fck",
	.parent		= &core_ck,
	.prcm_mod	= OMAP3430_IVA2_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP3430_CM_CLKSEL1_PLL,
	.clksel_mask	= OMAP3430_IVA2_CLK_SRC_MASK,
	.clksel		= div4_core_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk iva2_ck = {
	.name		= "iva2_ck",
	.parent		= &dpll2_m2_ck,
	.prcm_mod	= OMAP3430_IVA2_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_CM_FCLKEN_IVA2_EN_IVA2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "iva2_clkdm" },
	.recalc		= &followparent_recalc,
};

/* Common interface clocks */

static const struct clksel div2_core_clksel[] = {
	{ .parent = &core_ck, .rates = div2_rates },
	{ .parent = NULL }
};

static struct clk l3_ick = {
	.name		= "l3_ick",
	.parent		= &core_ck,
	.prcm_mod	= CORE_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_L3_MASK,
	.clksel		= div2_core_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel div2_l3_clksel[] = {
	{ .parent = &l3_ick, .rates = div2_rates },
	{ .parent = NULL }
};

static struct clk l4_ick = {
	.name		= "l4_ick",
	.parent		= &l3_ick,
	.prcm_mod	= CORE_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_L4_MASK,
	.clksel		= div2_l3_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,

};

static const struct clksel div2_l4_clksel[] = {
	{ .parent = &l4_ick, .rates = div2_rates },
	{ .parent = NULL }
};

static struct clk rm_ick = {
	.name		= "rm_ick",
	.parent		= &l4_ick,
	.prcm_mod	= WKUP_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_RM_MASK,
	.clksel		= div2_l4_clksel,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* GFX power domain */

/* GFX clocks are in 3430ES1 only. 3430ES2 and later uses the SGX instead */

static const struct clksel gfx_l3_clksel[] = {
	{ .parent = &l3_ick, .rates = gfx_l3_rates },
	{ .parent = NULL }
};

/* Virtual parent clock for gfx_l3_ick and gfx_l3_fck */
static struct clk gfx_l3_ck = {
	.name		= "gfx_l3_ck",
	.parent		= &l3_ick,
	.prcm_mod	= GFX_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP_EN_GFX_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "gfx_3430es1_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gfx_l3_fck = {
	.name		= "gfx_l3_fck",
	.parent		= &gfx_l3_ck,
	.prcm_mod	= GFX_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP_CLKSEL_GFX_MASK,
	.clksel		= gfx_l3_clksel,
	.flags		= CLOCK_IN_OMAP3430ES1 | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "gfx_3430es1_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gfx_l3_ick = {
	.name		= "gfx_l3_ick",
	.parent		= &gfx_l3_ck,
	.flags		= CLOCK_IN_OMAP3430ES1 | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "gfx_3430es1_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gfx_cg1_ck = {
	.name		= "gfx_cg1_ck",
	.parent		= &gfx_l3_fck, /* REVISIT: correct? */
	.prcm_mod	= GFX_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430ES1_EN_2D_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "gfx_3430es1_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gfx_cg2_ck = {
	.name		= "gfx_cg2_ck",
	.parent		= &gfx_l3_fck, /* REVISIT: correct? */
	.prcm_mod	= GFX_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430ES1_EN_3D_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "gfx_3430es1_clkdm" },
	.recalc		= &followparent_recalc,
};

/* SGX power domain - 3430ES2 only */

static const struct clksel_rate sgx_core_rates[] = {
	{ .div = 2, .val = 5, .flags = RATE_IN_363X },
	{ .div = 3, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 4, .val = 1, .flags = RATE_IN_343X },
	{ .div = 6, .val = 2, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel_rate sgx_192m_rates[] = {
	{ .div = 1,  .val = 4, .flags = RATE_IN_363X | DEFAULT_RATE },
	{ .div = 0 },
};

static const struct clksel_rate sgx_corex2_rates[] = {
	{ .div = 3, .val = 6, .flags = RATE_IN_363X | DEFAULT_RATE },
	{ .div = 5, .val = 7, .flags = RATE_IN_363X },
	{ .div = 0 },
};

static const struct clksel_rate sgx_96m_rates[] = {
	{ .div = 1,  .val = 3, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 },
};

static const struct clksel sgx_clksel[] = {
	{ .parent = &core_ck,	 .rates = sgx_core_rates },
	{ .parent = &cm_96m_fck, .rates = sgx_96m_rates },
	{ .parent = &omap_192m_alwon_ck, .rates = sgx_192m_rates },
	{ .parent = &corex2_fck, .rates = sgx_corex2_rates },

	{ .parent = NULL },
};

static struct clk sgx_fck = {
	.name		= "sgx_fck",
	.init		= &omap2_init_clksel_parent,
	.prcm_mod	= OMAP3430ES2_SGX_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430ES2_CM_FCLKEN_SGX_EN_SGX_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430ES2_CLKSEL_SGX_MASK,
	.clksel		= sgx_clksel,
	.flags		= CLOCK_IN_OMAP3430ES2,
	.clkdm		= { .name = "sgx_clkdm" },
	.recalc		= &omap2_clksel_recalc,
	.set_rate	= &omap2_clksel_set_rate,
	.round_rate	= &omap2_clksel_round_rate,
};

static struct clk sgx_ick = {
	.name		= "sgx_ick",
	.parent		= &l3_ick,
	.prcm_mod	= OMAP3430ES2_SGX_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430ES2_CM_ICLKEN_SGX_EN_SGX_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2,
	.clkdm		= { .name = "sgx_clkdm" },
	.recalc		= &followparent_recalc,
};

/* CORE power domain */

static struct clk d2d_26m_fck = {
	.name		= "d2d_26m_fck",
	.parent		= &sys_ck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430ES1_EN_D2D_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "d2d_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk modem_fck = {
	.name		= "modem_fck",
	.parent		= &sys_ck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MODEM_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "d2d_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk sad2d_ick = {
	.name		= "sad2d_ick",
	.parent		= &l3_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_SAD2D_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "d2d_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mad2d_ick = {
	.name		= "mad2d_ick",
	.parent		= &l3_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN3,
	.enable_bit	= OMAP3430_EN_MAD2D_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "d2d_clkdm" },
	.recalc		= &followparent_recalc,
};

static const struct clksel omap343x_gpt_clksel[] = {
	{ .parent = &omap_32k_fck, .rates = gpt_32k_rates },
	{ .parent = &sys_ck,	   .rates = gpt_sys_rates },
	{ .parent = NULL}
};

static struct clk gpt10_fck = {
	.name		= "gpt10_fck",
	.parent		= &sys_ck,
	.prcm_mod	= CORE_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_GPT10_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT10_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT10_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt11_fck = {
	.name		= "gpt11_fck",
	.parent		= &sys_ck,
	.prcm_mod	= CORE_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_GPT11_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT11_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT11_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk cpefuse_fck = {
	.name		= "cpefuse_fck",
	.parent		= &sys_ck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= OMAP3430ES2_CM_FCLKEN3,
	.enable_bit	= OMAP3430ES2_EN_CPEFUSE_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_CPEFUSE_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "cm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk ts_fck = {
	.name		= "ts_fck",
	.parent		= &omap_32k_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= OMAP3430ES2_CM_FCLKEN3,
	.enable_bit	= OMAP3430ES2_EN_TS_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk usbtll_fck = {
	.name		= "usbtll_fck",
	.parent		= &dpll5_m2_ck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= OMAP3430ES2_CM_FCLKEN3,
	.enable_bit	= OMAP3430ES2_EN_USBTLL_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_USBTLL_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* CORE 96M FCLK-derived clocks */

static struct clk core_96m_fck = {
	.name		= "core_96m_fck",
	.parent		= &omap_96m_fck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mmchs3_fck = {
	.name		= "mmchs_fck",
	.id		= 2,
	.parent		= &core_96m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430ES2_EN_MMC3_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_MMC3_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mmchs2_fck = {
	.name		= "mmchs_fck",
	.id		= 1,
	.parent		= &core_96m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MMC2_SHIFT,
	.idlest_bit	= OMAP3430_ST_MMC2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mspro_fck = {
	.name		= "mspro_fck",
	.parent		= &core_96m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MSPRO_SHIFT,
	.idlest_bit	= OMAP3430_ST_MSPRO_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mmchs1_fck = {
	.name		= "mmchs_fck",
	.parent		= &core_96m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MMC1_SHIFT,
	.idlest_bit	= OMAP3430_ST_MMC1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk i2c3_fck = {
	.name		= "i2c_fck",
	.id		= 3,
	.parent		= &core_96m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_I2C3_SHIFT,
	.idlest_bit	= OMAP3430_ST_I2C3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk i2c2_fck = {
	.name		= "i2c_fck",
	.id		= 2,
	.parent		= &core_96m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_I2C2_SHIFT,
	.idlest_bit	= OMAP3430_ST_I2C2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk i2c1_fck = {
	.name		= "i2c_fck",
	.id		= 1,
	.parent		= &core_96m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_I2C1_SHIFT,
	.idlest_bit	= OMAP3430_ST_I2C1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/*
 * MCBSP 1 & 5 get their 96MHz clock from core_96m_fck;
 * MCBSP 2, 3, 4 get their 96MHz clock from per_96m_fck.
 */
static const struct clksel_rate common_mcbsp_96m_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel_rate common_mcbsp_mcbsp_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 }
};

static const struct clksel mcbsp_15_clksel[] = {
	{ .parent = &core_96m_fck, .rates = common_mcbsp_96m_rates },
	{ .parent = &mcbsp_clks,   .rates = common_mcbsp_mcbsp_rates },
	{ .parent = NULL }
};

static struct clk mcbsp5_src_fck = {
	.name		= "mcbsp_src_fck",
	.id		= 5,
	.prcm_mod	= CLK_REG_IN_SCM,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP343X_CONTROL_DEVCONF1,
	.clksel_mask	= OMAP2_MCBSP5_CLKS_MASK,
	.clksel		= mcbsp_15_clksel,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mcbsp5_fck = {
	.name		= "mcbsp_fck",
	.id		= 5,
	.parent		= &mcbsp5_src_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MCBSP5_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP5_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcbsp1_src_fck = {
	.name		= "mcbsp_src_fck",
	.id		= 1,
	.prcm_mod	= CLK_REG_IN_SCM,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP2_CONTROL_DEVCONF0,
	.clksel_mask	= OMAP2_MCBSP1_CLKS_MASK,
	.clksel		= mcbsp_15_clksel,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mcbsp1_fck = {
	.name		= "mcbsp_fck",
	.id		= 1,
	.parent		= &mcbsp1_src_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MCBSP1_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* CORE_48M_FCK-derived clocks */

static struct clk core_48m_fck = {
	.name		= "core_48m_fck",
	.parent		= &omap_48m_fck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi4_fck = {
	.name		= "mcspi_fck",
	.id		= 4,
	.parent		= &core_48m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI4_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI4_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi3_fck = {
	.name		= "mcspi_fck",
	.id		= 3,
	.parent		= &core_48m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI3_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi2_fck = {
	.name		= "mcspi_fck",
	.id		= 2,
	.parent		= &core_48m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI2_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi1_fck = {
	.name		= "mcspi_fck",
	.id		= 1,
	.parent		= &core_48m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI1_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk uart2_fck = {
	.name		= "uart2_fck",
	.parent		= &core_48m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_UART2_SHIFT,
	.idlest_bit	= OMAP3430_ST_UART2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk uart1_fck = {
	.name		= "uart1_fck",
	.parent		= &core_48m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_UART1_SHIFT,
	.idlest_bit	= OMAP3430_ST_UART1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* XXX doublecheck: is this idle or standby? */
static struct clk fshostusb_fck = {
	.name		= "fshostusb_fck",
	.parent		= &core_48m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430ES1_EN_FSHOSTUSB_SHIFT,
	.idlest_bit	= OMAP3430ES1_ST_FSHOSTUSB_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* CORE_12M_FCK based clocks */

static struct clk core_12m_fck = {
	.name		= "core_12m_fck",
	.parent		= &omap_12m_fck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk hdq_fck = {
	.name		= "hdq_fck",
	.parent		= &core_12m_fck,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_HDQ_SHIFT,
	.idlest_bit	= OMAP3430_ST_HDQ_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* DPLL3-derived clock */

static const struct clksel_rate ssi_ssr_corex2_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 2, .flags = RATE_IN_343X },
	{ .div = 3, .val = 3, .flags = RATE_IN_343X },
	{ .div = 4, .val = 4, .flags = RATE_IN_343X },
	{ .div = 6, .val = 6, .flags = RATE_IN_343X },
	{ .div = 8, .val = 8, .flags = RATE_IN_343X },
	{ .div = 0 }
};

static const struct clksel ssi_ssr_clksel[] = {
	{ .parent = &corex2_fck, .rates = ssi_ssr_corex2_rates },
	{ .parent = NULL }
};

static struct clk ssi_ssr_fck_3430es1 = {
	.name		= "ssi_ssr_fck",
	.init		= &omap2_init_clksel_parent,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_SSI_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_SSI_MASK,
	.clksel		= ssi_ssr_clksel,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk ssi_ssr_fck_3430es2 = {
	.name		= "ssi_ssr_fck",
	.init		= &omap2_init_clksel_parent,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_FCLKEN1,
	.enable_bit	= OMAP3430_EN_SSI_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_SSI_IDLE_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_SSI_MASK,
	.clksel		= ssi_ssr_clksel,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* It's unfortunate that we need to duplicate this clock. */
static struct clk ssi_sst_fck_3430es1 = {
	.name		= "ssi_sst_fck",
	.parent		= &ssi_ssr_fck_3430es1,
	.fixed_div	= 2,
	.flags		= CLOCK_IN_OMAP3430ES1 | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_fixed_divisor_recalc,
};

static struct clk ssi_sst_fck_3430es2 = {
	.name		= "ssi_sst_fck",
	.parent		= &ssi_ssr_fck_3430es2,
	.fixed_div	= 2,
	.flags		= CLOCK_IN_OMAP3430ES2 | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_fixed_divisor_recalc,
};



/* CORE_L3_ICK based clocks */

/*
 * XXX must add clk_enable/clk_disable for these if standard code won't
 * handle it
 */
static struct clk core_l3_ick = {
	.name		= "core_l3_ick",
	.parent		= &l3_ick,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk hsotgusb_ick_3430es1 = {
	.name		= "hsotgusb_ick",
	.parent		= &core_l3_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_HSOTGUSB_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk hsotgusb_ick_3430es2 = {
	.name		= "hsotgusb_ick",
	.parent		= &core_l3_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_HSOTGUSB_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_HSOTGUSB_IDLE_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk sdrc_ick = {
	.name		= "sdrc_ick",
	.parent		= &core_l3_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_SDRC_SHIFT,
	.idlest_bit	= OMAP3430_ST_SDRC_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | ENABLE_ON_INIT | WAIT_READY,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpmc_fck = {
	.name		= "gpmc_fck",
	.parent		= &core_l3_ick,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK |
				ENABLE_ON_INIT,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &followparent_recalc,
};

/* SECURITY_L3_ICK based clocks */

static struct clk security_l3_ick = {
	.name		= "security_l3_ick",
	.parent		= &l3_ick,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk pka_ick = {
	.name		= "pka_ick",
	.parent		= &security_l3_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN2,
	.enable_bit	= OMAP3430_EN_PKA_SHIFT,
	.idlest_bit	= OMAP3430_ST_PKA_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l3_clkdm" },
	.recalc		= &followparent_recalc,
};

/* CORE_L4_ICK based clocks */

static struct clk core_l4_ick = {
	.name		= "core_l4_ick",
	.parent		= &l4_ick,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk usbtll_ick = {
	.name		= "usbtll_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN3,
	.enable_bit	= OMAP3430ES2_EN_USBTLL_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_USBTLL_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mmchs3_ick = {
	.name		= "mmchs_ick",
	.id		= 2,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430ES2_EN_MMC3_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_MMC3_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* Intersystem Communication Registers - chassis mode only */
static struct clk icr_ick = {
	.name		= "icr_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_ICR_SHIFT,
	.idlest_bit	= OMAP3430_ST_ICR_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk aes2_ick = {
	.name		= "aes2_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_AES2_SHIFT,
	.idlest_bit	= OMAP3430_ST_AES2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk sha12_ick = {
	.name		= "sha12_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_SHA12_SHIFT,
	.idlest_bit	= OMAP3430_ST_SHA12_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk des2_ick = {
	.name		= "des2_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_DES2_SHIFT,
	.idlest_bit	= OMAP3430_ST_DES2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mmchs2_ick = {
	.name		= "mmchs_ick",
	.id		= 1,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MMC2_SHIFT,
	.idlest_bit	= OMAP3430_ST_MMC2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mmchs1_ick = {
	.name		= "mmchs_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MMC1_SHIFT,
	.idlest_bit	= OMAP3430_ST_MMC1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mspro_ick = {
	.name		= "mspro_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MSPRO_SHIFT,
	.idlest_bit	= OMAP3430_ST_MSPRO_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk hdq_ick = {
	.name		= "hdq_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_HDQ_SHIFT,
	.idlest_bit	= OMAP3430_ST_HDQ_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi4_ick = {
	.name		= "mcspi_ick",
	.id		= 4,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI4_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI4_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi3_ick = {
	.name		= "mcspi_ick",
	.id		= 3,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI3_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi2_ick = {
	.name		= "mcspi_ick",
	.id		= 2,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI2_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcspi1_ick = {
	.name		= "mcspi_ick",
	.id		= 1,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MCSPI1_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCSPI1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk i2c3_ick = {
	.name		= "i2c_ick",
	.id		= 3,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_I2C3_SHIFT,
	.idlest_bit	= OMAP3430_ST_I2C3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk i2c2_ick = {
	.name		= "i2c_ick",
	.id		= 2,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_I2C2_SHIFT,
	.idlest_bit	= OMAP3430_ST_I2C2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk i2c1_ick = {
	.name		= "i2c_ick",
	.id		= 1,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_I2C1_SHIFT,
	.idlest_bit	= OMAP3430_ST_I2C1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk uart2_ick = {
	.name		= "uart2_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_UART2_SHIFT,
	.idlest_bit	= OMAP3430_ST_UART2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk uart1_ick = {
	.name		= "uart1_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_UART1_SHIFT,
	.idlest_bit	= OMAP3430_ST_UART1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt11_ick = {
	.name		= "gpt11_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_GPT11_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT11_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt10_ick = {
	.name		= "gpt10_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_GPT10_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT10_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcbsp5_ick = {
	.name		= "mcbsp_ick",
	.id		= 5,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MCBSP5_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP5_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcbsp1_ick = {
	.name		= "mcbsp_ick",
	.id		= 1,
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MCBSP1_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk fac_ick = {
	.name		= "fac_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430ES1_EN_FAC_SHIFT,
	.idlest_bit	= OMAP3430ES1_ST_FAC_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mailboxes_ick = {
	.name		= "mailboxes_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_MAILBOXES_SHIFT,
	.idlest_bit	= OMAP3430_ST_MAILBOXES_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk omapctrl_ick = {
	.name		= "omapctrl_ick",
	.parent		= &core_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_OMAPCTRL_SHIFT,
	.idlest_bit	= OMAP3430_ST_OMAPCTRL_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | ENABLE_ON_INIT | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* SSI_L4_ICK based clocks */

static struct clk ssi_l4_ick = {
	.name		= "ssi_l4_ick",
	.parent		= &l4_ick,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk ssi_ick_3430es1 = {
	.name		= "ssi_ick",
	.parent		= &ssi_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_SSI_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk ssi_ick_3430es2 = {
	.name		= "ssi_ick",
	.parent		= &ssi_l4_ick,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430_EN_SSI_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_SSI_IDLE_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/*
 * REVISIT: Technically the TRM claims that this is CORE_CLK based,
 * but l4_ick makes more sense to me
 */
static const struct clksel usb_l4_clksel[] = {
	{ .parent = &l4_ick, .rates = div2_rates },
	{ .parent = NULL },
};

static struct clk usb_l4_ick = {
	.name		= "usb_l4_ick",
	.parent		= &l4_ick,
	.prcm_mod	= CORE_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_ICLKEN1,
	.enable_bit	= OMAP3430ES1_EN_FSHOSTUSB_SHIFT,
	.idlest_bit	= OMAP3430ES1_ST_FSHOSTUSB_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430ES1_CLKSEL_FSHOSTUSB_MASK,
	.clksel		= usb_l4_clksel,
	.flags		= CLOCK_IN_OMAP3430ES1 | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* SECURITY_L4_ICK2 based clocks */

static struct clk security_l4_ick2 = {
	.name		= "security_l4_ick2",
	.parent		= &l4_ick,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk aes1_ick = {
	.name		= "aes1_ick",
	.parent		= &security_l4_ick2,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN2,
	.enable_bit	= OMAP3430_EN_AES1_SHIFT,
	.idlest_bit	= OMAP3430_ST_AES1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk rng_ick = {
	.name		= "rng_ick",
	.parent		= &security_l4_ick2,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN2,
	.enable_bit	= OMAP3430_EN_RNG_SHIFT,
	.idlest_bit	= OMAP3430_ST_RNG_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk sha11_ick = {
	.name		= "sha11_ick",
	.parent		= &security_l4_ick2,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN2,
	.enable_bit	= OMAP3430_EN_SHA11_SHIFT,
	.idlest_bit	= OMAP3430_ST_SHA11_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk des1_ick = {
	.name		= "des1_ick",
	.parent		= &security_l4_ick2,
	.prcm_mod	= CORE_MOD,
	.enable_reg	= CM_ICLKEN2,
	.enable_bit	= OMAP3430_EN_DES1_SHIFT,
	.idlest_bit	= OMAP3430_ST_DES1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* DSS */
static struct clk dss1_alwon_fck_3430es1 = {
	.name		= "dss1_alwon_fck",
	.parent		= &dpll4_m4x2_ck,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_DSS1_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "dss_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk dss1_alwon_fck_3430es2 = {
	.name		= "dss1_alwon_fck",
	.parent		= &dpll4_m4x2_ck,
	.init		= &omap2_init_clksel_parent,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_DSS1_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_DSS_IDLE_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "dss_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk dss_tv_fck = {
	.name		= "dss_tv_fck",
	.parent		= &omap_54m_fck,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_TV_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "dss_clkdm" }, /* XXX: in cm_clkdm? */
	.recalc		= &followparent_recalc,
};

static struct clk dss_96m_fck = {
	.name		= "dss_96m_fck",
	.parent		= &omap_96m_fck,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_TV_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "dss_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk dss2_alwon_fck = {
	.name		= "dss2_alwon_fck",
	.parent		= &sys_ck,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_DSS2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "dss_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk dss_ick_3430es1 = {
	/* Handles both L3 and L4 clocks */
	.name		= "dss_ick",
	.parent		= &l4_ick,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_CM_ICLKEN_DSS_EN_DSS_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES1,
	.clkdm		= { .name = "dss_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk dss_ick_3430es2 = {
	/* Handles both L3 and L4 clocks */
	.name		= "dss_ick",
	.parent		= &l4_ick,
	.prcm_mod	= OMAP3430_DSS_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_CM_ICLKEN_DSS_EN_DSS_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_DSS_IDLE_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "dss_clkdm" },
	.recalc		= &followparent_recalc,
};

/* CAM */

static struct clk cam_mclk = {
	.name		= "cam_mclk",
	.parent		= &dpll4_m5x2_ck,
	.prcm_mod	= OMAP3430_CAM_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_CAM_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "cam_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk cam_ick = {
	/* Handles both L3 and L4 clocks */
	.name		= "cam_ick",
	.parent		= &l4_ick,
	.prcm_mod	= OMAP3430_CAM_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_CAM_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "cam_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk csi2_96m_fck = {
	.name		= "csi2_96m_fck",
	.parent		= &core_96m_fck,
	.prcm_mod	= OMAP3430_CAM_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_CSI2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "cam_clkdm" },
	.recalc		= &followparent_recalc,
};

/* USBHOST - 3430ES2 only */

static struct clk usbhost_120m_fck = {
	.name		= "usbhost_120m_fck",
	.parent		= &dpll5_m2_ck,
	.prcm_mod	= OMAP3430ES2_USBHOST_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430ES2_EN_USBHOST2_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2,
	.clkdm		= { .name = "usbhost_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk usbhost_48m_fck = {
	.name		= "usbhost_48m_fck",
	.parent		= &omap_48m_fck,
	.prcm_mod	= OMAP3430ES2_USBHOST_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430ES2_EN_USBHOST1_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_USBHOST_IDLE_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "usbhost_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk usbhost_ick = {
	/* Handles both L3 and L4 clocks */
	.name		= "usbhost_ick",
	.parent		= &l4_ick,
	.prcm_mod	= OMAP3430ES2_USBHOST_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430ES2_EN_USBHOST_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_USBHOST_IDLE_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "usbhost_clkdm" },
	.recalc		= &followparent_recalc,
};

/* WKUP */

static const struct clksel_rate usim_96m_rates[] = {
	{ .div = 2,  .val = 3, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 4,  .val = 4, .flags = RATE_IN_343X },
	{ .div = 8,  .val = 5, .flags = RATE_IN_343X },
	{ .div = 10, .val = 6, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel_rate usim_120m_rates[] = {
	{ .div = 4,  .val = 7,	.flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 8,  .val = 8,	.flags = RATE_IN_343X },
	{ .div = 16, .val = 9,	.flags = RATE_IN_343X },
	{ .div = 20, .val = 10, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel usim_clksel[] = {
	{ .parent = &omap_96m_fck,	.rates = usim_96m_rates },
	{ .parent = &dpll5_m2_ck,	.rates = usim_120m_rates },
	{ .parent = &sys_ck,		.rates = div2_rates },
	{ .parent = NULL },
};

/* 3430ES2 only */
static struct clk usim_fck = {
	.name		= "usim_fck",
	.prcm_mod	= WKUP_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430ES2_EN_USIMOCP_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_USIMOCP_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430ES2_CLKSEL_USIMOCP_MASK,
	.clksel		= usim_clksel,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* XXX should gpt1's clksel have wkup_32k_fck as the 32k opt? */
static struct clk gpt1_fck = {
	.name		= "gpt1_fck",
	.prcm_mod	= WKUP_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT1_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT1_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT1_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk wkup_32k_fck = {
	.name		= "wkup_32k_fck",
	.parent		= &omap_32k_fck,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_dbck = {
	.name		= "gpio1_dbck",
	.parent		= &wkup_32k_fck,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPIO1_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk wdt2_fck = {
	.name		= "wdt2_fck",
	.parent		= &wkup_32k_fck,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_WDT2_SHIFT,
	.idlest_bit	= OMAP3430_ST_WDT2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk wkup_l4_ick = {
	.name		= "wkup_l4_ick",
	.parent		= &sys_ck,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk usim_ick = {
	.name		= "usim_ick",
	.parent		= &wkup_l4_ick,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430ES2_EN_USIMOCP_SHIFT,
	.idlest_bit	= OMAP3430ES2_ST_USIMOCP_SHIFT,
	.flags		= CLOCK_IN_OMAP3430ES2 | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk wdt2_ick = {
	.name		= "wdt2_ick",
	.parent		= &wkup_l4_ick,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_WDT2_SHIFT,
	.idlest_bit	= OMAP3430_ST_WDT2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk wdt1_ick = {
	.name		= "wdt1_ick",
	.parent		= &wkup_l4_ick,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_WDT1_SHIFT,
	.idlest_bit	= OMAP3430_ST_WDT1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_ick = {
	.name		= "gpio1_ick",
	.parent		= &wkup_l4_ick,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPIO1_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk omap_32ksync_ick = {
	.name		= "omap_32ksync_ick",
	.parent		= &wkup_l4_ick,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_32KSYNC_SHIFT,
	.idlest_bit	= OMAP3430_ST_32KSYNC_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt12_ick = {
	.name		= "gpt12_ick",
	.parent		= &wkup_l4_ick,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT12_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT12_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt1_ick = {
	.name		= "gpt1_ick",
	.parent		= &wkup_l4_ick,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT1_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};



/* PER clock domain */

static struct clk per_96m_fck = {
	.name		= "per_96m_fck",
	.parent		= &omap_96m_alwon_fck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk per_48m_fck = {
	.name		= "per_48m_fck",
	.parent		= &omap_48m_fck,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk uart3_fck = {
	.name		= "uart3_fck",
	.parent		= &per_48m_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_UART3_SHIFT,
	.idlest_bit	= OMAP3430_ST_UART3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt2_fck = {
	.name		= "gpt2_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT2_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT2_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT2_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt3_fck = {
	.name		= "gpt3_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT3_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT3_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT3_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt4_fck = {
	.name		= "gpt4_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT4_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT4_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT4_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt5_fck = {
	.name		= "gpt5_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT5_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT5_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT5_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt6_fck = {
	.name		= "gpt6_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT6_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT6_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT6_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt7_fck = {
	.name		= "gpt7_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT7_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT7_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT7_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt8_fck = {
	.name		= "gpt8_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT8_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT8_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT8_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpt9_fck = {
	.name		= "gpt9_fck",
	.prcm_mod	= OMAP3430_PER_MOD,
	.init		= &omap2_init_clksel_parent,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPT9_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT9_SHIFT,
	.clksel_reg	= CM_CLKSEL,
	.clksel_mask	= OMAP3430_CLKSEL_GPT9_MASK,
	.clksel		= omap343x_gpt_clksel,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk per_32k_alwon_fck = {
	.name		= "per_32k_alwon_fck",
	.parent		= &omap_32k_fck,
	.clkdm		= { .name = "per_clkdm" },
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.recalc		= &followparent_recalc,
};

static struct clk gpio6_dbck = {
	.name		= "gpio6_dbck",
	.parent		= &per_32k_alwon_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPIO6_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO6_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio5_dbck = {
	.name		= "gpio5_dbck",
	.parent		= &per_32k_alwon_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPIO5_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO5_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio4_dbck = {
	.name		= "gpio4_dbck",
	.parent		= &per_32k_alwon_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPIO4_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO4_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio3_dbck = {
	.name		= "gpio3_dbck",
	.parent		= &per_32k_alwon_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPIO3_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_dbck = {
	.name		= "gpio2_dbck",
	.parent		= &per_32k_alwon_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_GPIO2_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk wdt3_fck = {
	.name		= "wdt3_fck",
	.parent		= &per_32k_alwon_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_WDT3_SHIFT,
	.idlest_bit	= OMAP3430_ST_WDT3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk per_l4_ick = {
	.name		= "per_l4_ick",
	.parent		= &l4_ick,
	.flags		= CLOCK_IN_OMAP343X | PARENT_CONTROLS_CLOCK,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio6_ick = {
	.name		= "gpio6_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPIO6_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO6_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio5_ick = {
	.name		= "gpio5_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPIO5_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO5_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio4_ick = {
	.name		= "gpio4_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPIO4_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO4_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio3_ick = {
	.name		= "gpio3_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPIO3_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_ick = {
	.name		= "gpio2_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPIO2_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPIO2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk wdt3_ick = {
	.name		= "wdt3_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_WDT3_SHIFT,
	.idlest_bit	= OMAP3430_ST_WDT3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk uart3_ick = {
	.name		= "uart3_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_UART3_SHIFT,
	.idlest_bit	= OMAP3430_ST_UART3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt9_ick = {
	.name		= "gpt9_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT9_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT9_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt8_ick = {
	.name		= "gpt8_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT8_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT8_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt7_ick = {
	.name		= "gpt7_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT7_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT7_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt6_ick = {
	.name		= "gpt6_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT6_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT6_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt5_ick = {
	.name		= "gpt5_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT5_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT5_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt4_ick = {
	.name		= "gpt4_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT4_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT4_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt3_ick = {
	.name		= "gpt3_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT3_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk gpt2_ick = {
	.name		= "gpt2_ick",
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_GPT2_SHIFT,
	.idlest_bit	= OMAP3430_ST_GPT2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcbsp2_ick = {
	.name		= "mcbsp_ick",
	.id		= 2,
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_MCBSP2_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcbsp3_ick = {
	.name		= "mcbsp_ick",
	.id		= 3,
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_MCBSP3_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk mcbsp4_ick = {
	.name		= "mcbsp_ick",
	.id		= 4,
	.parent		= &per_l4_ick,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_ICLKEN,
	.enable_bit	= OMAP3430_EN_MCBSP4_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP4_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &followparent_recalc,
};

static const struct clksel mcbsp_234_clksel[] = {
	{ .parent = &core_96m_fck, .rates = common_mcbsp_96m_rates },
	{ .parent = &mcbsp_clks,   .rates = common_mcbsp_mcbsp_rates },
	{ .parent = NULL }
};

static struct clk mcbsp2_src_fck = {
	.name		= "mcbsp_src_fck",
	.id		= 2,
	.prcm_mod	= CLK_REG_IN_SCM,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP2_CONTROL_DEVCONF0,
	.clksel_mask	= OMAP2_MCBSP2_CLKS_MASK,
	.clksel		= mcbsp_234_clksel,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mcbsp2_fck = {
	.name		= "mcbsp_fck",
	.id		= 2,
	.parent		= &mcbsp2_src_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_MCBSP2_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mcbsp3_src_fck = {
	.name		= "mcbsp_src_fck",
	.id		= 3,
	.prcm_mod	= CLK_REG_IN_SCM,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP343X_CONTROL_DEVCONF1,
	.clksel_mask	= OMAP2_MCBSP3_CLKS_MASK,
	.clksel		= mcbsp_234_clksel,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mcbsp3_fck = {
	.name		= "mcbsp_fck",
	.id		= 3,
	.parent		= &mcbsp3_src_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_MCBSP3_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP3_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mcbsp4_src_fck = {
	.name		= "mcbsp_src_fck",
	.id		= 4,
	.prcm_mod	= CLK_REG_IN_SCM,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= OMAP343X_CONTROL_DEVCONF1,
	.clksel_mask	= OMAP2_MCBSP4_CLKS_MASK,
	.clksel		= mcbsp_234_clksel,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk mcbsp4_fck = {
	.name		= "mcbsp_fck",
	.id		= 4,
	.parent		= &mcbsp4_src_fck,
	.prcm_mod	= OMAP3430_PER_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_MCBSP4_SHIFT,
	.idlest_bit	= OMAP3430_ST_MCBSP4_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "per_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* EMU clocks */

/* More information: ARM Cortex-A8 Technical Reference Manual, sect 10.1 */

static const struct clksel_rate emu_src_sys_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 },
};

static const struct clksel_rate emu_src_core_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 },
};

static const struct clksel_rate emu_src_per_rates[] = {
	{ .div = 1, .val = 2, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 },
};

static const struct clksel_rate emu_src_mpu_rates[] = {
	{ .div = 1, .val = 3, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 0 },
};

static const struct clksel emu_src_clksel[] = {
	{ .parent = &sys_ck,		.rates = emu_src_sys_rates },
	{ .parent = &emu_core_alwon_ck, .rates = emu_src_core_rates },
	{ .parent = &emu_per_alwon_ck,	.rates = emu_src_per_rates },
	{ .parent = &emu_mpu_alwon_ck,	.rates = emu_src_mpu_rates },
	{ .parent = NULL },
};

/*
 * Like the clkout_src clocks, emu_src_clk is a virtual clock, existing only
 * to switch the source of some of the EMU clocks.
 * XXX Are there CLKEN bits for these EMU clks?
 */
static struct clk emu_src_ck = {
	.name		= "emu_src_ck",
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_MUX_CTRL_MASK,
	.clksel		= emu_src_clksel,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "emu_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate pclk_emu_rates[] = {
	{ .div = 2, .val = 2, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 3, .val = 3, .flags = RATE_IN_343X },
	{ .div = 4, .val = 4, .flags = RATE_IN_343X },
	{ .div = 6, .val = 6, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel pclk_emu_clksel[] = {
	{ .parent = &emu_src_ck, .rates = pclk_emu_rates },
	{ .parent = NULL },
};

static struct clk pclk_fck = {
	.name		= "pclk_fck",
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_CLKSEL_PCLK_MASK,
	.clksel		= pclk_emu_clksel,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "emu_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate pclkx2_emu_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 2, .flags = RATE_IN_343X },
	{ .div = 3, .val = 3, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel pclkx2_emu_clksel[] = {
	{ .parent = &emu_src_ck, .rates = pclkx2_emu_rates },
	{ .parent = NULL },
};

static struct clk pclkx2_fck = {
	.name		= "pclkx2_fck",
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_CLKSEL_PCLKX2_MASK,
	.clksel		= pclkx2_emu_clksel,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "emu_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel atclk_emu_clksel[] = {
	{ .parent = &emu_src_ck, .rates = div2_rates },
	{ .parent = NULL },
};

static struct clk atclk_fck = {
	.name		= "atclk_fck",
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_CLKSEL_ATCLK_MASK,
	.clksel		= atclk_emu_clksel,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "emu_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static struct clk traceclk_src_fck = {
	.name		= "traceclk_src_fck",
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_TRACE_MUX_CTRL_MASK,
	.clksel		= emu_src_clksel,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "emu_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate traceclk_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_343X | DEFAULT_RATE },
	{ .div = 2, .val = 2, .flags = RATE_IN_343X },
	{ .div = 4, .val = 4, .flags = RATE_IN_343X },
	{ .div = 0 },
};

static const struct clksel traceclk_clksel[] = {
	{ .parent = &traceclk_src_fck, .rates = traceclk_rates },
	{ .parent = NULL },
};

static struct clk traceclk_fck = {
	.name		= "traceclk_fck",
	.prcm_mod	= OMAP3430_EMU_MOD,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= CM_CLKSEL1,
	.clksel_mask	= OMAP3430_CLKSEL_TRACECLK_MASK,
	.clksel		= traceclk_clksel,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "emu_clkdm" },
	.recalc		= &omap2_clksel_recalc,
};

/* SR clocks */

/* SmartReflex fclk (VDD1) */
static struct clk sr1_fck = {
	.name		= "sr1_fck",
	.parent		= &sys_ck,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_SR1_SHIFT,
	.idlest_bit	= OMAP3430_ST_SR1_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

/* SmartReflex fclk (VDD2) */
static struct clk sr2_fck = {
	.name		= "sr2_fck",
	.parent		= &sys_ck,
	.prcm_mod	= WKUP_MOD,
	.enable_reg	= CM_FCLKEN,
	.enable_bit	= OMAP3430_EN_SR2_SHIFT,
	.idlest_bit	= OMAP3430_ST_SR2_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk sr_l4_ick = {
	.name		= "sr_l4_ick",
	.parent		= &l4_ick,
	.flags		= CLOCK_IN_OMAP343X,
	.clkdm		= { .name = "core_l4_clkdm" },
	.recalc		= &followparent_recalc,
};

/* SECURE_32K_FCK clocks */

/* XXX Make sure idlest_bit/wait_ready with no enable_bit works */
static struct clk gpt12_fck = {
	.name		= "gpt12_fck",
	.parent		= &secure_32k_fck,
	.idlest_bit	= OMAP3430_ST_GPT12_SHIFT,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED | WAIT_READY,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk wdt1_fck = {
	.name		= "wdt1_fck",
	.parent		= &secure_32k_fck,
	.flags		= CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.clkdm		= { .name = "prm_clkdm" },
	.recalc		= &followparent_recalc,
};

static struct clk *onchip_34xx_clks[] __initdata = {
	&omap_32k_fck,
	&virt_12m_ck,
	&virt_13m_ck,
	&virt_16_8m_ck,
	&virt_19_2m_ck,
	&virt_26m_ck,
	&virt_38_4m_ck,
	&osc_sys_ck,
	&sys_ck,
	&sys_altclk,
	&mcbsp_clks,
	&sys_clkout1,
	&dpll1_ck,
	&dpll1_x2_ck,
	&dpll1_x2m2_ck,
	&dpll2_ck,
	&dpll2_m2_ck,
	&dpll3_ck,
	&core_ck,
	&dpll3_m2_ck,
	&dpll3_m2x2_ck,
	&dpll3_m3_ck,
	&dpll3_m3x2_ck,
	&emu_core_alwon_ck,
	&dpll4_ck,
	&omap_192m_alwon_ck,
	&omap_96m_alwon_fck,
	&omap_96m_fck,
	&cm_96m_fck,
	&omap_54m_fck,
	&omap_48m_fck,
	&omap_12m_fck,
	&dpll4_m2_ck,
	&dpll4_m2x2_ck,
	&dpll4_m3_ck,
	&dpll4_m3x2_ck,
	&dpll4_m4_ck,
	&dpll4_m4x2_ck,
	&dpll4_m5_ck,
	&dpll4_m5x2_ck,
	&dpll4_m6_ck,
	&dpll4_m6x2_ck,
	&emu_per_alwon_ck,
	&dpll5_ck,
	&dpll5_m2_ck,
	&clkout2_src_ck,
	&sys_clkout2,
	&corex2_fck,
	&dpll1_fck,
	&mpu_ck,
	&arm_fck,
	&emu_mpu_alwon_ck,
	&dpll2_fck,
	&iva2_ck,
	&l3_ick,
	&l4_ick,
	&rm_ick,
	&gfx_l3_ck,
	&gfx_l3_fck,
	&gfx_l3_ick,
	&gfx_cg1_ck,
	&gfx_cg2_ck,
	&sgx_fck,
	&sgx_ick,
	&d2d_26m_fck,
	&modem_fck,
	&sad2d_ick,
	&mad2d_ick,
	&gpt10_fck,
	&gpt11_fck,
	&cpefuse_fck,
	&ts_fck,
	&usbtll_fck,
	&core_96m_fck,
	&mmchs3_fck,
	&mmchs2_fck,
	&mspro_fck,
	&mmchs1_fck,
	&i2c3_fck,
	&i2c2_fck,
	&i2c1_fck,
	&mcbsp5_src_fck,
	&mcbsp5_fck,
	&mcbsp1_src_fck,
	&mcbsp1_fck,
	&core_48m_fck,
	&mcspi4_fck,
	&mcspi3_fck,
	&mcspi2_fck,
	&mcspi1_fck,
	&uart2_fck,
	&uart1_fck,
	&fshostusb_fck,
	&core_12m_fck,
	&hdq_fck,
	&ssi_ssr_fck_3430es1,
	&ssi_ssr_fck_3430es2,
	&ssi_sst_fck_3430es1,
	&ssi_sst_fck_3430es2,
	&core_l3_ick,
	&hsotgusb_ick_3430es1,
	&hsotgusb_ick_3430es2,
	&sdrc_ick,
	&gpmc_fck,
	&security_l3_ick,
	&pka_ick,
	&core_l4_ick,
	&usbtll_ick,
	&mmchs3_ick,
	&icr_ick,
	&aes2_ick,
	&sha12_ick,
	&des2_ick,
	&mmchs2_ick,
	&mmchs1_ick,
	&mspro_ick,
	&hdq_ick,
	&mcspi4_ick,
	&mcspi3_ick,
	&mcspi2_ick,
	&mcspi1_ick,
	&i2c3_ick,
	&i2c2_ick,
	&i2c1_ick,
	&uart2_ick,
	&uart1_ick,
	&gpt11_ick,
	&gpt10_ick,
	&mcbsp5_ick,
	&mcbsp1_ick,
	&fac_ick,
	&mailboxes_ick,
	&omapctrl_ick,
	&ssi_l4_ick,
	&ssi_ick_3430es1,
	&ssi_ick_3430es2,
	&usb_l4_ick,
	&security_l4_ick2,
	&aes1_ick,
	&rng_ick,
	&sha11_ick,
	&des1_ick,
	&dss1_alwon_fck_3430es1,
	&dss1_alwon_fck_3430es2,
	&dss_tv_fck,
	&dss_96m_fck,
	&dss2_alwon_fck,
	&dss_ick_3430es1,
	&dss_ick_3430es2,
	&cam_mclk,
	&cam_ick,
	&csi2_96m_fck,
	&usbhost_120m_fck,
	&usbhost_48m_fck,
	&usbhost_ick,
	&usim_fck,
	&gpt1_fck,
	&wkup_32k_fck,
	&gpio1_dbck,
	&wdt2_fck,
	&wkup_l4_ick,
	&usim_ick,
	&wdt2_ick,
	&wdt1_ick,
	&gpio1_ick,
	&omap_32ksync_ick,
	&gpt12_ick,
	&gpt1_ick,
	&per_96m_fck,
	&per_48m_fck,
	&uart3_fck,
	&gpt2_fck,
	&gpt3_fck,
	&gpt4_fck,
	&gpt5_fck,
	&gpt6_fck,
	&gpt7_fck,
	&gpt8_fck,
	&gpt9_fck,
	&per_32k_alwon_fck,
	&gpio6_dbck,
	&gpio5_dbck,
	&gpio4_dbck,
	&gpio3_dbck,
	&gpio2_dbck,
	&wdt3_fck,
	&per_l4_ick,
	&gpio6_ick,
	&gpio5_ick,
	&gpio4_ick,
	&gpio3_ick,
	&gpio2_ick,
	&wdt3_ick,
	&uart3_ick,
	&gpt9_ick,
	&gpt8_ick,
	&gpt7_ick,
	&gpt6_ick,
	&gpt5_ick,
	&gpt4_ick,
	&gpt3_ick,
	&gpt2_ick,
	&mcbsp2_ick,
	&mcbsp3_ick,
	&mcbsp4_ick,
	&mcbsp2_src_fck,
	&mcbsp2_fck,
	&mcbsp3_src_fck,
	&mcbsp3_fck,
	&mcbsp4_src_fck,
	&mcbsp4_fck,
	&emu_src_ck,
	&pclk_fck,
	&pclkx2_fck,
	&atclk_fck,
	&traceclk_src_fck,
	&traceclk_fck,
	&sr1_fck,
	&sr2_fck,
	&sr_l4_ick,
	&secure_32k_fck,
	&gpt12_fck,
	&wdt1_fck,
};

#endif
