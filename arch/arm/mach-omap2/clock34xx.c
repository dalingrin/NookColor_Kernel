/*
 * OMAP3-specific clock framework functions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2008 Nokia Corporation
 *
 * Written by Paul Walmsley
 * Testing and integration fixes by Jouni Högander
 *
 * Parts of this code are based on code written by
 * Richard Woodruff, Tony Lindgren, Tuukka Tikkanen, Karthik Dasu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/cpufreq.h>
#include <linux/i2c/twl4030.h>

#include <mach/clock.h>
#include <mach/sram.h>
#include <asm/div64.h>

#include <mach/sdrc.h>
#include "clock.h"
#include "clock34xx.h"
#include "prm.h"
#include "prm-regbits-34xx.h"
#include "cm.h"
#include "cm-regbits-34xx.h"
#include "pm.h"

/* CM_AUTOIDLE_PLL*.AUTO_* bit values */
#define DPLL_AUTOIDLE_DISABLE			0x0
#define DPLL_AUTOIDLE_LOW_POWER_STOP		0x1

#define MAX_DPLL_WAIT_TRIES		1000000

#define MIN_SDRC_DLL_LOCK_FREQ		83000000

#define CYCLES_PER_MHZ			1000000

/* Scale factor for fixed-point arith in omap3_core_dpll_m2_set_rate() */
#define SDRC_MPURATE_SCALE		8

/* 2^SDRC_MPURATE_BASE_SHIFT: MPU MHz that SDRC_MPURATE_LOOPS is defined for */
#define SDRC_MPURATE_BASE_SHIFT		9

/*
 * SDRC_MPURATE_LOOPS: Number of MPU loops to execute at
 * 2^MPURATE_BASE_SHIFT MHz for SDRC to stabilize
 */
#define SDRC_MPURATE_LOOPS		24

/**
 * omap3_dpll_recalc - recalculate DPLL rate
 * @clk: DPLL struct clk
 * @parent_rate: rate of the DPLL's parent clock
 * @rate_storage: flag indicating whether current or temporary rate is changing
 *
 * Recalculate and propagate the DPLL rate.
 */
static void omap3_dpll_recalc(struct clk *clk, unsigned long parent_rate,
			      u8 rate_storage)
{
	unsigned long rate;

	rate = omap2_get_dpll_rate(clk, parent_rate);

	if (rate_storage == CURRENT_RATE)
		clk->rate = rate;
	else if (rate_storage == TEMP_RATE)
		clk->temp_rate = rate;
}

/* _omap3_dpll_write_clken - write clken_bits arg to a DPLL's enable bits */
static void _omap3_dpll_write_clken(struct clk *clk, u8 clken_bits)
{
	const struct dpll_data *dd;
	u32 v;

	dd = clk->dpll_data;

	v = cm_read_mod_reg(clk->prcm_mod, dd->control_reg);
	v &= ~dd->enable_mask;
	v |= clken_bits << __ffs(dd->enable_mask);
	cm_write_mod_reg(v, clk->prcm_mod, dd->control_reg);
}

/* _omap3_wait_dpll_status: wait for a DPLL to enter a specific state */
static int _omap3_wait_dpll_status(struct clk *clk, u8 state)
{
	const struct dpll_data *dd;
	int i = 0;
	int ret = -EINVAL;

	dd = clk->dpll_data;

	state <<= __ffs(dd->idlest_mask);

	while (((cm_read_mod_reg(clk->prcm_mod, dd->idlest_reg)
		 & dd->idlest_mask) != state) &&
	       i < MAX_DPLL_WAIT_TRIES) {
		i++;
		udelay(1);
	}

	if (i == MAX_DPLL_WAIT_TRIES) {
		printk(KERN_ERR "clock: %s failed transition to '%s'\n",
		       clk->name, (state) ? "locked" : "bypassed");
	} else {
		pr_debug("clock: %s transition to '%s' in %d loops\n",
			 clk->name, (state) ? "locked" : "bypassed", i);

		ret = 0;
	}

	return ret;
}

/* From 3430 TRM ES2 4.7.6.2 */
static u16 _omap3_dpll_compute_freqsel(struct clk *clk, u8 n)
{
	unsigned long fint;
	u16 f = 0;

	fint = clk->parent->rate / (n + 1);

	pr_debug("clock: fint is %lu\n", fint);

	if (fint >= 750000 && fint <= 1000000)
		f = 0x3;
	else if (fint > 1000000 && fint <= 1250000)
		f = 0x4;
	else if (fint > 1250000 && fint <= 1500000)
		f = 0x5;
	else if (fint > 1500000 && fint <= 1750000)
		f = 0x6;
	else if (fint > 1750000 && fint <= 2100000)
		f = 0x7;
	else if (fint > 7500000 && fint <= 10000000)
		f = 0xB;
	else if (fint > 10000000 && fint <= 12500000)
		f = 0xC;
	else if (fint > 12500000 && fint <= 15000000)
		f = 0xD;
	else if (fint > 15000000 && fint <= 17500000)
		f = 0xE;
	else if (fint > 17500000 && fint <= 21000000)
		f = 0xF;
	else
		pr_debug("clock: unknown freqsel setting for %d\n", n);

	return f;
}

/* Non-CORE DPLL (e.g., DPLLs that do not control SDRC) clock functions */

/*
 * _omap3_noncore_dpll_lock - instruct a DPLL to lock and wait for readiness
 * @clk: pointer to a DPLL struct clk
 *
 * Instructs a non-CORE DPLL to lock.  Waits for the DPLL to report
 * readiness before returning.  Will save and restore the DPLL's
 * autoidle state across the enable, per the CDP code.  If the DPLL
 * locked successfully, return 0; if the DPLL did not lock in the time
 * allotted, or DPLL3 was passed in, return -EINVAL.
 */
static int _omap3_noncore_dpll_lock(struct clk *clk)
{
	u8 ai;
	int r;

	if (clk == &dpll3_ck)
		return -EINVAL;

	pr_debug("clock: locking DPLL %s\n", clk->name);

	ai = omap3_dpll_autoidle_read(clk);

	omap3_dpll_deny_idle(clk);

	_omap3_dpll_write_clken(clk, DPLL_LOCKED);

	r = _omap3_wait_dpll_status(clk, 1);

	if (ai)
		omap3_dpll_allow_idle(clk);

	return r;
}

/*
 * _omap3_noncore_dpll_bypass - instruct a DPLL to bypass and wait for readiness
 * @clk: pointer to a DPLL struct clk
 *
 * Instructs a non-CORE DPLL to enter low-power bypass mode.  In
 * bypass mode, the DPLL's rate is set equal to its parent clock's
 * rate.  Waits for the DPLL to report readiness before returning.
 * Will save and restore the DPLL's autoidle state across the enable,
 * per the CDP code.  If the DPLL entered bypass mode successfully,
 * return 0; if the DPLL did not enter bypass in the time allotted, or
 * DPLL3 was passed in, or the DPLL does not support low-power bypass,
 * return -EINVAL.
 */
static int _omap3_noncore_dpll_bypass(struct clk *clk)
{
	int r;
	u8 ai;

	if (clk == &dpll3_ck)
		return -EINVAL;

	if (!(clk->dpll_data->modes & (1 << DPLL_LOW_POWER_BYPASS)))
		return -EINVAL;

	pr_debug("clock: configuring DPLL %s for low-power bypass\n",
		 clk->name);

	ai = omap3_dpll_autoidle_read(clk);

	_omap3_dpll_write_clken(clk, DPLL_LOW_POWER_BYPASS);

	r = _omap3_wait_dpll_status(clk, 0);

	if (ai)
		omap3_dpll_allow_idle(clk);
	else
		omap3_dpll_deny_idle(clk);

	return r;
}

/*
 * _omap3_noncore_dpll_stop - instruct a DPLL to stop
 * @clk: pointer to a DPLL struct clk
 *
 * Instructs a non-CORE DPLL to enter low-power stop. Will save and
 * restore the DPLL's autoidle state across the stop, per the CDP
 * code.  If DPLL3 was passed in, or the DPLL does not support
 * low-power stop, return -EINVAL; otherwise, return 0.
 */
static int _omap3_noncore_dpll_stop(struct clk *clk)
{
	u8 ai;

	if (clk == &dpll3_ck)
		return -EINVAL;

	if (!(clk->dpll_data->modes & (1 << DPLL_LOW_POWER_STOP)))
		return -EINVAL;

	pr_debug("clock: stopping DPLL %s\n", clk->name);

	ai = omap3_dpll_autoidle_read(clk);

	_omap3_dpll_write_clken(clk, DPLL_LOW_POWER_STOP);

	if (ai)
		omap3_dpll_allow_idle(clk);
	else
		omap3_dpll_deny_idle(clk);

	return 0;
}

/**
 * omap3_noncore_dpll_enable - instruct a DPLL to enter bypass or lock mode
 * @clk: pointer to a DPLL struct clk
 *
 * Instructs a non-CORE DPLL to enable, e.g., to enter bypass or lock.
 * The choice of modes depends on the DPLL's programmed rate: if it is
 * the same as the DPLL's parent clock, it will enter bypass;
 * otherwise, it will enter lock.  This code will wait for the DPLL to
 * indicate readiness before returning, unless the DPLL takes too long
 * to enter the target state.  Intended to be used as the struct clk's
 * enable function.  If DPLL3 was passed in, or the DPLL does not
 * support low-power stop, or if the DPLL took too long to enter
 * bypass or lock, return -EINVAL; otherwise, return 0.
 */
static int omap3_noncore_dpll_enable(struct clk *clk)
{
	int r;
	struct dpll_data *dd;

	if (clk == &dpll3_ck)
		return -EINVAL;

	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	if (clk->rate == dd->bypass_clk->rate)
		r = _omap3_noncore_dpll_bypass(clk);
	else
		r = _omap3_noncore_dpll_lock(clk);

	return r;
}

/**
 * omap3_noncore_dpll_disable - instruct a DPLL to enter bypass or lock mode
 * @clk: pointer to a DPLL struct clk
 *
 * Instructs a non-CORE DPLL to enable, e.g., to enter bypass or lock.
 * The choice of modes depends on the DPLL's programmed rate: if it is
 * the same as the DPLL's parent clock, it will enter bypass;
 * otherwise, it will enter lock.  This code will wait for the DPLL to
 * indicate readiness before returning, unless the DPLL takes too long
 * to enter the target state.  Intended to be used as the struct clk's
 * enable function.  If DPLL3 was passed in, or the DPLL does not
 * support low-power stop, or if the DPLL took too long to enter
 * bypass or lock, return -EINVAL; otherwise, return 0.
 */
static void omap3_noncore_dpll_disable(struct clk *clk)
{
	if (clk == &dpll3_ck)
		return;

	_omap3_noncore_dpll_stop(clk);
}

/* OMAP3630 j-type DPLL4 compensation variables */
void lookup_dco_sddiv(struct clk *clk, u8 *dco, u8 *sd_div, u16 m, u8 n)
{
	unsigned long fint, clkinp, sd; /* watch out for overflow */
	int mod1, mod2;

	++n; /* always n+1 below */
	clkinp = clk->parent->rate;
	fint = (clkinp / n) * m;

	if (fint < 1000000000)
		*dco = 2;
	else
		*dco = 4;
	/*
	* target sigma-delta to near 250MHz
	* sd = ceil[(m/(n+1)) * (clkinp_MHz / 250)]
	*/
	clkinp /= 100000; /* shift from MHz to 10*Hz for 38.4 and 19.2*/
	mod1 = (clkinp * m) % (250 * n);
	sd = (clkinp * m) / (250 * n);
	mod2 = sd % 10;
	sd /= 10;

	if (mod1 + mod2)
		++sd;
	*sd_div = sd;
}

/*
 * omap3_noncore_dpll_program - set non-core DPLL M,N values directly
 * @clk: struct clk * of DPLL to set
 * @m: DPLL multiplier to set
 * @n: DPLL divider to set
 * @freqsel: FREQSEL value to set
 *
 * Program the DPLL with the supplied M, N values, and wait for the DPLL to
 * lock..  Returns -EINVAL upon error, or 0 upon success.
 */
static int omap3_noncore_dpll_program(struct clk *clk, u16 m, u8 n, u16 freqsel)
{
	struct dpll_data *dd;
	u32 v;

	if (!clk)
		return -EINVAL;

	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	/*
	 * According to the 12-5 CDP code from TI, "Limitation 2.5"
	 * on 3430ES1 prevents us from changing DPLL multipliers or dividers
	 * on DPLL4.
	 */
	if (omap_rev() == OMAP3430_REV_ES1_0 &&
	    !strcmp("dpll4_ck", clk->name)) {
		printk(KERN_ERR "clock: DPLL4 cannot change rate due to "
		       "silicon 'Limitation 2.5' on 3430ES1.\n");
		return -EINVAL;
	}

	/* 3430 ES2 TRM: 4.7.6.9 DPLL Programming Sequence */
	_omap3_noncore_dpll_bypass(clk);

	/* Set jitter correction */
	v = cm_read_mod_reg(clk->prcm_mod, dd->control_reg);
	v &= ~dd->freqsel_mask;
	v |= freqsel << __ffs(dd->freqsel_mask);
	cm_write_mod_reg(v, clk->prcm_mod, dd->control_reg);

	/* Set DPLL multiplier, divider */
	v = cm_read_mod_reg(clk->prcm_mod, dd->mult_div1_reg);
	v &= ~(dd->mult_mask | dd->div1_mask);
	v |= m << __ffs(dd->mult_mask);
	v |= (n - 1) << __ffs(dd->div1_mask);
	if (dd->jtype) {
		u8 dco, sd_div;
		lookup_dco_sddiv(clk, &dco, &sd_div, m, n);
		v &= ~(dd->dco_sel_mask | dd->sd_div_mask);
		v |=  dco << __ffs(dd->dco_sel_mask);
		v |=  sd_div << __ffs(dd->sd_div_mask);
	}
	cm_write_mod_reg(v, clk->prcm_mod, dd->mult_div1_reg);

	/* We let the clock framework set the other output dividers later */

	/* REVISIT: Set ramp-up delay? */

	_omap3_noncore_dpll_lock(clk);

	return 0;
}

/**
 * omap3_noncore_dpll_set_rate - set non-core DPLL rate
 * @clk: struct clk * of DPLL to set
 * @rate: rounded target rate
 *
 * Set the DPLL CLKOUT to the target rate.  If the DPLL can enter
 * low-power bypass, and the target rate is the bypass source clock
 * rate, then configure the DPLL for bypass.  Otherwise, round the
 * target rate if it hasn't been done already, then program and lock
 * the DPLL.  Returns -EINVAL upon error, or 0 upon success.
 */
static int omap3_noncore_dpll_set_rate(struct clk *clk, unsigned long rate)
{
	u16 freqsel;
	struct dpll_data *dd;
	int ret;

	if (!clk || !rate)
		return -EINVAL;

	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	if (rate == omap2_get_dpll_rate(clk, clk->parent->rate))
		return 0;

	if (dd->bypass_clk->rate == rate &&
	    (clk->dpll_data->modes & (1 << DPLL_LOW_POWER_BYPASS))) {

		pr_debug("clock: %s: set rate: entering bypass.\n", clk->name);

		ret = _omap3_noncore_dpll_bypass(clk);
		if (!ret)
			clk->rate = rate;

	} else {

		if (dd->last_rounded_rate != rate)
			omap2_dpll_round_rate(clk, rate);

		if (dd->last_rounded_rate == 0)
			return -EINVAL;

		freqsel = _omap3_dpll_compute_freqsel(clk, dd->last_rounded_n);
		if (!freqsel)
			WARN_ON(1);

		pr_debug("clock: %s: set rate: locking rate to %lu.\n",
			 clk->name, rate);

		ret = omap3_noncore_dpll_program(clk, dd->last_rounded_m,
						 dd->last_rounded_n, freqsel);

		if (!ret)
			clk->rate = rate;

	}

	return 0;
}


/*
 * CORE DPLL (DPLL3) rate programming functions
 *
 * These call into SRAM code to do the actual CM writes, since the SDRAM
 * is clocked from DPLL3.
 */

/**
 * omap3_core_dpll_m2_set_rate - set CORE DPLL M2 divider
 * @clk: struct clk * of DPLL to set
 * @rate: rounded target rate
 *
 * Program the DPLL M2 divider with the rounded target rate.  Returns
 * -EINVAL upon error, or 0 upon success.
 */
static int omap3_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 new_div = 0;
	u32 unlock_dll = 0;
	u32 c;
	unsigned long validrate, sdrcrate, mpurate;
	struct omap_sdrc_params *sp;

	if (!clk || !rate)
		return -EINVAL;

	if (clk != &dpll3_m2_ck)
		return -EINVAL;

	validrate = omap2_clksel_round_rate_div(clk, rate, &new_div);
	if (validrate != rate)
		return -EINVAL;

	sdrcrate = sdrc_ick.rate;
	if (rate > clk->rate)
		sdrcrate <<= ((rate / clk->rate) >> 1);
	else
		sdrcrate >>= ((clk->rate / rate) >> 1);

	sp = omap2_sdrc_get_params(sdrcrate);
	if (!sp)
		return -EINVAL;

	if (sdrcrate < MIN_SDRC_DLL_LOCK_FREQ) {
		pr_debug("clock: will unlock SDRC DLL\n");
		unlock_dll = 1;
	}

	/*
	 * XXX This only needs to be done when the CPU frequency changes
	 */
	mpurate = arm_fck.rate / CYCLES_PER_MHZ;
	c = (mpurate << SDRC_MPURATE_SCALE) >> SDRC_MPURATE_BASE_SHIFT;
	c += 1;  /* for safety */
	c *= SDRC_MPURATE_LOOPS;
	c >>= SDRC_MPURATE_SCALE;
	if (c == 0)
		c = 1;

	pr_debug("clock: changing CORE DPLL rate from %lu to %lu\n", clk->rate,
		 validrate);
	pr_debug("clock: SDRC timing params used: %08x %08x %08x\n",
		 sp->rfr_ctrl, sp->actim_ctrla, sp->actim_ctrlb);

	omap3_configure_core_dpll(sp->rfr_ctrl, sp->actim_ctrla,
				  sp->actim_ctrlb, new_div, unlock_dll, c,
				  sp->mr, rate > clk->rate);

	return 0;
}


/* DPLL autoidle read/set code */


/**
 * omap3_dpll_autoidle_read - read a DPLL's autoidle bits
 * @clk: struct clk * of the DPLL to read
 *
 * Return the DPLL's autoidle bits, shifted down to bit 0.  Returns
 * -EINVAL if passed a null pointer or if the struct clk does not
 * appear to refer to a DPLL.
 */
static u32 omap3_dpll_autoidle_read(struct clk *clk)
{
	const struct dpll_data *dd;
	u32 v;

	if (!clk || !clk->dpll_data)
		return -EINVAL;

	dd = clk->dpll_data;

	v = cm_read_mod_reg(clk->prcm_mod, dd->autoidle_reg);
	v &= dd->autoidle_mask;
	v >>= __ffs(dd->autoidle_mask);

	return v;
}

/**
 * omap3_dpll_allow_idle - enable DPLL autoidle bits
 * @clk: struct clk * of the DPLL to operate on
 *
 * Enable DPLL automatic idle control.  This automatic idle mode
 * switching takes effect only when the DPLL is locked, at least on
 * OMAP3430.  The DPLL will enter low-power stop when its downstream
 * clocks are gated.  No return value.
 */
static void omap3_dpll_allow_idle(struct clk *clk)
{
	const struct dpll_data *dd;
	u32 v;

	if (!clk || !clk->dpll_data)
		return;

	dd = clk->dpll_data;

	/*
	 * REVISIT: CORE DPLL can optionally enter low-power bypass
	 * by writing 0x5 instead of 0x1.  Add some mechanism to
	 * optionally enter this mode.
	 */
	v = cm_read_mod_reg(clk->prcm_mod, dd->autoidle_reg);
	v &= ~dd->autoidle_mask;
	v |= DPLL_AUTOIDLE_LOW_POWER_STOP << __ffs(dd->autoidle_mask);
	cm_write_mod_reg(v, clk->prcm_mod, dd->autoidle_reg);
}

/**
 * omap3_dpll_deny_idle - prevent DPLL from automatically idling
 * @clk: struct clk * of the DPLL to operate on
 *
 * Disable DPLL automatic idle control.  No return value.
 */
static void omap3_dpll_deny_idle(struct clk *clk)
{
	const struct dpll_data *dd;
	u32 v;

	if (!clk || !clk->dpll_data)
		return;

	dd = clk->dpll_data;

	v = cm_read_mod_reg(clk->prcm_mod, dd->autoidle_reg);
	v &= ~dd->autoidle_mask;
	v |= DPLL_AUTOIDLE_DISABLE << __ffs(dd->autoidle_mask);
	cm_write_mod_reg(v, clk->prcm_mod, dd->autoidle_reg);
}

/* Clock control for DPLL outputs */

/**
 * omap3_clkoutx2_recalc - recalculate DPLL X2 output virtual clock rate
 * @clk: DPLL output struct clk
 * @parent_rate: rate of the parent clock of @clk
 * @rate_storage: flag indicating whether current or temporary rate is changing
 *
 * Using parent clock DPLL data, look up DPLL state.  If locked, set our
 * rate to the dpll_clk * 2; otherwise, just use dpll_clk.
 */
static void omap3_clkoutx2_recalc(struct clk *clk, unsigned long parent_rate,
				  u8 rate_storage)
{
	const struct dpll_data *dd;
	u32 v;
	unsigned long rate;
	struct clk *pclk;

	/* Walk up the parents of clk, looking for a DPLL */
	pclk = clk->parent;
	while (pclk && !pclk->dpll_data)
		pclk = pclk->parent;

	/* clk does not have a DPLL as a parent? */
	WARN_ON(!pclk);

	dd = pclk->dpll_data;

	WARN_ON(!dd->enable_mask);

	rate = parent_rate;

	v = cm_read_mod_reg(pclk->prcm_mod, dd->control_reg) & dd->enable_mask;
	v >>= __ffs(dd->enable_mask);
	if (v == OMAP3XXX_EN_DPLL_LOCKED && (!dd->jtype))
		rate *= 2;

	if (rate_storage == CURRENT_RATE)
		clk->rate = rate;
	else if (rate_storage == TEMP_RATE)
		clk->temp_rate = rate;
}

/* Common clock code */

/*
 * As it is structured now, this will prevent an OMAP2/3 multiboot
 * kernel from compiling.  This will need further attention.
 */
#if defined(CONFIG_ARCH_OMAP3)

#ifdef CONFIG_CPU_FREQ
static struct cpufreq_frequency_table freq_table[VDD1_OPP6+1];

void omap2_clk_init_cpufreq_table(struct cpufreq_frequency_table **table)
{
	struct omap_opp *prcm;
	int i = 0;

	if (!mpu_opps)
		return;

	prcm = mpu_opps + MAX_VDD1_OPP;
	for (; prcm->rate; prcm--) {
		freq_table[i].index = i;
		freq_table[i].frequency = prcm->rate / 1000;
		i++;
	}

	if (i == 0) {
		printk(KERN_WARNING "%s: failed to initialize frequency \
								table\n",
								__func__);
		return;
	}

	freq_table[i].index = i;
	freq_table[i].frequency = CPUFREQ_TABLE_END;

	*table = &freq_table[0];
}
#endif

static struct clk_functions omap2_clk_functions = {
	.clk_register		= omap2_clk_register,
	.clk_enable		= omap2_clk_enable,
	.clk_disable		= omap2_clk_disable,
	.clk_round_rate		= omap2_clk_round_rate,
	.clk_round_rate_parent	= omap2_clk_round_rate_parent,
	.clk_set_rate		= omap2_clk_set_rate,
	.clk_set_parent		= omap2_clk_set_parent,
	.clk_get_parent		= omap2_clk_get_parent,
	.clk_disable_unused	= omap2_clk_disable_unused,
#ifdef CONFIG_CPU_FREQ
	.clk_init_cpufreq_table = omap2_clk_init_cpufreq_table,
#endif
};

/*
 * Set clocks for bypass mode for reboot to work.
 */
void omap2_clk_prepare_for_reboot(void)
{
	int err = 0;

	/* REVISIT: Not ready for 343x */
#if 0
	u32 rate;

	if (vclk == NULL || sclk == NULL)
		return;

	rate = clk_get_rate(sclk);
	clk_set_rate(vclk, rate);
#endif

	/*
	 * PRCM on OMAP3 will drive SYS_OFFMODE low during DPLL3 warm reset.
	 * This causes Gaia sleep script to execute, usually killing VDD1 and
	 * VDD2 while code is running.  WA is to disable the sleep script
	 * before warm reset.
	 */
#ifdef CONFIG_TWL4030_POWER
	err = twl4030_remove_script(TRITON_SLEEP_SCRIPT);
	if (err)
		pr_err("twl4030: error trying to disable sleep script!\n");
#endif
}

void omap3_lock_dpll5(void)
{
	struct clk *dpll5_clk;
	struct clk *dpll5_m2_clk;

	dpll5_clk = clk_get(NULL, "dpll5_ck");
	clk_set_rate(dpll5_clk, 120000000);
	clk_enable(dpll5_clk);
	/* Enable autoidle to allow it to enter low power bypass */
	omap3_dpll_allow_idle(dpll5_clk);

	/* Program dpll5_m2_clk divider for no division */
	dpll5_m2_clk = clk_get(NULL, "dpll5_m2_ck");
	clk_enable(dpll5_m2_clk);
	clk_set_rate(dpll5_m2_clk, 120000000);
	return;
}

/* REVISIT: Move this init stuff out into clock.c */

/*
 * Switch the MPU rate if specified on cmdline.
 * We cannot do this early until cmdline is parsed.
 */
static int __init omap2_clk_arch_init(void)
{
	if (!mpurate)
		return -EINVAL;

	/* REVISIT: not yet ready for 343x */
#if 0
	if (clk_set_rate(&virt_prcm_set, mpurate))
		printk(KERN_ERR "Could not find matching MPU rate\n");
#endif

	recalculate_root_clocks();

	printk(KERN_INFO "Switched to new clocking rate (Crystal/DPLL3/MPU): "
	       "%ld.%01ld/%ld/%ld MHz\n",
	       (osc_sys_ck.rate / 1000000), (osc_sys_ck.rate / 100000) % 10,
	       (core_ck.rate / 1000000), (dpll1_fck.rate / 1000000)) ;

	return 0;
}
arch_initcall(omap2_clk_arch_init);

int __init omap2_clk_init(void)
{
	/* struct prcm_config *prcm; */
	struct clk **clkp;
	/* u32 clkrate; */
	u32 cpu_clkflg;

	/* REVISIT: Ultimately this will be used for multiboot */
#if 0
	if (cpu_is_omap242x()) {
		cpu_mask = RATE_IN_242X;
		cpu_clkflg = CLOCK_IN_OMAP242X;
		clkp = onchip_24xx_clks;
	} else if (cpu_is_omap2430()) {
		cpu_mask = RATE_IN_243X;
		cpu_clkflg = CLOCK_IN_OMAP243X;
		clkp = onchip_24xx_clks;
	}
#endif
	if (cpu_is_omap34xx()) {
		cpu_mask = RATE_IN_343X;
		cpu_clkflg = CLOCK_IN_OMAP343X;
		clkp = onchip_34xx_clks;

		/*
		 * Update this if there are further clock changes between ES2
		 * and production parts
		 */
		if (omap_rev() == OMAP3430_REV_ES1_0) {
			/* No 3430ES1-only rates exist, so no RATE_IN_3430ES1 */
			cpu_clkflg |= CLOCK_IN_OMAP3430ES1;
		} else {
			cpu_mask |= RATE_IN_3430ES2;
			cpu_clkflg |= CLOCK_IN_OMAP3430ES2;
		}
		if (cpu_is_omap3630()) {
			dpll4_ck.dpll_data->jtype = 1;
			dpll4_dd.mult_mask =
				OMAP3430_PERIPH_DPLL_36XX_MULT_MASK;

			dpll3_m3x2_ck.enable =
				&omap3_pwrdn_bug_clk_enable;
			dpll4_m2x2_ck.enable =
				&omap3_pwrdn_bug_clk_enable;
			dpll4_m3x2_ck.enable =
				&omap3_pwrdn_bug_clk_enable;
			dpll4_m4x2_ck.enable =
				&omap3_pwrdn_bug_clk_enable;
			dpll4_m5x2_ck.enable =
				&omap3_pwrdn_bug_clk_enable;
			dpll4_m6x2_ck.enable =
				&omap3_pwrdn_bug_clk_enable;

			cpu_clkflg |= CLOCK_IN_OMAP363X;
			cpu_mask |= RATE_IN_363X;

			omap_96m_alwon_fck.parent = &omap_192m_alwon_ck;
			omap_96m_alwon_fck.init = &omap2_init_clksel_parent;
			omap_96m_alwon_fck.clksel_reg = CM_CLKSEL;
			omap_96m_alwon_fck.prcm_mod = CORE_MOD;
			omap_96m_alwon_fck.clksel_mask =
				OMAP3630_CLKSEL_96M_MASK;
			omap_96m_alwon_fck.clksel = omap_96m_alwon_fck_clksel;
			omap_96m_alwon_fck.recalc = &omap2_clksel_recalc;
			}
	}
	clk_init(&omap2_clk_functions);

	for (clkp = onchip_34xx_clks;
	     clkp < onchip_34xx_clks + ARRAY_SIZE(onchip_34xx_clks);
	     clkp++) {
		if (cpu_is_omap3630()) {
			if ((*clkp)->flags & CLOCK_IN_OMAP363X)
				(*clkp)->clksel_mask = (*clkp)->clksel_mask2;
		}
		if ((*clkp)->flags & cpu_clkflg)
			clk_register(*clkp);
	}

	/* REVISIT: Not yet ready for OMAP3 */
#if 0
	/* Check the MPU rate set by bootloader */
	clkrate = omap2_get_dpll_rate_24xx(&dpll_ck);
	for (prcm = rate_table; prcm->mpu_speed; prcm++) {
		if (!(prcm->flags & cpu_mask))
			continue;
		if (prcm->xtal_speed != sys_ck.rate)
			continue;
		if (prcm->dpll_speed <= clkrate)
			 break;
	}
	curr_prcm_set = prcm;
#endif

	recalculate_root_clocks();

	printk(KERN_INFO "Clocking rate (Crystal/DPLL/ARM core): "
	       "%ld.%01ld/%ld/%ld MHz\n",
	       (osc_sys_ck.rate / 1000000), (osc_sys_ck.rate / 100000) % 10,
	       (core_ck.rate / 1000000), (arm_fck.rate / 1000000));

	/*
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary
	 */
	clk_enable_init_clocks();

	/*
	 * Lock DPLL5 and put it in autoidle.
	 */
	if (omap_rev() >= OMAP3430_REV_ES2_0)
		omap3_lock_dpll5();

	return 0;
}

#endif /* CONFIG_ARCH_OMAP3 */
