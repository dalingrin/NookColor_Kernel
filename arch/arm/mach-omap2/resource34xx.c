/*
 * linux/arch/arm/mach-omap2/resource34xx.c
 * OMAP3 resource init/change_level/validate_level functions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * History:
 *
 */

#include <linux/pm_qos_params.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <mach/powerdomain.h>
#include <mach/clockdomain.h>
#include <mach/control.h>
#include <mach/omap34xx.h>
#include <mach/omap-pm.h>
#include "smartreflex.h"
#include "resource34xx.h"
#include "pm.h"
#include "cm.h"
#include "cm-regbits-34xx.h"

/**
 * init_latency - Initializes the mpu/core latency resource.
 * @resp: Latency resource to be initalized
 *
 * No return value.
 */
void init_latency(struct shared_resource *resp)
{
	resp->no_of_users = 0;
	resp->curr_level = RES_LATENCY_DEFAULTLEVEL;
	*((u8 *)resp->resource_data) = 0;
	return;
}

/**
 * set_latency - Adds/Updates and removes the CPU_DMA_LATENCY in *pm_qos_params.
 * @resp: resource pointer
 * @latency: target latency to be set
 *
 * Returns 0 on success, or error values as returned by
 * pm_qos_update_requirement/pm_qos_add_requirement.
 */
int set_latency(struct shared_resource *resp, u32 latency)
{
	u8 *pm_qos_req_added;

	if (resp->curr_level == latency)
		return 0;
	else
		/* Update the resources current level */
		resp->curr_level = latency;

	pm_qos_req_added = resp->resource_data;
	if (latency == RES_LATENCY_DEFAULTLEVEL)
		/* No more users left, remove the pm_qos_req if present */
		if (*pm_qos_req_added) {
			pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY,
							resp->name);
			*pm_qos_req_added = 0;
			return 0;
		}

	if (*pm_qos_req_added) {
		return pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
						resp->name, latency);
	} else {
		*pm_qos_req_added = 1;
		return pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
						resp->name, latency);
	}
}

/**
 * init_pd_latency - Initializes the power domain latency resource.
 * @resp: Power Domain Latency resource to be initialized.
 *
 * No return value.
 */
void init_pd_latency(struct shared_resource *resp)
{
	struct pd_latency_db *pd_lat_db;

	resp->no_of_users = 0;
	if (enable_off_mode)
		resp->curr_level = PD_LATENCY_OFF;
	else
		resp->curr_level = PD_LATENCY_RET;
	pd_lat_db = resp->resource_data;
	/* Populate the power domain associated with the latency resource */
	pd_lat_db->pd = pwrdm_lookup(pd_lat_db->pwrdm_name);
	set_pwrdm_state(pd_lat_db->pd, resp->curr_level);
	return;
}

/**
 * set_pd_latency - Updates the curr_level of the power domain resource.
 * @resp: Power domain latency resource.
 * @latency: New latency value acceptable.
 *
 * This function maps the latency in microsecs to the acceptable
 * Power domain state using the latency DB.
 * It then programs the power domain to enter the target state.
 * Always returns 0.
 */
int set_pd_latency(struct shared_resource *resp, u32 latency)
{
	u32 pd_lat_level, ind;
	struct pd_latency_db *pd_lat_db;
	struct powerdomain *pwrdm;

	pd_lat_db = resp->resource_data;
	pwrdm = pd_lat_db->pd;
	pd_lat_level = PD_LATENCY_OFF;
	/* using the latency db map to the appropriate PD state */
	for (ind = 0; ind < PD_LATENCY_MAXLEVEL; ind++) {
		if (pd_lat_db->latency[ind] < latency) {
			pd_lat_level = ind;
			break;
		}
	}

	if (!enable_off_mode && pd_lat_level == PD_LATENCY_OFF)
		pd_lat_level = PD_LATENCY_RET;

	resp->curr_level = pd_lat_level;
	set_pwrdm_state(pwrdm, pd_lat_level);
	return 0;
}

static struct shared_resource *vdd1_resp;
static struct shared_resource *vdd2_resp;
static struct device dummy_mpu_dev;
static struct device dummy_dsp_dev;
static struct device vdd2_dev;
static int vdd1_lock;
static int vdd2_lock;
static struct clk *dpll1_clk, *dpll2_clk, *dpll3_clk;
static int curr_vdd1_opp;
static int curr_vdd2_opp;
static DEFINE_MUTEX(dvfs_mutex);

unsigned short get_opp_id(struct omap_opp *opp_freq_table,
		unsigned long freq)
{
	struct omap_opp *prcm_config;
	prcm_config = opp_freq_table;

	/*
	 * If the highest OPP has lower freq compared to other OPP's
	 * in the table, than the logic fails here.
	 * Fixing it with proper check ..
	 */
	if (prcm_config->rate == freq)
		return prcm_config->opp_id; /* Return the Highest OPP */
	else
		prcm_config--;
	for (; prcm_config->rate; prcm_config--)
		if (prcm_config->rate < freq)
			return (prcm_config+1)->opp_id;
		else if (prcm_config->rate == freq)
			return prcm_config->opp_id;
	/* Return the least OPP */
	return (prcm_config+1)->opp_id;
}

/**
 * init_opp - Initialize the OPP resource
 */
void init_opp(struct shared_resource *resp)
{
	struct clk *l3_clk;
	resp->no_of_users = 0;

	if (!mpu_opps || !dsp_opps || !l3_opps)
		return;

	/* Initialize the current level of the OPP resource
	* to the  opp set by u-boot.
	*/
	if (strcmp(resp->name, "vdd1_opp") == 0) {
		vdd1_resp = resp;
		dpll1_clk = clk_get(NULL, "dpll1_ck");
		dpll2_clk = clk_get(NULL, "dpll2_ck");
		resp->curr_level = get_opp_id(mpu_opps + MAX_VDD1_OPP,
				dpll1_clk->rate);
		curr_vdd1_opp = resp->curr_level;
	} else if (strcmp(resp->name, "vdd2_opp") == 0) {
		vdd2_resp = resp;
		dpll3_clk = clk_get(NULL, "dpll3_m2_ck");
		l3_clk = clk_get(NULL, "l3_ick");
		resp->curr_level = get_opp_id(l3_opps + MAX_VDD2_OPP,
				l3_clk->rate);
		curr_vdd2_opp = resp->curr_level;
	}
	return;
}

int resource_access_opp_lock(int res, int delta)
{
	if (res == VDD1_OPP) {
		vdd1_lock += delta;
		return vdd1_lock;
	} else if (res == VDD2_OPP) {
		vdd2_lock += delta;
		return vdd2_lock;
	}
	return -EINVAL;
}

#ifndef CONFIG_CPU_FREQ
static unsigned long compute_lpj(unsigned long ref, u_int div, u_int mult)
{
	unsigned long new_jiffy_l, new_jiffy_h;

	/*
	 * Recalculate loops_per_jiffy.  We do it this way to
	 * avoid math overflow on 32-bit machines.  Maybe we
	 * should make this architecture dependent?  If you have
	 * a better way of doing this, please replace!
	 *
	 *    new = old * mult / div
	 */
	new_jiffy_h = ref / div;
	new_jiffy_l = (ref % div) / 100;
	new_jiffy_h *= mult;
	new_jiffy_l = new_jiffy_l * mult / div;

	return new_jiffy_h + new_jiffy_l * 100;
}
#endif

static int program_opp_freq(int res, int target_level, int current_level)
{
	int ret = 0, l3_div;
	int *curr_opp;
	u32 cm_clksel1_mpu;

	lock_scratchpad_sem();
	if (res == VDD1_OPP) {
		if (target_level == VDD1_OPP1) {
			cm_clksel1_mpu = cm_read_mod_reg(MPU_MOD, CM_CLKSEL1);
			if (cpu_is_omap3630())
				cm_clksel1_mpu = (cm_clksel1_mpu & ~(OMAP3430_MPU_CLK_SRC_MASK)) |
							(0x2 << OMAP3430_MPU_CLK_SRC_SHIFT);
			else if (cpu_is_omap34xx())
				cm_clksel1_mpu = (cm_clksel1_mpu & ~(OMAP3430_MPU_CLK_SRC_MASK)) |
							(0x4 << OMAP3430_MPU_CLK_SRC_SHIFT);
			cm_write_mod_reg(cm_clksel1_mpu, MPU_MOD, CM_CLKSEL1);
		} else if ((current_level == VDD1_OPP1) && (target_level != VDD1_OPP1)) {
			cm_clksel1_mpu = cm_read_mod_reg(MPU_MOD, CM_CLKSEL1);
			if (cpu_is_omap3630())
				cm_clksel1_mpu = (cm_clksel1_mpu & ~(OMAP3430_MPU_CLK_SRC_MASK)) |
							(0x1 << OMAP3430_MPU_CLK_SRC_SHIFT);
			else if (cpu_is_omap34xx())
				cm_clksel1_mpu = (cm_clksel1_mpu & ~(OMAP3430_MPU_CLK_SRC_MASK)) |
							(0x2 << OMAP3430_MPU_CLK_SRC_SHIFT);
			cm_write_mod_reg(cm_clksel1_mpu, MPU_MOD, CM_CLKSEL1);
		}
		curr_opp = &curr_vdd1_opp;
		clk_set_rate(dpll1_clk, mpu_opps[target_level].rate);
		clk_set_rate(dpll2_clk, dsp_opps[target_level].rate);
#ifndef CONFIG_CPU_FREQ
		/*Update loops_per_jiffy if processor speed is being changed*/
		loops_per_jiffy = compute_lpj(loops_per_jiffy,
			mpu_opps[current_level].rate/1000,
			mpu_opps[target_level].rate/1000);
#endif
	} else {
		curr_opp = &curr_vdd2_opp;
		l3_div = cm_read_mod_reg(CORE_MOD, CM_CLKSEL) &
			OMAP3430_CLKSEL_L3_MASK;
		ret = clk_set_rate(dpll3_clk,
				l3_opps[target_level].rate * l3_div);
	}
	if (ret) {
		unlock_scratchpad_sem();
		return current_level;
	}
#ifdef CONFIG_PM
	omap3_save_scratchpad_contents();
#endif
	unlock_scratchpad_sem();

	*curr_opp = target_level;
	return target_level;
}

#ifdef CONFIG_OMAP_SMARTREFLEX_CLASS1P5
u8 sr_class1p5 = 1;
#else
/* use default of 0 */
u8 sr_class1p5;
#endif

static int program_opp(int res, struct omap_opp *opp, int target_level,
		int current_level)
{
	int i, ret = 0, raise;
	unsigned long t_opp, c_opp;
	u8 target_v, current_v;

	t_opp = ID_VDD(res) | ID_OPP_NO(opp[target_level].opp_id);
	c_opp = ID_VDD(res) | ID_OPP_NO(opp[current_level].opp_id);
	/* Sanity check of the OPP params before attempting to set */
	if (!opp[target_level].rate || !opp[target_level].vsel)
		return -EINVAL;
	if (target_level > current_level)
		raise = 1;
	else
		raise = 0;

	/* Start with nominal voltage */
	target_v = opp[target_level].vsel;
	current_v = opp[current_level].vsel;
	if (!sr_class1p5) {
		disable_smartreflex(res);
	} else {
		/* if use class 1.5, decide on which voltage to use */
		target_v = (opp[target_level].sr_adjust_vsel) ?
			opp[target_level].sr_vsr_step_vsel : target_v;
		current_v = (opp[current_level].sr_adjust_vsel) ?
			opp[current_level].sr_vsr_step_vsel : current_v;
	}

	for (i = 0; i < 2; i++) {
		if (i == raise)
			ret = program_opp_freq(res, target_level,
					current_level);
		else {
			omap_scale_voltage(t_opp, c_opp,
				target_v, current_v);
		}
	}

	if (!sr_class1p5)
		enable_smartreflex(res);
	else if (!opp[target_level].sr_adjust_vsel)
		sr_recalibrate(res, t_opp, c_opp);

	return ret;
}

int resource_set_opp_level(int res, u32 target_level, int flags)
{
	unsigned long mpu_freq, mpu_old_freq;
#ifdef CONFIG_CPU_FREQ
	struct cpufreq_freqs freqs_notify;
#endif
	struct shared_resource *resp;

	if (res == VDD1_OPP)
		resp = vdd1_resp;
	else if (res == VDD2_OPP)
		resp = vdd2_resp;
	else
		return 0;

	if (resp->curr_level == target_level)
		return 0;

	if (!mpu_opps || !dsp_opps || !l3_opps)
		return 0;

	mutex_lock(&dvfs_mutex);

	if (res == VDD1_OPP) {
		if (flags != OPP_IGNORE_LOCK && vdd1_lock) {
			mutex_unlock(&dvfs_mutex);
			return 0;
		}
		mpu_old_freq = mpu_opps[resp->curr_level].rate;
		mpu_freq = mpu_opps[target_level].rate;

#ifdef CONFIG_CPU_FREQ
		freqs_notify.old = mpu_old_freq/1000;
		freqs_notify.new = mpu_freq/1000;
		freqs_notify.cpu = 0;
		/* Send pre notification to CPUFreq */
		cpufreq_notify_transition(&freqs_notify, CPUFREQ_PRECHANGE);
#endif
		resp->curr_level = program_opp(res, mpu_opps, target_level,
			resp->curr_level);
#ifdef CONFIG_CPU_FREQ
		/* Send a post notification to CPUFreq */
		cpufreq_notify_transition(&freqs_notify, CPUFREQ_POSTCHANGE);
#endif
	} else {
		if (!(flags & OPP_IGNORE_LOCK) && vdd2_lock) {
			mutex_unlock(&dvfs_mutex);
			return 0;
		}
		resp->curr_level = program_opp(res, l3_opps, target_level,
			resp->curr_level);
	}
	mutex_unlock(&dvfs_mutex);
	return 0;
}

int set_opp(struct shared_resource *resp, u32 target_level)
{
	unsigned long tput;
	unsigned long req_l3_freq;
	int ind;

	if (resp == vdd1_resp) {
		if (target_level < VDD1_THRESHOLD)
			resource_release("vdd2_opp", &vdd2_dev);

		resource_set_opp_level(VDD1_OPP, target_level, 0);
		/*
		 * For VDD1 OPP3 and above, make sure the interconnect
		 * is strictly above 100Mhz.
		 * throughput in KiB/s for 200 Mhz = 200 * 1000 * 4.
		 */
		if (target_level >= VDD1_THRESHOLD)
			resource_request("vdd2_opp", &vdd2_dev,
				4 * l3_opps[omap_pm_get_max_vdd2_opp()].rate/1000);

	} else if (resp == vdd2_resp) {
		tput = target_level;

		/* Convert the tput in KiB/s to Bus frequency in MHz */
		req_l3_freq = (tput * 1000)/4;

		for (ind = MIN_VDD2_OPP; ind <= MAX_VDD2_OPP; ind++)
			if ((l3_opps + ind)->rate >= req_l3_freq) {
				target_level = ind;
				break;
			}

		/* Set the highest OPP possible */
		if (ind > MAX_VDD2_OPP)
			target_level = ind-1;
		resource_set_opp_level(VDD2_OPP, target_level, 0);
	}
	return 0;
}

/**
 * validate_opp - Validates if valid VDD1 OPP's are passed as the
 * target_level.
 * VDD2 OPP levels are passed as L3 throughput, which are then mapped
 * to an appropriate OPP.
 */
int validate_opp(struct shared_resource *resp, u32 target_level)
{
	return 0;
}

/**
 * init_freq - Initialize the frequency resource.
 */
void init_freq(struct shared_resource *resp)
{
	char *linked_res_name;
	resp->no_of_users = 0;

	if (!mpu_opps || !dsp_opps)
		return;

	linked_res_name = (char *)resp->resource_data;
	/* Initialize the current level of the Freq resource
	* to the frequency set by u-boot.
	*/
	if (strcmp(resp->name, "mpu_freq") == 0)
		/* MPU freq in Mhz */
		resp->curr_level = mpu_opps[curr_vdd1_opp].rate;
	else if (strcmp(resp->name, "dsp_freq") == 0)
		/* DSP freq in Mhz */
		resp->curr_level = dsp_opps[curr_vdd1_opp].rate;
	return;
}

int set_freq(struct shared_resource *resp, u32 target_level)
{
	unsigned int vdd1_opp;

	if (!mpu_opps || !dsp_opps)
		return 0;

	if (strcmp(resp->name, "mpu_freq") == 0) {
		/*
		 * For 3630 OPP3 and OPP4 has same ARM MHz but different
		 * DSP MHz. So whenever
		 * ARM side request is for OPP3/4 give OPP3.
		 */
		if (cpu_is_omap3630() && (target_level
					== mpu_opps[VDD1_OPP3].rate))
			vdd1_opp = VDD1_OPP3;
		else
			vdd1_opp = get_opp_id(mpu_opps + MAX_VDD1_OPP,
						 target_level);

		resource_request("vdd1_opp", &dummy_mpu_dev, vdd1_opp);
	} else if (strcmp(resp->name, "dsp_freq") == 0) {
		vdd1_opp = get_opp_id(dsp_opps + MAX_VDD1_OPP, target_level);
		resource_request("vdd1_opp", &dummy_dsp_dev, vdd1_opp);
	}
	resp->curr_level = target_level;
	return 0;
}

int validate_freq(struct shared_resource *resp, u32 target_level)
{
	return 0;
}

static u8 vdd2_save_opp;
int set_dpll3_volt_freq(bool dpll3_restore)
{
	struct shared_resource *resp = vdd2_resp;
	int ret = 0, l3_div;
	u8 min_opp = 0, max_opp = 0, t_vsel = 0, c_vsel = 0;
	unsigned long t_opp, c_opp;

	if (cpu_is_omap3430()) {
		min_opp = omap_pm_get_min_vdd2_opp();
		max_opp = omap_pm_get_max_vdd2_opp();
	} else if (cpu_is_omap3630()) {
		min_opp = omap_pm_get_min_vdd2_opp();
		max_opp = omap_pm_get_max_vdd2_opp();
	}

	if (!dpll3_restore) {
		t_opp = ID_VDD(VDD2_OPP) | ID_OPP_NO(l3_opps[max_opp].opp_id);
		c_opp = ID_VDD(VDD2_OPP) | ID_OPP_NO(l3_opps[resp->curr_level].opp_id);

		if (cpu_is_omap3430()) {
			/*
			 * change the vol to 1.2. As this volt is not in the
			 * table need to calculate here.
			 * As per TRM: vsel = (volt - 0.6) / 0.0125
			 * here Volt=1.2V
			 */
			t_vsel = 30;
			vdd2_save_opp = resp->curr_level;
		} else if (cpu_is_omap3630()) {
			t_vsel = l3_opps[max_opp].vsel;
			vdd2_save_opp = resp->curr_level;
		}

		c_vsel = l3_opps[resp->curr_level].vsel;
	} else {
		/* restore to the saved opp */
		t_opp = ID_VDD(VDD2_OPP) | ID_OPP_NO(l3_opps[vdd2_save_opp].opp_id);
		c_opp = ID_VDD(VDD2_OPP) | ID_OPP_NO(l3_opps[min_opp].opp_id);
		t_vsel = l3_opps[vdd2_save_opp].vsel;
		c_vsel = l3_opps[vdd2_save_opp].vsel;
		resp->curr_level = vdd2_save_opp;
	}

	if (resp->curr_level == min_opp) {
		/* change the voltage only */
		omap_scale_voltage(t_opp, c_opp,
					t_vsel, c_vsel);

	} else {
		/* step1: change the freq */
		lock_scratchpad_sem();
		l3_div = cm_read_mod_reg(CORE_MOD, CM_CLKSEL) &
			OMAP3430_CLKSEL_L3_MASK;
		if (!dpll3_restore) {

			ret = clk_set_rate(dpll3_clk,
					l3_opps[min_opp].rate * l3_div);
			resp->curr_level = min_opp;

		} else {
			ret = clk_set_rate(dpll3_clk,
					l3_opps[vdd2_save_opp].rate * l3_div);
			resp->curr_level = vdd2_save_opp;
		}

#ifdef CONFIG_PM
		omap3_save_scratchpad_contents();
#endif
		unlock_scratchpad_sem();

		/* step 2: change the volt to 1.2v for 3430 and retain the same for 3630 */
		if (cpu_is_omap3430())
			/* change the voltage to recommended value */
			omap_scale_voltage(t_opp, c_opp,
						t_vsel, c_vsel);
	}

	return ret;
}
