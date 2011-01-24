/*
 * linux/arch/arm/mach-omap2/cpuidle34xx.c
 *
 * OMAP3 CPU IDLE Routines
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <mach/prcm.h>
#include <mach/powerdomain.h>
#include <mach/clockdomain.h>
#include <mach/control.h>
#include <mach/serial.h>
#include <mach/irqs.h>

#include "pm.h"

#ifdef CONFIG_CPU_IDLE

#define OMAP3_MAX_STATES 9
#define OMAP3_STATE_C1 0 /* C1 - MPU WFI + Core active */
#define OMAP3_STATE_C2 1 /* C2 - MPU inactive + Core inactive */
#define OMAP3_STATE_C3 2 /* C3 - MPU CSWR + Core inactive */
#define OMAP3_STATE_C4 3 /* C4 - MPU OFF + Core inactive */
#define OMAP3_STATE_C5 4 /* C5 - MPU CSWR + Core CSWR */
#define OMAP3_STATE_C6 5 /* C6 - MPU OFF + Core CSWR */
#define OMAP3_STATE_C7 6 /* C7 - MPU OSWR + CORE OSWR */
#define OMAP3_STATE_C8 7 /* C8 - MPU OFF + CORE OSWR */
#define OMAP3_STATE_C9 8 /* C9 - MPU OFF + CORE OFF */


struct omap3_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 mpu_state;
	u32 core_state;
	u32 mpu_logicl1_ret_state;
	u32 mpu_l2cache_ret_state;
	u32 core_logic_state;
	u32 core_mem1_ret_state;
	u32 core_mem2_ret_state;
	u32 threshold;
	u32 flags;
};

struct omap3_processor_cx omap3_power_states[OMAP3_MAX_STATES];
struct omap3_processor_cx current_cx_state;
struct powerdomain *mpu_pd, *core_pd, *per_pd, *iva2_pd;
struct powerdomain *sgx_pd, *usb_pd, *cam_pd, *dss_pd;

static int omap3_idle_bm_check(void)
{
	if (!omap3_can_sleep())
		return 1;
	return 0;
}

static int _cpuidle_allow_idle(struct powerdomain *pwrdm,
				struct clockdomain *clkdm)
{
	omap2_clkdm_allow_idle(clkdm);
	return 0;
}

static int _cpuidle_deny_idle(struct powerdomain *pwrdm,
				struct clockdomain *clkdm)
{
	omap2_clkdm_deny_idle(clkdm);
	return 0;
}

static int pwrdm_get_idle_state(struct powerdomain *pwrdm)
{
	if (pwrdm_can_idle(pwrdm))
		return pwrdm_read_next_pwrst(pwrdm);
	return PWRDM_POWER_ON;
}


/**
 * omap3_enter_idle - Programs OMAP3 to enter the specified state
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static int omap3_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct omap3_processor_cx *cx = cpuidle_get_statedata(state);
	struct timespec ts_preidle, ts_postidle, ts_idle;
	u32 mpu_state = cx->mpu_state, core_state = cx->core_state;
	u32 mpu_logicl1_ret_state = cx->mpu_logicl1_ret_state;
	u32 mpu_l2cache_ret_state = cx->mpu_l2cache_ret_state;
	u32 core_logic_state = cx->core_logic_state;
	u32 core_mem1_ret_state = cx->core_mem1_ret_state;
	u32 core_mem2_ret_state = cx->core_mem2_ret_state;
	u32 saved_mpu_state;
	current_cx_state = *cx;

	/* Used to keep track of the total time in idle */
	getnstimeofday(&ts_preidle);

	local_irq_disable();
	local_fiq_disable();


	if (!enable_oswr_ret) {
		if (mpu_logicl1_ret_state == PWRDM_POWER_OFF)
			mpu_logicl1_ret_state = PWRDM_POWER_RET;
		if (mpu_l2cache_ret_state == PWRDM_POWER_OFF)
			mpu_l2cache_ret_state = PWRDM_POWER_RET;
		if (core_logic_state == PWRDM_POWER_OFF)
			core_logic_state = PWRDM_POWER_RET;
		if (core_mem1_ret_state == PWRDM_POWER_OFF)
			core_mem1_ret_state = PWRDM_POWER_RET;
		if (core_mem2_ret_state == PWRDM_POWER_OFF)
			core_mem2_ret_state = PWRDM_POWER_RET;
	}

	if (mpu_logicl1_ret_state != 0xFF)
		pwrdm_set_logic_retst(mpu_pd, mpu_logicl1_ret_state);

	if (mpu_l2cache_ret_state != 0xFF)
		pwrdm_set_mem_retst(mpu_pd, 0, mpu_l2cache_ret_state);

	if (core_logic_state != 0xFF)
		pwrdm_set_logic_retst(core_pd, core_logic_state);

	if (core_mem1_ret_state != 0xFF)
		pwrdm_set_mem_retst(core_pd, 0, core_mem1_ret_state);

	if (core_mem2_ret_state != 0xFF)
		pwrdm_set_mem_retst(core_pd, 1, core_mem2_ret_state);

	if (omap_irq_pending() || need_resched())
		goto return_sleep_time;

	saved_mpu_state = pwrdm_read_next_pwrst(mpu_pd);
	pwrdm_set_next_pwrst(mpu_pd, mpu_state);
	pwrdm_set_next_pwrst(core_pd, core_state);

	if (cx->type == OMAP3_STATE_C1) {
		pwrdm_for_each_clkdm(mpu_pd, _cpuidle_deny_idle);
		pwrdm_for_each_clkdm(core_pd, _cpuidle_deny_idle);
	}

	/* Execute ARM wfi */
	omap_sram_idle();

	if (cx->type == OMAP3_STATE_C1) {
		pwrdm_for_each_clkdm(mpu_pd, _cpuidle_allow_idle);
		pwrdm_for_each_clkdm(core_pd, _cpuidle_allow_idle);
	}

	pwrdm_set_next_pwrst(mpu_pd, saved_mpu_state);

return_sleep_time:
	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	local_irq_enable();
	local_fiq_enable();

	return ts_idle.tv_nsec/NSEC_PER_USEC + ts_idle.tv_sec * USEC_PER_SEC;
}
/**
 * omap3_enter_idle_bm - Checks for any bus activity
 * @dev: cpuidle device
 * @state: The target state to be programmed
 *
 * Called from the CPUidle framework for C states with CPUIDLE_FLAG_CHECK_BM
 * flag set. This function checks for any pending bus activity and then
 * programs the device to the specified or a lower possible state
 */
static int omap3_enter_idle_bm(struct cpuidle_device *dev,
			       struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = state;

	u32 per_state = 0, saved_per_state = 0, cam_state, usb_state;
	u32 iva2_state, sgx_state, dss_state, new_core_state, new_mpu_state;
	struct omap3_processor_cx *cx;
	int ret;

	if (state->flags & CPUIDLE_FLAG_CHECK_BM) {
		if (omap3_idle_bm_check()) {
			BUG_ON(!dev->safe_state);
			new_state = dev->safe_state;
			goto select_state;
		}
		cx = cpuidle_get_statedata(state);

		if (!enable_off_idle_path) {
			if (cx->core_state == PWRDM_POWER_OFF) {
				state--;
				cx = cpuidle_get_statedata(state);
			}
		}

		new_core_state = cx->core_state;

		/* Check if CORE is active, if yes, fallback to inactive */
		if (!pwrdm_can_idle(core_pd))
			new_core_state = PWRDM_POWER_INACTIVE;

		/*
		 * Prevent idle completely if CAM is active.
		 * CAM does not have wakeup capability in OMAP3.
		 */
		cam_state = pwrdm_get_idle_state(cam_pd);
		if (cam_state == PWRDM_POWER_ON) {
			new_state = dev->safe_state;
			goto select_state;
		}

		/*
		 * Check if PER can idle or not. If we are not likely
		 * to idle, deny PER off. This prevents unnecessary
		 * context save/restore.
		 */
		saved_per_state = pwrdm_read_next_pwrst(per_pd);
		if (pwrdm_can_idle(per_pd)) {
			per_state = saved_per_state;
			/*
			 * Prevent PER off if CORE is active as this
			 * would disable PER wakeups completely
			 */
			if (per_state == PWRDM_POWER_OFF &&
			    new_core_state > PWRDM_POWER_RET)
				per_state = PWRDM_POWER_RET;

		} else if (saved_per_state == PWRDM_POWER_OFF)
			per_state = PWRDM_POWER_RET;

		/*
		 * If we are attempting CORE off, check if any other
		 * powerdomains are at retention or higher. CORE off causes
		 * chipwide reset which would reset these domains also.
		 */
    
		if (new_core_state == PWRDM_POWER_OFF) {
			dss_state = pwrdm_get_idle_state(dss_pd);
			iva2_state = pwrdm_get_idle_state(iva2_pd);
			sgx_state = pwrdm_get_idle_state(sgx_pd);
			usb_state = pwrdm_get_idle_state(usb_pd);
      
			if (cam_state > PWRDM_POWER_OFF ||
			    dss_state > PWRDM_POWER_OFF ||
			    iva2_state > PWRDM_POWER_OFF ||
			    per_state > PWRDM_POWER_OFF ||
			    sgx_state > PWRDM_POWER_OFF ||
			    usb_state > PWRDM_POWER_OFF)
				new_core_state = PWRDM_POWER_RET;
		}
    
                /* WA for SGX performance issue  */
                /* Avoid having the MPU PWDM going to OFF while the SGX is active */
                /* Avoid having the CORE PWDM to go in RET or OFF, allow in fact only C1,C2,C3 */

                sgx_state = pwrdm_get_idle_state(sgx_pd);
                new_mpu_state = cx->mpu_state;

                if (sgx_state == PWRDM_POWER_ON)
                {
                  if (cx->mpu_state < PWRDM_POWER_RET)
                    new_mpu_state = PWRDM_POWER_RET;
                  new_core_state = PWRDM_POWER_INACTIVE;
                }
                
		/* Fallback to new target core state */
		while ((cx->core_state < new_core_state) || (cx->mpu_state < new_mpu_state)) {
			state--;
			cx = cpuidle_get_statedata(state);
		}
		new_state = state;
		/* Are we changing PER target state? */
		if (per_state != saved_per_state)
			pwrdm_set_next_pwrst(per_pd, per_state);

	}
select_state:
	dev->last_state = new_state;
	ret = omap3_enter_idle(dev, new_state);
	/* Restore potentially tampered PER state */
	if (per_state != saved_per_state)
		pwrdm_set_next_pwrst(per_pd, saved_per_state);
	return ret;
}

DEFINE_PER_CPU(struct cpuidle_device, omap3_idle_dev);

/* omap3_init_power_states - Initialises the OMAP3 specific C states.
 *
 * Below is the desciption of each C state.
 * 	C1 . MPU WFI + Core active
 *	C2 . MPU inactive + Core inactive
 *	C3 . MPU CSWR + Core inactive
 *	C4 . MPU OFF + Core inactive
 *	C5 . MPU CSWR + Core CSWR
 *	C6 . MPU OFF + Core CSWR
 *	C7 . MPU OSWR + Core OSWR
 *	C8 . MPU OFF + Core OSWR
 *	C9 . MPU OFF + Core OFF
 */
void omap_init_power_states(void)
{
	int i;
	struct omap3_processor_cx *cx;

	for (i = OMAP3_STATE_C1; i < OMAP3_MAX_STATES; i++) {
		cx = &omap3_power_states[i];
		cx->mpu_logicl1_ret_state = 0xFF;
		cx->mpu_l2cache_ret_state = 0xFF;
		cx->core_logic_state = 0xFF;
		cx->core_mem1_ret_state = 0xFF;
		cx->core_mem2_ret_state = 0xFF;
	}

	/* C1 . MPU WFI + Core active */
	omap3_power_states[OMAP3_STATE_C1].valid = 1;
	omap3_power_states[OMAP3_STATE_C1].type = OMAP3_STATE_C1;
	omap3_power_states[OMAP3_STATE_C1].sleep_latency = 0;
	omap3_power_states[OMAP3_STATE_C1].wakeup_latency = 12;
	omap3_power_states[OMAP3_STATE_C1].threshold = 15;
	omap3_power_states[OMAP3_STATE_C1].mpu_state = PWRDM_POWER_INACTIVE;
	omap3_power_states[OMAP3_STATE_C1].core_state = PWRDM_POWER_ON;
	omap3_power_states[OMAP3_STATE_C1].flags = CPUIDLE_FLAG_TIME_VALID;

	/* C2 . MPU WFI + Core inactive */
	omap3_power_states[OMAP3_STATE_C2].valid = 1;
	omap3_power_states[OMAP3_STATE_C2].type = OMAP3_STATE_C2;
	omap3_power_states[OMAP3_STATE_C2].sleep_latency = 0;
	omap3_power_states[OMAP3_STATE_C2].wakeup_latency = 18;
	omap3_power_states[OMAP3_STATE_C2].threshold = 20;
	omap3_power_states[OMAP3_STATE_C2].mpu_state = PWRDM_POWER_INACTIVE;
	omap3_power_states[OMAP3_STATE_C2].core_state = PWRDM_POWER_INACTIVE;
	omap3_power_states[OMAP3_STATE_C2].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C3 . MPU CSWR + Core inactive */
	omap3_power_states[OMAP3_STATE_C3].valid = 1;
	omap3_power_states[OMAP3_STATE_C3].type = OMAP3_STATE_C3;
	omap3_power_states[OMAP3_STATE_C3].sleep_latency = 150;
	omap3_power_states[OMAP3_STATE_C3].wakeup_latency = 260;
	omap3_power_states[OMAP3_STATE_C3].threshold = 500;
#ifdef CONFIG_OMAP3_MPU_L2_CACHE_WORKAROUND
	omap3_power_states[OMAP3_STATE_C3].mpu_state = PWRDM_POWER_INACTIVE;
#else
	omap3_power_states[OMAP3_STATE_C3].mpu_state = PWRDM_POWER_RET;
#endif
	omap3_power_states[OMAP3_STATE_C3].core_state = PWRDM_POWER_INACTIVE;
	omap3_power_states[OMAP3_STATE_C3].mpu_logicl1_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C3].mpu_l2cache_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C3].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C4 . MPU OFF + Core inactive */
	omap3_power_states[OMAP3_STATE_C4].valid = 1;
	omap3_power_states[OMAP3_STATE_C4].type = OMAP3_STATE_C4;
	omap3_power_states[OMAP3_STATE_C4].sleep_latency = 1600;
	omap3_power_states[OMAP3_STATE_C4].wakeup_latency = 1850;
	omap3_power_states[OMAP3_STATE_C4].threshold = 4000;
	omap3_power_states[OMAP3_STATE_C4].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C4].core_state = PWRDM_POWER_INACTIVE;
	omap3_power_states[OMAP3_STATE_C4].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C5 . MPU CSWR + Core CSWR*/
	omap3_power_states[OMAP3_STATE_C5].valid = 1;
	omap3_power_states[OMAP3_STATE_C5].type = OMAP3_STATE_C5;
	omap3_power_states[OMAP3_STATE_C5].sleep_latency = 310;
	omap3_power_states[OMAP3_STATE_C5].wakeup_latency = 2850;
	omap3_power_states[OMAP3_STATE_C5].threshold = 5000;
#ifdef CONFIG_OMAP3_MPU_L2_CACHE_WORKAROUND
	omap3_power_states[OMAP3_STATE_C5].mpu_state = PWRDM_POWER_INACTIVE;
#else
	omap3_power_states[OMAP3_STATE_C5].mpu_state = PWRDM_POWER_RET;
#endif
	omap3_power_states[OMAP3_STATE_C5].core_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].mpu_logicl1_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].mpu_l2cache_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].core_logic_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].core_mem1_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].core_mem2_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C5].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C6 . MPU OFF + Core CSWR */
	omap3_power_states[OMAP3_STATE_C6].valid = 1;
	omap3_power_states[OMAP3_STATE_C6].type = OMAP3_STATE_C6;
	omap3_power_states[OMAP3_STATE_C6].sleep_latency = 1800;
	omap3_power_states[OMAP3_STATE_C6].wakeup_latency = 4450;
	omap3_power_states[OMAP3_STATE_C6].threshold = 10000;
	omap3_power_states[OMAP3_STATE_C6].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C6].core_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C6].core_logic_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C6].core_mem1_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C6].core_mem2_ret_state =
				PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C6].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C7 . MPU OSWR + Core OSWR */
	omap3_power_states[OMAP3_STATE_C7].valid = 1;
	omap3_power_states[OMAP3_STATE_C7].type = OMAP3_STATE_C7;
	omap3_power_states[OMAP3_STATE_C7].sleep_latency = 4000;
	omap3_power_states[OMAP3_STATE_C7].wakeup_latency = 9000;
	omap3_power_states[OMAP3_STATE_C7].threshold = 18000;
	omap3_power_states[OMAP3_STATE_C7].mpu_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C7].core_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C7].mpu_logicl1_ret_state =
				PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C7].mpu_l2cache_ret_state =
				PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C7].core_logic_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C7].core_mem1_ret_state =
				PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C7].core_mem2_ret_state =
				PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C7].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C8 . MPU OFF + Core OSWR */
	omap3_power_states[OMAP3_STATE_C8].valid = 1;
	omap3_power_states[OMAP3_STATE_C8].type = OMAP3_STATE_C7;
	omap3_power_states[OMAP3_STATE_C8].sleep_latency = 8000;
	omap3_power_states[OMAP3_STATE_C8].wakeup_latency = 25000;
	omap3_power_states[OMAP3_STATE_C8].threshold = 250000;
	omap3_power_states[OMAP3_STATE_C8].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C8].core_state = PWRDM_POWER_RET;
	omap3_power_states[OMAP3_STATE_C8].core_logic_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C8].core_mem1_ret_state =
				PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C8].core_mem2_ret_state =
				PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C8].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

	/* C9 . MPU OFF + Core OFF */
	omap3_power_states[OMAP3_STATE_C9].valid = 1;
	omap3_power_states[OMAP3_STATE_C9].type = OMAP3_STATE_C7;
	omap3_power_states[OMAP3_STATE_C9].sleep_latency = 10000;
	omap3_power_states[OMAP3_STATE_C9].wakeup_latency = 30000;
	omap3_power_states[OMAP3_STATE_C9].threshold = 300000;
	omap3_power_states[OMAP3_STATE_C9].mpu_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C9].core_state = PWRDM_POWER_OFF;
	omap3_power_states[OMAP3_STATE_C9].flags = CPUIDLE_FLAG_TIME_VALID |
				CPUIDLE_FLAG_CHECK_BM;

}
/**
 * omap3_cpuidle_update_states - Update the cpuidle states.
 *
 * Currently, this function toggles the validity of idle states based upon
 * the flag 'enable_off_mode'. When the flag is set all states are valid.
 * Else, states leading to OFF state set to be invalid.
 */
void omap3_cpuidle_update_states(void)
{
	int i;

	for (i = OMAP3_STATE_C1; i < OMAP3_MAX_STATES; i++) {
		if (enable_off_mode) {
			omap3_power_states[i].valid = 1;
		} else {
			if ((omap3_power_states[i].mpu_state
					== PWRDM_POWER_OFF)
			|| (omap3_power_states[i].core_state
					== PWRDM_POWER_RET))
				omap3_power_states[i].valid = 0;
			}
		}
}



struct cpuidle_driver omap3_idle_driver = {
	.name = 	"omap3_idle",
	.owner = 	THIS_MODULE,
};

/**
 * omap3_idle_init - Init routine for OMAP3 idle
 *
 * Registers the OMAP3 specific cpuidle driver with the cpuidle
 * framework with the valid set of states.
 */
int omap3_idle_init(void)
{
	int i, count = 0;
	struct omap3_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;

	mpu_pd = pwrdm_lookup("mpu_pwrdm");
	core_pd = pwrdm_lookup("core_pwrdm");
	per_pd = pwrdm_lookup("per_pwrdm");
	iva2_pd = pwrdm_lookup("iva2_pwrdm");
	sgx_pd = pwrdm_lookup("sgx_pwrdm");
	usb_pd = pwrdm_lookup("usbhost_pwrdm");
	cam_pd = pwrdm_lookup("cam_pwrdm");
	dss_pd = pwrdm_lookup("dss_pwrdm");

	omap_init_power_states();
	cpuidle_register_driver(&omap3_idle_driver);

	dev = &per_cpu(omap3_idle_dev, smp_processor_id());

	for (i = OMAP3_STATE_C1; i < OMAP3_MAX_STATES; i++) {
		cx = &omap3_power_states[i];
		state = &dev->states[count];

		if (!cx->valid)
			continue;
		cpuidle_set_statedata(state, cx);
		state->exit_latency = cx->sleep_latency + cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		state->enter = (state->flags & CPUIDLE_FLAG_CHECK_BM) ?
			omap3_enter_idle_bm : omap3_enter_idle;
		if (cx->type == OMAP3_STATE_C1)
			dev->safe_state = state;
		sprintf(state->name, "C%d", count+1);
		count++;
	}

	if (!count)
		return -EINVAL;
	dev->state_count = count;

	omap3_cpuidle_update_states();

	if (cpuidle_register_device(dev)) {
		printk(KERN_ERR "%s: CPUidle register device failed\n",
		       __func__);
		return -EIO;
	}
	return 0;
}
#endif /* CONFIG_CPU_IDLE */
