#ifndef __ARCH_ARM_MACH_OMAP2_PM_H
#define __ARCH_ARM_MACH_OMAP2_PM_H
/*
 * linux/arch/arm/mach-omap2/pm.h
 *
 * OMAP Power Management Routines
 *
 * Copyright (C) 2008 Nokia Corporation
 * Jouni Hogander
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <mach/powerdomain.h>

extern int omap2_pm_init(void);
extern int omap3_pm_init(void);

#ifdef CONFIG_CPU_IDLE
int omap3_idle_init(void);
#else
static inline int omap3_idle_init(void) { return 0; }
#endif

extern unsigned short enable_dyn_sleep;
extern unsigned short enable_off_mode;
extern unsigned short enable_off_idle_path;
extern unsigned short enable_oswr_ret;
extern unsigned short voltage_off_while_idle;
extern atomic_t sleep_block;
extern void *omap3_secure_ram_storage;

extern unsigned short wakeup_timer_seconds;
extern struct omap_dm_timer *gptimer_wakeup;

#ifdef CONFIG_ARCH_OMAP3
struct prm_setup_times_vc {
	u16 clksetup;
	u16 voltsetup_time1;
	u16 voltsetup_time2;
	u16 voltsetup2;
	u16 voltsetup1;
};

struct prm_setup_vc {
	struct prm_setup_times_vc *setup_times;
	struct prm_setup_times_vc *setup_times_off;
	u16 voltoffset;
/* PRM_VC_CMD_VAL_0 specific bits */
	u16 vdd0_on;
	u16 vdd0_onlp;
	u16 vdd0_ret;
	u16 vdd0_off;
/* PRM_VC_CMD_VAL_1 specific bits */
	u16 vdd1_on;
	u16 vdd1_onlp;
	u16 vdd1_ret;
	u16 vdd1_off;
/* Values for VDD registers */
	u32 i2c_slave_ra;
	u32 vdd_vol_ra;
	u32 vdd_cmd_ra;
	u32 vdd_ch_conf;
	u32 vdd_i2c_cfg;
};
extern void omap3_pm_off_mode_enable(int);
extern int omap3_pm_get_suspend_state(struct powerdomain *pwrdm);
extern int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state);
extern void omap3_set_prm_setup_vc(struct prm_setup_vc *setup_vc);
extern int omap3_bypass_cmd(u8 slave_addr, u8 reg_addr, u8 cmd);
extern int set_dpll3_volt_freq(bool dpll3_restore);
#else
#define omap3_pm_off_mode_enable(int) do {} while (0);
#define omap3_pm_get_suspend_state(pwrdm) do {} while (0);
#define omap3_pm_set_suspend_state(pwrdm, state) do {} while (0);
#endif
extern int set_pwrdm_state(struct powerdomain *pwrdm, u32 state);
extern int resource_set_opp_level(int res, u32 target_level, int flags);
extern int resource_access_opp_lock(int res, int delta);
#define resource_lock_opp(res) resource_access_opp_lock(res, 1)
#define resource_unlock_opp(res) resource_access_opp_lock(res, -1)
#define resource_get_opp_lock(res) resource_access_opp_lock(res, 0)

#define OPP_IGNORE_LOCK 0x1

#ifdef CONFIG_PM_DEBUG
extern void omap2_pm_dump(int mode, int resume, unsigned int us);
extern int omap2_pm_debug;
extern void pm_dbg_update_time(struct powerdomain *pwrdm, int prev);
extern int pm_dbg_regset_save(int reg_set);
extern int pm_dbg_regset_init(int reg_set);
#else
#define omap2_pm_dump(mode, resume, us)		do {} while (0);
#define omap2_pm_debug				0
#define pm_dbg_update_time(pwrdm, prev) do {} while (0);
#define pm_dbg_regset_save(reg_set) do {} while (0);
#define pm_dbg_regset_init(reg_set) do {} while (0);
#endif /* CONFIG_PM_DEBUG */

extern void omap24xx_idle_loop_suspend(void);

extern void omap24xx_cpu_suspend(u32 dll_ctrl, void __iomem *sdrc_dlla_ctrl,
					void __iomem *sdrc_power);
extern void omap34xx_cpu_suspend(u32 *addr, int save_state);
extern void save_secure_ram_context(u32 *addr);
extern int omap3_can_sleep(void);

extern void omap_sram_idle(void);
#ifdef CONFIG_PM
extern void omap2_block_sleep(void);
extern void omap2_allow_sleep(void);
#else
static inline void omap2_block_sleep(void) { }
static inline void omap2_allow_sleep(void) { }
#endif

extern void omap3_cpuidle_update_states(void);
extern unsigned int omap24xx_idle_loop_suspend_sz;
extern unsigned int omap34xx_suspend_sz;
extern unsigned int save_secure_ram_context_sz;
extern unsigned int omap24xx_cpu_suspend_sz;
extern unsigned int omap34xx_cpu_suspend_sz;

#endif
