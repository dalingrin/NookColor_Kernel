/*
 * linux/arch/arm/mach-omap2/prcm.c
 *
 * OMAP 24xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2005 Nokia Corporation
 *
 * Written by Tony Lindgren <tony.lindgren@nokia.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Some pieces of code Copyright (C) 2005 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/common.h>
#include <mach/control.h>
#include <mach/prcm.h>
#include <mach/irqs.h>
#include <mach/control.h>
#include <mach/sram.h>

#include "sdrc.h"
#include "clock.h"
#include "cm.h"
#include "prm.h"
#include "prm-regbits-24xx.h"

static void __iomem *prm_base;
static void __iomem *cm_base;

struct omap3_prcm_regs {
	u32 control_padconf_sys_nirq;
	u32 iva2_cm_clksel1;
	u32 iva2_cm_clksel2;
	u32 cm_sysconfig;
	u32 sgx_cm_clksel;
	u32 wkup_cm_clksel;
	u32 dss_cm_clksel;
	u32 cam_cm_clksel;
	u32 per_cm_clksel;
	u32 emu_cm_clksel;
	u32 emu_cm_clkstctrl;
	u32 pll_cm_autoidle2;
	u32 pll_cm_clksel4;
	u32 pll_cm_clksel5;
	u32 pll_cm_clken;
	u32 pll_cm_clken2;
	u32 cm_polctrl;
	u32 iva2_cm_fclken;
	u32 iva2_cm_clken_pll;
	u32 core_cm_fclken1;
	u32 core_cm_fclken3;
	u32 sgx_cm_fclken;
	u32 wkup_cm_fclken;
	u32 dss_cm_fclken;
	u32 cam_cm_fclken;
	u32 per_cm_fclken;
	u32 usbhost_cm_fclken;
	u32 core_cm_iclken1;
	u32 core_cm_iclken2;
	u32 core_cm_iclken3;
	u32 sgx_cm_iclken;
	u32 wkup_cm_iclken;
	u32 dss_cm_iclken;
	u32 cam_cm_iclken;
	u32 per_cm_iclken;
	u32 usbhost_cm_iclken;
	u32 iva2_cm_autiidle2;
	u32 mpu_cm_autoidle2;
	u32 pll_cm_autoidle;
	u32 iva2_cm_clkstctrl;
	u32 mpu_cm_clkstctrl;
	u32 core_cm_clkstctrl;
	u32 sgx_cm_clkstctrl;
	u32 dss_cm_clkstctrl;
	u32 cam_cm_clkstctrl;
	u32 per_cm_clkstctrl;
	u32 neon_cm_clkstctrl;
	u32 usbhost_cm_clkstctrl;
	u32 core_cm_autoidle1;
	u32 core_cm_autoidle2;
	u32 core_cm_autoidle3;
	u32 wkup_cm_autoidle;
	u32 dss_cm_autoidle;
	u32 cam_cm_autoidle;
	u32 per_cm_autoidle;
	u32 usbhost_cm_autoidle;
	u32 sgx_cm_sleepdep;
	u32 dss_cm_sleepdep;
	u32 cam_cm_sleepdep;
	u32 per_cm_sleepdep;
	u32 usbhost_cm_sleepdep;
	u32 cm_clkout_ctrl;
	u32 prm_clkout_ctrl;
	u32 sgx_pm_wkdep;
	u32 dss_pm_wkdep;
	u32 cam_pm_wkdep;
	u32 per_pm_wkdep;
	u32 neon_pm_wkdep;
	u32 usbhost_pm_wkdep;
	u32 core_pm_mpugrpsel1;
	u32 iva2_pm_ivagrpsel1;
	u32 core_pm_mpugrpsel3;
	u32 core_pm_ivagrpsel3;
	u32 wkup_pm_mpugrpsel;
	u32 wkup_pm_ivagrpsel;
	u32 per_pm_mpugrpsel;
	u32 per_pm_ivagrpsel;
	u32 wkup_pm_wken;
};

struct omap3_prcm_regs prcm_context;

u32 omap_prcm_get_reset_sources(void)
{
	/* XXX This presumably needs modification for 34XX */
	return prm_read_mod_reg(WKUP_MOD, RM_RSTST) & 0x7f;
}
EXPORT_SYMBOL(omap_prcm_get_reset_sources);

/* Resets clock rates and reboots the system. Only called from system.h */
void omap_prcm_arch_reset(char mode, const char *cmd)
{
	s16 prcm_offs;

	omap2_clk_prepare_for_reboot();

	if (cpu_is_omap24xx()) {
		prcm_offs = WKUP_MOD;
		prm_set_mod_reg_bits(OMAP_RST_DPLL3, prcm_offs, RM_RSTCTRL);
	} else if (cpu_is_omap34xx()) {
		u32 l;

		/* Copy cmd into scratchpad memmory if any */
		if(cmd != 0)
		{
			u16  counter = 0;
		  
			while((counter < (OMAP343X_SCRATCHPAD_BCB_SIZE-1)) && (cmd[counter]!='\0')) {
				omap_writeb(cmd[counter], OMAP343X_SCRATCHPAD_BCB + counter);
				counter++;
			}

			omap_writeb('\0', OMAP343X_SCRATCHPAD_BCB + counter);
			
		} 

		prcm_offs = OMAP3430_GR_MOD;
		l = ('B' << 24) | ('M' << 16) | mode;
		/* Reserve the first word in scratchpad for communicating
		 * with the boot ROM. A pointer to a data structure
		 * describing the boot process can be stored there,
		 * cf. OMAP34xx TRM, Initialization / Software Booting
		 * Configuration. */
		omap_writel(l, OMAP343X_SCRATCHPAD + 4);

		omap3_configure_core_dpll_warmreset();

	} else
		WARN_ON(1);
}

static inline u32 __omap_prcm_read(void __iomem *base, s16 module, u16 reg)
{
	BUG_ON(!base);
	return __raw_readl(base + module + reg);
}

static inline void __omap_prcm_write(u32 value, void __iomem *base,
						s16 module, u16 reg)
{
	BUG_ON(!base);
	__raw_writel(value, base + module + reg);
}

/* Read a register in a PRM module */
u32 prm_read_mod_reg(s16 module, u16 idx)
{
	return __omap_prcm_read(prm_base, module, idx);
}
EXPORT_SYMBOL(prm_read_mod_reg);

/* Write into a register in a PRM module */
void prm_write_mod_reg(u32 val, s16 module, u16 idx)
{
	__omap_prcm_write(val, prm_base, module, idx);
}
EXPORT_SYMBOL(prm_write_mod_reg);

/* Read-modify-write a register in a PRM module. Caller must lock */
u32 prm_rmw_mod_reg_bits(u32 mask, u32 bits, s16 module, s16 idx)
{
	u32 v;

	v = prm_read_mod_reg(module, idx);
	v &= ~mask;
	v |= bits;
	prm_write_mod_reg(v, module, idx);

	return v;
}
EXPORT_SYMBOL(prm_rmw_mod_reg_bits);

/* Read a register in a CM module */
u32 cm_read_mod_reg(s16 module, u16 idx)
{
	return __omap_prcm_read(cm_base, module, idx);
}
EXPORT_SYMBOL(cm_read_mod_reg);

/* Write into a register in a CM module */
void cm_write_mod_reg(u32 val, s16 module, u16 idx)
{
	__omap_prcm_write(val, cm_base, module, idx);
}
EXPORT_SYMBOL(cm_write_mod_reg);

/* Read-modify-write a register in a CM module. Caller must lock */
u32 cm_rmw_mod_reg_bits(u32 mask, u32 bits, s16 module, s16 idx)
{
	u32 v;

	v = cm_read_mod_reg(module, idx);
	v &= ~mask;
	v |= bits;
	cm_write_mod_reg(v, module, idx);

	return v;
}
EXPORT_SYMBOL(cm_rmw_mod_reg_bits);

void __init omap2_set_globals_prcm(struct omap_globals *omap2_globals)
{
	prm_base = omap2_globals->prm;
	cm_base = omap2_globals->cm;
}

#ifdef CONFIG_ARCH_OMAP3
void omap3_prcm_save_context(void)
{
	prcm_context.control_padconf_sys_nirq =
			 omap_ctrl_readl(OMAP343X_CONTROL_PADCONF_SYSNIRQ);
	prcm_context.iva2_cm_clksel1 =
			 cm_read_mod_reg(OMAP3430_IVA2_MOD, CM_CLKSEL1);
	prcm_context.iva2_cm_clksel2 =
			 cm_read_mod_reg(OMAP3430_IVA2_MOD, CM_CLKSEL2);
	prcm_context.cm_sysconfig = __raw_readl(OMAP3430_CM_SYSCONFIG);
	prcm_context.sgx_cm_clksel =
			 cm_read_mod_reg(OMAP3430ES2_SGX_MOD, CM_CLKSEL);
	prcm_context.dss_cm_clksel =
			 cm_read_mod_reg(OMAP3430_DSS_MOD, CM_CLKSEL);
	prcm_context.cam_cm_clksel =
			 cm_read_mod_reg(OMAP3430_CAM_MOD, CM_CLKSEL);
	prcm_context.per_cm_clksel =
			 cm_read_mod_reg(OMAP3430_PER_MOD, CM_CLKSEL);
	prcm_context.emu_cm_clksel =
			 cm_read_mod_reg(OMAP3430_EMU_MOD, CM_CLKSEL1);
	prcm_context.emu_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430_EMU_MOD, CM_CLKSTCTRL);
	prcm_context.pll_cm_autoidle2 =
			 cm_read_mod_reg(PLL_MOD, CM_AUTOIDLE2);
	prcm_context.pll_cm_clksel4 =
			cm_read_mod_reg(PLL_MOD, OMAP3430ES2_CM_CLKSEL4);
	prcm_context.pll_cm_clksel5 =
			 cm_read_mod_reg(PLL_MOD, OMAP3430ES2_CM_CLKSEL5);
	prcm_context.pll_cm_clken2 =
			cm_read_mod_reg(PLL_MOD, OMAP3430ES2_CM_CLKEN2);
	prcm_context.cm_polctrl = __raw_readl(OMAP3430_CM_POLCTRL);
	prcm_context.iva2_cm_fclken =
			 cm_read_mod_reg(OMAP3430_IVA2_MOD, CM_FCLKEN);
	prcm_context.iva2_cm_clken_pll = cm_read_mod_reg(OMAP3430_IVA2_MOD,
			OMAP3430_CM_CLKEN_PLL);
	prcm_context.core_cm_fclken1 =
			 cm_read_mod_reg(CORE_MOD, CM_FCLKEN1);
	prcm_context.core_cm_fclken3 =
			 cm_read_mod_reg(CORE_MOD, OMAP3430ES2_CM_FCLKEN3);
	prcm_context.sgx_cm_fclken =
			 cm_read_mod_reg(OMAP3430ES2_SGX_MOD, CM_FCLKEN);
	prcm_context.wkup_cm_fclken =
			 cm_read_mod_reg(WKUP_MOD, CM_FCLKEN);
	prcm_context.dss_cm_fclken =
			 cm_read_mod_reg(OMAP3430_DSS_MOD, CM_FCLKEN);
	prcm_context.cam_cm_fclken =
			 cm_read_mod_reg(OMAP3430_CAM_MOD, CM_FCLKEN);
	prcm_context.per_cm_fclken =
			 cm_read_mod_reg(OMAP3430_PER_MOD, CM_FCLKEN);
	prcm_context.usbhost_cm_fclken =
			 cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_FCLKEN);
	prcm_context.core_cm_iclken1 =
			 cm_read_mod_reg(CORE_MOD, CM_ICLKEN1);
	prcm_context.core_cm_iclken2 =
			 cm_read_mod_reg(CORE_MOD, CM_ICLKEN2);
	prcm_context.core_cm_iclken3 =
			 cm_read_mod_reg(CORE_MOD, CM_ICLKEN3);
	prcm_context.sgx_cm_iclken =
			 cm_read_mod_reg(OMAP3430ES2_SGX_MOD, CM_ICLKEN);
	prcm_context.wkup_cm_iclken =
			 cm_read_mod_reg(WKUP_MOD, CM_ICLKEN);
	prcm_context.dss_cm_iclken =
			 cm_read_mod_reg(OMAP3430_DSS_MOD, CM_ICLKEN);
	prcm_context.cam_cm_iclken =
			 cm_read_mod_reg(OMAP3430_CAM_MOD, CM_ICLKEN);
	prcm_context.per_cm_iclken =
			 cm_read_mod_reg(OMAP3430_PER_MOD, CM_ICLKEN);
	prcm_context.usbhost_cm_iclken =
			 cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_ICLKEN);
	prcm_context.iva2_cm_autiidle2 =
			 cm_read_mod_reg(OMAP3430_IVA2_MOD, CM_AUTOIDLE2);
	prcm_context.mpu_cm_autoidle2 =
			 cm_read_mod_reg(MPU_MOD, CM_AUTOIDLE2);
	prcm_context.iva2_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430_IVA2_MOD, CM_CLKSTCTRL);
	prcm_context.mpu_cm_clkstctrl =
			 cm_read_mod_reg(MPU_MOD, CM_CLKSTCTRL);
	prcm_context.core_cm_clkstctrl =
			 cm_read_mod_reg(CORE_MOD, CM_CLKSTCTRL);
	prcm_context.sgx_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430ES2_SGX_MOD, CM_CLKSTCTRL);
	prcm_context.dss_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430_DSS_MOD, CM_CLKSTCTRL);
	prcm_context.cam_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430_CAM_MOD, CM_CLKSTCTRL);
	prcm_context.per_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430_PER_MOD, CM_CLKSTCTRL);
	prcm_context.neon_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430_NEON_MOD, CM_CLKSTCTRL);
	prcm_context.usbhost_cm_clkstctrl =
			 cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_CLKSTCTRL);
	prcm_context.core_cm_autoidle1 =
			 cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE1);
	prcm_context.core_cm_autoidle2 =
			 cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE2);
	prcm_context.core_cm_autoidle3 =
			 cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE3);
	prcm_context.wkup_cm_autoidle =
			 cm_read_mod_reg(WKUP_MOD, CM_AUTOIDLE);
	prcm_context.dss_cm_autoidle =
			 cm_read_mod_reg(OMAP3430_DSS_MOD, CM_AUTOIDLE);
	prcm_context.cam_cm_autoidle =
			 cm_read_mod_reg(OMAP3430_CAM_MOD, CM_AUTOIDLE);
	prcm_context.per_cm_autoidle =
			 cm_read_mod_reg(OMAP3430_PER_MOD, CM_AUTOIDLE);
	prcm_context.usbhost_cm_autoidle =
			 cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_AUTOIDLE);
	prcm_context.sgx_cm_sleepdep =
		 cm_read_mod_reg(OMAP3430ES2_SGX_MOD, OMAP3430_CM_SLEEPDEP);
	prcm_context.dss_cm_sleepdep =
		 cm_read_mod_reg(OMAP3430_DSS_MOD, OMAP3430_CM_SLEEPDEP);
	prcm_context.cam_cm_sleepdep =
		 cm_read_mod_reg(OMAP3430_CAM_MOD, OMAP3430_CM_SLEEPDEP);
	prcm_context.per_cm_sleepdep =
		 cm_read_mod_reg(OMAP3430_PER_MOD, OMAP3430_CM_SLEEPDEP);
	prcm_context.usbhost_cm_sleepdep =
		 cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, OMAP3430_CM_SLEEPDEP);
	prcm_context.cm_clkout_ctrl = cm_read_mod_reg(OMAP3430_CCR_MOD,
		 OMAP3430_CM_CLKOUT_CTRL_OFFSET);
	prcm_context.prm_clkout_ctrl = prm_read_mod_reg(OMAP3430_CCR_MOD,
		OMAP3_PRM_CLKOUT_CTRL_OFFSET);
	prcm_context.sgx_pm_wkdep =
		 prm_read_mod_reg(OMAP3430ES2_SGX_MOD, PM_WKDEP);
	prcm_context.dss_pm_wkdep =
		 prm_read_mod_reg(OMAP3430_DSS_MOD, PM_WKDEP);
	prcm_context.cam_pm_wkdep =
		 prm_read_mod_reg(OMAP3430_CAM_MOD, PM_WKDEP);
	prcm_context.per_pm_wkdep =
		 prm_read_mod_reg(OMAP3430_PER_MOD, PM_WKDEP);
	prcm_context.neon_pm_wkdep =
		 prm_read_mod_reg(OMAP3430_NEON_MOD, PM_WKDEP);
	prcm_context.usbhost_pm_wkdep =
		 prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, PM_WKDEP);
	prcm_context.core_pm_mpugrpsel1 =
		 prm_read_mod_reg(CORE_MOD, OMAP3430_PM_MPUGRPSEL1);
	prcm_context.iva2_pm_ivagrpsel1 =
		 prm_read_mod_reg(OMAP3430_IVA2_MOD, OMAP3430_PM_IVAGRPSEL1);
	prcm_context.core_pm_mpugrpsel3 =
		 prm_read_mod_reg(CORE_MOD, OMAP3430ES2_PM_MPUGRPSEL3);
	prcm_context.core_pm_ivagrpsel3 =
		 prm_read_mod_reg(CORE_MOD, OMAP3430ES2_PM_IVAGRPSEL3);
	prcm_context.wkup_pm_mpugrpsel =
		 prm_read_mod_reg(WKUP_MOD, OMAP3430_PM_MPUGRPSEL);
	prcm_context.wkup_pm_ivagrpsel =
		 prm_read_mod_reg(WKUP_MOD, OMAP3430_PM_IVAGRPSEL);
	prcm_context.per_pm_mpugrpsel =
		 prm_read_mod_reg(OMAP3430_PER_MOD, OMAP3430_PM_MPUGRPSEL);
	prcm_context.per_pm_ivagrpsel =
		 prm_read_mod_reg(OMAP3430_PER_MOD, OMAP3430_PM_IVAGRPSEL);
	prcm_context.wkup_pm_wken = prm_read_mod_reg(WKUP_MOD, PM_WKEN);
	return;
}

void omap3_prcm_restore_context(void)
{
	omap_ctrl_writel(prcm_context.control_padconf_sys_nirq,
					 OMAP343X_CONTROL_PADCONF_SYSNIRQ);
	cm_write_mod_reg(prcm_context.iva2_cm_clksel1, OMAP3430_IVA2_MOD,
					 CM_CLKSEL1);
	cm_write_mod_reg(prcm_context.iva2_cm_clksel2, OMAP3430_IVA2_MOD,
					 CM_CLKSEL2);
	__raw_writel(prcm_context.cm_sysconfig, OMAP3430_CM_SYSCONFIG);
	cm_write_mod_reg(prcm_context.sgx_cm_clksel, OMAP3430ES2_SGX_MOD,
					 CM_CLKSEL);
	cm_write_mod_reg(prcm_context.dss_cm_clksel, OMAP3430_DSS_MOD,
					 CM_CLKSEL);
	cm_write_mod_reg(prcm_context.cam_cm_clksel, OMAP3430_CAM_MOD,
					 CM_CLKSEL);
	cm_write_mod_reg(prcm_context.per_cm_clksel, OMAP3430_PER_MOD,
					 CM_CLKSEL);
	cm_write_mod_reg(prcm_context.emu_cm_clksel, OMAP3430_EMU_MOD,
					 CM_CLKSEL1);
	cm_write_mod_reg(prcm_context.emu_cm_clkstctrl, OMAP3430_EMU_MOD,
					 CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.pll_cm_autoidle2, PLL_MOD,
					 CM_AUTOIDLE2);
	cm_write_mod_reg(prcm_context.pll_cm_clksel4, PLL_MOD,
					OMAP3430ES2_CM_CLKSEL4);
	cm_write_mod_reg(prcm_context.pll_cm_clksel5, PLL_MOD,
					 OMAP3430ES2_CM_CLKSEL5);
	cm_write_mod_reg(prcm_context.pll_cm_clken2, PLL_MOD,
					OMAP3430ES2_CM_CLKEN2);
	__raw_writel(prcm_context.cm_polctrl, OMAP3430_CM_POLCTRL);
	cm_write_mod_reg(prcm_context.iva2_cm_fclken, OMAP3430_IVA2_MOD,
					 CM_FCLKEN);
	cm_write_mod_reg(prcm_context.iva2_cm_clken_pll, OMAP3430_IVA2_MOD,
					OMAP3430_CM_CLKEN_PLL);
	cm_write_mod_reg(prcm_context.core_cm_fclken1, CORE_MOD, CM_FCLKEN1);
	cm_write_mod_reg(prcm_context.core_cm_fclken3, CORE_MOD,
					 OMAP3430ES2_CM_FCLKEN3);
	cm_write_mod_reg(prcm_context.sgx_cm_fclken, OMAP3430ES2_SGX_MOD,
					 CM_FCLKEN);
	cm_write_mod_reg(prcm_context.wkup_cm_fclken, WKUP_MOD, CM_FCLKEN);
	cm_write_mod_reg(prcm_context.dss_cm_fclken, OMAP3430_DSS_MOD,
					 CM_FCLKEN);
	cm_write_mod_reg(prcm_context.cam_cm_fclken, OMAP3430_CAM_MOD,
					 CM_FCLKEN);
	cm_write_mod_reg(prcm_context.per_cm_fclken, OMAP3430_PER_MOD,
					 CM_FCLKEN);
	cm_write_mod_reg(prcm_context.usbhost_cm_fclken,
					 OMAP3430ES2_USBHOST_MOD, CM_FCLKEN);
	cm_write_mod_reg(prcm_context.core_cm_iclken1, CORE_MOD, CM_ICLKEN1);
	cm_write_mod_reg(prcm_context.core_cm_iclken2, CORE_MOD, CM_ICLKEN2);
	cm_write_mod_reg(prcm_context.core_cm_iclken3, CORE_MOD, CM_ICLKEN3);
	cm_write_mod_reg(prcm_context.sgx_cm_iclken, OMAP3430ES2_SGX_MOD,
					CM_ICLKEN);
	cm_write_mod_reg(prcm_context.wkup_cm_iclken, WKUP_MOD, CM_ICLKEN);
	cm_write_mod_reg(prcm_context.dss_cm_iclken, OMAP3430_DSS_MOD,
					CM_ICLKEN);
	cm_write_mod_reg(prcm_context.cam_cm_iclken, OMAP3430_CAM_MOD,
					CM_ICLKEN);
	cm_write_mod_reg(prcm_context.per_cm_iclken, OMAP3430_PER_MOD,
					CM_ICLKEN);
	cm_write_mod_reg(prcm_context.usbhost_cm_iclken,
					OMAP3430ES2_USBHOST_MOD, CM_ICLKEN);
	cm_write_mod_reg(prcm_context.iva2_cm_autiidle2, OMAP3430_IVA2_MOD,
					CM_AUTOIDLE2);
	cm_write_mod_reg(prcm_context.mpu_cm_autoidle2, MPU_MOD, CM_AUTOIDLE2);
	cm_write_mod_reg(prcm_context.iva2_cm_clkstctrl, OMAP3430_IVA2_MOD,
					CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.mpu_cm_clkstctrl, MPU_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.core_cm_clkstctrl, CORE_MOD,
					CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.sgx_cm_clkstctrl, OMAP3430ES2_SGX_MOD,
					CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.dss_cm_clkstctrl, OMAP3430_DSS_MOD,
					CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.cam_cm_clkstctrl, OMAP3430_CAM_MOD,
					CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.per_cm_clkstctrl, OMAP3430_PER_MOD,
					CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.neon_cm_clkstctrl, OMAP3430_NEON_MOD,
					CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.usbhost_cm_clkstctrl,
					OMAP3430ES2_USBHOST_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(prcm_context.core_cm_autoidle1, CORE_MOD,
					CM_AUTOIDLE1);
	cm_write_mod_reg(prcm_context.core_cm_autoidle2, CORE_MOD,
					CM_AUTOIDLE2);
	cm_write_mod_reg(prcm_context.core_cm_autoidle3, CORE_MOD,
					CM_AUTOIDLE3);
	cm_write_mod_reg(prcm_context.wkup_cm_autoidle, WKUP_MOD, CM_AUTOIDLE);
	cm_write_mod_reg(prcm_context.dss_cm_autoidle, OMAP3430_DSS_MOD,
					CM_AUTOIDLE);
	cm_write_mod_reg(prcm_context.cam_cm_autoidle, OMAP3430_CAM_MOD,
					CM_AUTOIDLE);
	cm_write_mod_reg(prcm_context.per_cm_autoidle, OMAP3430_PER_MOD,
					CM_AUTOIDLE);
	cm_write_mod_reg(prcm_context.usbhost_cm_autoidle,
					OMAP3430ES2_USBHOST_MOD, CM_AUTOIDLE);
	cm_write_mod_reg(prcm_context.sgx_cm_sleepdep, OMAP3430ES2_SGX_MOD,
					OMAP3430_CM_SLEEPDEP);
	cm_write_mod_reg(prcm_context.dss_cm_sleepdep, OMAP3430_DSS_MOD,
					OMAP3430_CM_SLEEPDEP);
	cm_write_mod_reg(prcm_context.cam_cm_sleepdep, OMAP3430_CAM_MOD,
					OMAP3430_CM_SLEEPDEP);
	cm_write_mod_reg(prcm_context.per_cm_sleepdep, OMAP3430_PER_MOD,
					OMAP3430_CM_SLEEPDEP);
	cm_write_mod_reg(prcm_context.usbhost_cm_sleepdep,
				OMAP3430ES2_USBHOST_MOD, OMAP3430_CM_SLEEPDEP);
	cm_write_mod_reg(prcm_context.cm_clkout_ctrl, OMAP3430_CCR_MOD,
					OMAP3430_CM_CLKOUT_CTRL_OFFSET);
	prm_write_mod_reg(prcm_context.prm_clkout_ctrl, OMAP3430_CCR_MOD,
					OMAP3_PRM_CLKOUT_CTRL_OFFSET);
	prm_write_mod_reg(prcm_context.sgx_pm_wkdep, OMAP3430ES2_SGX_MOD,
					PM_WKDEP);
	prm_write_mod_reg(prcm_context.dss_pm_wkdep, OMAP3430_DSS_MOD,
					PM_WKDEP);
	prm_write_mod_reg(prcm_context.cam_pm_wkdep, OMAP3430_CAM_MOD,
					PM_WKDEP);
	prm_write_mod_reg(prcm_context.per_pm_wkdep, OMAP3430_PER_MOD,
					PM_WKDEP);
	prm_write_mod_reg(prcm_context.neon_pm_wkdep, OMAP3430_NEON_MOD,
					PM_WKDEP);
	prm_write_mod_reg(prcm_context.usbhost_pm_wkdep,
					OMAP3430ES2_USBHOST_MOD, PM_WKDEP);
	prm_write_mod_reg(prcm_context.core_pm_mpugrpsel1, CORE_MOD,
					OMAP3430_PM_MPUGRPSEL1);
	prm_write_mod_reg(prcm_context.iva2_pm_ivagrpsel1, OMAP3430_IVA2_MOD,
					OMAP3430_PM_IVAGRPSEL1);
	prm_write_mod_reg(prcm_context.core_pm_mpugrpsel3, CORE_MOD,
					OMAP3430ES2_PM_MPUGRPSEL3);
	prm_write_mod_reg(prcm_context.core_pm_ivagrpsel3, CORE_MOD,
					OMAP3430ES2_PM_IVAGRPSEL3);
	prm_write_mod_reg(prcm_context.wkup_pm_mpugrpsel, WKUP_MOD,
					OMAP3430_PM_MPUGRPSEL);
	prm_write_mod_reg(prcm_context.wkup_pm_ivagrpsel, WKUP_MOD,
					OMAP3430_PM_IVAGRPSEL);
	prm_write_mod_reg(prcm_context.per_pm_mpugrpsel, OMAP3430_PER_MOD,
					OMAP3430_PM_MPUGRPSEL);
	prm_write_mod_reg(prcm_context.per_pm_ivagrpsel, OMAP3430_PER_MOD,
					 OMAP3430_PM_IVAGRPSEL);
	prm_write_mod_reg(prcm_context.wkup_pm_wken, WKUP_MOD, PM_WKEN);
	/* Relock DPLL5 and enable autoidle */

	if (omap_rev() >= OMAP3430_REV_ES2_0)
		omap3_lock_dpll5();

	return;
}
#endif
