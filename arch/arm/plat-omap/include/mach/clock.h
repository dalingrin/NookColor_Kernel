/*
 *  arch/arm/plat-omap/include/mach/clock.h
 *
 *  Copyright (C) 2004 - 2005 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *  Based on clocks.h by Tony Lindgren, Gordon McNutt and RidgeRun, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/notifier.h>

#ifndef __ARCH_ARM_OMAP_CLOCK_H
#define __ARCH_ARM_OMAP_CLOCK_H

struct module;
struct clk;
struct clockdomain;

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)

struct clksel_rate {
	u32			val;
	u8			div;
	u8			flags;
};

struct clksel {
	struct clk		 *parent;
	const struct clksel_rate *rates;
};

struct dpll_data {
	u32			mult_mask;
	u32			div1_mask;
	unsigned long		last_rounded_rate;
	unsigned int		rate_tolerance;
	u32			max_tolerance;
	struct clk		*bypass_clk;
	u32			enable_mask;
	u16			mult_div1_reg;
	u16			control_reg;
	u16			max_multiplier;
	u16			last_rounded_m;
	u8			last_rounded_n;
	u8			min_divider;
	u8			max_divider;
#  if defined(CONFIG_ARCH_OMAP3)
	u8			modes;
	u8			auto_recal_bit;
	u8			recal_en_bit;
	u8			recal_st_bit;
	u16			autoidle_reg;
	u16			idlest_reg;
	u32			autoidle_mask;
	u32			idlest_mask;
	u32			freqsel_mask;
	u32			dco_sel_mask;
	u32			sd_div_mask;
	u8			jtype;
#  endif
};

#endif

/**
 * struct clk_child - used to track the children of a clock
 * @clk: child struct clk *
 * @node: list_head
 * @flags: is this entry allocated in bootmem or slab?  is it deleted?
 *
 * One struct clk_child is allocated for each child clock @clk of a
 * parent clock.  @flags values are listed below and start with CLK_CHILD_*.
 */
struct clk_child {
	struct clk		*clk;
	struct list_head	node;
	u8			flags;
};

/**
 * struct clk_notifier - associate a clk with a notifier
 * @clk: struct clk * to associate the notifier with
 * @notifier_head: a blocking_notifier_head for this clk
 * @node: linked list pointers
 *
 * A list of struct clk_notifier is maintained by the notifier code.
 * An entry is created whenever code registers the first notifier on a
 * particular @clk.  Future notifiers on that @clk are added to the
 * @notifier_head.
 */
struct clk_notifier {
	struct clk			*clk;
	struct blocking_notifier_head	notifier_head;
	struct list_head		node;
};

/**
 * struct clk_notifier_data - rate data to pass to the notifier callback
 * @clk: struct clk * being changed
 * @old_rate: previous rate of this clock
 * @new_rate: new rate of this clock
 *
 * For a pre-notifier, old_rate is the clock's rate before this rate
 * change, and new_rate is what the rate will be in the future.  For a
 * post-notifier, old_rate and new_rate are both set to the clock's
 * current rate (this was done to optimize the implementation).
 */
struct clk_notifier_data {
	struct clk		*clk;
	unsigned long		old_rate;
	unsigned long		new_rate;
};

struct clk {
	struct list_head	node;
	const char		*name;
	int			id;
	struct clk		*parent;
	unsigned long		rate;
	unsigned long		temp_rate;
	struct list_head	children;
	__u32			flags;
	u32			enable_reg;
	void			(*recalc)(struct clk *, unsigned long, u8);
	int			(*set_rate)(struct clk *, unsigned long);
	long			(*round_rate)(struct clk *, unsigned long);
	void			(*init)(struct clk *);
	int			(*enable)(struct clk *);
	void			(*disable)(struct clk *);
	u16			notifier_count;
	__u8			enable_bit;
	__s8			usecount;
	u8			idlest_bit;
#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
	u8			fixed_div, clksel_shift;
	u32			clksel_mask, clksel_mask2;
	const struct clksel	*clksel;
	struct dpll_data	*dpll_data;
	union {
		const char		*name;
		struct clockdomain	*ptr;
	} clkdm;
	u16			clksel_reg;
	s16			prcm_mod;
#else
	__u8			rate_offset;
	__u8			src_offset;
#endif
#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
	struct dentry		*dent;	/* For visible tree hierarchy */
#endif
};

struct cpufreq_frequency_table;

struct clk_functions {
	int		(*clk_register)(struct clk *clk);
	int		(*clk_enable)(struct clk *clk);
	void		(*clk_disable)(struct clk *clk);
	long		(*clk_round_rate)(struct clk *clk, unsigned long rate);
	long		(*clk_round_rate_parent)(struct clk *clk,
						 struct clk *parent);
	int		(*clk_set_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_parent)(struct clk *clk, struct clk *parent);
	struct clk *	(*clk_get_parent)(struct clk *clk);
	void		(*clk_allow_idle)(struct clk *clk);
	void		(*clk_deny_idle)(struct clk *clk);
	void		(*clk_disable_unused)(struct clk *clk);
#ifdef CONFIG_CPU_FREQ
	void		(*clk_init_cpufreq_table)(struct cpufreq_frequency_table **);
#endif
};

extern unsigned int mpurate;

extern int clk_init(struct clk_functions *custom_clocks);
extern int clk_register(struct clk *clk);
extern void clk_unregister(struct clk *clk);
extern void propagate_rate(struct clk *clk, u8 rate_storage);
extern void recalculate_root_clocks(void);
extern void followparent_recalc(struct clk *clk, unsigned long parent_rate,
				u8 rate_storage);
extern void clk_allow_idle(struct clk *clk);
extern void clk_deny_idle(struct clk *clk);
extern void clk_enable_init_clocks(void);
extern int clk_notifier_register(struct clk *clk, struct notifier_block *nb);
extern int clk_notifier_unregister(struct clk *clk, struct notifier_block *nb);
#ifdef CONFIG_CPU_FREQ
extern void clk_init_cpufreq_table(struct cpufreq_frequency_table **table);
#endif
void omap_clk_add_child(struct clk *clk, struct clk *clk2);
void omap_clk_del_child(struct clk *clk, struct clk *clk2);

/* Clock flags */
#define RATE_CKCTL		(1 << 0)	/* Main fixed ratio clocks */
/* bits 1-3 are currently free */
#define ALWAYS_ENABLED		(1 << 4)	/* Clock cannot be disabled */
#define ENABLE_REG_32BIT	(1 << 5)	/* Use 32-bit access */
/* bit 6 is currently free */
#define CLOCK_IDLE_CONTROL	(1 << 7)
#define CLOCK_NO_IDLE_PARENT	(1 << 8)
#define DELAYED_APP		(1 << 9)	/* Delay application of clock */
/* bit 10 is currently free */
#define ENABLE_ON_INIT		(1 << 11)	/* Enable upon framework init */
#define INVERT_ENABLE		(1 << 12)	/* 0 enables, 1 disables */
#define WAIT_READY		(1 << 13)	/* wait for dev to leave idle */
#define RECALC_ON_ENABLE	(1 << 14)	/* recalc/prop on ena/disa */
/* bits 15-20 are currently free */
#define CLOCK_IN_OMAP310	(1 << 21)
#define CLOCK_IN_OMAP730	(1 << 22)
#define CLOCK_IN_OMAP1510	(1 << 23)
#define CLOCK_IN_OMAP16XX	(1 << 24)
#define CLOCK_IN_OMAP242X	(1 << 25)
#define CLOCK_IN_OMAP243X	(1 << 26)
#define CLOCK_IN_OMAP343X	(1 << 27)	/* clocks common to all 343X */
#define PARENT_CONTROLS_CLOCK	(1 << 28)
#define CLOCK_IN_OMAP3430ES1	(1 << 29)	/* 3430ES1 clocks only */
#define CLOCK_IN_OMAP3430ES2	(1 << 30)	/* 3430ES2+ clocks only */
#define CLOCK_IN_OMAP363X 	(1 << 31)

/* Clksel_rate flags */
#define DEFAULT_RATE		(1 << 0)
#define RATE_IN_242X		(1 << 1)
#define RATE_IN_243X		(1 << 2)
#define RATE_IN_343X		(1 << 3)	/* rates common to all 343X */
#define RATE_IN_3430ES2		(1 << 4)	/* 3430ES2+ rates only */
#define RATE_IN_363X		(1 << 5)	/* rates common to all 3630 */


#define RATE_IN_24XX		(RATE_IN_242X | RATE_IN_243X)

/* rate_storage parameter flags */
#define CURRENT_RATE		0
#define TEMP_RATE		1

/* clk_child flags */
#define CLK_CHILD_SLAB_ALLOC	(1 << 0)	/* if !set, bootmem was used */
#define CLK_CHILD_DELETED	(1 << 1)	/* can be reused */

/*
 * clk.prcm_mod flags (possible since only the top byte in clk.prcm_mod
 * is significant)
 */
#define PRCM_MOD_ADDR_MASK	0xff00
#define CLK_REG_IN_PRM		(1 << 0)
#define CLK_REG_IN_SCM		(1 << 1)

/*
 * Clk notifier callback types
 *
 * Since the notifier is called with interrupts disabled, any actions
 * taken by callbacks must be extremely fast and lightweight.
 *
 * CLK_PRE_RATE_CHANGE - called after all callbacks have approved the
 *     rate change, immediately before the clock rate is changed, to
 *     indicate that the rate change will proceed.  Drivers must
 *     immediately terminate any operations that will be affected by
 *     the rate change.  Callbacks must always return NOTIFY_DONE.
 *
 * CLK_ABORT_RATE_CHANGE: called if the rate change failed for some
 *     reason after CLK_PRE_RATE_CHANGE.  In this case, all registered
 *     notifiers on the clock will be called with
 *     CLK_ABORT_RATE_CHANGE. Callbacks must always return
 *     NOTIFY_DONE.
 *
 * CLK_POST_RATE_CHANGE - called after the clock rate change has
 *     successfully completed.  Callbacks must always return
 *     NOTIFY_DONE.
 *
 */
#define CLK_PRE_RATE_CHANGE		1
#define CLK_ABORT_RATE_CHANGE		2
#define CLK_POST_RATE_CHANGE		3

#endif
