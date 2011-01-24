/*
 * linux/arch/arm/plat-omap/mux.c
 *
 * Utility to set the Omap MUX and PULL_DWN registers from a table in mux.h
 *
 * Copyright (C) 2003 - 2008 Nokia Corporation
 *
 * Written by Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <asm/system.h>
#include <linux/spinlock.h>
#include <mach/mux.h>

#ifdef CONFIG_OMAP_MUX

static struct list_head config_list = LIST_HEAD_INIT(config_list);

int __init omap_mux_register(struct omap_mux_cfg *arch_mux_cfg)
{
	int i;

	if (!arch_mux_cfg || !arch_mux_cfg->pins || arch_mux_cfg->size == 0
			|| !arch_mux_cfg->cfg_reg) {
		printk(KERN_ERR "Invalid pin table\n");
		return -EINVAL;
	}

	/* Configure any initial states */
	if (!arch_mux_cfg->legacy)
		for (i = 0; i < arch_mux_cfg->size; i++)
			if (strcmp(arch_mux_cfg->pins[i].name, MUX_INITIALIZE) == 0)
				arch_mux_cfg->cfg_reg(&arch_mux_cfg->pins[i]);

	list_add_tail(&arch_mux_cfg->list, &config_list);

	return 0;
}

/*
 * Sets the Omap MUX and PULL_DWN registers based on the table
 */
int __init_or_module omap_cfg_reg(const unsigned long index)
{
	struct omap_mux_cfg *mux_cfg;

	if (list_empty(&config_list)) {
		printk(KERN_ERR "Pin mux table not initialized\n");
		return -ENODEV;
	}

	list_for_each_entry(mux_cfg, &config_list, list)
		if (mux_cfg->cfg_reg && mux_cfg->legacy) {
			struct pin_config *reg = &mux_cfg->pins[index];

			if (index >= mux_cfg->size) {
				printk(KERN_ERR "Invalid pin mux index: %lu (%lu)\n",
				       index, mux_cfg->size);
				dump_stack();
				return -ENODEV;
			}

			return mux_cfg->cfg_reg(reg);
		}

	return -ENODEV;
}
EXPORT_SYMBOL(omap_cfg_reg);

int __init_or_module omap_mux_config(const char *group)
{
	struct omap_mux_cfg *mux_cfg;
	int i;

	if (list_empty(&config_list)) {
		printk(KERN_ERR "Pin mux table not initialized\n");
		return -ENODEV;
	}

	list_for_each_entry(mux_cfg, &config_list, list)
		if (mux_cfg->cfg_reg && !mux_cfg->legacy)
			for (i = 0; i < mux_cfg->size; i++)
				if (strcmp(mux_cfg->pins[i].name, group) == 0) {
					struct pin_config *reg = &mux_cfg->pins[i];
					int rval;
					rval = mux_cfg->cfg_reg(reg);
					if (rval < 0)
						return rval;
				}

	return 0;
}
EXPORT_SYMBOL(omap_mux_config);
#else
#define omap_mux_init() do {} while(0)
#define omap_cfg_reg(x)	do {} while(0)
#endif	/* CONFIG_OMAP_MUX */
