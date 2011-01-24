/*
 * linux/arch/arm/mach-omap2/board-n800-usb.c
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Juha Yrjola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/usb/musb.h>
#include <mach/gpmc.h>

#include "pm.h"

#define TUSB_ASYNC_CS		1
#define TUSB_SYNC_CS		4
#define GPIO_TUSB_INT		58
#define GPIO_TUSB_ENABLE	0

static int tusb_set_power(int state);
static int tusb_set_clock(struct clk *osc_ck, int state);

#if	defined(CONFIG_USB_MUSB_OTG)
#	define BOARD_MODE	MUSB_OTG
#elif	defined(CONFIG_USB_MUSB_PERIPHERAL)
#	define BOARD_MODE	MUSB_PERIPHERAL
#else	/* defined(CONFIG_USB_MUSB_HOST) */
#	define BOARD_MODE	MUSB_HOST
#endif

static struct musb_hdrc_eps_bits musb_eps[] = {
	{	"ep1_tx", 5,	},
	{	"ep1_rx", 5,	},
	{	"ep2_tx", 5,	},
	{	"ep2_rx", 5,	},
	{	"ep3_tx", 3,	},
	{	"ep3_rx", 3,	},
	{	"ep4_tx", 3,	},
	{	"ep4_rx", 3,	},
	{	"ep5_tx", 2,	},
	{	"ep5_rx", 2,	},
	{	"ep6_tx", 2,	},
	{	"ep6_rx", 2,	},
	{	"ep7_tx", 2,	},
	{	"ep7_rx", 2,	},
	{	"ep8_tx", 2,	},
	{	"ep8_rx", 2,	},
	{	"ep9_tx", 2,	},
	{	"ep9_rx", 2,	},
	{	"ep10_tx", 2,	},
	{	"ep10_rx", 2,	},
	{	"ep11_tx", 2,	},
	{	"ep11_rx", 2,	},
	{	"ep12_tx", 2,	},
	{	"ep12_rx", 2,	},
	{	"ep13_tx", 2,	},
	{	"ep13_rx", 2,	},
	{	"ep14_tx", 2,	},
	{	"ep14_rx", 2,	},
	{	"ep15_tx", 2,	},
	{	"ep15_rx", 2,	},
};

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.soft_con	= 1,
	.dma		= 1,
	.num_eps	= 16,
	.dma_channels	= 7,
	.ram_bits	= 12,
	.eps_bits	= musb_eps,
};

static struct musb_hdrc_platform_data tusb_data = {
	.mode		= BOARD_MODE,
	.set_power	= tusb_set_power,
	.set_clock	= tusb_set_clock,
	.min_power	= 25,	/* x2 = 50 mA drawn from VBUS as peripheral */
	.power		= 100,	/* Max 100 mA VBUS for host mode */
	.clock		= "osc_ck",
	.config		= &musb_config,
};

/*
 * Enable or disable power to TUSB6010. When enabling, turn on 3.3 V and
 * 1.5 V voltage regulators of PM companion chip. Companion chip will then
 * provide then PGOOD signal to TUSB6010 which will release it from reset.
 */
static int tusb_set_power(int state)
{
	int i, retval = 0;

	if (state) {
		gpio_set_value(GPIO_TUSB_ENABLE, 1);
		msleep(1);

		/* Wait until TUSB6010 pulls INT pin down */
		i = 100;
		while (i && gpio_get_value(GPIO_TUSB_INT)) {
			msleep(1);
			i--;
		}

		if (!i) {
			printk(KERN_ERR "tusb: powerup failed\n");
			retval = -ENODEV;
		}
	} else {
		gpio_set_value(GPIO_TUSB_ENABLE, 0);
		msleep(10);
	}

	return retval;
}

static int		osc_ck_on;

static int tusb_set_clock(struct clk *osc_ck, int state)
{
	if (state) {
		if (osc_ck_on > 0)
			return -ENODEV;

		omap2_block_sleep();
		clk_enable(osc_ck);
		osc_ck_on = 1;
	} else {
		if (osc_ck_on == 0)
			return -ENODEV;

		clk_disable(osc_ck);
		osc_ck_on = 0;
		omap2_allow_sleep();
	}

	return 0;
}

void __init n800_usb_init(void)
{
	int ret = 0;
	static char	announce[] __initdata = KERN_INFO "TUSB 6010\n";

	/* PM companion chip power control pin */
	ret = gpio_request(GPIO_TUSB_ENABLE, "TUSB6010 enable");
	if (ret != 0) {
		printk(KERN_ERR "Could not get TUSB power GPIO%i\n",
		       GPIO_TUSB_ENABLE);
		return;
	}
	gpio_direction_output(GPIO_TUSB_ENABLE, 0);

	tusb_set_power(0);

	ret = tusb6010_setup_interface(&tusb_data, TUSB6010_REFCLK_19, 2,
					TUSB_ASYNC_CS, TUSB_SYNC_CS,
					GPIO_TUSB_INT, 0x3f);
	if (ret != 0)
		goto err;

	printk(announce);

	return;

err:
	gpio_free(GPIO_TUSB_ENABLE);
}
