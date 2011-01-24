/*
 * linux/arch/arm/mach-omap2/board-connectivity.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <mach/gpio.h>
#include <mach/mux.h>

#if defined (CONFIG_MACH_OMAP_FST_OMAP3_127x)\
           || defined (CONFIG_MACH_OMAP_FST_OMAP3_128x)
#include <mach/board-zoom2.h>
#include <mach/mcspi.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpmc.h>
#endif

#include <asm/io.h>
#include <asm/delay.h>

#include "board-connectivity.h"

#if 0
#ifdef CONFIG_WL127X_RFKILL
#include <linux/wl127x-rfkill.h>
#endif

#define WL127X_BTEN_GPIO	109

#ifdef CONFIG_WL127X_RFKILL
static struct wl127x_rfkill_platform_data wl127x_plat_data = {
	.bt_nshutdown_gpio = 109,	/* Bluetooth Enable GPIO */
	.fm_enable_gpio = 161,	/* FM Enable GPIO */
};

static struct platform_device zoom2_wl127x_device = {
	.name = "wl127x-rfkill",
	.id = -1,
	.dev.platform_data = &wl127x_plat_data,
};
#endif
#endif /* 0 */

/* GPIOS need to be in order of BT, FM and GPS;
 * provide -1 is if EN GPIO not applicable for a core */
#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
#define BT_EN_GPIO 109
#define FM_EN_GPIO 161
#define GPS_EN_GPIO -1
#elif defined (CONFIG_MACH_OMAP_FST_OMAP4_128x)\
              || defined (CONFIG_MACH_OMAP_FST_OMAP4_127x)
#define BT_EN_GPIO 55
#define FM_EN_GPIO -1
#define GPS_EN_GPIO -1
#else
#define BT_EN_GPIO -1
#define FM_EN_GPIO -1
#define GPS_EN_GPIO -1
#endif

static int conn_gpios[] = { BT_EN_GPIO, FM_EN_GPIO, GPS_EN_GPIO };

static struct platform_device conn_device = {
	.name = "kim",		/* named after init manager for ST */
	.id = -1,
	.dev.platform_data = &conn_gpios,
};

static struct platform_device *conn_plat_devices[] __initdata = {
#if 0
#ifdef CONFIG_WL127X_RFKILL
	&zoom2_wl127x_device,
#endif
#endif /* 0 */
	&conn_device,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
static void config_bt_mux_gpio(void)
{
	/* Configure Connectivity GPIOs */
	if (BT_EN_GPIO != -1)
		omap_mux_init_gpio(BT_EN_GPIO, OMAP_PIN_OUTPUT);
	if (FM_EN_GPIO != -1)
		omap_mux_init_gpio(FM_EN_GPIO, OMAP_PIN_OUTPUT);
	if (GPS_EN_GPIO != -1)
		omap_mux_init_gpio(GPS_EN_GPIO, OMAP_PIN_OUTPUT);

	return;
}
#else
static void config_bt_mux_gpio(void)
{
#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
	omap_cfg_reg(B25_34XX_GPIO109);
#elif defined (CONFIG_MACH_OMAP_FST_OMAP4_128x) || \
      defined(CONFIG_MACH_OMAP_FST_OMAP4_127x)
	/* we have no mux-config enum for gpio 55 (BT_EN GPIO) in 2.6.29 */
#endif

	return;
}
#endif

static void config_mux_mcbsp3(void)
{
	/*Mux setting for GPIO164 McBSP3 */
#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
	omap_cfg_reg(H19_34XX_GPIO164_OUT);
#endif
	return;
}

#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
/* Fix to prevent VIO leakage on wl127x */
static int wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	printk("[CONN_CFG] wl127x_vio_leakage_fix\n");

	ret = gpio_request(conn_gpios[0], "wl127x_bten");
	if (ret < 0) {
		printk(KERN_ERR "wl127x_bten gpio_%d request fail",
		       conn_gpios[0]);
		goto fail;
	}

	gpio_direction_output(conn_gpios[0], 1);
	mdelay(10);
	gpio_direction_output(conn_gpios[0], 0);
	udelay(64);

	gpio_free(conn_gpios[0]);
fail:
	return ret;
}
#endif /* MACH_OMAP_FST_OMAP3_127x */

void conn_config_gpios(void)
{
	printk("[CONN_CFG] Configuring Connectivity GPIOs\n");

	config_bt_mux_gpio();
	config_mux_mcbsp3();
}

void conn_add_plat_device(void)
{
	printk("[CONN_CFG] Adding Connectivity platform device\n");

	platform_add_devices(conn_plat_devices, ARRAY_SIZE(conn_plat_devices));
}

void conn_board_init(void)
{
	printk("[CONN_CFG] Connectivity board init\n");

#ifdef CONFIG_MACH_OMAP_FST_OMAP3_127x
	printk("[CONN_CFG] Connectivity board init for OMAP3+WL127x\n");

	wl127x_vio_leakage_fix();
#endif
}
