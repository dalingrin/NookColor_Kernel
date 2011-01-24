/*
 * linux/arch/arm/mach-omap2/board-n800.c
 *
 * Copyright (C) 2005-2007 Nokia Corporation
 * Author: Juha Yrjola <juha.yrjola@nokia.com>
 *
 * Modified from mach-omap2/board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2301.h>
#include <linux/spi/tsc2005.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/lm8323.h>
#include <linux/i2c/menelaus.h>
#include <linux/i2c/lp5521.h>
#include <linux/omapfb.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <mach/usb.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/mcspi.h>
#include <mach/lcd_mipid.h>
#include <mach/clock.h>
#include <mach/gpio-switch.h>
#include <mach/blizzard.h>
#include <mach/display.h>
#include <mach/vram.h>

#include <../drivers/cbus/tahvo.h>
#include <../drivers/media/video/tcm825x.h>

#define N800_BLIZZARD_POWERDOWN_GPIO	15
#define N800_STI_GPIO			62
#define N800_KEYB_IRQ_GPIO		109
#define N800_DAV_IRQ_GPIO		103
#define N800_TSC2301_RESET_GPIO		118

#ifdef CONFIG_MACH_NOKIA_N810
static s16 rx44_keymap[LM8323_KEYMAP_SIZE] = {
	[0x01] = KEY_Q,
	[0x02] = KEY_K,
	[0x03] = KEY_O,
	[0x04] = KEY_P,
	[0x05] = KEY_BACKSPACE,
	[0x06] = KEY_A,
	[0x07] = KEY_S,
	[0x08] = KEY_D,
	[0x09] = KEY_F,
	[0x0a] = KEY_G,
	[0x0b] = KEY_H,
	[0x0c] = KEY_J,

	[0x11] = KEY_W,
	[0x12] = KEY_F4,
	[0x13] = KEY_L,
	[0x14] = KEY_APOSTROPHE,
	[0x16] = KEY_Z,
	[0x17] = KEY_X,
	[0x18] = KEY_C,
	[0x19] = KEY_V,
	[0x1a] = KEY_B,
	[0x1b] = KEY_N,
	[0x1c] = KEY_LEFTSHIFT, /* Actually, this is both shift keys */
	[0x1f] = KEY_F7,

	[0x21] = KEY_E,
	[0x22] = KEY_SEMICOLON,
	[0x23] = KEY_MINUS,
	[0x24] = KEY_EQUAL,
	[0x2b] = KEY_FN,
	[0x2c] = KEY_M,
	[0x2f] = KEY_F8,

	[0x31] = KEY_R,
	[0x32] = KEY_RIGHTCTRL,
	[0x34] = KEY_SPACE,
	[0x35] = KEY_COMMA,
	[0x37] = KEY_UP,
	[0x3c] = KEY_COMPOSE,
	[0x3f] = KEY_F6,

	[0x41] = KEY_T,
	[0x44] = KEY_DOT,
	[0x46] = KEY_RIGHT,
	[0x4f] = KEY_F5,
	[0x51] = KEY_Y,
	[0x53] = KEY_DOWN,
	[0x55] = KEY_ENTER,
	[0x5f] = KEY_ESC,

	[0x61] = KEY_U,
	[0x64] = KEY_LEFT,

	[0x71] = KEY_I,
	[0x75] = KEY_KPENTER,
};

static struct lm8323_platform_data lm8323_pdata = {
	.repeat		= 0, /* Repeat is handled in userspace for now. */
	.keymap		= rx44_keymap,
	.size_x		= 8,
	.size_y		= 8,
	.debounce_time	= 12,
	.active_time	= 500,

	.name		= "Internal keyboard",
	.pwm1_name	= "n810::keyboard",
	.pwm2_name	= "n810::cover",
};
#endif

void __init nokia_n800_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();

#ifdef CONFIG_OMAP_STI
	if (gpio_request(N800_STI_GPIO, "STI") < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for STI\n",
		       N800_STI_GPIO);
		return;
	}

	gpio_direction_output(N800_STI_GPIO, 0);
#endif
}

#if defined(CONFIG_MENELAUS) && defined(CONFIG_SENSORS_TMP105)

static int n800_tmp105_set_power(int enable)
{
	return menelaus_set_vaux(enable ? 2800 : 0);
}

#else

#define n800_tmp105_set_power NULL

#endif

static struct omap_uart_config n800_uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 2),
};

#include "../../../drivers/cbus/retu.h"

static struct omap_tmp105_config n800_tmp105_config __initdata = {
	.tmp105_irq_pin = 125,
	.set_power = n800_tmp105_set_power,
};




/* DISPLAY */
static struct {
	struct clk *sys_ck;
} blizzard;

static int blizzard_get_clocks(void)
{
	blizzard.sys_ck = clk_get(0, "osc_ck");
	if (IS_ERR(blizzard.sys_ck)) {
		printk(KERN_ERR "can't get Blizzard clock\n");
		return PTR_ERR(blizzard.sys_ck);
	}
	return 0;
}

static unsigned long blizzard_get_clock_rate(void)
{
	return clk_get_rate(blizzard.sys_ck);
}

static int n800_pn800_enable(struct omap_display *display)
{
	if (display->hw_config.panel_reset_gpio != -1) {
		printk("enabling panel gpio\n");
		gpio_direction_output(display->hw_config.panel_reset_gpio, 1);
	}

	return 0;
}

static void n800_pn800_disable(struct omap_display *display)
{
	if (display->hw_config.panel_reset_gpio != -1) {
		printk("disabling panel gpio\n");
		gpio_direction_output(display->hw_config.panel_reset_gpio, 0);
		msleep(120);
	}
}

static int n800_blizzard_enable(struct omap_display *display)
{
	printk("enabling bliz powers\n");

	/* Vcore to 1.475V */
	tahvo_set_clear_reg_bits(0x07, 0, 0xf);
	msleep(10);

	clk_enable(blizzard.sys_ck);

	if (display->hw_config.ctrl_reset_gpio != -1)
		gpio_direction_output(display->hw_config.ctrl_reset_gpio, 1);

	printk("osc_ck %lu\n", blizzard_get_clock_rate());

	return 0;
}

static void n800_blizzard_disable(struct omap_display *display)
{
	printk("disabling bliz powers\n");

	if (display->hw_config.ctrl_reset_gpio != -1)
		gpio_direction_output(display->hw_config.ctrl_reset_gpio, 0);

	clk_disable(blizzard.sys_ck);

	/* Vcore to 1.005V */
	tahvo_set_clear_reg_bits(0x07, 0xf, 0);
}

static int n800_set_backlight_level(struct omap_display *display, int level)
{
	return 0;
}

static struct omap_dss_display_config n800_dsi_display_data = {
	.type = OMAP_DISPLAY_TYPE_DBI,
	.name = "lcd",
	.ctrl_name = "ctrl-blizzard",
	.panel_name = "panel-pn800",
	.panel_reset_gpio = -1,
	.ctrl_reset_gpio = N800_BLIZZARD_POWERDOWN_GPIO,
	.panel_enable = n800_pn800_enable,
	.panel_disable = n800_pn800_disable,
	.ctrl_enable = n800_blizzard_enable,
	.ctrl_disable = n800_blizzard_disable,
	.set_backlight = n800_set_backlight_level,
	.u.rfbi = {
		.channel = 0,
		/* 8 for cmd mode, 16 for pixel data. ctrl-blizzard handles switching */
		.data_lines = 8,
	},
	.panel_data = 0, // XXX used for panel datalines
};
static struct omap_dss_board_info n800_dss_data = {
	.num_displays = 1,
	.displays = {
		&n800_dsi_display_data,
	},
};

static struct platform_device n800_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &n800_dss_data,
	},
};

static void __init n800_display_init(void)
{
	int r;
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		n800_dsi_display_data.panel_reset_gpio = conf->nreset_gpio;
		n800_dsi_display_data.panel_data =
			(void*)(u32)conf->data_lines; // XXX
		//printk("\n\nTULI %d\n\n", conf->data_lines);
	} else {
		printk("\n\nEI TULLU MIOTÄÄÄ\n\n");
	}

	blizzard_get_clocks();
	clk_enable(blizzard.sys_ck); // XXX always enable

	//omapfb_set_ctrl_platform_data(&n800_blizzard_data);
	//
	if (n800_dsi_display_data.ctrl_reset_gpio != -1) {
		r = gpio_request(n800_dsi_display_data.ctrl_reset_gpio,
				"Blizzard pd");
		if (r < 0) {
			n800_dsi_display_data.ctrl_reset_gpio = -1;
			printk(KERN_ERR "Unable to get Blizzard GPIO\n");
		} else {
			gpio_direction_output(n800_dsi_display_data.ctrl_reset_gpio,
					1);
			// XXX always enable
		}
	}

	if (n800_dsi_display_data.panel_reset_gpio != -1) {
		r = gpio_request(n800_dsi_display_data.panel_reset_gpio,
				"panel reset");
		if (r < 0) {
			n800_dsi_display_data.panel_reset_gpio = -1;
			printk(KERN_ERR "Unable to get pn800 GPIO\n");
		} else {
			gpio_direction_output(n800_dsi_display_data.panel_reset_gpio,
					1);
			// XXX always enable
		}
	}
}

/* DISPLAY END */





static void mipid_shutdown(struct mipid_platform_data *pdata)
{
	if (pdata->nreset_gpio != -1) {
		pr_info("shutdown LCD\n");
		gpio_set_value(pdata->nreset_gpio, 0);
		msleep(120);
	}
}

static struct mipid_platform_data n800_mipid_platform_data = {
	.shutdown = mipid_shutdown,
};

#if 0
static void __init mipid_dev_init(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		n800_mipid_platform_data.nreset_gpio = conf->nreset_gpio;
		n800_mipid_platform_data.data_lines = conf->data_lines;
	}
}
#endif

#if 0
static void blizzard_enable_clocks(int enable)
{
	if (enable)
		clk_enable(blizzard.sys_ck);
	else
		clk_disable(blizzard.sys_ck);
}

static void blizzard_power_up(struct device *dev)
{
	/* Vcore to 1.475V */
	tahvo_set_clear_reg_bits(0x07, 0, 0xf);
	msleep(10);

	blizzard_enable_clocks(1);
	gpio_set_value(N800_BLIZZARD_POWERDOWN_GPIO, 1);
}

static void blizzard_power_down(struct device *dev)
{
	gpio_set_value(N800_BLIZZARD_POWERDOWN_GPIO, 0);
	blizzard_enable_clocks(0);

	/* Vcore to 1.005V */
	tahvo_set_clear_reg_bits(0x07, 0xf, 0);
}

static struct blizzard_platform_data n800_blizzard_data = {
	.power_up	= blizzard_power_up,
	.power_down	= blizzard_power_down,
	.get_clock_rate	= blizzard_get_clock_rate,
	.te_connected	= 1,
};

static void __init blizzard_dev_init(void)
{
	int r;

	r = gpio_request(N800_BLIZZARD_POWERDOWN_GPIO, "Blizzard pd");
	if (r < 0)
		return;
	gpio_direction_output(N800_BLIZZARD_POWERDOWN_GPIO, 1);

	blizzard_get_clocks();
	//omapfb_set_ctrl_platform_data(&n800_blizzard_data);
}
#endif

static struct omap_board_config_kernel n800_config[] __initdata = {
	{ OMAP_TAG_UART,	                &n800_uart_config },
	{ OMAP_TAG_TMP105,			&n800_tmp105_config },
};

static struct tsc2301_platform_data tsc2301_config = {
	.reset_gpio	= N800_TSC2301_RESET_GPIO,
	.keymap = {
		-1,		/* Event for bit 0 */
		KEY_UP,		/* Event for bit 1 (up) */
		KEY_F5,		/* Event for bit 2 (home) */
		-1,		/* Event for bit 3 */
		KEY_LEFT,	/* Event for bit 4 (left) */
		KEY_ENTER,	/* Event for bit 5 (enter) */
		KEY_RIGHT,	/* Event for bit 6 (right) */
		-1,		/* Event for bit 7 */
		KEY_ESC,	/* Event for bit 8 (cycle) */
		KEY_DOWN,	/* Event for bit 9 (down) */
		KEY_F4,		/* Event for bit 10 (menu) */
		-1,		/* Event for bit 11 */
		KEY_F8,		/* Event for bit 12 (Zoom-) */
		KEY_F6,		/* Event for bit 13 (FS) */
		KEY_F7,		/* Event for bit 14 (Zoom+) */
		-1,		/* Event for bit 15 */
	},
	.kp_rep 	= 0,
	.keyb_name	= "Internal keypad",
};

static void tsc2301_dev_init(void)
{
	int r;
	int gpio = N800_KEYB_IRQ_GPIO;

	r = gpio_request(gpio, "tsc2301 KBD IRQ");
	if (r >= 0) {
		gpio_direction_input(gpio);
		tsc2301_config.keyb_int = gpio_to_irq(gpio);
	} else {
		printk(KERN_ERR "unable to get KBD GPIO");
	}

	gpio = N800_DAV_IRQ_GPIO;
	r = gpio_request(gpio, "tsc2301 DAV IRQ");
	if (r >= 0) {
		gpio_direction_input(gpio);
		tsc2301_config.dav_int = gpio_to_irq(gpio);
	} else {
		printk(KERN_ERR "unable to get DAV GPIO");
	}
}

static int __init tea5761_dev_init(void)
{
	const struct omap_tea5761_config *info;
	int enable_gpio = 0;

	info = omap_get_config(OMAP_TAG_TEA5761, struct omap_tea5761_config);
	if (info)
		enable_gpio = info->enable_gpio;

	if (enable_gpio) {
		pr_debug("Enabling tea5761 at GPIO %d\n",
			 enable_gpio);

		if (gpio_request(enable_gpio, "TEA5761 enable") < 0) {
			printk(KERN_ERR "Can't request GPIO %d\n",
			       enable_gpio);
			return -ENODEV;
		}

		gpio_direction_output(enable_gpio, 0);
		udelay(50);
		gpio_set_value(enable_gpio, 1);
	}

	return 0;
}

static struct omap2_mcspi_device_config tsc2301_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
	.mode		= OMAP2_MCSPI_MASTER,
	.dma_mode	= 0,
	.force_cs_mode	= 0,
	.fifo_depth	= 0,
};

static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
	.mode		= OMAP2_MCSPI_MASTER,
	.dma_mode	= 0,
	.force_cs_mode	= 0,
	.fifo_depth	= 0,
};

static struct omap2_mcspi_device_config cx3110x_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
	.mode		= OMAP2_MCSPI_MASTER,
	.dma_mode	= 0,
	.force_cs_mode	= 0,
	.fifo_depth	= 0,
};

#ifdef CONFIG_TOUCHSCREEN_TSC2005
static struct tsc2005_platform_data tsc2005_config = {
	.reset_gpio = 94,
	.dav_gpio = 106
};

static struct omap2_mcspi_device_config tsc2005_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
	.mode		= OMAP2_MCSPI_MASTER,
	.dma_mode	= 0,
	.force_cs_mode	= 0,
	.fifo_depth	= 0,
};
#endif

static struct spi_board_info n800_spi_board_info[] __initdata = {
	{
		.modalias	= "panel-n800",
		.bus_num	= 1,
		.chip_select	= 1,
		.max_speed_hz	= 4000000,
		.controller_data= &mipid_mcspi_config,
		.platform_data	= &n800_mipid_platform_data,
	}, {
		.modalias	= "cx3110x",
		.bus_num	= 2,
		.chip_select	= 0,
		.max_speed_hz   = 48000000,
		.controller_data= &cx3110x_mcspi_config,
	},
	{
		.modalias	= "tsc2301",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz   = 6000000,
		.controller_data= &tsc2301_mcspi_config,
		.platform_data  = &tsc2301_config,
	},
};

static struct spi_board_info n810_spi_board_info[] __initdata = {
	{
		.modalias	 = "panel-n800",
		.bus_num	 = 1,
		.chip_select	 = 1,
		.max_speed_hz	 = 4000000,
		.controller_data = &mipid_mcspi_config,
		.platform_data	 = &n800_mipid_platform_data,
	},
	{
		.modalias	 = "cx3110x",
		.bus_num	 = 2,
		.chip_select	 = 0,
		.max_speed_hz    = 48000000,
		.controller_data = &cx3110x_mcspi_config,
	},
	{
		.modalias	 = "tsc2005",
		.bus_num	 = 1,
		.chip_select	 = 0,
		.max_speed_hz    = 6000000,
		.controller_data = &tsc2005_mcspi_config,
		.platform_data   = &tsc2005_config,
	},
};

static void __init tsc2005_set_config(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
#ifdef CONFIG_TOUCHSCREEN_TSC2005
		if (strcmp(conf->panel_name, "lph8923") == 0) {
			tsc2005_config.ts_x_plate_ohm = 180;
			tsc2005_config.ts_hw_avg = 0;
			tsc2005_config.ts_ignore_last = 0;
			tsc2005_config.ts_touch_pressure = 1500;
			tsc2005_config.ts_stab_time = 100;
			tsc2005_config.ts_pressure_max = 2048;
			tsc2005_config.ts_pressure_fudge = 2;
			tsc2005_config.ts_x_max = 4096;
			tsc2005_config.ts_x_fudge = 4;
			tsc2005_config.ts_y_max = 4096;
			tsc2005_config.ts_y_fudge = 7;
		} else if (strcmp(conf->panel_name, "ls041y3") == 0) {
			tsc2005_config.ts_x_plate_ohm = 280;
			tsc2005_config.ts_hw_avg = 0;
			tsc2005_config.ts_ignore_last = 0;
			tsc2005_config.ts_touch_pressure = 1500;
			tsc2005_config.ts_stab_time = 1000;
			tsc2005_config.ts_pressure_max = 2048;
			tsc2005_config.ts_pressure_fudge = 2;
			tsc2005_config.ts_x_max = 4096;
			tsc2005_config.ts_x_fudge = 4;
			tsc2005_config.ts_y_max = 4096;
			tsc2005_config.ts_y_fudge = 7;
		} else {
			printk(KERN_ERR "Unknown panel type, set default "
			       "touchscreen configuration\n");
			tsc2005_config.ts_x_plate_ohm = 200;
			tsc2005_config.ts_stab_time = 100;
		}
#endif
	}
}

#if defined(CONFIG_CBUS_RETU) && defined(CONFIG_LEDS_OMAP_PWM)

void retu_keypad_led_set_power(struct omap_pwm_led_platform_data *self,
			       int on_off)
{
	if (on_off) {
		retu_write_reg(RETU_REG_CTRL_SET, 1 << 6);
		msleep(2);
		retu_write_reg(RETU_REG_CTRL_SET, 1 << 3);
	} else {
		retu_write_reg(RETU_REG_CTRL_CLR, (1 << 6) | (1 << 3));
	}
}

static struct omap_pwm_led_platform_data n800_keypad_led_data = {
	.name = "keypad",
	.intensity_timer = 10,
	.blink_timer = 9,
	.set_power = retu_keypad_led_set_power,
};

static struct platform_device n800_keypad_led_device = {
	.name		= "omap_pwm_led",
	.id		= -1,
	.dev		= {
		.platform_data = &n800_keypad_led_data,
	},
};
#endif

#if defined(CONFIG_TOUCHSCREEN_TSC2301)
static void __init n800_ts_set_config(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		if (strcmp(conf->panel_name, "lph8923") == 0) {
			tsc2301_config.ts_x_plate_ohm	= 180;
			tsc2301_config.ts_hw_avg	= 8;
			tsc2301_config.ts_max_pressure	= 2048;
			tsc2301_config.ts_touch_pressure = 400;
			tsc2301_config.ts_stab_time	= 100;
			tsc2301_config.ts_pressure_fudge = 2;
			tsc2301_config.ts_x_max		= 4096;
			tsc2301_config.ts_x_fudge	= 4;
			tsc2301_config.ts_y_max		= 4096;
			tsc2301_config.ts_y_fudge	= 7;
		} else if (strcmp(conf->panel_name, "ls041y3") == 0) {
			tsc2301_config.ts_x_plate_ohm	= 280;
			tsc2301_config.ts_hw_avg	= 8;
			tsc2301_config.ts_touch_pressure = 400;
			tsc2301_config.ts_max_pressure	= 2048;
			tsc2301_config.ts_stab_time	= 1000;
			tsc2301_config.ts_pressure_fudge = 2;
			tsc2301_config.ts_x_max		= 4096;
			tsc2301_config.ts_x_fudge	= 4;
			tsc2301_config.ts_y_max		= 4096;
			tsc2301_config.ts_y_fudge	= 7;
		} else {
			printk(KERN_ERR "Unknown panel type, set default "
			       "touchscreen configuration\n");
			tsc2301_config.ts_x_plate_ohm	= 200;
			tsc2301_config.ts_stab_time	= 100;
		}
	}
}
#else
static inline void n800_ts_set_config(void)
{
}
#endif

static struct omap_gpio_switch n800_gpio_switches[] __initdata = {
	{
		.name			= "bat_cover",
		.gpio			= -1,
		.debounce_rising	= 100,
		.debounce_falling	= 0,
		.notify			= n800_mmc_slot1_cover_handler,
		.notify_data		= NULL,
	}, {
		.name			= "headphone",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
	}, {
		.name			= "cam_act",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
	}, {
		.name			= "cam_turn",
		.gpio			= -1,
		.debounce_rising	= 100,
		.debounce_falling	= 100,
	},
};

#if defined(CONFIG_CBUS_RETU_HEADSET)
static struct platform_device retu_headset_device = {
	.name	= "retu-headset",
	.id	= -1,
};
#endif

static struct platform_device *n800_devices[] __initdata = {
#if defined(CONFIG_CBUS_RETU) && defined(CONFIG_LEDS_OMAP_PWM)
	&n800_keypad_led_device,
#endif
#if defined(CONFIG_CBUS_RETU_HEADSET)
	&retu_headset_device,
#endif
	&n800_dss_device,
};

#ifdef CONFIG_MENELAUS
static int n800_auto_sleep_regulators(void)
{
	u32 val;
	int ret;

	val = EN_VPLL_SLEEP | EN_VMMC_SLEEP    \
		| EN_VAUX_SLEEP | EN_VIO_SLEEP \
		| EN_VMEM_SLEEP | EN_DC3_SLEEP \
		| EN_VC_SLEEP | EN_DC2_SLEEP;

	ret = menelaus_set_regulator_sleep(1, val);
	if (ret < 0) {
		printk(KERN_ERR "Could not set regulators to sleep on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n800_auto_voltage_scale(void)
{
	int ret;

	ret = menelaus_set_vcore_hw(1400, 1050);
	if (ret < 0) {
		printk(KERN_ERR "Could not set VCORE voltage on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n800_menelaus_init(struct device *dev)
{
	int ret;

	ret = n800_auto_voltage_scale();
	if (ret < 0)
		return ret;
	ret = n800_auto_sleep_regulators();
	if (ret < 0)
		return ret;
	return 0;
}

static struct menelaus_platform_data n800_menelaus_platform_data = {
	.late_init = n800_menelaus_init,
};
#endif

static struct i2c_board_info __initdata n800_i2c_board_info_1[] = {
	{
		I2C_BOARD_INFO("menelaus", 0x72),
		.irq = INT_24XX_SYS_NIRQ,
		.platform_data = &n800_menelaus_platform_data,
	},
};

static struct lp5521_platform_data n810_lp5521_platform_data = {
	.mode		= LP5521_MODE_DIRECT_CONTROL,
	.label		= "n810",
	.red_present	= true,
	.green_present	= true,
	.blue_present	= true,
};

extern struct tcm825x_platform_data n800_tcm825x_platform_data;

static struct i2c_board_info __initdata_or_module n8x0_i2c_board_info_2[] = {
	{
		I2C_BOARD_INFO(TCM825X_NAME, TCM825X_I2C_ADDR),
#if defined (CONFIG_VIDEO_TCM825X) || defined (CONFIG_VIDEO_TCM825X_MODULE)
		.platform_data = &n800_tcm825x_platform_data,
#endif
	},
};


static struct i2c_board_info __initdata_or_module n800_i2c_board_info_2[] = {
	{
		I2C_BOARD_INFO("tea5761", 0x10),
	},
};

static struct i2c_board_info __initdata_or_module n810_i2c_board_info_2[] = {
	{
		I2C_BOARD_INFO("lm8323", 0x45),
		.irq		= OMAP_GPIO_IRQ(109),
		.platform_data	= &lm8323_pdata,
	},
	{
		I2C_BOARD_INFO("tsl2563", 0x29),
	},
	{
		I2C_BOARD_INFO("lp5521", 0x32),
		.platform_data = &n810_lp5521_platform_data,
	},
};

void __init nokia_n800_common_init(void)
{
	platform_add_devices(n800_devices, ARRAY_SIZE(n800_devices));

	n800_flash_init();
	n800_mmc_init();
	n800_bt_init();
	n800_dsp_init();
	n800_usb_init();
	n800_cam_init();
	if (machine_is_nokia_n800())
		spi_register_board_info(n800_spi_board_info,
				ARRAY_SIZE(n800_spi_board_info));
	if (machine_is_nokia_n810()) {
		tsc2005_set_config();
		spi_register_board_info(n810_spi_board_info,
				ARRAY_SIZE(n810_spi_board_info));
	}
	omap_serial_init();
	omap_register_i2c_bus(1, 400, n800_i2c_board_info_1,
			      ARRAY_SIZE(n800_i2c_board_info_1));
	omap_register_i2c_bus(2, 400, n8x0_i2c_board_info_2,
			      ARRAY_SIZE(n8x0_i2c_board_info_2));
	if (machine_is_nokia_n800())
		i2c_register_board_info(2, n800_i2c_board_info_2,
			ARRAY_SIZE(n800_i2c_board_info_2));
	if (machine_is_nokia_n810())
		i2c_register_board_info(2, n810_i2c_board_info_2,
			ARRAY_SIZE(n810_i2c_board_info_2));

	//mipid_dev_init();
	//blizzard_dev_init();
	n800_display_init();
}

static void __init nokia_n800_init(void)
{
	nokia_n800_common_init();

	n800_ts_set_config();
	tsc2301_dev_init();
	tea5761_dev_init();
	omap_register_gpio_switches(n800_gpio_switches,
				    ARRAY_SIZE(n800_gpio_switches));
}

void __init nokia_n800_map_io(void)
{
	omap_board_config = n800_config;
	omap_board_config_size = ARRAY_SIZE(n800_config);

	omap2_set_globals_242x();
	omap2_set_sdram_vram(800 * 480 * 2 * 3, 0);
	omap2_map_common_io();
}

MACHINE_START(NOKIA_N800, "Nokia N800")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= nokia_n800_map_io,
	.init_irq	= nokia_n800_init_irq,
	.init_machine	= nokia_n800_init,
	.timer		= &omap_timer,
MACHINE_END
