/*
 * linux/arch/arm/mach-omap2/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl4030.h>
#include <linux/regulator/machine.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/usb.h>
#include <mach/common.h>
#include <mach/dma.h>
#include <mach/gpmc.h>
#include <mach/display.h>
#include <mach/omap-pm.h>

#include <mach/control.h>
#include <mach/clock.h>
#include "sdram-qimonda-hyb18m512160af-6.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"

#define CONFIG_DISABLE_HFCLK 1

#define SDP3430_ETHR_START		DEBUG_BASE
#define SDP3430_ETHR_GPIO_IRQ_SDPV1	29
#define SDP3430_ETHR_GPIO_IRQ_SDPV2	6
#define SDP3430_SMC91X_CS		3

#define SDP3430_TS_GPIO_IRQ_SDPV1	3
#define SDP3430_TS_GPIO_IRQ_SDPV2	2

#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20

#define TWL4030_MSECURE_GPIO 22

static struct resource sdp3430_smc91x_resources[] = {
	[0] = {
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device sdp3430_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sdp3430_smc91x_resources),
	.resource	= sdp3430_smc91x_resources,
};

static int sdp3430_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(0, 4, KEY_C),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(1, 4, KEY_G),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(2, 4, KEY_3),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P),
	KEY(3, 4, KEY_Q),
	KEY(4, 0, KEY_R),
	KEY(4, 1, KEY_4),
	KEY(4, 2, KEY_T),
	KEY(4, 3, KEY_U),
	KEY(4, 4, KEY_D),
	KEY(5, 0, KEY_V),
	KEY(5, 1, KEY_W),
	KEY(5, 2, KEY_L),
	KEY(5, 3, KEY_S),
	KEY(5, 4, KEY_H),
	0
};

static struct twl4030_keypad_data sdp3430_kp_data = {
	.rows		= 5,
	.cols		= 6,
	.keymap		= sdp3430_keymap,
	.keymapsize	= ARRAY_SIZE(sdp3430_keymap),
	.rep		= 1,
};

static int ts_gpio;	/* Needed for ads7846_get_pendown_state */

static int __init msecure_init(void)
{
	int ret = 0;

#ifdef CONFIG_RTC_DRV_TWL4030
	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (omap_type() == OMAP2_DEVICE_TYPE_GP &&
			omap_rev() < OMAP3430_REV_ES2_0) {
		void __iomem *msecure_pad_config_reg = omap_ctrl_base_get() +
			0xA3C;
		int mux_mask = 0x04;
		u16 tmp;

		ret = gpio_request(TWL4030_MSECURE_GPIO, "msecure");
		if (ret < 0) {
			printk(KERN_ERR "msecure_init: can't"
				"reserve GPIO:%d !\n", TWL4030_MSECURE_GPIO);
			goto out;
		}
		/*
		 * TWL4030 will be in secure mode if msecure line from OMAP
		 * is low. Make msecure line high in order to change the
		 * TWL4030 RTC time and calender registers.
		 */
		tmp = __raw_readw(msecure_pad_config_reg);
		tmp &= 0xF8; /* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		__raw_writew(tmp, msecure_pad_config_reg);

		gpio_direction_output(TWL4030_MSECURE_GPIO, 1);
	}
out:
#endif
	return ret;
}

/**
 * @brief ads7846_dev_init : Requests & sets GPIO line for pen-irq
 *
 * @return - void. If request gpio fails then Flag KERN_ERR.
 */
static void ads7846_dev_init(void)
{
	if (gpio_request(ts_gpio, "ADS7846 pendown") < 0) {
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");
		return;
	}

	gpio_direction_input(ts_gpio);

	omap_set_gpio_debounce(ts_gpio, 1);
	omap_set_gpio_debounce_time(ts_gpio, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(ts_gpio);
}

/*
 * This enable(1)/disable(0) the voltage for TS: uses twl4030 calls
 */
static int ads7846_vaux_control(int vaux_cntrl)
{
	int ret = 0;

	/* FIXME use regulator calls */

#ifdef CONFIG_TWL4030_CORE
	/* check for return value of ldo_use: if success it returns 0 */
	if (vaux_cntrl == VAUX_ENABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX3_DEDICATED, TWL4030_VAUX3_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX3_DEV_GRP, TWL4030_VAUX3_DEV_GRP))
			return -EIO;
	} else if (vaux_cntrl == VAUX_DISABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX3_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX3_DEV_GRP))
			return -EIO;
	}
#else
	ret = -EIO;
#endif
	return ret;
}

static struct ads7846_platform_data tsc2046_config __initdata = {
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.vaux_control		= ads7846_vaux_control,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
	.mode		= OMAP2_MCSPI_MASTER,
	.dma_mode	= 0,
	.force_cs_mode	= 0,
	.fifo_depth	= 0,
};

static struct spi_board_info sdp3430_spi_board_info[] __initdata = {
	[0] = {
		/*
		 * TSC2046 operates at a max freqency of 2MHz, so
		 * operate slightly below at 1.5MHz
		 */
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &tsc2046_mcspi_config,
		.irq			= 0,
		.platform_data		= &tsc2046_config,
	},
};


#define SDP3430_LCD_PANEL_BACKLIGHT_GPIO	8
#define SDP3430_LCD_PANEL_ENABLE_GPIO		5

static unsigned backlight_gpio;
static unsigned enable_gpio;
static int lcd_enabled;
static int dvi_enabled;

static void __init sdp3430_display_init(void)
{
	int r;

	enable_gpio    = SDP3430_LCD_PANEL_ENABLE_GPIO;
	backlight_gpio = SDP3430_LCD_PANEL_BACKLIGHT_GPIO;

	r = gpio_request(enable_gpio, "LCD reset");
	if (r) {
		printk(KERN_ERR "failed to get LCD reset GPIO\n");
		goto err0;
	}

	r = gpio_request(backlight_gpio, "LCD Backlight");
	if (r) {
		printk(KERN_ERR "failed to get LCD backlight GPIO\n");
		goto err1;
	}

	gpio_direction_output(enable_gpio, 0);
	gpio_direction_output(backlight_gpio, 0);

	return;
err1:
	gpio_free(enable_gpio);
err0:
	return;
}

static int sdp3430_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	gpio_direction_output(enable_gpio, 1);
	gpio_direction_output(backlight_gpio, 1);

	lcd_enabled = 1;

	return 0;
}

static void sdp3430_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	lcd_enabled = 0;

	gpio_direction_output(enable_gpio, 0);
	gpio_direction_output(backlight_gpio, 0);
}

static int sdp3430_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	dvi_enabled = 1;

	return 0;
}

static void sdp3430_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	dvi_enabled = 0;
}

static int sdp3430_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void sdp3430_panel_disable_tv(struct omap_dss_device *dssdev)
{
}


static struct omap_dss_device sdp3430_lcd_device = {
	.name = "lcd",
	.driver_name = "sharp_ls_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 16,
	.platform_enable = sdp3430_panel_enable_lcd,
	.platform_disable = sdp3430_panel_disable_lcd,
};

static struct omap_dss_device sdp3430_dvi_device = {
	.name = "dvi",
	.driver_name = "generic_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable = sdp3430_panel_enable_dvi,
	.platform_disable = sdp3430_panel_disable_dvi,
};

static struct omap_dss_device sdp3430_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable = sdp3430_panel_enable_tv,
	.platform_disable = sdp3430_panel_disable_tv,
};


static struct omap_dss_device *sdp3430_dss_devices[] = {
	&sdp3430_lcd_device,
	&sdp3430_dvi_device,
	&sdp3430_tv_device,
};

static struct omap_dss_board_info sdp3430_dss_data = {
	.num_devices = ARRAY_SIZE(sdp3430_dss_devices),
	.devices = sdp3430_dss_devices,
	.default_device = &sdp3430_lcd_device,
};

static struct platform_device sdp3430_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &sdp3430_dss_data,
	},
};

static struct regulator_consumer_supply sdp3430_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &sdp3430_dss_device.dev,
};

static struct platform_device *sdp3430_devices[] __initdata = {
	&sdp3430_smc91x_device,
	&sdp3430_dss_device,
};

static inline void __init sdp3430_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	eth_cs = SDP3430_SMC91X_CS;

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		return;
	}

	sdp3430_smc91x_resources[0].start = cs_mem_base + 0x300;
	sdp3430_smc91x_resources[0].end = cs_mem_base + 0x30f;
	udelay(100);

	if (omap_rev() > OMAP3430_REV_ES1_0)
		eth_gpio = SDP3430_ETHR_GPIO_IRQ_SDPV2;
	else
		eth_gpio = SDP3430_ETHR_GPIO_IRQ_SDPV1;

	sdp3430_smc91x_resources[1].start = gpio_to_irq(eth_gpio);

	if (gpio_request(eth_gpio, "SMC91x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			eth_gpio);
		return;
	}
	gpio_direction_input(eth_gpio);
}

static void __init omap_3430sdp_init_irq(void)
{
	switch (omap_rev_id()) {
	case OMAP_3420:
		omap2_init_common_hw(hyb18m512160af6_sdrc_params,
		omap3_mpu_rate_table, omap3_dsp_rate_table_3420,
		omap3_l3_rate_table);
		break;

	case OMAP_3430:
		omap2_init_common_hw(hyb18m512160af6_sdrc_params,
		omap3_mpu_rate_table, omap3_dsp_rate_table,
		omap3_l3_rate_table);
		break;

	case OMAP_3440:
		omap2_init_common_hw(hyb18m512160af6_sdrc_params,
		omap3_mpu_rate_table, omap3_dsp_rate_table_3440,
		omap3_l3_rate_table);
		break;

	default:
		omap2_init_common_hw(hyb18m512160af6_sdrc_params,
		omap3_mpu_rate_table, omap3_dsp_rate_table,
		omap3_l3_rate_table);
		break;
		}
	omap_init_irq();
	omap_gpio_init();
	sdp3430_init_smc91x();
}

static struct omap_uart_config sdp3430_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel sdp3430_config[] __initdata = {
	{ OMAP_TAG_UART,	&sdp3430_uart_config },
};

static int sdp3430_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,   9280,   8950,   8620,   8310,
8020,   7730,   7460,   7200,   6950,   6710,   6470,   6250,   6040,   5830,
5640,   5450,   5260,   5090,   4920,   4760,   4600,   4450,   4310,   4170,
4040,   3910,   3790,   3670,   3550
};

static struct twl4030_bci_platform_data sdp3430_bci_data = {
	.battery_tmp_tbl	= sdp3430_batt_table,
	.tblsize		= ARRAY_SIZE(sdp3430_batt_table),
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		/* 8 bits (default) requires S6.3 == ON,
		 * so the SIM card isn't used; else 4 bits.
		 */
		.wires		= 8,
		.gpio_wp	= 4,
	},
	{
		.mmc		= 2,
		.wires		= 8,
		.gpio_wp	= 7,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply sdp3430_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply sdp3430_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct regulator_consumer_supply sdp3430_vmmc2_supply = {
	.supply			= "vmmc",
};

static int sdp3430_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
	mmc[0].gpio_cd = gpio + 0;
	mmc[1].gpio_cd = gpio + 1;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	 */
	sdp3430_vmmc1_supply.dev = mmc[0].dev;
	sdp3430_vsim_supply.dev = mmc[0].dev;
	sdp3430_vmmc2_supply.dev = mmc[1].dev;

	/* gpio + 7 is "sub_lcd_en_bkl" (output/PWM1) */
	gpio_request(gpio + 7, "sub_lcd_en_bkl");
	gpio_direction_output(gpio + 7, 0);

	/* gpio + 15 is "sub_lcd_nRST" (output) */
	gpio_request(gpio + 15, "sub_lcd_nRST");
	gpio_direction_output(gpio + 15, 0);

	return 0;
}

static struct twl4030_gpio_platform_data sdp3430_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.pulldowns	= BIT(2) | BIT(6) | BIT(8) | BIT(13)
				| BIT(16) | BIT(17),
	.setup		= sdp3430_twl_gpio_setup,
};

static struct twl4030_usb_data sdp3430_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_madc_platform_data sdp3430_madc_data = {
	.irq_line	= 1,
};


static struct twl4030_ins __initdata sleep_on_seq[] = {
/*
 * Turn off VDD1 and VDD2.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 4},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
#ifdef CONFIG_DISABLE_HFCLK
/*
 * And also turn off the OMAP3 PLLs and the sysclk output.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 3},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 3},
#endif
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TRITON_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_seq[] __initdata = {
#ifndef CONFIG_DISABLE_HFCLK
/*
 * Wakeup VDD1 and VDD2.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 4},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
#else
/*
 * Reenable the OMAP3 PLLs.
 * Wakeup VDD1 and VDD2.
 * Reenable sysclk output.
 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 0x30},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 0x30},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 0x37},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 3},
#endif /* #ifndef CONFIG_DISABLE_HFCLK */
};

static struct twl4030_script wakeup_script __initdata = {
	.script	= wakeup_seq,
	.size	= ARRAY_SIZE(wakeup_seq),
	.flags	= TRITON_WAKEUP12_SCRIPT | TRITON_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};
static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wakeup_seq),
	.flags  = TRITON_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_script,
	&wrst_script,
};

static struct twl4030_power_data sdp3430_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.size		= ARRAY_SIZE(twl4030_scripts),
};

/*
 * Apply all the fixed voltages since most versions of U-Boot
 * don't bother with that initialization.
 */

/* VAUX1 for mainboard (irda and sub-lcd) */
static struct regulator_init_data sdp3430_vaux1 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VAUX2 for camera module */
static struct regulator_init_data sdp3430_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VAUX3 for LCD board */
static struct regulator_init_data sdp3430_vaux3 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VAUX4 for OMAP VDD_CSI2 (camera) */
static struct regulator_init_data sdp3430_vaux4 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data sdp3430_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sdp3430_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data sdp3430_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sdp3430_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data sdp3430_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sdp3430_vsim_supply,
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data sdp3430_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &sdp3430_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply sdp3430_vpll2_supplies[] = {
	{
		.supply		= "vdvi",
		.dev		= &sdp3430_lcd_device.dev,
	},
	{
		.supply		= "vdds_dsi",
		.dev		= &sdp3430_dss_device.dev,
	}
};

static struct regulator_init_data sdp3430_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(sdp3430_vpll2_supplies),
	.consumer_supplies	= sdp3430_vpll2_supplies,
};

static struct twl4030_platform_data sdp3430_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &sdp3430_bci_data,
	.gpio		= &sdp3430_gpio_data,
	.madc		= &sdp3430_madc_data,
	.keypad		= &sdp3430_kp_data,
	.power		= &sdp3430_t2scripts_data,
	.usb		= &sdp3430_usb_data,

	.vaux1		= &sdp3430_vaux1,
	.vaux2		= &sdp3430_vaux2,
	.vaux3		= &sdp3430_vaux3,
	.vaux4		= &sdp3430_vaux4,
	.vmmc1		= &sdp3430_vmmc1,
	.vmmc2		= &sdp3430_vmmc2,
	.vsim		= &sdp3430_vsim,
	.vdac		= &sdp3430_vdac,
	.vpll2		= &sdp3430_vpll2,
};

static struct i2c_board_info __initdata sdp3430_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &sdp3430_twldata,
	},
};

/* I2C Address for ISP1301 Transceiver */
#define ISP1301_I2C_ADDR1               0x2C
#define ISP1301_I2C_ADDR2               0x2D

static struct i2c_board_info __initdata sdp3430_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("isp1301_host", ISP1301_I2C_ADDR1),
		.type           = "isp1301_host",
	},
	{
		I2C_BOARD_INFO("isp1301_host", ISP1301_I2C_ADDR2),
		.type           = "isp1301_host",
	},
};

static int __init omap3430_i2c_init(void)
{
	/* i2c1 for PMIC only */
	omap_register_i2c_bus(1, 2600, sdp3430_i2c1_boardinfo,
			ARRAY_SIZE(sdp3430_i2c1_boardinfo));
	/* i2c2 optional isp1301 */
	omap_register_i2c_bus(2, 100, sdp3430_i2c2_boardinfo,
			ARRAY_SIZE(sdp3430_i2c2_boardinfo));
	/* i2c3 on display connector (for DVI, tfp410) */
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void __init omap_3430sdp_init(void)
{
	omap3430_i2c_init();
	platform_add_devices(sdp3430_devices, ARRAY_SIZE(sdp3430_devices));
	omap_board_config = sdp3430_config;
	omap_board_config_size = ARRAY_SIZE(sdp3430_config);
	if (omap_rev() > OMAP3430_REV_ES1_0)
		ts_gpio = SDP3430_TS_GPIO_IRQ_SDPV2;
	else
		ts_gpio = SDP3430_TS_GPIO_IRQ_SDPV1;
	sdp3430_spi_board_info[0].irq = gpio_to_irq(ts_gpio);
	spi_register_board_info(sdp3430_spi_board_info,
				ARRAY_SIZE(sdp3430_spi_board_info));
	ads7846_dev_init();
	sdp3430_flash_init();
	msecure_init();
	omap_serial_init();
	usb_musb_init();
	usb_ehci_init();
	usb_ohci_init();
	sdp3430_display_init();
}

static void __init omap_3430sdp_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_3430SDP, "OMAP3430 3430SDP board")
	/* Maintainer: Syed Khasim - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_3430sdp_map_io,
	.init_irq	= omap_3430sdp_init_irq,
	.init_machine	= omap_3430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
