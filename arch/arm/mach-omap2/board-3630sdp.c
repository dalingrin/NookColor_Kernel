/*
 * linux/arch/arm/mach-omap2/board-sdp3630.c
 *
 * Copyright (C) 2009 Texas Instruments Inc.
 * Harish Hanumanthappa <h-harish@ti.com>
 *
 * Modified from mach-omap2/board-zoom2.c
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
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl4030.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#ifdef CONFIG_SIL9022
#include <linux/sil9022.h>
#endif
#include <linux/switch.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/board-zoom2.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpmc.h>
#include <mach/usb.h>
#include <mach/mux.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>

#include <mach/display.h>
#include <mach/hdq.h>

#include "mmc-twl4030.h"
#include "omap3-opp.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"

#include <media/v4l2-int-device.h>

#ifdef CONFIG_PM
#include <../drivers/media/video/omap/omap_voutdef.h>
#endif

#ifdef CONFIG_VIDEO_LV8093
#include <media/lv8093.h>
#define LV8093_PS_GPIO			7
/* GPIO7 is connected to lens PS pin through inverter */
#define LV8093_PWR_OFF			1
#define LV8093_PWR_ON			(!LV8093_PWR_OFF)
#endif

#ifndef CONFIG_TWL4030_CORE
#error "no power companion board defined!"
#endif

#ifdef CONFIG_WL127X_RFKILL
#include <linux/wl127x-rfkill.h>
#endif

#define OMAP_SYNAPTICS_GPIO		163

#define SDP3630_SMC91X_CS	3
#define SDP3630_ETHR_SMC91X_GPIO 158

#define TWL4030_MSECURE_GPIO	22

static struct pin_config sdp3630_mux_pins[] = {
/*
 *		Name, reg-offset,
 *		mux-mode | [active-mode | off-mode]
 */

/* McBSPs */
MUX_CFG_34XX("MCBSP2_SLAVE", 0x13c /* FSX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP2_SLAVE", 0x13e /* CLKX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP2_SLAVE", 0x140 /* DR */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP2_SLAVE", 0x142 /* DX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP3_MASTER", 0x194 /* CLKS */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP3_MASTER", 0x172 /* FSX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP3_MASTER", 0x170 /* CLKX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP3_MASTER", 0x16e /* DR */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLDOWN)
MUX_CFG_34XX("MCBSP3_MASTER", 0x16c /* DX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP3_SLAVE", 0x172 /* FSX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLDOWN)
MUX_CFG_34XX("MCBSP3_SLAVE", 0x170 /* CLKX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLDOWN)
MUX_CFG_34XX("MCBSP3_SLAVE", 0x16e /* DR */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLDOWN)
MUX_CFG_34XX("MCBSP3_SLAVE", 0x16c /* DX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP3_TRISTATE", 0x172 /* FSX */,
		OMAP34XX_MUX_MODE7)
MUX_CFG_34XX("MCBSP3_TRISTATE", 0x170 /* CLKX */,
		OMAP34XX_MUX_MODE7)
MUX_CFG_34XX("MCBSP3_TRISTATE", 0x16e /* DR */,
		OMAP34XX_MUX_MODE7)
MUX_CFG_34XX("MCBSP3_TRISTATE", 0x16c /* DX */,
		OMAP34XX_MUX_MODE7)
MUX_CFG_34XX("MCBSP4_MASTER", 0x194 /* CLKS */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP4_MASTER", 0x18a /* FSX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP4_MASTER", 0x0b6 /* CLKX */,
		OMAP34XX_MUX_MODE2 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP4_MASTER", 0x0b8 /* DR */,
		OMAP34XX_MUX_MODE2 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP4_MASTER", 0x0ba /* DX */,
		OMAP34XX_MUX_MODE2 | OMAP34XX_PIN_OUTPUT)
MUX_CFG_34XX("MCBSP4_SLAVE", 0x18a /* FSX */,
		OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP4_SLAVE", 0x0b6 /* CLKX */,
		OMAP34XX_MUX_MODE2 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP4_SLAVE", 0x0b8 /* DR */,
		OMAP34XX_MUX_MODE2 | OMAP34XX_PIN_INPUT)
MUX_CFG_34XX("MCBSP4_SLAVE", 0x0ba /* DX */,
		OMAP34XX_MUX_MODE2 | OMAP34XX_PIN_OUTPUT)
};

static struct omap_mux_cfg sdp3630_mux_cfg = {
		.pins	= sdp3630_mux_pins,
		.size	= ARRAY_SIZE(sdp3630_mux_pins),
};

static struct resource sdp3630_smc91x_resources[] = {
	[0] = {
		.start	= OMAP34XX_ETHR_START,
		.end	= OMAP34XX_ETHR_START + SZ_4K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device sdp3630_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sdp3630_smc91x_resources),
	.resource	= sdp3630_smc91x_resources,
};

/* sdp3630 has Qwerty keyboard*/
static int sdp3630_twl4030_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(1, 0, KEY_R),
	KEY(2, 0, KEY_T),
	KEY(3, 0, KEY_HOME),
	KEY(6, 0, KEY_I),
	KEY(7, 0, KEY_LEFTSHIFT),
	KEY(0, 1, KEY_D),
	KEY(1, 1, KEY_F),
	KEY(2, 1, KEY_G),
	KEY(3, 1, KEY_SEND),
	KEY(6, 1, KEY_K),
	KEY(7, 1, KEY_ENTER),
	KEY(0, 2, KEY_X),
	KEY(1, 2, KEY_C),
	KEY(2, 2, KEY_V),
	KEY(3, 2, KEY_END),
	KEY(6, 2, KEY_DOT),
	KEY(7, 2, KEY_CAPSLOCK),
	KEY(0, 3, KEY_Z),
	KEY(1, 3, KEY_KPPLUS),
	KEY(2, 3, KEY_B),
	KEY(3, 3, KEY_F1),
	KEY(6, 3, KEY_O),
	KEY(7, 3, KEY_SPACE),
	KEY(0, 4, KEY_W),
	KEY(1, 4, KEY_Y),
	KEY(2, 4, KEY_U),
	KEY(3, 4, KEY_F2),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(6, 4, KEY_L),
	KEY(7, 4, KEY_LEFT),
	KEY(0, 5, KEY_S),
	KEY(1, 5, KEY_H),
	KEY(2, 5, KEY_J),
	KEY(3, 5, KEY_F3),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(6, 5, KEY_M),
	KEY(4, 5, KEY_ENTER),
	KEY(7, 5, KEY_RIGHT),
	KEY(0, 6, KEY_Q),
	KEY(1, 6, KEY_A),
	KEY(2, 6, KEY_N),
	KEY(3, 6, KEY_BACKSPACE),
	KEY(6, 6, KEY_P),
	KEY(7, 6, KEY_UP),
	KEY(6, 7, KEY_SELECT),
	KEY(7, 7, KEY_DOWN),
	KEY(0, 7, KEY_PROG1),	/*MACRO 1 <User defined> */
	KEY(1, 7, KEY_PROG2),	/*MACRO 2 <User defined> */
	KEY(2, 7, KEY_PROG3),	/*MACRO 3 <User defined> */
	KEY(3, 7, KEY_PROG4),	/*MACRO 4 <User defined> */
	0
};

static struct twl4030_keypad_data sdp3630_kp_twl4030_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= sdp3630_twl4030_keymap,
	.keymapsize	= ARRAY_SIZE(sdp3630_twl4030_keymap),
	.rep		= 1,
};

static int __init msecure_init(void)
{
	int ret = 0;

#ifdef CONFIG_RTC_DRV_TWL4030
	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (omap_type() == OMAP2_DEVICE_TYPE_GP &&
			omap_rev() < OMAP3430_REV_ES2_0) {
		void __iomem *msecure_pad_config_reg =
			omap_ctrl_base_get() + 0xA3C;
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
		tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		__raw_writew(tmp, msecure_pad_config_reg);

		gpio_direction_output(TWL4030_MSECURE_GPIO, 1);
	}
out:
#endif
	return ret;
}

static struct omap2_mcspi_device_config sdp3630_lcd_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
	.mode			= OMAP2_MCSPI_MASTER,
	.dma_mode		= 0,
	.force_cs_mode		= 0,
	.fifo_depth		= 0,
};

static struct spi_board_info sdp3630_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "sdp3630_disp_spi",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 375000,
		.controller_data 	= &sdp3630_lcd_mcspi_config,
	},
};

#define LCD_PANEL_BACKLIGHT_GPIO 	(7 + OMAP_MAX_GPIO_LINES)

#define LCD_PANEL_RESET_GPIO		55
#define LCD_PANEL_QVGA_GPIO		56

#define TV_PANEL_ENABLE_GPIO		95


#define PM_RECEIVER			TWL4030_MODULE_PM_RECEIVER
#define ENABLE_VAUX2_DEDICATED		0x09
#define ENABLE_VAUX2_DEV_GRP		0x20

#define ENABLE_VPLL2_DEDICATED          0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0
#define TWL4030_VPLL2_DEV_GRP           0x33
#define TWL4030_VPLL2_DEDICATED         0x36

#define SIL9022_RESET_GPIO 		97

#define t2_out(c, r, v) twl4030_i2c_write_u8(c, r, v)

static int sdp3630_panel_power_enable(int enable)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEDICATED : 0,
				TWL4030_VPLL2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEV_GRP : 0,
				TWL4030_VPLL2_DEV_GRP);
	return 0;
}

static int sdp3630_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	sdp3630_panel_power_enable(1);

	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 1);

	return 0;
}

static void sdp3630_panel_disable_lcd(struct omap_dss_device *dssdev)
{
#ifndef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	sdp3630_panel_power_enable(0);
#endif

	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);

}

static struct omap_dss_device sdp3630_lcd_device = {
	.name = "lcd",
	.driver_name = "sdp3630_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable = sdp3630_panel_enable_lcd,
	.platform_disable = sdp3630_panel_disable_lcd,
 };

#ifdef CONFIG_SIL9022
static void sdp3630_hdmi_reset_enable(int level)
{
	/* Set GPIO_97 to high to pull SiI9022 HDMI transmitter
	 * out of reset and low to disable it.
	 */
	gpio_request(SIL9022_RESET_GPIO, "hdmi reset");
	gpio_direction_output(SIL9022_RESET_GPIO, level);
}

static int sdp3630_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	sdp3630_panel_power_enable(1);
	sdp3630_hdmi_reset_enable(1);

	return 0;
}

static void sdp3630_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	sdp3630_hdmi_reset_enable(0);
#ifndef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	sdp3630_panel_power_enable(0);
#endif
}
struct hdmi_platform_data sdp3630_hdmi_data = {
#ifdef CONFIG_PM
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
#endif
};

static struct omap_dss_device sdp3630_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.phy.dpi.data_lines = 24,
	.platform_enable = sdp3630_panel_enable_hdmi,
	.platform_disable = sdp3630_panel_disable_hdmi,
	.dev		= {
		.platform_data = &sdp3630_hdmi_data,
	},
};
#endif


static int sdp3630_panel_enable_tv(struct omap_dss_device *dssdev)
{
#define ENABLE_VDAC_DEDICATED           0x03
#define ENABLE_VDAC_DEV_GRP             0x20

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void sdp3630_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}
static struct omap_dss_device sdp3630_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable = sdp3630_panel_enable_tv,
	.platform_disable = sdp3630_panel_disable_tv,
};

static struct omap_dss_device *sdp3630_dss_devices[] = {
	&sdp3630_lcd_device,
	&sdp3630_tv_device,
#ifdef CONFIG_SIL9022
	&sdp3630_hdmi_device,
#endif
};

static struct omap_dss_board_info sdp3630_dss_data = {
	.get_last_off_on_transaction_id = get_last_off_on_transaction_id,
	.num_devices = ARRAY_SIZE(sdp3630_dss_devices),
	.devices = sdp3630_dss_devices,
	.default_device = &sdp3630_lcd_device,
};

static struct platform_device sdp3630_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &sdp3630_dss_data,
	},
};

static struct regulator_consumer_supply sdp3630_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &sdp3630_dss_device.dev,
};

static struct regulator_consumer_supply sdp3630_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &sdp3630_dss_device.dev,
};

#ifdef CONFIG_FB_OMAP2
static struct resource sdp3630_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource sdp3630_vout_resource[2] = {
};
#endif

#ifdef CONFIG_PM
struct vout_platform_data sdp3630_vout_data = {
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
	.set_max_mpu_wakeup_lat =  omap_pm_set_max_mpu_wakeup_lat,
	.set_vdd1_opp = omap_pm_set_min_mpu_freq,
	.set_cpu_freq = omap_pm_cpu_set_freq,
};
#endif

static struct platform_device sdp3630_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(sdp3630_vout_resource),
	.resource	= &sdp3630_vout_resource[0],
	.id		= -1,
#ifdef CONFIG_PM
	.dev		= {
		.platform_data = &sdp3630_vout_data,
	}
#else
	.dev		= {
		.platform_data = NULL,
	}
#endif
};

static struct gpio_switch_platform_data headset_switch_data = {
	.name		= "h2w",
	.gpio		= OMAP_MAX_GPIO_LINES + 2, /* TWL4030 GPIO_2 */
};

static struct platform_device headset_switch_device = {
	.name		= "switch-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &headset_switch_data,
	}
};

static struct platform_device *sdp3630_devices[] __initdata = {
	&sdp3630_dss_device,
	&sdp3630_smc91x_device,
#if defined(CONFIG_HDQ_MASTER_OMAP) || defined(CONFIG_HDQ_MASTER_OMAP_MODULE)
	&omap_hdq_device,
#endif
	&sdp3630_vout_device,
	&headset_switch_device,
};

static inline void __init sdp3630_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	eth_cs = SDP3630_SMC91X_CS;

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc911x\n");
		return;
	}

	sdp3630_smc91x_resources[0].start = cs_mem_base + 0x300;
	sdp3630_smc91x_resources[0].end   = cs_mem_base + 0x30f;
	udelay(100);

	eth_gpio = SDP3630_ETHR_SMC91X_GPIO;

	sdp3630_smc91x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

	if (gpio_request(eth_gpio, "smc91x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
		       eth_gpio);
		return;
	}
	gpio_direction_input(eth_gpio);
}

/* Quad UART (TL16CP754C) is on sdp3630 debug board */
/* Map registers to GPMC CS3 */

static void __init omap_sdp3630_init_irq(void)
{
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
		omap3630_mpu_rate_table, omap3630_dsp_rate_table,
		omap3630_l3_rate_table);
	/* Production board supports McBSP4 */
	omap2_mux_register(&sdp3630_mux_cfg);
	omap_init_irq();
	omap_gpio_init();
	sdp3630_init_smc91x();
}

static struct regulator_consumer_supply sdp3630_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply sdp3630_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply sdp3630_vmmc2_supply = {
	.supply		= "vmmc",
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data sdp3630_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &sdp3630_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data sdp3630_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &sdp3630_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data sdp3630_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &sdp3630_vsim_supply,
};

static struct regulator_init_data sdp3630_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &sdp3630_vdda_dac_supply,
};

static struct regulator_init_data sdp3630_vdsi = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &sdp3630_vdds_dsi_supply,
};

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};

static int sdp3630_twl_gpio_setup(struct device *dev,
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
	sdp3630_vmmc1_supply.dev = mmc[0].dev;
	sdp3630_vsim_supply.dev = mmc[0].dev;
	sdp3630_vmmc2_supply.dev = mmc[1].dev;

	return 0;
}

static struct omap_lcd_config sdp3630_lcd_config __initdata = {
	.ctrl_name      = "internal",
};

static struct omap_uart_config sdp3630_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)),
};

static struct omap_board_config_kernel sdp3630_config[] __initdata = {
	{ OMAP_TAG_UART,	&sdp3630_uart_config },
	{ OMAP_TAG_LCD,         &sdp3630_lcd_config },
};

static int sdp3630_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,   9280,   8950,   8620,   8310,
8020,   7730,   7460,   7200,   6950,   6710,   6470,   6250,   6040,   5830,
5640,   5450,   5260,   5090,   4920,   4760,   4600,   4450,   4310,   4170,
4040,   3910,   3790,   3670,   3550
};

static struct twl4030_bci_platform_data sdp3630_bci_data = {
	.battery_tmp_tbl	= sdp3630_batt_table,
	.tblsize		= ARRAY_SIZE(sdp3630_batt_table),
	.twl4030_bci_charging_current	= 1100, /* 1.1A for sdp3630 */
};

static struct twl4030_usb_data sdp3630_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data sdp3630_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= sdp3630_twl_gpio_setup,
};

static struct twl4030_madc_platform_data sdp3630_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_ins __initdata sleep_on_seq[] = {

	/* Turn off HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 2},
	/* Turn OFF VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 2},
	/* Turn OFF VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
	/* Turn OFF VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TRITON_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] __initdata = {
	/* Turn on HFCLKOUT */
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 2},
	/* Turn ON VDD2 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
	/* Turn ON VPLL1 */
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script = wakeup_p12_seq,
	.size   = ARRAY_SIZE(wakeup_p12_seq),
	.flags  = TRITON_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] __initdata = {
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TRITON_WAKEUP3_SCRIPT,
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
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TRITON_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_p12_script,
	&wakeup_p3_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1, .type = -1,
		.type2 = -1 },
	{ 0, 0},
};

static struct twl4030_power_data sdp3630_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.size		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_platform_data sdp3630_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &sdp3630_bci_data,
	.madc		= &sdp3630_madc_data,
	.usb		= &sdp3630_usb_data,
	.gpio		= &sdp3630_gpio_data,
	.keypad		= &sdp3630_kp_twl4030_data,
	.power		= &sdp3630_t2scripts_data,
	.vmmc1          = &sdp3630_vmmc1,
	.vmmc2          = &sdp3630_vmmc2,
	.vsim           = &sdp3630_vsim,
	.vdac		= &sdp3630_vdac,
	.vpll2		= &sdp3630_vdsi,
};

static struct i2c_board_info __initdata sdp3630_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &sdp3630_twldata,
	},
};

static void synaptics_dev_init(void)
{
	/* Set the ts_gpio pin mux */
	omap_cfg_reg(H18_34XX_GPIO163);

	if (gpio_request(OMAP_SYNAPTICS_GPIO, "touch") < 0) {
		printk(KERN_ERR "can't get synaptics pen down GPIO\n");
		return;
	}
	gpio_direction_input(OMAP_SYNAPTICS_GPIO);
	omap_set_gpio_debounce(OMAP_SYNAPTICS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP_SYNAPTICS_GPIO, 0xa);
}

static int synaptics_power(int power_state)
{
	/* TODO: synaptics is powered by vbatt */
	return 0;
}

static struct synaptics_i2c_rmi_platform_data synaptics_platform_data[] = {
	{
		.version	= 0x0,
		.power		= &synaptics_power,
		.flags		= SYNAPTICS_SWAP_XY,
		.irqflags	= IRQF_TRIGGER_LOW,
	}
};

/* I2C Address for ISP1301 Transceiver */
#define ISP1301_I2C_ADDR1               0x2C
#define ISP1301_I2C_ADDR2               0x2D

static struct i2c_board_info __initdata sdp3630_i2c_bus2_info[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME,  0x20),
		.platform_data = &synaptics_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_SYNAPTICS_GPIO),
	},
#ifdef CONFIG_VIDEO_LV8093
	{
		I2C_BOARD_INFO(LV8093_NAME,  LV8093_AF_I2C_ADDR),
		.platform_data = &sdp3630_lv8093_platform_data,
	},
#endif
	{
		I2C_BOARD_INFO("isp1301_host", ISP1301_I2C_ADDR1),
		.type           = "isp1301_host",
	},
	{
		I2C_BOARD_INFO("isp1301_host", ISP1301_I2C_ADDR2),
		.type           = "isp1301_host",
	},
};

static struct i2c_board_info __initdata sdp3630_i2c_bus3_info[] = {
#ifdef CONFIG_SIL9022
	{
		I2C_BOARD_INFO(SIL9022_DRV_NAME,  SI9022_I2CSLAVEADDRESS),
	},
#endif
};


static int __init omap_i2c_init(void)
{
	omap_register_i2c_bus(1, 100, sdp3630_i2c_bus1_info,
			ARRAY_SIZE(sdp3630_i2c_bus1_info));
	omap_register_i2c_bus(2, 100, sdp3630_i2c_bus2_info,
			ARRAY_SIZE(sdp3630_i2c_bus2_info));
	omap_register_i2c_bus(3, 400, sdp3630_i2c_bus3_info,
			ARRAY_SIZE(sdp3630_i2c_bus3_info));
	return 0;
}

/* usb1hs reset is routed to chip select pin of ISP1504
 * on 750-2099-003(A) daughter card */
static void config_usb1hs_gpio(void)
{
	omap_cfg_reg(D25_34XX_GPIO126);
}

/* Set GPIO56 to safemode (input only),
 * this prevents a toggle on RCV pin of ISP1301
 */
static void config_usb3fs_gpio(void)
{
	omap_cfg_reg(R8_34XX_GPIO56_TRISTATE);
}

static void enable_board_wakeup_source(void)
{
	omap_cfg_reg(AF26_34XX_SYS_NIRQ);
}

static void __init omap_sdp3630_init(void)
{
	omap_i2c_init();
	platform_add_devices(sdp3630_devices, ARRAY_SIZE(sdp3630_devices));
	omap_board_config = sdp3630_config;
	omap_board_config_size = ARRAY_SIZE(sdp3630_config);
	spi_register_board_info(sdp3630_spi_board_info,
				ARRAY_SIZE(sdp3630_spi_board_info));
	synaptics_dev_init();
	msecure_init();
	omap_serial_init();
	usb_musb_init();
	config_usb1hs_gpio();
	usb_ehci_init();
	config_usb3fs_gpio();
	usb_ohci_init();
#ifdef CONFIG_SIL9022
	config_hdmi_gpio();
	sdp3630_hdmi_reset_enable(1);
#endif
	enable_board_wakeup_source();
}

static void __init omap_sdp3630_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_3630SDP, "OMAP 3630SDP board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_sdp3630_map_io,
	.init_irq	= omap_sdp3630_init_irq,
	.init_machine	= omap_sdp3630_init,
	.timer		= &omap_timer,
MACHINE_END
