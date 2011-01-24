/*
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Vikram Pandita <vikram.pandita@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
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
#include <linux/switch.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/board-edp1.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpmc.h>
#if 0
#include <mach/hsmmc.h>
#endif
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
#include "sdram-micron-mt46h32m32lf-6.h"

#include <media/v4l2-int-device.h>

#ifdef CONFIG_PM
#include <../drivers/media/video/omap/omap_voutdef.h>
#endif

#ifndef CONFIG_TWL4030_CORE
#error "no power companion board defined!"
#endif

#ifdef CONFIG_WL127X_RFKILL
#include <linux/wl127x-rfkill.h>
#endif

#define OMAP_SYNAPTICS_GPIO		163

#define CONFIG_DISABLE_HFCLK 1
#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20

#define ENABLE_VAUX3_DEDICATED  0x03
#define ENABLE_VAUX3_DEV_GRP  	0x20
#define WL127X_BTEN_GPIO	109

#define OMAP_BQ24072_CEN_GPIO		110
#define OMAP_BQ24072_EN1_GPIO		102
#define OMAP_BQ24072_EN2_GPIO		104

#ifdef CONFIG_WL127X_RFKILL
static struct wl127x_rfkill_platform_data wl127x_plat_data = {
	.bt_nshutdown_gpio = 109, 	/* Bluetooth Enable GPIO */
	.fm_enable_gpio = 161,		/* FM Enable GPIO */
};

static struct platform_device edp1_wl127x_device = {
	.name           = "wl127x-rfkill",
	.id             = -1,
	.dev.platform_data = &wl127x_plat_data,
};
#endif

/* EDP1 Qwerty keyboard: */
static int edp1_twl4030_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(1, 0, KEY_R),
	KEY(2, 0, KEY_T),
	KEY(6, 0, KEY_I),
	KEY(7, 0, KEY_F1),	/* SYM */

	KEY(0, 1, KEY_D),
	KEY(1, 1, KEY_F),
	KEY(2, 1, KEY_G),
	KEY(6, 1, KEY_K),

	KEY(0, 2, KEY_X),
	KEY(1, 2, KEY_C),
	KEY(2, 2, KEY_V),
	KEY(3, 2, KEY_DELETE),
	KEY(6, 2, KEY_M),
	KEY(7, 2, KEY_LEFTSHIFT),

	KEY(0, 3, KEY_Z),
	KEY(1, 3, KEY_TAB),
	KEY(2, 3, KEY_B),
	KEY(3, 3, KEY_LEFTALT),
	KEY(6, 3, KEY_O),
	KEY(7, 3, KEY_SPACE),

	KEY(0, 4, KEY_W),
	KEY(1, 4, KEY_Y),
	KEY(2, 4, KEY_U),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(6, 4, KEY_L),
	KEY(7, 4, KEY_LEFT),

	KEY(0, 5, KEY_S),
	KEY(1, 5, KEY_H),
	KEY(2, 5, KEY_J),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(6, 5, KEY_DOT),
	KEY(7, 5, KEY_RIGHT),

	KEY(0, 6, KEY_Q),
	KEY(1, 6, KEY_A),
	KEY(2, 6, KEY_N),
	KEY(6, 6, KEY_P),
	KEY(7, 6, KEY_UP),

	KEY(6, 7, KEY_ENTER),
	KEY(7, 7, KEY_DOWN),

	KEY(3, 0, KEY_F6),	/* HOME */
	KEY(3, 1, KEY_F7),	/* MENU */
	KEY(7, 1, KEY_F10),	/* joystick center */
	KEY(3, 4, KEY_F8),	/* NEXT */
	KEY(3, 5, KEY_BACK),
	KEY(3, 6, KEY_F9),	/* BACK */
	0
};

static struct twl4030_keypad_data edp1_kp_twl4030_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= edp1_twl4030_keymap,
	.keymapsize	= ARRAY_SIZE(edp1_twl4030_keymap),
	.rep		= 1,
};

static int __init msecure_init(void)
{
	return 0;
}

static struct omap2_mcspi_device_config edp1_lcd_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
};

static struct spi_board_info edp1_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "zoom2_disp_spi",
#ifdef CONFIG_MACH_OMAP3621_EDP1
		.bus_num                = 2,
		.chip_select            = 0,
#else
		.bus_num		= 1,
		.chip_select		= 2,
#endif
		.max_speed_hz		= 375000,
		.controller_data 	= &edp1_lcd_mcspi_config,
	},
};

#define LCD_PANEL_BACKLIGHT_GPIO 	(7 + OMAP_MAX_GPIO_LINES)

#define LCD_PANEL_RESET_GPIO		55
#define LCD_PANEL_QVGA_GPIO		56


#define PM_RECEIVER			TWL4030_MODULE_PM_RECEIVER
#define ENABLE_VAUX2_DEDICATED		0x09
#define ENABLE_VAUX2_DEV_GRP		0x20
#define ENABLE_VAUX3_DEDICATED		0x03
#define ENABLE_VAUX3_DEV_GRP		0x20

#define ENABLE_VPLL2_DEDICATED          0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0
#define TWL4030_VPLL2_DEV_GRP           0x33
#define TWL4030_VPLL2_DEDICATED         0x36

#define t2_out(c, r, v) twl4030_i2c_write_u8(c, r, v)

static void edp1_lcd_tv_panel_init(void)
{
	int lcd_panel_reset_gpio;
	
	if(machine_is_omap3621_edp1()) {
		lcd_panel_reset_gpio = 4;

	} else {
		//3630
		lcd_panel_reset_gpio = LCD_PANEL_RESET_GPIO;
	}

	gpio_request(lcd_panel_reset_gpio, "lcd reset");
	gpio_request(LCD_PANEL_QVGA_GPIO, "lcd qvga");
	gpio_request(LCD_PANEL_BACKLIGHT_GPIO, "lcd backlight");
	
	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 0);
	gpio_direction_output(lcd_panel_reset_gpio, 0);
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);

	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 1);
	gpio_direction_output(lcd_panel_reset_gpio, 1);
}

static int edp1_panel_power_enable(int enable)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEDICATED : 0,
				TWL4030_VPLL2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEV_GRP : 0,
				TWL4030_VPLL2_DEV_GRP);
	return 0;
}

static int edp1_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	edp1_panel_power_enable(1);

	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 1);

	return 0;
}

static void edp1_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);

}

static struct omap_dss_device edp1_lcd_device = {
	.name = "lcd",
	.driver_name = "zoom2_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
#ifdef CONFIG_EDP3_BOARD
	.phy.dpi.data_lines = 16,
#else
	.phy.dpi.data_lines = 24,
#endif
	.platform_enable = edp1_panel_enable_lcd,
	.platform_disable = edp1_panel_disable_lcd,
 };

static int edp1_panel_enable_tv(struct omap_dss_device *dssdev)
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

static void edp1_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}
static struct omap_dss_device edp1_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable = edp1_panel_enable_tv,
	.platform_disable = edp1_panel_disable_tv,
};

static struct omap_dss_device *edp1_dss_devices[] = {
	&edp1_lcd_device,
	&edp1_tv_device,
};

static struct omap_dss_board_info edp1_dss_data = {
	.get_last_off_on_transaction_id = get_last_off_on_transaction_id,
	.num_devices = ARRAY_SIZE(edp1_dss_devices),
	.devices = edp1_dss_devices,
	.default_device = &edp1_lcd_device,
};

static struct platform_device edp1_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &edp1_dss_data,
	},
};

static struct regulator_consumer_supply edp1_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &edp1_dss_device.dev,
};

static struct regulator_consumer_supply edp1_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &edp1_dss_device.dev,
};

#ifdef CONFIG_FB_OMAP2
static struct resource edp1_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource edp1_vout_resource[2] = {
};
#endif

#ifdef CONFIG_PM
struct vout_platform_data edp1_vout_data = {
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
	.set_max_mpu_wakeup_lat =  omap_pm_set_max_mpu_wakeup_lat,
	//.set_vdd1_opp = omap_pm_set_min_mpu_freq,
	.set_cpu_freq = omap_pm_cpu_set_freq,
};
#endif

static struct platform_device edp1_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(edp1_vout_resource),
	.resource	= &edp1_vout_resource[0],
	.id		= -1,
#ifdef CONFIG_PM
	.dev		= {
		.platform_data = &edp1_vout_data,
	}
#else
	.dev		= {
		.platform_data = NULL,
	}
#endif
};

#ifdef CONFIG_REGULATOR_BQ24073
static struct bq24073_mach_info bq24073_init = {
	.gpio_nce = OMAP_BQ24072_CEN_GPIO,
	.gpio_en1 = OMAP_BQ24072_EN1_GPIO,
	.gpio_en2 = OMAP_BQ24072_EN2_GPIO,
};

/* GPIOS need to be in order of BQ24073 */
static struct platform_device edp1_curr_regulator_device = {
	.name           = "bq24073", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &bq24073_init,
};
#endif

static struct platform_device *edp1_devices[] __initdata = {
	&edp1_dss_device,
#ifdef CONFIG_WL127X_RFKILL
	&edp1_wl127x_device,
#endif
	&edp1_vout_device,
#ifdef CONFIG_REGULATOR_BQ24073
	&edp1_curr_regulator_device,
#endif
};

static void __init omap_edp1_init_irq(void)
{
#if defined(CONFIG_MACH_OMAP3630_EDP1)
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
		omap3630_mpu_rate_table, omap3630_dsp_rate_table,
		omap3630_l3_rate_table);
#elif defined(CONFIG_MACH_OMAP3621_EDP1)
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
		omap3621_mpu_rate_table, omap3621_dsp_rate_table,
		omap3621_l3_rate_table);
#endif

	omap_init_irq();
	omap_gpio_init();
}

static struct regulator_consumer_supply edp1_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply edp1_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply edp1_vmmc2_supply = {
	.supply		= "vmmc",
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data edp1_vmmc1 = {
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
	.consumer_supplies      = &edp1_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data edp1_vmmc2 = {
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
	.consumer_supplies      = &edp1_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data edp1_vsim = {
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
	.consumer_supplies      = &edp1_vsim_supply,
};

static struct regulator_init_data edp1_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &edp1_vdda_dac_supply,
};

static struct regulator_init_data edp1_vdsi = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &edp1_vdds_dsi_supply,
};

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.wires		= 8,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 3,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};

static int edp1_twl_gpio_setup(struct device *dev,
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
	edp1_vmmc1_supply.dev = mmc[0].dev;
	edp1_vsim_supply.dev = mmc[0].dev;
	edp1_vmmc2_supply.dev = mmc[1].dev;

	return 0;
}

static struct omap_lcd_config edp1_lcd_config __initdata = {
        .ctrl_name      = "internal",
};

static struct omap_uart_config edp1_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel edp1_config[] __initdata = {
	{ OMAP_TAG_UART,	&edp1_uart_config },
        { OMAP_TAG_LCD,         &edp1_lcd_config },
};

static int edp1_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,   9280,   8950,   8620,   8310,
8020,   7730,   7460,   7200,   6950,   6710,   6470,   6250,   6040,   5830,
5640,   5450,   5260,   5090,   4920,   4760,   4600,   4450,   4310,   4170,
4040,   3910,   3790,   3670,   3550
};

static struct twl4030_bci_platform_data edp1_bci_data = {
	.battery_tmp_tbl	= edp1_batt_table,
	.tblsize		= ARRAY_SIZE(edp1_batt_table),
	.twl4030_bci_charging_current	= 1100, /* 1.1A for Zoom2 */
};

static struct twl4030_usb_data edp1_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data edp1_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= edp1_twl_gpio_setup,
};

static struct twl4030_madc_platform_data edp1_madc_data = {
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

static struct twl4030_power_data edp1_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.size		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_platform_data edp1_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &edp1_bci_data,
	.madc		= &edp1_madc_data,
	.usb		= &edp1_usb_data,
	.gpio		= &edp1_gpio_data,
	.keypad		= &edp1_kp_twl4030_data,
	.power		= &edp1_t2scripts_data,
	.vmmc1          = &edp1_vmmc1,
	.vmmc2          = &edp1_vmmc2,
	.vsim           = &edp1_vsim,
	.vdac		= &edp1_vdac,
	.vpll2		= &edp1_vdsi,
};

#if defined(CONFIG_MACH_OMAP3621_EDP1)
static struct i2c_board_info __initdata edp1_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65920", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &edp1_twldata,
	},
};
#else
static struct i2c_board_info __initdata edp1_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &edp1_twldata,
	},
};
#endif

static void synaptics_dev_init(void)
{
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

#define AIC3111_NAME "tlv320aic3111"
#define AIC3111_I2CSLAVEADDRESS 0x18
static struct i2c_board_info __initdata edp1_i2c_bus2_info[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME,  0x20),
		.platform_data = &synaptics_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_SYNAPTICS_GPIO),
	},
	{
		I2C_BOARD_INFO(AIC3111_NAME,  AIC3111_I2CSLAVEADDRESS),
	},
#if defined(CONFIG_BOSCH_BMA150) || defined(CONFIG_BOSCH_BMA150_MODULE)
	{
		I2C_BOARD_INFO("bma150", 0x38),
	},
#endif /* defined(CONFIG_BOSCH_BMA150) || defined(CONFIG_BOSCH_BMA150_MODULE) */
#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
	{
		I2C_BOARD_INFO("bq27510",  0x55),
	},
#endif
};

#if !defined(CONFIG_MACH_OMAP3630_EDP1)
static struct i2c_board_info __initdata edp1_i2c_bus3_info[] = {
};
#endif /* !defined(CONFIG_MACH_OMAP3630_EDP1) */


static int __init omap_i2c_init(void)
{

/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
		/* Program (bit 7)=1 to disable internal pull-up on I2C3 */
		prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}

	omap_register_i2c_bus(1, 100, edp1_i2c_bus1_info,
			ARRAY_SIZE(edp1_i2c_bus1_info));
	omap_register_i2c_bus(2, 100, edp1_i2c_bus2_info,
			ARRAY_SIZE(edp1_i2c_bus2_info));
#if !defined(CONFIG_MACH_OMAP3630_EDP1)
	omap_register_i2c_bus(3, 400, edp1_i2c_bus3_info,
			ARRAY_SIZE(edp1_i2c_bus3_info));
#endif /* !defined(CONFIG_MACH_OMAP3630_EDP1) */
	return 0;
}

static int __init wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	ret = gpio_request(WL127X_BTEN_GPIO, "wl127x_bten");
	if (ret < 0) {
		printk(KERN_ERR "wl127x_bten gpio_%d request fail",
						WL127X_BTEN_GPIO);
		goto fail;
	}

	gpio_direction_output(WL127X_BTEN_GPIO, 1);
	mdelay(10);
	gpio_direction_output(WL127X_BTEN_GPIO, 0);
	udelay(64);

	gpio_free(WL127X_BTEN_GPIO);
fail:
	return ret;
}

static void __init omap_edp1_init(void)
{
	omap_i2c_init();
	/* Fix to prevent VIO leakage on wl127x */
	wl127x_vio_leakage_fix();
	platform_add_devices(edp1_devices, ARRAY_SIZE(edp1_devices));
	omap_board_config = edp1_config;
	omap_board_config_size = ARRAY_SIZE(edp1_config);
	spi_register_board_info(edp1_spi_board_info,
				ARRAY_SIZE(edp1_spi_board_info));
	synaptics_dev_init();
	msecure_init();

	if(machine_is_omap3630_edp1()) {
		ldp_flash_init();
	}

	omap_serial_init();
	usb_musb_init();
	edp1_lcd_tv_panel_init();
}


static void __init omap_edp1_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

#if defined(CONFIG_MACH_OMAP3630_EDP1)
MACHINE_START(OMAP3630_EDP1, "OMAP3630 EDP1 board")
#elif defined(CONFIG_MACH_OMAP3621_EDP1)
MACHINE_START(OMAP3621_EDP1, "OMAP3621 EDP1 board")
#else
#error "Unsupported machine"
#endif
	.phys_io	= L4_34XX_PHYS,
	.io_pg_offst	= ((L4_34XX_VIRT) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_edp1_map_io,
	.init_irq	= omap_edp1_init_irq,
	.init_machine	= omap_edp1_init,
	.timer		= &omap_timer,
MACHINE_END
