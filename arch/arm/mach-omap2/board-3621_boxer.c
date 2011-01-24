
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
#ifdef CONFIG_TOUCHSCREEN_CY8CTMG120_I2C
#include <linux/tmg120.h>		/* include tmg120.h */
#endif
#ifdef CONFIG_TOUCHSCREEN_PIXCIR_I2C
#include <linux/pixcir_i2c_s32.h>
#endif

#ifdef CONFIG_INPUT_KXTF9
#include <linux/kxtf9.h>	
#define KXTF9_DEVICE_ID			"kxtf9"
#define KXTF9_I2C_SLAVE_ADDRESS		0x0F
#define KXTF9_GPIO_FOR_PWR		34
#define	KXTF9_GPIO_FOR_IRQ		113
#endif /* CONFIG_INPUT_KXTF9 */


#include <linux/spi/spi.h>
#include <linux/i2c/twl4030.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/switch.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/board-boxer.h>
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

#include <linux/usb/android.h>

#include "mmc-twl4030.h"
#include "omap3-opp.h"

#if defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
  #include "sdram-samsung-k4x4g303pb.h"
#else
  #include "sdram-elpida-edd20323abh.h"
#endif

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


#ifdef CONFIG_TOUCHSCREEN_CY8CTMG120_I2C
#define CY8CTMG120_I2C_NAME	"Cypress-CY8CTMG120"
#define CY8CTMG120_I2C_SLAVEADDRESS	0x20
#define OMAP_CY8CTMG120_GPIO	99
#endif

#define CONFIG_DISABLE_HFCLK 1
#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20

#define ENABLE_VAUX3_DEDICATED  0x03
#define ENABLE_VAUX3_DEV_GRP	0x20
#define TWL4030_MSECURE_GPIO	22

#if defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
  #define WL127X_BTEN_GPIO	60
#else
  #define WL127X_BTEN_GPIO	109
#endif

#if !defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
  #define OMAP_BQ24072_CEN_GPIO	110
  #define OMAP_BQ24072_EN1_GPIO	102
  #define OMAP_BQ24072_EN2_GPIO	104
#endif

#define	EMMC_POWER_ENABLE_GPIO	101

#define BOXER_EXT_QUART_PHYS	0x48000000
#define BOXER_EXT_QUART_VIRT	0xfa000000
#define BOXER_EXT_QUART_SIZE	SZ_256

#ifdef CONFIG_WL127X_RFKILL
#if 0
static struct wl127x_rfkill_platform_data wl127x_plat_data = {
	.bt_nshutdown_gpio = 109,	/* UART_GPIO (spare) Enable GPIO */
	.fm_enable_gpio = 161,		/* FM Enable GPIO */
};

static struct platform_device boxer_wl127x_device = {
	.name           = "wl127x-rfkill",
	.id             = -1,
	.dev.platform_data = &wl127x_plat_data,
};
#endif
#endif

/* Boxer R0 is not using TWL4030 keys, so define for EVT1A keyboard: */
static int boxer_twl4030_keymap[] = {
	KEY(1, 0, KEY_VOLUMEUP),
	KEY(2, 0, KEY_VOLUMEDOWN),
	KEY(3, 0, KEY_HOME),
	0
};

static struct twl4030_keypad_data boxer_kp_twl4030_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= boxer_twl4030_keymap,
	.keymapsize	= ARRAY_SIZE(boxer_twl4030_keymap),
	.rep		= 1,
};

#if !defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
static struct gpio_keys_button boxer_gpio_buttons[] = {
	{
		.code			= KEY_BACK,	
		.gpio			= 109,
		.desc			= "debug-back",
		.active_low		= 1,
		.wakeup			= 1,
	},
	{
		.code			= KEY_HOME,	
		.gpio			= 7,
		.desc			= "debug-home",
		.active_low		= 1,
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data boxer_gpio_key_info = {
	.buttons	= boxer_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(boxer_gpio_buttons),
//	.rep		= 1,		/* auto-repeat */
};

static struct platform_device boxer_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &boxer_gpio_key_info,
	},
};
#endif


static struct omap2_mcspi_device_config boxer_lcd_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
        .mode                   = OMAP2_MCSPI_MASTER,
        .dma_mode               = 0,
        .force_cs_mode          = 0,
        .fifo_depth             = 0,
};


#if defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
static struct spi_board_info boxer_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "evt1a_disp_spi",
		.bus_num		= 3,	/* McSPI4 */
		.chip_select		= 0,
		.max_speed_hz		= 375000,
		.controller_data	= &boxer_lcd_mcspi_config,
	},
};
#else
/* TODO - Check that Boxer R0 doesn't really use SPI! */
static struct spi_board_info boxer_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "boxer_disp_spi",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 375000,
		.controller_data	= &boxer_lcd_mcspi_config,
	},
};
#endif

#if defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
  #define LCD_EN_GPIO                     36
#else
  #define LCD_EN_GPIO                     14
#endif

#if !defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
  #define SENSOR_POWER_ENABLE_GPIO        112
#endif


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

static void boxer_sensors_power_enable_init(void)
{
#if !defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
        gpio_request(SENSOR_POWER_ENABLE_GPIO,"sensor power enable");
	gpio_direction_output(SENSOR_POWER_ENABLE_GPIO, 1);
#endif
}

static void boxer_eMMC_power_enable_init(void)
{
        gpio_request(EMMC_POWER_ENABLE_GPIO,"emmc power enable");
	gpio_direction_output(EMMC_POWER_ENABLE_GPIO, 1);
}

static int boxer_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_request(LCD_EN_GPIO ,"lcd power enable");
	gpio_direction_output(LCD_EN_GPIO , 1);

	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT,
							166 * 1000 * 4);
	return 0;
}

static void boxer_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	/* disabling LCD on Boxer R0 shuts down I2C2 bus, so don't touch! */
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

#ifdef CONFIG_PANEL_BOXER_CPT
/*
 * omap2dss - fixed size: 256 elements each four bytes / XRGB
 */
static int boxer_clut_fill(void * ptr, u32 size)
{
	int count;
	u32 temp;
	u32 *byte = (u32 *)ptr;

	for (count = 0; count < size / sizeof(u32); count++) {
		temp = 0;
		temp |= (count << 14);
		temp &= 0xffff0000;
		temp |= (count << 6);
		temp &= 0xffffff00;
		temp |= (count >> 2) & 0xff;
		*byte++ = temp;
	}

	return 0;
}
#endif

static struct omap_dss_device boxer_lcd_device = {
	.name = "lcd",
	.driver_name = "boxer_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable	= boxer_panel_enable_lcd,
	.platform_disable	= boxer_panel_disable_lcd,
#ifdef CONFIG_PANEL_BOXER_CPT
	.clut_size		= sizeof(u32) * 256,
	.clut_fill		= boxer_clut_fill,
#endif
 };

static int boxer_panel_enable_tv(struct omap_dss_device *dssdev)
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

static void boxer_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}
static struct omap_dss_device boxer_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable = boxer_panel_enable_tv,
	.platform_disable = boxer_panel_disable_tv,
};

static struct omap_dss_device *boxer_dss_devices[] = {
	&boxer_lcd_device,
	&boxer_tv_device,
};

static struct omap_dss_board_info boxer_dss_data = {
	.get_last_off_on_transaction_id = get_last_off_on_transaction_id,
	.num_devices = ARRAY_SIZE(boxer_dss_devices),
	.devices = boxer_dss_devices,
	.default_device = &boxer_lcd_device,
};

static struct platform_device boxer_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &boxer_dss_data,
	},
};

static struct regulator_consumer_supply boxer_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &boxer_dss_device.dev,
};

static struct regulator_consumer_supply boxer_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &boxer_dss_device.dev,
};

#ifdef CONFIG_FB_OMAP2
static struct resource boxer_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource boxer_vout_resource[2] = {
};
#endif

#ifdef CONFIG_PM
struct vout_platform_data boxer_vout_data = {
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
	.set_max_mpu_wakeup_lat =  omap_pm_set_max_mpu_wakeup_lat,
	.set_vdd1_opp = omap_pm_set_min_mpu_freq,
	.set_cpu_freq = omap_pm_cpu_set_freq,
};
#endif

static struct platform_device boxer_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(boxer_vout_resource),
	.resource	= &boxer_vout_resource[0],
	.id		= -1,
#ifdef CONFIG_PM
	.dev		= {
		.platform_data = &boxer_vout_data,
	}
#else
	.dev		= {
		.platform_data = NULL,
	}
#endif
};

#ifdef CONFIG_REGULATOR_BQ24073
static struct bq24073_mach_info bq24073_init_dev_data = {
	.gpio_nce = OMAP_BQ24072_CEN_GPIO,
	.gpio_en1 = OMAP_BQ24072_EN1_GPIO,
	.gpio_en2 = OMAP_BQ24072_EN2_GPIO,
	.gpio_nce_state = 1,
	.gpio_en1_state = 0,
	.gpio_en2_state = 0,
};

static struct regulator_consumer_supply bq24073_vcharge_supply = {
       .supply         = "bq24073",
};

static struct regulator_init_data bq24073_init  = {

       .constraints = {
               .min_uV                 = 0,
               .max_uV                 = 5000000,
               .min_uA                 = 0,
               .max_uA                 = 1500000,
               .valid_modes_mask       = REGULATOR_MODE_NORMAL
                                       | REGULATOR_MODE_STANDBY,
               .valid_ops_mask         = REGULATOR_CHANGE_CURRENT
                                       | REGULATOR_CHANGE_MODE
                                       | REGULATOR_CHANGE_STATUS,
       },
       .num_consumer_supplies  = 1,
       .consumer_supplies      = &bq24073_vcharge_supply,

       .driver_data = &bq24073_init_dev_data,
};

/* GPIOS need to be in order of BQ24073 */
static struct platform_device boxer_curr_regulator_device = {
	.name           = "bq24073", /* named after init manager for ST */
	.id             = -1,
	.dev 		= {
		.platform_data = &bq24073_init,
	},
};
#endif

static struct omap_pwm_led_platform_data boxer_backlight_data = {
	.name = "lcd-backlight",
	.intensity_timer = 8,
};

static struct platform_device boxer_backlight_led_device = {
	.name		= "omap_pwm_led",
	.id		= -1,
	.dev		= {
		.platform_data = &boxer_backlight_data,
	},
};

static void boxer_backlight_init(void)
{
	omap_cfg_reg(N8_34XX_GPIO58_PWM);
}

static struct platform_device *boxer_devices[] __initdata = {
	&boxer_dss_device,
	&boxer_backlight_led_device,
#if !defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
	&boxer_keys_gpio,
#endif
#ifdef CONFIG_WL127X_RFKILL
//	&boxer_wl127x_device,
#endif
	&boxer_vout_device,
#ifdef CONFIG_REGULATOR_BQ24073
	&boxer_curr_regulator_device,
#endif
};

static void __init omap_boxer_init_irq(void)
{
#if defined(CONFIG_MACH_OMAP3621_BOXER_EVT1A)
	omap2_init_common_hw(	samsung_k4x4g303pb_sdrc_params,
				omap3621_mpu_rate_table,
				omap3621_dsp_rate_table,
				omap3621_l3_rate_table);
#else
	omap2_init_common_hw(	edd20323abh_sdrc_params,
				omap3621_mpu_rate_table,
				omap3621_dsp_rate_table,
				omap3621_l3_rate_table);
#endif
	omap_init_irq();
	omap_gpio_init();
}

static struct regulator_consumer_supply boxer_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply boxer_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply boxer_vmmc2_supply = {
	.supply		= "vmmc",
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data boxer_vmmc1 = {
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
	.consumer_supplies      = &boxer_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data boxer_vmmc2 = {
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
	.consumer_supplies      = &boxer_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data boxer_vsim = {
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
	.consumer_supplies      = &boxer_vsim_supply,
};

static struct regulator_init_data boxer_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &boxer_vdda_dac_supply,
};

static struct regulator_init_data boxer_vdsi = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &boxer_vdds_dsi_supply,
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

static int boxer_twl_gpio_setup(struct device *dev,
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
	boxer_vmmc1_supply.dev = mmc[0].dev;
	boxer_vsim_supply.dev = mmc[0].dev;
	boxer_vmmc2_supply.dev = mmc[1].dev;

	return 0;
}

static struct omap_lcd_config boxer_lcd_config __initdata = {
        .ctrl_name      = "internal",
};

static struct omap_uart_config boxer_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)),
};

static struct omap_board_config_kernel boxer_config[] __initdata = {
	{ OMAP_TAG_UART,	&boxer_uart_config },
        { OMAP_TAG_LCD,         &boxer_lcd_config },
};

static struct twl4030_usb_data boxer_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
#ifdef CONFIG_REGULATOR_BQ24073
	.bci_supply 	= &bq24073_vcharge_supply,
#endif
};

static struct twl4030_gpio_platform_data boxer_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= boxer_twl_gpio_setup,
};

static struct twl4030_madc_platform_data boxer_madc_data = {
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

static struct twl4030_power_data boxer_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.size		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_platform_data boxer_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &boxer_madc_data,
	.usb		= &boxer_usb_data,
	.gpio		= &boxer_gpio_data,
	.keypad		= &boxer_kp_twl4030_data,
	.power		= &boxer_t2scripts_data,
	.vmmc1          = &boxer_vmmc1,
	.vmmc2          = &boxer_vmmc2,
	.vsim           = &boxer_vsim,
	.vdac		= &boxer_vdac,
	.vpll2		= &boxer_vdsi,
};

static struct i2c_board_info __initdata boxer_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65921", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &boxer_twldata,
	},
};

#ifdef CONFIG_TOUCHSCREEN_CY8CTMG120_I2C
static void cy8ctmg120_dev_init(void)
{
	if (gpio_request(OMAP_CY8CTMG120_GPIO, "tmg120_touch") < 0) {
		printk(KERN_ERR "can't get tmg120 pen down GPIO\n");
		return;
	}

	gpio_direction_input(OMAP_CY8CTMG120_GPIO);
	omap_set_gpio_debounce(OMAP_CY8CTMG120_GPIO, 0);
}

static int tmg120_power(int power_state)
{
	/* TODO: synaptics is powered by vbatt */
	return 0;
}

static struct tmg120_platform_data cy8ctmg120_platform_data[] = {
	{
		.version	= 0x0,
		.power		= &tmg120_power,
		.flags		= TMG120_SWAP_XY,
		.irqflags	= IRQF_TRIGGER_FALLING, /* IRQF_TRIGGER_LOW,*/
	}
};
#endif


#ifdef CONFIG_TOUCHSCREEN_PIXCIR_I2C
static void pixcir_dev_init(void)
{
	
        printk("board-3621_boxer.c: pixcir_dev_init ...\n");
        if (gpio_request(PIXCIR_I2C_S32_GPIO, "pixcir_touch") < 0) {
                printk(KERN_ERR "can't get pixcir pen down GPIO\n");
                return;
        }
        printk("board-3621_boxer.c: pixcir_dev_init > set gpio 14 to output High!\n");
        gpio_direction_output(14, 1);

        printk("board-3621_boxer.c: pixcir_dev_init > Initialize pixcir irq pin %d !\n", PIXCIR_I2C_S32_GPIO);
        gpio_direction_input(PIXCIR_I2C_S32_GPIO);
        omap_set_gpio_debounce(PIXCIR_I2C_S32_GPIO, 0);  // set 0 to disable debounce
        //omap_set_gpio_debounce(PIXCIR_I2C_S32_GPIO, 1);
        //omap_set_gpio_debounce_time(PIXCIR_I2C_S32_GPIO, 0x1);	
}

static int pixcir_power(int power_state)
{
	return 0;
}
 
static struct pixcir_i2c_s32_platform_data pixcir_platform_data[] = {
	{
		.version	= 0x0,
		.power		= &pixcir_power,
		.flags		= 0,
		.irqflags	= IRQF_TRIGGER_FALLING , /* IRQF_TRIGGER_LOW,*/
	}
};
#endif

#ifdef CONFIG_INPUT_KXTF9
/* KIONIX KXTF9 Digital Tri-axis Accelerometer */

static void kxtf9_dev_init(void)
{
	printk("board-3621_boxer.c: kxtf9_dev_init ...\n");

	if (gpio_request(KXTF9_GPIO_FOR_PWR, "kxtf9_pwr") < 0) {
		printk(KERN_ERR "+++++++++++++ Can't get GPIO for kxtf9 power\n");
		return;
	}

	gpio_direction_output(KXTF9_GPIO_FOR_PWR, 1);
	
	if (gpio_request(KXTF9_GPIO_FOR_IRQ, "kxtf9_irq") < 0) {
		printk(KERN_ERR "Can't get GPIO for kxtf9 IRQ\n");
		return;
	}

	printk("board-3621_boxer.c: kxtf9_dev_init > Init kxtf9 irq pin %d !\n", KXTF9_GPIO_FOR_IRQ);
	gpio_direction_input(KXTF9_GPIO_FOR_IRQ);
	omap_set_gpio_debounce(KXTF9_GPIO_FOR_IRQ, 0);
}

struct kxtf9_platform_data kxtf9_platform_data_here = {
        .min_interval   = 1,
        .poll_interval  = 1000,

        .g_range        = KXTF9_G_8G,
        .shift_adj      = SHIFT_ADJ_2G,

        .axis_map_x     = 0,
        .axis_map_y     = 1,
        .axis_map_z     = 2,

        .negate_x       = 0,
        .negate_y       = 0,
        .negate_z       = 0,

        .data_odr_init          = ODR12_5F,
        .ctrl_reg1_init         = KXTF9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
        .int_ctrl_init          = KXTF9_IEN | KXTF9_IEA | KXTF9_IEL,
        .int_ctrl_init          = KXTF9_IEN,
        .tilt_timer_init        = 0x03,
        .engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
        .wuf_timer_init         = 0x16,
        .wuf_thresh_init        = 0x28,
        .tdt_timer_init         = 0x78,
        .tdt_h_thresh_init      = 0xFF,
        .tdt_l_thresh_init      = 0x14,
        .tdt_tap_timer_init     = 0x53,
        .tdt_total_timer_init   = 0x24,
        .tdt_latency_timer_init = 0x10,
        .tdt_window_timer_init  = 0xA0,

        .gpio = KXTF9_GPIO_FOR_IRQ,
};
#endif	/* CONFIG_INPUT_KXTF9 */



#define AIC3111_NAME "tlv320aic3111"
#define AIC3111_I2CSLAVEADDRESS 0x18
static struct i2c_board_info __initdata boxer_i2c_bus2_info[] = {
	{
#ifdef CONFIG_TOUCHSCREEN_CY8CTMG120_I2C	
		I2C_BOARD_INFO(CY8CTMG120_I2C_NAME, CY8CTMG120_I2C_SLAVEADDRESS),
		.platform_data = &cy8ctmg120_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_CY8CTMG120_GPIO),
#endif
#ifdef CONFIG_TOUCHSCREEN_PIXCIR_I2C 
		I2C_BOARD_INFO(PIXCIR_I2C_S32_NAME, PIXCIR_I2C_S32_SLAVEADDRESS),
		.platform_data = &pixcir_platform_data,
		.irq = OMAP_GPIO_IRQ(PIXCIR_I2C_S32_GPIO),
#endif
	},

#ifdef CONFIG_INPUT_KXTF9
	{
		I2C_BOARD_INFO(KXTF9_DEVICE_ID, KXTF9_I2C_SLAVE_ADDRESS),
		.platform_data = &kxtf9_platform_data_here,
		.irq = OMAP_GPIO_IRQ(KXTF9_GPIO_FOR_IRQ),
	},
#endif /* CONFIG_INPUT_KXTF9 */

#if defined(CONFIG_SND_SOC_TLV320AIC3111) || defined(CONFIG_SND_SOC_TLV320AIC3111_MODULE)
	{
		I2C_BOARD_INFO(AIC3111_NAME,  AIC3111_I2CSLAVEADDRESS),
	},
#endif
#if defined(CONFIG_BATTERY_BQ27510) || defined(CONFIG_BATTERY_BQ27510_MODULE)
	{
		I2C_BOARD_INFO("bq27510",  0x55),
	},
#endif
};


#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.vendor = "B&N     ",
	.product = "Ebook Disk      ",
	.release = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
		},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x18D1,
	.product_id	= 0x0001,
	.adb_product_id	= 0x0002,
	.version	= 0x0100,
	.product_name	= "Nook",
	.manufacturer_name = "B&N",
	.serial_number	= "11223344556677",
	.nluns = 2,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif


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

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}

	omap_register_i2c_bus(1, 100, boxer_i2c_bus1_info,
			ARRAY_SIZE(boxer_i2c_bus1_info));
	omap_register_i2c_bus(2, 100, boxer_i2c_bus2_info,
			ARRAY_SIZE(boxer_i2c_bus2_info));
	return 0;
}

#if 0
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
#endif

static void __init omap_boxer_init(void)
{
	boxer_sensors_power_enable_init();
	boxer_eMMC_power_enable_init();
	boxer_panel_enable_lcd(&boxer_lcd_device);

	omap_i2c_init();
	/* Fix to prevent VIO leakage on wl127x */
//	wl127x_vio_leakage_fix();
	platform_add_devices(boxer_devices, ARRAY_SIZE(boxer_devices));
	omap_board_config = boxer_config;
	omap_board_config_size = ARRAY_SIZE(boxer_config);
	spi_register_board_info(boxer_spi_board_info,
				ARRAY_SIZE(boxer_spi_board_info));
#ifdef  CONFIG_TOUCHSCREEN_CY8CTMG120_I2C 
	cy8ctmg120_dev_init();
#endif
#ifdef  CONFIG_TOUCHSCREEN_PIXCIR_I2C
	pixcir_dev_init();
#endif

#ifdef CONFIG_INPUT_KXTF9
	kxtf9_dev_init();
#endif /* CONFIG_INPUT_KXTF9 */

//	synaptics_dev_init();
//	msecure_init();
//	ldp_flash_init();
	omap_serial_init();
	usb_musb_init();
	boxer_backlight_init();

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
#endif

	BUG_ON(!cpu_is_omap3630());
}

static struct map_desc boxer_io_desc[] __initdata = {
	{
		.virtual	= ZOOM2_QUART_VIRT,
		.pfn		= __phys_to_pfn(ZOOM2_QUART_PHYS),
		.length		= ZOOM2_QUART_SIZE,
		.type		= MT_DEVICE
	},
};

static void __init omap_boxer_map_io(void)
{
	omap2_set_globals_343x();
	iotable_init(boxer_io_desc, ARRAY_SIZE(boxer_io_desc));
	omap2_map_common_io();
}

MACHINE_START(OMAP3621_BOXER, "OMAP3621 Boxer board")
	/* phys_io is only used for DEBUG_LL early printing.  The Boxer's
	 * console is on an external quad UART sitting at address 0x10000000
	 */
	.phys_io	= BOXER_EXT_QUART_PHYS,
	.io_pg_offst	= ((BOXER_EXT_QUART_VIRT) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_boxer_map_io,
	.init_irq	= omap_boxer_init_irq,
	.init_machine	= omap_boxer_init,
	.timer		= &omap_timer,
MACHINE_END
