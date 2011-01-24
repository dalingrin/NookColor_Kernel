/*
 * LCD panel support for the TI LDP board
 *
 * Copyright (C) 2007 WindRiver
 * Author: Stanley Miao <stanley.miao@windriver.com>
 *
 * Derived from drivers/video/omap/lcd-2430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/i2c/twl4030.h>
#include <linux/spi/spi.h>
#include <linux/omapfb.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <asm/mach-types.h>
#include <mach/control.h>

#define LCD_PANEL_BACKLIGHT_GPIO 	(15 + OMAP_MAX_GPIO_LINES)
#define LCD_PANEL_ENABLE_GPIO 		(7 + OMAP_MAX_GPIO_LINES)

#define LCD_PANEL_RESET_GPIO		55
#define LCD_PANEL_QVGA_GPIO		56

#define LCD_XRES		800
#define LCD_YRES		480
#define LCD_PIXCLOCK_MAX	45871

#define PM_RECEIVER             TWL4030_MODULE_PM_RECEIVER
#define ENABLE_VAUX2_DEDICATED  0x09
#define ENABLE_VAUX2_DEV_GRP    0x20
#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20

#define ENABLE_VPLL2_DEDICATED          0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0
#define TWL4030_VPLL2_DEV_GRP           0x33
#define TWL4030_VPLL2_DEDICATED         0x36

#define t2_out(c, r, v) twl4030_i2c_write_u8(c, r, v)


static int zoom2_panel_init(struct lcd_panel *panel,
			    struct omapfb_device *fbdev)
{
	return 0;
}

static void zoom2_panel_cleanup(struct lcd_panel *panel)
{
}

static int zoom2_panel_enable(struct lcd_panel *panel)
{
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				     ENABLE_VPLL2_DEDICATED,
				     TWL4030_VPLL2_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				     ENABLE_VPLL2_DEV_GRP,
				     TWL4030_VPLL2_DEV_GRP);
	}

	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 1);
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 1);
	return 0;
}

static void zoom2_panel_disable(struct lcd_panel *panel)
{
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0,
				     TWL4030_VPLL2_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0,
				     TWL4030_VPLL2_DEV_GRP);
	}

	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);
	mdelay(4);
}

static unsigned long zoom2_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

struct lcd_panel zoom2_panel = {
	.name		= "zoom2",
	.config		= OMAP_LCDC_PANEL_TFT | OMAP_LCDC_INV_VSYNC | OMAP_LCDC_INV_HSYNC,

	.bpp		= 16,
	.data_lines	= 24,
	.x_res		= LCD_XRES,
	.y_res		= LCD_YRES,
	.hsw		= 0,		/* hsync_len (1) - 1 */
	.hfp		= 6,		/* right_margin (6) - 1 */
	.hbp		= 4,		/* left_margin (4) - 1 */
	.vsw		= 0,		/* vsync_len (1) - 1 */
	.vfp		= 2,		/* lower_margin */
	.vbp		= 7,		/* upper_margin (4) - 1 */
	.acb		= 0x28,

	.pixel_clock	= LCD_PIXCLOCK_MAX,

	.init		= zoom2_panel_init,
	.cleanup	= zoom2_panel_cleanup,
	.enable		= zoom2_panel_enable,
	.disable	= zoom2_panel_disable,
	.get_caps	= zoom2_panel_get_caps,
};

static int
spi_send(struct spi_device *spi, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = 0;
	unsigned int cmd = 0;
	unsigned int data = 0;
	cmd = 0x0000 | reg_addr; /* register address write */
	data = 0x0100 | reg_data ; /* register data write */
	data = (cmd << 16) | data;
	if (spi_write(spi, (unsigned char *)&data, 4))
		printk("error in spi_write %x\n", data);

	udelay(10);
	return ret;
}

static int init_nec_wvga_lcd(struct spi_device *spi)
{
        /* Initialization Sequence */
	spi_send(spi, 3, 0x01);    /* R3 = 01h */
	spi_send(spi, 0, 0x00);    /* R0 = 00h */
	spi_send(spi, 1, 0x01);    /* R1 = 0x01 (normal), 0x03 (reversed) */
	spi_send(spi, 4, 0x00);    /* R4 = 00h */
	spi_send(spi, 5, 0x14);    /* R5 = 14h */
	spi_send(spi, 6, 0x24);    /* R6 = 24h */
	spi_send(spi, 16, 0xD7);   /* R16 = D7h */
	spi_send(spi, 17, 0x00);   /* R17 = 00h */
	spi_send(spi, 18, 0x00);   /* R18 = 00h */
	spi_send(spi, 19, 0x55);   /* R19 = 55h */
	spi_send(spi, 20, 0x01);   /* R20 = 01h */
	spi_send(spi, 21, 0x70);   /* R21 = 70h */
	spi_send(spi, 22, 0x1E);   /* R22 = 1Eh */
	spi_send(spi, 23, 0x25);   /* R23 = 25h */
	spi_send(spi, 24, 0x25);   /* R24 = 25h */
	spi_send(spi, 25, 0x02);   /* R25 = 02h */
	spi_send(spi, 26, 0x02);   /* R26 = 02h */
	spi_send(spi, 27, 0xA0);   /* R27 = A0h */
	spi_send(spi, 32, 0x2F);   /* R32 = 2Fh */
	spi_send(spi, 33, 0x0F);   /* R33 = 0Fh */
	spi_send(spi, 34, 0x0F);   /* R34 = 0Fh */
	spi_send(spi, 35, 0x0F);   /* R35 = 0Fh */
	spi_send(spi, 36, 0x0F);   /* R36 = 0Fh */
	spi_send(spi, 37, 0x0F);   /* R37 = 0Fh */
	spi_send(spi, 38, 0x0F);   /* R38 = 0Fh */
	spi_send(spi, 39, 0x00);   /* R39 = 00h */
	spi_send(spi, 40, 0x02);   /* R40 = 02h */
	spi_send(spi, 41, 0x02);   /* R41 = 02h */
	spi_send(spi, 42, 0x02);   /* R42 = 02h */
	spi_send(spi, 43, 0x0F);   /* R43 = 0Fh */
	spi_send(spi, 44, 0x0F);   /* R44 = 0Fh */
	spi_send(spi, 45, 0x0F);   /* R45 = 0Fh */
	spi_send(spi, 46, 0x0F);   /* R46 = 0Fh */
	spi_send(spi, 47, 0x0F);   /* R47 = 0Fh */
	spi_send(spi, 48, 0x0F);   /* R48 = 0Fh */
	spi_send(spi, 49, 0x0F);   /* R49 = 0Fh */
	spi_send(spi, 50, 0x00);   /* R50 = 00h */
	spi_send(spi, 51, 0x02);   /* R51 = 02h */
	spi_send(spi, 52, 0x02);   /* R52 = 02h */
	spi_send(spi, 53, 0x02);   /* R53 = 02h */
	spi_send(spi, 80, 0x0C);   /* R80 = 0Ch */
	spi_send(spi, 83, 0x42);   /* R83 = 42h */
	spi_send(spi, 84, 0x42);   /* R84 = 42h */
	spi_send(spi, 85, 0x41);   /* R85 = 41h */
	spi_send(spi, 86, 0x14);   /* R86 = 14h */
	spi_send(spi, 89, 0x88);   /* R89 = 88h */
	spi_send(spi, 90, 0x01);   /* R90 = 01h */
	spi_send(spi, 91, 0x00);   /* R91 = 00h */
	spi_send(spi, 92, 0x02);   /* R92 = 02h */
	spi_send(spi, 93, 0x0C);   /* R93 = 0Ch */
	spi_send(spi, 94, 0x1C);   /* R94 = 1Ch */
	spi_send(spi, 95, 0x27);   /* R95 = 27h */
	spi_send(spi, 98, 0x49);   /* R98 = 49h */
	spi_send(spi, 99, 0x27);   /* R99 = 27h */
	spi_send(spi, 102, 0x76);  /* R102 = 76h */
	spi_send(spi, 103, 0x27);  /* R103 = 27h */
	spi_send(spi, 112, 0x01);  /* R112 = 01h */
	spi_send(spi, 113, 0x0E);  /* R113 = 0Eh */
	spi_send(spi, 114, 0x02);  /* R114 = 02h */
	spi_send(spi, 115, 0x0C);  /* R115 = 0Ch */
	spi_send(spi, 118, 0x0C);  /* R118 = 0Ch */
	spi_send(spi, 121, 0x30); /* R121 = 0x30 (normal), 0x10 (reversed) */
	spi_send(spi, 130, 0x00);  /* R130 = 00h */
	spi_send(spi, 131, 0x00);  /* R131 = 00h */
	spi_send(spi, 132, 0xFC);  /* R132 = FCh */
	spi_send(spi, 134, 0x00);  /* R134 = 00h */
	spi_send(spi, 136, 0x00);  /* R136 = 00h */
	spi_send(spi, 138, 0x00);  /* R138 = 00h */
	spi_send(spi, 139, 0x00);  /* R139 = 00h */
	spi_send(spi, 140, 0x00);  /* R140 = 00h */
	spi_send(spi, 141, 0xFC);  /* R141 = FCh */
	spi_send(spi, 143, 0x00);  /* R143 = 00h */
	spi_send(spi, 145, 0x00);  /* R145 = 00h */
	spi_send(spi, 147, 0x00);  /* R147 = 00h */
	spi_send(spi, 148, 0x00);  /* R148 = 00h */
	spi_send(spi, 149, 0x00);  /* R149 = 00h */
	spi_send(spi, 150, 0xFC);  /* R150 = FCh */
	spi_send(spi, 152, 0x00);  /* R152 = 00h */
	spi_send(spi, 154, 0x00);  /* R154 = 00h */
	spi_send(spi, 156, 0x00);  /* R156 = 00h */
	spi_send(spi, 157, 0x00);  /* R157 = 00h */
	udelay(20);
	spi_send(spi, 2, 0x00);    /* R2 = 00h */
	return 0;
}



static int zoom2_lcd_probe(struct spi_device *spi)
{
        omap_cfg_reg(AF21_3430_GPIO8);
        omap_cfg_reg(B23_3430_GPIO167);
        omap_cfg_reg(AB1_3430_McSPI1_CS2);
        spi->mode = SPI_MODE_0;
        spi->bits_per_word = 32;
        spi_setup(spi);

	gpio_request(LCD_PANEL_RESET_GPIO, "lcd reset");
	gpio_request(LCD_PANEL_QVGA_GPIO, "lcd qvga");
	gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd panel");
	gpio_request(LCD_PANEL_BACKLIGHT_GPIO, "lcd backlight");

	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 0);
	gpio_direction_output(LCD_PANEL_RESET_GPIO, 0);
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);

	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 1);
	gpio_direction_output(LCD_PANEL_RESET_GPIO, 1);

	init_nec_wvga_lcd(spi);

	omapfb_register_panel(&zoom2_panel);
	return 0;
}

static int
zoom2_lcd_suspend(struct spi_device *spi, pm_message_t state)
{
	spi_send(spi, 2, 0x01);  /* R2 = 01h */
	mdelay(40);
	return 0;
}

static int
zoom2_lcd_resume(struct spi_device *spi)
{
	/* reinitialize the panel */
	spi_setup(spi);
	spi_send(spi, 2, 0x00);  /* R2 = 00h */
	init_nec_wvga_lcd(spi);
	return 0;
}

static struct spi_driver zoom2_lcd_driver = {
	.probe           = zoom2_lcd_probe,
	.suspend         = zoom2_lcd_suspend,
	.resume          = zoom2_lcd_resume,
	.driver         = {
		.name   = "zoom2_lcd",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
};

static int __init zoom2_lcd_init(void)
{
	return spi_register_driver(&zoom2_lcd_driver);
}

static void __exit zoom2_lcd_exit(void)
{
	return spi_unregister_driver(&zoom2_lcd_driver);
}

module_init(zoom2_lcd_init);
module_exit(zoom2_lcd_exit);
