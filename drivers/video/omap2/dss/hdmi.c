/*
 * linux/drivers/video/omap2/dss/hdmi.c
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Author: srinivas pulukuru <srinivas.pulukuru@ti.com>
 *
 * hdmi settings from TI's DSS driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "HDMI"

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sil9022.h>
#include <linux/i2c/twl4030.h>

#include <mach/board.h>
#include <mach/display.h>
#include <mach/cpu.h>

#include "dss.h"

/* for enabling VPLL2*/
#define ENABLE_VPLL2_DEDICATED          0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0
#define TWL4030_VPLL2_DEV_GRP           0x33
#define TWL4030_VPLL2_DEDICATED         0x36

struct omap_dss_device *omap_dss_hdmi_device;

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
static int enable_vpll2_power(int enable)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEDICATED : 0,
				TWL4030_VPLL2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEV_GRP : 0,
				TWL4030_VPLL2_DEV_GRP);
	return 0;
}

static int hdmi_set_dsi_clk(bool is_tft, unsigned long pck_req,
		unsigned long *fck, int *lck_div, int *pck_div)
{
	struct dsi_clock_info cinfo;
	int r;

	r = dsi_pll_calc_pck(is_tft, pck_req, &cinfo);
	if (r)
		return r;

	r = dsi_pll_program(&cinfo);
	if (r)
		return r;

	dss_select_clk_source(0, 1);

	dispc_set_lcd_divisor(cinfo.lck_div, cinfo.pck_div);

	*fck = cinfo.dsi1_pll_fclk;
	*lck_div = cinfo.lck_div;
	*pck_div = cinfo.pck_div;

	return 0;
}
#else
static int hdmi_set_dispc_clk(bool is_tft, unsigned long pck_req,
		unsigned long *fck, int *lck_div, int *pck_div)
{
	struct dispc_clock_info cinfo;
	int r;

	r = dispc_calc_clock_div(is_tft, pck_req, &cinfo);
	if (r)
		return r;

	r = dispc_set_clock_div(&cinfo);
	if (r)
		return r;

	*fck = cinfo.fck;
	*lck_div = cinfo.lck_div;
	*pck_div = cinfo.pck_div;

	return 0;
}
#endif

static int hdmi_set_mode(struct omap_dss_device *dssdev)
{
	struct omap_video_timings *t = &dssdev->panel.timings;
	int lck_div, pck_div;
	unsigned long fck;
	unsigned long pck;
	bool is_tft;
	int r = 0;

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dispc_set_pol_freq(dssdev->panel.config, dssdev->panel.acbi,
			dssdev->panel.acb);

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	r = hdmi_set_dsi_clk(is_tft, t->pixel_clock * 1000,
			&fck, &lck_div, &pck_div);
#else
	r = hdmi_set_dispc_clk(is_tft, t->pixel_clock * 1000,
			&fck, &lck_div, &pck_div);
#endif
	if (r)
		goto err0;

	pck = fck / lck_div / pck_div / 1000;

	if (pck != t->pixel_clock) {
		DSSWARN("Could not find exact pixel clock. "
				"Requested %d kHz, got %lu kHz\n",
				t->pixel_clock, pck);

		t->pixel_clock = pck;
	}

	dispc_set_lcd_timings(t);

err0:
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
	return r;
}

static int hdmi_basic_init(struct omap_dss_device *dssdev)
{
	bool is_tft;

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

	dispc_set_parallel_interface_mode(OMAP_DSS_PARALLELMODE_BYPASS);
	dispc_set_lcd_display_type(is_tft ? OMAP_DSS_LCD_DISPLAY_TFT :
			OMAP_DSS_LCD_DISPLAY_STN);
	dispc_set_tft_data_lines(dssdev->phy.dpi.data_lines);

	return 0;
}

static int hdmi_enable_display(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		DSSERR("display already enabled\n");
		r = -EINVAL;
		goto err1;
	}

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	r = hdmi_basic_init(dssdev);
	if (r)
		goto err2;

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	dss_clk_enable(DSS_CLK_FCK2);
	/* this needs to be done here in order to
	 * get the VPLL2 to power up the DSI PLL module
	 */
	enable_vpll2_power(1);

	r = dsi_pll_init(1, 1);
	if (r)
		goto err3;
#endif
	r = hdmi_set_mode(dssdev);
	if (r)
		goto err4;

	mdelay(2);

	dispc_enable_lcd_out(1);

	r = dssdev->driver->enable(dssdev);
	if (r)
		goto err5;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	/* Default HDMI panel timings may not work for all monitors */
	/* Reset HDMI panel timings after enabling HDMI. */
	DSSINFO("Reset HDMI output timings based on monitor E-EDID timings\n");
	hdmi_set_timings(dssdev, &dssdev->panel.timings);

	return 0;

err5:
	dispc_enable_lcd_out(0);
err4:
#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	dsi_pll_uninit();
err3:
	dss_clk_disable(DSS_CLK_FCK2);
#endif
err2:
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
err1:
	omap_dss_stop_device(dssdev);
err0:
	return r;
}

static int hdmi_display_resume(struct omap_dss_device *dssdev);

static void hdmi_disable_display(struct omap_dss_device *dssdev)
{
	DSSDBG("hdmi_disable_display\n");
	if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED)
		return;

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
		hdmi_display_resume(dssdev);

	dssdev->driver->disable(dssdev);

	dispc_enable_lcd_out(0);

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	dss_select_clk_source(0, 0);
	dsi_pll_uninit();
	enable_vpll2_power(0);
	dss_clk_disable(DSS_CLK_FCK2);
#endif

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	omap_dss_stop_device(dssdev);
}

static int hdmi_display_suspend(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	DSSDBG("hdmi_display_suspend\n");

	if (dssdev->driver->suspend)
		dssdev->driver->suspend(dssdev);

	dispc_enable_lcd_out(0);

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	dss_select_clk_source(0, 0);
	dsi_pll_uninit();
	enable_vpll2_power(0);
	dss_clk_disable(DSS_CLK_FCK2);
#endif

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int hdmi_display_resume(struct omap_dss_device *dssdev)
{
	int r = 0;
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	DSSDBG("hdmi_display_resume\n");

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	dss_clk_enable(DSS_CLK_FCK2);
	enable_vpll2_power(1);

	r = dsi_pll_init(1, 1);
	if (r)
		goto err0;

	r = hdmi_set_mode(dssdev);
	if (r)
		goto err0;
#endif

	dispc_enable_lcd_out(1);

	if (dssdev->driver->resume) {
		r = dssdev->driver->resume(dssdev);
		if (r)
			goto err1;
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;

err1:
	dispc_enable_lcd_out(0);

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
err0:
	DSSERR("<%s!!> err0: failed to init DSI_PLL = %d\n", __func__, r);
	dss_select_clk_source(0, 0);
	dsi_pll_uninit();
	dss_clk_disable(DSS_CLK_FCK2);
	enable_vpll2_power(0);
#endif
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
	return r;
}

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSDBG("hdmi_set_timings\n");
	dssdev->panel.timings = *timings;
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		hdmi_set_mode(dssdev);
		dispc_go(OMAP_DSS_CHANNEL_LCD);
	}
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	bool is_tft;
	int r;
	int lck_div, pck_div;
	unsigned long fck;
	unsigned long pck;

	DSSDBG("hdmi_check_timings\n");
	if (!dispc_lcd_timings_ok(timings))
		return -EINVAL;

	if (timings->pixel_clock == 0)
		return -EINVAL;

	is_tft = (dssdev->panel.config & OMAP_DSS_LCD_TFT) != 0;

#ifdef CONFIG_OMAP2_DSS_USE_DSI_PLL_FOR_HDMI
	{
		struct dsi_clock_info cinfo;
		r = dsi_pll_calc_pck(is_tft, timings->pixel_clock * 1000,
				&cinfo);

		if (r)
			return r;

		fck = cinfo.dsi1_pll_fclk;
		lck_div = cinfo.lck_div;
		pck_div = cinfo.pck_div;
	}
#else
	{
		struct dispc_clock_info cinfo;
		r = dispc_calc_clock_div(is_tft, timings->pixel_clock * 1000,
				&cinfo);

		if (r)
			return r;

		fck = cinfo.fck;
		lck_div = cinfo.lck_div;
		pck_div = cinfo.pck_div;
	}
#endif

	pck = fck / lck_div / pck_div / 1000;

	timings->pixel_clock = pck;

	return 0;
}

void hdmi_exit(void)
{
	omap_dss_hdmi_device->driver->remove(omap_dss_hdmi_device);
}

int hdmi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("init_display\n");

	dssdev->enable = hdmi_enable_display;
	dssdev->disable = hdmi_disable_display;
	dssdev->suspend = hdmi_display_suspend;
	dssdev->resume = hdmi_display_resume;
	dssdev->get_timings = hdmi_get_timings;
	dssdev->set_timings = hdmi_set_timings;
	dssdev->check_timings = hdmi_check_timings;

	/* store the dss device as we need this to unregister the
	 * dss_hdmi driver when exiting
	 */
	omap_dss_hdmi_device = dssdev;

	return 0;
}

