/*
 * linux/drivers/video/omap2/omapfb-main.c
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/omapfb.h>

#include <mach/display.h>
#include <mach/vram.h>
#include <mach/vrfb.h>

#include "omapfb.h"

#define MODULE_NAME     "omapfb"

static char *def_mode;
static char *def_vram;
static int def_vrfb;
static int def_rotate;
static int def_mirror;

#ifdef DEBUG
unsigned int omapfb_debug;
module_param_named(debug, omapfb_debug, bool, 0644);
static unsigned int omapfb_test_pattern;
module_param_named(test, omapfb_test_pattern, bool, 0644);
#endif

#ifdef DEBUG
static void draw_pixel(struct fb_info *fbi, int x, int y, unsigned color)
{
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	void __iomem *addr = fbi->screen_base;
	const unsigned bytespp = var->bits_per_pixel >> 3;
	const unsigned line_len = fix->line_length / bytespp;

	int r = (color >> 16) & 0xff;
	int g = (color >> 8) & 0xff;
	int b = (color >> 0) & 0xff;

	if (var->bits_per_pixel == 16) {
		u16 __iomem *p = (u16 __iomem *)addr;
		p += y * line_len + x;

		r = r * 32 / 256;
		g = g * 64 / 256;
		b = b * 32 / 256;

		__raw_writew((r << 11) | (g << 5) | (b << 0), p);
	} else if (var->bits_per_pixel == 24) {
		u8 __iomem *p = (u8 __iomem *)addr;
		p += (y * line_len + x) * 3;

		__raw_writeb(b, p + 0);
		__raw_writeb(g, p + 1);
		__raw_writeb(r, p + 2);
	} else if (var->bits_per_pixel == 32) {
		u32 __iomem *p = (u32 __iomem *)addr;
		p += y * line_len + x;
		__raw_writel(color, p);
	}
}

static void fill_fb(struct fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->var;
	const short w = var->xres_virtual;
	const short h = var->yres_virtual;
	void __iomem *addr = fbi->screen_base;
	int y, x;

	if (!addr)
		return;

	DBG("fill_fb %dx%d, line_len %d bytes\n", w, h, fbi->fix.line_length);

	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			if (x < 20 && y < 20)
				draw_pixel(fbi, x, y, 0xffffff);
			else if (x < 20 && (y > 20 && y < h - 20))
				draw_pixel(fbi, x, y, 0xff);
			else if (y < 20 && (x > 20 && x < w - 20))
				draw_pixel(fbi, x, y, 0xff00);
			else if (x > w - 20 && (y > 20 && y < h - 20))
				draw_pixel(fbi, x, y, 0xff0000);
			else if (y > h - 20 && (x > 20 && x < w - 20))
				draw_pixel(fbi, x, y, 0xffff00);
			else if (x == 20 || x == w - 20 ||
					y == 20 || y == h - 20)
				draw_pixel(fbi, x, y, 0xffffff);
			else if (x == y || w - x == h - y)
				draw_pixel(fbi, x, y, 0xff00ff);
			else if (w - x == y || x == h - y)
				draw_pixel(fbi, x, y, 0x00ffff);
			else if (x > 20 && y > 20 && x < w - 20 && y < h - 20) {
				int t = x * 3 / w;
				unsigned r = 0, g = 0, b = 0;
				unsigned c;
				if (var->bits_per_pixel == 16) {
					if (t == 0)
						b = (y % 32) * 256 / 32;
					else if (t == 1)
						g = (y % 64) * 256 / 64;
					else if (t == 2)
						r = (y % 32) * 256 / 32;
				} else {
					if (t == 0)
						b = (y % 256);
					else if (t == 1)
						g = (y % 256);
					else if (t == 2)
						r = (y % 256);
				}
				c = (r << 16) | (g << 8) | (b << 0);
				draw_pixel(fbi, x, y, c);
			} else {
				draw_pixel(fbi, x, y, 0);
			}
		}
	}
}
#endif

static unsigned omapfb_get_vrfb_offset(struct omapfb_info *ofbi, int rot)
{
	struct vrfb *vrfb = &ofbi->region.vrfb;
	unsigned offset;

	switch (rot) {
	case FB_ROTATE_UR:
		offset = 0;
		break;
	case FB_ROTATE_CW:
		offset = vrfb->yoffset;
		break;
	case FB_ROTATE_UD:
		offset = vrfb->yoffset * OMAP_VRFB_LINE_LEN + vrfb->xoffset;
		break;
	case FB_ROTATE_CCW:
		offset = vrfb->xoffset * OMAP_VRFB_LINE_LEN;
		break;
	default:
		BUG();
	}

	offset *= vrfb->bytespp;

	return offset;
}

static u32 omapfb_get_region_rot_paddr(struct omapfb_info *ofbi, int rot)
{
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
		return ofbi->region.vrfb.paddr[rot]
			+ omapfb_get_vrfb_offset(ofbi, rot);
	} else {
		return ofbi->region.paddr;
	}
}

u32 omapfb_get_region_paddr(struct omapfb_info *ofbi)
{
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
		return ofbi->region.vrfb.paddr[0];
	else
		return ofbi->region.paddr;
}

void __iomem *omapfb_get_region_vaddr(struct omapfb_info *ofbi)
{
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
		return ofbi->region.vrfb.vaddr[0];
	else
		return ofbi->region.vaddr;
}

static struct omapfb_colormode omapfb_colormodes[] = {
	{
		.dssmode = OMAP_DSS_COLOR_UYVY,
		.bits_per_pixel = 16,
		.nonstd = OMAPFB_COLOR_YUV422,
	}, {
		.dssmode = OMAP_DSS_COLOR_YUV2,
		.bits_per_pixel = 16,
		.nonstd = OMAPFB_COLOR_YUY422,
	}, {
		.dssmode = OMAP_DSS_COLOR_ARGB16,
		.bits_per_pixel = 16,
		.red	= { .length = 4, .offset = 8, .msb_right = 0 },
		.green	= { .length = 4, .offset = 4, .msb_right = 0 },
		.blue	= { .length = 4, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 4, .offset = 12, .msb_right = 0 },
	}, {
		.dssmode = OMAP_DSS_COLOR_RGB16,
		.bits_per_pixel = 16,
		.red	= { .length = 5, .offset = 11, .msb_right = 0 },
		.green	= { .length = 6, .offset = 5, .msb_right = 0 },
		.blue	= { .length = 5, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.dssmode = OMAP_DSS_COLOR_RGB24P,
		.bits_per_pixel = 24,
		.red	= { .length = 8, .offset = 16, .msb_right = 0 },
		.green	= { .length = 8, .offset = 8, .msb_right = 0 },
		.blue	= { .length = 8, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.dssmode = OMAP_DSS_COLOR_RGB24U,
		.bits_per_pixel = 32,
		.red	= { .length = 8, .offset = 16, .msb_right = 0 },
		.green	= { .length = 8, .offset = 8, .msb_right = 0 },
		.blue	= { .length = 8, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 0, .offset = 0, .msb_right = 0 },
	}, {
		.dssmode = OMAP_DSS_COLOR_ARGB32,
		.bits_per_pixel = 32,
		.red	= { .length = 8, .offset = 16, .msb_right = 0 },
		.green	= { .length = 8, .offset = 8, .msb_right = 0 },
		.blue	= { .length = 8, .offset = 0, .msb_right = 0 },
		.transp	= { .length = 8, .offset = 24, .msb_right = 0 },
	}, {
		.dssmode = OMAP_DSS_COLOR_RGBA32,
		.bits_per_pixel = 32,
		.red	= { .length = 8, .offset = 24, .msb_right = 0 },
		.green	= { .length = 8, .offset = 16, .msb_right = 0 },
		.blue	= { .length = 8, .offset = 8, .msb_right = 0 },
		.transp	= { .length = 8, .offset = 0, .msb_right = 0 },
	}, {
		.dssmode = OMAP_DSS_COLOR_RGBX32,
		.bits_per_pixel = 32,
		.red	= { .length = 8, .offset = 24, .msb_right = 0 },
		.green	= { .length = 8, .offset = 16, .msb_right = 0 },
		.blue	= { .length = 8, .offset = 8, .msb_right = 0 },
		.transp	= { .length = 0, .offset = 0, .msb_right = 0 },
	},
};

static bool cmp_var_to_colormode(struct fb_var_screeninfo *var,
		struct omapfb_colormode *color)
{
	bool cmp_component(struct fb_bitfield *f1, struct fb_bitfield *f2)
	{
		return f1->length == f2->length &&
			f1->offset == f2->offset &&
			f1->msb_right == f2->msb_right;
	}

	if (var->bits_per_pixel == 0 ||
			var->red.length == 0 ||
			var->blue.length == 0 ||
			var->green.length == 0)
		return 0;

	return var->bits_per_pixel == color->bits_per_pixel &&
		cmp_component(&var->red, &color->red) &&
		cmp_component(&var->green, &color->green) &&
		cmp_component(&var->blue, &color->blue) &&
		cmp_component(&var->transp, &color->transp);
}

static void assign_colormode_to_var(struct fb_var_screeninfo *var,
		struct omapfb_colormode *color)
{
	var->bits_per_pixel = color->bits_per_pixel;
	var->nonstd = color->nonstd;
	var->red = color->red;
	var->green = color->green;
	var->blue = color->blue;
	var->transp = color->transp;
}

static enum omap_color_mode fb_mode_to_dss_mode(struct fb_var_screeninfo *var)
{
	enum omap_color_mode dssmode;
	int i;

	/* first match with nonstd field */
	if (var->nonstd) {
		for (i = 0; i < ARRAY_SIZE(omapfb_colormodes); ++i) {
			struct omapfb_colormode *mode = &omapfb_colormodes[i];
			if (var->nonstd == mode->nonstd) {
				assign_colormode_to_var(var, mode);
				return mode->dssmode;
			}
		}

		return -EINVAL;
	}

	/* then try exact match of bpp and colors */
	for (i = 0; i < ARRAY_SIZE(omapfb_colormodes); ++i) {
		struct omapfb_colormode *mode = &omapfb_colormodes[i];
		if (cmp_var_to_colormode(var, mode)) {
			assign_colormode_to_var(var, mode);
			return mode->dssmode;
		}
	}

	/* match with bpp if user has not filled color fields
	 * properly */
	switch (var->bits_per_pixel) {
	case 1:
		dssmode = OMAP_DSS_COLOR_CLUT1;
		break;
	case 2:
		dssmode = OMAP_DSS_COLOR_CLUT2;
		break;
	case 4:
		dssmode = OMAP_DSS_COLOR_CLUT4;
		break;
	case 8:
		dssmode = OMAP_DSS_COLOR_CLUT8;
		break;
	case 12:
		dssmode = OMAP_DSS_COLOR_RGB12U;
		break;
	case 16:
		dssmode = OMAP_DSS_COLOR_RGB16;
		break;
	case 24:
		dssmode = OMAP_DSS_COLOR_RGB24P;
		break;
	case 32:
		dssmode = OMAP_DSS_COLOR_RGB24U;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(omapfb_colormodes); ++i) {
		struct omapfb_colormode *mode = &omapfb_colormodes[i];
		if (dssmode == mode->dssmode) {
			assign_colormode_to_var(var, mode);
			return mode->dssmode;
		}
	}

	return -EINVAL;
}

int dss_mode_to_fb_mode(enum omap_color_mode dssmode,
			struct fb_var_screeninfo *var)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(omapfb_colormodes); ++i) {
		struct omapfb_colormode *mode = &omapfb_colormodes[i];
		if (dssmode == mode->dssmode) {
			assign_colormode_to_var(var, mode);
			return 0;
		}
	}
	return -ENOENT;
}

void set_fb_fix(struct fb_info *fbi)
{
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_mem_region *rg = &ofbi->region;

	DBG("set_fb_fix\n");

	/* used by open/write in fbmem.c */
	fbi->screen_base = (char __iomem *)omapfb_get_region_vaddr(ofbi);

	DBG("changing rotation to %d\n", var->rotate);

	/* used by mmap in fbmem.c */
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
		switch (var->nonstd) {
		case OMAPFB_COLOR_YUV422:
		case OMAPFB_COLOR_YUY422:
			fix->line_length =
				(OMAP_VRFB_LINE_LEN * var->bits_per_pixel) >> 2;
			break;
		default:
			fix->line_length =
				(OMAP_VRFB_LINE_LEN * var->bits_per_pixel) >> 3;
			break;
		}
	} else
		fix->line_length =
			(var->xres_virtual * var->bits_per_pixel) >> 3;
	fix->smem_start = omapfb_get_region_paddr(ofbi);
	fix->smem_len = rg->size;

	fix->mmio_start = DSS_BASE;
	fix->mmio_len = DSS_SZ_REGS;

	fix->type = FB_TYPE_PACKED_PIXELS;

	if (var->nonstd)
		fix->visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		switch (var->bits_per_pixel) {
		case 32:
		case 24:
		case 16:
		case 12:
			fix->visual = FB_VISUAL_TRUECOLOR;
			/* 12bpp is stored in 16 bits */
			break;
		case 1:
		case 2:
		case 4:
		case 8:
			fix->visual = FB_VISUAL_PSEUDOCOLOR;
			break;
		}
	}

	fix->accel = FB_ACCEL_NONE;

	fix->xpanstep = 1;
	fix->ypanstep = 1;

	if (rg->size && ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
		enum omap_color_mode mode = 0;
		mode = fb_mode_to_dss_mode(var);

		omap_vrfb_setup(&rg->vrfb, rg->paddr,
				var->xres_virtual,
				var->yres_virtual,
				mode, 0);
	}
}

/* check new var and possibly modify it to be ok */
int check_fb_var(struct fb_info *fbi, struct fb_var_screeninfo *var)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omap_dss_device *display = fb2display(fbi);
	unsigned long max_frame_size;
	unsigned long line_size;
	int xres_min, yres_min;
	int xres_max, yres_max;
	enum omap_color_mode mode = 0;
	int i;
	int bytespp;

	DBG("check_fb_var %d\n", ofbi->id);

	if (ofbi->region.size == 0)
		return 0;

	mode = fb_mode_to_dss_mode(var);
	if (mode < 0) {
		DBG("cannot convert var to omap dss mode\n");
		return -EINVAL;
	}

	for (i = 0; i < ofbi->num_overlays; ++i) {
		if ((ofbi->overlays[i]->supported_modes & mode) == 0) {
			DBG("invalid mode\n");
			return -EINVAL;
		}
	}

	if (var->rotate < 0 || var->rotate > 3)
		return -EINVAL;

	xres_min = OMAPFB_PLANE_XRES_MIN;
	xres_max = 2048;
	yres_min = OMAPFB_PLANE_YRES_MIN;
	yres_max = 2048;

	bytespp = var->bits_per_pixel >> 3;

	/* XXX: some applications seem to set virtual res to 0. */
	if (var->xres_virtual == 0)
		var->xres_virtual = var->xres;

	if (var->yres_virtual == 0)
		var->yres_virtual = var->yres;

	if (var->xres_virtual < xres_min || var->yres_virtual < yres_min)
		return -EINVAL;

	if (var->xres < xres_min)
		var->xres = xres_min;
	if (var->yres < yres_min)
		var->yres = yres_min;
	if (var->xres > xres_max)
		var->xres = xres_max;
	if (var->yres > yres_max)
		var->yres = yres_max;

	if (var->xres > var->xres_virtual)
		var->xres = var->xres_virtual;
	if (var->yres > var->yres_virtual)
		var->yres = var->yres_virtual;

	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
		line_size = OMAP_VRFB_LINE_LEN * bytespp;
	else
		line_size = var->xres_virtual * bytespp;

	max_frame_size = ofbi->region.size;

	DBG("max frame size %lu, line size %lu\n", max_frame_size, line_size);

	if (line_size * var->yres_virtual > max_frame_size) {
		DBG("can't fit FB into memory, reducing y\n");
		var->yres_virtual = max_frame_size / line_size;

		if (var->yres_virtual < yres_min)
			var->yres_virtual = yres_min;

		if (var->yres > var->yres_virtual)
			var->yres = var->yres_virtual;
	}

	if (line_size * var->yres_virtual > max_frame_size) {
		DBG("can't fit FB into memory, reducing x\n");
		if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
			return -EINVAL;

		var->xres_virtual = max_frame_size / var->yres_virtual /
			bytespp;

		if (var->xres_virtual < xres_min)
			var->xres_virtual = xres_min;

		if (var->xres > var->xres_virtual)
			var->xres = var->xres_virtual;

		line_size = var->xres_virtual * bytespp;
	}

	if (line_size * var->yres_virtual > max_frame_size) {
		DBG("cannot fit FB to memory\n");
		return -EINVAL;
	}

	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;

	DBG("xres = %d, yres = %d, vxres = %d, vyres = %d\n",
			var->xres, var->yres,
			var->xres_virtual, var->yres_virtual);

	var->height             = -1;
	var->width              = -1;
	var->grayscale          = 0;

	if (display && display->get_timings) {
		struct omap_video_timings timings;
		display->get_timings(display, &timings);

		/* pixclock in ps, the rest in pixclock */
		var->pixclock = timings.pixel_clock != 0 ?
			KHZ2PICOS(timings.pixel_clock) :
			0;
		var->left_margin = timings.hfp;
		var->right_margin = timings.hbp;
		var->upper_margin = timings.vfp;
		var->lower_margin = timings.vbp;
		var->hsync_len = timings.hsw;
		var->vsync_len = timings.vsw;
	} else {
		var->pixclock = 0;
		var->left_margin = 0;
		var->right_margin = 0;
		var->upper_margin = 0;
		var->lower_margin = 0;
		var->hsync_len = 0;
		var->vsync_len = 0;
	}

	/* TODO: get these from panel->config */
	var->vmode              = FB_VMODE_NONINTERLACED;
	var->sync               = 0;

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * fbdev framework callbacks
 * ---------------------------------------------------------------------------
 */
static int omapfb_open(struct fb_info *fbi, int user)
{
	return 0;
}

static int omapfb_release(struct fb_info *fbi, int user)
{
#if 0
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_dss_device *display = fb2display(fbi);

	DBG("Closing fb with plane index %d\n", ofbi->id);

	omapfb_lock(fbdev);

	if (display && display->get_update_mode && display->update) {
		/* XXX this update should be removed, I think. But it's
		 * good for debugging */
		if (display->get_update_mode(display) ==
				OMAP_DSS_UPDATE_MANUAL) {
			u16 w, h;

			if (display->sync)
				display->sync(display);

			display->get_resolution(display, &w, &h);
			display->update(display, 0, 0, w, h);
		}
	}

	if (display && display->sync)
		display->sync(display);

	omapfb_unlock(fbdev);
#endif
	return 0;
}

/* setup overlay according to the fb */
static int omapfb_setup_overlay(struct fb_info *fbi, struct omap_overlay *ovl,
		u16 posx, u16 posy, u16 outw, u16 outh)
{
	int r = 0;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_fix_screeninfo *fix = &fbi->fix;
	enum omap_color_mode mode = 0;
	int offset;
	u32 data_start_p;
	void __iomem *data_start_v;
	struct omap_overlay_info info;
	int xres, yres;
	int screen_width;
	int mirror;
	int rotation = var->rotate;
	int i;

	for (i = 0; i < ofbi->num_overlays; i++) {
		if (ovl != ofbi->overlays[i])
			continue;

		rotation = (rotation + ofbi->rotation[i]) % 4;
		break;
	}

	DBG("setup_overlay %d, posx %d, posy %d, outw %d, outh %d\n", ofbi->id,
			posx, posy, outw, outh);

	if (rotation == FB_ROTATE_CW || rotation == FB_ROTATE_CCW) {
		xres = var->yres;
		yres = var->xres;
	} else {
		xres = var->xres;
		yres = var->yres;
	}

	offset = ((var->yoffset * var->xres_virtual +
				var->xoffset) * var->bits_per_pixel) >> 3;

	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
		data_start_p = omapfb_get_region_rot_paddr(ofbi, rotation);
		data_start_v = NULL;
	} else {
		data_start_p = omapfb_get_region_paddr(ofbi);
		data_start_v = omapfb_get_region_vaddr(ofbi);
	}

	data_start_p += offset;
	data_start_v += offset;

	mode = fb_mode_to_dss_mode(var);

	if (mode == -EINVAL) {
		DBG("fb_mode_to_dss_mode failed");
		r = -EINVAL;
		goto err;
	}

	switch (var->nonstd) {
	case OMAPFB_COLOR_YUV422:
	case OMAPFB_COLOR_YUY422:
		if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
			screen_width = fix->line_length
				/ (var->bits_per_pixel >> 2);
			break;
		}
	default:
		screen_width = fix->line_length / (var->bits_per_pixel >> 3);
		break;
	}

	ovl->get_overlay_info(ovl, &info);

	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
		mirror = 0;
	else
		mirror = ofbi->mirror;

	info.paddr = data_start_p;
	info.vaddr = data_start_v;
	info.screen_width = screen_width;
	info.width = xres;
	info.height = yres;
	info.color_mode = mode;
	info.rotation_type = ofbi->rotation_type;
	info.rotation = rotation;
	info.mirror = mirror;

	info.pos_x = posx;
	info.pos_y = posy;
	info.out_width = outw;
	info.out_height = outh;

	r = ovl->set_overlay_info(ovl, &info);
	if (r) {
		DBG("ovl->setup_overlay_info failed\n");
		goto err;
	}

	return 0;

err:
	DBG("setup_overlay failed\n");
	return r;
}

/* apply var to the overlay */
int omapfb_apply_changes(struct fb_info *fbi, int init)
{
	int r = 0;
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct fb_var_screeninfo *var = &fbi->var;
	struct omap_overlay *ovl;
	u16 posx, posy;
	u16 outw, outh;
	int i;

#ifdef DEBUG
	if (omapfb_test_pattern)
		fill_fb(fbi);
#endif

	for (i = 0; i < ofbi->num_overlays; i++) {
		ovl = ofbi->overlays[i];

		DBG("apply_changes, fb %d, ovl %d\n", ofbi->id, ovl->id);

		if (ofbi->region.size == 0) {
			/* the fb is not available. disable the overlay */
			omapfb_overlay_enable(ovl, 0);
			if (!init && ovl->manager)
				ovl->manager->apply(ovl->manager);
			continue;
		}

		if (init || (ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0) {
			int rotation = (var->rotate + ofbi->rotation[i]) % 4;
			if (rotation == FB_ROTATE_CW ||
					rotation == FB_ROTATE_CCW) {
				outw = var->yres;
				outh = var->xres;
			} else {
				outw = var->xres;
				outh = var->yres;
			}
		} else {
			outw = ovl->info.out_width;
			outh = ovl->info.out_height;
		}

		if (init) {
			posx = 0;
			posy = 0;
		} else {
			posx = ovl->info.pos_x;
			posy = ovl->info.pos_y;
		}

		r = omapfb_setup_overlay(fbi, ovl, posx, posy, outw, outh);
		if (r)
			goto err;

		if (!init && ovl->manager)
			ovl->manager->apply(ovl->manager);
	}
	return 0;
err:
	DBG("apply_changes failed\n");
	return r;
}

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int omapfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fbi)
{
	int r;

	DBG("check_var(%d)\n", FB2OFB(fbi)->id);

	r = check_fb_var(fbi, var);

	return r;
}

/* set the video mode according to info->var */
static int omapfb_set_par(struct fb_info *fbi)
{
	int r;

	DBG("set_par(%d)\n", FB2OFB(fbi)->id);

	set_fb_fix(fbi);
	r = omapfb_apply_changes(fbi, 0);

	return r;
}

int omapfb_update_window(struct fb_info *fbi,
		u32 x, u32 y, u32 w, u32 h);

static int omapfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *fbi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omap_dss_device *display = fb2display(fbi);
	int r = 0;

	DBG("pan_display(%d)\n", ofbi->id);

	if (var->xoffset != fbi->var.xoffset ||
	    var->yoffset != fbi->var.yoffset) {
		struct fb_var_screeninfo new_var;

		new_var = fbi->var;
		new_var.xoffset = var->xoffset;
		new_var.yoffset = var->yoffset;

		r = check_fb_var(fbi, &new_var);

		if (r == 0) {
			fbi->var = new_var;
			set_fb_fix(fbi);
			r = omapfb_apply_changes(fbi, 0);
		}
	}

	if (display && display->update)
		display->update(display, 0, 0, var->xres, var->yres);

	return r;
}

static void mmap_user_open(struct vm_area_struct *vma)
{
	struct omapfb_info *ofbi = (struct omapfb_info *)vma->vm_private_data;

	atomic_inc(&ofbi->map_count);
}

static void mmap_user_close(struct vm_area_struct *vma)
{
	struct omapfb_info *ofbi = (struct omapfb_info *)vma->vm_private_data;

	atomic_dec(&ofbi->map_count);
}

static struct vm_operations_struct mmap_user_ops = {
	.open = mmap_user_open,
	.close = mmap_user_close,
};

static int omapfb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct fb_fix_screeninfo *fix = &fbi->fix;
	unsigned long off;
	unsigned long start;
	u32 len;

	if (vma->vm_end - vma->vm_start == 0)
		return 0;
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	off = vma->vm_pgoff << PAGE_SHIFT;

	start = omapfb_get_region_paddr(ofbi);
	len = fix->smem_len;
	if (off >= len)
		return -EINVAL;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;

	DBG("user mmap region start %lx, len %d, off %lx\n", start, len, off);

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &mmap_user_ops;
	vma->vm_private_data = ofbi;
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			     vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	/* vm_ops.open won't be called for mmap itself. */
	atomic_inc(&ofbi->map_count);
	return 0;
}

/* Store a single color palette entry into a pseudo palette or the hardware
 * palette if one is available. For now we support only 16bpp and thus store
 * the entry only to the pseudo palette.
 */
static int _setcolreg(struct fb_info *fbi, u_int regno, u_int red, u_int green,
		u_int blue, u_int transp, int update_hw_pal)
{
	/*struct omapfb_info *ofbi = FB2OFB(fbi);*/
	/*struct omapfb2_device *fbdev = ofbi->fbdev;*/
	struct fb_var_screeninfo *var = &fbi->var;
	int r = 0;

	enum omapfb_color_format mode = OMAPFB_COLOR_RGB24U; /* XXX */

	/*switch (plane->color_mode) {*/
	switch (mode) {
	case OMAPFB_COLOR_YUV422:
	case OMAPFB_COLOR_YUV420:
	case OMAPFB_COLOR_YUY422:
		r = -EINVAL;
		break;
	case OMAPFB_COLOR_CLUT_8BPP:
	case OMAPFB_COLOR_CLUT_4BPP:
	case OMAPFB_COLOR_CLUT_2BPP:
	case OMAPFB_COLOR_CLUT_1BPP:
		/*
		   if (fbdev->ctrl->setcolreg)
		   r = fbdev->ctrl->setcolreg(regno, red, green, blue,
		   transp, update_hw_pal);
		   */
		/* Fallthrough */
		r = -EINVAL;
		break;
	case OMAPFB_COLOR_RGB565:
	case OMAPFB_COLOR_RGB444:
	case OMAPFB_COLOR_RGB24P:
	case OMAPFB_COLOR_RGB24U:
		if (r != 0)
			break;

		if (regno < 0) {
			r = -EINVAL;
			break;
		}

		if (regno < 16) {
			u16 pal;
			pal = ((red >> (16 - var->red.length)) <<
					var->red.offset) |
				((green >> (16 - var->green.length)) <<
				 var->green.offset) |
				(blue >> (16 - var->blue.length));
			((u32 *)(fbi->pseudo_palette))[regno] = pal;
		}
		break;
	default:
		BUG();
	}
	return r;
}

static int omapfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		u_int transp, struct fb_info *info)
{
	DBG("setcolreg\n");

	return _setcolreg(info, regno, red, green, blue, transp, 1);
}

static int omapfb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;

	DBG("setcmap\n");

	red     = cmap->red;
	green   = cmap->green;
	blue    = cmap->blue;
	transp  = cmap->transp;
	index   = cmap->start;

	for (count = 0; count < cmap->len; count++) {
		if (transp)
			trans = *transp++;
		r = _setcolreg(info, index++, *red++, *green++, *blue++, trans,
				count == cmap->len - 1);
		if (r != 0)
			return r;
	}

	return 0;
}

static void omapfb_vrfb_suspend_all(struct omapfb2_device *fbdev)
{
	int i;

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[i]);

		if (ofbi->region.vrfb.vaddr[0])
			omap_vrfb_suspend_ctx(&ofbi->region.vrfb);
	}
}

static void omapfb_vrfb_resume_all(struct omapfb2_device *fbdev)
{
	int i;

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[i]);

		if (ofbi->region.vrfb.vaddr[0])
			omap_vrfb_resume_ctx(&ofbi->region.vrfb);
	}
}

static int omapfb_blank(int blank, struct fb_info *fbi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_dss_device *display = fb2display(fbi);
	int do_update = 0;
	int r = 0;

	omapfb_lock(fbdev);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		if (display->state != OMAP_DSS_DISPLAY_SUSPENDED)
			goto exit;

		omapfb_vrfb_resume_all(fbdev);

		if (display->resume)
			r = display->resume(display);

		if (r == 0 && display->get_update_mode &&
				display->get_update_mode(display) ==
				OMAP_DSS_UPDATE_MANUAL)
			do_update = 1;

		break;

	case FB_BLANK_NORMAL:
		/* FB_BLANK_NORMAL could be implemented.
		 * Needs DSS additions. */
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		if (display->state != OMAP_DSS_DISPLAY_ACTIVE)
			goto exit;

		if (display->suspend)
			r = display->suspend(display);

		omapfb_vrfb_suspend_all(fbdev);

		break;

	default:
		r = -EINVAL;
	}

exit:
	omapfb_unlock(fbdev);

	if (r == 0 && do_update && display->update) {
		u16 w, h;
		display->get_resolution(display, &w, &h);

		r = display->update(display, 0, 0, w, h);
	}

	return r;
}

#if 0
/* XXX fb_read and fb_write are needed for VRFB */
ssize_t omapfb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	DBG("omapfb_write %d, %lu\n", count, (unsigned long)*ppos);
	/* XXX needed for VRFB */
	return count;
}
#endif

static struct fb_ops omapfb_ops = {
	.owner          = THIS_MODULE,
	.fb_open        = omapfb_open,
	.fb_release     = omapfb_release,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_blank       = omapfb_blank,
	.fb_ioctl       = omapfb_ioctl,
	.fb_check_var   = omapfb_check_var,
	.fb_set_par     = omapfb_set_par,
	.fb_pan_display = omapfb_pan_display,
	.fb_mmap	= omapfb_mmap,
	.fb_setcolreg	= omapfb_setcolreg,
	.fb_setcmap	= omapfb_setcmap,
	/*.fb_write	= omapfb_write,*/
};

static void omapfb_free_fbmem(struct fb_info *fbi)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omapfb2_mem_region *rg;

	rg = &ofbi->region;

	if (rg->paddr)
		if (omap_vram_free(rg->paddr, rg->size))
			dev_err(fbdev->dev, "VRAM FREE failed\n");

	if (rg->vaddr)
		iounmap(rg->vaddr);

	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
		/* unmap the 0 angle rotation */
		if (rg->vrfb.vaddr[0]) {
			iounmap(rg->vrfb.vaddr[0]);
			omap_vrfb_release_ctx(&rg->vrfb);
		}
	}

	rg->vaddr = NULL;
	rg->paddr = 0;
	rg->alloc = 0;
	rg->size = 0;
}

static void clear_fb_info(struct fb_info *fbi)
{
	memset(&fbi->var, 0, sizeof(fbi->var));
	memset(&fbi->fix, 0, sizeof(fbi->fix));
	strlcpy(fbi->fix.id, MODULE_NAME, sizeof(fbi->fix.id));
}

static int omapfb_free_all_fbmem(struct omapfb2_device *fbdev)
{
	int i;

	DBG("free all fbmem\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct fb_info *fbi = fbdev->fbs[i];
		omapfb_free_fbmem(fbi);
		clear_fb_info(fbi);
	}

	return 0;
}

static int omapfb_alloc_fbmem(struct fb_info *fbi, unsigned long size,
		unsigned long paddr)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omapfb2_mem_region *rg;
	void __iomem *vaddr;
	int r;

	rg = &ofbi->region;
	memset(rg, 0, sizeof(*rg));

	size = PAGE_ALIGN(size);

	if (!paddr) {
		DBG("allocating %lu bytes for fb %d\n", size, ofbi->id);
		r = omap_vram_alloc(OMAPFB_MEMTYPE_SDRAM, size, &paddr);
	} else {
		DBG("reserving %lu bytes at %lx for fb %d\n", size, paddr,
				ofbi->id);
		r = omap_vram_reserve(paddr, size);
	}

	if (r) {
		dev_err(fbdev->dev, "failed to allocate framebuffer\n");
		return -ENOMEM;
	}

	if (ofbi->rotation_type != OMAP_DSS_ROT_VRFB) {
		vaddr = ioremap_wc(paddr, size);

		if (!vaddr) {
			dev_err(fbdev->dev, "failed to ioremap framebuffer\n");
			omap_vram_free(paddr, size);
			return -ENOMEM;
		}

		DBG("allocated VRAM paddr %lx, vaddr %p\n", paddr, vaddr);
	} else {
		void __iomem *va;

		r = omap_vrfb_request_ctx(&rg->vrfb);
		if (r) {
			dev_err(fbdev->dev, "vrfb create ctx failed\n");
			return r;
		}

		/* only ioremap the 0 angle view */
		va = ioremap_wc(rg->vrfb.paddr[0], size);

		if (!va) {
			printk(KERN_ERR "vrfb: ioremap failed\n");
			omap_vrfb_release_ctx(&rg->vrfb);
			return -ENOMEM;
		}

		DBG("ioremapped vrfb area 0 to %p\n", va);

		rg->vrfb.vaddr[0] = va;

		vaddr = NULL;
	}

	rg->paddr = paddr;
	rg->vaddr = vaddr;
	rg->size = size;
	rg->alloc = 1;

	return 0;
}

/* allocate fbmem using display resolution as reference */
static int omapfb_alloc_fbmem_display(struct fb_info *fbi, unsigned long size,
		unsigned long paddr)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omap_dss_device *display;
	int bytespp;

	display =  fb2display(fbi);

	if (!display)
		return 0;

	switch (display->get_recommended_bpp(display)) {
	case 16:
		bytespp = 2;
		break;
	case 24:
#ifdef CONFIG_FB_OMAP2_32_BPP
	case 32:
#endif
		bytespp = 4;
		break;
	default:
		bytespp = 4;
		break;
	}

	if (!size) {
		u16 w, h;

		display->get_resolution(display, &w, &h);

		if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
#ifdef DEBUG
			int oldw = w, oldh = h;
#endif

			omap_vrfb_adjust_size(&w, &h, bytespp);

			/* Because we change the resolution of the 0 degree
			 * view, we need to alloc max(w, h) for height */
			h = max(w, h);
			w = OMAP_VRFB_LINE_LEN;

			DBG("adjusting fb mem size for VRFB, %dx%d -> %dx%d\n",
					oldw, oldh, w, h);
		}

		size = w * h * bytespp;
	}

	if (!size)
		return 0;

	return omapfb_alloc_fbmem(fbi, size, paddr);
}

static enum omap_color_mode fb_format_to_dss_mode(enum omapfb_color_format fmt)
{
	enum omap_color_mode mode;

	switch (fmt) {
	case OMAPFB_COLOR_RGB565:
		mode = OMAP_DSS_COLOR_RGB16;
		break;
	case OMAPFB_COLOR_YUV422:
		mode = OMAP_DSS_COLOR_YUV2;
		break;
	case OMAPFB_COLOR_CLUT_8BPP:
		mode = OMAP_DSS_COLOR_CLUT8;
		break;
	case OMAPFB_COLOR_CLUT_4BPP:
		mode = OMAP_DSS_COLOR_CLUT4;
		break;
	case OMAPFB_COLOR_CLUT_2BPP:
		mode = OMAP_DSS_COLOR_CLUT2;
		break;
	case OMAPFB_COLOR_CLUT_1BPP:
		mode = OMAP_DSS_COLOR_CLUT1;
		break;
	case OMAPFB_COLOR_RGB444:
		mode = OMAP_DSS_COLOR_RGB12U;
		break;
	case OMAPFB_COLOR_YUY422:
		mode = OMAP_DSS_COLOR_UYVY;
		break;
	case OMAPFB_COLOR_ARGB16:
		mode = OMAP_DSS_COLOR_ARGB16;
		break;
	case OMAPFB_COLOR_RGB24U:
		mode = OMAP_DSS_COLOR_RGB24U;
		break;
	case OMAPFB_COLOR_RGB24P:
		mode = OMAP_DSS_COLOR_RGB24P;
		break;
	case OMAPFB_COLOR_ARGB32:
		mode = OMAP_DSS_COLOR_ARGB32;
		break;
	case OMAPFB_COLOR_RGBA32:
		mode = OMAP_DSS_COLOR_RGBA32;
		break;
	case OMAPFB_COLOR_RGBX32:
		mode = OMAP_DSS_COLOR_RGBX32;
		break;
	default:
		mode = -EINVAL;
	}

	return mode;
}

static int omapfb_parse_vram_param(const char *param, int max_entries,
		unsigned long *sizes, unsigned long *paddrs)
{
	int fbnum;
	unsigned long size;
	unsigned long paddr = 0;
	char *p, *start;

	start = (char *)param;

	while (1) {
		p = start;

		fbnum = simple_strtoul(p, &p, 10);

		if (p == param)
			return -EINVAL;

		if (*p != ':')
			return -EINVAL;

		if (fbnum >= max_entries)
			return -EINVAL;

		size = memparse(p + 1, &p);

		if (!size)
			return -EINVAL;

		paddr = 0;

		if (*p == '@') {
			paddr = simple_strtoul(p + 1, &p, 16);

			if (!paddr)
				return -EINVAL;

		}

		paddrs[fbnum] = paddr;
		sizes[fbnum] = size;

		if (*p == 0)
			break;

		if (*p != ',')
			return -EINVAL;

		++p;

		start = p;
	}

	return 0;
}

static int omapfb_allocate_all_fbs(struct omapfb2_device *fbdev)
{
	int i, r;
	unsigned long vram_sizes[10];
	unsigned long vram_paddrs[10];

	memset(&vram_sizes, 0, sizeof(vram_sizes));
	memset(&vram_paddrs, 0, sizeof(vram_paddrs));

	if (def_vram &&	omapfb_parse_vram_param(def_vram, 10,
				vram_sizes, vram_paddrs)) {
		dev_err(fbdev->dev, "failed to parse vram parameter\n");

		memset(&vram_sizes, 0, sizeof(vram_sizes));
		memset(&vram_paddrs, 0, sizeof(vram_paddrs));
	}

	if (fbdev->dev->platform_data) {
		struct omapfb_platform_data *opd;
		opd = fbdev->dev->platform_data;
		for (i = 0; i < opd->mem_desc.region_cnt; ++i) {
			if (!vram_sizes[i]) {
				unsigned long size;
				unsigned long paddr;

				size = opd->mem_desc.region[i].size;
				paddr = opd->mem_desc.region[i].paddr;

				vram_sizes[i] = size;
				vram_paddrs[i] = paddr;
			}
		}
	}

	for (i = 0; i < fbdev->num_fbs; i++) {
		/* allocate memory automatically only for fb0, or if
		 * excplicitly defined with vram or plat data option */
		if (i == 0 || vram_sizes[i] != 0) {
			r = omapfb_alloc_fbmem_display(fbdev->fbs[i],
					vram_sizes[i], vram_paddrs[i]);

			if (r)
				return r;
		}
	}

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[i]);
		struct omapfb2_mem_region *rg;
		rg = &ofbi->region;

		DBG("region%d phys %08x virt %p size=%lu\n",
				i,
				rg->paddr,
				rg->vaddr,
				rg->size);
	}

	return 0;
}

int omapfb_realloc_fbmem(struct fb_info *fbi, unsigned long size, int type)
{
	struct omapfb_info *ofbi = FB2OFB(fbi);
	struct omapfb2_device *fbdev = ofbi->fbdev;
	struct omap_dss_device *display = fb2display(fbi);
	struct omapfb2_mem_region *rg = &ofbi->region;
	unsigned long old_size = rg->size;
	unsigned long old_paddr = rg->paddr;
	int old_type = rg->type;
	int r;

	if (type > OMAPFB_MEMTYPE_MAX)
		return -EINVAL;

	size = PAGE_ALIGN(size);

	if (old_size == size && old_type == type)
		return 0;

	if (display && display->sync)
			display->sync(display);

	omapfb_free_fbmem(fbi);

	if (size == 0) {
		clear_fb_info(fbi);
		return 0;
	}

	r = omapfb_alloc_fbmem(fbi, size, 0);

	if (r) {
		if (old_size)
			omapfb_alloc_fbmem(fbi, old_size, old_paddr);

		if (rg->size == 0)
			clear_fb_info(fbi);

		return r;
	}

	if (old_size == size)
		return 0;

	if (old_size == 0) {
		DBG("initializing fb %d\n", ofbi->id);
		r = omapfb_fb_init(fbdev, fbi);
		if (r) {
			DBG("omapfb_fb_init failed\n");
			goto err;
		}
		r = omapfb_apply_changes(fbi, 1);
		if (r) {
			DBG("omapfb_apply_changes failed\n");
			goto err;
		}
	} else {
		struct fb_var_screeninfo new_var;
		memcpy(&new_var, &fbi->var, sizeof(new_var));
		r = check_fb_var(fbi, &new_var);
		if (r)
			goto err;
		memcpy(&fbi->var, &new_var, sizeof(fbi->var));
		set_fb_fix(fbi);
	}

	return 0;
err:
	omapfb_free_fbmem(fbi);
	clear_fb_info(fbi);
	return r;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void omapfb_suspend(struct early_suspend *h)
{
	struct omapfb_info *fbinfo = container_of(h, struct omapfb_info,
						early_suspend);
	omapfb_blank(FB_BLANK_POWERDOWN, fbinfo->fbdev->fbs[fbinfo->id]);
}

void omapfb_resume(struct early_suspend *h)
{
	struct omapfb_info *fbinfo = container_of(h, struct omapfb_info,
						early_suspend);
	omapfb_blank(FB_BLANK_UNBLANK, fbinfo->fbdev->fbs[fbinfo->id]);
}
#endif

/* initialize fb_info, var, fix to something sane based on the display */
int omapfb_fb_init(struct omapfb2_device *fbdev, struct fb_info *fbi)
{
	struct fb_var_screeninfo *var = &fbi->var;
	struct omap_dss_device *display = fb2display(fbi);
	struct omapfb_info *ofbi = FB2OFB(fbi);
	int r = 0;

	fbi->fbops = &omapfb_ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = fbdev->pseudo_palette;

	if (ofbi->region.size == 0) {
		clear_fb_info(fbi);
		return 0;
	}

	var->nonstd = 0;
	var->bits_per_pixel = 0;

	var->rotate = def_rotate;

	/*
	 * Check if there is a default color format set in the board file,
	 * and use this format instead the default deducted from the
	 * display bpp.
	 */
	if (fbdev->dev->platform_data) {
		struct omapfb_platform_data *opd;
		int id = ofbi->id;

		opd = fbdev->dev->platform_data;
		if (opd->mem_desc.region[id].format_used) {
			enum omap_color_mode mode;
			enum omapfb_color_format format;

			format = opd->mem_desc.region[id].format;
			mode = fb_format_to_dss_mode(format);
			if (mode < 0) {
				r = mode;
				goto err;
			}
			r = dss_mode_to_fb_mode(mode, var);
			if (r < 0)
				goto err;
		}
	}

	if (display) {
		u16 w, h;
		int rotation = (var->rotate + ofbi->rotation[0]) % 4;

		display->get_resolution(display, &w, &h);

		if (rotation == FB_ROTATE_CW ||
				rotation == FB_ROTATE_CCW) {
			var->xres = h;
			var->yres = w;
		} else {
			var->xres = w;
			var->yres = h;
		}

		var->xres_virtual = var->xres;
		var->yres_virtual = var->yres;

		if (!var->bits_per_pixel) {
			switch (display->get_recommended_bpp(display)) {
			case 16:
				var->bits_per_pixel = 16;
				break;
			case 24:
#ifdef CONFIG_FB_OMAP2_32_BPP
			case 32:
#endif
				var->bits_per_pixel = 32;
				break;
			default:
				dev_err(fbdev->dev, "illegal display "
						"bpp\n");
				return -EINVAL;
			}
		}
	} else {
		/* if there's no display, let's just guess some basic values */
		var->xres = 320;
		var->yres = 240;
		var->xres_virtual = var->xres;
		var->yres_virtual = var->yres;
		if (!var->bits_per_pixel)
			var->bits_per_pixel = 16;
	}

	r = check_fb_var(fbi, var);
	if (r)
		goto err;

	set_fb_fix(fbi);

	r = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (r)
		dev_err(fbdev->dev, "unable to allocate color map memory\n");
#ifdef CONFIG_HAS_EARLYSUSPEND
	ofbi->early_suspend.suspend = omapfb_suspend;
	ofbi->early_suspend.resume = omapfb_resume;
	ofbi->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&ofbi->early_suspend);
#endif

err:
	return r;
}

static void fbinfo_cleanup(struct omapfb2_device *fbdev, struct fb_info *fbi)
{
	fb_dealloc_cmap(&fbi->cmap);
}


static void omapfb_free_resources(struct omapfb2_device *fbdev)
{
	int i;

	DBG("free_resources\n");

	if (fbdev == NULL)
		return;

	for (i = 0; i < fbdev->num_fbs; i++)
		unregister_framebuffer(fbdev->fbs[i]);

	/* free the reserved fbmem */
	omapfb_free_all_fbmem(fbdev);

	for (i = 0; i < fbdev->num_fbs; i++) {
		fbinfo_cleanup(fbdev, fbdev->fbs[i]);
		framebuffer_release(fbdev->fbs[i]);
	}

	for (i = 0; i < fbdev->num_displays; i++) {
		if (fbdev->displays[i]->state != OMAP_DSS_DISPLAY_DISABLED)
			fbdev->displays[i]->disable(fbdev->displays[i]);

		omap_dss_put_device(fbdev->displays[i]);
	}

	dev_set_drvdata(fbdev->dev, NULL);
	kfree(fbdev);
}

static int omapfb_create_framebuffers(struct omapfb2_device *fbdev)
{
	int r, i;

	fbdev->num_fbs = 0;

	DBG("create %d framebuffers\n",	CONFIG_FB_OMAP2_NUM_FBS);

	/* allocate fb_infos */
	for (i = 0; i < CONFIG_FB_OMAP2_NUM_FBS; i++) {
		struct fb_info *fbi;
		struct omapfb_info *ofbi;

		fbi = framebuffer_alloc(sizeof(struct omapfb_info),
				fbdev->dev);

		if (fbi == NULL) {
			dev_err(fbdev->dev,
				"unable to allocate memory for plane info\n");
			return -ENOMEM;
		}

		clear_fb_info(fbi);

		fbdev->fbs[i] = fbi;

		ofbi = FB2OFB(fbi);
		ofbi->fbdev = fbdev;
		ofbi->id = i;

		/* assign these early, so that fb alloc can use them */
		ofbi->rotation_type = def_vrfb ? OMAP_DSS_ROT_VRFB :
			OMAP_DSS_ROT_DMA;
		ofbi->mirror = def_mirror;

		fbdev->num_fbs++;
	}

	DBG("fb_infos allocated\n");

	/* assign overlays for the fbs */
	for (i = 0; i < min(fbdev->num_fbs, fbdev->num_overlays); i++) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[i]);

		ofbi->overlays[0] = fbdev->overlays[i];
		ofbi->num_overlays = 1;
	}

	/* allocate fb memories */
	r = omapfb_allocate_all_fbs(fbdev);
	if (r) {
		dev_err(fbdev->dev, "failed to allocate fbmem\n");
		return r;
	}

	DBG("fbmems allocated\n");

	/* setup fb_infos */
	for (i = 0; i < fbdev->num_fbs; i++) {
		r = omapfb_fb_init(fbdev, fbdev->fbs[i]);
		if (r) {
			dev_err(fbdev->dev, "failed to setup fb_info\n");
			return r;
		}
	}

	DBG("fb_infos initialized\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		r = register_framebuffer(fbdev->fbs[i]);
		if (r != 0) {
			dev_err(fbdev->dev,
				"registering framebuffer %d failed\n", i);
			return r;
		}
	}

	DBG("framebuffers registered\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		r = omapfb_apply_changes(fbdev->fbs[i], 1);
		if (r) {
			dev_err(fbdev->dev, "failed to change mode\n");
			return r;
		}
	}

	DBG("create sysfs for fbs\n");
	r = omapfb_create_sysfs(fbdev);
	if (r) {
		dev_err(fbdev->dev, "failed to create sysfs entries\n");
		return r;
	}

	/* Enable fb0 */
	if (fbdev->num_fbs > 0) {
		struct omapfb_info *ofbi = FB2OFB(fbdev->fbs[0]);

		if (ofbi->num_overlays > 0) {
			struct omap_overlay *ovl = ofbi->overlays[0];

			r = omapfb_overlay_enable(ovl, 1);

			if (r) {
				dev_err(fbdev->dev,
						"failed to enable overlay\n");
				return r;
			}
		}
	}

	DBG("create_framebuffers done\n");

	return 0;
}

int omapfb_mode_to_timings(const char *mode_str,
		struct omap_video_timings *timings, u8 *bpp)
{
	struct fb_info fbi;
	struct fb_var_screeninfo var;
	struct fb_ops fbops;
	int r;

#ifdef CONFIG_OMAP2_DSS_VENC
	if (strcmp(mode_str, "pal") == 0) {
		*timings = omap_dss_pal_timings;
		*bpp = 0;
		return 0;
	} else if (strcmp(mode_str, "ntsc") == 0) {
		*timings = omap_dss_ntsc_timings;
		*bpp = 0;
		return 0;
	}
#endif

	/* this is quite a hack, but I wanted to use the modedb and for
	 * that we need fb_info and var, so we create dummy ones */

	memset(&fbi, 0, sizeof(fbi));
	memset(&var, 0, sizeof(var));
	memset(&fbops, 0, sizeof(fbops));
	fbi.fbops = &fbops;

	r = fb_find_mode(&var, &fbi, mode_str, NULL, 0, NULL, 24);

	if (r != 0) {
		timings->pixel_clock = PICOS2KHZ(var.pixclock);
		timings->hfp = var.left_margin;
		timings->hbp = var.right_margin;
		timings->vfp = var.upper_margin;
		timings->vbp = var.lower_margin;
		timings->hsw = var.hsync_len;
		timings->vsw = var.vsync_len;
		timings->x_res = var.xres;
		timings->y_res = var.yres;

		switch (var.bits_per_pixel) {
		case 16:
			*bpp = 16;
			break;
		case 24:
		case 32:
		default:
			*bpp = 24;
			break;
		}

		return 0;
	} else {
		return -EINVAL;
	}
}

static int omapfb_set_def_mode(struct omap_dss_device *display, char *mode_str)
{
	int r;
	u8 bpp;
	struct omap_video_timings timings;

	r = omapfb_mode_to_timings(mode_str, &timings, &bpp);
	if (r)
		return r;

	display->panel.recommended_bpp = bpp;

	if (!display->check_timings || !display->set_timings)
		return -EINVAL;

	r = display->check_timings(display, &timings);
	if (r)
		return r;

	display->set_timings(display, &timings);

	return 0;
}

static int omapfb_parse_def_modes(struct omapfb2_device *fbdev)
{
	char *str, *options, *this_opt;
	int r = 0;

	str = kmalloc(strlen(def_mode) + 1, GFP_KERNEL);
	strcpy(str, def_mode);
	options = str;

	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		char *p, *display_str, *mode_str;
		struct omap_dss_device *display;
		int i;

		p = strchr(this_opt, ':');
		if (!p) {
			r = -EINVAL;
			break;
		}

		*p = 0;
		display_str = this_opt;
		mode_str = p + 1;

		display = NULL;
		for (i = 0; i < fbdev->num_displays; ++i) {
			if (strcmp(fbdev->displays[i]->name,
						display_str) == 0) {
				display = fbdev->displays[i];
				break;
			}
		}

		if (!display) {
			r = -EINVAL;
			break;
		}

		r = omapfb_set_def_mode(display, mode_str);
		if (r)
			break;
	}

	kfree(str);

	return r;
}

static int omapfb_probe(struct platform_device *pdev)
{
	struct omapfb2_device *fbdev = NULL;
	int r = 0;
	int i;
	struct omap_overlay *ovl;
	struct omap_dss_device *def_display;
	struct omap_dss_device *dssdev;

	DBG("omapfb_probe\n");

	if (pdev->num_resources != 0) {
		dev_err(&pdev->dev, "probed for an unknown device\n");
		r = -ENODEV;
		goto err0;
	}

	fbdev = kzalloc(sizeof(struct omapfb2_device), GFP_KERNEL);
	if (fbdev == NULL) {
		r = -ENOMEM;
		goto err0;
	}

	mutex_init(&fbdev->mtx);

	fbdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, fbdev);

	fbdev->num_displays = 0;
	dssdev = NULL;
	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);
		fbdev->displays[fbdev->num_displays++] = dssdev;
	}

	if (fbdev->num_displays == 0) {
		dev_err(&pdev->dev, "no displays\n");
		r = -EINVAL;
		goto cleanup;
	}

	fbdev->num_overlays = omap_dss_get_num_overlays();
	for (i = 0; i < fbdev->num_overlays; i++)
		fbdev->overlays[i] = omap_dss_get_overlay(i);

	fbdev->num_managers = omap_dss_get_num_overlay_managers();
	for (i = 0; i < fbdev->num_managers; i++)
		fbdev->managers[i] = omap_dss_get_overlay_manager(i);


	/* gfx overlay should be the default one. find a display
	 * connected to that, and use it as default display */
	ovl = omap_dss_get_overlay(0);
	if (ovl->manager && ovl->manager->device) {
		def_display = ovl->manager->device;
	} else {
		dev_err(&pdev->dev, "cannot find default display\n");
		r = -EINVAL;
		goto cleanup;
	}

	if (def_mode && strlen(def_mode) > 0) {
		if (omapfb_parse_def_modes(fbdev))
			dev_err(&pdev->dev, "cannot parse default modes\n");
	}

	r = omapfb_create_framebuffers(fbdev);
	if (r)
		goto cleanup;

	for (i = 0; i < fbdev->num_managers; i++) {
		struct omap_overlay_manager *mgr;
		mgr = fbdev->managers[i];
		r = mgr->apply(mgr);
		if (r) {
			dev_err(fbdev->dev, "failed to apply dispc config\n");
			goto cleanup;
		}
	}

	DBG("mgr->apply'ed\n");

	r = def_display->enable(def_display);
	if (r) {
		dev_err(fbdev->dev, "Failed to enable display '%s'\n",
				def_display->name);
		goto cleanup;
	}

	/* set the update mode */
	if (def_display->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
#ifdef CONFIG_FB_OMAP2_FORCE_AUTO_UPDATE
		if (def_display->enable_te)
			def_display->enable_te(def_display, 1);
		if (def_display->set_update_mode)
			def_display->set_update_mode(def_display,
					OMAP_DSS_UPDATE_AUTO);
#else
		if (def_display->enable_te)
			def_display->enable_te(def_display, 0);
		if (def_display->set_update_mode)
			def_display->set_update_mode(def_display,
					OMAP_DSS_UPDATE_MANUAL);
#endif
	} else {
		if (def_display->set_update_mode)
			def_display->set_update_mode(def_display,
					OMAP_DSS_UPDATE_AUTO);
	}

	for (i = 0; i < fbdev->num_displays; i++) {
		struct omap_dss_device *display = fbdev->displays[i];
		u16 w, h;

		if (!display->get_update_mode || !display->update)
			continue;

		if (display->get_update_mode(display) ==
				OMAP_DSS_UPDATE_MANUAL) {

			display->get_resolution(display, &w, &h);
			display->update(display, 0, 0, w, h);
		}
	}

	DBG("display->updated\n");

	return 0;

cleanup:
	omapfb_free_resources(fbdev);
err0:
	dev_err(&pdev->dev, "failed to setup omapfb\n");
	return r;
}

static int omapfb_remove(struct platform_device *pdev)
{
	struct omapfb2_device *fbdev = platform_get_drvdata(pdev);

	/* FIXME: wait till completion of pending events */

	omapfb_remove_sysfs(fbdev);

	omapfb_free_resources(fbdev);

	return 0;
}

static struct platform_driver omapfb_driver = {
	.probe          = omapfb_probe,
	.remove         = omapfb_remove,
	.driver         = {
		.name   = "omapfb",
		.owner  = THIS_MODULE,
	},
};

static int __init omapfb_init(void)
{
	DBG("omapfb_init\n");

	if (platform_driver_register(&omapfb_driver)) {
		printk(KERN_ERR "failed to register omapfb driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit omapfb_exit(void)
{
	DBG("omapfb_exit\n");
	platform_driver_unregister(&omapfb_driver);
}

module_param_named(mode, def_mode, charp, 0);
module_param_named(vram, def_vram, charp, 0);
module_param_named(rotate, def_rotate, int, 0);
module_param_named(vrfb, def_vrfb, bool, 0);
module_param_named(mirror, def_mirror, bool, 0);

/* late_initcall to let panel/ctrl drivers loaded first.
 * I guess better option would be a more dynamic approach,
 * so that omapfb reacts to new panels when they are loaded */
late_initcall(omapfb_init);
/*module_init(omapfb_init);*/
module_exit(omapfb_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@nokia.com>");
MODULE_DESCRIPTION("OMAP2/3 Framebuffer");
MODULE_LICENSE("GPL v2");
