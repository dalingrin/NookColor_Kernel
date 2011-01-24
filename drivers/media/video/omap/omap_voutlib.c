/*
 * drivers/media/video/omap/omap_voutlib.c
 *
 * Copyright (C) 2005-2009 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Based on the OMAP2 camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2009 Texas Instruments.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/videodev2.h>
#include <linux/semaphore.h>
#include <mach/cpu.h>


/* Return the default overlay cropping rectangle in crop given the image
 * size in pix and the video display size in fbuf.  The default
 * cropping rectangle is the largest rectangle no larger than the capture size
 * that will fit on the display.  The default cropping rectangle is centered in
 * the image.  All dimensions and offsets are rounded down to even numbers.
 */
void omap_vout_default_crop(struct v4l2_pix_format *pix,
		  struct v4l2_framebuffer *fbuf, struct v4l2_rect *crop)
{
	crop->width = (pix->width < fbuf->fmt.width) ?
		pix->width : fbuf->fmt.width;
	crop->height = (pix->height < fbuf->fmt.height) ?
		pix->height : fbuf->fmt.height;
	crop->width &= ~1;
	crop->height &= ~1;
	crop->left = ((pix->width - crop->width) >> 1) & ~1;
	crop->top = ((pix->height - crop->height) >> 1) & ~1;
}
EXPORT_SYMBOL_GPL(omap_vout_default_crop);

/* Given a new render window in new_win, adjust the window to the
 * nearest supported configuration.  The adjusted window parameters are
 * returned in new_win.
 * Returns zero if succesful, or -EINVAL if the requested window is
 * impossible and cannot reasonably be adjusted.
 */
int omap_vout_try_window(struct v4l2_framebuffer *fbuf,
			struct v4l2_window *new_win)
{
	struct v4l2_rect try_win;

	/* make a working copy of the new_win rectangle */
	try_win = new_win->w;

	/* adjust the preview window so it fits on the display by clipping any
	 * offscreen areas
	 */
	if (try_win.left < 0) {
		try_win.width += try_win.left;
		try_win.left = 0;
	}
	if (try_win.top < 0) {
		try_win.height += try_win.top;
		try_win.top = 0;
	}
	try_win.width = (try_win.width < fbuf->fmt.width) ?
		try_win.width : fbuf->fmt.width;
	try_win.height = (try_win.height < fbuf->fmt.height) ?
		try_win.height : fbuf->fmt.height;
	if (try_win.left + try_win.width > fbuf->fmt.width)
		try_win.width = fbuf->fmt.width - try_win.left;
	if (try_win.top + try_win.height > fbuf->fmt.height)
		try_win.height = fbuf->fmt.height - try_win.top;
	try_win.width &= ~1;
	try_win.height &= ~1;

	if (try_win.width <= 0 || try_win.height <= 0)
		return -EINVAL;

	/* We now have a valid preview window, so go with it */
	new_win->w = try_win;
	new_win->field = V4L2_FIELD_ANY;
	return 0;
}
EXPORT_SYMBOL_GPL(omap_vout_try_window);

/* Given a new render window in new_win, adjust the window to the
 * nearest supported configuration.  The image cropping window in crop
 * will also be adjusted if necessary.  Preference is given to keeping the
 * the window as close to the requested configuration as possible.  If
 * successful, new_win, vout->win, and crop are updated.
 * Returns zero if succesful, or -EINVAL if the requested preview window is
 * impossible and cannot reasonably be adjusted.
 */
int omap_vout_new_window(struct v4l2_rect *crop,
		struct v4l2_window *win, struct v4l2_framebuffer *fbuf,
		struct v4l2_window *new_win)
{
	int err;

	err = omap_vout_try_window(fbuf, new_win);
	if (err)
		return err;

	/* update our preview window */
	win->w = new_win->w;
	win->field = new_win->field;
	win->chromakey = new_win->chromakey;

	if (cpu_is_omap24xx()) {
		/* adjust the cropping window to allow for resizing
		 * limitations. 24xx allow 8x to 1/2x scaling.
		 */
		if ((crop->height/win->w.height) >= 2) {
			/* The maximum vertical downsizing ratio is 2:1 */
			crop->height = win->w.height * 2;
		}
		if ((crop->width/win->w.width) >= 2) {
			/* The maximum horizontal downsizing ratio is 2:1 */
			crop->width = win->w.width * 2;
		}
		if (crop->width > 768) {
			/* The OMAP2420 vertical resizing line buffer is 768
			 * pixels  wide.  If the cropped image is wider than
			 * 768 pixels then it cannot be vertically resized.
			 */
			if (crop->height != win->w.height)
				crop->width = 768;
		}
	} else {
		/* adjust the cropping window to allow for resizing
		 * limitations 34xx allow 8x to 1/8x scaling.
		 */
		if ((crop->height/win->w.height) >= 8) {
			/* The maximum vertical downsizing ratio is 8:1 */
			crop->height = win->w.height * 8;
		}
		if ((crop->width/win->w.width) >= 8) {
			/* The maximum horizontal downsizing ratio is 8:1 */
			crop->width = win->w.width * 8;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(omap_vout_new_window);

/* Given a new cropping rectangle in new_crop, adjust the cropping rectangle to
 * the nearest supported configuration.  The image render window in win will
 * also be adjusted if necessary.  The preview window is adjusted such that the
 * horizontal and vertical rescaling ratios stay constant.  If the render
 * window would fall outside the display boundaries, the cropping rectangle
 * will also be adjusted to maintain the rescaling ratios.  If successful, crop
 * and win are updated.
 * Returns zero if succesful, or -EINVAL if the requested cropping rectangle is
 * impossible and cannot reasonably be adjusted.
 */
int omap_vout_new_crop(struct v4l2_pix_format *pix,
	      struct v4l2_rect *crop, struct v4l2_window *win,
	      struct v4l2_framebuffer *fbuf, const struct v4l2_rect *new_crop,
		       int *use_isp_rsz_for_downscale)
{
	struct v4l2_rect try_crop;
	unsigned long vresize, hresize;

	/* make a working copy of the new_crop rectangle */
	try_crop = *new_crop;

	/* adjust the cropping rectangle so it fits in the image */
	if (try_crop.left < 0) {
		try_crop.width += try_crop.left;
		try_crop.left = 0;
	}
	if (try_crop.top < 0) {
		try_crop.height += try_crop.top;
		try_crop.top = 0;
	}
	try_crop.width = (try_crop.width < pix->width) ?
		try_crop.width : pix->width;
	try_crop.height = (try_crop.height < pix->height) ?
		try_crop.height : pix->height;
	if (try_crop.left + try_crop.width > pix->width)
		try_crop.width = pix->width - try_crop.left;
	if (try_crop.top + try_crop.height > pix->height)
		try_crop.height = pix->height - try_crop.top;
	try_crop.width &= ~1;
	try_crop.height &= ~1;
	if (try_crop.width <= 0 || try_crop.height <= 0)
		return -EINVAL;
	if (cpu_is_omap24xx()) {
		if (crop->height != win->w.height) {
			/* If we're resizing vertically, we can't support a
			 * crop width wider than 768 pixels.
			 */
			if (try_crop.width > 768)
				try_crop.width = 768;
		}
	}

	/* vertical resizing */
	vresize = (1024 * try_crop.height) / win->w.height;
	if (cpu_is_omap24xx()) {
		if (vresize > 2048)
			vresize = 2048;
	} else {
		if (vresize > 4096) {
			*use_isp_rsz_for_downscale = 1;
			printk(KERN_ERR "\n<%s> Using ISP resizer vresize "
					"= %lu\n\n",
			       __func__, vresize);
			if (vresize > 8096)
				vresize = 8096;
		}
	}
	win->w.height = ((1024 * try_crop.height) / vresize) & ~1;
	if (win->w.height == 0)
		win->w.height = 2;
	if (win->w.height + win->w.top > fbuf->fmt.height) {
		/* We made the preview window extend below the bottom of the
		 * display, so clip it to the display boundary and resize the
		 * cropping height to maintain the vertical resizing ratio.
		 */
		win->w.height = (fbuf->fmt.height - win->w.top) & ~1;
		if (try_crop.height == 0)
			try_crop.height = 2;
	}
	/* horizontal resizing */
	hresize = (1024 * try_crop.width) / win->w.width;
	if (cpu_is_omap24xx()) {
		if (hresize > 2048)
			hresize = 2048;
	} else {
		/* DSS DMA resizer handles the 8x to 1/4x horz scaling
		 * for 1/4x to 1/8x scaling ISP resizer is used
		 * for width > 1024 and scaling 1/2x-1/8x ISP resizer is used
		 */
		if (hresize > 4096 ||
		    (hresize > 2048 && try_crop.width > 1024)) {
			*use_isp_rsz_for_downscale = 1;
			printk(KERN_ERR "\n<%s> Using ISP resizer "
					"hresize = %lu, width = %u\n\n",
			       __func__, hresize, try_crop.width);
			if (hresize > 8096)
				hresize = 8096;
		}
	}
	win->w.width = ((1024 * try_crop.width) / hresize) & ~1;
	if (win->w.width == 0)
		win->w.width = 2;
	if (win->w.width + win->w.left > fbuf->fmt.width) {
		/* We made the preview window extend past the right side of the
		 * display, so clip it to the display boundary and resize the
		 * cropping width to maintain the horizontal resizing ratio.
		 */
		win->w.width = (fbuf->fmt.width - win->w.left) & ~1;
		if (try_crop.width == 0)
			try_crop.width = 2;
	}
	if (cpu_is_omap24xx()) {
		/* Check for resizing constraints
		 * 24xx allow 8x to 1/2x scaling.
		 */
		if ((try_crop.height/win->w.height) >= 2) {
			/* The maximum vertical downsizing ratio is 2:1 */
			try_crop.height = win->w.height * 2;
		}
		if ((try_crop.width/win->w.width) >= 2) {
			/* The maximum horizontal downsizing ratio is 2:1 */
			try_crop.width = win->w.width * 2;
		}
		if (try_crop.width > 768) {
			/* The OMAP2420 vertical resizing line buffer is
			 * 768 pixels wide.  If the cropped image is wider
			 * than 768 pixels then it cannot be vertically resized.
			 */
			if (try_crop.height != win->w.height)
				try_crop.width = 768;
		}
	} else {
		/* Check for resizing constraints
		 * 34xx DSS allow 8x to 1/4x scaling.
		 * ISP resizer handles downscaling from 1/4x to 1/8x
		 */
		if ((try_crop.height/win->w.height) >= 8) {
			/* The maximum vertical downsizing ratio is 8:1 */
			try_crop.height = win->w.height * 8;
		}
		if ((try_crop.width/win->w.width) >= 8) {
			/* The maximum horizontal downsizing ratio is 8:1 */
			try_crop.width = win->w.width * 8;
		}
	}
	/* update our cropping rectangle and we're done */
	*crop = try_crop;
	return 0;
}
EXPORT_SYMBOL_GPL(omap_vout_new_crop);

/* Given a new format in pix and fbuf,  crop and win
 * structures are initialized to default values. crop
 * is initialized to the largest window size that will fit on the display.  The
 * crop window is centered in the image. win is initialized to
 * the same size as crop and is centered on the display.
 * All sizes and offsets are constrained to be even numbers.
 */
void omap_vout_new_format(struct v4l2_pix_format *pix,
		struct v4l2_framebuffer *fbuf, struct v4l2_rect *crop,
		struct v4l2_window *win)
{
	/* crop defines the preview source window in the image capture
	 * buffer
	 */
	omap_vout_default_crop(pix, fbuf, crop);

	/* win defines the preview target window on the display */
	win->w.width = crop->width;
	win->w.height = crop->height;
	win->w.left = ((fbuf->fmt.width - win->w.width) >> 1) & ~1;
	win->w.top = ((fbuf->fmt.height - win->w.height) >> 1) & ~1;
}
EXPORT_SYMBOL_GPL(omap_vout_new_format);

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("OMAP Video library");
MODULE_LICENSE("GPL");
