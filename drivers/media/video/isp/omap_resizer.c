/*
 * drivers/media/video/isp/omap_resizer.c
 *
 * Wrapper for Resizer module in TI's OMAP3430 ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Contributors:
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Troy Laramy <t-laramy@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <media/v4l2-dev.h>
#include <asm/cacheflush.h>
#include <mach/iovmm.h>

#include "isp.h"
#include "ispreg.h"
#include "ispresizer.h"
#include <linux/omap_resizer.h>

#define OMAP_REZR_NAME		"omap-resizer"

/* Defines and Constants*/
#define MAX_CHANNELS		16
#define ALIGNMENT		16
#define CHANNEL_BUSY		1
#define CHANNEL_FREE		0
#define PIXEL_EVEN		2
#define RATIO_MULTIPLIER	256
#define MAX_4TAP_OUTWIDTH_3430	3300
#define MAX_7TAP_OUTWIDTH_3430	1650
#define MAX_4TAP_OUTWIDTH_3630	4096
#define MAX_7TAP_OUTWIDTH_3630	2048
/* Bit position Macro */
/* macro for bit set and clear */
#define BITSET(variable, bit)	((variable) | (1 << bit))
#define BITRESET(variable, bit)	((variable) & ~(0x00000001 << (bit)))
#define SET_BIT_INPUTRAM	28
#define SET_BIT_CBLIN		29
#define SET_BIT_INPTYP		27
#define SET_BIT_YCPOS		26
#define INPUT_RAM		1
#define UP_RSZ_RATIO		64
#define DOWN_RSZ_RATIO		512
#define UP_RSZ_RATIO1		513
#define DOWN_RSZ_RATIO1		1024
#define RSZ_IN_SIZE_VERT_SHIFT	16
#define MAX_HORZ_PIXEL_8BIT	31
#define MAX_HORZ_PIXEL_16BIT	15
#define NUM_PHASES		8
#define NUM_TAPS		4
#define NUM_D2PH		4	/* for downsampling * 2+x ~ 4x,
					 * number of phases
					 */
#define NUM_D2TAPS		7 	/* for downsampling * 2+x ~ 4x,
					 * number of taps
					 */
#define ALIGN32			32
#define MAX_COEF_COUNTER	16
#define COEFF_ADDRESS_OFFSET	0x04

static DECLARE_MUTEX(resz_wrapper_mutex);
static int multipass_active;

static struct isp_interface_config reszwrap_config = {
	.ccdc_par_ser = ISP_NONE,
	.dataline_shift = 0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0,
	.prestrobe = 0,
	.shutter = 0,
	.wait_hs_vs = 0,
};

/* Register mapped structure which contains the every register
   information */
struct resizer_config {
	u32 rsz_pcr;				/* pcr register mapping
						 * variable.
						 */
	u32 rsz_in_start;			/* in_start register mapping
						 * variable.
						 */
	u32 rsz_in_size;			/* in_size register mapping
						 * variable.
						 */
	u32 rsz_out_size;			/* out_size register mapping
						 * variable.
						 */
	u32 rsz_cnt;				/* rsz_cnt register mapping
						 * variable.
						 */
	u32 rsz_sdr_inadd;			/* sdr_inadd register mapping
						 * variable.
						 */
	u32 rsz_sdr_inoff;			/* sdr_inoff register mapping
						 * variable.
						 */
	u32 rsz_sdr_outadd;			/* sdr_outadd register mapping
						 * variable.
						 */
	u32 rsz_sdr_outoff;			/* sdr_outbuff register
						 * mapping variable.
						 */
	u32 rsz_coeff_horz[16];			/* horizontal coefficients
						 * mapping array.
						 */
	u32 rsz_coeff_vert[16];			/* vertical coefficients
						 * mapping array.
						 */
	u32 rsz_yehn;				/* yehn(luma)register mapping
						 * variable.
						 */
};

struct rsz_mult {
	int in_hsize;				/* input frame horizontal
						 * size.
						 */
	int in_vsize;				/* input frame vertical size.
						 */
	int out_hsize;				/* output frame horizontal
						 * size.
						 */
	int out_vsize;				/* output frame vertical
						 * size.
						 */
	int in_pitch;				/* offset between two rows of
						 * input frame.
						 */
	int out_pitch;				/* offset between two rows of
						 * output frame.
						 */
	int end_hsize;
	int end_vsize;
	int num_htap;				/* 0 = 7tap; 1 = 4tap */
	int num_vtap;				/* 0 = 7tap; 1 = 4tap */
	int active;
	int inptyp;
	int vrsz;
	int hrsz;
	int hstph;				/* for specifying horizontal
						 * starting phase.
						 */
	int vstph;
	int pix_fmt;				/* # defined, UYVY or YUYV. */
	int cbilin;				/* # defined, filter with luma
						 * or bi-linear.
						 */
	u16 tap4filt_coeffs[32];		/* horizontal filter
						 * coefficients.
						 */
	u16 tap7filt_coeffs[32];		/* vertical filter
						 * coefficients.
						 */
};
/* Channel specific structure contains information regarding
   the every channel */
struct channel_config {
	struct resizer_config register_config;	/* Instance of register set
						 * mapping structure
						 */
	int status;				/* Specifies whether the
						 * channel is busy or not
						 */
	struct mutex chanprotection_mutex;
	enum config_done config_state;
	u8 input_buf_index;
	u8 output_buf_index;
};
/* Global structure which contains information about number of channels
   and protection variables */
struct device_params {
	struct rsz_params *params;
	struct channel_config *config;		/* Pointer to channel */
	struct rsz_mult *multipass;		/* Multipass to support */
	unsigned char opened;			/* state of the device */
	struct completion compl_isr;		/* Completion for interrupt */
	struct mutex reszwrap_mutex;		/* Semaphore for array */
	struct videobuf_queue_ops vbq_ops;	/* videobuf queue operations */
	rsz_callback 	callback;		/* callback function which gets
						 * called when Resizer
						 * finishes resizing
						 */
	void *callback_arg;
	u32 in_buf_virt_addr[32];
	u32 *out_buf_phy_addr[32];
	u32 out_buf_virt_addr[32];
	u32 num_video_buffers;
	dma_addr_t tmp_buf;
	size_t tmp_buf_size;
	struct rsz_mult original_multipass;
	struct resizer_config original_rsz_conf_chan;
	struct device *isp;
};

/* per-filehandle data structure */
struct rsz_fh {
	struct rsz_params *params;
	struct channel_config *config;
	struct rsz_mult *multipass;		/* Multipass to support
						 * resizing ration outside
						 * of 0.25x to 4x
						 */
	spinlock_t vbq_lock;			/* spinlock for videobuf
						 * queues.
						 */
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
	struct device_params *device;
	dma_addr_t isp_addr_read;		/* Input/Output address */
	dma_addr_t isp_addr_write;		/* Input/Output address */
	u32 rsz_bufsize;			/* channel specific buffersize
						 */
};

static struct device_params *device_config;
static struct device *rsz_device;
static int rsz_major = -1;

/* functions declaration */
static void rsz_hardware_setup(struct device_params *device,
			       struct channel_config *rsz_conf_chan);
static int rsz_set_params(struct rsz_mult *multipass, struct rsz_params *,
						struct channel_config *);
static int rsz_get_params(struct rsz_params *, struct channel_config *);
static void rsz_copy_data(struct rsz_mult *multipass,
						struct rsz_params *params);
static void rsz_isr(unsigned long status, isp_vbq_callback_ptr arg1,
						void *arg2);
static void rsz_calculate_crop(struct channel_config *rsz_conf_chan,
					struct rsz_cropsize *cropsize);
static int rsz_set_multipass(struct device_params *device,
			     struct rsz_mult *multipass,
			     struct channel_config *rsz_conf_chan);
static int rsz_set_ratio(struct rsz_mult *multipass,
					struct channel_config *rsz_conf_chan);
static void rsz_config_ratio(struct rsz_mult *multipass,
					struct channel_config *rsz_conf_chan);

static void rsz_tmp_buf_free(void);
static u32 rsz_tmp_buf_alloc(size_t size);
static void rsz_save_multipass_context(void);
static void rsz_restore_multipass_context(void);

static void isp_enable_interrupts(struct device *dev, int is_raw)
{
	isp_reg_writel(dev, IRQ0ENABLE_RSZ_DONE_IRQ,
		       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE);
}

/**
 * rsz_hardware_setup - Sets hardware configuration registers
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Set hardware configuration registers
 **/
static void rsz_hardware_setup(struct device_params *device,
			       struct channel_config *rsz_conf_chan)
{
	int coeffcounter;
	int coeffoffset = 0;

	down(&resz_wrapper_mutex);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_cnt,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_in_start,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_IN_START);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_in_size,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_IN_SIZE);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_out_size,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_OUT_SIZE);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_sdr_inadd,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_INADD);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_sdr_inoff,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_INOFF);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_sdr_outadd,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTADD);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_sdr_outoff,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTOFF);
	isp_reg_writel(device->isp, rsz_conf_chan->register_config.rsz_yehn,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_YENH);

	for (coeffcounter = 0; coeffcounter < MAX_COEF_COUNTER;
							coeffcounter++) {
		isp_reg_writel(device->isp, rsz_conf_chan->register_config.
						rsz_coeff_horz[coeffcounter],
						OMAP3_ISP_IOMEM_RESZ,
						ISPRSZ_HFILT10 + coeffoffset);

		isp_reg_writel(device->isp, rsz_conf_chan->register_config.
						rsz_coeff_vert[coeffcounter],
						OMAP3_ISP_IOMEM_RESZ,
						ISPRSZ_VFILT10 + coeffoffset);
		coeffoffset = coeffoffset + COEFF_ADDRESS_OFFSET;
	}
	up(&resz_wrapper_mutex);
}

static void rsz_save_multipass_context()
{
	struct channel_config *rsz_conf_chan = device_config->config;
	struct rsz_mult *multipass = device_config->multipass;

	struct resizer_config  *original_rsz_conf_chan
		= &device_config->original_rsz_conf_chan;
	struct rsz_mult *original_multipass
		= &device_config->original_multipass;

	memset(original_rsz_conf_chan,
		0, sizeof(struct  resizer_config));
	memcpy(original_rsz_conf_chan,
		(struct resizer_config *) &rsz_conf_chan->register_config,
		sizeof(struct  resizer_config));
	memset(original_multipass,
		0, sizeof(struct rsz_mult));
	memcpy(original_multipass,
		multipass, sizeof(struct rsz_mult));

	return;
}

static void rsz_restore_multipass_context()
{
	struct channel_config *rsz_conf_chan = device_config->config;
	struct rsz_mult *multipass = device_config->multipass;

	struct resizer_config  *original_rsz_conf_chan
		= &device_config->original_rsz_conf_chan;
	struct rsz_mult *original_multipass
		= &device_config->original_multipass;

	memcpy((struct resizer_config *) &rsz_conf_chan->register_config,
		original_rsz_conf_chan, sizeof(struct  resizer_config));

	memcpy(multipass, original_multipass, sizeof(struct rsz_mult));

	return;
}

/**
 * rsz_start - Enables Resizer Wrapper
 * @device: Structure containing ISP resizer wrapper global information
 *
 * Submits a resizing task specified by the rsz_resize structure. The call can
 * either be blocked until the task is completed or returned immediately based
 * on the value of the blocking argument in the rsz_resize structure. If it is
 * blocking, the status of the task can be checked by calling ioctl
 * RSZ_G_STATUS. Only one task can be outstanding for each logical channel.
 *
 * Returns 0 if successful, or -EINVAL if could not set callback for RSZR IRQ
 * event or the state of the channel is not configured.
 **/
int rsz_start(struct rsz_fh *fh)
{
	struct channel_config *rsz_conf_chan = fh->config;
	struct device_params *device = fh->device;
	struct isp_device *isp = dev_get_drvdata(device->isp);
	struct rsz_mult *multipass = fh->multipass;
	struct videobuf_queue *q = &fh->vbq;
	int ret;

	if (rsz_conf_chan->config_state) {
		dev_err(rsz_device, "State not configured \n");
		goto err_einval;
	}

	rsz_conf_chan->status = CHANNEL_BUSY;

	rsz_hardware_setup(device, rsz_conf_chan);

	if (isp_set_callback(device->isp, CBK_RESZ_DONE, rsz_isr,
			     (void *)NULL, (void *)NULL)) {
		dev_err(rsz_device, "No callback for RSZR\n");
		goto err_einval;
	}

	isp_configure_interface(device->isp, &reszwrap_config);

	isp_start(device->isp);

	isp_enable_interrupts(device->isp, 0);

mult:
	device_config->compl_isr.done = 0;

	ispresizer_enable(&isp->isp_res, 1);

	ret = wait_for_completion_interruptible(&device_config->compl_isr);
	if (ret != 0) {
		dev_dbg(rsz_device, "Unexpected exit from "
				"wait_for_completion_interruptible\n");
		wait_for_completion(&device_config->compl_isr);
	}

	if (multipass->active) {
		rsz_set_multipass(device, multipass, rsz_conf_chan);
		goto mult;
	}

	if (fh->isp_addr_read) {
		ispmmu_vunmap(device->isp, fh->isp_addr_read);
		fh->isp_addr_read = 0;
	}
	if (fh->isp_addr_write) {
		ispmmu_vunmap(device->isp, fh->isp_addr_write);
		fh->isp_addr_write = 0;
	}

	rsz_conf_chan->status = CHANNEL_FREE;
	q->bufs[rsz_conf_chan->input_buf_index]->state = VIDEOBUF_NEEDS_INIT;
	q->bufs[rsz_conf_chan->output_buf_index]->state = VIDEOBUF_NEEDS_INIT;
	rsz_conf_chan->register_config.rsz_sdr_outadd = 0;
	rsz_conf_chan->register_config.rsz_sdr_inadd = 0;

	/* Unmap and free the DMA memory allocated for buffers */
	videobuf_dma_unmap(q, videobuf_to_dma(
				q->bufs[rsz_conf_chan->input_buf_index]));
	videobuf_dma_unmap(q, videobuf_to_dma(
				q->bufs[rsz_conf_chan->output_buf_index]));
	videobuf_dma_free(videobuf_to_dma(
				q->bufs[rsz_conf_chan->input_buf_index]));
	videobuf_dma_free(videobuf_to_dma(
				q->bufs[rsz_conf_chan->output_buf_index]));

	isp_unset_callback(device->isp, CBK_RESZ_DONE);

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * rsz_set_multipass - Set resizer multipass
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Returns always 0
 **/
static int rsz_set_multipass(struct device_params *device,
			     struct rsz_mult *multipass,
			     struct channel_config *rsz_conf_chan)
{
	multipass->in_hsize = multipass->out_hsize;
	multipass->in_vsize = multipass->out_vsize;
	multipass->out_hsize = multipass->end_hsize;
	multipass->out_vsize = multipass->end_vsize;

	multipass->out_pitch = (multipass->inptyp ? multipass->out_hsize
						: (multipass->out_hsize * 2));
	multipass->in_pitch = (multipass->inptyp ? multipass->in_hsize
						: (multipass->in_hsize * 2));

	rsz_conf_chan->register_config.rsz_sdr_inadd =
		rsz_conf_chan->register_config.rsz_sdr_outadd;
	rsz_set_ratio(multipass, rsz_conf_chan);
	rsz_config_ratio(multipass, rsz_conf_chan);
	rsz_hardware_setup(device, rsz_conf_chan);
	return 0;
}

/**
 * rsz_copy_data - Copy data
 * @params: Structure containing the Resizer Wrapper parameters
 *
 * Copy data
 **/
static void rsz_copy_data(struct rsz_mult *multipass, struct rsz_params *params)
{
	int i;
	multipass->in_hsize = params->in_hsize;
	multipass->in_vsize = params->in_vsize;
	multipass->out_hsize = params->out_hsize;
	multipass->out_vsize = params->out_vsize;
	multipass->end_hsize = params->out_hsize;
	multipass->end_vsize = params->out_vsize;
	multipass->in_pitch = params->in_pitch;
	multipass->out_pitch = params->out_pitch;
	multipass->hstph = params->hstph;
	multipass->vstph = params->vstph;
	multipass->inptyp = params->inptyp;
	multipass->pix_fmt = params->pix_fmt;
	multipass->cbilin = params->cbilin;

	for (i = 0; i < 32; i++) {
		multipass->tap4filt_coeffs[i] = params->tap4filt_coeffs[i];
		multipass->tap7filt_coeffs[i] = params->tap7filt_coeffs[i];
	}
}

/**
 * rsz_set_params - Set parameters for resizer wrapper
 * @params: Structure containing the Resizer Wrapper parameters
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Used to set the parameters of the Resizer hardware, including input and
 * output image size, horizontal and vertical poly-phase filter coefficients,
 * luma enchancement filter coefficients, etc.
 **/
static int rsz_set_params(struct rsz_mult *multipass, struct rsz_params *params,
					struct channel_config *rsz_conf_chan)
{
	int mul = 1;
	if ((params->yenh_params.type < 0) || (params->yenh_params.type > 2)) {
		dev_err(rsz_device, "rsz_set_params: Wrong yenh type\n");
		return -EINVAL;
	}
	if ((params->in_vsize <= 0) || (params->in_hsize <= 0) ||
			(params->out_vsize <= 0) || (params->out_hsize <= 0) ||
			(params->in_pitch <= 0) || (params->out_pitch <= 0)) {
		dev_err(rsz_device, "rsz_set_params: Invalid size params\n");
		return -EINVAL;
	}
	if ((params->inptyp != RSZ_INTYPE_YCBCR422_16BIT) &&
			(params->inptyp != RSZ_INTYPE_PLANAR_8BIT)) {
		dev_err(rsz_device, "rsz_set_params: Invalid input type\n");
		return -EINVAL;
	}
	if ((params->pix_fmt != RSZ_PIX_FMT_UYVY) &&
			(params->pix_fmt != RSZ_PIX_FMT_YUYV)) {
		dev_err(rsz_device, "rsz_set_params: Invalid pixel format\n");
		return -EINVAL;
	}
	if (params->inptyp == RSZ_INTYPE_YCBCR422_16BIT)
		mul = 2;
	else
		mul = 1;
	if (params->in_pitch < (params->in_hsize * mul)) {
		dev_err(rsz_device, "rsz_set_params: Pitch is incorrect\n");
		return -EINVAL;
	}
	if (params->out_pitch < (params->out_hsize * mul)) {
		dev_err(rsz_device, "rsz_set_params: Out pitch cannot be less"
					" than out hsize\n");
		return -EINVAL;
	}
	/* Output H size should be even */
	if ((params->out_hsize % PIXEL_EVEN) != 0) {
		dev_err(rsz_device, "rsz_set_params: Output H size should"
					" be even\n");
		return -EINVAL;
	}
	if (params->horz_starting_pixel < 0) {
		dev_err(rsz_device, "rsz_set_params: Horz start pixel cannot"
					" be less than zero\n");
		return -EINVAL;
	}

	rsz_copy_data(multipass, params);
	if (0 != rsz_set_ratio(multipass, rsz_conf_chan))
		goto err_einval;

	if (params->yenh_params.type) {
		if ((multipass->num_htap && multipass->out_hsize >
				1280) ||
				(!multipass->num_htap && multipass->out_hsize >
				640))
			goto err_einval;
	}

	if (INPUT_RAM)
		params->vert_starting_pixel = 0;

	rsz_conf_chan->register_config.rsz_in_start =
						(params->vert_starting_pixel
						<< ISPRSZ_IN_SIZE_VERT_SHIFT)
						& ISPRSZ_IN_SIZE_VERT_MASK;

	if (params->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_8BIT)
			goto err_einval;
	}
	if (params->inptyp == RSZ_INTYPE_YCBCR422_16BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_16BIT)
			goto err_einval;
	}

	rsz_conf_chan->register_config.rsz_in_start |=
						params->horz_starting_pixel
						& ISPRSZ_IN_START_HORZ_ST_MASK;

	rsz_conf_chan->register_config.rsz_yehn =
						(params->yenh_params.type
						<< ISPRSZ_YENH_ALGO_SHIFT)
						& ISPRSZ_YENH_ALGO_MASK;

	if (params->yenh_params.type) {
		rsz_conf_chan->register_config.rsz_yehn |=
						params->yenh_params.core
						& ISPRSZ_YENH_CORE_MASK;

		rsz_conf_chan->register_config.rsz_yehn |=
						(params->yenh_params.gain
						<< ISPRSZ_YENH_GAIN_SHIFT)
						& ISPRSZ_YENH_GAIN_MASK;

		rsz_conf_chan->register_config.rsz_yehn |=
						(params->yenh_params.slop
						<< ISPRSZ_YENH_SLOP_SHIFT)
						& ISPRSZ_YENH_SLOP_MASK;
	}

	rsz_config_ratio(multipass, rsz_conf_chan);

	rsz_conf_chan->config_state = STATE_CONFIGURED;

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * rsz_set_ratio - Set ratio
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Returns 0 if successful, -EINVAL if invalid output size, upscaling ratio is
 * being requested, or other ratio configuration value is out of bounds
 **/
static int rsz_set_ratio(struct rsz_mult *multipass,
				struct channel_config *rsz_conf_chan)
{
	struct isp_device *isp = dev_get_drvdata(device_config->isp);
	int alignment = 0;

	rsz_conf_chan->register_config.rsz_cnt = 0;

	if (multipass->cbilin) {
		rsz_conf_chan->register_config.rsz_cnt =
				BITSET(rsz_conf_chan->register_config.rsz_cnt,
				SET_BIT_CBLIN);
	}
	if (INPUT_RAM) {
		rsz_conf_chan->register_config.rsz_cnt =
				BITSET(rsz_conf_chan->register_config.rsz_cnt,
				SET_BIT_INPUTRAM);
	}
	if (multipass->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		rsz_conf_chan->register_config.rsz_cnt =
				BITSET(rsz_conf_chan->register_config.rsz_cnt,
				SET_BIT_INPTYP);
	} else {
		rsz_conf_chan->register_config.rsz_cnt =
				BITRESET(rsz_conf_chan->register_config.
				rsz_cnt, SET_BIT_INPTYP);

		if (multipass->pix_fmt == RSZ_PIX_FMT_UYVY) {
			rsz_conf_chan->register_config.rsz_cnt =
				BITRESET(rsz_conf_chan->register_config.
				rsz_cnt, SET_BIT_YCPOS);
		} else if (multipass->pix_fmt == RSZ_PIX_FMT_YUYV) {
			rsz_conf_chan->register_config.rsz_cnt =
					BITSET(rsz_conf_chan->register_config.
					rsz_cnt, SET_BIT_YCPOS);
		}

	}

	multipass->vrsz = (multipass->in_vsize - NUM_D2TAPS) * RATIO_MULTIPLIER
						/ (multipass->out_vsize - 1);
	multipass->hrsz = ((multipass->in_hsize - NUM_D2TAPS)
						* RATIO_MULTIPLIER) /
						(multipass->out_hsize - 1);

	if (UP_RSZ_RATIO > multipass->vrsz || UP_RSZ_RATIO > multipass->hrsz) {
		dev_err(rsz_device, "Upscaling ratio not supported!");
		goto err_einval;
	}

	if (multipass->hrsz <= 512) {
		multipass->hrsz = (multipass->in_hsize - NUM_TAPS)
						* RATIO_MULTIPLIER
						/ (multipass->out_hsize - 1);
		if (multipass->hrsz < 64)
			multipass->hrsz = 64;
		if (multipass->hrsz > 512)
			multipass->hrsz = 512;
		if (multipass->hstph > NUM_PHASES)
			goto err_einval;
		multipass->num_htap = 1;
	} else if (multipass->hrsz >= 513 && multipass->hrsz <= 1024) {
		if (multipass->hstph > NUM_D2PH)
			goto err_einval;
		multipass->num_htap = 0;
	}

	if (multipass->vrsz <= 512) {
		multipass->vrsz = (multipass->in_vsize - NUM_TAPS)
						* RATIO_MULTIPLIER
						/ (multipass->out_vsize - 1);
		if (multipass->vrsz < 64)
			multipass->vrsz = 64;
		if (multipass->vrsz > 512)
			multipass->vrsz = 512;
		if (multipass->vstph > NUM_PHASES)
			goto err_einval;
		multipass->num_vtap = 1;
	} else if (multipass->vrsz >= 513 && multipass->vrsz <= 1024) {
		if (multipass->vstph > NUM_D2PH)
			goto err_einval;
		multipass->num_vtap = 0;
	}

	if ((multipass->in_pitch) % ALIGN32) {
		dev_err(rsz_device, "Invalid input pitch: %d \n",
							multipass->in_pitch);
		goto err_einval;
	}
	if ((multipass->out_pitch) % ALIGN32) {
		dev_err(rsz_device, "Invalid output pitch %d \n",
							multipass->out_pitch);
		goto err_einval;
	}

	if (multipass->vrsz < 256 &&
			(multipass->in_vsize < multipass->out_vsize)) {
		if (multipass->inptyp == RSZ_INTYPE_PLANAR_8BIT)
			alignment = ALIGNMENT;
		else if (multipass->inptyp == RSZ_INTYPE_YCBCR422_16BIT)
			alignment = (ALIGNMENT / 2);
		else
			dev_err(rsz_device, "Invalid input type\n");

		if (!(((multipass->out_hsize % PIXEL_EVEN) == 0)
				&& (multipass->out_hsize % alignment) == 0)) {
			dev_err(rsz_device, "wrong hsize\n");
			goto err_einval;
		}
	}
	if (multipass->hrsz >= 64 && multipass->hrsz <= 1024) {
		multipass->active = 0;

	} else if (multipass->hrsz > 1024) {
		if (multipass->hstph > NUM_D2PH)
			goto err_einval;
		multipass->num_htap = 0;
		multipass->out_hsize = multipass->in_hsize * 256 / 1024;
		if (multipass->out_hsize % ALIGN32) {
			multipass->out_hsize +=
				abs((multipass->out_hsize % ALIGN32) - ALIGN32);
		}
		multipass->out_pitch = ((multipass->inptyp) ?
						multipass->out_hsize :
						(multipass->out_hsize * 2));
		multipass->hrsz = ((multipass->in_hsize - NUM_D2TAPS)
						* RATIO_MULTIPLIER)
						/ (multipass->out_hsize - 1);
		multipass->active = 1;

	}

	/* 4Tap: Check max Output width */
	if (multipass->vrsz <= 512) {
		if (isp->revision <= ISP_REVISION_2_0 &&
		    multipass->out_vsize > MAX_4TAP_OUTWIDTH_3430) {
			dev_err(rsz_device, "wrong width\n");
			goto err_einval;
		}

		if (isp->revision > ISP_REVISION_2_0 &&
		    multipass->out_vsize > MAX_4TAP_OUTWIDTH_3630) {
			dev_err(rsz_device, "wrong width\n");
			goto err_einval;
		}
	}

	/* 7Tap: Check max Output width */
	if (multipass->vrsz > 512) {
		if (isp->revision <= ISP_REVISION_2_0 &&
		    multipass->out_vsize > MAX_7TAP_OUTWIDTH_3430) {
			dev_err(rsz_device, "wrong width\n");
			goto err_einval;
		}

		if (isp->revision > ISP_REVISION_2_0 &&
		    multipass->out_vsize > MAX_7TAP_OUTWIDTH_3630) {
			dev_err(rsz_device, "wrong width\n");
			goto err_einval;
		}
	}

	if (multipass->vrsz > 1024) {
		multipass->out_vsize = multipass->in_vsize * 256 / 1024;
		multipass->vrsz = ((multipass->in_vsize - NUM_D2TAPS)
						* RATIO_MULTIPLIER)
						/ (multipass->out_vsize - 1);
		multipass->active = 1;
		multipass->num_vtap = 0;

	}
	rsz_conf_chan->register_config.rsz_out_size =
						multipass->out_hsize
						& ISPRSZ_OUT_SIZE_HORZ_MASK;

	rsz_conf_chan->register_config.rsz_out_size |=
						(multipass->out_vsize
						<< ISPRSZ_OUT_SIZE_VERT_SHIFT)
						& ISPRSZ_OUT_SIZE_VERT_MASK;

	rsz_conf_chan->register_config.rsz_sdr_inoff =
						multipass->in_pitch
						& ISPRSZ_SDR_INOFF_OFFSET_MASK;

	rsz_conf_chan->register_config.rsz_sdr_outoff =
					multipass->out_pitch
					& ISPRSZ_SDR_OUTOFF_OFFSET_MASK;

	if (multipass->hrsz >= 64 && multipass->hrsz <= 512) {
		if (multipass->hstph > NUM_PHASES)
			goto err_einval;
	} else if (multipass->hrsz >= 64 && multipass->hrsz <= 512) {
		if (multipass->hstph > NUM_D2PH)
			goto err_einval;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
						(multipass->hstph
						<< ISPRSZ_CNT_HSTPH_SHIFT)
						& ISPRSZ_CNT_HSTPH_MASK;

	if (multipass->vrsz >= 64 && multipass->hrsz <= 512) {
		if (multipass->vstph > NUM_PHASES)
			goto err_einval;
	} else if (multipass->vrsz >= 64 && multipass->vrsz <= 512) {
		if (multipass->vstph > NUM_D2PH)
			goto err_einval;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
						(multipass->vstph
						<< ISPRSZ_CNT_VSTPH_SHIFT)
						& ISPRSZ_CNT_VSTPH_MASK;

	rsz_conf_chan->register_config.rsz_cnt |=
						(multipass->hrsz - 1)
						& ISPRSZ_CNT_HRSZ_MASK;

	rsz_conf_chan->register_config.rsz_cnt |=
						((multipass->vrsz - 1)
						<< ISPRSZ_CNT_VRSZ_SHIFT)
						& ISPRSZ_CNT_VRSZ_MASK;

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * rsz_config_ratio - Configure ratio
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Configure ratio
 **/
static void rsz_config_ratio(struct rsz_mult *multipass,
				struct channel_config *rsz_conf_chan)
{
	int hsize;
	int vsize;
	int coeffcounter;

	if (multipass->hrsz <= 512) {
		hsize = ((32 * multipass->hstph + (multipass->out_hsize - 1)
					* multipass->hrsz + 16) >> 8) + 7;
	} else {
		hsize = ((64 * multipass->hstph + (multipass->out_hsize - 1)
					* multipass->hrsz + 32) >> 8) + 7;
	}
	if (multipass->vrsz <= 512) {
		vsize = ((32 * multipass->vstph + (multipass->out_vsize - 1)
					* multipass->vrsz + 16) >> 8) + 4;
	} else {
		vsize = ((64 * multipass->vstph + (multipass->out_vsize - 1)
					* multipass->vrsz + 32) >> 8) + 7;
	}
	rsz_conf_chan->register_config.rsz_in_size = hsize;

	rsz_conf_chan->register_config.rsz_in_size |=
					((vsize << ISPRSZ_IN_SIZE_VERT_SHIFT)
					& ISPRSZ_IN_SIZE_VERT_MASK);

	for (coeffcounter = 0; coeffcounter < MAX_COEF_COUNTER;
							coeffcounter++) {
		if (multipass->num_htap) {
			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] =
					(multipass->tap4filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_HFILT10_COEF0_MASK);
			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] |=
					((multipass->tap4filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_HFILT10_COEF1_SHIFT)
					& ISPRSZ_HFILT10_COEF1_MASK);
		} else {
			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] =
					(multipass->tap7filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_HFILT10_COEF0_MASK);

			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] |=
					((multipass->tap7filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_HFILT10_COEF1_SHIFT)
					& ISPRSZ_HFILT10_COEF1_MASK);
		}

		if (multipass->num_vtap) {
			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] =
					(multipass->tap4filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_VFILT10_COEF0_MASK);

			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] |=
					((multipass->tap4filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_VFILT10_COEF1_SHIFT) &
					ISPRSZ_VFILT10_COEF1_MASK);
		} else {
			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] =
					(multipass->tap7filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_VFILT10_COEF0_MASK);
			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] |=
					((multipass->tap7filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_VFILT10_COEF1_SHIFT)
					& ISPRSZ_VFILT10_COEF1_MASK);
		}
	}
}

/**
 * rsz_get_params - Gets the parameter values
 * @params: Structure containing the Resizer Wrapper parameters
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Used to get the Resizer hardware settings associated with the
 * current logical channel represented by fd.
 **/
static int rsz_get_params(struct rsz_params *params,
					struct channel_config *rsz_conf_chan)
{
	int coeffcounter;

	if (rsz_conf_chan->config_state) {
		dev_err(rsz_device, "state not configured\n");
		return -EINVAL;
	}

	params->in_hsize = rsz_conf_chan->register_config.rsz_in_size
					& ISPRSZ_IN_SIZE_HORZ_MASK;
	params->in_vsize = (rsz_conf_chan->register_config.rsz_in_size
					& ISPRSZ_IN_SIZE_VERT_MASK)
					>> ISPRSZ_IN_SIZE_VERT_SHIFT;

	params->in_pitch = rsz_conf_chan->register_config.rsz_sdr_inoff
					& ISPRSZ_SDR_INOFF_OFFSET_MASK;

	params->out_hsize = rsz_conf_chan->register_config.rsz_out_size
					& ISPRSZ_OUT_SIZE_HORZ_MASK;

	params->out_vsize = (rsz_conf_chan->register_config.rsz_out_size
					& ISPRSZ_OUT_SIZE_VERT_MASK)
					>> ISPRSZ_OUT_SIZE_VERT_SHIFT;

	params->out_pitch = rsz_conf_chan->register_config.rsz_sdr_outoff
					& ISPRSZ_SDR_OUTOFF_OFFSET_MASK;

	params->cbilin = (rsz_conf_chan->register_config.rsz_cnt
					& SET_BIT_CBLIN) >> SET_BIT_CBLIN;

	params->inptyp = (rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_INPTYP_MASK)
					>> SET_BIT_INPTYP;
	params->horz_starting_pixel = ((rsz_conf_chan->register_config.
					rsz_in_start
					& ISPRSZ_IN_START_HORZ_ST_MASK));
	params->vert_starting_pixel = ((rsz_conf_chan->register_config.
					rsz_in_start
					& ISPRSZ_IN_START_VERT_ST_MASK)
					>> ISPRSZ_IN_START_VERT_ST_SHIFT);

	params->hstph = ((rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_HSTPH_MASK
					>> ISPRSZ_CNT_HSTPH_SHIFT));
	params->vstph = ((rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_VSTPH_MASK
					>> ISPRSZ_CNT_VSTPH_SHIFT));

	for (coeffcounter = 0; coeffcounter < MAX_COEF_COUNTER;
							coeffcounter++) {
		params->tap4filt_coeffs[2 * coeffcounter] =
					rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter]
					& ISPRSZ_HFILT10_COEF0_MASK;

		params->tap4filt_coeffs[2 * coeffcounter + 1] =
					(rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter]
					& ISPRSZ_HFILT10_COEF1_MASK)
					>> ISPRSZ_HFILT10_COEF1_SHIFT;

		params->tap7filt_coeffs[2 * coeffcounter] =
					rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter]
					& ISPRSZ_VFILT10_COEF0_MASK;

		params->tap7filt_coeffs[2 * coeffcounter + 1] =
					(rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter]
					& ISPRSZ_VFILT10_COEF1_MASK)
					>> ISPRSZ_VFILT10_COEF1_SHIFT;

	}

	params->yenh_params.type = (rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_ALGO_MASK)
					>> ISPRSZ_YENH_ALGO_SHIFT;

	params->yenh_params.core = rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_CORE_MASK;

	params->yenh_params.gain = (rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_GAIN_MASK)
					>> ISPRSZ_YENH_GAIN_SHIFT;

	params->yenh_params.slop = (rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_SLOP_MASK)
					>> ISPRSZ_YENH_SLOP_SHIFT;

	params->pix_fmt = ((rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_PIXFMT_MASK)
					>> SET_BIT_YCPOS);

	if (params->pix_fmt)
		params->pix_fmt = RSZ_PIX_FMT_UYVY;
	else
		params->pix_fmt = RSZ_PIX_FMT_YUYV;

	return 0;
}

/**
 * rsz_calculate_crop - Calculate Crop values
 * @rsz_conf_chan: Structure containing channel configuration
 * @cropsize: Structure containing crop parameters
 *
 * Calculate Crop values
 **/
static void rsz_calculate_crop(struct channel_config *rsz_conf_chan,
						struct rsz_cropsize *cropsize)
{
	int luma_enable;

	cropsize->hcrop = 0;
	cropsize->vcrop = 0;

	luma_enable = (rsz_conf_chan->register_config.rsz_yehn
						& ISPRSZ_YENH_ALGO_MASK)
						>> ISPRSZ_YENH_ALGO_SHIFT;

	if (luma_enable)
		cropsize->hcrop += 2;
}

/**
 * rsz_vbq_release - Videobuffer queue release
 * @q: Structure containing the videobuffer queue file handle, and device
 *     structure which contains the actual configuration.
 * @vb: Structure containing the videobuffer used for resizer processing.
 **/
static void rsz_vbq_release(struct videobuf_queue *q,
						struct videobuf_buffer *vb)
{
	int i;
	struct rsz_fh *fh = q->priv_data;
	struct device_params *device = fh->device;

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		struct videobuf_dmabuf *dma = NULL;
		if (!q->bufs[i])
			continue;
		if (q->bufs[i]->memory != V4L2_MEMORY_MMAP)
			continue;
		dma = videobuf_to_dma(q->bufs[i]);
		videobuf_dma_unmap(q, dma);
		videobuf_dma_free(dma);
	}

	ispmmu_vunmap(device->isp, fh->isp_addr_read);
	ispmmu_vunmap(device->isp, fh->isp_addr_write);
	fh->isp_addr_read = 0;
	fh->isp_addr_write = 0;
	spin_lock(&fh->vbq_lock);
	vb->state = VIDEOBUF_NEEDS_INIT;
	spin_unlock(&fh->vbq_lock);

}

/**
 * rsz_vbq_setup - Sets up the videobuffer size and validates count.
 * @q: Structure containing the videobuffer queue file handle, and device
 *     structure which contains the actual configuration.
 * @cnt: Number of buffers requested
 * @size: Size in bytes of the buffer used for previewing
 *
 * Always returns 0.
 **/
static int rsz_vbq_setup(struct videobuf_queue *q, unsigned int *cnt,
							unsigned int *size)
{
	struct rsz_fh *fh = q->priv_data;
	struct rsz_mult *multipass = fh->multipass;
	u32 insize, outsize;

	spin_lock(&fh->vbq_lock);
	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	outsize = multipass->out_pitch * multipass->out_vsize;
	insize = multipass->in_pitch * multipass->in_vsize;
	if (*cnt == 1 && (outsize > insize)) {
		dev_err(rsz_device, "2 buffers are required for Upscaling "
								"mode\n");
		goto err_einval;
	}
	if (!fh->params->in_hsize || !fh->params->in_vsize) {
		dev_err(rsz_device, "Can't setup buffer size\n");
		goto err_einval;
	} else {
		if (outsize > insize)
			*size = outsize;
		else
			*size = insize;

		fh->rsz_bufsize = *size;
	}
	spin_unlock(&fh->vbq_lock);

	return 0;
err_einval:
	spin_unlock(&fh->vbq_lock);
	return -EINVAL;
}

/**
 * rsz_vbq_prepare - Videobuffer is prepared and mmapped.
 * @q: Structure containing the videobuffer queue file handle, and device
 *     structure which contains the actual configuration.
 * @vb: Structure containing the videobuffer used for resizer processing.
 * @field: Type of field to set in videobuffer device.
 *
 * Returns 0 if successful, or -EINVAL if buffer couldn't get allocated, or
 * -EIO if the ISP MMU mapping fails
 **/
static int rsz_vbq_prepare(struct videobuf_queue *q,
						struct videobuf_buffer *vb,
						enum v4l2_field field)
{
	struct rsz_fh *fh = q->priv_data;
	struct device_params *device = fh->device;
	struct channel_config *rsz_conf_chan = fh->config;
	struct rsz_mult *multipass = fh->multipass;
	int err = 0;
	unsigned int isp_addr, insize, outsize;
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);

	spin_lock(&fh->vbq_lock);
	if (vb->baddr) {
		vb->size = fh->rsz_bufsize;
		vb->bsize = fh->rsz_bufsize;
	} else {
		spin_unlock(&fh->vbq_lock);
		dev_err(rsz_device, "No user buffer allocated\n");
		goto out;
	}
	if (vb->i) {
		vb->width = fh->params->out_hsize;
		vb->height = fh->params->out_vsize;
	} else {
		vb->width = fh->params->in_hsize;
		vb->height = fh->params->in_vsize;
	}

	vb->field = field;
	spin_unlock(&fh->vbq_lock);

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		err = videobuf_iolock(q, vb, NULL);
		if (!err) {
			isp_addr = ispmmu_vmap(device->isp, dma->sglist,
					       dma->sglen);
			if (!isp_addr)
				err = -EIO;
			else {
				if (vb->i) {
					rsz_conf_chan->register_config.
							rsz_sdr_outadd
							= isp_addr;
					fh->isp_addr_write = isp_addr;
					rsz_conf_chan->output_buf_index = vb->i;
				} else {
					rsz_conf_chan->register_config.
							rsz_sdr_inadd
							= isp_addr;
					rsz_conf_chan->input_buf_index = vb->i;
					outsize = multipass->out_pitch *
							multipass->out_vsize;
					insize = multipass->in_pitch *
							multipass->in_vsize;
					if (outsize < insize) {
						rsz_conf_chan->register_config.
								rsz_sdr_outadd
								= isp_addr;
						rsz_conf_chan->
							output_buf_index =
							vb->i;
					}

					fh->isp_addr_read = isp_addr;
				}
			}
		}

	}

	if (!err) {
		spin_lock(&fh->vbq_lock);
		vb->state = VIDEOBUF_PREPARED;
		spin_unlock(&fh->vbq_lock);
		flush_cache_user_range(NULL, vb->baddr, (vb->baddr
								+ vb->bsize));
	} else
		rsz_vbq_release(q, vb);

out:
	return err;
}

static void rsz_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	return;
}

/**
 * rsz_open - Initializes and opens the Resizer Wrapper
 * @inode: Inode structure associated with the Resizer Wrapper
 * @filp: File structure associated with the Resizer Wrapper
 *
 * Returns 0 if successful, -EBUSY if its already opened or the ISP module is
 * not available, or -ENOMEM if its unable to allocate the device in kernel
 * space memory.
 **/
static int rsz_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct channel_config *rsz_conf_chan;
	struct rsz_fh *fh;
	struct device_params *device = device_config;
	struct rsz_params *params;
	struct rsz_mult *multipass;
	struct device *isp;

	if ((filp->f_flags & O_NONBLOCK) == O_NONBLOCK) {
		printk(KERN_DEBUG "omap-resizer: Device is opened in "
					"non blocking mode\n");
	} else {
		printk(KERN_DEBUG "omap-resizer: Device is opened in blocking "
					"mode\n");
	}

	fh = kzalloc(sizeof(struct rsz_fh), GFP_KERNEL);
	if (NULL == fh)
		return -ENOMEM;

	isp = isp_get();
	if (!isp) {
		printk(KERN_ERR "Can't enable ISP clocks (ret %d)\n", ret);
		ret = -EACCES;
		goto err_resz;
	}
	device->isp = isp;

	rsz_conf_chan = kzalloc(sizeof(struct channel_config), GFP_KERNEL);
	if (rsz_conf_chan == NULL) {
		dev_err(rsz_device, "\n cannot allocate memory to config");
		ret = -ENOMEM;
		goto err_enomem0;
	}
	params = kzalloc(sizeof(struct rsz_params), GFP_KERNEL);
	if (params == NULL) {
		dev_err(rsz_device, "\n cannot allocate memory to params");
		ret = -ENOMEM;
		goto err_enomem1;
	}
	multipass = kzalloc(sizeof(struct rsz_mult), GFP_KERNEL);
	if (multipass == NULL) {
		dev_err(rsz_device, "\n cannot allocate memory to multipass");
		ret = -ENOMEM;
		goto err_enomem2;
	}

	fh->multipass = multipass;
	fh->params = params;
	fh->config = rsz_conf_chan;

	if (mutex_lock_interruptible(&device->reszwrap_mutex)) {
		ret = -EINTR;
		goto err_enomem2;
	}
	device->opened++;
	mutex_unlock(&device->reszwrap_mutex);

	rsz_conf_chan->config_state = STATE_NOT_CONFIGURED;
	rsz_conf_chan->status = CHANNEL_FREE;

	filp->private_data = fh;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->device = device;

	videobuf_queue_sg_init(&fh->vbq, &device->vbq_ops, NULL,
					&fh->vbq_lock, fh->type,
					V4L2_FIELD_NONE,
					sizeof(struct videobuf_buffer), fh);

	spin_lock_init(&fh->vbq_lock);
	mutex_init(&rsz_conf_chan->chanprotection_mutex);

	return 0;
err_enomem2:
	kfree(params);
err_enomem1:
	kfree(rsz_conf_chan);
err_enomem0:
	isp_put();
err_resz:
	kfree(fh);
	return ret;
}

/**
 * rsz_release - Releases Resizer Wrapper and frees up allocated memory
 * @inode: Inode structure associated with the Resizer Wrapper
 * @filp: File structure associated with the Resizer Wrapper
 *
 * Returns 0 if successful, or -EBUSY if channel is being used.
 **/
static int rsz_release(struct inode *inode, struct file *filp)
{
	u32 timeout = 0;
	struct rsz_fh *fh = filp->private_data;
	struct channel_config *rsz_conf_chan = fh->config;
	struct rsz_params *params = fh->params;
	struct rsz_mult *multipass = fh->multipass;
	struct videobuf_queue *q = &fh->vbq;

	while ((rsz_conf_chan->status != CHANNEL_FREE) && (timeout < 20)) {
		timeout++;
		schedule();
	}
	if (mutex_lock_interruptible(&device_config->reszwrap_mutex))
		return -EINTR;
	device_config->opened--;
	mutex_unlock(&device_config->reszwrap_mutex);
	/* This will Free memory allocated to the buffers,
	 * and flushes the queue
	 */
	videobuf_queue_cancel(q);
	fh->params = NULL;
	fh->config = NULL;

	fh->rsz_bufsize = 0;
	filp->private_data = NULL;

	kfree(rsz_conf_chan);
	kfree(params);
	kfree(multipass);
	kfree(fh);

	isp_put();

	return 0;
}

/**
 * rsz_mmap - Memory maps the Resizer Wrapper module.
 * @file: File structure associated with the Resizer Wrapper
 * @vma: Virtual memory area structure.
 *
 * Returns 0 if successful, or returned value by the videobuf_mmap_mapper()
 * function.
 **/
static int rsz_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct rsz_fh *fh = file->private_data;

	return videobuf_mmap_mapper(&fh->vbq, vma);
}

/**
 * rsz_ioctl - I/O control function for Resizer Wrapper
 * @inode: Inode structure associated with the Resizer Wrapper.
 * @file: File structure associated with the Resizer Wrapper.
 * @cmd: Type of command to execute.
 * @arg: Argument to send to requested command.
 *
 * Returns 0 if successful, -EBUSY if channel is being used, -1 if bad command
 * passed or access is denied, -EFAULT if copy_from_user() or copy_to_user()
 * fails, -EINVAL if parameter validation fails or parameter structure is not
 * present.
 **/
static long rsz_unlocked_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int ret = 0;
	struct rsz_fh *fh = file->private_data;
	struct device_params *device = fh->device;
	struct isp_device *isp = dev_get_drvdata(device->isp);
	struct channel_config *rsz_conf_chan = fh->config;

	if ((_IOC_TYPE(cmd) != RSZ_IOC_BASE)
					|| (_IOC_NR(cmd) > RSZ_IOC_MAXNR)) {
		dev_err(rsz_device, "Bad command value \n");
		goto err_minusone;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));

	if (ret) {
		dev_err(rsz_device, "Access denied\n");
		goto err_minusone;
	}

	switch (cmd) {
	case RSZ_REQBUF:
	{
		struct v4l2_requestbuffers req_buf;
		if (copy_from_user(&req_buf, (struct v4l2_requestbuffers *)arg,
					sizeof(struct v4l2_requestbuffers))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = videobuf_reqbufs(&fh->vbq, (void *)&req_buf);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		break;
	}
	case RSZ_QUERYBUF:
	{
		struct v4l2_buffer buf;
		if (copy_from_user(&buf, (struct v4l2_buffer *)arg,
						sizeof(struct v4l2_buffer))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = videobuf_querybuf(&fh->vbq, (void *)&buf);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		if (copy_to_user((struct v4l2_buffer *)arg, &buf,
						sizeof(struct v4l2_buffer)))
			ret = -EFAULT;
		break;
	}
	case RSZ_QUEUEBUF:
	{
		struct v4l2_buffer buf;
		if (copy_from_user(&buf, (struct v4l2_buffer *)arg,
						sizeof(struct v4l2_buffer))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = videobuf_qbuf(&fh->vbq, (void *)&buf);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		break;
	}
	case RSZ_S_PARAM:
	{
		struct rsz_params *params = fh->params;
		if (copy_from_user(params, (struct rsz_params *)arg,
						sizeof(struct rsz_params))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = rsz_set_params(fh->multipass, params, rsz_conf_chan);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		break;
	}

	case RSZ_G_PARAM:
	{
		struct rsz_params params;

		ret = rsz_get_params(&params, rsz_conf_chan);

		if (copy_to_user((struct rsz_params *)arg, &params,
					sizeof(struct rsz_params)))
			ret = -EFAULT;
		break;
	}

	case RSZ_G_STATUS:
	{
		struct rsz_status status;

		status.chan_busy = rsz_conf_chan->status;
		status.hw_busy = ispresizer_busy(&isp->isp_res);
		status.src = INPUT_RAM;

		if (copy_to_user((struct rsz_status *)arg, &status,
						sizeof(struct rsz_status)))
			ret = -EFAULT;
		break;
	}

	case RSZ_RESIZE:
		if (file->f_flags & O_NONBLOCK) {
			if (ispresizer_busy(&isp->isp_res))
				return -EBUSY;
			else {
				if (!mutex_trylock(&device->reszwrap_mutex))
					return -EBUSY;
			}
		} else {
			if (mutex_lock_interruptible(&device->reszwrap_mutex))
				goto err_eintr;
		}
		ret = rsz_start(fh);
		mutex_unlock(&device->reszwrap_mutex);
		break;

	case RSZ_GET_CROPSIZE:
	{
		struct rsz_cropsize sz;

		rsz_calculate_crop(rsz_conf_chan, &sz);

		if (copy_to_user((struct rsz_cropsize *)arg, &sz,
						sizeof(struct rsz_cropsize)))
			ret = -EFAULT;
		break;
	}

	default:
		dev_err(rsz_device, "resizer_ioctl: Invalid Command Value");
		ret = -EINVAL;
	}

out:
	return (long)ret;
err_minusone:
	ret = -1;
	goto out;
err_eintr:
	ret = -EINTR;
	goto out;
err_efault:
	ret = -EFAULT;
	goto out;
}

static const struct file_operations rsz_fops = {
	.owner = THIS_MODULE,
	.open = rsz_open,
	.release = rsz_release,
	.mmap = rsz_mmap,
	.unlocked_ioctl = rsz_unlocked_ioctl,
};

/**
 * rsz_get_resource - get the Resizer module from the kernel space.
 * This function will make sure that Resizer module is not used by any other
 * application. It is equivalent of open() call from user space.
 * It returns busy if the device is already opened by other application
 * or ENOMEM if it fails to allocate memory for structures
 * or 0 if the call is successful.
 **/

int rsz_get_resource(void)
{
	int ret = 0;
	struct channel_config *rsz_conf_chan;
	struct device_params *device = device_config;
	struct rsz_params *params;
	struct rsz_mult *multipass;
	struct device *isp;

	if (device->opened) {
		dev_err(rsz_device, "rsz_get_resource: device is "
						"already opened\n");
		return -EBUSY;
	}

	isp = isp_get();
	if (!isp) {
		printk(KERN_ERR "Can't enable ISP clocks (ret %d)\n", ret);
		return -EACCES;
	}
	device->isp = isp;

	rsz_conf_chan = kzalloc(sizeof(struct channel_config), GFP_KERNEL);
	if (rsz_conf_chan == NULL) {
		dev_err(rsz_device, "\n Can not allocate memory to config");
		ret = -ENOMEM;
		goto err_enomem0;
	}
	params = kzalloc(sizeof(struct rsz_params), GFP_KERNEL);
	if (params == NULL) {
		dev_warn(rsz_device, "\n Can not allocate memory to params");
		ret = -ENOMEM;
		goto err_enomem1;
	}
	multipass = kzalloc(sizeof(struct rsz_mult), GFP_KERNEL);
	if (multipass == NULL) {
		dev_err(rsz_device, "\n cannot allocate memory to multipass");
		ret = -ENOMEM;
		goto err_enomem2;
	}

	device->params = params;
	device->config = rsz_conf_chan;
	device->multipass = multipass;
	device->opened = 1;

	rsz_conf_chan->config_state = STATE_NOT_CONFIGURED;

	init_completion(&device->compl_isr);
	mutex_init(&device->reszwrap_mutex);

	return 0;
err_enomem2:
	kfree(params);
err_enomem1:
	kfree(rsz_conf_chan);
err_enomem0:
	isp_put();

	return ret;
}
EXPORT_SYMBOL(rsz_get_resource);

/**
 * rsz_put_resource - release all the resource which were acquired during
 * rsz_get_resource() call.
 **/
void rsz_put_resource(void)
{
	struct device_params *device = device_config;
	struct channel_config *rsz_conf_chan = device->config;
	struct rsz_params *params = device->params;
	struct rsz_mult *multipass = device->multipass;
	struct isp_device *isp = dev_get_drvdata(device->isp);
	int i = 0;

	if (device->opened != 1)
		return;

	/* unmap output buffers if allocated */
	for (i = 0; i < device->num_video_buffers; ++i) {
		if (device->out_buf_virt_addr[i]) {
			iommu_kunmap(isp->iommu, device->out_buf_virt_addr[i]);
			device_config->out_buf_virt_addr[i] = 0;
		}
		if (device->in_buf_virt_addr[i]) {
			iommu_kunmap(isp->iommu, device->in_buf_virt_addr[i]);
			device_config->in_buf_virt_addr[i] = 0;
		}
	}

	if (multipass_active) {
		rsz_tmp_buf_free();
		multipass_active = 0;
	}
	/* free all memory which was allocate during get() */
	kfree(rsz_conf_chan);
	kfree(params);
	kfree(multipass);

	/* make device available */
	device->opened = 0;
	device->params = NULL;
	device->config = NULL;
	device->multipass = NULL;

	/* release isp resource*/
	isp_put();

	return ;
}
EXPORT_SYMBOL(rsz_put_resource);

/**
  * rsz_configure - configure the Resizer parameters
  * @params: Structure containing the Resizer Wrapper parameters
  * @callback: callback function to call after resizing is over
  * @arg1: argument to callback function
  *
  * This function can be called from DSS to set the parameter of resizer
  * in case there is need to downsize the input image through resizer
  * before calling this function calling driver must already have called
  * rsz_get_resource() function so that necessory initialization is over.
  * Returns 0 if successful,
  **/
int rsz_configure(struct rsz_params *params, rsz_callback callback,
		  u32 num_video_buffers, void *arg1)
{
	int retval;
	struct channel_config *rsz_conf_chan = device_config->config;
	struct rsz_mult *multipass = device_config->multipass;
	size_t tmp_size;

	multipass_active = 0;
	retval = rsz_set_params(multipass, params, rsz_conf_chan);
	if (retval != 0)
		return retval;

	if (device_config->multipass->active) {
		multipass_active = 1;
		tmp_size = PAGE_ALIGN(multipass->out_hsize
				     * multipass->out_vsize
				     * (multipass->inptyp ? 1 : 2));
		rsz_tmp_buf_alloc(tmp_size);
		rsz_conf_chan->register_config.rsz_sdr_outadd =
			(u32)device_config->tmp_buf;
		rsz_save_multipass_context();
	}

	rsz_hardware_setup(device_config, rsz_conf_chan);
	device_config->callback = callback;
	device_config->callback_arg = arg1;
	device_config->num_video_buffers = num_video_buffers;

	return 0;
}
EXPORT_SYMBOL(rsz_configure);

static void rsz_tmp_buf_free(void)
{
	struct isp_device *isp = dev_get_drvdata(device_config->isp);

	if (device_config->tmp_buf) {
		iommu_vfree(isp->iommu, device_config->tmp_buf);
		device_config->tmp_buf = 0;
		device_config->tmp_buf_size = 0;
	}
}
static u32 rsz_tmp_buf_alloc(size_t size)
{
	struct isp_device *isp = dev_get_drvdata(device_config->isp);

	rsz_tmp_buf_free();
	printk(KERN_INFO "%s: allocating %d bytes\n", __func__, size);

	device_config->tmp_buf =
		iommu_vmalloc(isp->iommu, 0, size, IOMMU_FLAG);
	if (IS_ERR((void *)device_config->tmp_buf)) {
		printk(KERN_ERR "ispmmu_vmap mapping failed ");
		return -ENOMEM;
	}
	device_config->tmp_buf_size = size;

	return 0;
}

/** rsz_begin - Function to be called by DSS when resizing of the input image/
  * buffer is needed
  * @slot: buffer index where the input image is stored
  * @output_buffer_index: output buffer index where output of resizer will
  * be stored
  * @out_off: The line size in bytes for output buffer. as most probably
  * this will be VRFB with YUV422 data, it should come 0x2000 as input
  * @out_phy_add: physical address of the start of output memory area for this
  * @in_phy_add: physical address of the start of input memory area for this
  * @in_off:: The line size in bytes for output buffer.
  * rsz_begin()  takes the input buffer index and output buffer index
  * to start the process of resizing. after resizing is complete,
  * the callback function will be called with the argument.
  * Indexes of the input and output buffers are used so that it is faster
  * and easier to configure the input and output address for the ISP resizer.
  * As per the current implementation, DSS uses six VRFB contexts for rotation.
  * for both input and output buffers index and physical address has been taken
  * as argument. if this buffer is not already mapped to ISP address space we
  * use physical address to map it, otherwise only the index is used.
  **/
int rsz_begin(u32 input_buffer_index, int output_buffer_index,
		u32 out_off, u32 out_phy_add, u32 in_phy_add, u32 in_off)
{
	unsigned int output_size;
	struct channel_config *rsz_conf_chan = device_config->config;
	struct isp_device *isp = dev_get_drvdata(device_config->isp);

	if (output_buffer_index >= device_config->num_video_buffers) {
		dev_err(rsz_device,
		"ouput buffer index is out of range %d", output_buffer_index);
		return -EINVAL;
	}

	if (multipass_active) {
		rsz_restore_multipass_context();
		rsz_hardware_setup(device_config, rsz_conf_chan);
	}

	/* If this output buffer has not been mapped till now then map it */
	if (!device_config->out_buf_virt_addr[output_buffer_index]) {
		output_size =
			(rsz_conf_chan->register_config.rsz_out_size >>
			ISPRSZ_OUT_SIZE_VERT_SHIFT) * out_off;
		device_config->out_buf_virt_addr[output_buffer_index] = iommu_kmap(
						isp->iommu,
						0,
						out_phy_add,
						output_size,
						IOMMU_FLAG);
		if (IS_ERR_VALUE(
			device_config->out_buf_virt_addr[output_buffer_index])) {
			dev_err(rsz_device, "Mapping of output buffer failed"
						"for index \n");
			return -ENOMEM;
		}
	}

	if (!device_config->in_buf_virt_addr[input_buffer_index]) {
		device_config->in_buf_virt_addr[input_buffer_index] =
				iommu_kmap(isp->iommu,
						0,
						in_phy_add,
						in_off,
						IOMMU_FLAG);
		if (IS_ERR_VALUE(
			device_config->in_buf_virt_addr[input_buffer_index])) {
			dev_err(rsz_device, "Mapping of output buffer failed"
						"for index \n");
			return -ENOMEM;
		}
	}
	down(&resz_wrapper_mutex);
	rsz_conf_chan->register_config.rsz_sdr_inadd =
		device_config->in_buf_virt_addr[input_buffer_index];

	/* Configure the input and output address with output line size
	in resizer hardware */
	isp_reg_writel(device_config->isp,
		rsz_conf_chan->register_config.rsz_sdr_inadd,
		OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_INADD);

	if (!multipass_active) {
		rsz_conf_chan->register_config.rsz_sdr_outoff = out_off;
		isp_reg_writel(device_config->isp,
			rsz_conf_chan->register_config.rsz_sdr_outoff,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTOFF);
		rsz_conf_chan->register_config.rsz_sdr_outadd =
			(u32)device_config->\
			out_buf_virt_addr[output_buffer_index];
		isp_reg_writel(device_config->isp,
			rsz_conf_chan->register_config.rsz_sdr_outadd,
			OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTADD);
	}

	up(&resz_wrapper_mutex);

	/* Set ISP callback for the resizing complete even */
	if (isp_set_callback(device_config->isp,
			     CBK_RESZ_DONE, rsz_isr,
			     (void *) NULL, (void *)NULL)) {
		dev_err(rsz_device, "No callback for RSZR\n");
		return -1;
	}

	/* All settings are done.Enable the resizer */

mult:
	device_config->compl_isr.done = 0;

	ispresizer_enable(&isp->isp_res, 1);
	/* Wait for resizing complete event */
	wait_for_completion_interruptible(&device_config->compl_isr);

	if (device_config->multipass->active) {
		multipass_active = 1;
		rsz_set_multipass(device_config,
				  device_config->multipass, rsz_conf_chan);
		down(&resz_wrapper_mutex);
		if (!device_config->multipass->active) {
			rsz_conf_chan->register_config.rsz_sdr_outoff = out_off;
			isp_reg_writel(device_config->isp,
				rsz_conf_chan->register_config.rsz_sdr_outoff,
				OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTOFF);

			rsz_conf_chan->register_config.rsz_sdr_outadd =
				(u32)device_config->\
				out_buf_virt_addr[output_buffer_index];

			isp_reg_writel(device_config->isp,
				rsz_conf_chan->register_config.rsz_sdr_outadd,
				OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTADD);
		}
		up(&resz_wrapper_mutex);
		goto mult;
	}
	/* Unset the ISP callback function */
	isp_unset_callback(device_config->isp, CBK_RESZ_DONE);

	/* Callback function for DSS driver */
	if (device_config->callback)
		device_config->callback(device_config->callback_arg);

	return 0;
}
EXPORT_SYMBOL(rsz_begin);

 /* rsz_isr - Interrupt Service Routine for Resizer wrapper
 * @status: ISP IRQ0STATUS register value
 * @arg1: Currently not used
 * @arg2: Currently not used
 *
 * Interrupt Service Routine for Resizer wrapper
 **/
static void rsz_isr(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2)
{

	if ((status & RESZ_DONE) != RESZ_DONE)
		return;

	complete(&(device_config->compl_isr));

}

/**
 * resizer_platform_release - Acts when Reference count is zero
 * @device: Structure containing ISP resizer wrapper global information
 *
 * This is called when the reference count goes to zero.
 **/
static void resizer_platform_release(struct device *device)
{
}

/**
 * resizer_probe - Checks for device presence
 * @device: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int __init resizer_probe(struct platform_device *device)
{
	return 0;
}

/**
 * resizer_remove - Handles the removal of the driver
 * @omap_resizer_device: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int resizer_remove(struct platform_device *omap_resizer_device)
{
	return 0;
}

static struct class *rsz_class;
static struct cdev c_dev;
static dev_t dev;
static struct platform_device omap_resizer_device = {
	.name = OMAP_REZR_NAME,
	.id = 2,
	.dev = {
		.release = resizer_platform_release,}
};

static struct platform_driver omap_resizer_driver = {
	.probe = resizer_probe,
	.remove = resizer_remove,
	.driver = {
			.bus = &platform_bus_type,
			.name = OMAP_REZR_NAME,
	},
};

/**
 * omap_rsz_init - Initialization of Resizer Wrapper
 *
 * Returns 0 if successful, -ENOMEM if could not allocate memory, -ENODEV if
 * could not register the wrapper as a character device, or other errors if the
 * device or driver can't register.
 **/
static int __init omap_rsz_init(void)
{
	int ret = 0;
	struct device_params *device;

	device = kzalloc(sizeof(struct device_params), GFP_KERNEL);
	if (!device) {
		printk(KERN_ERR  "%s : could not allocate "
					"memory\n", OMAP_REZR_NAME);
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&dev, 0, 1, OMAP_REZR_NAME);
	if (ret < 0) {
		printk(KERN_ERR "%s : intialization failed. "
			"Could not allocate region "
			"for character device\n" , OMAP_REZR_NAME);
		kfree(device);
		return -ENODEV;
	}

	/* Register the driver in the kernel */
	/* Initialize of character device */
	cdev_init(&c_dev, &rsz_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &rsz_fops;

	/* Addding character device */
	ret = cdev_add(&c_dev, dev, 1);
	if (ret) {
		printk(KERN_ERR  "%s : Error adding "
			"device - %d\n", OMAP_REZR_NAME, ret);
		goto fail2;
	}
	rsz_major = MAJOR(dev);

	/* register driver as a platform driver */
	ret = platform_driver_register(&omap_resizer_driver);
	if (ret) {
		printk(KERN_ERR "%s : Failed to "
			"register platform driver!\n", OMAP_REZR_NAME);
		goto fail3;
	}

	/* Register the drive as a platform device */
	ret = platform_device_register(&omap_resizer_device);
	if (ret) {
		printk(KERN_ERR "%s : Failed to register "
			"platform device!\n", OMAP_REZR_NAME);
		goto fail4;
	}

	rsz_class = class_create(THIS_MODULE, OMAP_REZR_NAME);
	if (!rsz_class) {
		printk(KERN_ERR "%s : Failed to "
			"create class!\n", OMAP_REZR_NAME);
		goto fail5;
	}

	/* make entry in the devfs */
	rsz_device = device_create(rsz_class, rsz_device,
					MKDEV(rsz_major, 0), NULL,
					OMAP_REZR_NAME);
	if (rsz_device)
		dev_dbg(rsz_device, OMAP_REZR_NAME ":"
			"Registered Resizer Wrapper\n");
	else {
		printk(KERN_ERR "%s : Registered Resizer Wrapper",
					OMAP_REZR_NAME);
		ret = -ENODEV;
		goto fail6;
	}

	device->opened = 0;

	device->vbq_ops.buf_setup = rsz_vbq_setup;
	device->vbq_ops.buf_prepare = rsz_vbq_prepare;
	device->vbq_ops.buf_release = rsz_vbq_release;
	device->vbq_ops.buf_queue = rsz_vbq_queue;
	init_completion(&device->compl_isr);
	mutex_init(&device->reszwrap_mutex);

	device_config = device;
	return 0;

fail6:
	class_destroy(rsz_class);
fail5:
	platform_device_unregister(&omap_resizer_device);
fail4:
	platform_driver_unregister(&omap_resizer_driver);
fail3:
	cdev_del(&c_dev);
fail2:
	unregister_chrdev_region(dev, 1);
	kfree(device);
	return ret;
}

/**
 * omap_rsz_exit - Close of Resizer Wrapper
 **/
void __exit omap_rsz_exit(void)
{
	device_destroy(rsz_class, dev);
	class_destroy(rsz_class);
	platform_device_unregister(&omap_resizer_device);
	platform_driver_unregister(&omap_resizer_driver);
	cdev_del(&c_dev);
	unregister_chrdev_region(dev, 1);
	kfree(device_config);
}

module_init(omap_rsz_init)
module_exit(omap_rsz_exit)

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP ISP Resizer");
MODULE_LICENSE("GPL");
