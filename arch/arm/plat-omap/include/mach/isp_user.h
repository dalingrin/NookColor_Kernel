/*
 * isp_user.h
 *
 * Include file for OMAP ISP module in TI's OMAP3.
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Mohit Jalori <mjalori@ti.com>
 *	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_USER_H
#define OMAP_ISP_USER_H

/* ISP Private IOCTLs */
#define VIDIOC_PRIVATE_ISP_CCDC_CFG	\
	_IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct ispccdc_update_config)
#define VIDIOC_PRIVATE_ISP_PRV_CFG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct ispprv_update_config)
#define VIDIOC_PRIVATE_ISP_AEWB_CFG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct isph3a_aewb_config)
#define VIDIOC_PRIVATE_ISP_AEWB_REQ \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 5, struct isph3a_aewb_data)
#define VIDIOC_PRIVATE_ISP_HIST_CFG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct isp_hist_config)
#define VIDIOC_PRIVATE_ISP_HIST_REQ \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 7, struct isp_hist_data)
#define VIDIOC_PRIVATE_ISP_AF_CFG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct af_configuration)
#define VIDIOC_PRIVATE_ISP_AF_REQ \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 9, struct isp_af_data)
#define VIDIOC_PRIVATE_OMAP34XXCAM_SENSOR_INFO	\
	_IOWR('V', BASE_VIDIOC_PRIVATE + 10, struct omap34xxcam_sensor_info)

/*
 *	P R I V A T E   E V E N T S
 */

#define V4L2_EVENT_OMAP3ISP_CLASS	(V4L2_EVENT_PRIVATE_START | 0x100)
#define V4L2_EVENT_OMAP3ISP_AEWB	(V4L2_EVENT_OMAP3ISP_CLASS | 0x1)
#define V4L2_EVENT_OMAP3ISP_AF		(V4L2_EVENT_OMAP3ISP_CLASS | 0x2)
#define V4L2_EVENT_OMAP3ISP_HIST	(V4L2_EVENT_OMAP3ISP_CLASS | 0x3)
#define V4L2_EVENT_OMAP3ISP_HS_VS	(V4L2_EVENT_OMAP3ISP_CLASS | 0x4)

/* Structre for getting Sensor Information*/
struct omap34xxcam_sensor_info {
	__u32 current_xclk;
	struct v4l2_rect active_size;
	struct v4l2_rect full_size;
	struct v4l2_rect pixel_size;
};

/* AE/AWB related structures and flags*/

/* Flags for update field */
#define REQUEST_STATISTICS	(1 << 0)
#define SET_COLOR_GAINS		(1 << 1)
#define SET_DIGITAL_GAIN	(1 << 2)
#define SET_EXPOSURE		(1 << 3)
#define SET_ANALOG_GAIN		(1 << 4)

#define MAX_FRAME_COUNT		0x0FFF
#define MAX_FUTURE_FRAMES	10

#define MAX_SATURATION_LIM	1023
#define MIN_WIN_H		2
#define MAX_WIN_H		256
#define MIN_WIN_W		6
#define MAX_WIN_W		256
#define MAX_WINVC		128
#define MAX_WINHC		36
#define MAX_WINSTART		4095
#define MIN_SUB_INC		2
#define MAX_SUB_INC		32

/* Range Constants */
#define AF_IIRSH_MIN			0
#define AF_IIRSH_MAX			4094
#define AF_PAXEL_HORIZONTAL_COUNT_MIN	0
#define AF_PAXEL_HORIZONTAL_COUNT_MAX	35
#define AF_PAXEL_VERTICAL_COUNT_MIN	0
#define AF_PAXEL_VERTICAL_COUNT_MAX	127
#define AF_PAXEL_INCREMENT_MIN		0
#define AF_PAXEL_INCREMENT_MAX		14
#define AF_PAXEL_HEIGHT_MIN		0
#define AF_PAXEL_HEIGHT_MAX		127
#define AF_PAXEL_WIDTH_MIN		0
#define AF_PAXEL_WIDTH_MAX		127
#define AF_PAXEL_HZSTART_MIN		2
#define AF_PAXEL_HZSTART_MAX		4094

#define AF_PAXEL_VTSTART_MIN		0
#define AF_PAXEL_VTSTART_MAX		4095
#define AF_THRESHOLD_MAX		255
#define AF_COEF_MAX			4095
#define AF_PAXEL_SIZE			48

/**
 * struct isph3a_aewb_config - AE AWB configuration reset values.
 * saturation_limit: Saturation limit.
 * @win_height: Window Height. Range 2 - 256, even values only.
 * @win_width: Window Width. Range 6 - 256, even values only.
 * @ver_win_count: Vertical Window Count. Range 1 - 128.
 * @hor_win_count: Horizontal Window Count. Range 1 - 36.
 * @ver_win_start: Vertical Window Start. Range 0 - 4095.
 * @hor_win_start: Horizontal Window Start. Range 0 - 4095.
 * @blk_ver_win_start: Black Vertical Windows Start. Range 0 - 4095.
 * @blk_win_height: Black Window Height. Range 2 - 256, even values only.
 * @subsample_ver_inc: Subsample Vertical points increment Range 2 - 32, even
 *                     values only.
 * @subsample_hor_inc: Subsample Horizontal points increment Range 2 - 32, even
 *                     values only.
 * @alaw_enable: AEW ALAW EN flag.
 * @aewb_enable: AE AWB stats generation EN flag.
 */
struct isph3a_aewb_config {
	__u16 saturation_limit;
	__u16 win_height;
	__u16 win_width;
	__u16 ver_win_count;
	__u16 hor_win_count;
	__u16 ver_win_start;
	__u16 hor_win_start;
	__u16 blk_ver_win_start;
	__u16 blk_win_height;
	__u16 subsample_ver_inc;
	__u16 subsample_hor_inc;
	__u8 alaw_enable;
	__u8 aewb_enable;
};

/**
 * struct isph3a_aewb_data - Structure of data sent to or received from user
 * @h3a_aewb_statistics_buf: Pointer to pass to user.
 * @shutter: Shutter speed.
 * @gain: Sensor analog Gain.
 * @shutter_cap: Shutter speed for capture.
 * @gain_cap: Sensor Gain for capture.
 * @dgain: White balance digital gain.
 * @wb_gain_b: White balance color gain blue.
 * @wb_gain_r: White balance color gain red.
 * @wb_gain_gb: White balance color gain green blue.
 * @wb_gain_gr: White balance color gain green red.
 * @frame_number: Frame number of requested stats.
 * @curr_frame: Current frame number being processed.
 * @update: Bitwise flags to update parameters.
 * @ts: Timestamp of returned framestats.
 * @field_count: Sequence number of returned framestats.
 */
struct isph3a_aewb_data {
	void *h3a_aewb_statistics_buf;
	__u32 shutter;
	__u16 gain;
	__u32 shutter_cap;
	__u16 gain_cap;
	__u16 dgain;
	__u16 wb_gain_b;
	__u16 wb_gain_r;
	__u16 wb_gain_gb;
	__u16 wb_gain_gr;
	__u16 frame_number;
	__u16 curr_frame;
	__u8 update;
	struct timeval ts;
	__u32 config_counter;
	unsigned long field_count;
};


/* Histogram related structs */

/* Flags for number of bins */
#define HIST_BINS_32		0
#define HIST_BINS_64		1
#define HIST_BINS_128		2
#define HIST_BINS_256		3
#define HIST_MEM_SIZE_BINS(n)	((n)*16)

#define HIST_MEM_SIZE		1024
#define HIST_MIN_REGIONS	1
#define HIST_MAX_REGIONS	4
#define HIST_MAX_WB_GAIN	255
#define HIST_MIN_WB_GAIN	0
#define HIST_MAX_BIT_WIDTH	14
#define HIST_MIN_BIT_WIDTH	8
#define HIST_MAX_BUFF		5
#define HIST_MAX_WG		4

/* Source */
#define HIST_SOURCE_CCDC	0
#define HIST_SOURCE_MEM		1

/* CFA pattern */
#define HIST_CFA_BAYER		0
#define HIST_CFA_FOVEONX3	1

struct isp_hist_config {
	__u8 enable;
	__u8 source;		/* CCDC or memory */
	__u8 input_bit_width;	/* Needed o know the size per pixel */
	__u8 num_acc_frames;	/* Num of frames to be processed and accumulated
				   for each generated histogram frame */
	__u8 hist_h_v_info;	/* frame-input width and height if source is
				 * memory */
	__u16 hist_radd;	/* frame-input address in memory */
	__u16 hist_radd_off;	/* line-offset for frame-input */
	__u16 hist_bins;	/* number of bins: 32, 64, 128, or 256 */
	__u8 cfa;		/* BAYER or FOVEON X3 */
	__u8 wg[HIST_MAX_WG];	/* White Balance Gain */
	__u8 num_regions;	/* number of regions to be configured */
	__u32 reg_hor[HIST_MAX_REGIONS];	/* Regions size and position */
	__u32 reg_ver[HIST_MAX_REGIONS];	/* Regions size and position */
};

struct isp_hist_data {
	__u32 *hist_statistics_buf;	/* Pointer to pass to user */
	__u8 update;
	__u16 frame_number;
	__u16 curr_frame;
	__u32 config_counter;
	struct timeval ts;
};

/* Auto Focus related structs */

#define AF_NUMBER_OF_COEF		11

/* Flags for update field */
#define REQUEST_STATISTICS		(1 << 0)
#define LENS_DESIRED_POSITION	(1 << 1)
#define LENS_CURRENT_POSITION	(1 << 2)

/**
 * struct isp_af_xtrastats - Extra statistics related to AF generated stats.
 * @ts: Timestamp when the frame gets delivered to the user.
 * @field_count: Field count of the frame delivered to the user.
 * @lens_position: Lens position when the stats are being generated.
 */
struct isp_af_xtrastats {
	struct timeval ts;
	unsigned long field_count;
	__u16 lens_position; /* deprecated */
};

/**
 * struct isp_af_data - AF statistics data to transfer between driver and user.
 * @af_statistics_buf: Pointer to pass to user.
 * @lens_current_position: Read value of lens absolute position.
 * @desired_lens_direction: Lens desired location.
 * @update: Bitwise flags to update parameters.
 * @frame_number: Data for which frame is desired/given.
 * @curr_frame: Current frame number being processed by AF module.
 * @xtrastats: Extra statistics structure.
 */
struct isp_af_data {
	void *af_statistics_buf;
	__u16 lens_current_position; /* deprecated */
	__u16 desired_lens_direction; /* deprecated */
	__u16 update;
	__u16 frame_number;
	__u16 curr_frame;
	__u32 config_counter;
	struct isp_af_xtrastats xtrastats;
};

/* enum used for status of specific feature */
enum af_alaw_enable {
	H3A_AF_ALAW_DISABLE = 0,
	H3A_AF_ALAW_ENABLE = 1
};

enum af_hmf_enable {
	H3A_AF_HMF_DISABLE = 0,
	H3A_AF_HMF_ENABLE = 1
};

enum af_config_flag {
	H3A_AF_CFG_DISABLE = 0,
	H3A_AF_CFG_ENABLE = 1
};

enum af_mode {
	ACCUMULATOR_SUMMED = 0,
	ACCUMULATOR_PEAK = 1
};

/* Red, Green, and blue pixel location in the AF windows */
enum rgbpos {
	GR_GB_BAYER = 0,	/* GR and GB as Bayer pattern */
	RG_GB_BAYER = 1,	/* RG and GB as Bayer pattern */
	GR_BG_BAYER = 2,	/* GR and BG as Bayer pattern */
	RG_BG_BAYER = 3,	/* RG and BG as Bayer pattern */
	GG_RB_CUSTOM = 4,	/* GG and RB as custom pattern */
	RB_GG_CUSTOM = 5	/* RB and GG as custom pattern */
};

/* Contains the information regarding the Horizontal Median Filter */
struct af_hmf {
	enum af_hmf_enable enable;	/* Status of Horizontal Median Filter */
	unsigned int threshold;	/* Threshhold Value for Horizontal Median
				 * Filter
				 */
};

/* Contains the information regarding the IIR Filters */
struct af_iir {
	unsigned int hz_start_pos;	/* IIR Start Register Value */
	int coeff_set0[AF_NUMBER_OF_COEF];	/*
						 * IIR Filter Coefficient for
						 * Set 0
						 */
	int coeff_set1[AF_NUMBER_OF_COEF];	/*
						 * IIR Filter Coefficient for
						 * Set 1
						 */
};

/* Contains the information regarding the Paxels Structure in AF Engine */
struct af_paxel {
	unsigned int width;	/* Width of the Paxel */
	unsigned int height;	/* Height of the Paxel */
	unsigned int hz_start;	/* Horizontal Start Position */
	unsigned int vt_start;	/* Vertical Start Position */
	unsigned int hz_cnt;	/* Horizontal Count */
	unsigned int vt_cnt;	/* vertical Count */
	unsigned int line_incr;	/* Line Increment */
};
/* Contains the parameters required for hardware set up of AF Engine */
struct af_configuration {
	enum af_alaw_enable alaw_enable;	/*ALWAW status */
	struct af_hmf hmf_config;	/*HMF configurations */
	enum rgbpos rgb_pos;		/*RGB Positions */
	struct af_iir iir_config;	/*IIR filter configurations */
	struct af_paxel paxel_config;	/*Paxel parameters */
	enum af_mode mode;		/*Accumulator mode */
	enum af_config_flag af_config; /*Flag indicates Engine is configured */
};

/* ISP CCDC structs */

/* Abstraction layer CCDC configurations */
#define ISP_ABS_CCDC_ALAW		(1 << 0)
#define ISP_ABS_CCDC_LPF 		(1 << 1)
#define ISP_ABS_CCDC_BLCLAMP		(1 << 2)
#define ISP_ABS_CCDC_BCOMP		(1 << 3)
#define ISP_ABS_CCDC_FPC		(1 << 4)
#define ISP_ABS_CCDC_CULL		(1 << 5)
#define ISP_ABS_CCDC_COLPTN		(1 << 6)
#define ISP_ABS_CCDC_CONFIG_LSC		(1 << 7)
#define ISP_ABS_TBL_LSC			(1 << 8)

#define RGB_MAX				3

/* Enumeration constants for Alaw input width */
enum alaw_ipwidth {
	ALAW_BIT12_3 = 0x3,
	ALAW_BIT11_2 = 0x4,
	ALAW_BIT10_1 = 0x5,
	ALAW_BIT9_0 = 0x6
};

/* Enumeration constants for Video Port */
enum vpin {
	BIT12_3 = 3,
	BIT11_2 = 4,
	BIT10_1 = 5,
	BIT9_0 = 6
};

enum vpif_freq {
	PIXCLKBY2,
	PIXCLKBY3_5,
	PIXCLKBY4_5,
	PIXCLKBY5_5,
	PIXCLKBY6_5
};

/**
 * struct ispccdc_lsc_config - Structure for LSC configuration.
 * @offset: Table Offset of the gain table.
 * @gain_mode_n: Vertical dimension of a paxel in LSC configuration.
 * @gain_mode_m: Horizontal dimension of a paxel in LSC configuration.
 * @gain_format: Gain table format.
 * @fmtsph: Start pixel horizontal from start of the HS sync pulse.
 * @fmtlnh: Number of pixels in horizontal direction to use for the data
 *          reformatter.
 * @fmtslv: Start line from start of VS sync pulse for the data reformatter.
 * @fmtlnv: Number of lines in vertical direction for the data reformatter.
 * @initial_x: X position, in pixels, of the first active pixel in reference
 *             to the first active paxel. Must be an even number.
 * @initial_y: Y position, in pixels, of the first active pixel in reference
 *             to the first active paxel. Must be an even number.
 * @size: Size of LSC gain table. Filled when loaded from userspace.
 */
struct ispccdc_lsc_config {
	__u16 offset;
	__u8 gain_mode_n;
	__u8 gain_mode_m;
	__u8 gain_format;
	__u16 fmtsph;
	__u16 fmtlnh;
	__u16 fmtslv;
	__u16 fmtlnv;
	__u8 initial_x;
	__u8 initial_y;
	__u32 size;
};

/**
 * struct ispccdc_bclamp - Structure for Optical & Digital black clamp subtract
 * @obgain: Optical black average gain.
 * @obstpixel: Start Pixel w.r.t. HS pulse in Optical black sample.
 * @oblines: Optical Black Sample lines.
 * @oblen: Optical Black Sample Length.
 * @dcsubval: Digital Black Clamp subtract value.
 */
struct ispccdc_bclamp {
	__u8 obgain;
	__u8 obstpixel;
	__u8 oblines;
	__u8 oblen;
	__u16 dcsubval;
};

/**
 * ispccdc_fpc - Structure for FPC
 * @fpnum: Number of faulty pixels to be corrected in the frame.
 * @fpcaddr: Memory address of the FPC Table
 */
struct ispccdc_fpc {
	__u16 fpnum;
	__u32 fpcaddr;
};

/**
 * ispccdc_blcomp - Structure for Black Level Compensation parameters.
 * @b_mg: B/Mg pixels. 2's complement. -128 to +127.
 * @gb_g: Gb/G pixels. 2's complement. -128 to +127.
 * @gr_cy: Gr/Cy pixels. 2's complement. -128 to +127.
 * @r_ye: R/Ye pixels. 2's complement. -128 to +127.
 */
struct ispccdc_blcomp {
	__u8 b_mg;
	__u8 gb_g;
	__u8 gr_cy;
	__u8 r_ye;
};

/**
 * struct ispccdc_vp - Structure for Video Port parameters
 * @bitshift_sel: Video port input select. 3 - bits 12-3, 4 - bits 11-2,
 *                5 - bits 10-1, 6 - bits 9-0.
 * @freq_sel: Video port data ready frequency. 1 - 1/3.5, 2 - 1/4.5,
 *            3 - 1/5.5, 4 - 1/6.5.
 */
struct ispccdc_vp {
	enum vpin bitshift_sel;
	enum vpif_freq freq_sel;
};

/**
 * ispccdc_culling - Structure for Culling parameters.
 * @v_pattern: Vertical culling pattern.
 * @h_odd: Horizontal Culling pattern for odd lines.
 * @h_even: Horizontal Culling pattern for even lines.
 */
struct ispccdc_culling {
	__u8 v_pattern;
	__u16 h_odd;
	__u16 h_even;
};

/**
 * ispccdc_update_config - Structure for CCDC configuration.
 * @update: Specifies which CCDC registers should be updated.
 * @flag: Specifies which CCDC functions should be enabled.
 * @alawip: Enable/Disable A-Law compression.
 * @bclamp: Black clamp control register.
 * @blcomp: Black level compensation value for RGrGbB Pixels. 2's complement.
 * @fpc: Number of faulty pixels corrected in the frame, address of FPC table.
 * @cull: Cull control register.
 * @colptn: Color pattern of the sensor.
 * @lsc: Pointer to LSC gain table.
 */
struct ispccdc_update_config {
	__u16 update;
	__u16 flag;
	enum alaw_ipwidth alawip;
	struct ispccdc_bclamp *bclamp;
	struct ispccdc_blcomp *blcomp;
	struct ispccdc_fpc *fpc;
	struct ispccdc_lsc_config *lsc_cfg;
	struct ispccdc_culling *cull;
	__u32 colptn;
	__u8 *lsc;
};

/* Preview configuration */

/*Abstraction layer preview configurations*/
#define ISP_ABS_PREV_LUMAENH		(1 << 0)
#define ISP_ABS_PREV_INVALAW		(1 << 1)
#define ISP_ABS_PREV_HRZ_MED		(1 << 2)
#define ISP_ABS_PREV_CFA		(1 << 3)
#define ISP_ABS_PREV_CHROMA_SUPP	(1 << 4)
#define ISP_ABS_PREV_WB			(1 << 5)
#define ISP_ABS_PREV_BLKADJ		(1 << 6)
#define ISP_ABS_PREV_RGB2RGB		(1 << 7)
#define ISP_ABS_PREV_COLOR_CONV		(1 << 8)
#define ISP_ABS_PREV_YC_LIMIT		(1 << 9)
#define ISP_ABS_PREV_DEFECT_COR		(1 << 10)
#define ISP_ABS_PREV_GAMMABYPASS	(1 << 11)
#define ISP_ABS_TBL_NF 			(1 << 12)
#define ISP_ABS_TBL_REDGAMMA		(1 << 13)
#define ISP_ABS_TBL_GREENGAMMA		(1 << 14)
#define ISP_ABS_TBL_BLUEGAMMA		(1 << 15)

#define ISPPRV_NF_TBL_SIZE		64
#define ISPPRV_CFA_TBL_SIZE		576
#define ISPPRV_GAMMA_TBL_SIZE		1024
#define ISPPRV_YENH_TBL_SIZE		128

/**
 * struct ispprev_hmed - Structure for Horizontal Median Filter.
 * @odddist: Distance between consecutive pixels of same color in the odd line.
 * @evendist: Distance between consecutive pixels of same color in the even
 *            line.
 * @thres: Horizontal median filter threshold.
 */
struct ispprev_hmed {
	__u8 odddist;
	__u8 evendist;
	__u8 thres;
};

/*
 * Enumeration for CFA Formats supported by preview
 */
enum cfa_fmt {
	CFAFMT_BAYER, CFAFMT_SONYVGA, CFAFMT_RGBFOVEON,
	CFAFMT_DNSPL, CFAFMT_HONEYCOMB, CFAFMT_RRGGBBFOVEON
};

/**
 * struct ispprev_cfa - Structure for CFA Inpterpolation.
 * @cfafmt: CFA Format Enum value supported by preview.
 * @cfa_gradthrs_vert: CFA Gradient Threshold - Vertical.
 * @cfa_gradthrs_horz: CFA Gradient Threshold - Horizontal.
 * @cfa_table: Pointer to the CFA table.
 */
struct ispprev_cfa {
	enum cfa_fmt cfafmt;
	__u8 cfa_gradthrs_vert;
	__u8 cfa_gradthrs_horz;
	__u32 *cfa_table;
};

/**
 * struct ispprev_csup - Structure for Chrominance Suppression.
 * @gain: Gain.
 * @thres: Threshold.
 * @hypf_en: Flag to enable/disable the High Pass Filter.
 */
struct ispprev_csup {
	__u8 gain;
	__u8 thres;
	__u8 hypf_en;
};

/**
 * struct ispprev_wbal - Structure for White Balance.
 * @dgain: Digital gain (U10Q8).
 * @coef3: White balance gain - COEF 3 (U8Q5).
 * @coef2: White balance gain - COEF 2 (U8Q5).
 * @coef1: White balance gain - COEF 1 (U8Q5).
 * @coef0: White balance gain - COEF 0 (U8Q5).
 */
struct ispprev_wbal {
	__u16 dgain;
	__u8 coef3;
	__u8 coef2;
	__u8 coef1;
	__u8 coef0;
};

/**
 * struct ispprev_blkadj - Structure for Black Adjustment.
 * @red: Black level offset adjustment for Red in 2's complement format
 * @green: Black level offset adjustment for Green in 2's complement format
 * @blue: Black level offset adjustment for Blue in 2's complement format
 */
struct ispprev_blkadj {
	/*Black level offset adjustment for Red in 2's complement format */
	__u8 red;
	/*Black level offset adjustment for Green in 2's complement format */
	__u8 green;
	/* Black level offset adjustment for Blue in 2's complement format */
	__u8 blue;
};

/**
 * struct ispprev_rgbtorgb - Structure for RGB to RGB Blending.
 * @matrix: Blending values(S12Q8 format)
 *              [RR] [GR] [BR]
 *              [RG] [GG] [BG]
 *              [RB] [GB] [BB]
 * @offset: Blending offset value for R,G,B in 2's complement integer format.
 */
struct ispprev_rgbtorgb {
	__u16 matrix[3][3];
	__u16 offset[3];
};

/**
 * struct ispprev_csc - Structure for Color Space Conversion from RGB-YCbYCr
 * @matrix: Color space conversion coefficients(S10Q8)
 *              [CSCRY]  [CSCGY]  [CSCBY]
 *              [CSCRCB] [CSCGCB] [CSCBCB]
 *              [CSCRCR] [CSCGCR] [CSCBCR]
 * @offset: CSC offset values for Y offset, CB offset and CR offset respectively
 */
struct ispprev_csc {
	__u16 matrix[RGB_MAX][RGB_MAX];
	__s16 offset[RGB_MAX];
};

/**
 * struct ispprev_yclimit - Structure for Y, C Value Limit.
 * @minC: Minimum C value
 * @maxC: Maximum C value
 * @minY: Minimum Y value
 * @maxY: Maximum Y value
 */
struct ispprev_yclimit {
	__u8 minC;
	__u8 maxC;
	__u8 minY;
	__u8 maxY;
};

/**
 * struct ispprev_dcor - Structure for Defect correction.
 * @couplet_mode_en: Flag to enable or disable the couplet dc Correction in NF
 * @detect_correct: Thresholds for correction bit 0:10 detect 16:25 correct
 */
struct ispprev_dcor {
	__u8 couplet_mode_en;
	__u32 detect_correct[4];
};

/**
 * struct ispprev_nf - Structure for Noise Filter
 * @spread: Spread value to be used in Noise Filter
 * @table: Pointer to the Noise Filter table
 */
struct ispprev_nf {
	__u8 spread;
	__u32 table[ISPPRV_NF_TBL_SIZE];
};

/**
 * struct ispprv_update_config - Structure for Preview Configuration (user).
 * @update: Specifies which ISP Preview registers should be updated.
 * @flag: Specifies which ISP Preview functions should be enabled.
 * @yen: Pointer to luma enhancement table.
 * @shading_shift: 3bit value of shift used in shading compensation.
 * @prev_hmed: Pointer to structure containing the odd and even distance.
 *             between the pixels in the image along with the filter threshold.
 * @prev_cfa: Pointer to structure containing the CFA interpolation table, CFA.
 *            format in the image, vertical and horizontal gradient threshold.
 * @csup: Pointer to Structure for Chrominance Suppression coefficients.
 * @prev_wbal: Pointer to structure for White Balance.
 * @prev_blkadj: Pointer to structure for Black Adjustment.
 * @rgb2rgb: Pointer to structure for RGB to RGB Blending.
 * @prev_csc: Pointer to structure for Color Space Conversion from RGB-YCbYCr.
 * @yclimit: Pointer to structure for Y, C Value Limit.
 * @prev_dcor: Pointer to structure for defect correction.
 * @prev_nf: Pointer to structure for Noise Filter
 * @red_gamma: Pointer to red gamma correction table.
 * @green_gamma: Pointer to green gamma correction table.
 * @blue_gamma: Pointer to blue gamma correction table.
 */
struct ispprv_update_config {
	__u16 update;
	__u16 flag;
	void *yen;
	__u32 shading_shift;
	struct ispprev_hmed *prev_hmed;
	struct ispprev_cfa *prev_cfa;
	struct ispprev_csup *csup;
	struct ispprev_wbal *prev_wbal;
	struct ispprev_blkadj *prev_blkadj;
	struct ispprev_rgbtorgb *rgb2rgb;
	struct ispprev_csc *prev_csc;
	struct ispprev_yclimit *yclimit;
	struct ispprev_dcor *prev_dcor;
	struct ispprev_nf *prev_nf;
	__u32 *red_gamma;
	__u32 *green_gamma;
	__u32 *blue_gamma;
};

#endif /* OMAP_ISP_USER_H */
