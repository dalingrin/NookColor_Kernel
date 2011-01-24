/*
 * linux/sound/soc/codecs/tlv320aic3111.c
 *
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Based on sound/soc/codecs/wm8753.c by Liam Girdwood
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support   AIC3111         27-11-2009
 *   
 *			 The AIC3252 ASoC driver is ported for the codec AIC3111.
 *     
 *
 * 
 * 
 */

/***************************** INCLUDES ************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/workqueue.h>


#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "tlv320aic3111.h"
#include <mach/gpio.h>

#include <linux/i2c/twl4030.h>

//#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define DBG(x...) printk(KERN_ALERT x)
#else
#define DBG(x...)
#endif

/*
 ***************************************************************************** 
 * Macros
 ***************************************************************************** 
 */
 #define AIC_FORCE_SWITCHES_ON 


#ifdef CONFIG_MINI_DSP
extern int aic3111_minidsp_program(struct snd_soc_codec *codec);
extern void aic3111_add_minidsp_controls(struct snd_soc_codec *codec);
#endif

#define SOC_SINGLE_AIC3111(xname) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = __new_control_info, .get = __new_control_get,\
	.put = __new_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
}



#define SOC_DOUBLE_R_AIC3111(xname, reg_left, reg_right, shift, mask, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw_2r_aic3111, \
	.get = snd_soc_get_volsw_2r_aic3111, .put = snd_soc_put_volsw_2r_aic3111, \
	.private_value = (reg_left) | ((shift) << 8)  | \
		((mask) << 12) | ((invert) << 20) | ((reg_right) << 24) }
#if defined(CONFIG_MACH_OMAP3621_EDP1) || defined(CONFIG_MACH_OMAP3630_EDP1) \
     || defined(CONFIG_MACH_OMAP3430_EDP1) || defined(CONFIG_MACH_OMAP3621_BOXER)
#define    AUDIO_CODEC_HPH_DETECT_GPIO		(156)
#define    AUDIO_CODEC_PWR_ON_GPIO		(158)
  #ifdef CONFIG_MACH_OMAP3621_BOXER
  #define    AUDIO_CODEC_RESET_GPIO		(37)
  #endif
#else
#error "Please check you AIC3111 GPIOs!"
#endif
#define    AUDIO_CODEC_PWR_ON_GPIO_NAME		"audio_codec_pwron"
#define    AUDIO_CODEC_RESET_GPIO_NAME		"audio_codec_reset"

/*
 ***************************************************************************** 
 * Function Prototype
 ***************************************************************************** 
 */
static int aic3111_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *tmp);

static int aic3111_mute(struct snd_soc_dai *dai, int mute);

static int aic3111_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir);

static int aic3111_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);

static int aic3111_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level);

static unsigned int aic3111_read(struct snd_soc_codec *codec, unsigned int reg);

static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);

static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int __new_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int snd_soc_info_volsw_2r_aic3111(struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_info *uinfo);

static int snd_soc_get_volsw_2r_aic3111(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

static int snd_soc_put_volsw_2r_aic3111(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

static int aic3111_headset_speaker_path(struct snd_soc_codec *codec);
static irqreturn_t aic3111_irq_handler(int irq, struct snd_soc_codec *codec);

/*
 ***************************************************************************** 
 * Global Variable
 ***************************************************************************** 
 */
static u8 aic3111_reg_ctl;

/* whenever aplay/arecord is run, aic3111_hw_params() function gets called. 
 * This function reprograms the clock dividers etc. this flag can be used to 
 * disable this when the clock dividers are programmed by pps config file
 */
static int soc_static_freq_config = 1;

/*
 ***************************************************************************** 
 * Structure Declaration
 ***************************************************************************** 
 */
static struct snd_soc_device *aic3111_socdev;

/*
 ***************************************************************************** 
 * Structure Initialization
 ***************************************************************************** 
 */
static const struct snd_kcontrol_new aic3111_snd_controls[] = {
	/* Output */
	/* sound new kcontrol for PCM Playback volume control */
	SOC_DOUBLE_R_AIC3111("PCM Playback Volume", LDAC_VOL, RDAC_VOL, 0, 0xAf,
			     0),
	/* sound new kcontrol for HP driver gain */
	SOC_DOUBLE_R_AIC3111("HP Driver Gain", HPL_DRIVER, HPR_DRIVER, 0, 0x23, 0),
	/* sound new kcontrol for LO driver gain */
	SOC_DOUBLE_R_AIC3111("LO Driver Gain", SPL_DRIVER, SPR_DRIVER, 0, 0x23, 0),
	/* sound new kcontrol for HP mute */
	SOC_DOUBLE_R("HP DAC Playback Switch", HPL_DRIVER, HPR_DRIVER, 2,
		     0x01, 1),
	/* sound new kcontrol for LO mute */
	SOC_DOUBLE_R("LO DAC Playback Switch", SPL_DRIVER, SPR_DRIVER, 2,
		     0x01, 1),

	/* Input */
	/* sound new kcontrol for PGA capture volume */
	SOC_DOUBLE_R_AIC3111("PGA Capture Volume", ADC_VOL, ADC_VOL, 0, 0x3F,
			     0),

	/* sound new kcontrol for Programming the registers from user space */
	SOC_SINGLE_AIC3111("Program Registers"),

};


/* the sturcture contains the different values for mclk */
static const struct aic3111_rate_divs aic3111_divs[] = {
/* 
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc, blck_N, 
 * codec_speficic_initializations 
 */
	/* 8k rate */
	// DDenchev (MMS)
	{12000000, 8000, 1, 7, 1680, 128, 2, 42, 128, 2, 42, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 1}}},
//	{12000000, 8000, 1, 6, 9120, 128, 3, 27, 128, 3, 27, 24,
//	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 8000, 1, 6, 3803, 128, 3, 27, 128, 3, 27, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 4}}},
	{24000000, 8000, 2, 7, 6800, 768, 15, 1, 64, 45, 4, 24,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 11.025k rate */
	// DDenchev (MMS)
	{12000000, 11025, 1, 7, 560, 128, 5, 12, 128, 5, 12, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 11025, 1, 6, 1876, 128, 3, 19, 128, 3, 19, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 11025, 2, 7, 5264, 512, 16, 1, 64, 32, 4, 16,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 12k rate */
	// DDenchev (MMS)
	{12000000, 12000, 1, 7, 1680, 128, 2, 28, 128, 2, 28, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 12000, 1, 6, 3803, 128, 3, 18, 128, 3, 18, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},

	/* 16k rate */
	// DDenchev (MMS)
	{12000000, 16000, 1, 7, 1680, 128, 2, 21, 128, 2, 21, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 16000, 1, 6, 6166, 128, 3, 14, 128, 3, 14, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 16000, 2, 7, 6800, 384, 15, 1, 64, 18, 5, 12,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 22.05k rate */
	// DDenchev (MMS)
	{12000000, 22050, 1, 7, 560, 128, 5, 6, 128, 5, 6, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 22050, 1, 6, 5132, 128, 3, 10, 128, 3, 10, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 22050, 2, 7, 5264, 256, 16, 1, 64, 16, 4, 8,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 24k rate */
	// DDenchev (MMS)
	{12000000, 24000, 1, 7, 1680, 128, 2, 14, 128, 2, 14, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 24000, 1, 6, 3803, 128, 3, 9, 128, 3, 9, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},

	/* 32k rate */
	// DDenchev (MMS)
	{12000000, 32000, 1, 6, 1440, 128, 2, 9, 128, 2, 9, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 32000, 1, 6, 6166, 128, 3, 7, 128, 3, 7, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 32000, 2, 7, 1680, 192, 7, 2, 64, 7, 6, 6,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 44.1k rate */
	// DDenchev (MMS)
	{12000000, 44100, 1, 7, 560, 128, 5, 3, 128, 5, 3, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 1}}},
	{13000000, 44100, 1, 6, 5132, 128, 3, 5, 128, 3, 5, 4,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 4}}},
	{24000000, 44100, 2, 7, 5264, 128, 8, 2, 64, 8, 4, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/* 48k rate */
	// DDenchev (MMS)
	{12000000, 48000, 1, 7, 1680, 128, 2, 7, 128, 2, 7, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},
	{13000000, 48000, 1, 6, 6166, 128, 7, 2, 128, 7, 2, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 4}}},
	{24000000, 48000, 2, 8, 1920, 128, 8, 2, 64, 8, 4, 4,
	 {{DAC_INSTRUCTION_SET, 1}, {61, 1}}},

	/*96k rate : GT 21/12/2009: NOT MODIFIED */
	{12000000, 96000, 1, 8, 1920, 64, 2, 8, 64, 2, 8, 2,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 7}}},
	{13000000, 96000, 1, 6, 6166, 64, 7, 2, 64, 7, 2, 2,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 10}}},
	{24000000, 96000, 2, 8, 1920, 64, 4, 4, 64, 8, 2, 2,
	 {{DAC_INSTRUCTION_SET, 7}, {61, 7}}},

	/*192k : GT 21/12/2009: NOT MODIFIED */
	{12000000, 192000, 1, 8, 1920, 32, 2, 8, 32, 2, 8, 1,
	 {{DAC_INSTRUCTION_SET, 17}, {61, 13}}},
	{13000000, 192000, 1, 6, 6166, 32, 7, 2, 32, 7, 2, 1,
	 {{DAC_INSTRUCTION_SET, 17}, {61, 13}}},
	{24000000, 192000, 2, 8, 1920, 32, 4, 4, 32, 4, 4, 1,
	 {{DAC_INSTRUCTION_SET, 17}, {61, 13}}},
};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *          It is SoC Codec DAI structure which has DAI capabilities viz., 
 *          playback and capture, DAI runtime information viz. state of DAI 
 *			and pop wait state, and DAI private data. 
 *          The AIC3111 rates ranges from 8k to 192k
 *          The PCM bit format supported are 16, 20, 24 and 32 bits
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai tlv320aic3111_dai = {
	.name = "TLV320AIC3111",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = AIC3111_RATES,
		     .formats = AIC3111_FORMATS,},
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = AIC3111_RATES,
		    .formats = AIC3111_FORMATS,},
	.ops = {
		.hw_params = aic3111_hw_params,
		.digital_mute = aic3111_mute,
		.set_sysclk = aic3111_set_dai_sysclk,
		.set_fmt = aic3111_set_dai_fmt,
		}
};

EXPORT_SYMBOL_GPL(tlv320aic3111_dai);

#ifdef DEBUG
void debug_print_registers (struct snd_soc_codec *codec)
{
	int i;
	u32 data;

	for (i = 0 ; i < 80 ; i++) {
		data = aic3111_read(codec, i);
		printk(KERN_ALERT "reg = %d val = %x\n", i, data);
	}
}
#endif //DEBUG

/*
 ***************************************************************************** 
 * Initializations
 ***************************************************************************** 
 */
/*
 * AIC3111 register cache
 * We are caching the registers here.
 * There is no point in caching the reset register.
 * NOTE: In AIC32, there are 127 registers supported in both page0 and page1
 *       The following table contains the page0 and page 1registers values.
 */
static const u8 aic3111_reg[AIC3111_CACHEREGNUM] = {
	0x00, 0x00, 0x00, 0x02,	/* 0 */
	0x00, 0x11, 0x04, 0x00,	/* 4 */
	0x00, 0x00, 0x00, 0x01,	/* 8 */
	0x01, 0x00, 0x80, 0x80,	/* 12 */
	0x08, 0x00, 0x01, 0x01,	/* 16 */
	0x80, 0x80, 0x04, 0x00,	/* 20 */
	0x00, 0x00, 0x01, 0x00,	/* 24 */
	0x00, 0x00, 0x01, 0x00,	/* 28 */
	0x00, 0x00, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x00, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x00,	/* 40 */
	0x00, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
	0x00, 0x02, 0x02, 0x00,	/* 52 */
	0x00, 0x00, 0x00, 0x00,	/* 56 */
	0x01, 0x04, 0x00, 0x14,	/* 60 */
	0x0C, 0x00, 0x00, 0x00,	/* 64 */
	0x0F, 0x38, 0x00, 0x00,	/* 68 */
	0x00, 0x00, 0x00, 0xEE,	/* 72 */
	0x10, 0xD8, 0x7E, 0xE3,	/* 76 */
	0x00, 0x00, 0x80, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
	0x7F, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x00, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x00, 0x00,	/* 100 */
	0x00, 0x00, 0x00, 0x00,	/* 104 */
	0x00, 0x00, 0x00, 0x00,	/* 108 */
	0x00, 0x00, 0x00, 0x00,	/* 112 */
	0x00, 0x00, 0x00, 0x00,	/* 116 */
	0x00, 0x00, 0x00, 0x00,	/* 120 */
	0x00, 0x00, 0x00, 0x00,	/* 124 - PAGE0 Registers(127) ends here */
	0x00, 0x00, 0x00, 0x00,	/* 128, PAGE1-0 */
	0x00, 0x00, 0x00, 0x00,	/* 132, PAGE1-4 */
	0x00, 0x00, 0x00, 0x00,	/* 136, PAGE1-8 */
	0x00, 0x00, 0x00, 0x00,	/* 140, PAGE1-12 */
	0x00, 0x00, 0x00, 0x00,	/* 144, PAGE1-16 */
	0x00, 0x00, 0x00, 0x00,	/* 148, PAGE1-20 */
	0x00, 0x00, 0x00, 0x00,	/* 152, PAGE1-24 */
	0x00, 0x00, 0x00, 0x04,	/* 156, PAGE1-28 */
	0x06, 0x3E, 0x00, 0x00,	/* 160, PAGE1-32 */
	0x7F, 0x7F, 0x7F, 0x7F,	/* 164, PAGE1-36 */
	0x02, 0x02, 0x00, 0x00,	/* 168, PAGE1-40 */
	0x00, 0x00, 0x00, 0x80,	/* 172, PAGE1-44 */
	0x00, 0x00, 0x00, 0x00,	/* 176, PAGE1-48 */
	0x00, 0x00, 0x00, 0x00,	/* 180, PAGE1-52 */
	0x00, 0x00, 0x00, 0x80,	/* 184, PAGE1-56 */
	0x80, 0x00, 0x00, 0x00,	/* 188, PAGE1-60 */
	0x00, 0x00, 0x00, 0x00,	/* 192, PAGE1-64 */
	0x00, 0x00, 0x00, 0x00,	/* 196, PAGE1-68 */
	0x00, 0x00, 0x00, 0x00,	/* 200, PAGE1-72 */
	0x00, 0x00, 0x00, 0x00,	/* 204, PAGE1-76 */
	0x00, 0x00, 0x00, 0x00,	/* 208, PAGE1-80 */
	0x00, 0x00, 0x00, 0x00,	/* 212, PAGE1-84 */
	0x00, 0x00, 0x00, 0x00,	/* 216, PAGE1-88 */
	0x00, 0x00, 0x00, 0x00,	/* 220, PAGE1-92 */
	0x00, 0x00, 0x00, 0x00,	/* 224, PAGE1-96 */
	0x00, 0x00, 0x00, 0x00,	/* 228, PAGE1-100 */
	0x00, 0x00, 0x00, 0x00,	/* 232, PAGE1-104 */
	0x00, 0x00, 0x00, 0x00,	/* 236, PAGE1-108 */
	0x00, 0x00, 0x00, 0x00,	/* 240, PAGE1-112 */
	0x00, 0x00, 0x00, 0x00,	/* 244, PAGE1-116 */
	0x00, 0x00, 0x00, 0x00,	/* 248, PAGE1-120 */
	0x00, 0x00, 0x00, 0x00	/* 252, PAGE1-124 */
};

/* 
 * aic3111 initialization data 
 * This structure initialization contains the initialization required for
 * AIC3111.
 * These registers values (reg_val) are written into the respective AIC3111 
 * register offset (reg_offset) to  initialize AIC3111. 
 * These values are used in aic3111_init() function only. 
 */
#if 1
static const struct aic3111_configs aic3111_reg_init[] = {
	/* Carry out the software reset */
	{RESET, 0x01},
	/* Connect MIC1_L and MIC1_R to CM */
	{MICPGA_CM, 0xC0},
	/* PLL is CODEC_CLKIN */
//	{CLK_REG_1, MCLK_2_CODEC_CLKIN}, //if pb set DIN
	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
	/* DAC_MOD_CLK is BCLK source */
	{AIS_REG_3, DAC_MOD_CLK_2_BDIV_CLKIN},
	/* Setting up DAC Channel */
//	{DAC_CHN_REG, LDAC_2_LCHN | RDAC_2_RCHN | SOFT_STEP_2WCLK},
	{DAC_CHN_REG, 0xD5},
	/* Headphone powerup */
	{HP_OUT_DRIVERS, 0x3D},  // reset value
	/* DAC_L and DAC_R Output Mixer Routing */ 
	{DAC_MIXER_ROUTING, 0x44},  //DAC_X is routed to the channel mixer amplifier
	/* HPL unmute and gain 0db */
	{HPL_DRIVER, 0x6},
	/* HPR unmute and gain 0db */
	{HPR_DRIVER, 0x6},//e
	/* Headphone drivers */
	{HPHONE_DRIVERS, 0xC4}, //0x44
	/* HPL unmute and gain 0db */
	{LEFT_ANALOG_HPL, 0x80}, //0
	/* HPR unmute and gain 0db */
	{RIGHT_ANALOG_HPR, 0x80}, //0
	{LEFT_ANALOG_SPL, 0x88}, //en plus 0x80 - 0x92
	{RIGHT_ANALOG_SPR, 0x88},//en plus 0x80 - 0x92
	/* LOL unmute and gain 0db */
	{SPL_DRIVER, 0x1c}, //x04
	/* LOR unmute and gain 0db */
	{SPR_DRIVER, 0x1c}, //0x4
	/* Unmute DAC Left and Right channels */
	{DAC_MUTE_CTRL_REG, 0x0C},
	{MICPGA_VOL_CTRL, 0x40},
	/* IN1_L is selected for left P */
	{MICPGA_PIN_CFG, 0x40},
	/* IN1_R is selected for right P */
	{MICPGA_MIN_CFG, 0x00},
	/* mic PGA unmuted */
	{MICPGA_VOL_CTRL, 0x00},
	/* ADC volume control change by 2 gain step per ADC Word Clock */
	{ADC_REG_1, 0x02},
	/* Unmute ADC left and right channels */
	{ADC_FGA, 0x00},
	/* DAC power on */
	{CLASS_D_SPK, 0xC6},
};
#else

//config  de gilles.
static const struct aic3111_configs aic3111_reg_init[] = {
	/* Carry out the software reset */
	{RESET, 0x01},
	/* PLL is CODEC_CLKIN */
	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
	/* DAC_MOD_CLK is BCLK source */
//	{AIS_REG_3, DAC_MOD_CLK_2_BDIV_CLKIN},
	/* Setting up DAC Channel */
	{DAC_CHN_REG,
	 LDAC_2_LCHN | RDAC_2_RCHN },
	/* Headphone powerup */
	{HP_OUT_DRIVERS, 0xc2},  // reset value
	/* DAC_L and DAC_R Output Mixer Routing */ 
	{DAC_MIXER_ROUTING, 0x44},  //DAC_X is routed to the channel mixer amplifier
	/* HPL unmute and gain 0db */
	{HPL_DRIVER, 0xe},
	/* HPR unmute and gain 0db */
	{HPR_DRIVER, 0xe},
	/* HPL unmute and gain 0db */
	{LEFT_ANALOG_HPL, 0},
	/* HPR unmute and gain 0db */
	{RIGHT_ANALOG_HPR, 0},
//@@GT: taken from vincent's config[START]
	{LEFT_ANALOG_SPL, 0x80}, //en plus 0x80
	{RIGHT_ANALOG_SPR, 0x80},//en plus 0x80
//@@GT: taken from vincent's config[END]
	/* LOL unmute and gain 12db */
	{SPL_DRIVER, 0x0c},
	/* LOR unmute and gain 12db */
	{SPR_DRIVER, 0x0c},
//@@GT: taken from vincent's config[START]
	/* Unmute DAC Left and Right channels */
	{DAC_MUTE_CTRL_REG, 0x0C},
//@@GT: taken from vincent's config[END]
	/* MICBIAS control */
	{MICBIAS_CTRL, 0x0b},
	/* MICBIAS control */
	{MICPGA_VOL_CTRL, 0x40},
	/* IN1_L is selected for left P */
	{MICPGA_PIN_CFG, 0x40},
	/* IN1_R is selected for right P */
	{MICPGA_MIN_CFG, 0x00},
	/* ADC volume control change by 2 gain step per ADC Word Clock */
//	{ADC_REG_1, 0x80},
	/* Unmute ADC left and right channels */
//	{ADC_FGA, 0x00},
	{ CLASS_D_SPK, 0xC6 }, // forced for now
};
#endif

/* Left DAC_L Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIXER_ROUTING, 6, 2, 1),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIXER_ROUTING, 5, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIXER_ROUTING, 2, 2, 1),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIXER_ROUTING, 1, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", HPR_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", DAC_MIXER_ROUTING, 6, 2, 1),    //SOC_DAPM_SINGLE("L_DAC switch", LOL_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC1_L switch", DAC_MIXER_ROUTING, 5, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", DAC_MIXER_ROUTING, 2, 2, 1),    //SOC_DAPM_SINGLE("R_DAC switch", LOR_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("MIC1_R switch", DAC_MIXER_ROUTING, 1, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", LOR_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_L switch", MICPGA_PIN_CFG, 6, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MICPGA_PIN_CFG, 2, 1, 0),
};

static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("MIC1_R switch", MICPGA_PIN_CFG, 4, 1, 0),
	SOC_DAPM_SINGLE("MIC1_M switch", MICPGA_PIN_CFG, 2, 1, 0),
};

static const struct snd_soc_dapm_widget aic3111_dapm_widgets[] = {
	/* Left DAC to Left Outputs */
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_CHN_REG, 7, 1),
	/* dapm widget (path domain) for left DAC_L Mixer */

	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_output_mixer_controls[0],
			   ARRAY_SIZE(hpl_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPL Power", HPHONE_DRIVERS, 7, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lol_output_mixer_controls[0],
			   ARRAY_SIZE(lol_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOL Power", CLASS_D_SPK, 7, 1, NULL, 0),

	/* Right DAC to Right Outputs */
	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_CHN_REG, 6, 1),
	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_output_mixer_controls[0],
			   ARRAY_SIZE(hpr_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPR Power", HPHONE_DRIVERS, 6, 0, NULL, 0), 

	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lor_output_mixer_controls[0],
			   ARRAY_SIZE(lor_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOR Power", CLASS_D_SPK, 6, 1, NULL, 0),

	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			   &left_input_mixer_controls[0],
			   ARRAY_SIZE(left_input_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			   &right_input_mixer_controls[0],
			   ARRAY_SIZE(right_input_mixer_controls)),

	/* Inputs to Left ADC */
	SND_SOC_DAPM_ADC("ADC", "Capture", ADC_REG_1, 7, 0),

	/* Right Inputs to Right ADC */
	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", ADC_REG_1, 6, 0),

	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),
	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),
	/* dapm widget (platform domain) name for LOLOUT */
	SND_SOC_DAPM_OUTPUT("LOL"),
	/* dapm widget (platform domain) name for LOROUT */
	SND_SOC_DAPM_OUTPUT("LOR"),

	/* dapm widget (platform domain) name for MIC1LP */
	SND_SOC_DAPM_INPUT("MIC1LP"),
	/* dapm widget (platform domain) name for MIC1RP*/
	SND_SOC_DAPM_INPUT("MIC1RP"),
	/* dapm widget (platform domain) name for MIC1LM */
	SND_SOC_DAPM_INPUT("MIC1LM"),
};


/*
* DAPM audio route definition. *
* Defines an audio route originating at source via control and finishing 
* at sink. 
*/
static const struct snd_soc_dapm_route aic3111_dapm_routes[] = {
	/* ******** Right Output ******** */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer",  "MIC1_R switch", "MIC1RP"},
	//{"HPR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"HPR Power", NULL, "HPR Output Mixer"},
	{"HPR", NULL, "HPR Power"},

	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOR Output Mixer",  "MIC1_R switch", "MIC1RP"},
//	{"LOR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"LOR Power", NULL, "LOR Output Mixer"},
	{"LOR", NULL, "LOR Power"},
	
	/* ******** Left Output ******** */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "MIC1_L switch", "MIC1LP"},
	//{"HPL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"HPL Power", NULL, "HPL Output Mixer"},
	{"HPL", NULL, "HPL Power"},

	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
	{"LOL Output Mixer", "MIC1_L switch", "MIC1LP"},
//	{"LOL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"LOL Power", NULL, "LOL Output Mixer"},
	{"LOL", NULL, "LOL Power"},


	/* ******** Left input ******** */
	{"Left Input Mixer", "MIC1_L switch", "MIC1LP"},
	//{"Left_Bypass", NULL, "Left Input Mixer"},

	//{"Left ADC", NULL, "Left Input Mixer"},
        {"ADC", NULL, "Left Input Mixer"},

	/* ******** Right Input ******** */
	{"Right Input Mixer", "MIC1_R switch", "MIC1RP"},
//	{"Right_Bypass", NULL, "Right Input Mixer"},

	//{"Right ADC", NULL, "Right Input Mixer"},
	{"ADC", NULL, "Right Input Mixer"},

	/* ******** terminator ******** */
	//{NULL, NULL, NULL},
};

#define AIC3111_DAPM_ROUTE_NUM (sizeof(aic3111_dapm_routes)/sizeof(struct snd_soc_dapm_route))

static void i2c_aic3111_headset_access_work(struct work_struct *work);
static struct work_struct works;
static struct snd_soc_codec *codec_work_var_glob;

/*
 ***************************************************************************** 
 * Function Definitions
 ***************************************************************************** 
 */
static int snd_soc_info_volsw_2r_aic3111(struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_info *uinfo)
{
    int mask = (kcontrol->private_value >> 12) & 0xff;

	DBG("snd_soc_info_volsw_2r_aic3111 (%s)\n", kcontrol->id.name);

    uinfo->type =
        mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = mask;
    return 0;
}
/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_get_volsw_2r_aic3111
 * Purpose  : Callback to get the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_get_volsw_2r_aic3111(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC3111_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC3111_8BITS_MASK;
	int mask;
	int shift;
	unsigned short val, val2;

	DBG("snd_soc_get_volsw_2r_aic3111 %s\n", kcontrol->id.name);

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		mask = AIC3111_8BITS_MASK;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		mask = 0xF;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		mask = 0x3;
		shift = 3;
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		mask = 0x7F;
		shift = 0;
	} else {
		printk(KERN_ALERT "Invalid kcontrol name\n");
		return -1;
	}

	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		ucontrol->value.integer.value[0] =
		    (val <= 48) ? (val + 127) : (val - 129);
		ucontrol->value.integer.value[1] =
		    (val2 <= 48) ? (val2 + 127) : (val2 - 129);
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		ucontrol->value.integer.value[0] =
		    (val <= 9) ? (val + 0) : (val - 15);
		ucontrol->value.integer.value[1] =
		    (val2 <= 9) ? (val2 + 0) : (val2 - 15);
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		ucontrol->value.integer.value[0] =
		    ((val/6) <= 4) ? ((val/6 -1)*6) : ((val/6 - 0)*6);
		ucontrol->value.integer.value[1] =
		    ((val2/6) <= 4) ? ((val2/6-1)*6) : ((val2/6 - 0)*6);
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		ucontrol->value.integer.value[0] =
		    ((val*2) <= 40) ? ((val*2 + 24)/2) : ((val*2 - 254)/2);
		ucontrol->value.integer.value[1] =
		    ((val2*2) <= 40) ? ((val2*2 + 24)/2) : ((val2*2 - 254)/2);
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_put_volsw_2r_aic3111
 * Purpose  : Callback to set the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_put_volsw_2r_aic3111(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC3111_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC3111_8BITS_MASK;
	int err;
	unsigned short val, val2, val_mask;

	DBG("snd_soc_put_volsw_2r_aic3111 (%s)\n", kcontrol->id.name);

	val = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
//		 (val <= 48) ? (val + 127) : (val - 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = AIC3111_8BITS_MASK;	/* 8 bits */
	} else if (!strcmp(kcontrol->id.name, "HP Driver Gain")) {
		val = (val >= 0) ? (val - 0) : (val + 15);
		val2 = (val2 >= 0) ? (val2 - 0) : (val2 + 15);
		val_mask = 0xF;	/* 4 bits */
	} else if (!strcmp(kcontrol->id.name, "LO Driver Gain")) {
		val = (val/6 >= 1) ? ((val/6 +1)*6) : ((val/6 + 0)*6);
		val2 = (val2/6 >= 1) ? ((val2/6 +1)*6) : ((val2/6 + 0)*6);
		val_mask = 0x3;	/* 2 bits */
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		val = (val*2 >= 24) ? ((val*2 - 24)/2) : ((val*2 + 254)/2);
		val2 = (val2*2 >= 24) ? ((val2*2 - 24)/2) : ((val2*2 + 254)/2);
		val_mask = 0x7F;	/* 7 bits */
	} else {
		printk(KERN_ALERT "Invalid control name\n");
		return -1;
	}

	if ((err = snd_soc_update_bits(codec, reg, val_mask, val)) < 0) {
		printk(KERN_ALERT "Error while updating bits\n");
		return err;
	}

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info
 * Purpose  : This function is to initialize data for new control required to 
 *            program the AIC3111 registers.
 *            
 *----------------------------------------------------------------------------
 */
static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{

	DBG("+ new control info\n");

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get
 * Purpose  : This function is to read data of new control for 
 *            program the AIC3111 registers.
 *            
 *----------------------------------------------------------------------------
 */
static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	DBG("+ new control get (%d)\n", aic3111_reg_ctl);

	val = aic3111_read(codec, aic3111_reg_ctl);
	ucontrol->value.integer.value[0] = val;

	DBG("+ new control get val(%d)\n", val);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 * 
 *----------------------------------------------------------------------------
 */
static int __new_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3111_priv *aic3111 = codec->private_data;

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	DBG("+ new control put (%s)\n", kcontrol->id.name);

	DBG("reg = %d val = %x\n", data[0], data[1]);

	aic3111_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0]) {
		aic3111->page_no = data[1];
	}

	DBG("reg = %d val = %x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in i2c write\n");
		return -EIO;
	}
	DBG("- new control put\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_change_page
 * Purpose  : This function is to switch between page 0 and page 1.
 *            
 *----------------------------------------------------------------------------
 */
int aic3111_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct aic3111_priv *aic3111 = codec->private_data;
	u8 data[2];

	data[0] = 0;
	data[1] = new_page;
	aic3111->page_no = new_page;
	DBG("aic3111_change_page => %d (w 30 %02x %02x)\n", new_page, data[0], data[1]);

//	DBG("w 30 %02x %02x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in changing page to %d\n", new_page);
		return -1;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_write_reg_cache
 * Purpose  : This function is to write aic3111 register cache
 *            
 *----------------------------------------------------------------------------
 */
static inline void aic3111_write_reg_cache(struct snd_soc_codec *codec,
					   u16 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= AIC3111_CACHEREGNUM) {
		return;
	}
	cache[reg] = value;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_write
 * Purpose  : This function is to write to the aic3111 register space.
 *            
 *----------------------------------------------------------------------------
 */
int aic3111_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	struct aic3111_priv *aic3111 = codec->private_data;
	u8 data[2];
	u8 page;

	page = reg / 128;
	data[AIC3111_REG_OFFSET_INDEX] = reg % 128;
//	DBG("# aic3111 write reg(%d) new_page(%d) old_page(%d) value(0x%02x)\n", reg, page, aic3111->page_no, value);


	if (aic3111->page_no != page) {
		aic3111_change_page(codec, page);
	}
	
	DBG("w 30 %02x %02x\n", data[AIC3111_REG_OFFSET_INDEX], value);
	

	/* data is
	 *   D15..D8 aic3111 register offset
	 *   D7...D0 register data
	 */
	data[AIC3111_REG_DATA_INDEX] = value & AIC3111_8BITS_MASK;
#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1)) {
		aic3111_write_reg_cache(codec, reg, value);
	}
#endif
	if (!data[AIC3111_REG_OFFSET_INDEX]) {
		/* if the write is to reg0 update aic3111->page_no */
		aic3111->page_no = value;
	}

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk(KERN_ALERT "Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_read
 * Purpose  : This function is to read the aic3111 register space.
 *            
 *----------------------------------------------------------------------------
 */
static unsigned int aic3111_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct aic3111_priv *aic3111 = codec->private_data;
	u8 value;
	u8 page = reg / 128;


	reg = reg % 128;

	DBG("r 30 %02x\n", reg);
	
	if (aic3111->page_no != page) {
		aic3111_change_page(codec, page);
	}

	i2c_master_send(codec->control_data, (char *)&reg, 1);
	i2c_master_recv(codec->control_data, &value, 1);
	return value;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_get_divs
 * Purpose  : This function is to get required divisor from the "aic3111_divs"
 *            table.
 *            
 *----------------------------------------------------------------------------
 */
static inline int aic3111_get_divs(int mclk, int rate)
{
	int i;

	DBG("+ aic3111_get_divs mclk(%d) rate(%d)\n", mclk, rate);

	for (i = 0; i < ARRAY_SIZE(aic3111_divs); i++) {
		if ((aic3111_divs[i].rate == rate)
		    && (aic3111_divs[i].mclk == mclk)) {
	DBG("%d %d %d %d %d %d %d %d %d %d\n",
	aic3111_divs[i].p_val,
	aic3111_divs[i].pll_j,
	aic3111_divs[i].pll_d,
	aic3111_divs[i].dosr,
	aic3111_divs[i].ndac,
	aic3111_divs[i].mdac,
	aic3111_divs[i].aosr,
	aic3111_divs[i].nadc,
	aic3111_divs[i].madc,
	aic3111_divs[i].blck_N);

			return i;
		}
	}
	printk(KERN_ALERT "Master clock and sample rate is not supported\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_add_controls
 * Purpose  : This function is to add non dapm kcontrols.  The different 
 *            controls are in "aic3111_snd_controls" table.
 *            The following different controls are supported
 *                # PCM Playback volume control 
 *				  # PCM Playback Volume
 *				  # HP Driver Gain
 *				  # HP DAC Playback Switch
 *				  # PGA Capture Volume
 *				  # Program Registers
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	DBG("+ aic3111_add_controls num_controls(%d)\n", ARRAY_SIZE(aic3111_snd_controls));
	for (i = 0; i < ARRAY_SIZE(aic3111_snd_controls); i++) {
		err =
		    snd_ctl_add(codec->card,
				snd_soc_cnew(&aic3111_snd_controls[i], codec,
					     NULL));
		if (err < 0) {
			printk(KERN_ALERT "Invalid control\n");
			return err;
		}
	}
	DBG("- aic3111_add_controls \n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_add_widgets
 * Purpose  : This function is to add the dapm widgets 
 *            The following are the main widgets supported
 *                # Left DAC to Left Outputs
 *                # Right DAC to Right Outputs
 *		  # Left Inputs to Left ADC
 *		  # Right Inputs to Right ADC
 *
 *----------------------------------------------------------------------------
 */
static int aic3111_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	DBG("+ aic3111_add_widgets num_widgets(%d) num_routes(%d)\n",
			ARRAY_SIZE(aic3111_dapm_widgets), AIC3111_DAPM_ROUTE_NUM);
	for (i = 0; i < ARRAY_SIZE(aic3111_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &aic3111_dapm_widgets[i]);
	}

	DBG("snd_soc_dapm_add_routes\n");

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, &aic3111_dapm_routes[0],
				AIC3111_DAPM_ROUTE_NUM);


	DBG("snd_soc_dapm_new_widgets\n");
	snd_soc_dapm_new_widgets(codec);
	DBG("- aic3111_add_widgets\n");
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_hw_params
 * Purpose  : This function is to set the hardware parameters for AIC3111.
 *            The functions set the sample rate and audio serial data word 
 *            length.
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *tmp)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct aic3111_priv *aic3111 = codec->private_data;
	int i, j;
	u8 data;

	DBG("+ SET aic3111_hw_params\n");


	aic3111_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	i = aic3111_get_divs(aic3111->sysclk, params_rate(params));
	DBG("- Sampling rate: %d, %d\n", params_rate(params), i);


	if (i < 0) {
		printk(KERN_ALERT "sampling rate not supported\n");
		return i;
	}

	if (soc_static_freq_config) {

		/* We will fix R value to 1 and will make P & J=K.D as varialble */

		/* Setting P & R values */
		aic3111_write(codec, CLK_REG_2,
			      ((aic3111_divs[i].p_val << 4) | 0x01));

		/* J value */
		aic3111_write(codec, CLK_REG_3, aic3111_divs[i].pll_j);

		/* MSB & LSB for D value */
		aic3111_write(codec, CLK_REG_4, (aic3111_divs[i].pll_d >> 8));
		aic3111_write(codec, CLK_REG_5,
			      (aic3111_divs[i].pll_d & AIC3111_8BITS_MASK));

		/* NDAC divider value */
		aic3111_write(codec, NDAC_CLK_REG_6, aic3111_divs[i].ndac);

		/* MDAC divider value */
		aic3111_write(codec, MDAC_CLK_REG_7, aic3111_divs[i].mdac);

		/* DOSR MSB & LSB values */
		aic3111_write(codec, DAC_OSR_MSB, aic3111_divs[i].dosr >> 8);
		aic3111_write(codec, DAC_OSR_LSB,
			      aic3111_divs[i].dosr & AIC3111_8BITS_MASK);

		/* NADC divider value */
		aic3111_write(codec, NADC_CLK_REG_8, aic3111_divs[i].nadc);

		/* MADC divider value */
		aic3111_write(codec, MADC_CLK_REG_9, aic3111_divs[i].madc);

		/* AOSR value */
		aic3111_write(codec, ADC_OSR_REG, aic3111_divs[i].aosr);
	}
	/* BCLK N divider */
	aic3111_write(codec, CLK_REG_11, aic3111_divs[i].blck_N);

	aic3111_set_bias_level(codec, SND_SOC_BIAS_ON);

	data = aic3111_read(codec, INTERFACE_SET_REG_1);

	data = data & ~(3 << 4);

	printk(KERN_ALERT "- Data length: %d\n", params_format(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (AIC3111_WORD_LEN_20BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (AIC3111_WORD_LEN_24BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (AIC3111_WORD_LEN_32BITS << DAC_OSR_MSB_SHIFT);
		break;
	}

	aic3111_write(codec, INTERFACE_SET_REG_1, data);


	for (j = 0; j < NO_FEATURE_REGS; j++) {
		aic3111_write(codec,
			      aic3111_divs[i].codec_specific_regs[j].reg_offset,
			      aic3111_divs[i].codec_specific_regs[j].reg_val);
	}

	DBG("- SET aic3111_hw_params\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_mute
 * Purpose  : This function is to mute or unmute the left and right DAC
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 dac_reg;

	DBG("+ aic3111_mute %d\n", mute);

	dac_reg = aic3111_read(codec, DAC_MUTE_CTRL_REG) & ~MUTE_ON;
	if (mute)
		aic3111_write(codec, DAC_MUTE_CTRL_REG, dac_reg | MUTE_ON);
	else
		aic3111_write(codec, DAC_MUTE_CTRL_REG, dac_reg);

	DBG("- aic3111_mute %d\n", mute);

#ifdef DEBUG
	DBG("++ aic3111_dump\n");
	debug_print_registers (codec);
	DBG("-- aic3111_dump\n");
#endif //DEBUG

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_set_dai_sysclk
 * Purpose  : This function is to set the DAI system clock
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3111_priv *aic3111 = codec->private_data;

	DBG("aic3111_set_dai_sysclk clk_id(%d) (%d)\n", clk_id, freq);

	switch (freq) {
	case AIC3111_FREQ_12000000:
	case AIC3111_FREQ_24000000:
	case AIC3111_FREQ_13000000:
		aic3111->sysclk = freq;
		return 0;
	}
	printk(KERN_ALERT "Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_set_dai_fmt
 * Purpose  : This function is to set the DAI format
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3111_priv *aic3111 = codec->private_data;
	u8 iface_reg;

	iface_reg = aic3111_read(codec, INTERFACE_SET_REG_1);
	iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	DBG("+ aic3111_set_dai_fmt (%x) \n", fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic3111->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic3111->master = 0;
		break;
	default:
		printk(KERN_ALERT "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (AIC3111_DSP_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (AIC3111_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (AIC3111_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	default:
		printk(KERN_ALERT "Invalid DAI interface format\n");
		return -EINVAL;
	}

	DBG("- aic3111_set_dai_fmt (%x) \n", iface_reg);
	aic3111_write(codec, INTERFACE_SET_REG_1, iface_reg);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_set_bias_level
 * Purpose  : This function is to get triggered when dapm events occurs.
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct aic3111_priv *aic3111 = codec->private_data;
	u8 value;

	DBG("++ aic3111_set_bias_level\n");

	if (level == codec->bias_level)
		return 0;

	switch (level) {
		/* full On */
	case SND_SOC_BIAS_ON:
		DBG("aic3111_set_bias_level ON\n");
		/* all power is driven by DAPM system */
		if (aic3111->master) {
			/* Switch on PLL */
			value = aic3111_read(codec, CLK_REG_2);
			aic3111_write(codec, CLK_REG_2, (value | ENABLE_PLL));

			/* Switch on NDAC Divider */
			value = aic3111_read(codec, NDAC_CLK_REG_6);
			aic3111_write(codec, NDAC_CLK_REG_6,
				      value | ENABLE_NDAC);

			/* Switch on MDAC Divider */
			value = aic3111_read(codec, MDAC_CLK_REG_7);
			aic3111_write(codec, MDAC_CLK_REG_7,
				      value | ENABLE_MDAC);

			/* Switch on NADC Divider */
			value = aic3111_read(codec, NADC_CLK_REG_8);
			aic3111_write(codec, NADC_CLK_REG_8,
				      value | ENABLE_MDAC);

			/* Switch on MADC Divider */
			value = aic3111_read(codec, MADC_CLK_REG_9);
			aic3111_write(codec, MADC_CLK_REG_9,
				      value | ENABLE_MDAC);

			/* Switch on BCLK_N Divider */
			value = aic3111_read(codec, CLK_REG_11);
			aic3111_write(codec, CLK_REG_11, value | ENABLE_BCLK);
		}
		break;

		/* partial On */
	case SND_SOC_BIAS_PREPARE:
		break;

		/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		/*
		 * all power is driven by DAPM system,
		 * so output power is safe if bypass was set
		 */
		 DBG("aic3111_set_bias_level STANDBY\n");
		if (aic3111->master) {
			/* Switch off PLL */
			value = aic3111_read(codec, CLK_REG_2);
			aic3111_write(codec, CLK_REG_2, (value & ~ENABLE_PLL));

			/* Switch off NDAC Divider */
			value = aic3111_read(codec, NDAC_CLK_REG_6);
			aic3111_write(codec, NDAC_CLK_REG_6,
				      value & ~ENABLE_NDAC);

			/* Switch off MDAC Divider */
			value = aic3111_read(codec, MDAC_CLK_REG_7);
			aic3111_write(codec, MDAC_CLK_REG_7,
				      value & ~ENABLE_MDAC);

			/* Switch off NADC Divider */
			value = aic3111_read(codec, NADC_CLK_REG_8);
			aic3111_write(codec, NADC_CLK_REG_8,
				      value & ~ENABLE_NDAC);

			/* Switch off MADC Divider */
			value = aic3111_read(codec, MADC_CLK_REG_9);
			aic3111_write(codec, MADC_CLK_REG_9,
				      value & ~ENABLE_MDAC);
			value = aic3111_read(codec, CLK_REG_11);

			/* Switch off BCLK_N Divider */
			aic3111_write(codec, CLK_REG_11, value & ~ENABLE_BCLK);
		}
		break;

		/* Off, without power */
	case SND_SOC_BIAS_OFF:
		/* force all power off */
		break;
	}
	codec->bias_level = level;
	DBG("-- aic3111_set_bias_level\n");

	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : aic3111_suspend
 * Purpose  : This function is to suspend the AIC3111 driver.
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	DBG("+ aic3111_suspend\n");

	aic3111_set_bias_level(codec, SND_SOC_BIAS_OFF);
	DBG("-aic3111_suspend\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_resume
 * Purpose  : This function is to resume the AIC3111 driver
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u8 *cache = codec->reg_cache;

	DBG("+ aic3111_resume\n");

	aic3111_change_page(codec, 0);
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(aic3111_reg); i++) {
		aic3111_write(codec, i, cache[i]);
	}
	aic3111_change_page(codec, 0);
	aic3111_set_bias_level(codec, codec->suspend_bias_level);

	DBG("- aic3111_resume\n");

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_init
 * Purpose  : This function is to initialise the AIC3111 driver
 *            register the mixer and dsp interfaces with the kernel.
 *            
 *----------------------------------------------------------------------------
 */
 #define TRITON_AUDIO_IF_PAGE 	0x1
static int tlv320aic3111_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	struct aic3111_priv *aic3111 = codec->private_data;
	int ret = 0;
	int i = 0;
	int hph_detect_gpio = AUDIO_CODEC_HPH_DETECT_GPIO;
	int hph_detect_irq = 0;

	printk(KERN_ALERT "+tlv320aic3111_init\n");

	ret = gpio_request(hph_detect_gpio, "AIC3111-headset");
	if (ret < 0) {
		goto err1;
	}
	gpio_direction_input(hph_detect_gpio);
	omap_set_gpio_debounce(hph_detect_gpio,1);
	omap_set_gpio_debounce_time(hph_detect_gpio,0xFF);
	hph_detect_irq = OMAP_GPIO_IRQ(hph_detect_gpio);

	codec->name = "aic3111";
	codec->owner = THIS_MODULE;
	codec->read = aic3111_read;
	codec->write = aic3111_write;
	//codec->set_bias_level = aic3111_set_bias_level;
	codec->dai = &tlv320aic3111_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(aic3111_reg);
	codec->reg_cache =
	    kmemdup(aic3111_reg, sizeof(aic3111_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		printk(KERN_ERR "aic3111: kmemdup failed\n");
		return -ENOMEM;
	}

	aic3111->page_no = 0;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "aic3111: failed to create pcms\n");
		goto pcm_err;
	}

	printk(KERN_ALERT "*** Configuring AIC3111 registers ***\n");
	for (i = 0; i < sizeof(aic3111_reg_init) / sizeof(struct aic3111_configs); i++) {
		aic3111_write(codec, aic3111_reg_init[i].reg_offset, aic3111_reg_init[i].reg_val);
	}
	aic3111_headset_speaker_path(codec);
	printk(KERN_ALERT "*** Done Configuring AIC3111 registers ***\n");

#if !(defined(CONFIG_MACH_OMAP3621_EDP1) || defined(CONFIG_MACH_OMAP3621_BOXER))
//no TWL on OMAP3621

// *********************************************************
//@@GT: Patch needed on EDP1 - Set Triton audio I/F as slave[START]
// Otherwise Triton audio I/F is driven low and blocks audio I/F btw McBSP2 & AIC3111

	// set triton codec interface to 0 (CODEC_MODE = 0)
	twl4030_i2c_write_u8(TRITON_AUDIO_IF_PAGE, 0, 0x00);

	// set triton in slave mode, enable audio_if
	twl4030_i2c_write_u8(TRITON_AUDIO_IF_PAGE, 0x85, 0x0E);
	
	// enable codec interface to 0 (CODEC_MODE = 1)
	twl4030_i2c_write_u8(TRITON_AUDIO_IF_PAGE, 0x2, 0x00);
//@@GT Patch needed on EDP1 - Set Triton audio I/F as slave[END]
// *********************************************************
#endif

	ret = request_irq(hph_detect_irq, aic3111_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_DISABLED | IRQF_SHARED , "aic3111", codec);

	/* off, with power on */
	aic3111_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	aic3111_add_controls(codec);
	aic3111_add_widgets(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "aic3111: failed to register card\n");
		goto card_err;
	}

	  printk(KERN_ALERT "-tlv320aic3111_init\n");

	return ret;

err1:
	free_irq(hph_detect_irq, codec);

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

static int aic3111_headset_speaker_path(struct snd_soc_codec *codec)
{
	int headset_detect_gpio = AUDIO_CODEC_HPH_DETECT_GPIO;
	int headset_present = 0;

	headset_present = gpio_get_value(headset_detect_gpio);
#if defined(CONFIG_MACH_OMAP3621_EDP1) || defined(CONFIG_MACH_OMAP3630_EDP1) \
     || defined(CONFIG_MACH_OMAP3430_EDP1) || defined(CONFIG_MACH_OMAP3621_BOXER)

  #if defined(CONFIG_MACH_OMAP3621_BOXER)
	if(!headset_present) {
  #else
	if(headset_present) {
  #endif
		// Header
		printk(KERN_DEBUG "headset present and headset path activate\n");
		aic3111_write(codec, HPHONE_DRIVERS, 0xC4); // ON
		aic3111_write(codec, CLASS_D_SPK, 0x06); // OFF
	} else {
		//SPK
		printk(KERN_DEBUG "headset removed and headset path desactivate\n");
		aic3111_write(codec, HPHONE_DRIVERS ,0x4); // OFF
		aic3111_write(codec, CLASS_D_SPK ,0xC6 ); //ON
	}
#else
#error "Please check headset detection"
#endif
	return 0;
}

/*
 * This interrupt is called when HEADSET is insert or remove from conector.
   On this interup sound will be rouote to HEadset or speaker path.
 */
static irqreturn_t aic3111_irq_handler(int irq, struct snd_soc_codec *codec)
{
	printk(KERN_DEBUG "interrupt of headset found\n");
	//disable_irq(irq);
	codec_work_var_glob = codec;
	schedule_work(&works);
	//enable_irq(irq);
	return IRQ_HANDLED;
}

static void i2c_aic3111_headset_access_work(struct work_struct *work)
{
	aic3111_headset_speaker_path(codec_work_var_glob);
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 *----------------------------------------------------------------------------
 * Function : aic3111_codec_probe
 * Purpose  : This function attaches the i2c client and initializes 
 *				AIC3111 CODEC.
 *            NOTE:
 *            This function is called from i2c core when the I2C address is
 *            valid.
 *            If the i2c layer weren't so broken, we could pass this kind of 
 *            data around
 *            
 *----------------------------------------------------------------------------
 */
static int tlv320aic3111_codec_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = aic3111_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = tlv320aic3111_init(socdev);
	INIT_WORK(&works, i2c_aic3111_headset_access_work);
	if (ret < 0) {
		printk(KERN_ERR "aic3111: failed to attach codec at addr\n");
		return -1;
	}
#ifdef CONFIG_MINI_DSP
	/* Program MINI DSP for ADC and DAC */
	aic3111_minidsp_program(codec);
	aic3111_change_page(codec, 0x0);
#endif

	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : tlv320aic3007_i2c_remove
 * Purpose  : This function removes the i2c client and uninitializes 
 *                              AIC3007 CODEC.
 *            NOTE:
 *            This function is called from i2c core 
 *            If the i2c layer weren't so broken, we could pass this kind of 
 *            data around
 *            
 *----------------------------------------------------------------------------
 */

static int __exit tlv320aic3111_i2c_remove(struct i2c_client *i2c)
{
        put_device(&i2c->dev);
        return 0;
}

static const struct i2c_device_id tlv320aic3111_id[] = {
        {"tlv320aic3111", 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, tlv320aic3111_id);

static struct i2c_driver tlv320aic3111_i2c_driver = {
	.driver = {
		.name = "tlv320aic3111",
	},
	.probe = tlv320aic3111_codec_probe,
	.remove = __exit_p(tlv320aic3111_i2c_remove),
	.id_table = tlv320aic3111_id,
};

#endif //#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_probe
 * Purpose  : This is first driver function called by the SoC core driver.
 *            
 *----------------------------------------------------------------------------
 */

static int aic3111_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct aic3111_priv *aic3111;
	int ret = 0;
	
	int gpio = AUDIO_CODEC_PWR_ON_GPIO;

	ret = gpio_request(gpio, AUDIO_CODEC_PWR_ON_GPIO_NAME);
	gpio_direction_output(gpio, 0);
	gpio_set_value(gpio, 1);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if(codec == NULL)
		return -ENOMEM;

	aic3111 = kzalloc(sizeof(struct aic3111_priv), GFP_KERNEL);
	if (aic3111 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = aic3111;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	aic3111_socdev = socdev;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
		codec->hw_write = (hw_write_t) i2c_master_send;
		codec->hw_read = (hw_read_t) i2c_master_recv;
		ret = i2c_add_driver(&tlv320aic3111_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "Can't add TLV320AIC3111 i2c driver!!");
#else
	/* Add other interfaces here */
#endif

	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_remove
 * Purpose  : to remove aic3111 soc device 
 *            
 *----------------------------------------------------------------------------
 */
static int aic3111_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	/* power down chip */
	if (codec->control_data)
		aic3111_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&tlv320aic3111_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_device |
 *          This structure is soc audio codec device sturecute which pointer
 *          to basic functions aic3111_probe(), aic3111_remove(),  
 *          aic3111_suspend() and aic3111_resume()
 *
 */
struct snd_soc_codec_device soc_codec_dev_aic3111 = {
	.probe = aic3111_probe,
	.remove = aic3111_remove,
	.suspend = aic3111_suspend,
	.resume = aic3111_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_aic3111);

static int __init tlv320aic3111_modinit(void)
{
	return snd_soc_register_dai(&tlv320aic3111_dai);
}

module_init(tlv320aic3111_modinit);

static void __exit tlv320aic3111_exit(void)
{
	snd_soc_unregister_dai(&tlv320aic3111_dai);
}

module_exit(tlv320aic3111_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3111 codec driver");
MODULE_AUTHOR("sandeepsp@mistralsolutions.com");
MODULE_LICENSE("GPL");
