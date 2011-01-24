/*
 * linux/sound/soc/codecs/tlv320aic3254.c
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
 * Rev 0.1   ASoC driver support    Mistral         31-04-2009
 *   
 *			 The AIC32 ASoC driver is ported for the codec AIC3254.
 *     
 *
 * Rev 1.0   Mini DSP support    Mistral         11-05-2009
 *
 *			 Added mini DSP programming support
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

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "tlv320aic3254.h"

/*
 ***************************************************************************** 
 * Macros
 ***************************************************************************** 
 */

#ifdef CONFIG_MINI_DSP
extern int aic3254_minidsp_program(struct snd_soc_codec *codec);
extern void aic3254_add_minidsp_controls(struct snd_soc_codec *codec);
#endif

#define SOC_SINGLE_AIC3254(xname) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = __new_control_info, .get = __new_control_get,\
	.put = __new_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
}



#define SOC_DOUBLE_R_AIC3254(xname, reg_left, reg_right, shift, mask, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw_2r_aic3254, \
	.get = snd_soc_get_volsw_2r_aic3254, .put = snd_soc_put_volsw_2r_aic3254, \
	.private_value = (reg_left) | ((shift) << 8)  | \
		((mask) << 12) | ((invert) << 20) | ((reg_right) << 24) }

/*
 ***************************************************************************** 
 * Function Prototype
 ***************************************************************************** 
 */
static int aic3254_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params);

static int aic3254_mute(struct snd_soc_dai *dai, int mute);

static int aic3254_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir);

static int aic3254_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);

static int aic3254_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level);

static u8 aic3254_read(struct snd_soc_codec *codec, u16 reg);

static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);

static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int __new_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);

static int snd_soc_info_volsw_2r_aic3254(struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_info *uinfo);

static int snd_soc_get_volsw_2r_aic3254(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

static int snd_soc_put_volsw_2r_aic3254(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

/*
 ***************************************************************************** 
 * Global Variable
 ***************************************************************************** 
 */
static u8 aic3254_reg_ctl;

/* whenever aplay/arecord is run, aic3254_hw_params() function gets called. 
 * This function reprograms the clock dividers etc. this flag can be used to 
 * disable this when the clock dividers are programmed by pps config file
 */
static int soc_static_freq_config = 1;

/*
 ***************************************************************************** 
 * Structure Declaration
 ***************************************************************************** 
 */
static struct snd_soc_device *aic3254_socdev;

/*
 ***************************************************************************** 
 * Structure Initialization
 ***************************************************************************** 
 */
static const struct snd_kcontrol_new aic3254_snd_controls[] = {
	/* Output */
	/* sound new kcontrol for PCM Playback volume control */
	SOC_DOUBLE_R_AIC3254("PCM Playback Volume", LDAC_VOL, RDAC_VOL, 0, 0xAf,
			     0),
	/* sound new kcontrol for HP driver gain */
	SOC_DOUBLE_R_AIC3254("HP Driver Gain", HPL_GAIN, HPR_GAIN, 0, 0x23, 0),
	/* sound new kcontrol for LO driver gain */
	SOC_DOUBLE_R_AIC3254("LO Driver Gain", LOL_GAIN, LOR_GAIN, 0, 0x23, 0),
	/* sound new kcontrol for HP mute */
	SOC_DOUBLE_R("HP DAC Playback Switch", HPL_GAIN, HPR_GAIN, 6,
		     0x01, 1),
	/* sound new kcontrol for LO mute */
	SOC_DOUBLE_R("LO DAC Playback Switch", LOL_GAIN, LOR_GAIN, 6,
		     0x01, 1),

	/* Input */
	/* sound new kcontrol for PGA capture volume */
	SOC_DOUBLE_R_AIC3254("PGA Capture Volume", LADC_VOL, RADC_VOL, 0, 0x3F,
			     0),

	/* sound new kcontrol for Programming the registers from user space */
	SOC_SINGLE_AIC3254("Program Registers"),

};

/* the sturcture contains the different values for mclk */
static const struct aic3254_rate_divs aic3254_divs[] = {
/* 
 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc, blck_N, 
 * codec_speficic_initializations 
 */
	/* 8k rate */
	{12000000, 8000, 1, 7, 6800, 768, 5, 3, 128, 5, 18, 24,
	 {{60, 1}, {61, 1}}},
	{24000000, 8000, 2, 7, 6800, 768, 15, 1, 64, 45, 4, 24,
	 {{60, 1}, {61, 1}}},
	/* 11.025k rate */
	{12000000, 11025, 1, 7, 5264, 512, 8, 2, 128, 8, 8, 16,
	 {{60, 1}, {61, 1}}},
	{24000000, 11025, 2, 7, 5264, 512, 16, 1, 64, 32, 4, 16,
	 {{60, 1}, {61, 1}}},
	/* 16k rate */
	{12000000, 16000, 1, 7, 6800, 384, 5, 3, 128, 5, 9, 12,
	 {{60, 1}, {61, 1}}},
	{24000000, 16000, 2, 7, 6800, 384, 15, 1, 64, 18, 5, 12,
	 {{60, 1}, {61, 1}}},
	/* 22.05k rate */
	{12000000, 22050, 1, 7, 5264, 256, 4, 4, 128, 4, 8, 8,
	 {{60, 1}, {61, 1}}},
	{24000000, 22050, 2, 7, 5264, 256, 16, 1, 64, 16, 4, 8,
	 {{60, 1}, {61, 1}}},
	/* 32k rate */
	{12000000, 32000, 1, 7, 1680, 192, 2, 7, 64, 2, 21, 6,
	 {{60, 1}, {61, 1}}},
	{24000000, 32000, 2, 7, 1680, 192, 7, 2, 64, 7, 6, 6,
	 {{60, 1}, {61, 1}}},
	/* 44.1k rate */
#ifdef  CONFIG_MINI_DSP
	{12000000, 44100, 1, 7, 5264, 128, 2, 8, 128, 2, 8, 4,
	 {}},
	{24000000, 44100, 2, 7, 5264, 128, 8, 2, 64, 8, 4, 4,
	 {}},
#else
	{12000000, 44100, 1, 7, 5264, 128, 2, 8, 128, 2, 8, 4,
	 {{60, 1}, {61, 1}}},
	{24000000, 44100, 2, 7, 5264, 128, 8, 2, 64, 8, 4, 4,
	 {{60, 1}, {61, 1}}},
#endif
	/* 48k rate */
	{12000000, 48000, 1, 8, 1920, 128, 2, 8, 128, 2, 8, 4,
	 {{60, 1}, {61, 1}}},
	{24000000, 48000, 2, 8, 1920, 128, 8, 2, 64, 8, 4, 4,
	 {{60, 1}, {61, 1}}},
	/*96k rate */
	{12000000, 96000, 1, 8, 1920, 64, 2, 8, 64, 2, 8, 2,
	 {{60, 7}, {61, 7}}},
	{24000000, 96000, 2, 8, 1920, 64, 4, 4, 64, 8, 2, 2,
	 {{60, 7}, {61, 7}}},
	/*192k */
	{12000000, 192000, 1, 8, 1920, 32, 2, 8, 32, 2, 8, 1,
	 {{60, 17}, {61, 13}}},
	{24000000, 192000, 2, 8, 1920, 32, 4, 4, 32, 4, 4, 1,
	 {{60, 17}, {61, 13}}},
};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *          It is SoC Codec DAI structure which has DAI capabilities viz., 
 *          playback and capture, DAI runtime information viz. state of DAI 
 *			and pop wait state, and DAI private data. 
 *          The AIC3254 rates ranges from 8k to 192k
 *          The PCM bit format supported are 16, 20, 24 and 32 bits
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai tlv320aic3254_dai = {
	.name = "aic3254",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = AIC3254_RATES,
		     .formats = AIC3254_FORMATS,},
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = AIC3254_RATES,
		    .formats = AIC3254_FORMATS,},
	.ops = {
		.hw_params = aic3254_hw_params,
		.digital_mute = aic3254_mute,
		.set_sysclk = aic3254_set_dai_sysclk,
		.set_fmt = aic3254_set_dai_fmt,
		}
};

EXPORT_SYMBOL_GPL(tlv320aic3254_dai);

/*
 ***************************************************************************** 
 * Initializations
 ***************************************************************************** 
 */
/*
 * AIC3254 register cache
 * We are caching the registers here.
 * There is no point in caching the reset register.
 * NOTE: In AIC32, there are 127 registers supported in both page0 and page1
 *       The following table contains the page0 and page 1registers values.
 */
static const u8 aic3254_reg[AIC3254_CACHEREGNUM] = {
	0x00, 0x00, 0x50, 0x00,	/* 0 */
	0x00, 0x11, 0x04, 0x00,	/* 4 */
	0x00, 0x00, 0x00, 0x01,	/* 8 */
	0x01, 0x00, 0x80, 0x02,	/* 12 */
	0x00, 0x08, 0x01, 0x01,	/* 16 */
	0x80, 0x01, 0x00, 0x04,	/* 20 */
	0x00, 0x00, 0x01, 0x00,	/* 24 */
	0x00, 0x00, 0x01, 0x00,	/* 28 */
	0x00, 0x00, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x00, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x00,	/* 40 */
	0x00, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
	0x00, 0x42, 0x02, 0x02,	/* 52 */
	0x42, 0x02, 0x02, 0x02,	/* 56 */
	0x00, 0x00, 0x00, 0x01,	/* 60 */
	0x01, 0x00, 0x14, 0x00,	/* 64 */
	0x0C, 0x00, 0x00, 0x00,	/* 68 */
	0x00, 0x00, 0x00, 0xEE,	/* 72 */
	0x10, 0xD8, 0x10, 0xD8,	/* 76 */
	0x00, 0x00, 0x88, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
	0x7F, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x7F, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x00, 0x00,	/* 100 */
	0x00, 0x00, 0x00, 0x00,	/* 104 */
	0x00, 0x00, 0x00, 0x00,	/* 108 */
	0x00, 0x00, 0x00, 0x00,	/* 112 */
	0x00, 0x00, 0x00, 0x00,	/* 116 */
	0x00, 0x00, 0x00, 0x00,	/* 120 */
	0x00, 0x00, 0x00, 0x00,	/* 124 - PAGE0 Registers(127) ends here */
	0x01, 0x00, 0x08, 0x00,	/* 128, PAGE1-0 */
	0x00, 0x00, 0x00, 0x00,	/* 132, PAGE1-4 */
	0x00, 0x00, 0x00, 0x10,	/* 136, PAGE1-8 */
	0x00, 0x00, 0x00, 0x00,	/* 140, PAGE1-12 */
	0x40, 0x40, 0x40, 0x40,	/* 144, PAGE1-16 */
	0x00, 0x00, 0x00, 0x00,	/* 148, PAGE1-20 */
	0x00, 0x00, 0x00, 0x00,	/* 152, PAGE1-24 */
	0x00, 0x00, 0x00, 0x00,	/* 156, PAGE1-28 */
	0x00, 0x00, 0x00, 0x00,	/* 160, PAGE1-32 */
	0x00, 0x00, 0x00, 0x00,	/* 164, PAGE1-36 */
	0x00, 0x00, 0x00, 0x00,	/* 168, PAGE1-40 */
	0x00, 0x00, 0x00, 0x00,	/* 172, PAGE1-44 */
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
 * aic3254 initialization data 
 * This structure initialization contains the initialization required for
 * AIC3254.
 * These registers values (reg_val) are written into the respective AIC3254 
 * register offset (reg_offset) to  initialize AIC3254. 
 * These values are used in aic3254_init() function only. 
 */
static const struct aic3254_configs aic3254_reg_init[] = {
	/* Carry out the software reset */
	{RESET, 0x01},
	/* Disable crude LDO */
	{POW_CFG, 0x00},
	/* Switch on the analog blocks */
	{LDO_CTL, 0x00},
	/* Connect IN1_L and IN1_R to CM */
	{INPUT_CFG_REG, 0xc0},
	/* PLL is CODEC_CLKIN */
	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
	/* DAC_MOD_CLK is BCLK source */
	{AIS_REG_3, DAC_MOD_CLK_2_BDIV_CLKIN},
	/* Setting up DAC Channel */
	{DAC_CHN_REG,
	 LDAC_2_LCHN | RDAC_2_RCHN | SOFT_STEP_2WCLK},
	/* Headphone powerup */
	{HPHONE_STARTUP_CTRL, 0x35},
	/* Left Channel DAC recons filter's positive terminal is routed to HPL */
	{HPL_ROUTE_CTL, LDAC_CHNL_2_HPL},
	/* Right Channel DAC recons filter's positive terminal is routed to HPR */
	{HPR_ROUTE_CTL, RDAC_CHNL_2_HPR},
	/* Left Channel DAC recons filter's positive terminal is routed to LOL */
	{LOL_ROUTE_CTL, LDAC_CHNL_2_HPL},
	/* Right Channel DAC recons filter's positive terminal is routed to LOR */
	{LOR_ROUTE_CTL, RDAC_CHNL_2_HPR},
	/* HPL unmute and gain 0db */
	{HPL_GAIN, 0x0},
	/* HPR unmute and gain 0db */
	{HPR_GAIN, 0x0},
	/* LOL unmute and gain 0db */
	{LOL_GAIN, 0x0},
	/* LOR unmute and gain 0db */
	{LOR_GAIN, 0x0},
	/* Unmute DAC Left and Right channels */
	{DAC_MUTE_CTRL_REG, 0x00},
	/* IN1_L is selected for left P */
	{LMICPGA_PIN_CFG, 0x40},
	/* CM1 is selected for left M */
	{LMICPGA_NIN_CFG, 0x40},
	/* IN1_R is selected for right P */
	{RMICPGA_PIN_CFG, 0x40},
	/* CM1 is selected for right M */
	{RMICPGA_NIN_CFG, 0x40},
	/* Left mic PGA unmuted */
	{LMICPGA_VOL_CTRL, 0x00},
	/* Right mic PGA unmuted */
	{RMICPGA_VOL_CTRL, 0x00},
	/* ADC volume control change by 2 gain step per ADC Word Clock */
	{ADC_REG_1, 0x02},
	/* Unmute ADC left and right channels */
	{ADC_FGA, 0x00},
};

/* Left DAC_L Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", HPL_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_L switch", HPL_ROUTE_CTL, 2, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", HPR_ROUTE_CTL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_R switch", HPR_ROUTE_CTL, 2, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", HPR_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", LOL_ROUTE_CTL, 3, 1, 0),
//	SOC_DAPM_SINGLE("Left_Bypass switch", HPL_ROUTE_CTL, 1, 1, 0),
};

static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", LOR_ROUTE_CTL, 3, 1, 0),
//	SOC_DAPM_SINGLE("Right_Bypass switch", LOR_ROUTE_CTL, 1, 1, 0),
};

/* Right DAC_R Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_L switch", LMICPGA_PIN_CFG, 6, 1, 0),
	SOC_DAPM_SINGLE("IN2_L switch", LMICPGA_PIN_CFG, 4, 1, 0),
	SOC_DAPM_SINGLE("IN3_L switch", LMICPGA_PIN_CFG, 2, 1, 0),
};

static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_R switch", RMICPGA_PIN_CFG, 6, 1, 0),
	SOC_DAPM_SINGLE("IN2_R switch", RMICPGA_PIN_CFG, 4, 1, 0),
	SOC_DAPM_SINGLE("IN3_R switch", RMICPGA_PIN_CFG, 2, 1, 0),
};

static const struct snd_soc_dapm_widget aic3254_dapm_widgets[] = {
	/* Left DAC to Left Outputs */
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_CHN_REG, 7, 0),
	/* dapm widget (path domain) for left DAC_L Mixer */
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpl_output_mixer_controls[0],
			   ARRAY_SIZE(hpl_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPL Power", OUT_PWR_CTL, 5, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lol_output_mixer_controls[0],
			   ARRAY_SIZE(lol_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOL Power", OUT_PWR_CTL, 3, 0, NULL, 0),

	/* Right DAC to Right Outputs */
	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_CHN_REG, 6, 0),
	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &hpr_output_mixer_controls[0],
			   ARRAY_SIZE(hpr_output_mixer_controls)),
	SND_SOC_DAPM_PGA("HPR Power", OUT_PWR_CTL, 4, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			   &lor_output_mixer_controls[0],
			   ARRAY_SIZE(lor_output_mixer_controls)),
	SND_SOC_DAPM_PGA("LOR Power", OUT_PWR_CTL, 2, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			   &left_input_mixer_controls[0],
			   ARRAY_SIZE(left_input_mixer_controls)),

	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			   &right_input_mixer_controls[0],
			   ARRAY_SIZE(right_input_mixer_controls)),

	/* Left Inputs to Left ADC */
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", ADC_REG_1, 7, 0),

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

	/* dapm widget (platform domain) name for LINE1L */
	SND_SOC_DAPM_INPUT("IN1_L"),
	/* dapm widget (platform domain) name for LINE1R */
	SND_SOC_DAPM_INPUT("IN1_R"),
	/* dapm widget (platform domain) name for LINE2L */
	SND_SOC_DAPM_INPUT("IN2_L"),
	/* dapm widget (platform domain) name for LINE2R */
	SND_SOC_DAPM_INPUT("IN2_R"),
	/* dapm widget (platform domain) name for LINE3L */
	SND_SOC_DAPM_INPUT("IN3_L"),
	/* dapm widget (platform domain) name for LINE3R */
	SND_SOC_DAPM_INPUT("IN3_R"),
};

static const struct snd_soc_dapm_route aic3254_dapm_routes[] = {
	/* ******** Left Output ******** */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "IN1_L switch", "IN1_L"},
	//{"HPL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"HPL Power", NULL, "HPL Output Mixer"},
	{"HPL", NULL, "HPL Power"},

	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
//	{"LOL Output Mixer", "Left_Bypass switch", "Left_Bypass"},

	{"LOL Power", NULL, "LOL Output Mixer"},
	{"LOL", NULL, "LOL Power"},

	/* ******** Right Output ******** */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer", "IN1_R switch", "IN1_R"},
	//{"HPR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"HPR Power", NULL, "HPR Output Mixer"},
	{"HPR", NULL, "HPR Power"},

	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
//	{"LOR Output Mixer", "Right_Bypass switch", "Right_Bypass"},

	{"LOR Power", NULL, "LOR Output Mixer"},
	{"LOR", NULL, "LOR Power"},

	/* ******** Left input ******** */
	{"Left Input Mixer", "IN1_L switch", "IN1_L"},
	{"Left Input Mixer", "IN2_L switch", "IN2_L"},
	{"Left Input Mixer", "IN3_L switch", "IN3_L"},

	//{"Left_Bypass", NULL, "Left Input Mixer"},

	{"Left ADC", NULL, "Left Input Mixer"},

	/* ******** Right Input ******** */
	{"Right Input Mixer", "IN1_R switch", "IN1_R"},
	{"Right Input Mixer", "IN2_R switch", "IN2_R"},
	{"Right Input Mixer", "IN3_R switch", "IN3_R"},

//	{"Right_Bypass", NULL, "Right Input Mixer"},

	{"Right ADC", NULL, "Right Input Mixer"},

	/* ******** terminator ******** */
	//{NULL, NULL, NULL},
};

#define AIC3254_DAPM_ROUTE_NUM (sizeof(aic3254_dapm_routes)/sizeof(struct snd_soc_dapm_route))

/*
 ***************************************************************************** 
 * Function Definitions
 ***************************************************************************** 
 */
static int snd_soc_info_volsw_2r_aic3254(struct snd_kcontrol *kcontrol,
    struct snd_ctl_elem_info *uinfo)
{
    int mask = (kcontrol->private_value >> 12) & 0xff;

    uinfo->type =
        mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = mask;
    return 0;
}
/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_get_volsw_2r_aic3254
 * Purpose  : Callback to get the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_get_volsw_2r_aic3254(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC3254_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC3254_8BITS_MASK;
	int mask;
	int shift;
	unsigned short val, val2;

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		mask = AIC3254_8BITS_MASK;
		shift = 0;
	} else if ((!strcmp(kcontrol->id.name, "HP Driver Gain")) ||
		   (!strcmp(kcontrol->id.name, "LO Driver Gain"))) {
		mask = 0x3F;
		shift = 0;
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		mask = 0x7F;
		shift = 0;
	} else {
		printk("Invalid kcontrol name\n");
		return -1;
	}

	val = (snd_soc_read(codec, reg) >> shift) & mask;
	val2 = (snd_soc_read(codec, reg2) >> shift) & mask;

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		ucontrol->value.integer.value[0] =
		    (val <= 48) ? (val + 127) : (val - 129);
		ucontrol->value.integer.value[1] =
		    (val2 <= 48) ? (val2 + 127) : (val2 - 129);
	} else if ((!strcmp(kcontrol->id.name, "HP Driver Gain"))
		   || (!strcmp(kcontrol->id.name, "LO Driver Gain"))) {
		ucontrol->value.integer.value[0] =
		    (val <= 29) ? (val + 6) : (val - 58);
		ucontrol->value.integer.value[1] =
		    (val2 <= 29) ? (val2 + 6) : (val2 - 58);
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		ucontrol->value.integer.value[0] =
		    (val <= 38) ? (val + 25) : (val - 103);
		ucontrol->value.integer.value[1] =
		    (val2 <= 38) ? (val2 + 25) : (val2 - 103);
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : snd_soc_put_volsw_2r_aic3254
 * Purpose  : Callback to set the value of a double mixer control that spans
 *            two registers.
 *
 *----------------------------------------------------------------------------
 */
int snd_soc_put_volsw_2r_aic3254(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & AIC3254_8BITS_MASK;
	int reg2 = (kcontrol->private_value >> 24) & AIC3254_8BITS_MASK;
	int err;
	unsigned short val, val2, val_mask;

	val = ucontrol->value.integer.value[0];
	val2 = ucontrol->value.integer.value[1];

	if (!strcmp(kcontrol->id.name, "PCM Playback Volume")) {
		val = (val >= 127) ? (val - 127) : (val + 129);
		val2 = (val2 >= 127) ? (val2 - 127) : (val2 + 129);
		val_mask = AIC3254_8BITS_MASK;	/* 8 bits */
	} else if ((!strcmp(kcontrol->id.name, "HP Driver Gain")) ||
		   (!strcmp(kcontrol->id.name, "LO Driver Gain"))) {
		val = (val >= 6) ? (val - 6) : (val + 58);
		val2 = (val2 >= 6) ? (val2 - 6) : (val2 + 58);
		val_mask = 0x3F;	/* 6 bits */
	} else if (!strcmp(kcontrol->id.name, "PGA Capture Volume")) {
		val = (val >= 25) ? (val - 25) : (val + 103);
		val2 = (val2 >= 25) ? (val2 - 25) : (val2 + 103);
		val_mask = 0x7F;	/* 7 bits */
	} else {
		printk("Invalid control name\n");
		return -1;
	}

	if ((err = snd_soc_update_bits(codec, reg, val_mask, val)) < 0) {
		printk("Error while updating bits\n");
		return err;
	}

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info
 * Purpose  : This function is to initialize data for new control required to 
 *            program the AIC3254 registers.
 *            
 *----------------------------------------------------------------------------
 */
static int __new_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
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
 *            program the AIC3254 registers.
 *            
 *----------------------------------------------------------------------------
 */
static int __new_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	val = aic3254_read(codec, aic3254_reg_ctl);
	ucontrol->value.integer.value[0] = val;

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
	struct aic3254_priv *aic3254 = codec->private_data;

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];

	aic3254_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0]) {
		aic3254->page_no = data[1];
	}

	printk("reg = %d val = %x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_change_page
 * Purpose  : This function is to switch between page 0 and page 1.
 *            
 *----------------------------------------------------------------------------
 */
int aic3254_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct aic3254_priv *aic3254 = codec->private_data;
	u8 data[2];

	data[0] = 0;
	data[1] = new_page;
	aic3254->page_no = new_page;

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in changing page to 1\n");
		return -1;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_write_reg_cache
 * Purpose  : This function is to write aic3254 register cache
 *            
 *----------------------------------------------------------------------------
 */
static inline void aic3254_write_reg_cache(struct snd_soc_codec *codec,
					   u16 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= AIC3254_CACHEREGNUM) {
		return;
	}
	cache[reg] = value;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_write
 * Purpose  : This function is to write to the aic3254 register space.
 *            
 *----------------------------------------------------------------------------
 */
int aic3254_write(struct snd_soc_codec *codec, u16 reg, u8 value)
{
	struct aic3254_priv *aic3254 = codec->private_data;
	u8 data[2];
	u8 page;

	page = reg / 128;
	data[AIC3254_REG_OFFSET_INDEX] = reg % 128;

	if (aic3254->page_no != page) {
		aic3254_change_page(codec, page);
	}

	/* data is
	 *   D15..D8 aic3254 register offset
	 *   D7...D0 register data
	 */
	data[AIC3254_REG_DATA_INDEX] = value & AIC3254_8BITS_MASK;
#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1)) {
		aic3254_write_reg_cache(codec, reg, value);
	}
#endif
	if (!data[AIC3254_REG_OFFSET_INDEX]) {
		/* if the write is to reg0 update aic3254->page_no */
		aic3254->page_no = value;
	}

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		printk("Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_read
 * Purpose  : This function is to read the aic3254 register space.
 *            
 *----------------------------------------------------------------------------
 */
static u8 aic3254_read(struct snd_soc_codec *codec, u16 reg)
{
	struct aic3254_priv *aic3254 = codec->private_data;
	u8 value;
	u8 page = reg / 128;

	reg = reg % 128;

	if (aic3254->page_no != page) {
		aic3254_change_page(codec, page);
	}

	i2c_master_send(codec->control_data, (char *)&reg, 1);
	i2c_master_recv(codec->control_data, &value, 1);
	return value;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_get_divs
 * Purpose  : This function is to get required divisor from the "aic3254_divs"
 *            table.
 *            
 *----------------------------------------------------------------------------
 */
static inline int aic3254_get_divs(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aic3254_divs); i++) {
		if ((aic3254_divs[i].rate == rate)
		    && (aic3254_divs[i].mclk == mclk)) {
			return i;
		}
	}
	printk("Master clock and sample rate is not supported\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_add_controls
 * Purpose  : This function is to add non dapm kcontrols.  The different 
 *            controls are in "aic3254_snd_controls" table.
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
static int aic3254_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(aic3254_snd_controls); i++) {
		err =
		    snd_ctl_add(codec->card,
				snd_soc_cnew(&aic3254_snd_controls[i], codec,
					     NULL));
		if (err < 0) {
			printk("Invalid control\n");
			return err;
		}
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_add_widgets
 * Purpose  : This function is to add the dapm widgets 
 *            The following are the main widgets supported
 *                # Left DAC to Left Outputs
 *                # Right DAC to Right Outputs
 *		  # Left Inputs to Left ADC
 *		  # Right Inputs to Right ADC
 *
 *----------------------------------------------------------------------------
 */
static int aic3254_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aic3254_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &aic3254_dapm_widgets[i]);
	}

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, &aic3254_dapm_routes[0],
				AIC3254_DAPM_ROUTE_NUM);

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_hw_params
 * Purpose  : This function is to set the hardware parameters for AIC3254.
 *            The functions set the sample rate and audio serial data word 
 *            length.
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct aic3254_priv *aic3254 = codec->private_data;
	int i, j;
	u8 data;

	aic3254_set_bias_level(codec, SNDRV_CTL_POWER_D3hot);

	i = aic3254_get_divs(aic3254->sysclk, params_rate(params));

	if (i < 0) {
		printk("sampling rate not supported\n");
		return i;
	}

	if (soc_static_freq_config) {
		/* We will fix R value to 1 and will make P & J=K.D as varialble */

		/* Setting P & R values */
		aic3254_write(codec, CLK_REG_2,
			      ((aic3254_divs[i].p_val << 4) | 0x01));

		/* J value */
		aic3254_write(codec, CLK_REG_3, aic3254_divs[i].pll_j);

		/* MSB & LSB for D value */
		aic3254_write(codec, CLK_REG_4, (aic3254_divs[i].pll_d >> 8));
		aic3254_write(codec, CLK_REG_5,
			      (aic3254_divs[i].pll_d & AIC3254_8BITS_MASK));

		/* NDAC divider value */
		aic3254_write(codec, NDAC_CLK_REG_6, aic3254_divs[i].ndac);

		/* MDAC divider value */
		aic3254_write(codec, MDAC_CLK_REG_7, aic3254_divs[i].mdac);

		/* DOSR MSB & LSB values */
		aic3254_write(codec, DAC_OSR_MSB, aic3254_divs[i].dosr >> 8);
		aic3254_write(codec, DAC_OSR_LSB,
			      aic3254_divs[i].dosr & AIC3254_8BITS_MASK);

		/* NADC divider value */
		aic3254_write(codec, NADC_CLK_REG_8, aic3254_divs[i].nadc);

		/* MADC divider value */
		aic3254_write(codec, MADC_CLK_REG_9, aic3254_divs[i].madc);

		/* AOSR value */
		aic3254_write(codec, ADC_OSR_REG, aic3254_divs[i].aosr);
	}
	/* BCLK N divider */
	aic3254_write(codec, CLK_REG_11, aic3254_divs[i].blck_N);

	aic3254_set_bias_level(codec, SNDRV_CTL_POWER_D0);

	data = aic3254_read(codec, INTERFACE_SET_REG_1);

	data = data & ~(3 << 4);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (AIC3254_WORD_LEN_20BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (AIC3254_WORD_LEN_24BITS << DAC_OSR_MSB_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (AIC3254_WORD_LEN_32BITS << DAC_OSR_MSB_SHIFT);
		break;
	}

	aic3254_write(codec, INTERFACE_SET_REG_1, data);

	for (j = 0; j < NO_FEATURE_REGS; j++) {
		aic3254_write(codec,
			      aic3254_divs[i].codec_specific_regs[j].reg_offset,
			      aic3254_divs[i].codec_specific_regs[j].reg_val);
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_mute
 * Purpose  : This function is to mute or unmute the left and right DAC
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 dac_reg;

	dac_reg = aic3254_read(codec, DAC_MUTE_CTRL_REG) & ~MUTE_ON;
	if (mute)
		aic3254_write(codec, DAC_MUTE_CTRL_REG, dac_reg | MUTE_ON);
	else
		aic3254_write(codec, DAC_MUTE_CTRL_REG, dac_reg);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_set_dai_sysclk
 * Purpose  : This function is to set the DAI system clock
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3254_priv *aic3254 = codec->private_data;

	switch (freq) {
	case AIC3254_FREQ_12000000:
	case AIC3254_FREQ_24000000:
		aic3254->sysclk = freq;
		return 0;
	}
	printk("Invalid frequency to set DAI system clock\n");
	return -EINVAL;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_set_dai_fmt
 * Purpose  : This function is to set the DAI format
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3254_priv *aic3254 = codec->private_data;
	u8 iface_reg;

	iface_reg = aic3254_read(codec, INTERFACE_SET_REG_1);
	iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic3254->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic3254->master = 0;
		break;
	default:
		printk("Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (AIC3254_DSP_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (AIC3254_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (AIC3254_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
		break;
	default:
		printk("Invalid DAI interface format\n");
		return -EINVAL;
	}

	aic3254_write(codec, INTERFACE_SET_REG_1, iface_reg);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_set_bias_level
 * Purpose  : This function is to get triggered when dapm events occurs.
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct aic3254_priv *aic3254 = codec->private_data;
	u8 value;

	switch (level) {
		/* full On */
	case SND_SOC_BIAS_ON:
		/* all power is driven by DAPM system */
		if (aic3254->master) {
			/* Switch on PLL */
			value = aic3254_read(codec, CLK_REG_2);
			aic3254_write(codec, CLK_REG_2, (value | ENABLE_PLL));

			/* Switch on NDAC Divider */
			value = aic3254_read(codec, NDAC_CLK_REG_6);
			aic3254_write(codec, NDAC_CLK_REG_6,
				      value | ENABLE_NDAC);

			/* Switch on MDAC Divider */
			value = aic3254_read(codec, MDAC_CLK_REG_7);
			aic3254_write(codec, MDAC_CLK_REG_7,
				      value | ENABLE_MDAC);

			/* Switch on NADC Divider */
			value = aic3254_read(codec, NADC_CLK_REG_8);
			aic3254_write(codec, NADC_CLK_REG_8,
				      value | ENABLE_MDAC);

			/* Switch on MADC Divider */
			value = aic3254_read(codec, MADC_CLK_REG_9);
			aic3254_write(codec, MADC_CLK_REG_9,
				      value | ENABLE_MDAC);

			/* Switch on BCLK_N Divider */
			value = aic3254_read(codec, CLK_REG_11);
			aic3254_write(codec, CLK_REG_11, value | ENABLE_BCLK);
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
		if (aic3254->master) {
			/* Switch off PLL */
			value = aic3254_read(codec, CLK_REG_2);
			aic3254_write(codec, CLK_REG_2, (value & ~ENABLE_PLL));

			/* Switch off NDAC Divider */
			value = aic3254_read(codec, NDAC_CLK_REG_6);
			aic3254_write(codec, NDAC_CLK_REG_6,
				      value & ~ENABLE_NDAC);

			/* Switch off MDAC Divider */
			value = aic3254_read(codec, MDAC_CLK_REG_7);
			aic3254_write(codec, MDAC_CLK_REG_7,
				      value & ~ENABLE_MDAC);

			/* Switch off NADC Divider */
			value = aic3254_read(codec, NADC_CLK_REG_8);
			aic3254_write(codec, NADC_CLK_REG_8,
				      value & ~ENABLE_NDAC);

			/* Switch off MADC Divider */
			value = aic3254_read(codec, MADC_CLK_REG_9);
			aic3254_write(codec, MADC_CLK_REG_9,
				      value & ~ENABLE_MDAC);
			value = aic3254_read(codec, CLK_REG_11);

			/* Switch off BCLK_N Divider */
			aic3254_write(codec, CLK_REG_11, value & ~ENABLE_BCLK);
		}
		break;

		/* Off, without power */
	case SND_SOC_BIAS_OFF:
		/* force all power off */
		break;
	}
	codec->bias_level = level;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_suspend
 * Purpose  : This function is to suspend the AIC3254 driver.
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	aic3254_set_bias_level(codec, SNDRV_CTL_POWER_D3cold);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_resume
 * Purpose  : This function is to resume the AIC3254 driver
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u8 *cache = codec->reg_cache;

	aic3254_change_page(codec, 0);
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(aic3254_reg); i++) {
		data[0] = i % 128;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, 2);
	}
	aic3254_change_page(codec, 0);
	aic3254_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_init
 * Purpose  : This function is to initialise the AIC3254 driver
 *            register the mixer and dsp interfaces with the kernel.
 *            
 *----------------------------------------------------------------------------
 */
static int tlv320aic3254_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	struct aic3254_priv *aic3254 = codec->private_data;
	int ret = 0;
	int i = 0;

	codec->name = "aic3254";
	codec->owner = THIS_MODULE;
	codec->read = aic3254_read;
	codec->write = aic3254_write;
	codec->set_bias_level = aic3254_set_bias_level;
	codec->dai = &tlv320aic3254_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(aic3254_reg);
	codec->reg_cache =
	    kmemdup(aic3254_reg, sizeof(aic3254_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		printk(KERN_ERR "aic3254: kmemdup failed\n");
		return -ENOMEM;
	}

	aic3254->page_no = 0;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "aic3254: failed to create pcms\n");
		goto pcm_err;
	}

	for (i = 0;
	     i < sizeof(aic3254_reg_init) / sizeof(struct aic3254_configs);
	     i++) {
		aic3254_write(codec, aic3254_reg_init[i].reg_offset,
			      aic3254_reg_init[i].reg_val);
	}

	/* off, with power on */
	aic3254_set_bias_level(codec, SNDRV_CTL_POWER_D3hot);
	aic3254_add_controls(codec);
	aic3254_add_widgets(codec);

#ifdef CONFIG_MINI_DSP
	aic3254_add_minidsp_controls(codec);
#endif
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "aic3254: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 *----------------------------------------------------------------------------
 * Function : aic3254_codec_probe
 * Purpose  : This function attaches the i2c client and initializes 
 *				AIC3254 CODEC.
 *            NOTE:
 *            This function is called from i2c core when the I2C address is
 *            valid.
 *            If the i2c layer weren't so broken, we could pass this kind of 
 *            data around
 *            
 *----------------------------------------------------------------------------
 */
static int tlv320aic3254_codec_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = aic3254_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

    ret = tlv320aic3254_init(socdev);
	if (ret < 0) {
		printk(KERN_ERR "aic3254: failed to attach codec at addr\n");
		return -1;
	}
#ifdef CONFIG_MINI_DSP
	/* Program MINI DSP for ADC and DAC */
	aic3254_minidsp_program(codec);
	aic3254_change_page(codec, 0x0);
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

static int __exit tlv320aic3254_i2c_remove(struct i2c_client *i2c)
{
        put_device(&i2c->dev);
        return 0;
}

static const struct i2c_device_id tlv320aic3254_id[] = {
        {"tlv320aic3254", 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, tlv320aic3254_id);

static struct i2c_driver tlv320aic3254_i2c_driver = {
	.driver = {
		.name = "tlv320aic3254",
	},
	.probe = tlv320aic3254_codec_probe,
	.remove = __exit_p(tlv320aic3254_i2c_remove),
	.id_table = tlv320aic3254_id,
};

#endif //#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_probe
 * Purpose  : This is first driver function called by the SoC core driver.
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct aic3254_priv *aic3254;
	int ret = 0;

	printk(KERN_INFO "AIC3254 Audio Codec %s\n", AIC3254_VERSION);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if(codec == NULL)
		return -ENOMEM;

	aic3254 = kzalloc(sizeof(struct aic3254_priv), GFP_KERNEL);
	if (aic3254 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = aic3254;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	aic3254_socdev = socdev;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
		codec->hw_write = (hw_write_t) i2c_master_send;
		codec->hw_read = (hw_read_t) i2c_master_recv;
		ret = i2c_add_driver(&tlv320aic3254_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
#else
	/* Add other interfaces here */
#endif
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_remove
 * Purpose  : to remove aic3254 soc device 
 *            
 *----------------------------------------------------------------------------
 */
static int aic3254_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	/* power down chip */
	if (codec->control_data)
		aic3254_set_bias_level(codec, SNDRV_CTL_POWER_D3);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&tlv320aic3254_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_device |
 *          This structure is soc audio codec device sturecute which pointer
 *          to basic functions aic3254_probe(), aic3254_remove(),  
 *          aic3254_suspend() and aic3254_resume()
 *
 */
struct snd_soc_codec_device soc_codec_dev_aic3254 = {
	.probe = aic3254_probe,
	.remove = aic3254_remove,
	.suspend = aic3254_suspend,
	.resume = aic3254_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_aic3254);

static int __init tlv320aic3254_modinit(void)
{
	return snd_soc_register_dai(&tlv320aic3254_dai);
}

module_init(tlv320aic3254_modinit);

static void __exit tlv320aic3254_exit(void)
{
	snd_soc_unregister_dai(&tlv320aic3254_dai);
}

module_exit(tlv320aic3254_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3254 codec driver");
MODULE_AUTHOR("sandeepsp@mistralsolutions.com");
MODULE_LICENSE("GPL");
