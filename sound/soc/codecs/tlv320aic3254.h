/*
 * linux/sound/soc/codecs/tlv320aic3254.h
 *
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 *
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
 * 
 */

#ifndef _TLV320AIC3254_H
#define _TLV320AIC3254_H

#define AUDIO_NAME "aic3254"
#define AIC3254_VERSION "1.1"

/* Macro enables or disables support for miniDSP in the driver */
#define CONFIG_MINI_DSP
//#undef CONFIG_MINI_DSP        

/* Enable register caching on write */
#define EN_REG_CACHE

/* AIC3254 supported sample rate are 8k to 192k */
#define AIC3254_RATES	SNDRV_PCM_RATE_8000_192000

/* AIC3254 supports the word formats 16bits, 20bits, 24bits and 32 bits */
#define AIC3254_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE \
			 | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

#define AIC3254_FREQ_12000000 12000000
#define AIC3254_FREQ_24000000 24000000

/* Audio data word length = 16-bits (default setting) */
#define AIC3254_WORD_LEN_16BITS		0x00
#define AIC3254_WORD_LEN_20BITS		0x01
#define AIC3254_WORD_LEN_24BITS		0x02
#define AIC3254_WORD_LEN_32BITS		0x03

/* sink: name of target widget */
#define AIC3254_WIDGET_NAME			0
/* control: mixer control name */
#define AIC3254_CONTROL_NAME		1
/* source: name of source name */
#define AIC3254_SOURCE_NAME			2

/* D15..D8 aic3254 register offset */
#define AIC3254_REG_OFFSET_INDEX    0
/* D7...D0 register data */
#define AIC3254_REG_DATA_INDEX      1

/* Serial data bus uses I2S mode (Default mode) */
#define AIC3254_I2S_MODE				0x00
#define AIC3254_DSP_MODE				0x01
#define AIC3254_RIGHT_JUSTIFIED_MODE	0x02
#define AIC3254_LEFT_JUSTIFIED_MODE		0x03

/* 8 bit mask value */
#define AIC3254_8BITS_MASK           0xFF

/* shift value for CLK_REG_3 register */
#define CLK_REG_3_SHIFT					6
/* shift value for DAC_OSR_MSB register */
#define DAC_OSR_MSB_SHIFT				4

/* number of codec specific register for configuration */
#define NO_FEATURE_REGS     			2

/* AIC3254 register space */
#define	AIC3254_CACHEREGNUM			256

/* ****************** Page 0 Registers **************************************/
/* Page select register */
#define	PAGE_SELECT			0
/* Software reset register */
#define	RESET				1
/*Clock setting register 1, Multiplexers*/
#define	CLK_REG_1			4
/*Clock setting register 2, PLL*/
#define	CLK_REG_2			5
/*Clock setting register 3, PLL*/
#define	CLK_REG_3			6
/*Clock setting register 4, PLL*/
#define	CLK_REG_4			7
/*Clock setting register 5, PLL*/
#define	CLK_REG_5			8
/*Clock setting register 6, PLL*/
#define	NDAC_CLK_REG_6		11
/*Clock setting register 7, PLL*/
#define	MDAC_CLK_REG_7		12
/*DAC OSR setting register1,MSB value*/
#define DAC_OSR_MSB			13
/*DAC OSR setting register 2,LSB value*/
#define DAC_OSR_LSB			14
/*Clock setting register 8, PLL*/
#define	NADC_CLK_REG_8		18
/*Clock setting register 9, PLL*/
#define	MADC_CLK_REG_9		19
/*ADC Oversampling (AOSR) Register*/
#define ADC_OSR_REG			20
/*Clock setting register 9, Multiplexers*/
#define CLK_MUX_REG_9		25
/*Clock setting register 10, CLOCKOUT M divider value*/
#define CLK_REG_10			26
/*Audio Interface Setting Register 1*/
#define INTERFACE_SET_REG_1	27
/*Audio Interface Setting Register 2*/
#define AIS_REG_2			28
/*Audio Interface Setting Register 3*/
#define AIS_REG_3			29
/*Clock setting register 11,BCLK N Divider*/
#define CLK_REG_11			30
/*Audio Interface Setting Register 4,Secondary Audio Interface*/
#define AIS_REG_4			31
/*Audio Interface Setting Register 5*/
#define AIS_REG_5			32
/*Audio Interface Setting Register 6*/
#define AIS_REG_6			33
 /**/
/*DOUT Function Control*/
#define DOUT_CTRL			53
/*DIN Function Control*/
#define DIN_CTL				54
/*DAC channel setup register*/
#define DAC_CHN_REG			63
/*DAC Mute and volume control register*/
#define DAC_MUTE_CTRL_REG	64
/*Left DAC channel digital volume control*/
#define LDAC_VOL			65
/*Right DAC channel digital volume control*/
#define RDAC_VOL			66
/*ADC Register 1*/
#define ADC_REG_1			81
/*ADC Fine Gain Adjust*/
#define	ADC_FGA				82
/*Left ADC Channel Volume Control*/
#define LADC_VOL			83
/*Right ADC Channel Volume Control*/
#define RADC_VOL			84
/*Left Channel AGC Control Register 1*/
#define LEFT_CHN_AGC_1		86
/*Left Channel AGC Control Register 2*/
#define LEFT_CHN_AGC_2		87
/*Left Channel AGC Control Register 3 */
#define LEFT_CHN_AGC_3		88
/*Left Channel AGC Control Register 4 */
#define LEFT_CHN_AGC_4		89
/*Left Channel AGC Control Register 5 */
#define LEFT_CHN_AGC_5		90
/*Left Channel AGC Control Register 6 */
#define LEFT_CHN_AGC_6		91
/*Left Channel AGC Control Register 7 */
#define LEFT_CHN_AGC_7		92
/*Right Channel AGC Control Register 1*/
#define RIGHT_CHN_AGC_1		94
/*Right Channel AGC Control Register 2*/
#define RIGHT_CHN_AGC_2 	95
/*Right Channel AGC Control Register 3 */
#define RIGHT_CHN_AGC_3		96
/*Right Channel AGC Control Register 4 */
#define RIGHT_CHN_AGC_4		97
/*Right Channel AGC Control Register 5 */
#define RIGHT_CHN_AGC_5		98
/*Right Channel AGC Control Register 6 */
#define RIGHT_CHN_AGC_6		99
/*Right Channel AGC Control Register 7 */
#define RIGHT_CHN_AGC_7		100
/******************** Page 1 Registers **************************************/
#define PAGE_1				128
/*Power Conguration*/
#define POW_CFG				(PAGE_1 + 1)
/*LDO Control*/
#define LDO_CTL				(PAGE_1 + 2)
/*Output Driver Power Control*/
#define OUT_PWR_CTL			(PAGE_1 + 9)
/*HPL Routing Selection*/
#define HPL_ROUTE_CTL		(PAGE_1 + 12)
/*HPR Routing Selection*/
#define HPR_ROUTE_CTL		(PAGE_1 + 13)
/*LOL Routing Selection*/
#define LOL_ROUTE_CTL		(PAGE_1 + 14)
/*LOR Routing Selection*/
#define LOR_ROUTE_CTL		(PAGE_1 + 15)
/*HPL Driver Gain*/
#define	HPL_GAIN			(PAGE_1 + 16)
/*HPR Driver Gain*/
#define	HPR_GAIN			(PAGE_1 + 17)
/*LOL Driver Gain*/
#define	LOL_GAIN			(PAGE_1 + 18)
/*LOR Driver Gain*/
#define	LOR_GAIN			(PAGE_1 + 19)
/*Headphone Driver Startup Control Register*/
#define HPHONE_STARTUP_CTRL	(PAGE_1 + 20)
/*MICBIAS Configuration Register*/
#define MICBIAS_CTRL		(PAGE_1 + 51)
/*Left MICPGA Positive Terminal Input Routing Configuration Register*/
#define LMICPGA_PIN_CFG		(PAGE_1 + 52)
/*Left MICPGA Negative Terminal Input Routing Configuration Register*/
#define LMICPGA_NIN_CFG		(PAGE_1 + 54)
/*Right MICPGA Positive Terminal Input Routing Configuration Register*/
#define RMICPGA_PIN_CFG		(PAGE_1 + 55)
/*Right MICPGA Negative Terminal Input Routing Configuration Register*/
#define RMICPGA_NIN_CFG		(PAGE_1 + 57)
/*Floating Input Configuration Register*/
#define INPUT_CFG_REG		(PAGE_1 + 58)
/*Left MICPGA Volume Control Register*/
#define LMICPGA_VOL_CTRL	(PAGE_1 + 59)
/*Right MICPGA Volume Control Register*/
#define RMICPGA_VOL_CTRL	(PAGE_1 + 60)
/*MICBIAS Configuration*/
#define MICBIAS				(PAGE_1 + 51)
/****************************************************************************/
#define BIT7					(0x01 << 7)
#define CODEC_CLKIN_MASK		0x03
#define MCLK_2_CODEC_CLKIN		0x00
#define PLLCLK_2_CODEC_CLKIN	0x03
/*Bclk_in selection*/
#define BDIV_CLKIN_MASK			0x03
#define	DAC_MOD_CLK_2_BDIV_CLKIN 0x01
#define SOFT_RESET				0x01
#define PAGE0					0x00
#define PAGE1					0x01
#define BIT_CLK_MASTER			0x08
#define WORD_CLK_MASTER			0x04
#define	HIGH_PLL 				(0x01 << 6)
#define ENABLE_PLL				BIT7
#define ENABLE_NDAC				BIT7
#define ENABLE_MDAC				BIT7
#define ENABLE_NADC				BIT7
#define ENABLE_MADC				BIT7
#define ENABLE_BCLK				BIT7
#define ENABLE_DAC				(0x03 << 6)
#define LDAC_2_LCHN				(0x01 << 4 )
#define RDAC_2_RCHN				(0x01 << 2 )
#define LDAC_CHNL_2_HPL			(0x01 << 3)
#define RDAC_CHNL_2_HPR			(0x01 << 3)
#define SOFT_STEP_2WCLK			(0x01)
#define MUTE_ON					0x0C
#define DEFAULT_VOL				0x0
#define DISABLE_ANALOG			(0x01 << 3)
#define LDAC_2_HPL_ROUTEON		0x08
#define RDAC_2_HPR_ROUTEON		0x08
/*
 ***************************************************************************** 
 * Structures Definitions
 ***************************************************************************** 
 */
/*
 *----------------------------------------------------------------------------
 * @struct  aic3254_setup_data |
 *          i2c specific data setup for AIC3254.
 * @field   unsigned short |i2c_address |
 *          Unsigned short for i2c address.
 *----------------------------------------------------------------------------
 */
    struct aic3254_setup_data {
	unsigned short i2c_address;
};

/*
 *----------------------------------------------------------------------------
 * @struct  aic3254_priv |
 *          AIC3254 priviate data structure to set the system clock, mode and
 *          page number. 
 * @field   u32 | sysclk |
 *          system clock
 * @field   s32 | master |
 *          master/slave mode setting for AIC3254
 * @field   u8 | page_no |
 *          page number. Here, page 0 and page 1 are used.
 *----------------------------------------------------------------------------
 */
struct aic3254_priv {
	u32 sysclk;
	s32 master;
	u8 page_no;
};

/*
 *----------------------------------------------------------------------------
 * @struct  aic3254_configs |
 *          AIC3254 initialization data which has register offset and register 
 *          value.
 * @field   u16 | reg_offset |
 *          AIC3254 Register offsets required for initialization..
 * @field   u8 | reg_val |
 *          value to set the AIC3254 register to initialize the AIC3254.
 *----------------------------------------------------------------------------
 */
struct aic3254_configs {
	u8 reg_offset;
	u8 reg_val;
};

/*
 *----------------------------------------------------------------------------
 * @struct  aic3254_rate_divs |
 *          Setting up the values to get different freqencies 
 *          
 * @field   u32 | mclk |
 *          Master clock 
 * @field   u32 | rate |
 *          sample rate
 * @field   u8 | p_val |
 *          value of p in PLL
 * @field   u32 | pll_j |
 *          value for pll_j
 * @field   u32 | pll_d |
 *          value for pll_d
 * @field   u32 | dosr |
 *          value to store dosr
 * @field   u32 | ndac |
 *          value for ndac
 * @field   u32 | mdac |
 *          value for mdac
 * @field   u32 | aosr |
 *          value for aosr
 * @field   u32 | nadc |
 *          value for nadc
 * @field   u32 | madc |
 *          value for madc
 * @field   u32 | blck_N |
 *          value for block N
 * @field   u32 | aic3254_configs |
 *          configurations for aic3254 register value
 *----------------------------------------------------------------------------
 */
struct aic3254_rate_divs {
	u32 mclk;
	u32 rate;
	u8 p_val;
	u8 pll_j;
	u16 pll_d;
	u16 dosr;
	u8 ndac;
	u8 mdac;
	u8 aosr;
	u8 nadc;
	u8 madc;
	u8 blck_N;
	struct aic3254_configs codec_specific_regs[NO_FEATURE_REGS];
};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai |
 *          It is SoC Codec DAI structure which has DAI capabilities viz., 
 *          playback and capture, DAI runtime information viz. state of DAI 
 *			and pop wait state, and DAI private data. 
 *----------------------------------------------------------------------------
 */
extern struct snd_soc_dai tlv320aic3254_dai;

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_device |
 *          This structure is soc audio codec device sturecute which pointer
 *          to basic functions aic3254_probe(), aic3254_remove(), 
 *			aic3254_suspend() and aic3254_resume()
 *
 */
extern struct snd_soc_codec_device soc_codec_dev_aic3254;

#endif				/* _TLV320AIC3254_H */
