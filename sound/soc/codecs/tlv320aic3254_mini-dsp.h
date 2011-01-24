/*
 * linux/sound/soc/codecs/tlv320aic3254_mini-dsp.h
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
 *
 * 
 */

#ifndef _TLV320AIC3254_MINI_DSP_H
#define _TLV320AIC3254_MINI_DSP_H

/* typedefs required for the included header files */
typedef char *string;

/* Include header file generated from PPS */
#include "main_Rate44_pps_driver.h"

/* defines */

/* Select the functionalities to be used in mini dsp module */
#define PROGRAM_MINI_DSP_A
#define PROGRAM_MINI_DSP_D
#define PROGRAM_CODEC_REG_SECTIONS
#define ADD_MINI_DSP_CONTROLS

/* volume ranges from -110db to 6db
 * amixer controls doesnot accept negative values
 * Therefore we are normalizing vlues to start from value 0
 * value 0 corresponds to -110db and 116 to 6db
 */
#define MAX_VOLUME_CONTROLS				2
#define MIN_VOLUME						0
#define MAX_VOLUME						116
#define VOLUME_REG_SIZE					3	/*  3 bytes */
#define VOLUME_KCONTROL_NAME			" (0=-110dB, 116=+6dB)"

#define FILT_CTL_NAME_ADC				"ADC adaptive filter(0=Disable, 1=Enable)"
#define FILT_CTL_NAME_DAC				"DAC adaptive filter(0=Disable, 1=Enable)"
#define COEFF_CTL_NAME_ADC				"ADC coeff Buffer(0=Buffer A, 1=Buffer B)"
#define COEFF_CTL_NAME_DAC				"DAC coeff Buffer(0=Buffer A, 1=Buffer B)"

#define BUFFER_PAGE_ADC					0x8
#define BUFFER_PAGE_DAC					0x2c

#define ADAPTIVE_MAX_CONTROLS			4

/*
 * MUX controls,  3 bytes of control data.
 */
#define MAX_MUX_CONTROLS             	2
#define MIN_MUX_CTRL	                0
#define MAX_MUX_CTRL    	            8
#define MUX_CTRL_REG_SIZE               3	/*  3 bytes */

#define MINIDSP_PARSING_START			0
#define MINIDSP_PARSING_END				(-1)

#define CODEC_REG_DONT_IGNORE			0
#define CODEC_REG_IGNORE				1

#define CODEC_REG_PRE_INIT				0
#define CODEC_REG_POST_INIT				1
#define INIT_SEQ_DELIMITER				255	/* Delimiter register */
#define DELIMITER_COUNT					2	/* 2 delimiter entries */

/* Parser info structure */
typedef struct {
	char page_num;
	char burst_array[129];
	int burst_size;
	int current_loc;
} minidsp_parser_data;

#define AIC3254_IOC_MAGIC	0xE0
#define AIC3254_IOMAGICNUM_GET	_IOR(AIC3254_IOC_MAGIC, 1, int)
#define AIC3254_IOMAGICNUM_SET	_IOW(AIC3254_IOC_MAGIC, 2, int)

#endif
