/*
 * linux/sound/soc/codecs/tlv320aic3254_mini-dsp.c
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
 * Rev 0.1 	 mini DSP support    		Mistral         4-05-2009
 *
 *          The mini DSP programming support is added to codec AIC3254.
 *
 *
 * Rev 1.0 	 mini DSP mixer controls 	Mistral         11-05-2009
 *
 *         	 Added mixer controls for aic3254 codec, and code cleanup
 *
 * Rev 1.1 	 TiLoad support         	Mistral         21-05-2009
 *
 *         	 Added char driver for TiLoad support, and mixer controls 
 *         	 for adaptive filtering and coefficient buffer controls
 * 
 * Rev 2.0 	 TiLoad support         	Mistral         12-06-2009
 *
 *         	 Added support for register read and for magic number 
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/control.h>

#include "tlv320aic3254.h"
#include "tlv320aic3254_mini-dsp.h"

/* enable debug prints in the driver */
//#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define dprintk(x...) 	printk(x)
#else
#define dprintk(x...)
#endif

#ifdef CONFIG_MINI_DSP

//#define VERIFY_MINIDSP                                1

/* Function prototypes */
#ifdef REG_DUMP_MINIDSP
static void aic3254_dump_page(struct i2c_client *i2c, u8 page);
#endif

/* externs */
extern int aic3254_change_page(struct snd_soc_codec *codec, u8 new_page);
extern int aic3254_write(struct snd_soc_codec *codec, u16 reg, u8 value);
static int minidsp_driver_init(struct snd_soc_codec *codec);

static unsigned int magic_num;

/******************************** Debug section *****************************/

#ifdef REG_DUMP_MINIDSP
/*
 *----------------------------------------------------------------------------
 * Function : aic3254_dump_page
 * Purpose  : Read and display one codec register page, for debugging purpose
 *----------------------------------------------------------------------------
 */
static void aic3254_dump_page(struct i2c_client *i2c, u8 page)
{
	int i;
	u8 data;
	u8 test_page_array[256];

	aic3254_change_page(codec, page);

	data = 0x0;

	i2c_master_send(i2c, data, 1);
	i2c_master_recv(i2c, test_page_array, 128);

	printk("\n------- MINI_DSP PAGE %d DUMP --------\n", page);
	for (i = 0; i < 128; i++) {
		printk(" [ %d ] = 0x%x\n", i, test_page_array[i]);
	}
}
#endif

/******************** MINI DSP Static Programming section *******************/

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_minidsp_get_burst
 * Purpose  : Format one I2C burst for transfer from mini dsp program array.
 * 			  This function will parse the program array and get next burst 
 * 			  data for doing an I2C bulk transfer.
 *----------------------------------------------------------------------------
 */
static void aic3254_minidsp_get_burst(reg_value * program_ptr, int program_size,
				      minidsp_parser_data * parse_data)
{
	int index = parse_data->current_loc;
	int burst_write_count = 0;

	/* check if first location is page register, and populate page addr */
	if (program_ptr[index].reg_off == 0) {
		parse_data->page_num = program_ptr[index].reg_val;
		index++;
	}

	parse_data->burst_array[burst_write_count++] =
	    program_ptr[index].reg_off;
	parse_data->burst_array[burst_write_count++] =
	    program_ptr[index].reg_val;
	index++;

	for (; index < program_size; index++) {
		if (program_ptr[index].reg_off !=
		    (program_ptr[index - 1].reg_off + 1))
			break;
		else {
			parse_data->burst_array[burst_write_count++] =
			    program_ptr[index].reg_val;
		}
	}

	parse_data->burst_size = burst_write_count;

	if (index == program_size) {
		/* parsing completed */
		parse_data->current_loc = MINIDSP_PARSING_END;
	} else
		parse_data->current_loc = index;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_minidsp_write_burst
 *
 * Purpose  : Write one I2C burst to the codec registers. The buffer for burst 
 * 			  transfer is given by aic3254_minidsp_get_burst() function.
 *----------------------------------------------------------------------------
 */
static int aic3254_minidsp_write_burst(struct snd_soc_codec *codec,
				       minidsp_parser_data * parse_data)
{
#ifdef VERIFY_MINIDSP
	int i;
	char read_addr;
	char test_page_array[256];
#endif
	struct i2c_client *i2c = codec->control_data;

	aic3254_change_page(codec, parse_data->page_num);

	/* write burst data */
	if ((i2c_master_send(i2c, parse_data->burst_array,
			     parse_data->burst_size)) !=
	    parse_data->burst_size) {
		dprintk("Mini DSP: i2c_master_send failed\n");
		return -1;
	}
#ifdef VERIFY_MINIDSP
	read_addr = parse_data->burst_array[0];
	i2c_master_send(i2c, &read_addr, 1);

	if ((i2c_master_recv(i2c, test_page_array, parse_data->burst_size))
	    != parse_data->burst_size) {
		dprintk("Mini DSP: i2c_master_recv failed\n");
		return -1;
	}

	for (i = 0; i < parse_data->burst_size - 1; i++) {
		if (test_page_array[i] != parse_data->burst_array[i + 1]) {
			dprintk
			    ("MINI DSP program verification failure on page 0x%x\n",
			     parse_data->page_num);
			return -1;
		}
	}
	dprintk("MINI DSP program verification success on page 0x%x\n",
		parse_data->page_num);
#endif

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_program
 * Purpose  : Program mini dsp instructions and/or coeffients
 *----------------------------------------------------------------------------
 */
static int minidsp_program(struct snd_soc_codec *codec,
			   reg_value * program_ptr, int program_size)
{
	minidsp_parser_data parse_data;

	/* point the current location to start of program array */
	parse_data.current_loc = 0;

	do {
		/* Get first burst data */
		aic3254_minidsp_get_burst(program_ptr, program_size,
					  &parse_data);

		dprintk("Burst,PAGE=0x%x Size=%d\n", parse_data.page_num,
			parse_data.burst_size);

		/* Write one burst to the MINI DSP register space */
		aic3254_minidsp_write_burst(codec, &parse_data);

		/* Proceed to the next burst reg_addr_incruence */
	} while (parse_data.current_loc != MINIDSP_PARSING_END);

	return 0;
}

/********************* CODEC REGISTER PROGRAMMING ***********************/

/* 
 * There may be instance where the same register may be programmed by the
 * default ASoC driver and when programming the registers in the PPS 
 * configuration file. The register number in the below array indicates which
 * registers in the PPS config file will be written. Others will be ignored
 */

/* Allowed registers for page '0' */
static u8 page_0_allow_regs[] = { 21, 22, 15, 16, 60, 61, 17, 255, 23, 35 };

/* Allowed registers for page '1' */
static u8 page_1_allow_regs[] = { };

/*
 *----------------------------------------------------------------------------
 * Function : check_allow_list
 *
 * Purpose  : Check and return if the register is allowed to write.
 * 			  Allowed registers are store in an arry per page. The register
 * 			  and page address passed are checked against this list and 
 * 			  return whether to ignore or not 
 *----------------------------------------------------------------------------
 */
static int check_allow_list(u8 page, u8 reg_addr)
{
	int i;

	if (page == 0) {
		for (i = 0; i < sizeof(page_0_allow_regs); i++) {
			if (page_0_allow_regs[i] == reg_addr)
				/* allow register write */
				return (CODEC_REG_DONT_IGNORE);
		}
		/* not in allow list, ignore the register */
		return (CODEC_REG_IGNORE);
	} else if (page == 1) {
		for (i = 0; i < sizeof(page_1_allow_regs); i++) {
			if (page_1_allow_regs[i] == reg_addr)
				/* allow register write */
				return (CODEC_REG_DONT_IGNORE);

		}
		/* not in allow list, ignore the register */
		return (CODEC_REG_IGNORE);
	} else {
		/* Write to pages other than 0,1 are allowed */
		return (CODEC_REG_DONT_IGNORE);
	}
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_codec_reg_program
 *
 * Purpose  : Program codec registers, from the codec register section
 * 			  of mini dsp program header file.
 *----------------------------------------------------------------------------
 */
static void minidsp_codec_reg_program(struct snd_soc_codec *codec,
				      reg_value * reg_array, int program_size,
				      int pre_post)
{
	int i, ret;
	u8 current_page = 0;
	u16 reg_addr;
	static int stored_index = 0x0;
	static u8 stored_page = 0x0;

	/* If post init is defined, start writing
	 * codec registers from previously stored 
	 * array index and page, else from beginning
	 */
	if (pre_post == CODEC_REG_POST_INIT) {
		i = stored_index;
		aic3254_change_page(codec, stored_page);
	} else {
		i = 0;
	}

	/* parse the register array */
	for (; i < program_size; i++) {
		/* check if page number is changed */
		if (reg_array[i].reg_off == 0x0) {
			current_page = reg_array[i].reg_val;
			aic3254_change_page(codec, current_page);
		} else {
			/*  Initialization is prior to MINI DSP programming 
			 *      stop register writes, at the delimiter register, and 
			 *      continue after DSP initialization is complete 
			 */
			if ((pre_post == CODEC_REG_PRE_INIT) &&
			    (reg_array[i].reg_off == INIT_SEQ_DELIMITER)) {
				/*
				 * Stop writing codec registers and 
				 * move index to next valid register (2 entries ahead)
				 * and store index and page number statically
				 * and return back
				 */
				stored_index = i + DELIMITER_COUNT;
				stored_page = current_page;
				return;
			}

			/* Check the register to be ignored or written */
			ret =
			    check_allow_list(current_page,
					     reg_array[i].reg_off);

			if (ret == CODEC_REG_DONT_IGNORE) {
				dprintk("Writing page 0x%x reg %d = 0x%x\n",
					current_page,
					reg_array[i].reg_off,
					reg_array[i].reg_val);
				/* convert addr to 16 bit address, by adding page offset */
				reg_addr =
				    (current_page * 128) + reg_array[i].reg_off;

				ret =
				    aic3254_write(codec, reg_addr,
						  reg_array[i].reg_val);
				if (ret) {
					dprintk
					    ("Write failed for register %d\n",
					     reg_array[i].reg_off);
				}
			}
		}
	}
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3254_minidsp_program
 *
 * Purpose  : Program mini dsp for AIC3254 codec chip. This routine is
 * 			  called from the aic3254 codec driver, if mini dsp programming 
 * 			  is enabled.  
 *----------------------------------------------------------------------------
 */
int aic3254_minidsp_program(struct snd_soc_codec *codec)
{
	int i, ret;
	printk("AIC3254: programming mini dsp\n");

	printk("The register sections found in pps header file:\n");
	for (i = 0; i < ARRAY_SIZE(REG_Section_names); i++) {
		printk("%s\n", REG_Section_names[i]);
	}

#ifdef PROGRAM_CODEC_REG_SECTIONS
	/* Array size should be greater than 1 to start programming,
	 * since first write command will be the page register 
	 */

	if (ARRAY_SIZE(REG_Section_program) > 1) {
		minidsp_codec_reg_program(codec, REG_Section_program,
					  ARRAY_SIZE(REG_Section_program),
					  CODEC_REG_PRE_INIT);
	} else {
		dprintk("CODEC_REGS: Insufficient data for programming\n");
	}
#endif

#ifdef PROGRAM_MINI_DSP_A
	if (ARRAY_SIZE(miniDSP_A_reg_values) > 1) {
		minidsp_program(codec, miniDSP_A_reg_values,
				ARRAY_SIZE(miniDSP_A_reg_values));
	} else {
		dprintk("MINI_DSP_A: Insufficient data for programming\n");
	}
#endif

#ifdef PROGRAM_MINI_DSP_D
	if (ARRAY_SIZE(miniDSP_D_reg_values) > 1) {
		minidsp_program(codec, miniDSP_D_reg_values,
				ARRAY_SIZE(miniDSP_D_reg_values));
	} else {
		dprintk("MINI_DSP_D: Insufficient data for programming\n");
	}
#endif

#ifdef PROGRAM_CODEC_REG_SECTIONS
	if (ARRAY_SIZE(REG_Section_program) > 1) {
		minidsp_codec_reg_program(codec, REG_Section_program,
					  ARRAY_SIZE(REG_Section_program),
					  CODEC_REG_POST_INIT);
	} else {
		dprintk("CODEC_REGS: Insufficient data for programming\n");
	}
#endif

	ret = minidsp_driver_init(codec);

	return ret;
}

/********************* AMIXER Controls for mini dsp *************************/

#ifdef ADD_MINI_DSP_CONTROLS

/* Volume Lite coefficents table */
static int volume_lite_table[] = {
	0x00000D, 0x00000E, 0x00000E, 0x00000F,
	0x000010, 0x000011, 0x000012, 0x000013,
	0x000015, 0x000016, 0x000017, 0x000018,
	0x00001A, 0x00001C, 0x00001D, 0x00001F,
	0x000021, 0x000023, 0x000025, 0x000027,
	0x000029, 0x00002C, 0x00002F, 0x000031,
	0x000034, 0x000037, 0x00003B, 0x00003E,
	0x000042, 0x000046, 0x00004A, 0x00004F,
	0x000053, 0x000058, 0x00005D, 0x000063,
	0x000069, 0x00006F, 0x000076, 0x00007D,
	0x000084, 0x00008C, 0x000094, 0x00009D,
	0x0000A6, 0x0000B0, 0x0000BB, 0x0000C6,
	0x0000D2, 0x0000DE, 0x0000EB, 0x0000F9,
	0x000108, 0x000118, 0x000128, 0x00013A,
	0x00014D, 0x000160, 0x000175, 0x00018B,
	0x0001A3, 0x0001BC, 0x0001D6, 0x0001F2,
	0x000210, 0x00022F, 0x000250, 0x000273,
	0x000298, 0x0002C0, 0x0002E9, 0x000316,
	0x000344, 0x000376, 0x0003AA, 0x0003E2,
	0x00041D, 0x00045B, 0x00049E, 0x0004E4,
	0x00052E, 0x00057C, 0x0005D0, 0x000628,
	0x000685, 0x0006E8, 0x000751, 0x0007C0,
	0x000836, 0x0008B2, 0x000936, 0x0009C2,
	0x000A56, 0x000AF3, 0x000B99, 0x000C49,
	0x000D03, 0x000DC9, 0x000E9A, 0x000F77,
	0x001062, 0x00115A, 0x001262, 0x001378,
	0x0014A0, 0x0015D9, 0x001724, 0x001883,
	0x0019F7, 0x001B81, 0x001D22, 0x001EDC,
	0x0020B0, 0x0022A0, 0x0024AD, 0x0026DA,
	0x002927, 0x002B97, 0x002E2D, 0x0030E9,
	0x0033CF, 0x0036E1, 0x003A21, 0x003D93,
	0x004139, 0x004517, 0x00492F, 0x004D85,
	0x00521D, 0x0056FA, 0x005C22, 0x006197,
	0x006760, 0x006D80, 0x0073FD, 0x007ADC,
	0x008224, 0x0089DA, 0x009205, 0x009AAC,
	0x00A3D7, 0x00B7D4, 0x00AD8C, 0x00C2B9,
	0x00CE43, 0x00DA7B, 0x00E76E, 0x00F524,
	0x0103AB, 0x01130E, 0x01235A, 0x01349D,
	0x0146E7, 0x015A46, 0x016ECA, 0x018486,
	0x019B8C, 0x01B3EE, 0x01CDC3, 0x01E920,
	0x02061B, 0x0224CE, 0x024553, 0x0267C5,
	0x028C42, 0x02B2E8, 0x02DBD8, 0x030736,
	0x033525, 0x0365CD, 0x039957, 0x03CFEE,
	0x0409C2, 0x044703, 0x0487E5, 0x04CCA0,
	0x05156D, 0x05628A, 0x05B439, 0x060ABF,
	0x066666, 0x06C77B, 0x072E50, 0x079B3D,
	0x080E9F, 0x0888D7, 0x090A4D, 0x09936E,
	0x0A24B0, 0x0ABE8D, 0x0B6188, 0x0C0E2B,
	0x0CC509, 0x0D86BD, 0x0E53EB, 0x0F2D42,
	0x101379, 0x110754, 0x1209A3, 0x131B40,
	0x143D13, 0x157012, 0x16B543, 0x180DB8,
	0x197A96, 0x1AFD13, 0x1C9676, 0x1E481C,
	0x201373, 0x21FA02, 0x23FD66, 0x261F54,
	0x28619A, 0x2AC625, 0x2D4EFB, 0x2FFE44,
	0x32D646, 0x35D96B, 0x390A41, 0x3C6B7E,
	0x400000, 0x43CAD0, 0x47CF26, 0x4C106B,
	0x50923B, 0x55586A, 0x5A6703, 0x5FC253,
	0x656EE3, 0x6B7186, 0x71CF54, 0x788DB4,
	0x7FB260,
};

/************************ VolumeLite control section ************************/

static struct snd_kcontrol_new snd_vol_controls[MAX_VOLUME_CONTROLS];

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_volume
 *
 * Purpose  : info routine for volumeLite amixer kcontrols 
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_volume(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_info *uinfo)
{
	int index;
	int ret_val = -1;

	for (index = 0; index < ARRAY_SIZE(VOLUME_controls); index++) {
		if (strstr(kcontrol->id.name, VOLUME_control_names[index]))
			break;
	}

	if (index < ARRAY_SIZE(VOLUME_controls)) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_VOLUME;
		uinfo->value.integer.max = MAX_VOLUME;
		ret_val = 0;
	}
	return ret_val;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_vol
 *
 * Purpose  : get routine for amixer kcontrols, read current register 
 * 			  values. Used for for mini dsp 'VolumeLite' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_volume(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = kcontrol->private_value;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_minidsp_volume
 *
 * Purpose  : put routine for amixer kcontrols, write user values to registers
 * 			  values. Used for for mini dsp 'VolumeLite' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_put_minidsp_volume(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	u8 data[5];
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int index;
	int user_value = ucontrol->value.integer.value[0];
	struct i2c_client *i2c;
	int ret_val = -1;
	int coeff;
	i2c = codec->control_data;

	dprintk("user value = 0x%x\n", user_value);

	for (index = 0; index < ARRAY_SIZE(VOLUME_controls); index++) {
		if (strstr(kcontrol->id.name, VOLUME_control_names[index]))
			break;
	}

	if (index < ARRAY_SIZE(VOLUME_controls)) {
		aic3254_change_page(codec, VOLUME_controls[index].control_page);

		coeff = volume_lite_table[user_value << 1];

		data[1] = (u8) ((coeff >> 16) & AIC3254_8BITS_MASK);
		data[2] = (u8) ((coeff >> 8) & AIC3254_8BITS_MASK);
		data[3] = (u8) ((coeff) & AIC3254_8BITS_MASK);

		/* Start register address */
		data[0] = VOLUME_controls[index].control_base;

		ret_val = i2c_master_send(i2c, data, VOLUME_REG_SIZE + 1);

		if (ret_val != VOLUME_REG_SIZE + 1) {
			dprintk("i2c_master_send transfer failed\n");
		} else {
			/* store the current level */
			kcontrol->private_value = user_value;
			ret_val = 0;
			/* Enable adaptive filtering for ADC/DAC */
			//data[0] = 0x1;  /* reg 1*/
			//data[1] = 0x05; /* Enable shifting buffer from A to B */
			//i2c_master_send(i2c, data, 2);
		}
	}

	aic3254_change_page(codec, 0);
	return (ret_val);
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_volume_lite_mixer_controls
 *
 * Purpose  : Add amixer kcontrols for mini dsp volume Lite controls, 
 *----------------------------------------------------------------------------
 */
static int minidsp_volume_mixer_controls(struct snd_soc_codec *codec)
{
	int i, err, no_volume_controls;
	static char volume_control_name[MAX_VOLUME_CONTROLS][40];

	no_volume_controls = ARRAY_SIZE(VOLUME_controls);

	dprintk(" %d mixer controls for mini dsp 'volumeLite' \n",
		no_volume_controls);

	if (no_volume_controls) {
		for (i = 0; i < no_volume_controls; i++) {
			strcpy(volume_control_name[i], VOLUME_control_names[i]);
			strcat(volume_control_name[i], VOLUME_KCONTROL_NAME);

			dprintk("Volume controls: %s\n",
				volume_control_name[i]);

			snd_vol_controls[i].name = volume_control_name[i];
			snd_vol_controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			snd_vol_controls[i].access =
			    SNDRV_CTL_ELEM_ACCESS_READWRITE;
			snd_vol_controls[i].info =
			    __new_control_info_minidsp_volume;
			snd_vol_controls[i].get =
			    __new_control_get_minidsp_volume;
			snd_vol_controls[i].put =
			    __new_control_put_minidsp_volume;
			/* 
			 *      TBD: read volume reg and update the index number 
			 */
			snd_vol_controls[i].private_value = 0;
			snd_vol_controls[i].count = 0;

			err = snd_ctl_add(codec->card,
					  snd_soc_cnew(&snd_vol_controls[i],
						       codec, NULL));
			if (err < 0) {
				printk("%s:Invalid control %s\n", __FILE__,
				       snd_vol_controls[i].name);
			}
		}
	}
	return 0;
}

/************************** MUX CONTROL section *****************************/
static struct snd_kcontrol_new snd_mux_controls[MAX_MUX_CONTROLS];

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_mux
 *
 * Purpose  : info routine for mini dsp mux control amixer kcontrols 
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_mux(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	int index;
	int ret_val = -1;

	for (index = 0; index < ARRAY_SIZE(MUX_controls); index++) {
		if (strstr(kcontrol->id.name, MUX_control_names[index]))
			break;
	}

	if (index < ARRAY_SIZE(MUX_controls)) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
	}
	return ret_val;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_mux
 *
 * Purpose  : get routine for  mux control amixer kcontrols, 
 * 			  read current register values to user. 
 * 			  Used for for mini dsp 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_mux(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = kcontrol->private_value;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_minidsp_mux
 *
 * Purpose  : put routine for amixer kcontrols, write user values to registers
 *            values. Used for for mini dsp 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_put_minidsp_mux(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u8 data[MUX_CTRL_REG_SIZE + 1];
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int index;
	int user_value = ucontrol->value.integer.value[0];
	struct i2c_client *i2c;
	int ret_val = -1;
	i2c = codec->control_data;

	dprintk("user value = 0x%x\n", user_value);

	for (index = 0; index < ARRAY_SIZE(MUX_controls); index++) {
		if (strstr(kcontrol->id.name, MUX_control_names[index]))
			break;
	}

	if (index < ARRAY_SIZE(MUX_controls)) {
		aic3254_change_page(codec, MUX_controls[index].control_page);

		data[1] = (u8) ((user_value >> 16) & AIC3254_8BITS_MASK);
		data[2] = (u8) ((user_value >> 8) & AIC3254_8BITS_MASK);
		data[3] = (u8) ((user_value) & AIC3254_8BITS_MASK);

		/* start register address */
		data[0] = MUX_controls[index].control_base;

		ret_val = i2c_master_send(i2c, data, MUX_CTRL_REG_SIZE + 1);

		if (ret_val != MUX_CTRL_REG_SIZE + 1) {
			dprintk("i2c_master_send transfer failed\n");
		} else {
			/* store the current level */
			kcontrol->private_value = user_value;
			ret_val = 0;
			/* Enable adaptive filtering for ADC/DAC */
			//data[0] = 0x1;  /* reg 1*/
			//data[1] = 0x05; /* Enable shifting buffer from A to B */
			//i2c_master_send(i2c, data, 2);
		}
	}

	aic3254_change_page(codec, 0);
	return (ret_val);
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_mux_ctrl_mixer_controls
 *
 * Purpose  : Add amixer kcontrols for mini dsp mux controls, 
 *----------------------------------------------------------------------------
 */
static int minidsp_mux_ctrl_mixer_controls(struct snd_soc_codec *codec)
{
	int i, err, no_mux_controls;

	no_mux_controls = ARRAY_SIZE(MUX_controls);

	dprintk(" %d mixer controls for mini dsp MUX \n", no_volume_controls);

	if (no_mux_controls) {
		for (i = 0; i < no_mux_controls; i++) {

			snd_mux_controls[i].name = MUX_control_names[i];
			snd_mux_controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			snd_mux_controls[i].access =
			    SNDRV_CTL_ELEM_ACCESS_READWRITE;
			snd_mux_controls[i].info =
			    __new_control_info_minidsp_mux;
			snd_mux_controls[i].get = __new_control_get_minidsp_mux;
			snd_mux_controls[i].put = __new_control_put_minidsp_mux;
			/* 
			 *  TBD: read volume reg and update the index number 
			 */
			snd_vol_controls[i].private_value = 0;
			snd_vol_controls[i].count = 0;

			err = snd_ctl_add(codec->card,
					  snd_soc_cnew(&snd_mux_controls[i],
						       codec, NULL));
			if (err < 0) {
				printk("%s:Invalid control %s\n", __FILE__,
				       snd_mux_controls[i].name);
			}
		}
	}
	return 0;
}

/************************** Adaptive filtering section **********************/

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_filter
 *
 * Purpose  : info routine for adaptive filter control amixer kcontrols 
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_adaptive(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_adaptive
 *
 * Purpose  : get routine for  adaptive filter control amixer kcontrols, 
 *            reads to user if adaptive filtering is enabled or disabled. 
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_adaptive(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value
					      *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct i2c_client *i2c;
	char data[2];
	int ret = 0;

	u8 page = (kcontrol->private_value) & AIC3254_8BITS_MASK;
	u8 reg = (kcontrol->private_value >> 8) & AIC3254_8BITS_MASK;
	u8 rmask = (kcontrol->private_value >> 16) & AIC3254_8BITS_MASK;

	i2c = codec->control_data;

	dprintk("page %d, reg %d, mask 0x%x\n", page, reg, rmask);

	/* Read the register value */
	aic3254_change_page(codec, page);

	/* write register addr to read */
	data[0] = reg;

	if (i2c_master_send(i2c, data, 1) != 1) {
		printk("Can not write register address\n");
		ret = -1;
		goto revert;
	}
	/* read the codec/minidsp registers */
	if (i2c_master_recv(i2c, data, 1) != 1) {
		printk("Can not read codec registers\n");
		ret = -1;
		goto revert;
	}

	dprintk("read: 0x%x\n", data[0]);

	/* return the read status to the user */
	if (data[0] & rmask) {
		ucontrol->value.integer.value[0] = 1;
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

      revert:
	/* put page back to zero */
	aic3254_change_page(codec, 0);
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_minidsp_adaptive
 *
 * Purpose  : put routine for adaptive filter controls amixer kcontrols.
 * 			  This routine will enable/disable adaptive filtering.
 *----------------------------------------------------------------------------
 */
static int __new_control_put_minidsp_adaptive(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value
					      *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int user_value = ucontrol->value.integer.value[0];
	struct i2c_client *i2c;
	char data[2];
	int ret = 0;

	u8 page = (kcontrol->private_value) & AIC3254_8BITS_MASK;
	u8 reg = (kcontrol->private_value >> 8) & AIC3254_8BITS_MASK;
	u8 wmask = (kcontrol->private_value >> 24) & AIC3254_8BITS_MASK;
	u8 rmask = (kcontrol->private_value >> 16) & AIC3254_8BITS_MASK;

	i2c = codec->control_data;

	dprintk("page %d, reg %d, mask 0x%x, user_value %d\n",
		page, reg, wmask, user_value);

	/* Program the register value */
	aic3254_change_page(codec, page);

	/* read register addr to read */
	data[0] = reg;

	if (i2c_master_send(i2c, data, 1) != 1) {
		printk("Can not write register address\n");
		ret = -1;
		goto revert;
	}
	/* read the codec/minidsp registers */
	if (i2c_master_recv(i2c, data, 1) != 1) {
		printk("Can not read codec registers\n");
		ret = -1;
		goto revert;
	}

	dprintk("read: 0x%x\n", data[0]);

	/* set the bitmask and update the register */
	if (user_value == 0) {
		data[1] = (data[0]) & (~wmask);
	} else {
		data[1] = (data[0]) | wmask;
	}
	data[0] = reg;

	if (i2c_master_send(i2c, data, 2) != 2) {
		dprintk("Can not write register address\n");
		ret = -1;
	}

      revert:
	/* put page back to zero */
	aic3254_change_page(codec, 0);
	return ret;
}

#define SOC_ADAPTIVE_CTL_AIC3254(xname, page, reg, read_mask, write_mask) \
{   .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
    .info = __new_control_info_minidsp_adaptive, \
    .get = __new_control_get_minidsp_adaptive, 	\
	.put = __new_control_put_minidsp_adaptive, \
	.count = 0,	\
    .private_value = (page) | (reg << 8) | 	\
		( read_mask << 16) | (write_mask << 24) \
}

/* Adaptive filtering control and buffer swap  mixer kcontrols */
static struct snd_kcontrol_new snd_adaptive_controls[] = {
	SOC_ADAPTIVE_CTL_AIC3254(FILT_CTL_NAME_ADC, BUFFER_PAGE_ADC, 0x1, 0x4,
				 0x4),
	SOC_ADAPTIVE_CTL_AIC3254(FILT_CTL_NAME_DAC, BUFFER_PAGE_DAC, 0x1, 0x4,
				 0x4),
	SOC_ADAPTIVE_CTL_AIC3254(COEFF_CTL_NAME_ADC, BUFFER_PAGE_ADC, 0x1, 0x2,
				 0x1),
	SOC_ADAPTIVE_CTL_AIC3254(COEFF_CTL_NAME_DAC, BUFFER_PAGE_DAC, 0x1, 0x2,
				 0x1),
};

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_adaptive_filter_mixer_controls
 *
 * Purpose  : registers adaptive filter mixer kcontrols 
 *----------------------------------------------------------------------------
 */
static int minidsp_adaptive_filter_mixer_controls(struct snd_soc_codec *codec)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(snd_adaptive_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&snd_adaptive_controls[i], codec,
					       NULL));

		if (err < 0) {
			printk("%s:Invalid control %s\n", __FILE__,
			       snd_adaptive_controls[i].name);
			return err;
		}
	}
	return 0;
}

#endif

void aic3254_add_minidsp_controls(struct snd_soc_codec *codec)
{
#ifdef ADD_MINI_DSP_CONTROLS
	if (minidsp_volume_mixer_controls(codec)) {
		printk
		    ("mini DSP volumeLite mixer control registration failed\n");
	}

	if (minidsp_mux_ctrl_mixer_controls(codec)) {
		printk
		    ("mini DSP mux selection mixer control registration failed\n");
	}

	if (minidsp_adaptive_filter_mixer_controls(codec)) {
		printk("Adaptive filter mixer control registration failed\n");
	}
#endif
}

/************** Dynamic MINI DSP programmer, TI LOAD support  ***************/

static struct cdev *minidsp_cdev;
static int minidsp_major = 0;	/* Dynamic allocation of Mjr No. */
static int minidsp_opened = 0;	/* Dynamic allocation of Mjr No. */
static struct snd_soc_codec *minidsp_codec;

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_open
 *
 * Purpose  : open method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static int minidsp_open(struct inode *in, struct file *filp)
{
	if (minidsp_opened) {
		printk("%s device is already opened\n", "minidsp");
		printk("%s: only one instance of driver is allowed\n",
		       "minidsp");
		return -1;
	}
	minidsp_opened++;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_release
 *
 * Purpose  : close method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static int minidsp_release(struct inode *in, struct file *filp)
{
	minidsp_opened--;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_read
 *
 * Purpose  : read method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t minidsp_read(struct file *file, char __user * buf,
			    size_t count, loff_t * offset)
{
	static char rd_data[256];
	char reg_addr;
	size_t size;
	struct i2c_client *i2c = minidsp_codec->control_data;

	if (count > 128) {
		printk("Max 256 bytes can be read\n");
		count = 128;
	}

	/* copy register address from user space  */
	size = copy_from_user(&reg_addr, buf, 1);
	if (size != 0) {
		printk("read: copy_from_user failure\n");
		return -1;
	}

	if (i2c_master_send(i2c, &reg_addr, 1) != 1) {
		dprintk("Can not write register address\n");
		return -1;
	}
	/* read the codec/minidsp registers */
	size = i2c_master_recv(i2c, rd_data, count);

	if (size != count) {
		printk("read %d registers from the codec\n", size);
	}

	if (copy_to_user(buf, rd_data, size) != 0) {
		dprintk("copy_to_user failed\n");
		return -1;
	}

	return size;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_write
 *
 * Purpose  : write method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t minidsp_write(struct file *file, const char __user * buf,
			     size_t count, loff_t * offset)
{
	static char wr_data[258];
	size_t size;
	struct i2c_client *i2c = minidsp_codec->control_data;

	/* copy buffer from user space  */
	size = copy_from_user(wr_data, buf, count);
	if (size != 0) {
		printk("copy_from_user failure %d\n", size);
		return -1;
	}

	if (wr_data[0] == 0) {
		aic3254_change_page(minidsp_codec, wr_data[1]);
	}

	size = i2c_master_send(i2c, wr_data, count);
	return size;
}

static int minidsp_ioctl(struct inode *inode, struct file *filp,
			 unsigned int cmd, unsigned long arg)
{

	if (_IOC_TYPE(cmd) != AIC3254_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case AIC3254_IOMAGICNUM_GET:
		copy_to_user(arg, &magic_num, sizeof(int));
		break;
	case AIC3254_IOMAGICNUM_SET:
		copy_from_user(&magic_num, arg, sizeof(int));
		break;
	}
}

/*********** File operations structure for minidsp programming *************/
static struct file_operations minidsp_fops = {
	.owner = THIS_MODULE,
	.open = minidsp_open,
	.release = minidsp_release,
	.read = minidsp_read,
	.write = minidsp_write,
	.ioctl = minidsp_ioctl,
};

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_driver_init
 *
 * Purpose  : Registeer a char driver for dynamic mini dsp programming
 *----------------------------------------------------------------------------
 */
static int minidsp_driver_init(struct snd_soc_codec *codec)
{
	int result;
	dev_t dev = MKDEV(minidsp_major, 0);

	minidsp_codec = codec;

	dprintk("allocating dynamic major number\n");

	result = alloc_chrdev_region(&dev, 0, 1, "minidsp");

	if (result < 0) {
		dprintk("cannot allocate major number %d\n", minidsp_major);
		return result;
	}

	minidsp_major = MAJOR(dev);
	dprintk("allocated Major Number: %d\n", minidsp_major);

	minidsp_cdev = cdev_alloc();
	cdev_init(minidsp_cdev, &minidsp_fops);
	minidsp_cdev->owner = THIS_MODULE;
	minidsp_cdev->ops = &minidsp_fops;

	if (cdev_add(minidsp_cdev, dev, 1) < 0) {
		dprintk("minidsp_driver: cdev_add failed \n");
		unregister_chrdev_region(dev, 1);
		minidsp_cdev = NULL;
		return 1;
	}
	printk("Registered minidsp driver, Major number: %d \n", minidsp_major);
	return 0;
}

#endif
