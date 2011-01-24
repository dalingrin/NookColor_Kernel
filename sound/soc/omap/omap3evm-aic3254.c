/*
 * omap3evm-aic3254.c - SoC audio for OMAP3530 EVM.
 * 
 * Copyright (C) 2009 Mistral Solutions
 *
 * Author: Sandeep S Prabhu	,sandeepsp@mistralsolutions.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Modified from n810.c  --  SoC audio for Nokia N810
 *
 * Revision History
 *
 * Inital code : May 7, 2009 :	Sandeep S Prabhu 
 * 					<sandeepsp@mistralsolutions.com>
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/soundcard.h>
#include <linux/clk.h>
#include <linux/cdev.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/mux.h>
#include <mach/io.h>
#include <asm/io.h>


#include "../codecs/tlv320aic3254.h"

#include "omap-pcm.h"
#include "omap-mcbsp.h"

static struct clk *sys_clkout2;
static struct clk *clkout2_src_ck;
static struct clk *cm_96m_ck;

static int n810_spk_func;
static int n810_jack_func=1;

static void n810_ext_control(struct snd_soc_codec *codec)
{
	if(n810_spk_func)
		snd_soc_dapm_enable_pin(codec, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Spk");

	if(n810_jack_func)
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");

	snd_soc_dapm_sync(codec);
}


static int omap3evm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int err;

	/* Set codec DAI configuration */
	err = codec_dai->ops.set_fmt(codec_dai,
					 SND_SOC_DAIFMT_I2S |
					 SND_SOC_DAIFMT_NB_NF |
					 SND_SOC_DAIFMT_CBM_CFM);

	if (err < 0)
		return err;

	/* Set cpu DAI configuration */
	err = cpu_dai->ops.set_fmt(cpu_dai,
				       SND_SOC_DAIFMT_I2S |
				       SND_SOC_DAIFMT_NB_NF |
				       SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
		return err;

	/* Set the codec system clock for DAC and ADC */
	err = codec_dai->ops.set_sysclk(codec_dai, 0, 12000000,
					    SND_SOC_CLOCK_IN);

	return err;
}

static int omap3evm_startup(struct snd_pcm_substream *substream)
{
	return clk_enable(sys_clkout2);
}
static void omap3evm_shutdown(struct snd_pcm_substream *substream)
{
	clk_disable(sys_clkout2);
}

static struct snd_soc_ops omap3evm_ops = {
	.startup = omap3evm_startup,
	.hw_params = omap3evm_hw_params,
	.shutdown = omap3evm_shutdown,
};

static const struct snd_soc_dapm_widget aic3254_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_LINE("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route omap3evm_aic3254_audio_routes[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},

	/* Line Out connected to LOLOUT, LOROUT */
	{"Ext Spk", NULL, "LOL"},
	{"Ext Spk", NULL, "LOR"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R)) */
	{"IN1_L", NULL, "Line In"},
	{"IN1_R", NULL, "Line In"},
};

#define OMAP3EVM_AIC3254_DAPM_ROUTE_NUM (sizeof(omap3evm_aic3254_audio_routes)/sizeof(struct snd_soc_dapm_route))
static const char *spk_function[] = {"Off", "On"};
static const char *jack_function[] = {"Off", "Headphone"};
static const struct soc_enum n810_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(jack_function), jack_function),
};

static int n810_get_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = n810_spk_func;

	return 0;
}

static int n810_set_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (n810_spk_func == ucontrol->value.integer.value[0])
		return 0;

	n810_spk_func = ucontrol->value.integer.value[0];
	n810_ext_control(codec);

	return 1;
}

static int n810_get_jack(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = n810_jack_func;

	return 0;
}

static int n810_set_jack(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (n810_jack_func == ucontrol->value.integer.value[0])
		return 0;

	n810_jack_func = ucontrol->value.integer.value[0];
	n810_ext_control(codec);

	return 1;
}

static const struct snd_kcontrol_new aic33_n810_controls[] = {
	SOC_ENUM_EXT("Speaker Function", n810_enum[0],
		     n810_get_spk, n810_set_spk),
	SOC_ENUM_EXT("Jack Function", n810_enum[1],
		     n810_get_jack, n810_set_jack),
};

static int omap3evm_aic3254_init(struct snd_soc_codec *codec)
{
	int i; 
	int err;



	/* Add N810 specific controls */
#if 0
	err = snd_soc_add_controls(codec, aic33_n810_controls,
				ARRAY_SIZE(aic33_n810_controls));
#endif
for (i = 0; i < ARRAY_SIZE(aic33_n810_controls); i++) {
        err =
            snd_ctl_add(codec->card,
                snd_soc_cnew(&aic33_n810_controls[i], codec,
                         NULL));
        if (err < 0) {
            printk("Invalid control\n");
            return err;
        }
    }



	/* Add N810 specific widgets */
	for (i = 0; i < ARRAY_SIZE(aic3254_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &aic3254_dapm_widgets[i]);

	/* Set up N810 specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, &omap3evm_aic3254_audio_routes[0], OMAP3EVM_AIC3254_DAPM_ROUTE_NUM);

	//snd_soc_dapm_sync_endpoints(codec);
	n810_ext_control(codec);

	return 0;
}

static struct snd_soc_dai_link omap3evm_dai = {
	.name = "TLV320AIC3254",
	.stream_name = "AIC3254",
	.codec_dai = &tlv320aic3254_dai,
	.init = omap3evm_aic3254_init,
	.cpu_dai = &omap_mcbsp_dai[0],
	.ops = &omap3evm_ops,
};

static struct snd_soc_card snd_soc_card_omap3evm = {
    .name = "OMAP3EVM",
    .platform = &omap_soc_platform,
    .dai_link = &omap3evm_dai,
    .num_links = 1,
};

static struct snd_soc_device omap3evm_snd_devdata = {
    .card = &snd_soc_card_omap3evm,
    .codec_dev = &soc_codec_dev_aic3254,
};

/* Audio private data */
static struct aic3254_setup_data omap3evm_aic3254_setup = {
	.i2c_address = 0x18,	//i2c address for tlv320aic3254 
};

static struct platform_device *omap3evm_snd_device;

static int __init omap3evm_init(void)
{
	int ret = 0;
	struct device *dev;

	omap3evm_snd_device = platform_device_alloc("soc-audio", -1);

	if (!omap3evm_snd_device)
		return -ENOMEM;

	platform_set_drvdata(omap3evm_snd_device, &omap3evm_snd_devdata);
	omap3evm_snd_devdata.dev = &omap3evm_snd_device->dev;
	*(unsigned int *)omap3evm_dai.cpu_dai->private_data = 1; /* McBSP2 */

	ret = platform_device_add(omap3evm_snd_device);

	if (ret)
		platform_device_put(omap3evm_snd_device);

	dev = &omap3evm_snd_device->dev;

	sys_clkout2 = clk_get(dev, "sys_clkout2");
	if (IS_ERR(sys_clkout2)) {
		printk("Could not get sys_clkout2\n");
		return -1;
	}

 sys_clkout2 = clk_get(dev, "sys_clkout2");
        if (IS_ERR(sys_clkout2)) {
                printk("Could not get sys_clkout2\n");
                return -1;
        }

    /*
 *      * Configure 12 MHz output on SYS_CLKOUT2. Therefore we must use
 *           * 96 MHz as its parent in order to get 12 MHz
 *                */
    cm_96m_ck = clk_get(dev, "cm_96m_fck");
    if (IS_ERR(cm_96m_ck)) {
        printk("Could not get func 96M clock\n");
        ret = -1;
        goto err1;
    }
    clk_set_parent(clkout2_src_ck, cm_96m_ck);
    clk_set_rate(sys_clkout2, 12000000);

    omap_writel(0x1a, 0x48004d70);

    clk_enable(sys_clkout2);


// 	omap_cfg_reg(AE22_3430_SYS_CLKOUT2);
// 
// 	omap_cfg_reg(Y21_3430_MCBSP1_CLKR);
// 	omap_cfg_reg(AA21_3430_MCBSP1_FSR);
// 	omap_cfg_reg(V21_3430_MCBSP1_DX);
// 	omap_cfg_reg(U21_3430_MCBSP1_DR);
// 	omap_cfg_reg(T21_3430_MCBSP1_CLKS);
// 	omap_cfg_reg(K26_3430_MCBSP1_FSX);
// 	omap_cfg_reg(W21_3430_MCBSP1_CLKX);

	return 0;
      err1:
	clk_put(sys_clkout2);

	return ret;
}

static void __exit omap3evm_exit(void)
{
	platform_device_add(omap3evm_snd_device);
}

module_init(omap3evm_init);
module_exit(omap3evm_exit);

MODULE_AUTHOR("Sandeep S Prabhu<sandeepsp@mistralsolutions.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3530 EVM");
MODULE_LICENSE("GPL");
