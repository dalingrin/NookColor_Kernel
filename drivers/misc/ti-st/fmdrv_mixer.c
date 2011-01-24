/*
 *  FM Driver for Connectivity chip of Texas Instruments.
 *
 *  This file manages FM driver's ALSA mixer controls.
 *
 *  The standard V4L2 subsystem provides limited V4L2 IOCTLs
 *  to perform FM operation. So, this module will expose
 *  some of mixer controls (ex., Band Selection,
 *  FM TX/RX Mode Switch, etc) via ALSA to user space.
 *
 *  Copyright (C) 2009 Texas Instruments
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/version.h>
#include <sound/core.h>
#include <sound/control.h>

#include "fmdrv.h"
#include "fmdrv_mixer.h"
#include "fmdrv_core.h"

static int fm_mixer_mode_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	static char *fm_modes[] = { "Off", "Tx", "Rx" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 3;
	if (uinfo->value.enumerated.item > 2)
		uinfo->value.enumerated.item = 2;
	strcpy(uinfo->value.enumerated.name,
	       fm_modes[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_mode_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned char current_fmmode;

	FMDRV_API_START();
	ret = fm_core_mode_get(&current_fmmode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = current_fmmode & 3;

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_mode_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	int changed;
	unsigned char mode;
	int ret;

	FMDRV_API_START();

	mode = ucontrol->value.integer.value[0] & 3;
	ret = fm_core_mode_set(mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}

	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_region_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	static char *region[] = { "Europe/US", "Japan" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	strcpy(uinfo->value.enumerated.name,
	       region[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_region_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	unsigned char region;
	int ret;

	FMDRV_API_START();
	ret = fm_core_region_get(&region);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = region & 0x1;

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_region_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned char region;
	int changed;

	FMDRV_API_START();

	region = ucontrol->value.integer.value[0] & 0x1;
	ret = fm_core_region_set(region);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_rfdepend_mute_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static char *region[] = { "Off", "On" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	strcpy(uinfo->value.enumerated.name,
	       region[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rfdepend_mute_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	unsigned char en_dis;
	int ret;

	FMDRV_API_START();

	ret = fm_core_rx_get_rfdepend_softmute(&en_dis);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = en_dis & 0x1;
	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rfdepend_mute_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	unsigned char en_dis;
	int changed, ret;

	FMDRV_API_START();

	en_dis = ucontrol->value.integer.value[0] & 0x1;
	ret = fm_core_rx_set_rfdepend_softmute(en_dis);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_rssi_level_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = -16;
	uinfo->value.integer.max = 15;

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rssi_level_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	unsigned short curr_rssi_lvl;
	int ret;
	FMDRV_API_START();

	ret = fm_core_rx_get_rssi_level(&curr_rssi_lvl);
	if (ret)
		return ret;

	ucontrol->value.integer.value[0] = curr_rssi_lvl;

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_rssi_threshold_info(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_info *uinfo)
{
	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = -16;
	uinfo->value.integer.max = 15;

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_rssi_threshold_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	short curr_rssi_threshold;
	int ret;
	FMDRV_API_START();

	ret = fm_core_rx_get_rssi_threshold(&curr_rssi_threshold);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}

	ucontrol->value.integer.value[0] = curr_rssi_threshold;

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_rssi_threshold_put(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	short rssi_threshold_toset;
	int changed, ret;

	FMDRV_API_START();

	rssi_threshold_toset = ucontrol->value.integer.value[0];
	ret = fm_core_rx_set_rssi_threshold(rssi_threshold_toset);
	if (ret)
		return ret;

	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_stereo_mono_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	static char *modes[] = { "Stereo", "Mono" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	strcpy(uinfo->value.enumerated.name,
	       modes[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_stereo_mono_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	unsigned short mode;
	int ret;

	FMDRV_API_START();

	ret = fm_core_rx_get_stereo_mono(&mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = mode & 0x1;
	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_stereo_mono_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned short mode;
	int changed;

	FMDRV_API_START();

	mode = ucontrol->value.integer.value[0] & 0x1;
	ret = fm_core_set_stereo_mono(mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_rx_deemphasis_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static char *filter_mode[] = { "50 us", "75 us" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	strcpy(uinfo->value.enumerated.name,
	       filter_mode[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_deemphasis_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	unsigned short mode;
	int ret;

	FMDRV_API_START();
	ret = fm_core_rx_get_deemphasis_mode(&mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = mode & 0x1;
	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_deemphasis_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned short mode;
	int changed;

	FMDRV_API_START();

	mode = ucontrol->value.integer.value[0] & 0x1;
	ret = fm_core_rx_set_deemphasis_mode(mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_rx_rds_switch_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static char *rds_mode[] = { "Off", "On" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	strcpy(uinfo->value.enumerated.name,
	       rds_mode[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_rds_switch_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	unsigned char rds_onoff;
	int ret;

	FMDRV_API_START();
	ret = fm_core_rx_get_rds_mode(&rds_onoff);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = rds_onoff & 0x1;
	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_rds_switch_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned char rds_onoff;
	int changed;

	FMDRV_API_START();

	rds_onoff = ucontrol->value.integer.value[0] & 0x1;
	ret = fm_core_set_rds_mode(rds_onoff);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_rx_rds_opmode_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	static char *rds_mode[] = { "RDS", "RDBS" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	strcpy(uinfo->value.enumerated.name,
	       rds_mode[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_rds_opmode_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	unsigned char rds_mode;
	int ret;

	FMDRV_API_START();
	ret = fm_core_rx_get_rds_system(&rds_mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = rds_mode & 0x1;
	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_rds_opmode_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned char rds_mode;
	int changed;

	FMDRV_API_START();

	rds_mode = ucontrol->value.integer.value[0] & 0x1;
	ret = fm_core_rx_set_rds_system(rds_mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static int fm_mixer_rx_af_switch_info(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_info *uinfo)
{
	static char *af_mode[] = { "Off", "On" };

	FMDRV_API_START();

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = 2;
	if (uinfo->value.enumerated.item > 1)
		uinfo->value.enumerated.item = 1;
	strcpy(uinfo->value.enumerated.name,
	       af_mode[uinfo->value.enumerated.item]);

	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_af_switch_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	unsigned char af_mode;
	int ret;

	FMDRV_API_START();

	ret = fm_core_rx_get_af_switch(&af_mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ucontrol->value.enumerated.item[0] = af_mode & 0x1;
	FMDRV_API_EXIT(0);
	return 0;
}

static int fm_mixer_rx_af_switch_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	unsigned char af_mode;
	int changed, ret;

	FMDRV_API_START();

	af_mode = ucontrol->value.integer.value[0] & 0x1;
	ret = fm_core_rx_set_af_switch(af_mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	changed = 1;

	FMDRV_API_EXIT(changed);
	return changed;
}

static struct snd_kcontrol_new snd_fm_controls[] = {
	{
	 .name = "Mode Switch",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_mode_info,
	 .get = fm_mixer_mode_get,
	 .put = fm_mixer_mode_put,
	 },
	{
	 .name = "Region Switch",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_region_info,
	 .get = fm_mixer_region_get,
	 .put = fm_mixer_region_put,
	 },
	{
	 .name = "RF Dependent Mute",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_rfdepend_mute_info,
	 .get = fm_mixer_rfdepend_mute_get,
	 .put = fm_mixer_rfdepend_mute_put,
	 },
	{
	 .name = "RSSI Level",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .access = SNDRV_CTL_ELEM_ACCESS_READ,
	 .info = fm_mixer_rssi_level_info,
	 .get = fm_mixer_rssi_level_get,
	 },
	{
	 .name = "RSSI Threshold",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_rx_rssi_threshold_info,
	 .get = fm_mixer_rx_rssi_threshold_get,
	 .put = fm_mixer_rx_rssi_threshold_put,
	 },
	{
	 .name = "Stereo/Mono",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_stereo_mono_info,
	 .get = fm_mixer_stereo_mono_get,
	 .put = fm_mixer_stereo_mono_put,
	 },
	{
	 .name = "De-emphasis Filter",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_rx_deemphasis_info,
	 .get = fm_mixer_rx_deemphasis_get,
	 .put = fm_mixer_rx_deemphasis_put,
	 },
	{
	 .name = "RDS Switch",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_rx_rds_switch_info,
	 .get = fm_mixer_rx_rds_switch_get,
	 .put = fm_mixer_rx_rds_switch_put,
	 },
	{
	 .name = "RDS Operation Mode",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_rx_rds_opmode_info,
	 .get = fm_mixer_rx_rds_opmode_get,
	 .put = fm_mixer_rx_rds_opmode_put,
	 },
	{
	 .name = "AF Switch",
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = fm_mixer_rx_af_switch_info,
	 .get = fm_mixer_rx_af_switch_get,
	 .put = fm_mixer_rx_af_switch_put,
	 },
};

int fm_mixer_init(struct fmdrv_ops *fmdev)
{
	int idx;
	int ret;

	FMDRV_API_START();

	/* Allocate new card for FM driver */
	fmdev->card = snd_card_new(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
				   THIS_MODULE, 0);
	if (!fmdev->card) {
		FM_DRV_ERR("No memory to allocate new card");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	fmdev->card->private_data = fmdev;

	/* Add FM mixer controls to the card */
	strcpy(fmdev->card->mixername, FM_DRV_MIXER_NAME);
	for (idx = 0; idx < ARRAY_SIZE(snd_fm_controls); idx++) {
		ret = snd_ctl_add(fmdev->card,
				  snd_ctl_new1(&snd_fm_controls[idx], fmdev));
		if (ret < 0) {
			snd_card_free(fmdev->card);
			FM_DRV_ERR("Failed to add mixer controls");
			FMDRV_API_EXIT(ret);
			return ret;
		}
	}

	/* Register FM card with ALSA */
	ret = snd_card_register(fmdev->card);
	if (ret) {
		snd_card_free(fmdev->card);
		FM_DRV_ERR("Failed to register new card");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	strcpy(fmdev->card->driver, FM_DRV_NAME);
	strcpy(fmdev->card->shortname, FM_DRV_CARD_SHORT_NAME);
	sprintf(fmdev->card->longname, FM_DRV_CARD_LONG_NAME);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_mixer_deinit(struct fmdrv_ops *fmdev)
{
	FMDRV_API_START();

	/* Unregister FM card from ALSA */
	snd_card_free(fmdev->card);

	FMDRV_API_EXIT(0);
	return 0;
}
