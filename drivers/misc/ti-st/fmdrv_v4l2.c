/*
 *  FM Driver for Connectivity chip of Texas Instruments.
 *
 *  This file provides interfaces to V4L2 subsystem.
 *
 *  This module registers with V4L2 subsystem as Radio
 *  data system interface (/dev/radio). During the registration,
 *  it will expose two set of function pointers to V4L2 subsystem.
 *
 *    1) File operation related API (open, close, read, write, poll...etc).
 *    2) Set of V4L2 IOCTL complaint API.
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

#include "fmdrv.h"
#include "fmdrv_v4l2.h"
#include "fmdrv_core.h"

static unsigned char radio_disconnected;

/* Query control */
static struct v4l2_queryctrl fmdrv_v4l2_queryctrl[] = {
	{
	 .id = V4L2_CID_AUDIO_VOLUME,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Volume",
	 .minimum = FM_RX_VOLUME_MIN,
	 .maximum = FM_RX_VOLUME_MAX,
	 .step = 1,
	 .default_value = FM_DEFAULT_RX_VOLUME,
	 },
	{
	 .id = V4L2_CID_AUDIO_BALANCE,
	 .flags = V4L2_CTRL_FLAG_DISABLED,
	 },
	{
	 .id = V4L2_CID_AUDIO_BASS,
	 .flags = V4L2_CTRL_FLAG_DISABLED,
	 },
	{
	 .id = V4L2_CID_AUDIO_TREBLE,
	 .flags = V4L2_CTRL_FLAG_DISABLED,
	 },
	{
	 .id = V4L2_CID_AUDIO_MUTE,
	 .type = V4L2_CTRL_TYPE_BOOLEAN,
	 .name = "Mute",
	 .minimum = 0,
	 .maximum = 2,
	 .step = 1,
	 .default_value = FM_MUTE_OFF,
	 },
	{
	 .id = V4L2_CID_AUDIO_LOUDNESS,
	 .flags = V4L2_CTRL_FLAG_DISABLED,
	 },
};

/* -- V4L2 RADIO (/dev/radioX) device file operation interfaces --- */

/* Read RDS data */
static ssize_t fm_v4l2_fops_read(struct file *file, char __user * buf,
				 size_t count, loff_t *ppos)
{
	unsigned char rds_mode;
	int ret, noof_bytes_copied;
	FMDRV_API_START();

	if (!radio_disconnected) {
		FM_DRV_ERR("FM device is already disconnected\n");
		FMDRV_API_EXIT(-EIO);
		return -EIO;
	}
	/* Turn on RDS mode , if it is disabled */
	ret = fm_core_rx_get_rds_mode(&rds_mode);
	if (ret) {
		FM_DRV_ERR("Unable to read current rds mode");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (rds_mode == FM_RX_RDS_DISABLE) {
		ret = fm_core_set_rds_mode(FM_RX_RDS_ENABLE);
		if (ret < 0) {
			FM_DRV_ERR("Unable to enable rds mode");
			FMDRV_API_EXIT(ret);
			return ret;
		}
	}
	/* Copy RDS data from internal buffer to user buffer */
	noof_bytes_copied =
	    fm_core_transfer_rds_from_internal_buff(file, buf, count);

	FMDRV_API_EXIT(noof_bytes_copied);
	return noof_bytes_copied;
}

/* Write RDS data */
static ssize_t fm_v4l2_fops_write(struct file *file, const char __user * buf,
				  size_t count, loff_t *ppos)
{
	struct tx_rds rds;
	int ret;
	FMDRV_API_START();

	ret = copy_from_user(&rds, buf, sizeof(rds));
	FM_DRV_DBG("(%d)type: %d, text %s, af %d",
		   ret, rds.text_type, rds.text, rds.af_freq);

	fm_core_tx_set_radio_text(rds.text, rds.text_type);
	fm_core_tx_set_af(rds.af_freq);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Poll RDS data */
static unsigned int fm_v4l2_fops_poll(struct file *file,
				      struct poll_table_struct *pts)
{
	int ret;
	FMDRV_API_START();

	ret = fm_core_is_rds_data_available(file, pts);
	if (!ret) {
		FMDRV_API_EXIT(POLLIN | POLLRDNORM);
		return POLLIN | POLLRDNORM;
	}

	FMDRV_API_EXIT(0);
	return 0;
}

/* File Open */
static int fm_v4l2_fops_open(struct file *file)
{
	int ret;

	FMDRV_API_START();

	/* Don't allow multiple open */
	if (radio_disconnected) {
		FM_DRV_ERR("FM device is already opened\n");
		FMDRV_API_EXIT(-EBUSY);
		return -EBUSY;
	}

	/* Request FM Core to link with FM ST */
	ret = fm_core_setup_transport();
	if (ret) {
		FM_DRV_ERR("Unable to setup FM Core transport");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Initialize FM Core */
	ret = fm_core_prepare();
	if (ret) {
		FM_DRV_ERR("Unable to prepare FM CORE");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	FM_DRV_DBG("Load FM RX firmware..");
	/* By default load FM RX firmware */
	ret = fm_core_mode_set(FM_MODE_RX);
	if (ret) {
		FM_DRV_ERR("Unable to load FM RX firmware");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	radio_disconnected = 1;
	FM_DRV_DBG("FM CORE is ready");

	FMDRV_API_EXIT(0);
	return 0;
}

/* File Release */
static int fm_v4l2_fops_release(struct file *file)
{
	int ret;

	FMDRV_API_START();

	if (!radio_disconnected) {
		FM_DRV_DBG("FM device already closed,close called again?");
		FMDRV_API_EXIT(0);
		return 0;
	}

	FM_DRV_DBG("Turning off..");
	ret = fm_core_mode_set(FM_MODE_OFF);
	if (ret) {
		FM_DRV_ERR("Unable to turn off the chip");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Request FM Core to unlink from ST driver */
	ret = fm_core_release();
	if (ret) {
		FM_DRV_ERR("FM CORE release failed");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	/* Release FM Core transport */
	ret = fm_core_release_transport();
	if (ret) {
		FM_DRV_ERR("Unable to setup FM Core transport");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	radio_disconnected = 0;
	FM_DRV_DBG("FM CORE released successfully");

	FMDRV_API_EXIT(0);
	return 0;
}

/* V4L2 RADIO (/dev/radioX) device IOCTL interfaces */

/* Query device capabilities */
static int fm_v4l2_vidioc_querycap(struct file *file, void *priv,
				   struct v4l2_capability *capability)
{
	FMDRV_API_START();

	strlcpy(capability->driver, FM_DRV_NAME, sizeof(capability->driver));
	strlcpy(capability->card, FM_DRV_CARD_SHORT_NAME,
		sizeof(capability->card));
	sprintf(capability->bus_info, "UART");
	capability->version = FM_DRV_RADIO_VERSION;
	capability->capabilities = V4L2_CAP_HW_FREQ_SEEK | V4L2_CAP_TUNER |
	    V4L2_CAP_RADIO | V4L2_CAP_READWRITE | V4L2_CAP_AUDIO;
	FMDRV_API_EXIT(0);
	return 0;
}

/* Enumerate control items */
static int fm_v4l2_vidioc_queryctrl(struct file *file, void *priv,
				    struct v4l2_queryctrl *qc)
{
	int index;
	int ret;

	FMDRV_API_START();

	ret = -EINVAL;
	if (qc->id < V4L2_CID_BASE) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Search control ID and copy its properties */
	for (index = 0; index < ARRAY_SIZE(fmdrv_v4l2_queryctrl); index++) {
		if (qc->id && qc->id == fmdrv_v4l2_queryctrl[index].id) {
			memcpy(qc, &(fmdrv_v4l2_queryctrl[index]), sizeof(*qc));
			ret = 0;
			break;
		}
	}
	FMDRV_API_EXIT(ret);
	return ret;
}

/* Get the value of a control */
static int fm_v4l2_vidioc_g_ctrl(struct file *file, void *priv,
				 struct v4l2_control *ctrl)
{
	int ret;
	unsigned short curr_vol;
	unsigned char curr_mute_mode;

	FMDRV_API_START();

	switch (ctrl->id) {

	case V4L2_CID_AUDIO_MUTE:	/* get mute mode */
		ret = fm_core_rx_get_mute_mode(&curr_mute_mode);
		if (ret < 0) {
			FMDRV_API_EXIT(ret);
			return ret;
		}
		ctrl->value = curr_mute_mode;
		break;

	case V4L2_CID_AUDIO_VOLUME:	/* get volume */
		ret = fm_core_rx_get_volume(&curr_vol);
		if (ret < 0) {
			FMDRV_API_EXIT(ret);
			return ret;
		}
		ctrl->value = curr_vol;
		break;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Set the value of a control */
static int fm_v4l2_vidioc_s_ctrl(struct file *file, void *priv,
				 struct v4l2_control *ctrl)
{
	int ret;
	FMDRV_API_START();

	switch (ctrl->id) {

	case V4L2_CID_AUDIO_MUTE:	/* set mute */
		ret = fm_core_set_mute_mode((unsigned char)ctrl->value);
		if (ret < 0) {
			FMDRV_API_EXIT(ret);
			return ret;
		}
		break;

	case V4L2_CID_AUDIO_VOLUME:	/* set volume */
		ret = fm_core_rx_set_volume((unsigned short)ctrl->value);
		if (ret < 0) {
			FMDRV_API_EXIT(ret);
			return ret;
		}
		break;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Get audio attributes */
static int fm_v4l2_vidioc_g_audio(struct file *file, void *priv,
				  struct v4l2_audio *audio)
{
	FMDRV_API_START();

	memset(audio, 0, sizeof(*audio));
	audio->index = 0;
	strcpy(audio->name, "Radio");
	audio->capability = V4L2_AUDCAP_STEREO;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Set audio attributes */
static int fm_v4l2_vidioc_s_audio(struct file *file, void *priv,
				  struct v4l2_audio *audio)
{
	FMDRV_API_START();

	if (audio->index != 0) {
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Get tuner attributes */
static int fm_v4l2_vidioc_g_tuner(struct file *file, void *priv,
				  struct v4l2_tuner *tuner)
{
	unsigned int bottom_frequency;
	unsigned int top_frequency;
	unsigned short stereo_mono_mode;
	unsigned short rssilvl;
	int ret;

	FMDRV_API_START();

	if (tuner->index != 0) {
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	ret =
	    fm_core_rx_get_currband_lowhigh_freq(&bottom_frequency,
						 &top_frequency);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ret = fm_core_rx_get_stereo_mono(&stereo_mono_mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	ret = fm_core_rx_get_rssi_level(&rssilvl);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	strcpy(tuner->name, "FM");
	tuner->type = V4L2_TUNER_RADIO;
	/* Store rangelow and rangehigh freq in unit of 62.5 KHz */
	tuner->rangelow = (bottom_frequency * 10000) / 625;
	tuner->rangehigh = (top_frequency * 10000) / 625;
	tuner->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	tuner->capability = V4L2_TUNER_CAP_STEREO | V4L2_TUNER_CAP_LOW;
	tuner->audmode = (stereo_mono_mode ?
			  V4L2_TUNER_MODE_MONO : V4L2_TUNER_MODE_STEREO);

	/* Actual rssi value lies in between -128 to +127.
	 * Convert this range from 0 to 255 by adding +128
	 */
	rssilvl += 128;

	/* Return signal strength value should be within 0 to 65535.
	 * Find out correct signal radio by multiplying (65535/255) = 257
	 */
	tuner->signal = rssilvl * 257;
	tuner->afc = 0;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Set tuner attributes */
static int fm_v4l2_vidioc_s_tuner(struct file *file, void *priv,
				  struct v4l2_tuner *tuner)
{
	unsigned short mode;
	int ret;

	FMDRV_API_START();

	if ((tuner->index != 0) ||
	    (tuner->audmode != V4L2_TUNER_MODE_MONO &&
	     tuner->audmode != V4L2_TUNER_MODE_STEREO)) {
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	/* Map V4L2 stereo/mono macro to our local stereo/mono macro */
	mode = (tuner->audmode == V4L2_TUNER_MODE_STEREO) ?
	    FM_STEREO_MODE : FM_MONO_MODE;
	ret = fm_core_set_stereo_mono(mode);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}

	FMDRV_API_EXIT(0);
	return 0;
}

/* Get tuner or modulator radio frequency */
static int fm_v4l2_vidioc_g_frequency(struct file *file, void *priv,
				      struct v4l2_frequency *freq)
{
	int ret;
	FMDRV_API_START();

	ret = fm_core_get_frequency(&freq->frequency);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}

	FMDRV_API_EXIT(0);
	return 0;
}

/* Set tuner or modulator radio frequency */
static int fm_v4l2_vidioc_s_frequency(struct file *file, void *priv,
				      struct v4l2_frequency *freq)
{
	int ret;
	FMDRV_API_START();

	ret = fm_core_set_frequency(freq->frequency);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Set hardware frequency seek */
static int fm_v4l2_vidioc_s_hw_freq_seek(struct file *file, void *priv,
					 struct v4l2_hw_freq_seek *seek)
{
	int ret;

	FMDRV_API_START();

	ret = fm_core_rx_seek(seek->seek_upward, seek->wrap_around);
	if (ret) {
		FMDRV_API_EXIT(ret);
		return ret;
	}

	FMDRV_API_EXIT(0);
	return 0;
}

static const struct v4l2_file_operations fm_drv_fops = {
	.owner = THIS_MODULE,
	.read = fm_v4l2_fops_read,
	.write = fm_v4l2_fops_write,
	.poll = fm_v4l2_fops_poll,
	.ioctl = video_ioctl2,
	.open = fm_v4l2_fops_open,
	.release = fm_v4l2_fops_release,
};

static const struct v4l2_ioctl_ops fm_drv_ioctl_ops = {
	.vidioc_querycap = fm_v4l2_vidioc_querycap,
	.vidioc_queryctrl = fm_v4l2_vidioc_queryctrl,
	.vidioc_g_ctrl = fm_v4l2_vidioc_g_ctrl,
	.vidioc_s_ctrl = fm_v4l2_vidioc_s_ctrl,
	.vidioc_g_audio = fm_v4l2_vidioc_g_audio,
	.vidioc_s_audio = fm_v4l2_vidioc_s_audio,
	.vidioc_g_tuner = fm_v4l2_vidioc_g_tuner,
	.vidioc_s_tuner = fm_v4l2_vidioc_s_tuner,
	.vidioc_g_frequency = fm_v4l2_vidioc_g_frequency,
	.vidioc_s_frequency = fm_v4l2_vidioc_s_frequency,
	.vidioc_s_hw_freq_seek = fm_v4l2_vidioc_s_hw_freq_seek,
};

/*
 * V4L2 RADIO device parent structure
 */
static struct video_device fm_viddev_template = {
	.fops = &fm_drv_fops,
	.ioctl_ops = &fm_drv_ioctl_ops,
	.name = FM_DRV_NAME,
	.release = video_device_release,
};

int fm_v4l2_init_video_device(struct fmdrv_ops *fmdev)
{
	FMDRV_API_START();

	/* Allocate new video device */
	fmdev->v4l2dev = video_device_alloc();
	if (!fmdev->v4l2dev) {
		FM_DRV_ERR("Can't allocate video device");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}

	/* Setup FM driver's V4L2 properties */
	memcpy(fmdev->v4l2dev, &fm_viddev_template, sizeof(fm_viddev_template));

	video_set_drvdata(fmdev->v4l2dev, fmdev);

	/* Register with V4L2 subsystem as RADIO device */
	if (video_register_device(fmdev->v4l2dev, VFL_TYPE_RADIO, 0)) {
		video_device_release(fmdev->v4l2dev);
		fmdev->v4l2dev = NULL;

		FM_DRV_ERR("Could not register video device");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

int fm_v4l2_deinit_video_device(struct fmdrv_ops *fmdev)
{
	FMDRV_API_START();

	/* Unregister RADIO device from V4L2 subsystem */
	video_unregister_device(fmdev->v4l2dev);

	FMDRV_API_EXIT(0);
	return 0;
}
