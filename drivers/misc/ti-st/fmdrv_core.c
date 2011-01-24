/*
 *  FM Driver for Connectivity chip of Texas Instruments.
 *
 *  This sub-module of FM driver does,
 *  1) Forming group of Channel-8 commands to perform particular
 *     functionality (ex., frequency set require more than
 *     one Channel-8 command to be sent to the chip).
 *  2) Sending each Channel-8 command to the chip and reading
 *     response back over Shared Transport.
 *  3) Managing TX and RX Queues and Tasklets
 *  4) Handling FM Interrupt packet and taking appropriate action.
 *  5) Loading FM firmware to the chip (common, TX, and RX
 *     firmware files based on mode selection)
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

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include "fmdrv.h"
#include "fmdrv_v4l2.h"
#include "fmdrv_core.h"
#include "fmdrv_st.h"
#include "fmdrv_mixer.h"
#include "fmdrv_chr.h"

/* -- FM chip register table -- */
static struct fm_reg_table fm_reg_info[] = {
	/* ----- FM RX registers -------*/
	/* opcode, type(rd/wr), reg name */
	{0x00, REG_RD, "STEREO_GET"},
	{0x01, REG_RD, "RSSI_LVL_GET"},
	{0x02, REG_RD, "IF_COUNT_GET"},
	{0x03, REG_RD, "FLAG_GET"},
	{0x04, REG_RD, "RDS_SYNC_GET"},
	{0x05, REG_RD, "RDS_DATA_GET"},
	{0x0a, REG_WR, "FREQ_SET"},
	{0x0a, REG_RD, "FREQ_GET"},
	{0x0b, REG_WR, "AF_FREQ_SET"},
	{0x0b, REG_RD, "AF_FREQ_GET"},
	{0x0c, REG_WR, "MOST_MODE_SET"},
	{0x0c, REG_RD, "MOST_MODE_GET"},
	{0x0d, REG_WR, "MOST_BLEND_SET"},
	{0x0d, REG_RD, "MOST_BLEND_GET"},
	{0x0e, REG_WR, "DEMPH_MODE_SET"},
	{0x0e, REG_RD, "DEMPH_MODE_GET"},
	{0x0f, REG_WR, "SEARCH_LVL_SET"},
	{0x0f, REG_RD, "SEARCH_LVL_GET"},
	{0x10, REG_WR, "RX_BAND_SET"},
	{0x10, REG_RD, "RX_BAND_GET"},
	{0x11, REG_WR, "MUTE_STATUS_SET"},
	{0x11, REG_RD, "MUTE_STATUS_GET"},
	{0x12, REG_WR, "RDS_PAUSE_LVL_SET"},
	{0x12, REG_RD, "RDS_PAUSE_LVL_GET"},
	{0x13, REG_WR, "RDS_PAUSE_DUR_SET"},
	{0x13, REG_RD, "RDS_PAUSE_DUR_GET"},
	{0x14, REG_WR, "RDS_MEM_SET"},
	{0x14, REG_RD, "RDS_MEM_GET"},
	{0x15, REG_WR, "RDS_BLK_B_SET"},
	{0x15, REG_RD, "RDS_BLK_B_GET"},
	{0x16, REG_WR, "RDS_MSK_B_SET"},
	{0x16, REG_RD, "RDS_MSK_B_GET"},
	{0x17, REG_WR, "RDS_PI_MASK_SET"},
	{0x17, REG_RD, "RDS_PI_MASK_GET"},
	{0x18, REG_WR, "RDS_PI_SET"},
	{0x18, REG_RD, "RDS_PI_GET"},
	{0x19, REG_WR, "RDS_SYSTEM_SET"},
	{0x19, REG_RD, "RDS_SYSTEM_GET"},
	{0x1a, REG_WR, "INT_MASK_SET"},
	{0x1a, REG_RD, "INT_MASK_GET"},
	{0x1b, REG_WR, "SRCH_DIR_SET"},
	{0x1b, REG_RD, "SRCH_DIR_GET"},
	{0x1c, REG_WR, "VOLUME_SET"},
	{0x1c, REG_RD, "VOLUME_GET"},
	{0x1d, REG_WR, "AUDIO_ENABLE(SET)"},
	{0x1d, REG_RD, "AUDIO_ENABLE(GET)"},
	{0x1e, REG_WR, "PCM_MODE_SET"},
	{0x1e, REG_RD, "PCM_MODE_SET"},
	{0x1f, REG_WR, "I2S_MD_CFG_SET"},
	{0x1f, REG_RD, "I2S_MD_CFG_GET"},
	{0x20, REG_WR, "POWER_SET"},
	{0x20, REG_RD, "POWER_GET"},
	{0x21, REG_WR, "INTx_CONFIG_SET"},
	{0x21, REG_RD, "INTx_CONFIG_GET"},
	{0x22, REG_WR, "PULL_EN_SET"},
	{0x22, REG_RD, "PULL_EN_GET"},
	{0x23, REG_WR, "HILO_SET"},
	{0x23, REG_RD, "HILO_GET"},
	{0x24, REG_WR, "SWITCH2FREF"},
	{0x25, REG_WR, "FREQ_DRIFT_REP"},
	{0x28, REG_RD, "PCE_GET"},
	{0x29, REG_RD, "FIRM_VER_GET"},
	{0x2a, REG_RD, "ASIC_VER_GET"},
	{0x2b, REG_RD, "ASIC_ID_GET"},
	{0x2c, REG_RD, "MAIN_ID_GET"},
	{0x2d, REG_WR, "TUNER_MODE_SET"},
	{0x2e, REG_WR, "STOP_SEARCH"},
	{0x2f, REG_WR, "RDS_CNTRL_SET"},
	{0x64, REG_WR, "WR_HW_REG"},
	{0x65, REG_WR, "CODE_DOWNLOAD"},
	{0x66, REG_WR, "RESET"},
	{0xfe, REG_WR, "FM_POWER_MODE(SET)"},
	{0xff, REG_RD, "FM_INTERRUPT"},

	/* --- FM TX registers ------ */
	{0x37, REG_WR, "CHANL_SET"},
	{0x37, REG_RD, "CHANL_GET"},
	{0x38, REG_WR, "CHANL_BW_SET"},
	{0x38, REG_RD, "CHANL_BW_GET"},
	{0x87, REG_WR, "REF_SET"},
	{0x87, REG_RD, "REF_GET"},
	{0x5a, REG_WR, "POWER_ENB_SET"},
	{0x3a, REG_WR, "POWER_ATT_SET"},
	{0x3a, REG_RD, "POWER_ATT_GET"},
	{0x3b, REG_WR, "POWER_LEL_SET"},
	{0x3b, REG_RD, "POWER_LEL_GET"},
	{0x3c, REG_WR, "AUDIO_DEV_SET"},
	{0x3c, REG_RD, "AUDIO_DEV_GET"},
	{0x3d, REG_WR, "PILOT_DEV_SET"},
	{0x3d, REG_RD, "PILOT_DEV_GET"},
	{0x3e, REG_WR, "RDS_DEV_SET"},
	{0x3e, REG_RD, "RDS_DEV_GET"},
	{0x5b, REG_WR, "PUPD_SET"},
	{0x3f, REG_WR, "AUDIO_IO_SET"},
	{0x40, REG_WR, "PREMPH_SET"},
	{0x40, REG_RD, "PREMPH_GET"},
	{0x41, REG_WR, "TX_BAND_SET"},
	{0x41, REG_RD, "TX_BAND_GET"},
	{0x42, REG_WR, "MONO_SET"},
	{0x42, REG_RD, "MONO_GET"},
	{0x5C, REG_WR, "MUTE"},
	{0x43, REG_WR, "MPX_LMT_ENABLE"},
	{0x06, REG_RD, "LOCK_GET"},
	{0x5d, REG_WR, "REF_ERR_SET"},
	{0x44, REG_WR, "PI_SET"},
	{0x44, REG_RD, "PI_GET"},
	{0x45, REG_WR, "TYPE_SET"},
	{0x45, REG_RD, "TYPE_GET"},
	{0x46, REG_WR, "PTY_SET"},
	{0x46, REG_RD, "PTY_GET"},
	{0x47, REG_WR, "AF_SET"},
	{0x47, REG_RD, "AF_GET"},
	{0x48, REG_WR, "DISPLAY_SIZE_SET"},
	{0x48, REG_RD, "DISPLAY_SIZE_GET"},
	{0x49, REG_WR, "RDS_MODE_SET"},
	{0x49, REG_RD, "RDS_MODE_GET"},
	{0x4a, REG_WR, "DISPLAY_MODE_SET"},
	{0x4a, REG_RD, "DISPLAY_MODE_GET"},
	{0x62, REG_WR, "LENGHT_SET"},
	{0x4b, REG_RD, "LENGHT_GET"},
	{0x4c, REG_WR, "TOGGLE_AB_SET"},
	{0x4c, REG_RD, "TOGGLE_AB_GET"},
	{0x4d, REG_WR, "RDS_REP_SET"},
	{0x4d, REG_RD, "RDS_REP_GET"},
	{0x63, REG_WR, "RDS_DATA_SET"},
	{0x5e, REG_WR, "RDS_DATA_ENB"},
	{0x4e, REG_WR, "TA_SET"},
	{0x4e, REG_RD, "TA_GET"},
	{0x4f, REG_WR, "TP_SET"},
	{0x4f, REG_RD, "TP_GET"},
	{0x50, REG_WR, "DI_SET"},
	{0x50, REG_RD, "DI_GET"},
	{0x51, REG_WR, "MS_SET"},
	{0x51, REG_RD, "MS_GET"},
	{0x52, REG_WR, "PS_SCROLL_SPEED_SET"},
	{0x52, REG_RD, "PS_SCROLL_SPEED_GET"},
};

/* Region info */
static struct region_info region_configs[] = {
	/* Europe/US */
	{
	 .channel_spacing = 50,	/* 50 KHz */
	 .bottom_frequency = 87500,	/* 87.5 MHz */
	 .top_frequency = 108000,	/* 108 MHz */
	 .region_index = 0,
	 },
	/* Japan */
	{
	 .channel_spacing = 50,	/* 50 KHz */
	 .bottom_frequency = 76000,	/* 76 MHz */
	 .top_frequency = 90000,	/* 90 MHz */
	 .region_index = 1,
	 },
};

/* Band selection */
static unsigned char default_radio_region;	/* Europe/US */
module_param(default_radio_region, byte, 0);
MODULE_PARM_DESC(default_radio_region, "Region: 0=Europe/US, 1=Japan");

/* RDS buffer blocks */
static unsigned int default_rds_buf = 300;
module_param(default_rds_buf, uint, 0444);
MODULE_PARM_DESC(rds_buf, "RDS buffer entries");

/* FM irq handlers forward declaration */
static void fm_core_irq_send_flag_getcmd(void);
static void fm_core_irq_handle_flag_getcmd_resp(void);
static void fm_core_irq_handle_hw_malfunction(void);
static void fm_core_irq_handle_rds_start(void);
static void fm_core_irq_send_rdsdata_getcmd(void);
static void fm_core_irq_handle_rdsdata_getcmd_resp(void);
static void fm_core_irq_handle_rds_finish(void);
static void fm_core_irq_handle_tune_op_ended(void);
static void fm_core_irq_handle_power_enb(void);
static void fm_core_irq_handle_low_rssi_start(void);
static void fm_core_irq_afjump_set_pi(void);
static void fm_core_irq_handle_set_pi_resp(void);
static void fm_core_irq_afjump_set_pimask(void);
static void fm_core_irq_handle_set_pimask_resp(void);
static void fm_core_irq_afjump_setfreq(void);
static void fm_core_irq_handle_setfreq_resp(void);
static void fm_core_irq_afjump_enableint(void);
static void fm_core_irq_afjump_enableint_resp(void);
static void fm_core_irq_start_afjump(void);
static void fm_core_irq_handle_start_afjump_resp(void);
static void fm_core_irq_afjump_rd_freq(void);
static void fm_core_irq_afjump_rd_freq_resp(void);
static void fm_core_irq_handle_low_rssi_finish(void);
static void fm_core_irq_send_intmsk_cmd(void);
static void fm_core_irq_handle_intmsk_cmd_resp(void);

/* When FM core receives interrupt packet , following handlers
 * will be executed one after another to service the interrupt(s) */

/* Interrupt handler index */
enum fm_irq_handler_index{

	FM_SEND_FLAG_GETCMD_INDEX,
	FM_HANDLE_FLAG_GETCMD_RESP_INDEX,

	/* HW malfunction irq handler */
	FM_HW_MAL_FUNC_INDEX,

	/* RDS threshold reached irq handler */
	FM_RDS_START_INDEX,
	FM_RDS_SEND_RDS_GETCMD_INDEX,
	FM_RDS_HANDLE_RDS_GETCMD_RESP_INDEX,
	FM_RDS_FINISH_INDEX,

	/* Tune operation ended irq handler */
	FM_HW_TUNE_OP_ENDED_INDEX,

	/* TX power enable irq handler */
	FM_HW_POWER_ENB_INDEX,

	/* Low RSSI irq handler */
	FM_LOW_RSSI_START_INDEX,
	FM_AF_JUMP_SETPI_INDEX,
	FM_AF_JUMP_HANDLE_SETPI_RESP_INDEX,
	FM_AF_JUMP_SETPI_MASK_INDEX,
	FM_AF_JUMP_HANDLE_SETPI_MASK_RESP_INDEX,
	FM_AF_JUMP_SET_AF_FREQ_INDEX,
	FM_AF_JUMP_HENDLE_SET_AFFREQ_RESP_INDEX,
	FM_AF_JUMP_ENABLE_INT_INDEX,
	FM_AF_JUMP_ENABLE_INT_RESP_INDEX,
	FM_AF_JUMP_START_AFJUMP_INDEX,
	FM_AF_JUMP_HANDLE_START_AFJUMP_RESP_INDEX,
	FM_AF_JUMP_RD_FREQ_INDEX,
	FM_AF_JUMP_RD_FREQ_RESP_INDEX,
	FM_LOW_RSSI_FINISH_INDEX,

	/* Interrupt process post action */
	FM_SEND_INTMSK_CMD_INDEX,
	FM_HANDLE_INTMSK_CMD_RESP_INDEX,
};

/* FM interrupt handler table */
static Int_Handler_ProtoType g_IntHandlerTable[] = {
	fm_core_irq_send_flag_getcmd,
	fm_core_irq_handle_flag_getcmd_resp,

	/* HW malfunction irq handler */
	fm_core_irq_handle_hw_malfunction,

	/* RDS threshold reached irq handler */
	fm_core_irq_handle_rds_start,
	fm_core_irq_send_rdsdata_getcmd,
	fm_core_irq_handle_rdsdata_getcmd_resp,
	fm_core_irq_handle_rds_finish,

	/* Tune operation ended irq handler */
	fm_core_irq_handle_tune_op_ended,

	/* TX power enable irq handler */
	fm_core_irq_handle_power_enb,

	/* Low RSSI irq handler */
	fm_core_irq_handle_low_rssi_start,
	fm_core_irq_afjump_set_pi,
	fm_core_irq_handle_set_pi_resp,
	fm_core_irq_afjump_set_pimask,
	fm_core_irq_handle_set_pimask_resp,
	fm_core_irq_afjump_setfreq,
	fm_core_irq_handle_setfreq_resp,
	fm_core_irq_afjump_enableint,
	fm_core_irq_afjump_enableint_resp,
	fm_core_irq_start_afjump,
	fm_core_irq_handle_start_afjump_resp,
	fm_core_irq_afjump_rd_freq,
	fm_core_irq_afjump_rd_freq_resp,
	fm_core_irq_handle_low_rssi_finish,

	/* Interrupt process post action */
	fm_core_irq_send_intmsk_cmd,
	fm_core_irq_handle_intmsk_cmd_resp
};

static struct fmdrv_ops *fmdev;

#ifdef FM_DUMP_TXRX_PKT

 /* To dump outgoing FM Channel-8 packets */
inline void dump_tx_skb_data(struct sk_buff *skb)
{
	int len, len_org;
	char index;
	struct fm_cmd_msg_hdr *cmd_hdr;

	cmd_hdr = (struct fm_cmd_msg_hdr *)skb->data;
	printk(KERN_INFO "<<%shdr:%02x len:%02x opcode:%02x type:%s dlen:%02x",
	       fm_cb(skb)->completion ? " " : "*", cmd_hdr->header,
	       cmd_hdr->len, cmd_hdr->fm_opcode,
	       cmd_hdr->rd_wr ? "RD" : "WR", cmd_hdr->dlen);

	len_org = skb->len - FM_CMD_MSG_HDR_SIZE;
	if (len_org > 0) {
		printk("\n   data(%d): ", cmd_hdr->dlen);
		len = min(len_org, 14);
		for (index = 0; index < len; index++)
			printk("%x ",
			       skb->data[FM_CMD_MSG_HDR_SIZE + index]);
		printk("%s", (len_org > 14) ? ".." : "");
	}
	printk("\n");
}

 /* To dump incoming FM Channel-8 packets */
inline void dump_rx_skb_data(struct sk_buff *skb)
{
	int len, len_org;
	char index;
	struct fm_event_msg_hdr *evt_hdr;

	evt_hdr = (struct fm_event_msg_hdr *)skb->data;
	printk(KERN_INFO ">> hdr:%02x len:%02x sts:%02x numhci:%02x "
	    "opcode:%02x type:%s dlen:%02x", evt_hdr->header, evt_hdr->len,
	    evt_hdr->status, evt_hdr->num_fm_hci_cmds, evt_hdr->fm_opcode,
	    (evt_hdr->rd_wr) ? "RD" : "WR", evt_hdr->dlen);

	len_org = skb->len - FM_EVT_MSG_HDR_SIZE;
	if (len_org > 0) {
		printk("\n   data(%d): ", evt_hdr->dlen);
		len = min(len_org, 14);
		for (index = 0; index < len; index++)
			printk("%x ",
			       skb->data[FM_EVT_MSG_HDR_SIZE + index]);
		printk("%s", (len_org > 14) ? ".." : "");
	}
	printk("\n");
}
#endif

/* Queues FM Channel-8 packet to FM TX queue and schedules FM TX tasklet for
 * transmission */
static int __fm_core_send_cmd(unsigned char fmreg_index, void *payload,
			      int payload_len,
			      struct completion *wait_completion)
{
	struct sk_buff *skb;
	struct fm_cmd_msg_hdr *cmd_hdr;
	int size;

	FMDRV_API_START();

	if (fmreg_index >= FM_REG_MAX_ENTRIES) {
		FM_DRV_ERR("Invalid fm register index");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	if (test_bit(FM_FIRMWARE_DW_INPROGRESS, &fmdev->flag) &&
			payload == NULL) {
		FM_DRV_ERR("Payload data is NULL during firmware download");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	if (!test_bit(FM_FIRMWARE_DW_INPROGRESS, &fmdev->flag))
		size =
		    FM_CMD_MSG_HDR_SIZE + ((payload == NULL) ? 0 : payload_len);
	else
		size = payload_len;

	/* Allocate memory for new packet */
	skb = alloc_skb(size, GFP_ATOMIC);
	if (!skb) {
		FM_DRV_ERR("No memory to create new SKB");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	/* Don't fill FM header info for the commands which come from
	 * FM firmware file */
	if (!test_bit(FM_FIRMWARE_DW_INPROGRESS, &fmdev->flag) ||
	    test_bit(FM_INTTASK_RUNNING, &fmdev->flag)) {
		/* Fill command header info */
		cmd_hdr =
		    (struct fm_cmd_msg_hdr *)skb_put(skb, FM_CMD_MSG_HDR_SIZE);
		cmd_hdr->header = FM_PKT_LOGICAL_CHAN_NUMBER;	/* 0x08 */
		/* 3 (fm_opcode,rd_wr,dlen) + payload len) */
		cmd_hdr->len = ((payload == NULL) ? 0 : payload_len) + 3;
		/* FM opcode */
		cmd_hdr->fm_opcode = fm_reg_info[fmreg_index].opcode;
		/* read/write type */
		cmd_hdr->rd_wr = fm_reg_info[fmreg_index].type;
		cmd_hdr->dlen = payload_len;
		fm_cb(skb)->fm_opcode = fm_reg_info[fmreg_index].opcode;
	} else if (payload != NULL) {
		fm_cb(skb)->fm_opcode = *((char *)payload + 2);
	}
	/* Fill payload data */
	if (payload != NULL)
		memcpy(skb_put(skb, payload_len), payload, payload_len);

	/* Fill completion handler TX tasklet needs
	 * this info before sending packet to TX Q */
	fm_cb(skb)->completion = wait_completion;

	/* Add this new packet to TX Q and schedule TX tasklet */
	skb_queue_tail(&fmdev->tx_q, skb);
	tasklet_schedule(&fmdev->tx_task);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Sends FM Channel-8 command to the chip and waits for the reponse */
int fm_core_send_cmd(unsigned char fmreg_index, void *payload, int payload_len,
		     struct completion *wait_completion, void *reponse,
		     int *reponse_len)
{
	struct sk_buff *skb;
	struct fm_event_msg_hdr *fm_evt_hdr;
	unsigned long timeleft;
	unsigned long flags;
	int ret;

	FMDRV_API_START();

	init_completion(wait_completion);
	ret = __fm_core_send_cmd(fmreg_index, payload, payload_len,
				 wait_completion);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Wait for the reponse from the chip */
	timeleft = wait_for_completion_timeout(wait_completion,
					       FM_DRV_TX_TIMEOUT);
	if (!timeleft) {
		FM_DRV_ERR("Timeout(%d sec),didn't get reg"
			   "completion signal from RX tasklet",
			   jiffies_to_msecs(FM_DRV_TX_TIMEOUT) / 1000);
		FMDRV_API_EXIT(-ETIMEDOUT);
		return -ETIMEDOUT;
	}
	if (!fmdev->response_skb) {
		FM_DRV_ERR("Reponse SKB is missing ");
		FMDRV_API_EXIT(-EFAULT);
		return -EFAULT;
	}
	spin_lock_irqsave(&fmdev->resp_skb_lock, flags);
	skb = fmdev->response_skb;
	fmdev->response_skb = NULL;
	spin_unlock_irqrestore(&fmdev->resp_skb_lock, flags);

	fm_evt_hdr = (void *)skb->data;
	if (fm_evt_hdr->status != 0) {
		FM_DRV_ERR("Received event pkt status(%d) is not zero",
			   fm_evt_hdr->status);
		kfree_skb(skb);
		FMDRV_API_EXIT(-EIO);
		return -EIO;
	}
	/* Send reponse data to caller */
	if (reponse != NULL && reponse_len != NULL && fm_evt_hdr->dlen) {
		/* Skip header info and copy only response data */
		skb_pull(skb, sizeof(struct fm_event_msg_hdr));
		memcpy(reponse, skb->data, fm_evt_hdr->dlen);
		*reponse_len = fm_evt_hdr->dlen;
	} else if (reponse_len != NULL && fm_evt_hdr->dlen == 0) {
		*reponse_len = 0;
	}
	kfree_skb(skb);
	FMDRV_API_EXIT(0);
	return 0;
}

/* Resets RDS cache parameters */
static inline void fm_core_rx_reset_rds_cache(void)
{
	FMDRV_API_START();

	fmdev->rx.rds.flag = FM_RX_RDS_DISABLE;
	fmdev->rx.rds.last_block_index = 0;
	fmdev->rx.rds.wr_index = 0;
	fmdev->rx.rds.rd_index = 0;

	if (fmdev->rx.af_mode == FM_RX_RDS_AF_SWITCH_MODE_ON)
		fmdev->irq_info.mask |= FM_LEV_EVENT;

	FMDRV_API_EXIT(0);
}

/* Resets current station info */
static inline void fm_core_rx_reset_curr_station_info(void)
{
	FMDRV_API_START();

	fmdev->rx.cur_station_info.picode = FM_NO_PI_CODE;
	fmdev->rx.cur_station_info.no_of_items_in_afcache = 0;
	fmdev->rx.cur_station_info.af_list_max = 0;

	FMDRV_API_EXIT(0);
}

/* --- Helper functions used in FM interrupt handlers ---*/

static inline char fm_core_check_cmdresp_status(struct sk_buff **skb)
{
	struct fm_event_msg_hdr *fm_evt_hdr;
	unsigned long flags;

	FMDRV_API_START();

	spin_lock_irqsave(&fmdev->resp_skb_lock, flags);
	*skb = fmdev->response_skb;
	fmdev->response_skb = NULL;
	spin_unlock_irqrestore(&fmdev->resp_skb_lock, flags);

	fm_evt_hdr = (void *)(*skb)->data;
	if (fm_evt_hdr->status != 0) {
		FM_DRV_ERR("irq : opcode %x response status field is not zero",
			   fm_evt_hdr->fm_opcode);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return FM_ST_FAILED;
	}

	FMDRV_API_EXIT(FM_ST_SUCCESS);
	return FM_ST_SUCCESS;
}

/* Interrupt process timeout handler */
static void fm_core_int_timeout_handler(unsigned long data)
{
	FMDRV_API_START();

	FM_DRV_DBG("irq : timeout,trying to re-enable fm interrupts");
	fmdev->irq_info.irq_service_timeout_retry++;

	/* One of the irq handler did not get proper response from the chip.
	 * So take recovery action here. FM interrupts are disabled in the
	 * beginning of interrupt process. Therefore reset stage index to
	 * re-enable default interrupts. So that next interrupt will be
	 * processed as usual.
	 */
	if (fmdev->irq_info.irq_service_timeout_retry <=
	    FM_IRQ_TIMEOUT_RETRY_MAX) {
		fmdev->irq_info.stage_index = FM_SEND_INTMSK_CMD_INDEX;
		fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
							 stage_index] ();
	} else {
		/* Retry count reached maximum limit.
		 * Stop recovery action (interrupt
		 * reenable process) and reset stage
		 * index & retry count values
		 */
		fmdev->irq_info.stage_index = 0;
		fmdev->irq_info.irq_service_timeout_retry = 0;
		FM_DRV_ERR("Recovery action failed during \
		    irq processing,max retry reached");
	}
	FMDRV_API_EXIT(0);
}

/* --------- FM interrupt handlers ------------*/

static void fm_core_irq_send_flag_getcmd(void)
{
	unsigned short flag;
	int ret;

	FMDRV_API_START();

	/* Send FLAG_GET command , to know the source of interrupt */
	ret = __fm_core_send_cmd(FLAG_GET, NULL, sizeof(flag), NULL);
	if (ret) {
		/* Failed to send the cmd , Don't update the stage index &
		 * Trigger the timer to take recovery action */
		FM_DRV_ERR("irq : failed to send flag_get command,"
			   "initiating irq recovery process");
	} else
		fmdev->irq_info.stage_index = FM_HANDLE_FLAG_GETCMD_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_flag_getcmd_resp(void)
{
	struct sk_buff *skb;
	struct fm_event_msg_hdr *fm_evt_hdr;
	char ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);

	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	fm_evt_hdr = (void *)skb->data;

	/* Skip header info and copy only response data */
	skb_pull(skb, sizeof(struct fm_event_msg_hdr));
	memcpy(&fmdev->irq_info.flag, skb->data, fm_evt_hdr->dlen);

	FM_STORE_BE16_TO_LE16(fmdev->irq_info.flag, fmdev->irq_info.flag);
	FM_DRV_DBG("irq : flag register(0x%x)", fmdev->irq_info.flag);

	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_HW_MAL_FUNC_INDEX;

	FMDRV_API_EXIT(0);
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
}

static void fm_core_irq_handle_hw_malfunction(void)
{
	FMDRV_API_START();

	if (fmdev->irq_info.flag & FM_MAL_EVENT & fmdev->irq_info.mask)
		FM_DRV_ERR("irq : Hw MAL interrupt received - do nothing");

	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_RDS_START_INDEX;
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();

	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_rds_start(void)
{
	FMDRV_API_START();

	if (fmdev->irq_info.flag & FM_RDS_EVENT & fmdev->irq_info.mask) {
		FM_DRV_DBG("irq : rds threshold reached");
		fmdev->irq_info.stage_index = FM_RDS_SEND_RDS_GETCMD_INDEX;
	} else {
		/* Continue next function in interrupt handler table */
		fmdev->irq_info.stage_index = FM_HW_TUNE_OP_ENDED_INDEX;
	}
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();

	FMDRV_API_EXIT(0);
}

static void fm_core_irq_send_rdsdata_getcmd(void)
{
	int ret;

	FMDRV_API_START();

	/* Send the command to read RDS data from the chip */
	ret = __fm_core_send_cmd(RDS_DATA_GET, NULL,
				 (FM_RX_RDS_FIFO_THRESHOLD * 3), NULL);
	if (ret)
		FM_DRV_ERR("irq : failed to send rds get commandm,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index =
		    FM_RDS_HANDLE_RDS_GETCMD_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

/* Keeps track of current RX channel AF (Alternate Frequency) */
static void __fm_core_rx_update_af_cache(unsigned char af)
{
	unsigned char index;
	unsigned int freq;
	FMDRV_API_START();

	/* First AF indicates the number of AF follows. Reset the list */
	if ((af >= FM_RDS_1_AF_FOLLOWS) && (af <= FM_RDS_25_AF_FOLLOWS)) {
		fmdev->rx.cur_station_info.af_list_max =
		    (af - FM_RDS_1_AF_FOLLOWS + 1);
		fmdev->rx.cur_station_info.no_of_items_in_afcache = 0;
		FM_DRV_DBG("No of expected AF : %d",
			   fmdev->rx.cur_station_info.af_list_max);
	} else if (((af >= FM_RDS_MIN_AF)
		    && (fmdev->rx.region.region_index == FM_BAND_EUROPE_US)
		    && (af <= FM_RDS_MAX_AF)) || ((af >= FM_RDS_MIN_AF)
						  && (fmdev->rx.region.
						      region_index ==
						      FM_BAND_JAPAN)
						  && (af <=
						      FM_RDS_MAX_AF_JAPAN))) {
		freq = fmdev->rx.region.bottom_frequency + (af * 100);
		/* If the AF is the same
		 * as the tuned station frequency - ignore it
		 */
		if (freq == fmdev->rx.curr_freq) {
			FM_DRV_DBG("Current frequency(%d) is \
			    matching with received AF(%d)",
			    fmdev->rx.curr_freq, freq);
			return;
		}
		/* Do check in AF cache */
		for (index = 0;
		     index < fmdev->rx.cur_station_info.no_of_items_in_afcache;
		     index++) {
			if (fmdev->rx.cur_station_info.af_cache[index] == freq)
				break;
		}
		/* Reached the limit of the list - ignore the next AF */
		if (index == fmdev->rx.cur_station_info.af_list_max) {
			FM_DRV_DBG("AF cache is full");
			return;
		}

		/* If we reached the end of the list then
		 * this AF is not in the list - add it
		 */
		if (index == fmdev->rx.cur_station_info.
				   no_of_items_in_afcache) {
			FM_DRV_DBG("Storing AF %d into AF cache index %d", freq,
				   index);
			fmdev->rx.cur_station_info.af_cache[index] = freq;
			fmdev->rx.cur_station_info.no_of_items_in_afcache++;
		}
	}
	FMDRV_API_EXIT(0);
}

/* Converts RDS buffer data from big endian format
 * to little endian format
 */
static void __fm_core_rdsparse_swapbytes(struct fm_rdsdata_format *rds_format)
{
	unsigned char byte1;
	unsigned char index = 0;
	char *rds_buff;

	FMDRV_API_START();

	/* Since in Orca the 2 RDS Data bytes are in little endian and
	 * in Dolphin they are in big endian, the parsing of the RDS data
	 * is chip dependent */
	if (fmdev->asci_id != 0x6350) {
		rds_buff = &rds_format->rdsData.groupDataBuff.rdsBuff[0];
		while (index + 1 < FM_RX_RDS_INFO_FIELD_MAX) {
			byte1 = rds_buff[index];
			rds_buff[index] = rds_buff[index + 1];
			rds_buff[index + 1] = byte1;
			index += 2;
		}
	}

	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_rdsdata_getcmd_resp(void)
{
	struct sk_buff *skb;
	char *rds_data;
	char metaData;
	unsigned char type, block_index;
	unsigned long group_index;
	struct fm_rdsdata_format rds_format;
	int rds_len, ret;
	unsigned short cur_picode;
	unsigned char tmpbuf[3];
	unsigned long flags;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);

	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	/* Skip header info */
	skb_pull(skb, sizeof(struct fm_event_msg_hdr));
	rds_data = skb->data;
	rds_len = skb->len;

	/* Parse the RDS data */
	while (rds_len >= FM_RDS_BLOCK_SIZE) {
		metaData = rds_data[2];
		/* Get the type:
		 * 0=A, 1=B, 2=C, 3=C', 4=D, 5=E */
		type = (metaData & 0x07);

		/* Transform the block type
		 * into an index sequence (0, 1, 2, 3, 4) */
		block_index = (type <= FM_RDS_BLOCK_C ? type : (type - 1));
		FM_DRV_DBG("Block index:%d(%s) ", block_index,
			   (metaData & FM_RDS_STATUS_ERROR_MASK) ? "Bad" :
			   "Ok");
		if (((metaData & FM_RDS_STATUS_ERROR_MASK) == 0)
		    && (block_index == FM_RDS_BLOCK_INDEX_A
			|| (block_index == fmdev->rx.rds.last_block_index + 1
			    && block_index <= FM_RDS_BLOCK_INDEX_D))) {
			/* Skip checkword (control) byte
			 * and copy only data byte */
			memcpy(&rds_format.rdsData.groupDataBuff.
			       rdsBuff[block_index * (FM_RDS_BLOCK_SIZE - 1)],
			       rds_data, (FM_RDS_BLOCK_SIZE - 1));
			fmdev->rx.rds.last_block_index = block_index;

			/* If completed a whole group then handle it */
			if (block_index == FM_RDS_BLOCK_INDEX_D) {
				FM_DRV_DBG("Good block received");
				__fm_core_rdsparse_swapbytes(&rds_format);

				/* Extract PI code and store in local cache.
				 * We need this during AF switch processing */
				cur_picode =
				    FM_BE16_TO_LE16(rds_format.rdsData.
						    groupGeneral.piData);
				if (fmdev->rx.cur_station_info.picode !=
				    cur_picode)
					fmdev->rx.cur_station_info.picode =
					    cur_picode;
				FM_DRV_DBG("picode:%d", cur_picode);

				group_index =
				    (rds_format.rdsData.groupGeneral.
				     blockB_byte1 >> 3);
				FM_DRV_DBG("Group:%ld%s", group_index / 2,
					   (group_index % 2) ? "B" : "A");

				group_index =
				    1 << (rds_format.rdsData.groupGeneral.
					  blockB_byte1 >> 3);
				if (group_index == FM_RDS_GROUP_TYPE_MASK_0A) {
					__fm_core_rx_update_af_cache
					    (rds_format.rdsData.group0A.
					     firstAf);
					__fm_core_rx_update_af_cache(rds_format.
								     rdsData.
								     group0A.
								     secondAf);
				}
			}
		} else {
			FM_DRV_DBG("Block sequence mismatch");
			fmdev->rx.rds.last_block_index = -1;
		}
		rds_len -= FM_RDS_BLOCK_SIZE;
		rds_data += FM_RDS_BLOCK_SIZE;
	}

     /* Copy raw rds data to internal rds buffer */
	rds_data = skb->data;
	rds_len = skb->len;

	spin_lock_irqsave(&fmdev->rds_buff_lock, flags);
	while (rds_len > 0) {
		/* Fill RDS buffer as per V4L2 specification.
		 * Store control byte
		 */
		type = (rds_data[2] & 0x07);
		block_index = (type <= FM_RDS_BLOCK_C ? type : (type - 1));
		tmpbuf[2] = block_index;	/* Offset name */
		tmpbuf[2] |= block_index << 3;	/* Received offset */

		/* Store data byte */
		tmpbuf[0] = rds_data[0];
		tmpbuf[1] = rds_data[1];

		memcpy(&fmdev->rx.rds.buffer[fmdev->rx.rds.wr_index], &tmpbuf,
		       FM_RDS_BLOCK_SIZE);
		fmdev->rx.rds.wr_index =
		    (fmdev->rx.rds.wr_index +
		     FM_RDS_BLOCK_SIZE) % fmdev->rx.rds.buf_size;

		/* Check for overflow & start over */
		if (fmdev->rx.rds.wr_index == fmdev->rx.rds.rd_index) {
			FM_DRV_DBG("RDS buffer overflow");
			fmdev->rx.rds.wr_index = 0;
			fmdev->rx.rds.rd_index = 0;
			break;
		}
		rds_len -= FM_RDS_BLOCK_SIZE;
		rds_data += FM_RDS_BLOCK_SIZE;
	}
	spin_unlock_irqrestore(&fmdev->rds_buff_lock, flags);

	/* Wakeup read queue */
	if (fmdev->rx.rds.wr_index != fmdev->rx.rds.rd_index)
		wake_up_interruptible(&fmdev->rx.rds.read_queue);

	fmdev->irq_info.stage_index = FM_RDS_FINISH_INDEX;
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_rds_finish(void)
{
	FMDRV_API_START();

	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_HW_TUNE_OP_ENDED_INDEX;
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();

	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_tune_op_ended(void)
{
	FMDRV_API_START();

	if (fmdev->irq_info.flag & (FM_FR_EVENT | FM_BL_EVENT) & fmdev->
	    irq_info.mask) {
		FM_DRV_DBG("irq : tune ended/bandlimit reached");
		if (test_and_clear_bit(FM_AF_SWITCH_INPROGRESS, &fmdev->flag)) {
			fmdev->irq_info.stage_index = FM_AF_JUMP_RD_FREQ_INDEX;
		} else {
			complete(&fmdev->maintask_completion);
			fmdev->irq_info.stage_index = FM_HW_POWER_ENB_INDEX;
		}
	} else
		fmdev->irq_info.stage_index = FM_HW_POWER_ENB_INDEX;

	FMDRV_API_EXIT(0);
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
}

static void fm_core_irq_handle_power_enb(void)
{
	FMDRV_API_START();

	if (fmdev->irq_info.flag & FM_POW_ENB_EVENT) {
		FM_DRV_DBG("irq : Power Enabled/Disabled");
		complete(&fmdev->maintask_completion);
	}

	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_LOW_RSSI_START_INDEX;
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();

	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_low_rssi_start(void)
{
	FMDRV_API_START();

	if ((fmdev->rx.af_mode == FM_RX_RDS_AF_SWITCH_MODE_ON) &&
	    (fmdev->irq_info.flag & FM_LEV_EVENT & fmdev->irq_info.mask) &&
	    (fmdev->rx.curr_freq != FM_UNDEFINED_FREQ) &&
	    (fmdev->rx.cur_station_info.no_of_items_in_afcache != 0)) {
		FM_DRV_DBG("irq : rssi level has fallen below threshold level");

		/* Disable further low RSSI interrupts */
		fmdev->irq_info.mask &= ~FM_LEV_EVENT;

		fmdev->rx.cur_Afjump_index = 0;
		fmdev->rx.freq_before_jump = fmdev->rx.curr_freq;
		fmdev->irq_info.stage_index = FM_AF_JUMP_SETPI_INDEX;
	} else {
		/* Continue next function in interrupt handler table */
		fmdev->irq_info.stage_index = FM_SEND_INTMSK_CMD_INDEX;
	}
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_afjump_set_pi(void)
{
	int ret;
	unsigned short payload;

	FMDRV_API_START();

	/* Set PI code - must be updated if the AF list is not empty */
	payload = FM_LE16_TO_BE16(fmdev->rx.cur_station_info.picode);
	ret = __fm_core_send_cmd(RDS_PI_SET, &payload, sizeof(payload), NULL);
	if (ret)
		FM_DRV_ERR("irq : failed to set PI,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index =
		    FM_AF_JUMP_HANDLE_SETPI_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_set_pi_resp(void)
{
	struct sk_buff *skb;
	int ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);

	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_AF_JUMP_SETPI_MASK_INDEX;

	FMDRV_API_EXIT(0);
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
}

static void fm_core_irq_afjump_set_pimask(void)
{
	int ret;
	unsigned short payload;

	FMDRV_API_START();

	/* Set PI mask.
	 * 0xFFFF = Enable PI code matching
	 * 0x0000 = Disable PI code matching
	 */
	payload = 0x0000;
	ret =
	    __fm_core_send_cmd(RDS_PI_MASK_SET, &payload, sizeof(payload),
			       NULL);
	if (ret < 0)
		FM_DRV_ERR("irq : failed to set PI mask,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index =
		    FM_AF_JUMP_HANDLE_SETPI_MASK_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_set_pimask_resp(void)
{
	struct sk_buff *skb;
	int ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);
	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_AF_JUMP_SET_AF_FREQ_INDEX;
	FMDRV_API_EXIT(0);
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
}

static void fm_core_irq_afjump_setfreq(void)
{
	unsigned short frq_index;
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("Swtiching to %d KHz\n",
	       fmdev->rx.cur_station_info.af_cache[fmdev->rx.cur_Afjump_index]);
	frq_index =
	    (fmdev->rx.cur_station_info.af_cache[fmdev->rx.cur_Afjump_index] -
	     fmdev->rx.region.bottom_frequency) /
	    fmdev->rx.region.channel_spacing;

	FM_STORE_LE16_TO_BE16(payload, frq_index);
	ret = __fm_core_send_cmd(AF_FREQ_SET, &payload, sizeof(payload), NULL);
	if (ret < 0)
		FM_DRV_ERR("irq : failed to set AF freq,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index =
		    FM_AF_JUMP_HENDLE_SET_AFFREQ_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_setfreq_resp(void)
{
	struct sk_buff *skb;
	int ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);
	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	} else {
		/* Continue next function in interrupt handler table */
		fmdev->irq_info.stage_index = FM_AF_JUMP_ENABLE_INT_INDEX;
	}
	FMDRV_API_EXIT(0);
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
}

static void fm_core_irq_afjump_enableint(void)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	/* Enable FR (tuning operation ended) interrupt */
	payload = FM_LE16_TO_BE16(FM_FR_EVENT);
	ret = __fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload), NULL);
	if (ret)
		FM_DRV_ERR("irq : failed to enable FR interrupt,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index = FM_AF_JUMP_ENABLE_INT_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_afjump_enableint_resp(void)
{
	struct sk_buff *skb;
	int ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);
	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_AF_JUMP_START_AFJUMP_INDEX;
	FMDRV_API_EXIT(0);
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
}

static void fm_core_irq_start_afjump(void)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	/* Issue AF switch start command to the chip */
	FM_STORE_LE16_TO_BE16(payload, FM_TUNER_AF_JUMP_MODE);
	ret =
	    __fm_core_send_cmd(TUNER_MODE_SET, &payload, sizeof(payload), NULL);
	if (ret)
		FM_DRV_ERR("irq : failed to start af switch,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index =
		    FM_AF_JUMP_HANDLE_START_AFJUMP_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_start_afjump_resp(void)
{
	struct sk_buff *skb;
	int ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);
	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	fmdev->irq_info.stage_index = FM_SEND_FLAG_GETCMD_INDEX;
	set_bit(FM_AF_SWITCH_INPROGRESS, &fmdev->flag);
	clear_bit(FM_INTTASK_RUNNING, &fmdev->flag);

	FMDRV_API_EXIT(0);
}

static void fm_core_irq_afjump_rd_freq(void)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	ret = __fm_core_send_cmd(FREQ_GET, NULL, sizeof(payload), NULL);
	if (ret)
		FM_DRV_ERR("irq : failed to read cur freq,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index = FM_AF_JUMP_RD_FREQ_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_afjump_rd_freq_resp(void)
{
	struct sk_buff *skb;
	unsigned short read_freq;
	unsigned int curr_freq, jumped_freq;
	int ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);
	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	/* Skip header info and copy only response data */
	skb_pull(skb, sizeof(struct fm_event_msg_hdr));
	memcpy(&read_freq, skb->data, sizeof(read_freq));
	read_freq = FM_BE16_TO_LE16(read_freq);
	curr_freq = fmdev->rx.region.bottom_frequency +
	    ((unsigned int)read_freq * fmdev->rx.region.channel_spacing);

	jumped_freq =
	    fmdev->rx.cur_station_info.af_cache[fmdev->rx.cur_Afjump_index];

	/* If the frequency was changed the jump succeeded */
	if ((curr_freq != fmdev->rx.freq_before_jump) &&
	    (curr_freq == jumped_freq)) {
		FM_DRV_DBG("Successfully switched to alternate frequency %d",
			   curr_freq);
		fmdev->rx.curr_freq = curr_freq;

		/* Reset RDS cache */
		fm_core_rx_reset_rds_cache();

		/* AF feature is on, enable low level RSSI interrupt */
		if (fmdev->rx.af_mode == FM_RX_RDS_AF_SWITCH_MODE_ON)
			fmdev->irq_info.mask |= FM_LEV_EVENT;

		/* Call the next stage of general
		 * interrupts handler to handle other interrupts
		 */
		fmdev->irq_info.stage_index = FM_LOW_RSSI_FINISH_INDEX;
	} else {		/* jump to the next freq in the AF list */

		/* Go to next index in the AF list */
		fmdev->rx.cur_Afjump_index++;

		/* If we reached the end of the list - stop searching */
		if (fmdev->rx.cur_Afjump_index >=
		    fmdev->rx.cur_station_info.no_of_items_in_afcache) {
			FM_DRV_DBG("AF switch processing failed");
			/* Call the next stage of general
			 * interrupts handler to handle other interrupts
			 */
			fmdev->irq_info.stage_index = FM_LOW_RSSI_FINISH_INDEX;
		} else {	/* AF List is not over - try next one */

			FM_DRV_DBG("Trying next frequency in af cache");
			fmdev->irq_info.stage_index = FM_AF_JUMP_SETPI_INDEX;
		}
	}
	FMDRV_API_EXIT(0);
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();
}

static void fm_core_irq_handle_low_rssi_finish(void)
{
	FMDRV_API_START();

	/* Continue next function in interrupt handler table */
	fmdev->irq_info.stage_index = FM_SEND_INTMSK_CMD_INDEX;
	fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
						 stage_index] ();

	FMDRV_API_EXIT(0);
}

static void fm_core_irq_send_intmsk_cmd(void)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	/* Re-enable FM interrupts */
	FM_STORE_LE16_TO_BE16(payload, fmdev->irq_info.mask);
	ret = __fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload), NULL);
	if (ret)
		FM_DRV_ERR("irq : failed to send int_mask_set cmd,"
			   "initiating irq recovery process");
	else
		fmdev->irq_info.stage_index = FM_HANDLE_INTMSK_CMD_RESP_INDEX;

	/* Start timer to track timeout */
	mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
		  FM_DRV_TX_TIMEOUT);
	FMDRV_API_EXIT(0);
}

static void fm_core_irq_handle_intmsk_cmd_resp(void)
{
	struct sk_buff *skb;
	int ret;

	FMDRV_API_START();

	/* Stop timeout timer */
	del_timer(&fmdev->irq_info.int_timeout_timer);

	ret = fm_core_check_cmdresp_status(&skb);
	if (ret) {
		FM_DRV_ERR("Initiating irq recovery process");
		mod_timer(&fmdev->irq_info.int_timeout_timer, jiffies +
			  FM_DRV_TX_TIMEOUT);
		FMDRV_API_EXIT(FM_ST_FAILED);
		return;
	}
	/* This is last function in interrupt table to be executed.
	 * So, reset stage index to 0.
	 */
	fmdev->irq_info.stage_index = FM_SEND_FLAG_GETCMD_INDEX;

	/* Start processing any pending interrupt */
	if (test_and_clear_bit(FM_INTTASK_SCHEDULE_PENDING, &fmdev->flag)) {
		FMDRV_API_EXIT(0);
		fmdev->irq_info.fm_IntActionHandlerTable[fmdev->irq_info.
							 stage_index]
		    ();
	} else
		clear_bit(FM_INTTASK_RUNNING, &fmdev->flag);

	FMDRV_API_EXIT(0);
}

/* --------------- FM TX tasklet -----------------*/

/* FM V4L2 interface and FM mixer control interface layer
 * schedule this tasklet whenever it wants to transmit FM packet.
 */
static void fm_core_tx_tasklet(unsigned long arg)
{
	struct sk_buff *skb;
	int ret;

	FMDRV_API_START();

	/* Check, is there any timeout happenned to last transmitted packet */
	if (!atomic_read(&fmdev->tx_cnt) &&
	    ((jiffies - fmdev->last_tx_jiffies) > FM_DRV_TX_TIMEOUT)) {
		FM_DRV_ERR("TX timeout occurred");
		atomic_set(&fmdev->tx_cnt, 1);
	}
	/* Send queued FM TX packets */
	if (atomic_read(&fmdev->tx_cnt)) {
		skb = skb_dequeue(&fmdev->tx_q);
		if (skb) {
			atomic_dec(&fmdev->tx_cnt);
			fmdev->last_sent_pkt_opcode = fm_cb(skb)->fm_opcode;

			if (fmdev->response_completion != NULL)
				FM_DRV_ERR
				    ("Response completion handler is not NULL");

			fmdev->response_completion = fm_cb(skb)->completion;
#ifdef FM_DUMP_TXRX_PKT
			dump_tx_skb_data(skb);
#endif
			/* Forward SKB to FM ST,
			 * FM ST will internally forward SKB to ST driver.
			 */
			ret = fm_st_send(skb);
			if (ret < 0) {
				fmdev->response_completion = NULL;
				FM_DRV_ERR
				    ("TX tasklet failed to send skb(%p)", skb);
				atomic_set(&fmdev->tx_cnt, 1);
			} else
				fmdev->last_tx_jiffies = jiffies;
		}
	}
	FMDRV_API_EXIT(0);
}

/* --------------- FM RX tasklet -----------------*/

/* FM ST will schedule this tasklet whenever it receives
 * FM packet from ST driver.
 */
static void fm_core_rx_tasklet(unsigned long arg)
{
	struct fm_event_msg_hdr *fm_evt_hdr;
	struct sk_buff *skb;
	unsigned char num_fm_hci_cmds;
	unsigned long flags;

	FMDRV_API_START();

	/* Process all packets in the RX queue */
	while ((skb = skb_dequeue(&fmdev->rx_q))) {
		if (skb->len < sizeof(struct fm_event_msg_hdr)) {
			FM_DRV_ERR("skb(%p) has only %d bytes"
				   "atleast need %d bytes to decode",
				   skb, skb->len,
				   sizeof(struct fm_event_msg_hdr));
			kfree_skb(skb);
			continue;
		}

		fm_evt_hdr = (void *)skb->data;
		num_fm_hci_cmds = fm_evt_hdr->num_fm_hci_cmds;

#ifdef FM_DUMP_TXRX_PKT
		dump_rx_skb_data(skb);
#endif
		/* FM interrupt packet? */
		if (fm_evt_hdr->fm_opcode == fm_reg_info[FM_INTERRUPT].opcode) {
			/* FM interrupt handler started already? */
			if (!test_bit(FM_INTTASK_RUNNING, &fmdev->flag)) {
				set_bit(FM_INTTASK_RUNNING, &fmdev->flag);
				if (fmdev->irq_info.stage_index != 0) {
					FM_DRV_ERR("Invalid stage index,"
						   "resetting to zero");
					fmdev->irq_info.stage_index = 0;
				}

				/* Execute first function
				 * in interrupt handler table
				 */
				fmdev->irq_info.fm_IntActionHandlerTable
				    [fmdev->irq_info.stage_index]
				    ();
			} else {
				set_bit(FM_INTTASK_SCHEDULE_PENDING,
					&fmdev->flag);
			}
			kfree_skb(skb);
		}
		/* Anyone waiting for this with completion handler? */
		else if (fm_evt_hdr->fm_opcode == fmdev->last_sent_pkt_opcode &&
			 fmdev->response_completion != NULL) {
			if (fmdev->response_skb != NULL)
				FM_DRV_ERR("Response SKB pointer is not NULL");

			spin_lock_irqsave(&fmdev->resp_skb_lock, flags);
			fmdev->response_skb = skb;
			spin_unlock_irqrestore(&fmdev->resp_skb_lock, flags);
			complete(fmdev->response_completion);

			fmdev->response_completion = NULL;
			atomic_set(&fmdev->tx_cnt, 1);
		}
		/* Is this for interrupt handler? */
		else if (fm_evt_hdr->fm_opcode == fmdev->last_sent_pkt_opcode &&
			 fmdev->response_completion == NULL) {
			if (fmdev->response_skb != NULL)
				FM_DRV_ERR("Response SKB pointer is not NULL");

			spin_lock_irqsave(&fmdev->resp_skb_lock, flags);
			fmdev->response_skb = skb;
			spin_unlock_irqrestore(&fmdev->resp_skb_lock, flags);

			/* Execute interrupt handler where state index points */
			fmdev->irq_info.fm_IntActionHandlerTable
			    [fmdev->irq_info.stage_index]
			    ();

			kfree_skb(skb);
			atomic_set(&fmdev->tx_cnt, 1);
		} else {
			FM_DRV_ERR("Nobody claimed SKB(%p),purging", skb);
		}

		/* Check flow control field.
		 * If Num_FM_HCI_Commands field is not zero,
		 * schedule FM TX tasklet.
		 */
		if (num_fm_hci_cmds && atomic_read(&fmdev->tx_cnt)) {
			if (!skb_queue_empty(&fmdev->tx_q))
				tasklet_schedule(&fmdev->tx_task);
		}
	}
	FMDRV_API_EXIT(0);
}

/* ----- Fucntions exported to FM V4L2 layer ----- */
int fm_core_tx_set_stereo_mono(unsigned short mode)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("stereo mode: %d", (int)mode);

	/* Set Stereo/Mono mode */
	FM_STORE_LE16_TO_BE16(payload, (1 - mode));
	ret = fm_core_send_cmd(MONO_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_rds_text(unsigned char *rds_text)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("rds_text: %s", rds_text);

	ret = fm_core_send_cmd(RDS_DATA_SET, rds_text, strlen(rds_text),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Scroll mode */
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)0x1);
	ret = fm_core_send_cmd(DISPLAY_MODE_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_rds_data_mode(unsigned char mode)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("data mode: %d", (int)mode);

	/* Setting unique PI TODO: how unique? */
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)0xcafe);
	ret = fm_core_send_cmd(PI_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Set decoder id */
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)0xa);
	ret = fm_core_send_cmd(DI_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* TODO: RDS_MODE_GET? */
	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_rds_len(unsigned char type, unsigned short len)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("len: %d", (int)len);

	len |= type << 8;
	FM_STORE_LE16_TO_BE16(payload, len);
	ret = fm_core_send_cmd(LENGHT_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);

	FM_CHECK_SEND_CMD_STATUS(ret);

	/* TODO: LENGHT_GET? */
	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_rds_mode(unsigned char rds_en_dis)
{
	unsigned short payload;
	int ret;
	unsigned char rds_text[] = "Zoom2\n";

	FMDRV_API_START();

	FM_DRV_DBG("rds_en_dis:%d(E:%d, D:%d)", rds_en_dis,
		   FM_RX_RDS_ENABLE, FM_RX_RDS_DISABLE);

	if (rds_en_dis == FM_RX_RDS_ENABLE) {
		/* Set RDS length */
		fm_core_tx_set_rds_len(0, strlen(rds_text));
		/* Set RDS text */
		fm_core_tx_set_rds_text(rds_text);
		/* Set RDS mode */
		fm_core_tx_set_rds_data_mode(0x0);
	}

	/* Send command to enable RDS */
	if (rds_en_dis == FM_RX_RDS_ENABLE)
		FM_STORE_LE16_TO_BE16(payload, 0x01);
	else
		FM_STORE_LE16_TO_BE16(payload, 0x00);

	ret = fm_core_send_cmd(RDS_DATA_ENB, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	if (rds_en_dis == FM_RX_RDS_ENABLE) {
		/* Set RDS length */
		fm_core_tx_set_rds_len(0, strlen(rds_text));
		/* Set RDS text */
		fm_core_tx_set_rds_text(rds_text);
	}

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_radio_text(unsigned char *rds_text, unsigned char rds_type)
{
	unsigned short payload;
	int ret;
	FMDRV_API_START();

	fm_core_tx_set_rds_mode(0);
	/* Set RDS length */
	fm_core_tx_set_rds_len(rds_type, strlen(rds_text));
	/* Set RDS text */
	fm_core_tx_set_rds_text(rds_text);
	/* Set RDS mode */
	fm_core_tx_set_rds_data_mode(0x0);

	FM_STORE_LE16_TO_BE16(payload, 1);
	ret = fm_core_send_cmd(RDS_DATA_ENB, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_af(unsigned int af)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("AF: %d", af);

	/* Set AF */
	af = (af - 87500) / 100;

	FM_STORE_LE16_TO_BE16(payload, (unsigned short)af);
	ret = fm_core_send_cmd(TA_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);
	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_region(unsigned char region_to_set)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("region_to_set %ld(Eu/US %d, Jp %d)",
		   (unsigned long)region_to_set, FM_BAND_EUROPE_US,
		   FM_BAND_JAPAN);

	if (region_to_set != FM_BAND_EUROPE_US &&
	    region_to_set != FM_BAND_JAPAN) {
		FM_DRV_ERR("Invalid band\n");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}

	/* Send command to set the band */
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)region_to_set);
	ret = fm_core_send_cmd(TX_BAND_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_tx_set_mute_mode(unsigned char mute_mode_toset)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("tx: mute mode %ld", (unsigned long)mute_mode_toset);
	FM_STORE_LE16_TO_BE16(payload, mute_mode_toset);
	ret = fm_core_send_cmd(MUTE, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Set TX Audio I/O */
static int fm_core_tx_set_audio_io(void)
{
	struct fmtx_data *tx = &fmdev->tx_data;
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("tx: audio_io %ld ", (long int)tx->audio_io);

	/* Set Audio I/O Enable */
	FM_STORE_LE16_TO_BE16(payload, tx->audio_io);
	ret = fm_core_send_cmd(AUDIO_IO_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* TODO: is audio set? */
	FMDRV_API_EXIT(0);
	return 0;
}

/* Start TX Transmission */
static int fm_core_tx_xmit(unsigned char new_xmit_state)
{
	struct fmtx_data *tx = &fmdev->tx_data;
	unsigned short payload;
	unsigned long timeleft;
	int ret;

	FMDRV_API_START();

	/* Enable POWER_ENB interrupts */
	FM_STORE_LE16_TO_BE16(payload, FM_POW_ENB_EVENT);
	ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);

	/* Set Power Enable */
	FM_STORE_LE16_TO_BE16(payload, new_xmit_state);
	ret = fm_core_send_cmd(POWER_ENB_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Wait for Power Enabled */
	init_completion(&fmdev->maintask_completion);
	timeleft = wait_for_completion_timeout(&fmdev->maintask_completion,
					       FM_DRV_TX_TIMEOUT);
	if (!timeleft) {
		FM_DRV_ERR("Timeout(%d sec),didn't get tune ended interrupt",
			   jiffies_to_msecs(FM_DRV_TX_TIMEOUT) / 1000);
		FMDRV_API_EXIT(-ETIMEDOUT);
		return -ETIMEDOUT;
	}

	set_bit(FM_CORE_TX_XMITING, &fmdev->flag);
	tx->xmit_state = new_xmit_state;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Set TX power level */
int fm_core_tx_set_pwr_lvl(unsigned char new_pwr_lvl)
{
	unsigned short payload;
	struct fmtx_data *tx = &fmdev->tx_data;
	int ret;

	FMDRV_API_START();

	FM_DRV_DBG("tx: pwr_level_to_set %ld ", (long int)new_pwr_lvl);
	/* If the core isn't ready update global variable */
	if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
		tx->pwr_lvl = new_pwr_lvl;
		return 0;
	}

	/* Set power level */
	FM_STORE_LE16_TO_BE16(payload, new_pwr_lvl);
	ret = fm_core_send_cmd(POWER_LEL_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* TODO: is the power level set? */
	tx->pwr_lvl = new_pwr_lvl;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Set TX Frequency */
static int fm_core_tx_set_frequency(unsigned int freq_to_set)
{
	struct fmtx_data *tx = &fmdev->tx_data;
	unsigned short payload, chanl_index;
	int ret;

	FMDRV_API_START();

	if (test_bit(FM_CORE_TX_XMITING, &fmdev->flag)) {
		fm_core_tx_xmit(0);
		clear_bit(FM_CORE_TX_XMITING, &fmdev->flag);
	}

	/* Enable FR, BL interrupts */
	FM_STORE_LE16_TO_BE16(payload, (FM_FR_EVENT | FM_BL_EVENT));
	ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);

	tx->tx_frq = (unsigned long)freq_to_set;
	FM_DRV_DBG("tx: freq_to_set %ld ", (long int)tx->tx_frq);

	chanl_index = freq_to_set / 10;
	/* Set current tuner channel */
	FM_STORE_LE16_TO_BE16(payload, chanl_index);
	ret = fm_core_send_cmd(CHANL_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* TODO:confirm the freq set */
	fm_core_tx_set_pwr_lvl(tx->pwr_lvl);

	tx->audio_io = 0x01;	/* I2S */
	fm_core_tx_set_audio_io();

	fm_core_tx_xmit(0x01);	/* Enable transmission */

	FMDRV_API_EXIT(0);
	return 0;
}

/* Returns availability of RDS data in internel buffer */
int fm_core_is_rds_data_available(struct file *file,
				  struct poll_table_struct *pts)
{
	FMDRV_API_START();

	poll_wait(file, &fmdev->rx.rds.read_queue, pts);
	if (fmdev->rx.rds.rd_index != fmdev->rx.rds.wr_index) {
		FMDRV_API_EXIT(0);
		return 0;
	}

	FMDRV_API_EXIT(-EAGAIN);
	return -EAGAIN;
}

/* Copies RDS data from internal buffer to user buffer */
int fm_core_transfer_rds_from_internal_buff(struct file *file,
					    char __user *buf, size_t count)
{
	unsigned int block_count;
	unsigned long flags;
	int ret;
	FMDRV_API_START();

	/* Block if no new data available */
	if (fmdev->rx.rds.wr_index == fmdev->rx.rds.rd_index) {
		if (file->f_flags & O_NONBLOCK)
			return -EWOULDBLOCK;

		ret = wait_event_interruptible(fmdev->rx.rds.read_queue,
					       (fmdev->rx.rds.wr_index !=
						fmdev->rx.rds.rd_index));
		if (ret) {
			FMDRV_API_EXIT(-EINTR);
			return -EINTR;
		}
	}

	/* Calculate block count from byte count */
	count /= 3;
	block_count = 0;
	ret = 0;

	spin_lock_irqsave(&fmdev->rds_buff_lock, flags);

	/* Copy RDS blocks from the internal buffer and to user buffer */
	while (block_count < count) {
		if (fmdev->rx.rds.wr_index == fmdev->rx.rds.rd_index)
			break;

		/* Always transfer complete RDS blocks */
		if (copy_to_user
		    (buf, &fmdev->rx.rds.buffer[fmdev->rx.rds.rd_index],
		     FM_RDS_BLOCK_SIZE))
			break;

		/* Increment and wrap the read pointer */
		fmdev->rx.rds.rd_index += FM_RDS_BLOCK_SIZE;

		/* Wrap read pointer */
		if (fmdev->rx.rds.rd_index >= fmdev->rx.rds.buf_size)
			fmdev->rx.rds.rd_index = 0;

		/* Increment counters */
		block_count++;
		buf += FM_RDS_BLOCK_SIZE;
		ret += FM_RDS_BLOCK_SIZE;
	}
	spin_unlock_irqrestore(&fmdev->rds_buff_lock, flags);
	return ret;
}

/* Checks FM core active status and TX/RX/OFF mode status */
static inline int fm_core_check_mode(unsigned char mode)
{
	if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
		FM_DRV_ERR("FM core is not ready");
		return -EPERM;
	}
	if (mode >= FM_MODE_ENTRY_MAX) {
		FM_DRV_ERR("Invalid fm mode");
		return -EPERM;
	}
	if (fmdev->curr_fmmode != mode) {
		FM_DRV_ERR("Current mode is not in %s mode",
			   mode ? ((mode == 1) ? "TX" : "RX") : "OFF");
		return -EPERM;
	}
	return 0;
}

/* Set RX Frequency */
static int fm_core_rx_set_frequency(unsigned int freq_to_set)
{
	unsigned long timeleft;
	unsigned short payload, curr_frq, frq_index;
	unsigned int curr_frq_in_khz;
	int ret, resp_len;

	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (freq_to_set < fmdev->rx.region.bottom_frequency ||
	    freq_to_set > fmdev->rx.region.top_frequency) {
		FM_DRV_ERR("Invalid frequency %d", freq_to_set);
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	/* Set audio enable */
	FM_STORE_LE16_TO_BE16(payload, FM_RX_FM_AUDIO_ENABLE_I2S_AND_ANALOG);
	ret = fm_core_send_cmd(AUDIO_ENABLE_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Set hilo to automatic selection */
	FM_STORE_LE16_TO_BE16(payload, FM_RX_IFFREQ_HILO_AUTOMATIC);
	ret = fm_core_send_cmd(HILO_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Calculate frequency index to write */
	frq_index = (freq_to_set - fmdev->rx.region.bottom_frequency) /
	    fmdev->rx.region.channel_spacing;

	/* Set frequency index */
	FM_STORE_LE16_TO_BE16(payload, frq_index);
	ret = fm_core_send_cmd(FREQ_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Read flags - just to clear any pending interrupts if we had */
	ret = fm_core_send_cmd(FLAG_GET, NULL, 2,
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Enable FR, BL interrupts */
	fmdev->irq_info.mask |= (FM_FR_EVENT | FM_BL_EVENT);
	FM_STORE_LE16_TO_BE16(payload, fmdev->irq_info.mask);
	ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Start tune */
	FM_STORE_LE16_TO_BE16(payload, FM_TUNER_PRESET_MODE);
	ret = fm_core_send_cmd(TUNER_MODE_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Wait for tune ended interrupt */
	init_completion(&fmdev->maintask_completion);
	timeleft = wait_for_completion_timeout(&fmdev->maintask_completion,
					       FM_DRV_TX_TIMEOUT);
	if (!timeleft) {
		FM_DRV_ERR("Timeout(%d sec),didn't get tune ended interrupt",
			   jiffies_to_msecs(FM_DRV_TX_TIMEOUT) / 1000);
		FMDRV_API_EXIT(-ETIMEDOUT);
		return -ETIMEDOUT;
	}

	/* Read freq back to confirm */
	ret = fm_core_send_cmd(FREQ_GET, NULL, 2,
			       &fmdev->maintask_completion, &curr_frq,
			       &resp_len);
	FM_CHECK_SEND_CMD_STATUS(ret);

	curr_frq = FM_BE16_TO_LE16(curr_frq);
	curr_frq_in_khz = (fmdev->rx.region.bottom_frequency
			   +
			   ((unsigned int)curr_frq *
			    fmdev->rx.region.channel_spacing));

	/* Re-enable default FM interrupts */
	fmdev->irq_info.mask &= ~(FM_FR_EVENT | FM_BL_EVENT);
	FM_STORE_LE16_TO_BE16(payload, fmdev->irq_info.mask);
	ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	if (curr_frq_in_khz != freq_to_set) {
		FM_DRV_ERR("Current chip frequency(%d) is not matching with"
			   " requested frequency(%d)", curr_frq_in_khz,
			   freq_to_set);
		FMDRV_API_EXIT(-EAGAIN);
		return -EAGAIN;
	}

	/* Update local cache  */
	fmdev->rx.curr_freq = curr_frq_in_khz;

	/* Reset RDS cache and current station pointers */
	fm_core_rx_reset_rds_cache();
	fm_core_rx_reset_curr_station_info();

	/* Do we need to reset anything else? */

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_set_frequency(unsigned int freq_to_set)
{
	int ret;

	FMDRV_API_START();
	switch (fmdev->curr_fmmode) {
	case FM_MODE_RX:
		ret = fm_core_rx_set_frequency(freq_to_set);
		break;

	case FM_MODE_TX:
		ret = fm_core_tx_set_frequency(freq_to_set);
		break;

	default:
		ret = -EINVAL;
	}
	FMDRV_API_EXIT(ret);
	return ret;
}

int fm_core_get_frequency(unsigned int *cur_tuned_frq)
{
	int ret;

	FMDRV_API_START();
	if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
		FM_DRV_ERR("FM core is not ready");
		FMDRV_API_EXIT(-EPERM);
		return -EPERM;
	}
	if (fmdev->rx.curr_freq == FM_UNDEFINED_FREQ) {
		FM_DRV_ERR("RX frequency is not set");
		FMDRV_API_EXIT(-EPERM);
		return -EPERM;
	}
	if (cur_tuned_frq == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	ret = 0;
	switch (fmdev->curr_fmmode) {
	case FM_MODE_RX:
		*cur_tuned_frq = fmdev->rx.curr_freq;
		ret = 0;
		break;

	case FM_MODE_TX:
		*cur_tuned_frq = 0;	/* TODO : Change this later */
		ret = 0;
		break;

	default:
		ret = -EINVAL;
	}
	FMDRV_API_EXIT(ret);
	return ret;
}

/* Seek */
int fm_core_rx_seek(unsigned int seek_upward, unsigned int wrap_around)
{
	int resp_len;
	unsigned short curr_frq, next_frq, last_frq;
	unsigned short payload, int_reason;
	char offset;
	unsigned long timeleft;
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Read the current frequency from chip */
	ret = fm_core_send_cmd(FREQ_GET, NULL, sizeof(curr_frq),
			       &fmdev->maintask_completion, &curr_frq,
			       &resp_len);
	FM_CHECK_SEND_CMD_STATUS(ret);

	curr_frq = FM_BE16_TO_LE16(curr_frq);

	last_frq =
	    (fmdev->rx.region.top_frequency - fmdev->rx.region.bottom_frequency)
	    / fmdev->rx.region.channel_spacing;

	/* Check the offset in order to be aligned to the 100KHz steps */
	offset = curr_frq % 2;

	next_frq = seek_upward ? curr_frq + 2 /* Seek Up */ :
	    curr_frq - 2 /* Seek Down */ ;

	/* Add or subtract offset (0/1) in order
	 * to stay aligned to the 100KHz steps
	 */
	if ((short)next_frq < 0)
		next_frq = last_frq - offset;
	else if (next_frq > last_frq)
		next_frq = 0 + offset;

	/* Set calculated next frequency to perform seek */
	FM_STORE_LE16_TO_BE16(payload, next_frq);
	ret = fm_core_send_cmd(FREQ_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Set search direction (0:Seek Down, 1:Seek Up) */
	FM_STORE_LE16_TO_BE16(payload, (seek_upward ? FM_SEARCH_DIRECTION_UP :
					FM_SEARCH_DIRECTION_DOWN));
	ret = fm_core_send_cmd(SEARCH_DIR_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Read flags - just to clear any pending interrupts if we had */
	ret = fm_core_send_cmd(FLAG_GET, NULL, 2,
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Enable FR, BL interrupts */
	fmdev->irq_info.mask |= (FM_FR_EVENT | FM_BL_EVENT);
	FM_STORE_LE16_TO_BE16(payload, fmdev->irq_info.mask);
	ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Start seek */
	FM_STORE_LE16_TO_BE16(payload, FM_TUNER_AUTONOMOUS_SEARCH_MODE);
	ret = fm_core_send_cmd(TUNER_MODE_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Wait for tune ended/band limit reached interrupt */
	init_completion(&fmdev->maintask_completion);
	timeleft = wait_for_completion_timeout(&fmdev->maintask_completion,
					       FM_DRV_RX_SEEK_TIMEOUT);
	if (!timeleft) {
		FM_DRV_ERR("Timeout(%d sec),didn't get tune ended interrupt",
			   jiffies_to_msecs(FM_DRV_RX_SEEK_TIMEOUT) / 1000);
		FMDRV_API_EXIT(-ETIMEDOUT);
		return -ETIMEDOUT;
	}
	int_reason = fmdev->irq_info.flag & 0x3;

	/* Re-enable default FM interrupts */
	fmdev->irq_info.mask &= ~(FM_FR_EVENT | FM_BL_EVENT);
	FM_STORE_LE16_TO_BE16(payload, fmdev->irq_info.mask);
	ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Read freq to know where operation tune operation stopped */
	ret = fm_core_send_cmd(FREQ_GET, NULL, 2,
			       &fmdev->maintask_completion, &curr_frq,
			       &resp_len);
	FM_CHECK_SEND_CMD_STATUS(ret);

	curr_frq = FM_BE16_TO_LE16(curr_frq);
	fmdev->rx.curr_freq = (fmdev->rx.region.bottom_frequency +
			       ((unsigned int)curr_frq *
				fmdev->rx.region.channel_spacing));

	/* Reset RDS cache and current station pointers */
	fm_core_rx_reset_rds_cache();
	fm_core_rx_reset_curr_station_info();

	/* Return error if band limit is reached */
	if (int_reason & FM_BL_EVENT) {
		FM_DRV_DBG("band limit reached\n");
		FMDRV_API_EXIT(-EAGAIN);
		return -EAGAIN;
	}

	FMDRV_API_EXIT(0);
	return 0;
}

/* Set volume */
int fm_core_rx_set_volume(unsigned short vol_to_set)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (vol_to_set > FM_RX_VOLUME_MAX) {
		FM_DRV_ERR("Volume is not within(%d-%d) range",
			   FM_RX_VOLUME_MIN, FM_RX_VOLUME_MAX);
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	vol_to_set *= FM_RX_VOLUME_GAIN_STEP;

	FM_STORE_LE16_TO_BE16(payload, vol_to_set);
	ret = fm_core_send_cmd(VOLUME_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	fmdev->rx.curr_volume = vol_to_set;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Get volume */
int fm_core_rx_get_volume(unsigned short *curr_vol)
{
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (curr_vol == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*curr_vol = fmdev->rx.curr_volume / FM_RX_VOLUME_GAIN_STEP;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Returns current band index (0-Europe/US; 1-Japan) */
int fm_core_region_get(unsigned char *region)
{
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (region == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*region = fmdev->rx.region.region_index;

	FMDRV_API_EXIT(0);
	return 0;
}

/* To get current band's bottom and top frequency */
int fm_core_rx_get_currband_lowhigh_freq(unsigned int *bottom_frequency,
					 unsigned int *top_frequency)
{
	FMDRV_API_START();

	if (bottom_frequency != NULL)
		*bottom_frequency = fmdev->rx.region.bottom_frequency;

	if (top_frequency != NULL)
		*top_frequency = fmdev->rx.region.top_frequency;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Sets band (0-Europe/US; 1-Japan) */
int fm_core_rx_region_set(unsigned char region_to_set)
{
	unsigned short payload;
	unsigned int new_frq;
	int ret;

	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (region_to_set != FM_BAND_EUROPE_US &&
	    region_to_set != FM_BAND_JAPAN) {
		FM_DRV_ERR("Invalid band\n");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	if (fmdev->rx.region.region_index == region_to_set) {
		FM_DRV_ERR("Requested band is already configured\n");
		FMDRV_API_EXIT(-EPERM);
		return -EPERM;
	}
	/* Send cmd to set the band  */
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)region_to_set);
	ret = fm_core_send_cmd(RX_BAND_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Update local region info cache */
	memcpy(&fmdev->rx.region, &region_configs[region_to_set],
	       sizeof(struct region_info));

	/* Check whether current RX frequency is within band boundary */
	if (fmdev->curr_fmmode == FM_MODE_RX) {
		new_frq = 0;
		if (fmdev->rx.curr_freq < fmdev->rx.region.bottom_frequency)
			new_frq = fmdev->rx.region.bottom_frequency;
		else if (fmdev->rx.curr_freq > fmdev->rx.region.top_frequency)
			new_frq = fmdev->rx.region.top_frequency;

		if (new_frq) {
			FM_DRV_DBG
			    ("Current freq is not within band limit boundary,"
			     "switching to %d KHz", new_frq);
			/* Current RX frequency is not
			 * within boundary. So, update it
			 */
			ret = fm_core_rx_set_frequency(new_frq);
			if (ret < 0) {
				FMDRV_API_EXIT(ret);
				return ret;
			}
		}
	}

	/* TODO : Add above boundary check in TX mode also */

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_region_set(unsigned char region_to_set)
{
	int ret;

	FMDRV_API_START();
	switch (fmdev->curr_fmmode) {
	case FM_MODE_RX:
		ret = fm_core_rx_region_set(region_to_set);
		break;

	case FM_MODE_TX:
		ret = fm_core_tx_set_region(region_to_set);
		break;

	default:
		ret = -EINVAL;
	}
	FMDRV_API_EXIT(ret);
	return ret;
}

/* Reads current mute mode (Mute Off/On/Attenuate)*/
int fm_core_rx_get_mute_mode(unsigned char *curr_mute_mode)
{
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (curr_mute_mode == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*curr_mute_mode = fmdev->rx.curr_mute_mode;
	FMDRV_API_EXIT(0);
	return 0;
}

static int __fm_core_config_rx_mute_reg(void)
{
	unsigned short payload, muteval;
	int ret;

	FMDRV_API_START();

	muteval = 0;
	switch (fmdev->rx.curr_mute_mode) {
	case FM_MUTE_ON:
		muteval = FM_RX_MUTE_AC_MUTE_MODE;
		break;
	case FM_MUTE_OFF:
		muteval = FM_RX_MUTE_UNMUTE_MODE;
		break;
	case FM_MUTE_ATTENUATE:
		muteval = FM_RX_MUTE_SOFT_MUTE_FORCE_MODE;
		break;
	}
	if (fmdev->rx.curr_rf_depend_mute == FM_RX_RF_DEPENDENT_MUTE_ON)
		muteval |= FM_RX_MUTE_RF_DEP_MODE;
	else
		muteval &= ~FM_RX_MUTE_RF_DEP_MODE;

	FM_STORE_LE16_TO_BE16(payload, muteval);
	ret = fm_core_send_cmd(MUTE_STATUS_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Configures mute mode (Mute Off/On/Attenuate) */
int fm_core_rx_set_mute_mode(unsigned char mute_mode_toset)
{
	unsigned char org_state;
	int ret;

	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (fmdev->rx.curr_mute_mode == mute_mode_toset) {
		FMDRV_API_EXIT(0);
		return 0;
	}
	org_state = fmdev->rx.curr_mute_mode;
	fmdev->rx.curr_mute_mode = mute_mode_toset;

	ret = __fm_core_config_rx_mute_reg();
	if (ret < 0) {
		fmdev->rx.curr_mute_mode = org_state;
		FMDRV_API_EXIT(ret);
		return ret;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_set_mute_mode(unsigned char mute_mode_toset)
{
	int ret;

	FMDRV_API_START();
	switch (fmdev->curr_fmmode) {
	case FM_MODE_RX:
		ret = fm_core_rx_set_mute_mode(mute_mode_toset);
		break;

	case FM_MODE_TX:
		ret = fm_core_tx_set_mute_mode(mute_mode_toset);
		break;

	default:
		ret = -EINVAL;
	}
	FMDRV_API_EXIT(ret);
	return ret;
}

/* Gets RF dependent soft mute mode enable/disable status */
int fm_core_rx_get_rfdepend_softmute(unsigned char *curr_mute_mode)
{
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (curr_mute_mode == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*curr_mute_mode = fmdev->rx.curr_rf_depend_mute;
	FMDRV_API_EXIT(0);
	return 0;
}

/* Sets RF dependent soft mute mode */
int fm_core_rx_set_rfdepend_softmute(unsigned char rfdepend_mute)
{
	unsigned char org_state;
	int ret;

	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (rfdepend_mute != FM_RX_RF_DEPENDENT_MUTE_ON &&
	    rfdepend_mute != FM_RX_RF_DEPENDENT_MUTE_OFF) {
		FM_DRV_ERR("Invalid RF dependent soft mute");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	if (fmdev->rx.curr_rf_depend_mute == rfdepend_mute) {
		FMDRV_API_EXIT(0);
		return 0;
	}

	org_state = fmdev->rx.curr_rf_depend_mute;
	fmdev->rx.curr_rf_depend_mute = rfdepend_mute;

	ret = __fm_core_config_rx_mute_reg();
	if (ret < 0) {
		fmdev->rx.curr_rf_depend_mute = org_state;
		FMDRV_API_EXIT(ret);
		return ret;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Returns the signal strength level of current channel */
int fm_core_rx_get_rssi_level(unsigned short *rssilvl)
{
	unsigned short curr_rssi_lel;
	int resp_len;
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (rssilvl == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	/* Read current RSSI level */
	ret = fm_core_send_cmd(RSSI_LVL_GET, NULL, 2,
			       &fmdev->maintask_completion, &curr_rssi_lel,
			       &resp_len);
	FM_CHECK_SEND_CMD_STATUS(ret);

	*rssilvl = FM_BE16_TO_LE16(curr_rssi_lel);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Sets the signal strength level that once reached
 * will stop the auto search process
 */
int fm_core_rx_set_rssi_threshold(short rssi_lvl_toset)
{
	unsigned short payload;
	int ret;
	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (rssi_lvl_toset < FM_RX_RSSI_THRESHOLD_MIN ||
	    rssi_lvl_toset > FM_RX_RSSI_THRESHOLD_MAX) {
		FM_DRV_ERR("Invalid RSSI threshold level");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)rssi_lvl_toset);
	ret = fm_core_send_cmd(SEARCH_LVL_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	fmdev->rx.curr_rssi_threshold = rssi_lvl_toset;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Returns current RX RSSI threshold value */
int fm_core_rx_get_rssi_threshold(short *curr_rssi_lvl)
{
	int ret;
	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (curr_rssi_lvl == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*curr_rssi_lvl = fmdev->rx.curr_rssi_threshold;
	FMDRV_API_EXIT(0);
	return 0;
}

/* Sets RX stereo/mono modes */
int fm_core_rx_set_stereo_mono(unsigned short mode)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (mode != FM_STEREO_MODE && mode != FM_MONO_MODE) {
		FM_DRV_ERR("Invalid mode");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	/* Set stereo/mono mode */
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)mode);
	ret = fm_core_send_cmd(MOST_MODE_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Set stereo blending mode */
	FM_STORE_LE16_TO_BE16(payload, FM_STEREO_SOFT_BLEND);
	ret = fm_core_send_cmd(MOST_BLEND_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_set_stereo_mono(unsigned short mode)
{
	int ret;

	FMDRV_API_START();
	switch (fmdev->curr_fmmode) {
	case FM_MODE_RX:
		ret = fm_core_rx_set_stereo_mono(mode);
		break;

	case FM_MODE_TX:
		ret = fm_core_tx_set_stereo_mono(mode);
		break;

	default:
		ret = -EINVAL;
	}
	FMDRV_API_EXIT(ret);
	return ret;
}

/* Gets current RX stereo/mono mode */
int fm_core_rx_get_stereo_mono(unsigned short *mode)
{
	unsigned short curr_mode;
	int ret, resp_len;

	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (mode == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	ret = fm_core_send_cmd(MOST_MODE_GET, NULL, 2,
			       &fmdev->maintask_completion, &curr_mode,
			       &resp_len);
	FM_CHECK_SEND_CMD_STATUS(ret);

	*mode = FM_BE16_TO_LE16(curr_mode);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Chooses RX de-emphasis filter mode (50us/75us) */
int fm_core_rx_set_deemphasis_mode(unsigned short mode)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (mode != FM_RX_EMPHASIS_FILTER_50_USEC &&
	    mode != FM_RX_EMPHASIS_FILTER_75_USEC) {
		FM_DRV_ERR("Invalid rx de-emphasis mode");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	FM_STORE_LE16_TO_BE16(payload, mode);
	ret = fm_core_send_cmd(DEMPH_MODE_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Gets current RX de-emphasis filter mode */
int fm_core_rx_get_deemphasis_mode(unsigned short *curr_deemphasis_mode)
{
	unsigned short curr_mode;
	int ret, resp_len;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (curr_deemphasis_mode == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	ret = fm_core_send_cmd(DEMPH_MODE_GET, NULL, 2,
			       &fmdev->maintask_completion, &curr_mode,
			       &resp_len);
	FM_CHECK_SEND_CMD_STATUS(ret);

	*curr_deemphasis_mode = FM_BE16_TO_LE16(curr_mode);

	FMDRV_API_EXIT(0);
	return 0;
}

/* Enable/Disable RX RDS */
int fm_core_rx_set_rds_mode(unsigned char rds_en_dis)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (rds_en_dis != FM_RX_RDS_ENABLE && rds_en_dis != FM_RX_RDS_DISABLE) {
		FM_DRV_ERR("Invalid rds option");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	if (rds_en_dis == FM_RX_RDS_ENABLE
	    && fmdev->rx.rds.flag == FM_RX_RDS_DISABLE) {
		/* Turn on RX RDS */
		/* Turn on RDS circuit */
		FM_STORE_LE16_TO_BE16(payload,
				      FM_RX_POWET_SET_FM_AND_RDS_BLK_ON);
		ret =
		    fm_core_send_cmd(POWER_SET, &payload, sizeof(payload),
				     &fmdev->maintask_completion, NULL, NULL);
		FM_CHECK_SEND_CMD_STATUS(ret);

		/* Clear and rest RDS FIFO */
		FM_STORE_LE16_TO_BE16(payload, FM_RX_RDS_FLUSH_FIFO);
		ret = fm_core_send_cmd(RDS_CNTRL_SET, &payload, sizeof(payload),
				       &fmdev->maintask_completion, NULL, NULL);
		FM_CHECK_SEND_CMD_STATUS(ret);

		/* Read flags - just to clear any
		 * pending interrupts if we had
		 */
		ret = fm_core_send_cmd(FLAG_GET, NULL, 2,
				       &fmdev->maintask_completion, NULL, NULL);
		FM_CHECK_SEND_CMD_STATUS(ret);

		/* Set RDS FIFO threshold value */
		FM_STORE_LE16_TO_BE16(payload, FM_RX_RDS_FIFO_THRESHOLD);
		ret = fm_core_send_cmd(RDS_MEM_SET, &payload, sizeof(payload),
				       &fmdev->maintask_completion, NULL, NULL);
		FM_CHECK_SEND_CMD_STATUS(ret);

		/* Enable RDS interrupt */
		fmdev->irq_info.mask |= FM_RDS_EVENT;
		FM_STORE_LE16_TO_BE16(payload, fmdev->irq_info.mask);
		ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
				       &fmdev->maintask_completion, NULL, NULL);
		FM_CHECK_SEND_CMD_STATUS(ret);

		/* Update our local flag */
		fmdev->rx.rds.flag = FM_RX_RDS_ENABLE;
	} else if (rds_en_dis == FM_RX_RDS_DISABLE
		   && fmdev->rx.rds.flag == FM_RX_RDS_ENABLE) {
		/* Turn off RX RDS */
		/* Turn off RDS circuit */
		FM_STORE_LE16_TO_BE16(payload, FM_RX_POWER_SET_FM_ON_RDS_OFF);
		ret = fm_core_send_cmd(POWER_SET, &payload, sizeof(payload),
				       &fmdev->maintask_completion, NULL, NULL);
		FM_CHECK_SEND_CMD_STATUS(ret);

		/* Reset RDS pointers */
		fmdev->rx.rds.last_block_index = 0;
		fmdev->rx.rds.wr_index = 0;
		fmdev->rx.rds.rd_index = 0;
		fm_core_rx_reset_curr_station_info();

		/* Update RDS local cache */
		fmdev->irq_info.mask &= ~(FM_RDS_EVENT);
		fmdev->rx.rds.flag = FM_RX_RDS_DISABLE;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_set_rds_mode(unsigned char rds_en_dis)
{
	int ret;

	FMDRV_API_START();
	switch (fmdev->curr_fmmode) {
	case FM_MODE_RX:
		ret = fm_core_rx_set_rds_mode(rds_en_dis);
		break;

	case FM_MODE_TX:
		ret = fm_core_tx_set_rds_mode(rds_en_dis);
		break;

	default:
		ret = -EINVAL;
	}
	FMDRV_API_EXIT(ret);
	return ret;
}

/* Returns current RX RDS enable/disable status */
int fm_core_rx_get_rds_mode(unsigned char *curr_rds_en_dis)
{
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (curr_rds_en_dis == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*curr_rds_en_dis = fmdev->rx.rds.flag;
	FMDRV_API_EXIT(0);
	return 0;
}

/* Sets RDS operation mode (RDS/RDBS) */
int fm_core_rx_set_rds_system(unsigned char rds_mode)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (rds_mode != FM_RDS_SYSTEM_RDS && rds_mode != FM_RDS_SYSTEM_RBDS) {
		FM_DRV_ERR("Invalid rds mode");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	/* Set RDS operation mode */
	FM_STORE_LE16_TO_BE16(payload, (unsigned short)rds_mode);
	ret = fm_core_send_cmd(RDS_SYSTEM_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	fmdev->rx.rds_mode = rds_mode;
	FMDRV_API_EXIT(0);
	return 0;
}

/* Returns current RDS operation mode */
int fm_core_rx_get_rds_system(unsigned char *rds_mode)
{
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (rds_mode == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*rds_mode = fmdev->rx.rds_mode;
	FMDRV_API_EXIT(0);
	return 0;
}

/* Sends power off command to the chip */
int fm_core_power_down(void)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();
	if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
		FM_DRV_ERR("FM core is not ready");
		FMDRV_API_EXIT(-EPERM);
		return -EPERM;
	}
	if (fmdev->curr_fmmode == FM_MODE_OFF) {
		FM_DRV_ERR("FM chip is already in OFF state");
		FMDRV_API_EXIT(0);
		return 0;
	}
	/* Disable FM functions over Channel-8 */
	FM_STORE_LE16_TO_BE16(payload, 0x0);
	ret = fm_core_send_cmd(FM_POWER_MODE, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	/* Unlink from ST. FM GPIO toggling is taken
	 * care in Shared Transport layer
	 */
	ret = fm_core_release();
	if (ret < 0) {
		FM_DRV_ERR("FM CORE release failed");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Configures Alternate Frequency switch mode */
int fm_core_rx_set_af_switch(unsigned char af_mode)
{
	unsigned short payload;
	int ret;

	FMDRV_API_START();

	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (af_mode != FM_RX_RDS_AF_SWITCH_MODE_ON &&
	    af_mode != FM_RX_RDS_AF_SWITCH_MODE_OFF) {
		FM_DRV_ERR("Invalid af mode");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	/* Enable/disable low RSSI interrupt based on af_mode */
	if (af_mode == FM_RX_RDS_AF_SWITCH_MODE_ON)
		fmdev->irq_info.mask |= FM_LEV_EVENT;
	else
		fmdev->irq_info.mask &= ~FM_LEV_EVENT;

	FM_STORE_LE16_TO_BE16(payload, fmdev->irq_info.mask);
	ret = fm_core_send_cmd(INT_MASK_SET, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	FM_CHECK_SEND_CMD_STATUS(ret);

	fmdev->rx.af_mode = af_mode;

	FMDRV_API_EXIT(0);
	return 0;
}

/* Returns Alternate Frequency switch status */
int fm_core_rx_get_af_switch(unsigned char *af_mode)
{
	int ret;

	FMDRV_API_START();
	ret = fm_core_check_mode(FM_MODE_RX);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	if (af_mode == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	*af_mode = fmdev->rx.af_mode;
	FMDRV_API_EXIT(0);
	return 0;
}

/* Reads init command from FM firmware file and loads to the chip */
static int fm_core_download_firmware(const char *firmware_name)
{
	const struct firmware *fw_entry;
	struct bts_header *fw_header;
	struct bts_action *action;
	struct bts_action_delay *delay;
	char *fw_data;
	int ret, fw_len, cmd_cnt;

	FMDRV_API_START();

	cmd_cnt = 0;
	set_bit(FM_FIRMWARE_DW_INPROGRESS, &fmdev->flag);

	/* Read firmware data */
	ret = request_firmware(&fw_entry, firmware_name, &fmdev->v4l2dev->dev);
	if (unlikely(ret)) {
		FM_DRV_ERR("Unable to read firmware(%s) content\n",
			   firmware_name);
		FMDRV_API_EXIT(ret);
		return ret;
	}
	FM_DRV_DBG("Firmware(%s) length : %d bytes", firmware_name,
		   fw_entry->size);

	fw_data = (void *)fw_entry->data;
	fw_len = fw_entry->size;

	/* Check TI's magic number in firmware file */
	fw_header = (struct bts_header *)fw_data;
	if (fw_header->magic != FM_FW_FILE_HEADER_MAGIC) {
		release_firmware(fw_entry);
		clear_bit(FM_FIRMWARE_DW_INPROGRESS, &fmdev->flag);
		FM_DRV_ERR("%s not a legal TI firmware file", firmware_name);
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	FM_DRV_DBG("Firmware(%s) magic number : 0x%x", firmware_name,
		   fw_header->magic);

	/* Skip file header info , we already verified it */
	fw_data += sizeof(struct bts_header);
	fw_len -= sizeof(struct bts_header);

	while (fw_data && fw_len > 0) {
		action = (struct bts_action *)fw_data;

		switch (action->type) {

		case ACTION_SEND_COMMAND:	/* Send */

			/* Send the command to chip */
			ret = fm_core_send_cmd(0, action->data, action->size,
					       &fmdev->maintask_completion,
					       NULL, NULL);
			if (unlikely(ret < 0)) {
				release_firmware(fw_entry);
				clear_bit(FM_FIRMWARE_DW_INPROGRESS,
					  &fmdev->flag);
				FMDRV_API_EXIT(ret);
				return ret;
			}
			cmd_cnt++;
			break;

		case ACTION_DELAY:	/* Delay */
			delay = (struct bts_action_delay *)action->data;
			mdelay(delay->msec);
			break;
		}

		fw_data += (sizeof(struct bts_action) + (action->size));
		fw_len -= (sizeof(struct bts_action) + (action->size));
	}

	release_firmware(fw_entry);
	clear_bit(FM_FIRMWARE_DW_INPROGRESS, &fmdev->flag);

	FM_DRV_DBG("Firmare commands(%d) loaded to the chip", cmd_cnt);
	FMDRV_API_EXIT(0);
	return 0;
}

/* Loads default RX configuration to the chip */
static int __fm_core_rx_load_default_configuration(void)
{
	int ret;
	FMDRV_API_START();

	/* Set default RX volume level */
	ret = fm_core_rx_set_volume(FM_DEFAULT_RX_VOLUME);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Set default RX RSSI level threshold */
	ret = fm_core_rx_set_rssi_threshold(FM_DEFAULT_RSSI_THRESHOLD);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Does FM power on sequence */
static int fm_core_power_up(unsigned char fw_option)
{
	unsigned short payload, asic_id, asic_ver;
	int resp_len, ret;
	char fw_name[50];

	FMDRV_API_START();
	if (fw_option >= FM_MODE_ENTRY_MAX) {
		FM_DRV_ERR("Invalid firmware download option");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	/* Initialize FM Core. FM GPIO toggling is
	 * taken care in Shared Transport
	 */
	ret = fm_core_prepare();
	if (ret < 0) {
		FM_DRV_ERR("Unable to prepare FM CORE");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Enable Channel-8 communication on chip side. This is must for
	 * reading ASIC ID and ASIC version of the chip.
	 */
	FM_STORE_LE16_TO_BE16(payload, FM_ENABLE);

	ret = fm_core_send_cmd(FM_POWER_MODE, &payload, sizeof(payload),
			       &fmdev->maintask_completion, NULL, NULL);
	if (ret < 0) {
		FM_DRV_ERR("Failed enable FM over Channel 8");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	/* Allow the chip to settle down in Channel-8 mode */
	msleep(5);

	/* Read ASIC ID */
	ret = fm_core_send_cmd(ASIC_ID_GET, NULL, sizeof(asic_id),
			       &fmdev->maintask_completion, &asic_id,
			       &resp_len);
	if (ret < 0) {
		FM_DRV_ERR("Failed read FM chip ASIC ID");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Read ASIC version */
	ret = fm_core_send_cmd(ASIC_VER_GET, NULL, sizeof(asic_ver),
			       &fmdev->maintask_completion, &asic_ver,
			       &resp_len);
	if (ret < 0) {
		FM_DRV_ERR("Failed read FM chip ASIC Version");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	FM_DRV_DBG("ASIC ID: 0x%x , ASIC Version: %d", FM_BE16_TO_LE16(asic_id),
		   FM_BE16_TO_LE16(asic_ver));

	/* Frame common firmware file name and load init commands */
	sprintf(fw_name, "%s_%x.%d.bts", FM_FMC_FW_FILE_START,
		FM_BE16_TO_LE16(asic_id), FM_BE16_TO_LE16(asic_ver));
	ret = fm_core_download_firmware(fw_name);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	/* Frame TX or RX firmware file name
	 * based on request and load init commands.
	 */
	sprintf(fw_name, "%s_%x.%d.bts", (fw_option == FM_MODE_RX) ?
		FM_RX_FW_FILE_START : FM_TX_FW_FILE_START,
		FM_BE16_TO_LE16(asic_id), FM_BE16_TO_LE16(asic_ver));
	ret = fm_core_download_firmware(fw_name);
	if (ret < 0) {
		FMDRV_API_EXIT(ret);
		return ret;
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* ----- Fucntions exported to FM mixer control layer ----- */

/* All the functions which are exported from this file
 * to FM mixer controls layer will check FM Core ready
 * status. If FM Core is not ready, it will return
 * immediately with an error.
 */

/* Set FM Modes(TX, RX, OFF) */
int fm_core_mode_set(unsigned char fm_mode)
{
	int ret;

	FMDRV_API_START();

	if (fm_mode >= FM_MODE_ENTRY_MAX) {
		FM_DRV_ERR("Invalid FM mode");
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}
	if (fmdev->curr_fmmode == fm_mode) {
		FM_DRV_DBG("Already fm is in mode(%d)", fm_mode);
		FMDRV_API_EXIT(0);
		return 0;
	}
	switch (fm_mode) {
	case FM_MODE_OFF:	/* OFF Mode */
		ret = fm_core_power_down();
		if (ret < 0) {
			FM_DRV_ERR("Failed to set OFF mode");
			FMDRV_API_EXIT(ret);
			return ret;
		}
		break;

	case FM_MODE_TX:	/* TX Mode */
	case FM_MODE_RX:	/* RX Mode */
		/* Power down before switching to TX or RX mode */
		if (fmdev->curr_fmmode != FM_MODE_OFF) {
			ret = fm_core_power_down();
			if (ret < 0) {
				FM_DRV_ERR("Failed to set OFF mode");
				FMDRV_API_EXIT(ret);
				return ret;
			}
			msleep(30);
		}
		/* Load requested mode's firmware */
		ret = fm_core_power_up(fm_mode);
		if (ret < 0) {
			FM_DRV_ERR("Failed to load firmware\n");
			FMDRV_API_EXIT(ret);
			return ret;
		}
	}
	/* Set current mode */
	fmdev->curr_fmmode = fm_mode;

	/* Set default configuration */
	if (fmdev->curr_fmmode == FM_MODE_RX) {
		FM_DRV_DBG("Loading default rx configuration..\n");
		ret = __fm_core_rx_load_default_configuration();
		if (ret < 0) {
			FM_DRV_ERR("Failed to load default values\n");
			FMDRV_API_EXIT(ret);
			return ret;
		}
	}
	FMDRV_API_EXIT(0);
	return 0;
}

/* Returns current FM mode (TX, RX, OFF) */
int fm_core_mode_get(unsigned char *fmmode)
{
	FMDRV_API_START();

	if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
		FM_DRV_ERR("FM core is not ready");
		return -EPERM;
	}
	if (fmmode == NULL) {
		FM_DRV_ERR("Invalid memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}

	*fmmode = fmdev->curr_fmmode;

	FMDRV_API_EXIT(0);
	return 0;
}

/* ----------------- FM Core Init -------------- */

/* This function will be called from FM V4L2 open function.
 * This will request FM ST interface to register with ST driver.
 */
int fm_core_prepare(void)
{
	int ret;

	FMDRV_API_START();

	if (test_bit(FM_CORE_READY, &fmdev->flag)) {
		FM_DRV_DBG("FM Core is already up");
		FMDRV_API_EXIT(0);
		return 0;
	}
	if (!test_bit(FM_CORE_TRANSPORT_READY, &fmdev->flag)) {
		FM_DRV_ERR("FM Core transport is not ready");
		FMDRV_API_EXIT(-EPERM);
		return -EPERM;
	}

	/* Request FM ST to get register with ST driver. Send FM Core RX
	 * queue and RX tasklet pointers to FM ST. FM ST will push
	 * received FM packet into this queue and schedule this RX
	 * tasklet.
	 */
	ret = fm_st_register(&fmdev->rx_q, &fmdev->rx_task);
	if (ret < 0) {
		fm_st_release();
		FM_DRV_ERR("Unable to register with ST");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	spin_lock_init(&fmdev->rds_buff_lock);
	spin_lock_init(&fmdev->resp_skb_lock);

	/* Initialize TX queue and TX tasklet */
	skb_queue_head_init(&fmdev->tx_q);
	tasklet_init(&fmdev->tx_task, fm_core_tx_tasklet, (unsigned long)fmdev);

	/* Initialize RX Queue and RX tasklet */
	skb_queue_head_init(&fmdev->rx_q);
	tasklet_init(&fmdev->rx_task, fm_core_rx_tasklet, (unsigned long)fmdev);

	/* Initialize interrupt related info */
	fmdev->irq_info.stage_index = 0;

	/* Number of packet can send at a time */
	atomic_set(&fmdev->tx_cnt, 1);

	fmdev->response_completion = NULL;

	init_timer(&fmdev->irq_info.int_timeout_timer);
	fmdev->irq_info.int_timeout_timer.function =
	    &fm_core_int_timeout_handler;

	/* Default interrupt bits to be enabled */
	fmdev->irq_info.mask =
	    FM_MAL_EVENT /*| FM_STIC_EVENT <<Enable this later>> */ ;

	/* Region info */
	memcpy(&fmdev->rx.region, &region_configs[default_radio_region],
	       sizeof(struct region_info));

	fmdev->rx.curr_mute_mode = FM_MUTE_OFF;
	fmdev->rx.curr_rf_depend_mute = FM_RX_RF_DEPENDENT_MUTE_OFF;
	fmdev->rx.rds.flag = FM_RX_RDS_DISABLE;
	fmdev->rx.curr_freq = FM_UNDEFINED_FREQ;
	fmdev->rx.rds_mode = FM_RDS_SYSTEM_RDS;
	fmdev->rx.af_mode = FM_RX_RDS_AF_SWITCH_MODE_OFF;
	fmdev->irq_info.irq_service_timeout_retry = 0;

	/* RDS ring buffer */
	fm_core_rx_reset_rds_cache();
	init_waitqueue_head(&fmdev->rx.rds.read_queue);

	/* Current station info */
	fm_core_rx_reset_curr_station_info();

	/* FM core is ready */
	set_bit(FM_CORE_READY, &fmdev->flag);

	FMDRV_API_EXIT(0);
	return 0;
}

/* -------------- FM Core De-Init ------------- */

/* This function will be called from FM V4L2 release function.
 * This will request FM ST interface to unregister from ST driver.
 */
int fm_core_release(void)
{
	int ret;

	FMDRV_API_START();

	if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
		FM_DRV_DBG("FM Core is already down");
		FMDRV_API_EXIT(0);
		return 0;
	}
	/* Sevice pending read */
	wake_up_interruptible(&fmdev->rx.rds.read_queue);

	/* Stop all tasklets */
	tasklet_kill(&fmdev->tx_task);
	tasklet_kill(&fmdev->rx_task);

	/* Flush Tx and Rx queues */
	skb_queue_purge(&fmdev->tx_q);
	skb_queue_purge(&fmdev->rx_q);

	fmdev->response_completion = NULL;
	fmdev->rx.curr_freq = 0;

	/* Unlink from Shared Transport driver */
	ret = fm_st_unregister();
	if (ret < 0) {
		FM_DRV_ERR("Unable to unregister from ST");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	FM_DRV_DBG("Successfully unregistered from ST");

	/* Clear FM Core ready flag */
	clear_bit(FM_CORE_READY, &fmdev->flag);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_setup_transport(void)
{
	int ret;

	FMDRV_API_START();

	if (test_bit(FM_CORE_TRANSPORT_READY, &fmdev->flag)) {
		FM_DRV_ERR("FM Core transport is already up");
		FMDRV_API_EXIT(0);
		return 0;
	}
	/* Check availabilty of FM ST. If FM ST is already claimed
	 * by FM Char device interface, don't proceed. Both FM Core
	 * and FM Char device interface can't co-exist.
	 */
	ret = fm_st_claim();
	if (ret < 0) {
		FM_DRV_ERR
		    ("FM char device interface is using FM ST, can't coexist");
		FMDRV_API_EXIT(ret);
		return ret;
	}
	set_bit(FM_CORE_TRANSPORT_READY, &fmdev->flag);

	FMDRV_API_EXIT(0);
	return 0;
}

int fm_core_release_transport(void)
{
	int ret;

	FMDRV_API_START();

	if (!test_bit(FM_CORE_TRANSPORT_READY, &fmdev->flag)) {
		FM_DRV_ERR("FM Core transport is already down");
		FMDRV_API_EXIT(0);
		return 0;
	}

	/* Release FM ST */
	ret = fm_st_release();
	if (ret < 0) {
		FM_DRV_ERR("Failed to release FM ST");
		FMDRV_API_EXIT(ret);
		return ret;
	}

	clear_bit(FM_CORE_TRANSPORT_READY, &fmdev->flag);

	FMDRV_API_EXIT(0);
	return 0;
}

/* ----------- Module Init interface --------- */

static int __init fm_drv_init(void)
{
	int ret;

	FMDRV_API_START();

	ret = 0;

	FM_DRV_DBG("FM driver version %s", FM_DRV_VERSION);

	/* Allocate local resource memory */
	fmdev = kzalloc(sizeof(struct fmdrv_ops), GFP_KERNEL);
	if (!fmdev) {
		FM_DRV_ERR("Can't allocate operation structure memory");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}
	fmdev->rx.rds.buf_size = default_rds_buf * FM_RDS_BLOCK_SIZE;

	/* Allocate memory for RDS ring buffer */
	fmdev->rx.rds.buffer = kzalloc(fmdev->rx.rds.buf_size, GFP_KERNEL);
	if (fmdev->rx.rds.buffer == NULL) {
		kfree(fmdev);
		FM_DRV_ERR("Can't allocate rds ring buffer");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}

	/* Register with V4L2 subsystem. This will internally
	 * expose '/dev/radio' device to user space.
	 */
	ret = fm_v4l2_init_video_device(fmdev);
	if (ret < 0) {
		kfree(fmdev);
		FMDRV_API_EXIT(ret);
		return ret;
	}

	/* Register with ALSA subsystem. This will internally expose
	 * set of FM mixer controls via ALSA to user space.
	 */
	ret = fm_mixer_init(fmdev);
	if (ret < 0) {
		/* Unregister from V4L2 subsystem */
		fm_v4l2_deinit_video_device(fmdev);

		kfree(fmdev);
		FMDRV_API_EXIT(ret);
		return ret;
	}

	/* Initialize the FM Character driver */
	ret = fm_chr_init();
	if (ret < 0) {
		FM_DRV_ERR("Can't init FM Character driver");
		FMDRV_API_EXIT(0);
		return ret;
	}

	/* Assign interrupt handling table pointer */
	fmdev->irq_info.fm_IntActionHandlerTable = g_IntHandlerTable;

	/* Default FM driver mode */
	fmdev->curr_fmmode = FM_MODE_OFF;

	FMDRV_API_EXIT(0);
	return 0;
}

/* -------- Module Exit interface -------- */

static void __exit fm_drv_exit(void)
{
	int ret;

	FMDRV_API_START();

	ret = 0;

	/* Unregister from V4L2 subsystem */
	ret = fm_v4l2_deinit_video_device(fmdev);
	if (ret < 0)
		FM_DRV_ERR("Unable to unregister from V4L2 subsystem(%d)", ret);

	/* Unregister from ALSA subsystem */
	ret = fm_mixer_deinit(fmdev);
	if (ret < 0)
		FM_DRV_ERR("Unable to unregister from ALSA(%d)", ret);

	/* De-initialize the FM Character driver */
	fm_chr_exit();

	kfree(fmdev->rx.rds.buffer);
	kfree(fmdev);

	FMDRV_API_EXIT(0);
}

module_init(fm_drv_init);
module_exit(fm_drv_exit);

/* ------------- Module Info ------------- */

MODULE_AUTHOR("Raja Mani <raja_mani@ti.com>");
MODULE_DESCRIPTION("FM Driver for Connectivity chip of Texas Instruments. "
		   FM_DRV_VERSION);
MODULE_VERSION(FM_DRV_VERSION);
MODULE_LICENSE("GPL");
