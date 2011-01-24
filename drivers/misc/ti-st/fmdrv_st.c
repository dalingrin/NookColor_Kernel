/*
 *  FM Driver for Connectivity chip of Texas Instruments.
 *
 *  This file provide interfaces to Shared Transport.
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

#include <linux/kernel.h>

#include "st.h"
#include "fmdrv.h"
#include "fmdrv_st.h"

static char streg_cbdata;

/* Wait on comepletion handler needed to synchronize
 * fm_st_register() and fm_st_registration_completion_cb()
 * functions.
 */
static struct completion wait_for_fmdrv_reg_completion;

/* Write function pointer of ST driver */
long (*g_st_write) (struct sk_buff *skb);

/* RX Queue and RX Tasklet pointer */
static struct sk_buff_head *g_rx_q;
static struct tasklet_struct *g_rx_task;

/* Flag to maintain whether FM ST is claimed or not */
static char is_fm_st_claimed;

/* Called from FM Core and FM Char device interface to claim
 * FM ST. Who ever comes first, ownership of FM ST will be
 * given to them.
 */
int fm_st_claim(void)
{
	FMDRV_API_START();

	/* Give ownership of FM ST to first caller */
	if (is_fm_st_claimed == FM_ST_NOT_CLAIMED) {
		is_fm_st_claimed = FM_ST_CLAIMED;

		FMDRV_API_EXIT(FM_ST_SUCCESS);
		return FM_ST_SUCCESS;
	}

	FM_DRV_DBG("FM ST claimed already");

	FMDRV_API_EXIT(FM_ST_FAILED);
	return FM_ST_FAILED;
}

/* Called from FM Core and FM Char device interface
 * to release FM ST.
 */
int fm_st_release(void)
{
	FMDRV_API_START();

	/* Release FM ST if it is already claimed */
	if (is_fm_st_claimed == FM_ST_CLAIMED) {
		is_fm_st_claimed = FM_ST_NOT_CLAIMED;

		FMDRV_API_EXIT(FM_ST_SUCCESS);
		return FM_ST_SUCCESS;

	}

	FM_DRV_ERR("FM ST is not claimed,called again?");

	FMDRV_API_EXIT(FM_ST_FAILED);
	return FM_ST_FAILED;
}

/* Called by Shared Transport layer when FM packet is
 * available
 */
static long fm_st_receive(struct sk_buff *skb)
{
	FMDRV_API_START();

	if (skb == NULL) {
		FM_DRV_ERR("Invalid SKB received from ST");
		FMDRV_API_EXIT(-EFAULT);
		return -EFAULT;
	}

	/* Is this FM Channel-8 packet? */
	if (skb->cb[0] != FM_PKT_LOGICAL_CHAN_NUMBER) {
		FM_DRV_ERR("Received SKB(%p) is not FM Channel 8 pkt", skb);
		FMDRV_API_EXIT(-EINVAL);
		return -EINVAL;
	}

	/* One byte is already reserved for Channel-8 in skb,
	 * so prepend skb with Channel-8 packet type byte.
	 */
	memcpy(skb_push(skb, 1), &skb->cb[0], 1);

	if (g_rx_q != NULL && g_rx_task != NULL) {
		/* Queue FM packet for FM RX task */
		skb_queue_tail(g_rx_q, skb);
		tasklet_schedule(g_rx_task);
	} else {
		FM_DRV_ERR
		    ("Invalid RX queue and RX tasklet pointer,puring skb");
		kfree_skb(skb);
		FMDRV_API_EXIT(-EFAULT);
		return -EFAULT;
	}

	FMDRV_API_EXIT(0);
	return 0;
}

/* Forwards FM Packets to Shared Transport */
int fm_st_send(struct sk_buff *skb)
{
	long len;

	FMDRV_API_START();

	if (skb == NULL) {
		FM_DRV_ERR("Invalid skb, can't send");
		FMDRV_API_EXIT(-ENOMEM);
		return -ENOMEM;
	}

	/* Is anyone called without claiming FM ST? */
	if (is_fm_st_claimed == FM_ST_CLAIMED && g_st_write != NULL) {
		/* Forward FM packet(SKB) to ST for the transmission */
		len = g_st_write(skb);
		if (len < 0) {
			/* Something went wrong in st write , free skb memory */
			kfree_skb(skb);
			FM_DRV_ERR(" ST write failed (%ld)", len);
			FMDRV_API_EXIT(-EAGAIN);
			return -EAGAIN;
		}
	} else {		/* Nobody calimed FM ST */

		kfree_skb(skb);
		FM_DRV_ERR("FM ST is not claimed, Can't send skb");
		FMDRV_API_EXIT(-EAGAIN);
		return -EAGAIN;
	}

	FMDRV_API_EXIT(0);
	return 0;
}

/* Called by ST layer to indicate protocol registration completion
 * status. fm_st_register() function will wait for signal from this
 * API when st_register() function returns ST_PENDING.
 */
static void fm_st_registration_completion_cb(char data)
{
	FMDRV_API_START();

	/* fm_st_register() function needs value of 'data' to know
	 * the registration status(success/fail). So, have a back
	 * up of it.
	 */
	streg_cbdata = data;

	/* Got a feedback from ST for FM driver registration
	 * request. Wackup fm_st_register() function to continue
	 * it's open operation.
	 */
	complete(&wait_for_fmdrv_reg_completion);

	FMDRV_API_EXIT(0);
}

/* Called from V4L2 RADIO open function (fm_fops_open()) to
 * register FM driver with Shared Transport
 */
int fm_st_register(struct sk_buff_head *rx_q, struct tasklet_struct *rx_task)
{
	static struct st_proto_s fm_st_proto;
	unsigned long timeleft;
	int ret;

	ret = 0;

	FMDRV_API_START();

	/* Populate FM driver info required by ST */
	memset(&fm_st_proto, 0, sizeof(fm_st_proto));

	/* FM driver ID */
	fm_st_proto.type = ST_FM;

	/* Receive function which called from ST */
	fm_st_proto.recv = fm_st_receive;

	/* Packet match function may used in future */
	fm_st_proto.match_packet = NULL;

	/* Callback to be called when registration is pending */
	fm_st_proto.reg_complete_cb = fm_st_registration_completion_cb;

	/* This is write function pointer of ST. BT driver will make use of this
	 * for sending any packets to chip. ST will assign and give to us, so
	 * make it as NULL
	 */
	fm_st_proto.write = NULL;

	/* Register with ST layer */
	ret = st_register(&fm_st_proto);
	if (ret == ST_ERR_PENDING) {
		/* Prepare wait-for-completion handler data structures.
		 * Needed to syncronize this and
		 * fm_st_registration_completion_cb() functions.
		 */
		init_completion(&wait_for_fmdrv_reg_completion);

		/* Reset ST registration callback status flag. This value
		 * will be updated in fm_st_registration_completion_cb()
		 * function whenever it is called from ST driver.
		 */
		streg_cbdata = -EINPROGRESS;

		/* ST is busy with other protocol registration (may be busy with
		 * firmware download). So, wait till the registration callback
		 * (passed as a argument to st_register() function) getting
		 * called from ST.
		 */
		FM_DRV_DBG(" %s waiting for reg completion signal from ST",
			   __func__);

		timeleft =
		    wait_for_completion_timeout(&wait_for_fmdrv_reg_completion,
						FM_ST_REGISTER_TIMEOUT);
		if (!timeleft) {
			FM_DRV_ERR("Timeout(%d sec), didn't get reg"
				   "completion signal from ST",
				   jiffies_to_msecs(FM_ST_REGISTER_TIMEOUT) /
				   1000);
			FMDRV_API_EXIT(-ETIMEDOUT);
			return -ETIMEDOUT;
		}

		/* Is ST registration callback called with ERROR value? */
		if (streg_cbdata != 0) {
			FM_DRV_ERR("ST reg completion CB called with invalid"
				   "status %d", streg_cbdata);
			FMDRV_API_EXIT(-EAGAIN);
			return -EAGAIN;
		}
		ret = 0;
	} else if (ret == ST_ERR_FAILURE) {
		FM_DRV_ERR("st_register failed %d", ret);
		FMDRV_API_EXIT(-EAGAIN);
		return -EAGAIN;
	}

	/* Do we have proper ST write function? */
	if (fm_st_proto.write != NULL) {
		/* We need this pointer for sending any FM pkts */
		g_st_write = fm_st_proto.write;
	} else {
		FM_DRV_ERR("Failed to get ST write func pointer");

		/* Undo registration with ST */
		ret = st_unregister(ST_FM);
		if (ret < 0)
			FM_DRV_ERR("st_unregister failed %d", ret);

		FMDRV_API_EXIT(-EAGAIN);
		return -EAGAIN;
	}

	/* Store Rx Q and Rx tasklet pointers. This pointers should
	 * already initialized by caller
	 */
	g_rx_task = rx_task;
	g_rx_q = rx_q;

	FMDRV_API_EXIT(ret);
	return ret;
}

/* Unregister FM Driver from Shared Transport */
int fm_st_unregister(void)
{
	int ret;

	FMDRV_API_START();

	/* Unregister FM Driver from ST */
	ret = st_unregister(ST_FM);
	if (ret != ST_SUCCESS) {
		FM_DRV_ERR("st_unregister failed %d", ret);
		FMDRV_API_EXIT(-EBUSY);
		return -EBUSY;
	}

	g_rx_task = NULL;
	g_rx_q = NULL;

	FMDRV_API_EXIT(0);
	return 0;
}
