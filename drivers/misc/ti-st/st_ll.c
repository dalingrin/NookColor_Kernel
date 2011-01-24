/*
 *  Shared Transport driver
 *  	HCI-LL module responsible for TI proprietary HCI_LL protocol
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

#include "st_ll.h"

/* all debug macros go in here */
#define ST_LL_ERR(fmt, arg...)  printk(KERN_ERR "(stll):"fmt"\n" , ## arg)
#if defined(DEBUG)		/* limited debug messages */
#define ST_LL_DBG(fmt, arg...)  printk(KERN_INFO "(stll):"fmt"\n" , ## arg)
#define ST_LL_VER(fmt, arg...)
#elif defined(VERBOSE)		/* very verbose */
#define ST_LL_DBG(fmt, arg...)  printk(KERN_INFO "(stll):"fmt"\n" , ## arg)
#define ST_LL_VER(fmt, arg...)  printk(KERN_INFO "(stll):"fmt"\n" , ## arg)
#else /* error msgs only */
#define ST_LL_DBG(fmt, arg...)
#define ST_LL_VER(fmt, arg...)
#endif

static struct ll_struct_s *ll;

/**********************************************************************/
/* internal functions */
static void send_ll_cmd(unsigned char cmd)
{

	ST_LL_DBG("%s: writing %x", __func__, cmd);
	st_int_write(&cmd, 1);
	return;
}

static void ll_device_want_to_sleep(void)
{
	ST_LL_DBG("%s", __func__);
	/* sanity check */
	if (ll->ll_state != ST_LL_AWAKE)
		ST_LL_ERR("ERR hcill: ST_LL_GO_TO_SLEEP_IND"
			  "in state %ld", ll->ll_state);

	spin_lock(&ll->lock);
	send_ll_cmd(LL_SLEEP_ACK);
	/* update state */
	ll->ll_state = ST_LL_ASLEEP;
	spin_unlock(&ll->lock);
}

static void ll_device_want_to_wakeup(void)
{
	spin_lock(&ll->lock);
	/* diff actions in diff states */
	switch (ll->ll_state) {
	case ST_LL_ASLEEP:
		send_ll_cmd(LL_WAKE_UP_ACK);	/* send wake_ack */
		break;
	case ST_LL_ASLEEP_TO_AWAKE:
		/* duplicate wake_ind */
		ST_LL_ERR("duplicate wake_ind while waiting for Wake ack");
		break;
	case ST_LL_AWAKE:
		/* duplicate wake_ind */
		ST_LL_ERR("duplicate wake_ind already AWAKE");
		break;
	case ST_LL_AWAKE_TO_ASLEEP:
		/* duplicate wake_ind */
		ST_LL_ERR("duplicate wake_ind");
		break;
	}
	/* update state */
	ll->ll_state = ST_LL_AWAKE;
	spin_unlock(&ll->lock);
}

/**********************************************************************/
/* functions invoked by ST Core */

/* called when ST Core wants to
 * enable ST LL */
void st_ll_enable(void)
{
	ll->ll_state = ST_LL_AWAKE;
}

/* called when ST Core /local module wants to
 * disable ST LL */
void st_ll_disable(void)
{
	ll->ll_state = ST_LL_INVALID;
}

/* called when ST Core wants to update the state */
void st_ll_wakeup(void)
{
	if (likely(ll->ll_state != ST_LL_AWAKE)) {
		send_ll_cmd(LL_WAKE_UP_IND);	/* WAKE_IND */
		ll->ll_state = ST_LL_ASLEEP_TO_AWAKE;
	} else {
		/* don't send the duplicate wake_indication */
		ST_LL_ERR(" Chip already AWAKE ");
	}
}

/* called when ST Core wants the state */
unsigned long st_ll_getstate(void)
{
	ST_LL_DBG(" returning state %ld", ll->ll_state);
	return ll->ll_state;
}

/* called from ST Core, when a PM related packet arrives */
unsigned long st_ll_sleep_state(unsigned char cmd)
{
	switch (cmd) {
	case LL_SLEEP_IND:	/* sleep ind */
		ST_LL_DBG("sleep indication recvd");
		ll_device_want_to_sleep();
		break;
	case LL_SLEEP_ACK:	/* sleep ack */
		ST_LL_ERR("sleep ack rcvd: host shouldn't");
		break;
	case LL_WAKE_UP_IND:	/* wake ind */
		ST_LL_DBG("wake indication recvd");
		ll_device_want_to_wakeup();
		break;
	case LL_WAKE_UP_ACK:	/* wake ack */
		ST_LL_DBG("wake ack rcvd");
		ll->ll_state = ST_LL_AWAKE;
		break;
	default:
		ST_LL_ERR(" unknown input/state ");
		return ST_ERR_FAILURE;
	}
	return ST_SUCCESS;
}

/* Called from ST CORE to initialize ST LL */
long st_ll_init(void)
{
	long err = ST_SUCCESS;

	/* Allocate memory for ST LL private structure */
	ll = kzalloc(sizeof(*ll), GFP_ATOMIC);
	if (!ll) {
		ST_LL_ERR("kzalloc failed to alloc memory for ST LL");
		err = -ENOMEM;
		return err;
	}
	spin_lock_init(&ll->lock);
	/* set state to invalid */
	ll->ll_state = ST_LL_INVALID;
	return err;
}

/* Called from ST CORE to de-initialize ST LL */
long st_ll_deinit(void)
{
	kfree(ll);
	return 0;
}
