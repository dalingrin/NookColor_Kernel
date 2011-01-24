/*
 *  FM Driver for Connectivity chip of Texas Instruments.
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

#ifndef _FM_ST_H
#define _FM_ST_H

#include <linux/interrupt.h>

/* Defines number of seconds to wait for reg completion
 * callback getting called from ST (in case, registration
 * with ST returns PENDING status)
 */
#define FM_ST_REGISTER_TIMEOUT   msecs_to_jiffies(6000)	/* 6 sec */

#define FM_PKT_LOGICAL_CHAN_NUMBER  0x08   /* Logical channel 8 */

/* Claim ownership of FM ST */
int fm_st_claim(void);

/* To release FM ST */
int fm_st_release(void);

/* Forwards FM Packets to Shared Transport */
int fm_st_send(struct sk_buff *skb);

/* Register with Shared Transport */
int fm_st_register(struct sk_buff_head  *rx_q,
		   struct tasklet_struct *rx_task);

/* Unregister from Shared Transport */
int fm_st_unregister(void);

#endif
