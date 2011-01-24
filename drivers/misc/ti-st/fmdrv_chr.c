/*
 *   FM Driver for Connectivity chip of Texas Instruments.
 *
 *   Copyright (C) 2009 Texas Instruments
 *   Written by Raghavendra Shenoy (x0099675@ti.com)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>

#include <linux/tty.h>
#include <linux/sched.h>

#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/uaccess.h>

#include "fmdrv_st.h"
#include "fmdrv_chr.h"

/* Structure that has FM character driver data */
struct fm_chrdrv_ops *fm_chr_dev;

/* File operations structure initialization */
const struct file_operations fm_chr_ops = {
	.owner = THIS_MODULE,
	.fasync = fm_chr_fasync,
	.open = fm_chr_open,
	.read = fm_chr_read,
	.write = fm_chr_write,
	.poll = fm_chr_poll,
	.ioctl = fm_chr_ioctl,
	.release = fm_chr_release,
};

/* Converting Channel-8 response to Channel-4 response */
static struct sk_buff *convert2_channel_4(struct sk_buff *ch8_skb)
{
	struct evt_cmd_complete *evt;
	struct sk_buff *ch4_skb;

	unsigned char ncmds;	/* num_hci cmds */
	unsigned char status;
	unsigned char pkt_type;

	unsigned short opcode;
	unsigned short hci_vs_opcode;

	unsigned char *ptr, *chan4_mem;
	unsigned char chan4_index, chan8_index;
	unsigned char num_hci, param_len, fm_opcode;
	unsigned char read_write, evt_code, cmd_len, int_len, chan8_cmd_len;

	FM_CHR_DRV_START();

	fm_opcode = read_write = 0;
	chan8_index = chan4_index = 0;
	int_len = num_hci = cmd_len = param_len = 0;
	opcode = 0x0000;

	ptr = NULL;
	ch4_skb = NULL;
	chan4_mem = NULL;

	chan8_cmd_len = ch8_skb->data[CHAN8_RESP_CMD_LEN_POS];
	if ((chan8_cmd_len + CHAN8_RESP_CMD_LEN_SIZE + CHAN8_RESP_TYPE_SIZE) !=
	    ch8_skb->len) {
		FM_CHR_DRV_ERR(" Channel-8 response length mismatch ");

		return NULL;
	}

	/* Copy the opcode from Channel-1 command packet.
	 * Channel-8 response packets can only be converted into Channel-4
	 * HCI command complete event packets.
	 *
	 * Mark the packet type as Channel-4 packet type.
	 */
	evt_code = CHAN4_CMD_COMPLETE_TYPE;

	/* Copy the opcode from Channel-8 reponse packet */
	fm_opcode = ch8_skb->data[CHAN8_RESP_FM_OPCODE_POS];

	/* Conversion logic for response packets with Power mode opcode */
	if (fm_opcode == CHAN8_FM_PWR_OPCODE) {
		ch4_skb = alloc_skb(7, GFP_ATOMIC);
		if (ch4_skb == NULL) {
			FM_CHR_DRV_ERR(" SKB allocation failed ");
			return NULL;
		}

		/* Copy from Channel-8 reponse packet */
		pkt_type = CHAN4_PKT_TYPE;
		evt_code = CHAN4_CMD_COMPLETE_TYPE;	/* cmd complete event */
		ncmds = ch8_skb->data[CHAN8_RESP_NUM_HCI_POS];/* num_hci cmds */
		param_len = 4;	/* fd37 + num hci cmds */
		hci_vs_opcode = CHAN1_FM_PWR_OPCODE;
		status = ch8_skb->data[CHAN8_RESP_STATUS_POS];

		/* Copy to Channel-4 response packet */
		memcpy(skb_put(ch4_skb, 1), &pkt_type, 1);
		memcpy(skb_put(ch4_skb, 1), &evt_code, 1);
		memcpy(skb_put(ch4_skb, 1), &param_len, 1);
		memcpy(skb_put(ch4_skb, 1), &ncmds, 1);
		memcpy(skb_put(ch4_skb, 2), &hci_vs_opcode, 2);
		memcpy(skb_put(ch4_skb, 1), &status, 1);

		/* Trim the data to specified length */
		skb_trim(ch4_skb, 7);

		return ch4_skb;
	}

	/* To accomodate an extra 0 that Channel-4 response sends */
	ch4_skb = alloc_skb((ch8_skb->len + 1), GFP_ATOMIC);
	if (ch4_skb == NULL) {
		FM_CHR_DRV_ERR(" SKB allocation failed ");

		return NULL;
	}
	skb_put(ch4_skb, ch8_skb->len + 1);

	/*  Copy the command length, NUM_HCI, R/W values from Channl-8 reponse
	 *  packet
	 */
	cmd_len = ch8_skb->data[CHAN8_RESP_CMD_LEN_POS];
	num_hci = ch8_skb->data[CHAN8_RESP_NUM_HCI_POS];
	read_write = ch8_skb->data[CHAN8_RESP_RW_POS];

	/* Update Channel-4 response packet's opcode bit */
	if (read_write == CHAN8_RD_RESP)
		opcode = CHAN1_READ_OPCODE;
	else
		opcode = CHAN1_WRITE_OPCODE;

	param_len = ch8_skb->data[CHAN8_RESP_PARAM_LEN_POS];

	/* Copying the header-data to Channel-4 response packet */
	ch4_skb->data[CHAN4_TYPE_POS] = CHAN4_PKT_TYPE;
	ch4_skb->data[CHAN4_EVT_TYPE_POS] = evt_code;

	/* Copying the parameters from Channel-8 response packets to Channel-4
	 * response packets.
	 */
	evt = (void *)&ch4_skb->data[CHAN4_EVT_POS];
	evt->opcode = opcode;
	evt->ncmd = num_hci;

	chan8_index = CHAN8_RESP_PARAM_POS;
	chan4_index = CHAN4_PARAM_POS;

	/* Allocate memory to copy the parameters. The parameters are
	 * copied from Channel-8 response packet to this memory and then
	 * copied to Channel-4 response packet.
	 */
	chan4_mem = kzalloc((param_len) * (sizeof(unsigned char)), GFP_ATOMIC);

	if (chan4_mem == NULL) {
		FM_CHR_DRV_ERR(" chan4_mem == NULL ");

		return ch8_skb;
	}

	int_len = param_len;
	ptr = (void *)chan4_mem;

	/* The parameters are copied from Channel-8 response packet to the
	*  intermediate memory location.
	*/
	while (param_len--) {
		*ptr = ch8_skb->data[chan8_index];
		ptr++;
		chan8_index++;
	}

	/* Copy the extra 0 */
	ch4_skb->data[chan4_index] = 0x00;
	chan4_index++;

	param_len = int_len;
	ptr = (void *)chan4_mem;

	/* The parameters are copied from intermediate memory location
	 * to Channel-4 response packet.
	 */
	while (param_len--) {
		ch4_skb->data[chan4_index++] = *ptr;
		ptr++;
	}
	kfree(chan4_mem);
	/* plen would be chan8_index - pkt_type - evt code */
	ch4_skb->data[CHAN4_PLEN_POS] =
	    chan8_index - CHAN4_PKT_TYPE_SIZE - CHAN4_EVT_TYPE_SIZE;

	return ch4_skb;

}

/* Convert the Channel-1 commands into Channel-8 commands */
static int convert2_channel_8(struct sk_buff *skb)
{
	struct hci_command_hdr *hdr;

	unsigned short chan8_index, chan1_index;
	unsigned char fm_opcode, read_write, num_params, chan1_cmd_len;

	/* FM power ON/OFF commands */
	unsigned char fm_pwr_on[] = { 0x08, 0x05, 0xFE, 0x00,
				      0x02, 0x00, 0x01 };
	unsigned char fm_pwr_off[] = { 0x08, 0x05, 0xFE, 0x00,
				      0x02, 0x00, 0x00 };

	FM_CHR_DRV_START();

	hdr = NULL;
	fm_opcode = 0;
	read_write = 0;
	num_params = 0;
	chan8_index = 0;
	chan1_index = 0;

	chan1_cmd_len = skb->data[CHAN1_CMD_LEN_POS];

	if ((chan1_cmd_len + CHAN1_TYPE_SIZE + CHAN1_OPCODE_SIZE +
	     CHAN1_CMD_LEN_SIZE) != skb->len) {
		FM_CHR_DRV_ERR(" Channel-1 length mismatch ");

		return -1;
	}

	/* Copy the opcode from Channel-1 command packet */
	hdr = (void *)&skb->data[CHAN1_TYPE_POS];

	/* Conversion for commands with Power mode opcode */
	if (hdr->opcode == CHAN1_FM_PWR_OPCODE) {
		/* Set skb length to zero and skb tail pointer
		 * to data pointer.
		 */
		skb_trim(skb, 0);

		/* For FM Power ON command */
		if (skb->data[CHAN1_FM_OPCODE_POS] == 0x01) {
			memcpy(skb_put(skb, sizeof(fm_pwr_on)), fm_pwr_on,
			       sizeof(fm_pwr_on));
		} else {
			/* For Power OFF command */
			memcpy(skb_put(skb, sizeof(fm_pwr_off)), fm_pwr_off,
			       sizeof(fm_pwr_off));
		}

		return 0;
	}

	/* Copy the parameters from Channel-1 command packet */
	/* Only the LSB of this value will be used */
	num_params = skb->data[CHAN1_PARAM_LEN_POS];
	fm_opcode = skb->data[CHAN1_FM_OPCODE_POS];
	chan1_index = CHAN1_PARAM_POS;

	/* Copy the Channel-1 header data to Channel-8 header fields */
	read_write =
	    (hdr->opcode == CHAN1_READ_OPCODE) ? CHAN8_RD_CMD : CHAN8_WR_CMD;
	skb->data[CHAN8_TYPE_POS] = CHAN8_PKT_TYPE;

	/* cmd_length/skb->data[1] = fm_opcode + read_write + param_len
	 * in addition to num_params
	 */
	skb->data[CHAN8_FM_OPCODE_POS] = fm_opcode;
	skb->data[CHAN8_RW_POS] = read_write;
	chan8_index = CHAN8_PARAM_POS;

	skb->data[CHAN8_CMD_LEN_POS] = num_params + CHAN8_OPCODE_SIZE +
	    CHAN8_RD_WR_SIZE + CHAN8_PARAM_LEN_SIZE;

	/* Copy the Channel-1 parameters to Channel-8 parameter field */

	/* READ command except read_hw_register */
	if ((read_write == CHAN8_RD_CMD) &&
	    (fm_opcode != CHAN1_HW_REG_OPCODE)) {
		/* Expected bytes to read */
		skb->data[CHAN8_PARAM_LEN_POS] = num_params;

		chan8_index = CHAN8_PARAM_POS;
		skb->data[CHAN8_CMD_LEN_POS] = CHAN8_OPCODE_SIZE +
		    CHAN8_RD_WR_SIZE + CHAN8_PARAM_LEN_SIZE;
	} else {

		/* skb->data[4] contains length of parameters following */
		skb->data[CHAN8_PARAM_LEN_POS] = num_params;

		while (num_params) {
			skb->data[chan8_index] = skb->data[chan1_index];
			chan8_index++;
			chan1_index++;
			num_params--;
		}
	}

	skb_trim(skb, chan8_index);

	return 0;
}

/* Tasklet function which will be scheduled by FM ST sub-module
 * when data is received
 */
long fm_rx_task(void)
{
	struct sk_buff *skb;

#ifdef VERBOSE
	int len;
#endif

	FM_CHR_DRV_START();

	skb = NULL;
	spin_lock(&fm_chr_dev->lock);

	if (skb_queue_empty(&fm_chr_dev->rx_q)) {
		FM_CHR_DRV_ERR(" Rx Queue empty ");
		spin_unlock(&fm_chr_dev->lock);
		return FM_CHR_DRV_ERR_FAILURE;
	}

	/* Dequeue the skb received from the FM_ST interface */
	skb = skb_dequeue(&fm_chr_dev->rx_q);
	if (skb == NULL) {
		FM_CHR_DRV_ERR(" Dequeued skb from Rx queue is NULL ");
		spin_unlock(&fm_chr_dev->lock);
		return FM_CHR_DRV_ERR_FAILURE;
	}

	/* Send a signal to TI FM stack when an Interrupt packet arrives */
	if (skb->data[CHAN8_RESP_FM_OPCODE_POS] == CHAN8_FM_INTERRUPT) {
		FM_CHR_DRV_VER(" Interrupt arrived: ");

#ifdef VERBOSE
		for (len = 0; ((skb) && (len < skb->len)); len++)
			printk(KERN_INFO " 0x%02x ", skb->data[len]);
		printk("\n");
#endif

		kfree_skb(skb);

		/* kill_fasync here - Sends the signal. Check if the queue is
		 * empty. If not, it means that command complete response for
		 * the previous command is not sent to the stack. Don't send
		 * the Signal. Set the SIGNAL_PENDING bit. This bit is cleared
		 * in the read() of the stack, when it reads the command
		 * complete response.
		 */
		if (likely(skb_queue_empty(&fm_chr_dev->rx_q))) {
			/* Should come here most often */
			kill_fasync(&fm_chr_dev->fm_fasync, SIGIO, POLLIN);
			clear_bit(SIGNAL_PENDING, &fm_chr_dev->state_flags);
		} else {
			FM_CHR_DRV_VER(" signal not sent..");
			set_bit(SIGNAL_PENDING, &fm_chr_dev->state_flags);
		}

		spin_unlock(&fm_chr_dev->lock);

		return FM_CHR_DRV_SUCCESS;
	}

	/* Queue it back to the queue if the received packet
	 * is not an interrupt packet
	 */
	skb_queue_head(&fm_chr_dev->rx_q, skb);
	wake_up_interruptible(&fm_chr_dev->fm_data_q);

	spin_unlock(&fm_chr_dev->lock);

	return FM_CHR_DRV_SUCCESS;
}

/* Functions called from the TI-FM stack */
int fm_chr_fasync(int fd, struct file *file, int on)
{
	FM_CHR_DRV_START();

	return fasync_helper(fd, file, on, &fm_chr_dev->fm_fasync);
}

int fm_chr_open(struct inode *inod, struct file *fil)
{
	int err;
	int err_st_release;

	FM_CHR_DRV_START();

	err = FM_CHR_DRV_SUCCESS;
	err_st_release = FM_CHR_DRV_SUCCESS;

	/* Check if V4L2 FM device is already using ST driver */
	if (fm_st_claim() < FM_CHR_DRV_SUCCESS) {
		FM_CHR_DRV_ERR(" Other FM device is already active ");

		return FM_CHR_DRV_ERR_FAILURE;
	}

	/* Register with the FM_ST interface, which will register with the
	 * ST driver
	 */
	err = fm_st_register(&fm_chr_dev->rx_q, &fm_chr_dev->rx_task);
	if (err < FM_CHR_DRV_SUCCESS) {
		FM_CHR_DRV_ERR(" Registration with ST Failed ");
		FM_CHR_DRV_VER(" Releasing FM ST Interface... ");

		err_st_release = fm_st_release();
		if (err_st_release < FM_CHR_DRV_SUCCESS) {
			FM_CHR_DRV_ERR(" Failed to release FM ST (%d)",
				       err_st_release);

			return err;
		}
		FM_CHR_DRV_VER(" Successfully released FM_ST interface ");

		return err;
	}

	/* Initialize the wait queue and the Rx tasklets */
	init_waitqueue_head(&fm_chr_dev->fm_data_q);
	tasklet_init(&fm_chr_dev->rx_task, (void *)fm_rx_task,
		     (unsigned long)fm_chr_dev);

	FM_CHR_DRV_VER(" st_register completed");

	return FM_CHR_DRV_SUCCESS;
}

int fm_chr_release(struct inode *inod, struct file *fil)
{
	int err;

	FM_CHR_DRV_START();

	err = 0;

	/* Clear the asynchronous notifications to the process linked
	 * with the file descriptor 'fil'
	 */
	fm_chr_fasync(-1, fil, 0);

	/* Release the tasklets and the queues */
	tasklet_kill(&fm_chr_dev->rx_task);
	skb_queue_purge(&fm_chr_dev->rx_q);

	/* Unregister FM with ST driver */
	err = fm_st_unregister();
	if (err < FM_CHR_DRV_SUCCESS) {
		FM_CHR_DRV_ERR(" Unable to unregister from ST ");

		return err;
	}
	FM_CHR_DRV_VER(" Successfully unregistered from ST ");

	/* Release FM ST - FM character driver is not using
	 * the ST driver
	 */
	err = fm_st_release();
	if (err < FM_CHR_DRV_SUCCESS) {
		FM_CHR_DRV_ERR(" Failed to release FM ST ");

		return err;
	}
	FM_CHR_DRV_VER(" Successfully released FM_ST interface ");

	return FM_CHR_DRV_SUCCESS;
}

ssize_t fm_chr_write(struct file *fil, const char __user *data,
		     size_t size, loff_t *offset)
{
	int err;
	struct sk_buff *skb;

#ifdef VERBOSE
	int len;
#endif

	FM_CHR_DRV_START();

	err = FM_CHR_DRV_SUCCESS;

	if (data[CHAN1_TYPE_POS] != CHAN1_PKT_TYPE) {
		FM_CHR_DRV_ERR(" Packet type is not Channel-1 ");

		return FM_CHR_DRV_ERR_FAILURE;
	}

	/* Allocate the SKB */
	skb = alloc_skb(size, GFP_ATOMIC);
	if (!skb) {
		FM_CHR_DRV_ERR(" Did not allocate SKB ");

		return FM_CHR_DRV_ERR_FAILURE;
	}

	/* Forward the data from the user space to ST core */
	if (copy_from_user(skb_put(skb, size), data, size)) {
		FM_CHR_DRV_ERR(" Unable to copy from user space");
		kfree_skb(skb);

		return -EFAULT;
	}
#ifdef VERBOSE
	printk(KERN_INFO "Write: Before conversion\n");
	for (len = 0; ((skb) && (len < skb->len)); len++)
		printk(KERN_INFO "0x%02x ", skb->data[len]);
	printk("\n");
#endif

	/* Convert the Channel-1 command packet to Channel-8 command packet */
	if (convert2_channel_8(skb) < 0) {
		FM_CHR_DRV_ERR(" Conversion to Channel-8 failed ");
		kfree_skb(skb);

		return FM_CHR_DRV_ERR_FAILURE;
	}
#ifdef VERBOSE
	printk(KERN_INFO "Write: After conversion\n");
	for (len = 0; ((skb) && (len < skb->len)); len++)
		printk(KERN_INFO "0x%02x ", skb->data[len]);
	printk("\n");
#endif

	/* Write to ST driver through FM_ST interface */
	err = fm_st_send(skb);
	if (err < FM_CHR_DRV_SUCCESS) {
		FM_CHR_DRV_ERR(" Cannot write to ST, fm_st_send failed (%d)",
			       err);
		kfree_skb(skb);

		return FM_CHR_DRV_ERR_FAILURE;
	}

	init_waitqueue_head(&fm_chr_dev->fm_data_q);

	return size;
}

ssize_t fm_chr_read(struct file *fil, char __user *data, size_t size,
		    loff_t *offset)
{
	int len;
	struct sk_buff *skb;
	struct sk_buff *ch4_skb;

	FM_CHR_DRV_START();

	skb = NULL;
	ch4_skb = NULL;

	/* Wait till the data is available */
	if (!wait_event_interruptible_timeout(fm_chr_dev->fm_data_q,
					      !skb_queue_empty
					      (&fm_chr_dev->rx_q), 500)) {

		FM_CHR_DRV_ERR(" Read timed out ");

		return -EAGAIN;
	}
	FM_CHR_DRV_VER(" Completed Read wait ");

	spin_lock(&fm_chr_dev->lock);

	/* Dequeue the data from the queue if the data
	 * is already available
	 */
	if (!skb_queue_empty(&fm_chr_dev->rx_q))
		skb = skb_dequeue(&fm_chr_dev->rx_q);

	spin_unlock(&fm_chr_dev->lock);

	if (unlikely
	    (skb == NULL || skb->len == 0 || skb->data == NULL
	     || skb->cb[0] != CHAN8_PKT_TYPE)) {
		FM_CHR_DRV_VER(" Length of the response packet is invalid ");
		kfree_skb(skb);

		return FM_CHR_DRV_ERR_FAILURE;
	}
#ifdef VERBOSE
	printk(KERN_INFO "Read: Before conversion\n");
	for (len = 0; len < skb->len; len++)
		printk(KERN_INFO "0x%02x ", skb->data[len]);
	printk("\n");
#endif

	/* Convert if the response packet is of Channel-8 type */
	if (!skb->data[CHAN8_RESP_CMD_LEN_POS]) {
		kfree(skb);

		return FM_CHR_DRV_ERR_FAILURE;
	}

	/* Convert the Channel-8 response packet to Channel-4 response packet */
	ch4_skb = convert2_channel_4(skb);

	if (ch4_skb == NULL) {
		FM_CHR_DRV_ERR(" Converted SKB is NULL ");
		kfree(skb);

		return FM_CHR_DRV_ERR_FAILURE;
	}
#ifdef VERBOSE
	printk(KERN_INFO "Read: After conversion\n");
	for (len = 0; (ch4_skb != NULL) && len < (ch4_skb->len); len++)
		printk(KERN_INFO "0x%02x ", ch4_skb->data[len]);
	printk("\n");
#endif

	if (size >= ch4_skb->len) {
		/* Forward the data to the user */
		if (copy_to_user(data, ch4_skb->data, ch4_skb->len)) {
			FM_CHR_DRV_ERR(" Unable to copy to user space ");
			kfree_skb(skb);
			kfree_skb(ch4_skb);
			spin_unlock(&fm_chr_dev->lock);

			return -EFAULT;
		}
	} else {
		FM_CHR_DRV_ERR(" Buffer overflow ");
		kfree_skb(skb);
		kfree_skb(ch4_skb);

		return -ENOMEM;
	}

	len = ch4_skb->len;
	kfree(skb);
	kfree(ch4_skb);

	/* Clear the pending bit that was set when an interrupt packet had
	 * arrived
	 */
	if (test_bit(SIGNAL_PENDING, &fm_chr_dev->state_flags)
	    && skb_queue_empty(&fm_chr_dev->rx_q)) {
		FM_CHR_DRV_DBG(" Sending signal ");

		kill_fasync(&fm_chr_dev->fm_fasync, SIGIO, POLLIN);
		clear_bit(SIGNAL_PENDING, &fm_chr_dev->state_flags);
	}

	return len;
}

unsigned int fm_chr_poll(struct file *file, poll_table * wait)
{
	unsigned long mask;

	FM_CHR_DRV_DBG(" inside %s", __func__);

	mask = 0;

	/* Poll and wait */
	poll_wait(file, &fm_chr_dev->fm_data_q, wait);

	spin_lock(&fm_chr_dev->lock);

	FM_CHR_DRV_VER(" Completed poll ");

	if (!skb_queue_empty(&fm_chr_dev->rx_q))
		mask |= POLLIN;

	FM_CHR_DRV_VER(" return 0x%02x \n", (unsigned int)mask);

	spin_unlock(&fm_chr_dev->lock);

	return mask;
}

int fm_chr_ioctl(struct inode *inode, struct file *file,
		 unsigned int cmd, unsigned long arg)
{
	FM_CHR_DRV_START();

	return FM_CHR_DRV_SUCCESS;
}

/* FM character driver init: Initializes the FM character driver parametes */
int fm_chr_init(void)
{
	FM_CHR_DRV_START();

	fm_chr_dev = kzalloc(sizeof(struct fm_chrdrv_ops), GFP_ATOMIC);

	if (fm_chr_dev == NULL) {
		FM_CHR_DRV_ERR
		    (" Cannot allocate memory for FM character driver ");

		return -ENOMEM;
	}

	skb_queue_head_init(&fm_chr_dev->rx_q);

	/* Initialize wait queue and a flag, so that the
	 * wait queue can be used later
	 */
	init_completion(&fm_chr_dev->reg_completed);
	init_waitqueue_head(&fm_chr_dev->fm_data_q);

	/* Expose the device FM_CHAR_DEVICE_NAME to user space
	 * and obtain the major number for the device
	 */
	fm_chr_dev->fm_chr_major =
	    register_chrdev(0, FM_CHAR_DEVICE_NAME, &fm_chr_ops);
	FM_CHR_DRV_VER(" allocated %d, %d", fm_chr_dev->fm_chr_major, 0);

	if (fm_chr_dev->fm_chr_major < 0) {
		FM_CHR_DRV_ERR(" register_chrdev failed ");

		return FM_CHR_DRV_ERR_FAILURE;
	}

	/* udev */
	fm_chr_dev->fm_chr_class =
	    class_create(THIS_MODULE, FM_CHAR_DEVICE_NAME);
	if (IS_ERR(fm_chr_dev->fm_chr_class)) {
		FM_CHR_DRV_ERR(" Something went wrong in class_create");

		return FM_CHR_DRV_ERR_CLASS;
	}

	fm_chr_dev->fm_chr_dev =
	    device_create(fm_chr_dev->fm_chr_class, NULL,
			  MKDEV(fm_chr_dev->fm_chr_major, 0), NULL,
			  FM_CHAR_DEVICE_NAME);

	if (IS_ERR(fm_chr_dev->fm_chr_dev)) {
		FM_CHR_DRV_ERR(" Cannot create the device %s ",
			       FM_CHAR_DEVICE_NAME);

		return -ENODEV;
	}

	return FM_CHR_DRV_SUCCESS;
}

/* FM character driver init: Destroys the FM character driver parametes */
void fm_chr_exit(void)
{
	FM_CHR_DRV_START();

	FM_CHR_DRV_VER(" Freeing up %d", fm_chr_dev->fm_chr_major);

	skb_queue_purge(&fm_chr_dev->rx_q);
	device_destroy(fm_chr_dev->fm_chr_class,
		       MKDEV(fm_chr_dev->fm_chr_major, 0));

	class_unregister(fm_chr_dev->fm_chr_class);
	class_destroy(fm_chr_dev->fm_chr_class);

	unregister_chrdev(fm_chr_dev->fm_chr_major, FM_CHAR_DEVICE_NAME);
}
