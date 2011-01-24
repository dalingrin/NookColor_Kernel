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

#ifndef _FM_CHR_DRV_H
#define _FM_CHR_DRV_H

/* #define VERBOSE */

/* Debug macros */
#define FM_CHR_DRV_ERR(fmt, arg...) \
	pr_err("(fm_chr_drv):"fmt"\n" , ## arg)

#if defined(DEBUG)	/* limited debug messages */
#define FM_CHR_DRV_DBG(fmt, arg...) \
	pr_info("(fm_chr_drv):"fmt"\n" , ## arg)
#define FM_CHR_DRV_VER(fmt, arg...)
#define	FM_CHR_DRV_START()	\
	pr_info("(fm_chr_drv): Inside %s\n", __func__)
#elif defined(VERBOSE)	/* very verbose */
#define FM_CHR_DRV_DBG(fmt, arg...) \
	pr_info("(fm_chr_drv):"fmt"\n" , ## arg)
#define FM_CHR_DRV_VER(fmt, arg...) \
	pr_info("(fm_chr_drv):"fmt"\n" , ## arg)
#define FM_CHR_DRV_START()	\
	pr_info("(fm_chr_drv): Inside %s\n", __func__)
#else /* Error msgs only */
#define FM_CHR_DRV_DBG(fmt, arg...)
#define FM_CHR_DRV_VER(fmt, arg...)
#define FM_CHR_DRV_START()
#endif

/* FM device */
#define FM_CHAR_DEVICE_NAME 	"tifm"

/* Macros for packet conversion */
#define	CHAN1_FM_PWR_OPCODE		0xFD37
#define	CHAN1_READ_OPCODE		0xFD33
#define	CHAN1_WRITE_OPCODE		0xFD35

#define CHAN1_HW_REG_OPCODE		0x64
#define CHAN8_FM_PWR_OPCODE		0xFE
#define CHAN8_FM_INTERRUPT		0xFF
#define CHAN8_FM_INTERRUPT_OPCODE	0xF0

/* Packet types */
#define CHAN1_PKT_TYPE		0x01
#define CHAN8_PKT_TYPE		0x08
#define CHAN4_PKT_TYPE		0x04
#define CHAN4_CMD_COMPLETE_TYPE	0x0E

#define CHAN8_RD_CMD	0x01
#define CHAN8_WR_CMD	0x00

#define CHAN8_RD_RESP	0x01
#define CHAN8_WR_RESP	0x00

/* Channel-4 data sizes */
#define CHAN4_PKT_TYPE_SIZE	1
#define CHAN4_EVT_TYPE_SIZE	1

/* Channel-8 data sizes */
#define CHAN8_OPCODE_SIZE 	1
#define CHAN8_RD_WR_SIZE 	1
#define CHAN8_PARAM_LEN_SIZE 	1

/* Channel-8 response data sizes */
#define CHAN8_RESP_TYPE_SIZE	1
#define CHAN8_RESP_CMD_LEN_SIZE 1

/* HCI-VS data sizes */
#define CHAN1_TYPE_SIZE		1
#define CHAN1_OPCODE_SIZE	2
#define CHAN1_CMD_LEN_SIZE	1

/* HCI-VS data positions */
#define CHAN1_TYPE_POS		0
#define CHAN1_OPCODE_POS	1
#define CHAN1_CMD_LEN_POS	3
#define CHAN1_FM_OPCODE_POS	4
#define CHAN1_PARAM_LEN_POS	5

/* Because of an extra Zero appended to param_len */
#define CHAN1_PARAM_POS		7

/* Channel-8 command positions */
#define CHAN8_TYPE_POS		0
#define CHAN8_CMD_LEN_POS	1
#define CHAN8_FM_OPCODE_POS	2
#define CHAN8_RW_POS		3
#define CHAN8_PARAM_LEN_POS	4
#define CHAN8_PARAM_POS		5

/* Channel-8 response positions */
#define CHAN8_RESP_CMD_LEN_POS		1
#define CHAN8_RESP_STATUS_POS		2
#define CHAN8_RESP_NUM_HCI_POS		3
#define CHAN8_RESP_FM_OPCODE_POS	4
#define CHAN8_RESP_RW_POS		5
#define CHAN8_RESP_PARAM_LEN_POS	6
#define CHAN8_RESP_PARAM_POS		7

/* Channel-4 data positions */
#define CHAN4_TYPE_POS		0
#define CHAN4_EVT_TYPE_POS	1
#define CHAN4_PLEN_POS		2
#define CHAN4_EVT_POS		3
#define CHAN4_PARAM_POS		6

/* Forward declaration file operations */
void fm_chr_exit(void);
int fm_chr_init(void);

int fm_chr_fasync(int, struct file *, int);

int fm_chr_open(struct inode *, struct file *);
int fm_chr_release(struct inode *, struct file *);

ssize_t fm_chr_write(struct file *, const char __user *, size_t, loff_t *);
ssize_t fm_chr_read(struct file *, char __user *, size_t, loff_t *);
unsigned int fm_chr_poll(struct file *, poll_table *);

int fm_chr_ioctl(struct inode *, struct file *, unsigned int, unsigned long);

long fm_rx_task(void);

/* List of error codes returned by the FM driver */
enum {
	FM_CHR_DRV_ERR_FAILURE = -1,	/* check struct */
	FM_CHR_DRV_SUCCESS,
	FM_CHR_DRV_ERR_PENDING = -5,	/* to call reg_complete_cb */
	FM_CHR_DRV_ERR_ALREADY,	/* already registered */
	FM_CHR_DRV_ERR_INPROGRESS,
	FM_CHR_DRV_ERR_NOPROTO,	/* protocol not supported */
	FM_CHR_DRV_ERR_CLASS = -15,
	FM_CHR_DRV_READ_TIMEOUT,
};

/* Header structure for HCI command packet */
struct hci_command_hdr {
	uint8_t prefix;
	uint16_t opcode;
	uint8_t plen;
} __attribute__ ((packed));

/* Header structure for HCI event packet */
struct evt_cmd_complete {
	uint8_t ncmd;
	uint16_t opcode;
} __attribute__ ((packed));

/* Flag to send the signal after the command complete
 * event's skb is removed from the RX queue
 */
#define SIGNAL_PENDING	1

/* FM Character driver data */
struct fm_chrdrv_ops {
	int fm_chr_major;
	struct device *fm_chr_dev;
	struct class *fm_chr_class;

	spinlock_t lock;

	unsigned long state_flags;

	struct sk_buff_head rx_q;
	struct tasklet_struct rx_task;

	wait_queue_head_t fm_data_q;
	struct completion reg_completed;

	struct fasync_struct *fm_fasync;

	long (*st_write) (struct sk_buff *);
};

#endif /*_FM_CHR_DRV_H */
