/*
*	BMA150 linux driver
* 
* Usage:	BMA150 driver by i2c for linux
*
*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include "smb380.h"
#include "smb380calib.h"

#ifdef BMA150_MODULES
#include "smb380.c"
#include "smb380calib.c"
#endif


#define	BMA150_ENABLE_IRQ
#define EVENT_TIPATP					ABS_X
#define EVENT_SHAKE						ABS_Y
#define EVENT_FFALL						ABS_Z
#define BMA150_DEBUG
//#define BMA150_SMBUS

/* i2c operation for bma150 API */
static char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);
static void bma150_i2c_delay(unsigned int msec);

/* globe variant */
static struct i2c_client *bma150_client = NULL;
struct bma150_data {
	smb380_t			smb380;
	int IRQ;
	struct fasync_struct *async_queue;
};

/*definition for GPIO*/
#ifdef BMA150_ENABLE_IRQ
static int bma150_interrupt_config(void);


#define BMA150_IRQ_PIN					133
//#define OMAP_CTRL_READADDR			0x48002000
//#define OMAP_MMC2_DATA1				0x15C+2
#define BMA150_IRQ						OMAP_GPIO_IRQ(BMA150_IRQ_PIN)

/* config interval of tap-tip */
#define TAPTIP_INTERVAL					60
#define SHAKE_INTERVAL					40
#define LOW_G_THRES						20
#define LOW_G_DUR						150
#define HIGH_G_THRES					160
#define HIGH_G_DUR						60
#define ANY_MOTION_THRES				30
#define ANY_MOTION_CT					2
#endif

#ifdef BMA150_ENABLE_IRQ
static int bma150_interrupt_config()
{
	unsigned char temp;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

/* config interrupt pin for ZOOM_1 omap3430*/
	if(gpio_request(BMA150_IRQ_PIN, "bma150_irq") < 0)
	{
		printk(KERN_ERR "Failed to request GPIO%d for bma150 IRQ\n",
                       BMA150_IRQ_PIN);
               return -1;
	}
	gpio_direction_input(BMA150_IRQ_PIN);

/* low-g interrupt config: 0.156g, 150ms */
	temp = LOW_G_THRES;
	smb380_set_low_g_threshold(temp);
	temp = LOW_G_DUR;
	smb380_set_low_g_duration(temp);

/* high-g interrupt config: 1.50g, 150ms */
	temp = HIGH_G_THRES;
	smb380_set_high_g_threshold(temp);
	temp = HIGH_G_DUR;
	smb380_set_high_g_duration(temp);

/* any motion interrupt config: 0.75g, 3 */
	temp = ANY_MOTION_THRES;
	smb380_set_any_motion_threshold(temp);
	temp = ANY_MOTION_CT;
	smb380_set_any_motion_count(temp);
	return 0;
}
#endif


static irqreturn_t bma150_irq_handler(int irq, void *_id)
{
	struct bma150_data *data;
    unsigned long flags;
	if(((smb380_t*)_id)->chip_id != 0x02)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "%s error\n",__FUNCTION__);
#endif
		return IRQ_HANDLED;
	}
	if(bma150_client == NULL)
		return IRQ_HANDLED;
    printk("bma150 irq handler\n");
    data = i2c_get_clientdata(bma150_client);
    if(data == NULL)
		return IRQ_HANDLED;
	local_irq_save(flags);
    if(data->async_queue)
				kill_fasync(&data->async_queue,SIGIO, POLL_IN);
	local_irq_restore(flags);
	return IRQ_HANDLED;
}


/*	i2c delay routine for eeprom	*/
static inline void bma150_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*	i2c write routine for bma150	*/
static inline char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
#ifndef BMA150_SMBUS
	unsigned char buffer[2];
#endif
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{
#ifdef BMA150_SMBUS
		dummy = i2c_smbus_write_byte_data(bma150_client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(bma150_client, (char*)buffer, 2);
#endif
		reg_addr++;
		data++;
		if(dummy < 0)
			return -1;
	}
	return 0;
}

/*	i2c read routine for bma150	*/
static inline char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{
#ifdef BMA150_SMBUS
		dummy = i2c_smbus_read_byte_data(bma150_client, reg_addr);
		if(dummy < 0)
			return -1;
		*data = dummy & 0x000000ff;
#else
		dummy = i2c_master_send(bma150_client, (char*)&reg_addr, 1);
		if(dummy < 0)
			return -1;
		dummy = i2c_master_recv(bma150_client, (char*)data, 1);
		if(dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
}


/*	read command for BMA150 device file	*/
static ssize_t bma150_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	smb380acc_t acc;
	int ret;
	if( bma150_client == NULL )
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -1;
	}

	smb380_read_accel_xyz(&acc);
#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);
#endif

#if 0
	if( count != sizeof(acc) )
	{
		return -1;
	}
#endif
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150: copy_to_user result: %d\n", ret);
#endif
	}
	return sizeof(acc);
}

/*	write command for BMA150 device file	*/
static ssize_t bma150_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( bma150_client == NULL )
		return -1;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150 should be accessed with ioctl command\n");
#endif
	return 0;
}

/*	open command for BMA150 device file	*/
static int bma150_open(struct inode *inode, struct file *file)
{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	if( bma150_client == NULL)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -1;
	}

#ifdef BMA150_DEBUG
	printk(KERN_INFO "BMA150 has been opened\n");
#endif
	return 0;
}

/*	release command for BMA150 device file	*/
static int bma150_close(struct inode *inode, struct file *file)
{
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	return 0;
}


/*	ioctl command for BMA150 device file	*/
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int temp;
	struct bma150_data* pdata;
	pdata = i2c_get_clientdata(bma150_client);


#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA150_IOC_MAGIC)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA150_IOC_MAXNR)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "cmd number error\n");
#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "cmd access_ok error\n");
#endif
		return -EFAULT;
	}
	/* check bam150_client */
	if( bma150_client == NULL)
	{
#ifdef BMA150_DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -EFAULT;
	}

	/* cmd mapping */

	switch(cmd)
	{
	case BMA150_SOFT_RESET:
		err = smb380_soft_reset();
		return err;

	case BMA150_SET_RANGE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_range(*data);
		return err;

	case BMA150_GET_RANGE:
		err = smb380_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_mode(*data);
		return err;

	case BMA150_GET_MODE:
		err = smb380_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_BANDWIDTH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_bandwidth(*data);
		return err;

	case BMA150_GET_BANDWIDTH:
		err = smb380_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_WAKE_UP_PAUSE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_wake_up_pause(*data);
		return err;

	case BMA150_GET_WAKE_UP_PAUSE:
		err = smb380_get_wake_up_pause(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_threshold(*data);
		return err;

	case BMA150_GET_LOW_G_THRESHOLD:
		err = smb380_get_low_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_countdown(*data);
		return err;

	case BMA150_GET_LOW_G_COUNTDOWN:
		err = smb380_get_low_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_countdown(*data);
		return err;

	case BMA150_GET_HIGH_G_COUNTDOWN:
		err = smb380_get_high_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_duration(*data);
		return err;

	case BMA150_GET_LOW_G_DURATION:
		err = smb380_get_low_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_threshold(*data);
		return err;

	case BMA150_GET_HIGH_G_THRESHOLD:
		err = smb380_get_high_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_duration(*data);
		return err;

	case BMA150_GET_HIGH_G_DURATION:
		err = smb380_get_high_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_threshold(*data);
		return err;

	case BMA150_GET_ANY_MOTION_THRESHOLD:
		err = smb380_get_any_motion_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_COUNT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_count(*data);
		return err;

	case BMA150_GET_ANY_MOTION_COUNT:
		err = smb380_get_any_motion_count(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_INTERRUPT_MASK:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_interrupt_mask(*data);
		return err;

	case BMA150_GET_INTERRUPT_MASK:
		err = smb380_get_interrupt_mask(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_RESET_INTERRUPT:
		err = smb380_reset_interrupt();
		return err;

	case BMA150_READ_ACCEL_X:
		err = smb380_read_accel_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Y:
		err = smb380_read_accel_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Z:
		err = smb380_read_accel_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_GET_INTERRUPT_STATUS:
		err = smb380_get_interrupt_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_int(*data);
		return err;

	case BMA150_SET_HIGH_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_int(*data);
		return err;

	case BMA150_SET_ANY_MOTION_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_int(*data);
		return err;

	case BMA150_SET_ALERT_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_alert_int(*data);
		return err;

	case BMA150_SET_ADVANCED_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_advanced_int(*data);
		return err;

	case BMA150_LATCH_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_latch_int(*data);
		return err;

	case BMA150_SET_NEW_DATA_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_new_data_int(*data);
		return err;

	case BMA150_GET_LOW_G_HYST:
		err = smb380_get_low_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_hysteresis(*data);
		return err;

	case BMA150_GET_HIGH_G_HYST:
		err = smb380_get_high_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_hysteresis(*data);
		return err;

	case BMA150_READ_ACCEL_XYZ:
		err = smb380_read_accel_xyz((smb380acc_t*)data);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_TEMPERATURE:
		err = smb380_read_temperature(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	/* offset calibration routine */
	case BMA150_CALIBRATION:
		if(copy_from_user((smb380acc_t*)data,(smb380acc_t*)arg,6)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		/* iteration time 20 */
		temp = 20;
		err = smb380_calibrate(*(smb380acc_t*)data, &temp);
		return err;

	case BMA150_READ_INT:
		//data[0] = omap_get_gpio_datain(BMA150_IRQ_PIN);
		smb380_read_reg(0x0B,data,1);
		smb380_read_reg(0x15,data+1,1);
		data[0] = data[0] & 0x43;
		data[1] = data[1] & 0x40;
		data[1] <<= 1;
		data[0] |= data[1];
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA150_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	default:
		return 0;
	}
}

static int bma150_fasync(int fd, struct file *file, int mode)
{
    struct bma150_data* data;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	data=i2c_get_clientdata(bma150_client);
	return fasync_helper(fd,file,mode,&data->async_queue);
}

static const struct file_operations bma150_fops = {
	.owner = THIS_MODULE,
	.read = bma150_read,
	.write = bma150_write,
	.open = bma150_open,
	.release = bma150_close,
	.ioctl = bma150_ioctl,
	.fasync = bma150_fasync,
};

/* May 4th 2009 modified*
 * add miscdevices for bma
 */
static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150",
	.fops = &bma150_fops,
};

static int bma150_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "bma150", I2C_NAME_SIZE);

	return 0;
}

static int bma150_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma150_data *data;
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma150_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	/* read chip id */
	tempvalue = 0;
#ifdef BMA150_SMBUS
	tempvalue = i2c_smbus_read_word_data(client, 0x00);
#else
	i2c_master_send(client, (char*)&tempvalue, 1);
	i2c_master_recv(client, (char*)&tempvalue, 1);
#endif
	if((tempvalue & 0x00FF) == 0x0002)
	{
		printk(KERN_INFO "Bosch Sensortec Device detected!\nBMA150/SMB380 registered I2C driver!\n");
		bma150_client = client;
	}
	else
	{
		printk(KERN_INFO "Bosch Sensortec Device not found, i2c error %d \n", tempvalue);
		i2c_detach_client(client);				/*modified on Apr 27 2009*/
		bma150_client = NULL;
		err = -1;
		goto kfree_exit;
	}

	i2c_set_clientdata(bma150_client, data);
	/* May 4th modified for device create*/
	err = misc_register(&bma_device);
	if (err) {
		printk(KERN_ERR "bma150 device register failed\n");
		goto kfree_exit;
	}
	printk(KERN_INFO "bma150 device create ok\n");

	/* bma150 sensor initial */
	data->smb380.bus_write = bma150_i2c_write;
	data->smb380.bus_read = bma150_i2c_read;
	data->smb380.delay_msec = bma150_i2c_delay;
	smb380_init(&data->smb380);

	smb380_set_bandwidth(4);		//bandwidth 375Hz
	smb380_set_range(0);			//range +/-2G

	//close all interrupt
	smb380_set_low_g_int(0);
	smb380_set_high_g_int(0);
	smb380_set_any_motion_int(0);
	smb380_set_alert_int(0);
	smb380_set_advanced_int(1);
	smb380_reset_interrupt();

	/* register interrupt */
#ifdef	BMA150_ENABLE_IRQ

	err = bma150_interrupt_config();
	if (err < 0)
		goto exit_dereg;
	data->IRQ = BMA150_IRQ;
	err = request_irq(data->IRQ, bma150_irq_handler, IRQF_TRIGGER_RISING, "bma150", &data->smb380);
	if (err)
	{
		printk(KERN_ERR "could not request irq\n");
		goto exit_dereg;
	}
#endif
	return 0;

#ifdef BMA150_ENABLE_IRQ


exit_dereg:
    misc_deregister(&bma_device);
#endif
kfree_exit:
	kfree(data);
exit:
	return err;
}


static int bma150_remove(struct i2c_client *client)
{
	struct bma150_data *data = i2c_get_clientdata(client);
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	misc_deregister(&bma_device);
#ifdef BMA150_ENABLE_IRQ
	free_irq(data->IRQ, &data->smb380);
#endif
	i2c_detach_client(client);
	kfree(data);
	bma150_client = NULL;
	return 0;
}


static unsigned short normal_i2c[] = { 0x38, I2C_CLIENT_END};

I2C_CLIENT_INSMOD_1(bma150);

static const struct i2c_device_id bma150_id[] = {
	{ "bma150", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma150_id);

static struct i2c_driver bma150_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "bma150",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= bma150_id,
	.address_data	= &addr_data,
	.probe		= bma150_probe,
	.remove		= bma150_remove,
	.detect		= bma150_detect,
};

static int __init BMA150_init(void)
{
#ifdef BMA150_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	return i2c_add_driver(&bma150_driver);
}

static void __exit BMA150_exit(void)
{
	i2c_del_driver(&bma150_driver);
	printk(KERN_ERR "BMA150 exit\n");
}



MODULE_DESCRIPTION("BMA150 driver");
MODULE_LICENSE("GPL");

module_init(BMA150_init);
module_exit(BMA150_exit);

