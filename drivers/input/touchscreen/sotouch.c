/*
 *  Copyright (c) Multimedia Solutions
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>

#define DRIVER_DESC	"Sensitive Object touchscreen"

/*
 * Definitions & global arrays.
 */

#define	SOTOUCH_MAX_LENGTH	10

#define SOTOUCH_MIN_X 0x0
#define SOTOUCH_MAX_X 0x2F80
#define SOTOUCH_MIN_Y 0x0
#define SOTOUCH_MAX_Y 0x1E88

#define SOT_VER		0x0051
#define SOT_ID		0x0100

/*
 * Per-touchscreen data.
 */

struct sotouch_data {
	struct input_dev *dev;
	struct serio *serio;
	char phys[32];
};

/* PACKET:
    Length	- 1	(0E)
    Header	- 4	(AA-00-AA-AA)
    Data	- 14	(
    CRC		- 1
*/
unsigned char sot_pkg[20];
int pkg_cnt;

// status:
char in_range;
char tip_switch;

// coordiantes:
int sot_x;
int sot_y;



static void sotouch_process_packet(struct sotouch_data* sotouch_data)
{
	struct input_dev *dev = sotouch_data->dev;

	int pressure;	//0x48
	int width;	//0x2
	int touch;	

	// FIRST RECOGNITION (same as RELEASE?)
	if((in_range==1) && (tip_switch==0)) {
		pressure = 0x0;
		width 	 = 0x0;
		touch 	 = 0x0;

	// DOWN
	} else if((in_range==1) && (tip_switch==1)) {
		pressure = 0x48;
		width 	 = 0x2;
		touch 	 = 0x1;
	}

	input_report_abs(dev, ABS_X, sot_x);
	input_report_abs(dev, ABS_Y, sot_y);

	input_report_key(dev, BTN_TOUCH, touch);
	
	// fake (synaptic average)
	input_report_abs(dev, ABS_PRESSURE, pressure);
	input_report_abs(dev, ABS_TOOL_WIDTH, width);
	input_report_abs(dev, ABS_MT_TOUCH_MAJOR, pressure);
	input_report_abs(dev, ABS_MT_WIDTH_MAJOR, width);
	input_report_abs(dev, ABS_MT_POSITION_X, sot_x);
	input_report_abs(dev, ABS_MT_POSITION_Y, sot_y);

	input_sync(dev);
}

/* checks if the first 5 bytes received are the header */
int check_sot_hdr(void)
{ 
    if((sot_pkg[0]!=0x0E)&&(sot_pkg[1]!=0xAA)&&(sot_pkg[2]!=0x00)&&(sot_pkg[3]!=0xAA)&&(sot_pkg[4]!=0xAA))
	return 1;
    else
	return 0;
}

/* ckecks CRC of packet data */
int check_sot_pkg(void)
{
    int i;
    int crc;
    unsigned int data_sum=0;

    for(i=5; i<19; i++)
	data_sum += sot_pkg[i];

    data_sum= data_sum & 0xFF;

    crc = (data_sum%256) & 0xFF;
    if(crc != sot_pkg[19])
	return 1;

    return 0;
}


/* extract data from received package */
int extract_sot_data(void)
{
    int status;

    status = sot_pkg[6];

    // for status only bits 0 and 1 are used
    in_range	= (status >> 1) & 0x1;
    tip_switch	= status & 0x1;

    // X coord
    sot_x = sot_pkg[9]; //MSB first
    sot_x = (sot_x << 8) & 0xFF00;
    sot_x += sot_pkg[8];

    // Y coord
    sot_y = sot_pkg[11]; //MSB first
    sot_y = (sot_y << 8) & 0xFF00;
    sot_y += sot_pkg[10];

    //TODO!!
    if((sot_x > SOTOUCH_MAX_X) || (sot_y > SOTOUCH_MAX_Y)) {
	printk(KERN_DEBUG "sot_x > SOTOUCH_MAX_X | sot_y > SOTOUCH_MAX_Y\n");
	return 1;
    }

    //printk(KERN_DEBUG "-- %d %d 0x%04x 0x%04x\n", in_range, tip_switch, sot_x, sot_y);

    return 0;
}


static irqreturn_t sotouch_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct sotouch_data* sotouch_data = serio_get_drvdata(serio);
	int err;

	//printk(KERN_ALERT "!!! %s: enter (data=0x%02x)\n", __func__, (int)data);
	printk(KERN_DEBUG "data: 0x%02x\n", (int)data);

	sot_pkg[pkg_cnt] = data;
	pkg_cnt++;

	// Received data CRC:
	if(pkg_cnt==20) {

		err = check_sot_hdr();
		err |= check_sot_pkg();
		if(err) {
			int i;
			printk(KERN_DEBUG "Bad CRC!\n");

			/* ignore oldest character */
			for (i=0; i<(pkg_cnt-1); i++)
				sot_pkg[i] = sot_pkg[i+1];
			pkg_cnt--;
		} else {
			pkg_cnt=0; //ready for new packet

			err = extract_sot_data();

			if(!err)
				sotouch_process_packet(sotouch_data);
		}
	}

	return IRQ_HANDLED;
}

/*
 * sotouch_disconnect() is the opposite of sotouch_connect()
 */

static void sotouch_disconnect(struct serio *serio)
{
	struct sotouch_data *sotd = serio_get_drvdata(serio);

	printk(KERN_ERR "!!! %s: enter\n", __func__);


	input_get_device(sotd->dev);
	input_unregister_device(sotd->dev);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_put_device(sotd->dev);
	kfree(sotd);
}

/*
 * sotouch_connect() is the routine that is called when someone adds a
 * new serio device that supports sotouch protocol and registers it as
 * an input device.
 */

static int sotouch_connect(struct serio *serio, struct serio_driver *drv)
{
	struct sotouch_data *sotd;
	struct input_dev *input_dev;
	int err;

	printk(KERN_ERR "!!! %s: enter\n", __func__);

	sotd = kzalloc(sizeof(struct sotouch_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!sotd || !input_dev) {
		err = -ENOMEM;
		goto fail1;
	}

	sotd->serio = serio;
	sotd->dev = input_dev;
	snprintf(sotd->phys, sizeof(serio->phys), "%s/input0", serio->phys);

	input_dev->name = "Sensitive Object touchscreen";
	input_dev->phys = sotd->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = SERIO_SOTOUCH;

	/* ? */
	input_dev->id.product = SOT_ID;
	input_dev->id.version = SOT_VER;

	input_dev->dev.parent = &serio->dev;
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/* ? */
	input_set_abs_params(input_dev, ABS_X, SOTOUCH_MIN_X, SOTOUCH_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, SOTOUCH_MIN_Y, SOTOUCH_MAX_Y, 0, 0);

	// fake (synaptics)
	input_set_abs_params(input_dev, ABS_PRESSURE , 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_TOOL_WIDTH , 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);			//48
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);			//50
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, SOTOUCH_MIN_X, SOTOUCH_MAX_X, 0, 0);	//53
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, SOTOUCH_MIN_Y, SOTOUCH_MAX_Y, 0, 0);	//54

	serio_set_drvdata(serio, sotd);

	err = serio_open(serio, drv);
	if (err)
		goto fail2;

	err = input_register_device(sotd->dev);
	if (err)
		goto fail3;

	return 0;

 fail3:	serio_close(serio);
 fail2:	serio_set_drvdata(serio, NULL);
 fail1:	input_free_device(input_dev);
	kfree(sotd);
	return err;
}

/*
 * The serio driver structure.
 */
static struct serio_device_id sotouch_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_SOTOUCH,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, sotouch_serio_ids);

static struct serio_driver sotouch_drv = {
	.driver		= {
		.name	= "sotouch",
	},
	.description	= DRIVER_DESC,
	.id_table	= sotouch_serio_ids,
	.interrupt	= sotouch_interrupt,
	.connect	= sotouch_connect,
	.disconnect	= sotouch_disconnect,
};

static int __init sotouch_init(void)
{
	return serio_register_driver(&sotouch_drv);
}

static void __exit sotouch_exit(void)
{
	serio_unregister_driver(&sotouch_drv);
}

module_init(sotouch_init);
module_exit(sotouch_exit);

MODULE_AUTHOR("Yavor Trifonov <ytrifonov@mm-sol.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
