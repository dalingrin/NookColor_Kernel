#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pixcir_i2c_s32.h>

//#define PIXCIR_DEBUG
#ifdef PIXCIR_DEBUG
#define pixcir_print(fmt, args...)		printk(fmt, ##args)
#else
#define pixcir_print(fmt, args...)		/* not debugging: nothing */
#endif


#define MAP_TO_7_INCH_LCD 1

#define PIXCIR_9IN_ABS_X_MAX	1024
#define PIXCIR_9IN_ABS_Y_MAX 	768

//PIXCIR S32 setting
#ifdef MAP_TO_7_INCH_LCD
#define PIXCIR_TS_ABS_X_MIN 	0
#define PIXCIR_TS_ABS_X_MAX 	848 	// 1024 * (6" / 7.25")
#define PIXCIR_TS_ABS_Y_MIN 	0
#define PIXCIR_TS_ABS_Y_MAX 	524	// 768 * (3.5" / 5.125")
#else
#define PIXCIR_TS_ABS_X_MIN 	0
#define PIXCIR_TS_ABS_X_MAX 	PIXCIR_9IN_ABS_X_MAX
#define PIXCIR_TS_ABS_Y_MIN 	0
#define PIXCIR_TS_ABS_Y_MAX 	PIXCIR_9IN_ABS_Y_MAX
#endif

#define PIXCIR_TS_FUZZ 	0
#define PIXCIR_TS_FLAT 	0

static struct workqueue_struct *pixcir_wq;

struct pixcir_ts_data {
	uint32_t	version;
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	int snap_state[2][2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	uint32_t irqflags;
	int reported_finger_count;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_ts_early_suspend(struct early_suspend *h);
static void pixcir_ts_late_resume(struct early_suspend *h);
#endif

static int pixcir_init_panel(struct pixcir_ts_data *ts)
{
	return 0;
}

static void pixcir_ts_work_func(struct work_struct *work)
{
	//define ouput data start
	int finger;
	int previous_state;
        static int z = 10;		// Set a default value for TP IC,this is fake data
	static int w = 1;		// Set a default value for TP IC,this is fake data
	int finger2_pressed;
	int touch_state;
        int x1; 
	int y1; 
        int x2; 
	int y2; 
 	//end of define data
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t buf[10];
	uint8_t buf1[1];
	struct pixcir_ts_data *ts = container_of(work, struct pixcir_ts_data, work);

	msg[0].addr = ts->client->addr;				//Write start address : 0x00
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf1;
        buf1[0] = 0x00;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		bad_data = 1;
		goto done;
	} else {
		 //pixcir_print("Pixcir_ts_work_func: %x %x %x %x %x %x" 
		 //       " %x %x %x %x %x %x, ret %d\n", 
		 //       buf[0], buf[1], buf[2], buf[3], 
		 //       buf[4], buf[5], buf[6], buf[7], 
		 //       buf[8], buf[9], buf[10], buf[11], ret); 
		/* RAW XY coordinate */
		/*
			(1024,768)						(0,768)
			   	+---------------------------+
				|						        |
				|			    				 |
				|			    				 |
				|			    				 |
				|			    				 |
 				+---------------------------+ 
			(1024,0)	 [conn]					(0,0)
					 [ up  ]
		*/


		 
		bad_data = 0;
		{
			previous_state = buf[1];
			touch_state = buf[0] & 0xff;
 			if (touch_state == 1) {
				finger = 1;
				finger2_pressed = 0;
			}
			else if (touch_state == 2) {
				finger = 2;
				finger2_pressed = 1;
                        }
			else {
 				finger = 0;
				finger2_pressed = 0;
			}
                        /* XY coordinate */
                        x1 = buf[2] | (uint16_t)buf[3] << 8; 
			y1 = buf[4] | (uint16_t)buf[5] << 8;
			x2 = buf[6] | (uint16_t)buf[7] << 8; 
			y2 = buf[8] | (uint16_t)buf[9] << 8; 

			/* flip Y axis */
			y1 = PIXCIR_9IN_ABS_Y_MAX - y1;
			y2 = PIXCIR_9IN_ABS_Y_MAX - y2;

			/* constrain to max X/Y */
			x1 = (x1 > PIXCIR_TS_ABS_X_MAX) ? PIXCIR_TS_ABS_X_MAX : x1;
			y1 = (y1 > PIXCIR_TS_ABS_Y_MAX) ? PIXCIR_TS_ABS_Y_MAX : y1;
			x2 = (x2 > PIXCIR_TS_ABS_X_MAX) ? PIXCIR_TS_ABS_X_MAX : x2;
			y2 = (y2 > PIXCIR_TS_ABS_Y_MAX) ? PIXCIR_TS_ABS_Y_MAX : y2;

			// For kernel 1.6 multi touch use 
			if (finger != 0) {
				pixcir_print("x1 = %4d, y1 = %4d, Finger %2d\n", x1, y1, finger);
				input_report_abs(ts->input_dev, ABS_X, x1);					//X1 coordinate
				input_report_abs(ts->input_dev, ABS_Y, y1);		//Y1 coordinate
			}
			input_report_abs(ts->input_dev, ABS_PRESSURE, z);
			input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
			input_report_key(ts->input_dev, BTN_TOUCH, finger);
			input_report_key(ts->input_dev, BTN_2, finger2_pressed);
                                                    
			if (finger2_pressed) {
				pixcir_print("x2 = %4d, y2 = %4d, Previous state %2d\n", x2, y2, previous_state);
				input_report_abs(ts->input_dev, ABS_HAT0X, x2);					//X2 coordinate
				input_report_abs(ts->input_dev, ABS_HAT0Y, y2);		//Y2 coordinate			
			}

			/* For kernel 2.0 multi touch use */
 			if (!finger) {
				z = 0;		//Reset midiwave value
				w = 0;		//Reset midiwave value
			}
			//pixcir_print("x1 = %4d, y1 = %4d, Finger %2d\n", x1, y1, finger);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y1);
			input_mt_sync(ts->input_dev);

			z = z + 10;
			w = w + 1;
                        if (z > 60)
				z = 10;
                        if (w > 3)
				w = 1;

			if (finger2_pressed) {
				pixcir_print("x2 = %4d, y2 = %4d, Previous state %2d\n", x2, y2, previous_state);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, z+1);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w+1);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x2);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y2);
				input_mt_sync(ts->input_dev);
			}
  			else if (ts->reported_finger_count > 1) {
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
				input_mt_sync(ts->input_dev);
			}
			ts->reported_finger_count = finger;
			/* End of multi touch */

			input_sync(ts->input_dev);                              
		}
	}             
done:
	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart pixcir_ts_timer_func(struct hrtimer *timer)
{
	struct pixcir_ts_data *ts = container_of(timer, struct pixcir_ts_data, timer);
	queue_work(pixcir_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);			//report rate 1/12500000=80Hz
	return HRTIMER_NORESTART;
}

static irqreturn_t pixcir_ts_irq_handler(int irq, void *dev_id)
{
	struct pixcir_ts_data *ts = dev_id;

	//pixcir_print(KERN_ERR "Pixcir_ts_irq_handler\n"); 
	disable_irq(ts->client->irq);
	queue_work(pixcir_wq, &ts->work);
	return IRQ_HANDLED;
}


static int pixcir_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pixcir_ts_data *ts;
	uint8_t buf0[2];
	struct i2c_msg msg[1];
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct pixcir_i2c_s32_platform_data *pdata;
	int inactive_area_left;
	int inactive_area_right;
	int inactive_area_top;
	int inactive_area_bottom;
	int snap_left_on;
	int snap_left_off;
	int snap_right_on;
	int snap_right_off;
	int snap_top_on;
	int snap_top_off;
	int snap_bottom_on;
	int snap_bottom_off;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "Pixcir_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, pixcir_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "Pixcir_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}
		inactive_area_left = 0;
		inactive_area_right = 0;
		inactive_area_top = 0;
		inactive_area_bottom = 0;
		snap_left_on = 0;
		snap_left_off = 0;
		snap_right_on = 0;
		snap_right_off = 0;
		snap_top_on = 0;
		snap_top_off = 0;
		snap_bottom_on = 0;
		snap_bottom_off = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
/*
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf0;
	buf0[0] = 0x14;		//Active mode
	buf0[1] = 0x00;
        ret = i2c_transfer(ts->client->adapter, msg, 1);
*/

	i2c_smbus_write_byte_data(ts->client, 0x15, 0x09);

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf0;
	buf0[0] = 0x37;		//Calibration mode
	buf0[1] = 0x03;
        ret = i2c_transfer(ts->client->adapter, msg, 1);


	ts->max[0] = max_x = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
	ts->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
	if (ts->flags & PIXCIR_SWAP_XY)
		swap(max_x, max_y);

	ret = pixcir_init_panel(ts); 
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "Pixcir_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "Pixcir-CTP-S32";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	printk("Pixcir_ts_probe: ---------------------------------------------------setting ready\n");

	printk(KERN_INFO "==>: max_x %4d, max_y %4d\n", PIXCIR_TS_ABS_X_MAX, PIXCIR_TS_ABS_Y_MAX);

	input_set_abs_params(ts->input_dev, ABS_X,
			     PIXCIR_TS_ABS_X_MIN, PIXCIR_TS_ABS_X_MAX, PIXCIR_TS_FUZZ, PIXCIR_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_Y,
			     PIXCIR_TS_ABS_Y_MIN, PIXCIR_TS_ABS_Y_MAX, PIXCIR_TS_FUZZ, PIXCIR_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 3, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0X,
			     PIXCIR_TS_ABS_X_MIN, PIXCIR_TS_ABS_X_MAX, PIXCIR_TS_FUZZ, PIXCIR_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y,
			     PIXCIR_TS_ABS_Y_MIN, PIXCIR_TS_ABS_Y_MAX, PIXCIR_TS_FUZZ, PIXCIR_TS_FLAT);

	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 
			     PIXCIR_TS_ABS_X_MIN, PIXCIR_TS_ABS_X_MAX, PIXCIR_TS_FUZZ, PIXCIR_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     PIXCIR_TS_ABS_Y_MIN, PIXCIR_TS_ABS_Y_MAX, PIXCIR_TS_FUZZ, PIXCIR_TS_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 3, fuzz_w, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "==>: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	if (client->irq) {
		ret = request_irq(client->irq, pixcir_ts_irq_handler, pdata->irqflags, client->name, ts);
		if (ret == 0) {
			ts->use_irq = 1;
			//free_irq(client->irq, ts);
			printk(KERN_ERR "==>: Request IRQ ok, ret=%d\n", ret);
		}
		else {
			ts->use_irq = 0;
			dev_err(&client->dev, "request_irq failed\n");
			free_irq(client->irq, ts);	
			printk(KERN_ERR "==>: Request IRQ errir, ret=%d\n", ret);
		}		
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = pixcir_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = pixcir_ts_early_suspend;
	ts->early_suspend.resume = pixcir_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	printk(KERN_INFO "==>:Touchscreen Up %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int pixcir_ts_remove(struct i2c_client *client)
{
	struct pixcir_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int pixcir_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct pixcir_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0); /* disable interrupt */
	if (ret < 0)
		printk(KERN_ERR "Pixcir_ts_suspend: i2c_smbus_write_byte_data failed\n");

	ret = i2c_smbus_write_byte_data(client, 0xf0, 0x86); /* deep sleep */
	if (ret < 0)
		printk(KERN_ERR "Pixcir_ts_suspend: i2c_smbus_write_byte_data failed\n");
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "Pixcir_ts_resume power off failed\n");
	}
	return 0;
}

static int pixcir_ts_resume(struct i2c_client *client)
{
	int ret;
	struct pixcir_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "Pixcir_ts_resume power on failed\n");
	}

	pixcir_init_panel(ts);

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else
		i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_ts_early_suspend(struct early_suspend *h)
{
	struct pixcir_ts_data *ts;
	ts = container_of(h, struct pixcir_ts_data, early_suspend);
	pixcir_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void pixcir_ts_late_resume(struct early_suspend *h)
{
	struct pixcir_ts_data *ts;
	ts = container_of(h, struct pixcir_ts_data, early_suspend);
	pixcir_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id pixcir_ts_id[] = {
	{ PIXCIR_I2C_S32_NAME, 0 },
	{ }
};

static struct i2c_driver pixcir_ts_driver = {
	.probe		= pixcir_ts_probe,
	.remove		= pixcir_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= pixcir_ts_suspend,
	.resume		= pixcir_ts_resume,
#endif
	.id_table	= pixcir_ts_id,
	.driver = {
		.name	= PIXCIR_I2C_S32_NAME,
	},
};

static int __devinit pixcir_ts_init(void)
{
	pixcir_wq = create_singlethread_workqueue("pixcir_wq");
	if (!pixcir_wq)
		return -ENOMEM;
	return i2c_add_driver(&pixcir_ts_driver);
}

static void __exit pixcir_ts_exit(void)
{
	i2c_del_driver(&pixcir_ts_driver);
	if (pixcir_wq)
		destroy_workqueue(pixcir_wq);
}

module_init(pixcir_ts_init);
module_exit(pixcir_ts_exit);

MODULE_DESCRIPTION("Pixcir Touchscreen Driver");
MODULE_LICENSE("GPL");
