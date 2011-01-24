#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/tmg120.h>
#include <mach/gpio.h>

/* TPK Cypress CY8CTMG120 setting. */
#define TMG120_ABS_X_MIN 	0
#define TMG120_ABS_X_MAX 	480
#define TMG120_ABS_Y_MIN 	0
#define TMG120_ABS_Y_MAX 	800
#define TMG120_FUZZ 	0
#define TMG120_FLAT 	0

static struct workqueue_struct *tmg120_wq;

#define INVOKED_BY_IRQ		0
#define INVOKED_BY_TIMER	1

struct tmg120_data {
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
	int reported_finger_count;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

#if defined(CONFIG_MACH_OMAP3621_EVT1A)
static struct regulator* boxer_tp_regulator;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tmg120_early_suspend(struct early_suspend *h);
static void tmg120_late_resume(struct early_suspend *h);
#endif

int  cy8ctmg120_dev_init(int resource);

static void tmg120_work_func(struct work_struct *work)
{
	//define ouput data start
	int previous_state = 0;
        static int z = 10;		// Set a default value for TP IC,this is fake data
	static int w = 1;		// Set a default value for TP IC,this is fake data
 	//end of define data
	int ret = 0;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t buf[13];
	uint8_t buf1[1];
	struct tmg120_data *ts = container_of(work, struct tmg120_data, work);
	struct tmg120_platform_data * pdata = NULL;
	uint8_t gesture_code = 0;

	if (ts != NULL) {
		pdata = ts->client->dev.platform_data;
	}

	//tmg120_dump("tmg120: Interrupt Handler -- tmg120_work_func working by %d\n", pdata->invoked_by);

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
#if 0
		tmg120_dump("TMG120_work_func: %s %02x %02x %02x,%02x %02x, %02x" 
		        " %02x, %02x %02x,%02x %02x,%02x, %02x, ret %d\n", 
			(pdata->invoked_by ? "IRQ" : "TIM"),
		        buf[0], buf[1], buf[2], buf[3], 
		        buf[4], buf[5], buf[6], buf[7], 
		        buf[8], buf[9], buf[10], buf[11], buf[12], ret);
#endif	
		
        // RAW XY coordinates
		//
		//    (0,0)    (480,0)
		//    c +----------+
		//    o |          |
		//    n |          |
		//    n |          |
		//    e |          |
		//    c |          |
		//    t |          |
		//    o |          |
		//    r +----------+
		//    (0,800)  (480,800)

		int rx1 = (uint16_t)buf[ 4] | (uint16_t)buf[ 3] << 8;
		int ry1 = (uint16_t)buf[ 6] | (uint16_t)buf[ 5] << 8;
		int rx2 = (uint16_t)buf[ 8] | (uint16_t)buf[ 7] << 8;
        int ry2 = (uint16_t)buf[10] | (uint16_t)buf[ 9] << 8;
        int finger_number = buf[11];
        int gesture_code = buf[12];

        // Transform to Android co-ordinates.
		// Note this is for EVT1A!!
		// int x1 = rx1;
		// int y1 = ry1;
		// int x2 = rx2;
		// int y2 = ry2;

        // Transform to Android co-ordinates.
		// Note this is for EVT1B!!
		int x1 = TMG120_ABS_X_MAX - rx1;
		int y1 = TMG120_ABS_Y_MAX - ry1;
		int x2 = TMG120_ABS_X_MAX - rx2;
		int y2 = TMG120_ABS_Y_MAX - ry2;

    	tmg120_dump("+#+ %03d,%03d & %03d,%03d -> %03d,%03d & %03d,%03d f=%d g=%d\n",
					rx1, ry1, rx2, ry2, x1, y1, x2, y2, finger_number, gesture_code);

    	int finger1_pressed = finger_number > 0 ? 1 : 0 ;
    	int finger2_pressed = finger_number == 2 ? 1 : 0 ;
		z = finger_number == 0 ? 0 : 20;

		if ( pdata != NULL ) {
			input_report_key(ts->input_dev, BTN_TOUCH, finger1_pressed);
			input_report_abs(ts->input_dev, ABS_X, x1);		//X1 coordinate
			input_report_abs(ts->input_dev, ABS_Y, y1);		//Y1 coordinate
			input_report_abs(ts->input_dev, ABS_PRESSURE, z);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w+1);

			if (finger2_pressed) {
				tmg120_dump("x2 = %4d, y2 = %4d, Previous state %2d\n", x2, y2, previous_state);
				input_report_key(ts->input_dev, BTN_2, finger2_pressed);
				input_report_abs(ts->input_dev, ABS_HAT0X, x2);					//X2 coordinate
				input_report_abs(ts->input_dev, ABS_HAT0Y, y2);		//Y2 coordinate			
			}

			/* For kernel 2.0 multi touch use */
 			if (!finger1_pressed) {
				z = 0;		//Reset midiwave value
				w = 0;		//Reset midiwave value
			}
			//tmg120_dump("x1 = %4d, y1 = %4d, Finger %2d\n", x1, y1, finger);
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
				tmg120_dump("x2 = %4d, y2 = %4d, Previous state %2d\n", x2, y2, previous_state);
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
			ts->reported_finger_count = finger_number;
			/* End of multi touch */

			input_sync(ts->input_dev);  


			if (finger1_pressed == 0) {
				hrtimer_cancel(&ts->timer);	
				tmg120_dump("timer cancelled\n");			
			}	
		}

	}             
done:
       	ret = i2c_smbus_write_byte_data(ts->client, 0x01, 1); /* disable interrupt */
       	if (ret < 0)
               	tmg120_dump(KERN_ERR "tmg120_suspend: i2c_smbus_write_byte_data failed\n");

	return;
}

static enum hrtimer_restart tmg120_timer_func(struct hrtimer *timer)
{
	struct tmg120_data *ts = container_of(timer, struct tmg120_data, timer);
	struct tmg120_platform_data *pdata = NULL;

	//tmg120_dump("tmg120: tmg120_timer_func > ts->use_irq = %d\n", ts->use_irq);
	
	if (ts != NULL) {
		pdata = ts->client->dev.platform_data;

		if (pdata != NULL) 
			pdata->invoked_by = INVOKED_BY_TIMER;
	}

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);			//report rate 1/12500000=80Hz
	queue_work(tmg120_wq, &ts->work);
	return HRTIMER_NORESTART;
}

static irqreturn_t tmg120_irq_handler(int irq, void *dev_id)
{
	struct tmg120_platform_data *pdata = NULL;
	struct tmg120_data *ts = dev_id;

	// tmg120_dump("TMG120: TMG120_irq_handler\n"); 

	pdata = ts->client->dev.platform_data;
	if (pdata) {
		pdata->invoked_by = INVOKED_BY_IRQ;

		if ( (pdata->irqflags & IRQF_TRIGGER_FALLING) != 0) {
			//tmg120_dump("tmg120_irq_handler: IRQ_TRIGGER_FALLING, calling hrtimer_start()\n");
			queue_work(tmg120_wq, &ts->work);
			hrtimer_start(&ts->timer, ktime_set(0, 100000), HRTIMER_MODE_REL);
		}
	}

	return IRQ_HANDLED;
}


static int tmg120_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmg120_data *ts;
	uint8_t buf0[2];
	struct i2c_msg msg[1];
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct tmg120_platform_data *pdata;
	unsigned long irqflags;
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
    
	#define OMAP_CY8CTMG120_RESET 46

	printk("Reseting TMG120\n");
	// request gpio resources
	if (cy8ctmg120_dev_init(1) <0) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	mdelay(20);
	gpio_direction_output(OMAP_CY8CTMG120_RESET, 1);
	mdelay(10);
	gpio_direction_output(OMAP_CY8CTMG120_RESET, 0);
	mdelay(100);//TODO: Delay is needed for chip to reset but may not need 100.

	//TODO: temp. needed to determine TP type (tma340 or tmg120)
	//TODO: To be removed once tmg120 is not supported.
	{
		unsigned char buf;
		int rc;
		client->addr = 32;
		buf=15;
		rc = i2c_master_send(client, &buf, 1);
		rc = i2c_master_recv(client, &buf, 1);
		if (rc < 0)
		{
			printk("No TMG120 found; exiting probe.\n");
			ret = -ENODEV;

			// free resources
			cy8ctmg120_dev_init(0);
            regulator_disable(boxer_tp_regulator);
			goto err_check_functionality_failed;
		}
	}
	
	tmg120_dump("tmg120: tmg120_probe is running!!\n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		tmg120_dump(KERN_ERR "tmg120_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, tmg120_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			tmg120_dump(KERN_ERR "tmg120_probe power on failed\n");
			goto err_power_failed;
		}
	}

	tmg120_dump("tmg120: 1 \n");

	if (pdata) {
		ts->flags = pdata->flags;
		irqflags = pdata->irqflags;
		inactive_area_left = pdata->inactive_left;
		inactive_area_right = pdata->inactive_right;
		inactive_area_top = pdata->inactive_top;
		inactive_area_bottom = pdata->inactive_bottom;
		snap_left_on = pdata->snap_left_on;
		snap_left_off = pdata->snap_left_off;
		snap_right_on = pdata->snap_right_on;
		snap_right_off = pdata->snap_right_off;
		snap_top_on = pdata->snap_top_on;
		snap_top_off = pdata->snap_top_off;
		snap_bottom_on = pdata->snap_bottom_on;
		snap_bottom_off = pdata->snap_bottom_off;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		irqflags = IRQF_TRIGGER_FALLING;
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
	}

	tmg120_dump("tmg120: 2 \n");

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf0;
	buf0[0] = 0x37;		
	buf0[1] = 0x03;
        ret = i2c_transfer(ts->client->adapter, msg, 1);


	ts->max[0] = max_x = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
	ts->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
	if (ts->flags & TMG120_SWAP_XY)
		swap(max_x, max_y);

	tmg120_dump("tmg120: 3 \n");
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		tmg120_dump(KERN_ERR "tmg120_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "Cypress-TMG120";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	tmg120_dump("tmg120_probe: ---------------------------------------------------setting ready\n");

	tmg120_dump("==>: max_x %4d, max_y %4d\n", TMG120_ABS_X_MAX, TMG120_ABS_Y_MAX);

	tmg120_dump("tmg120: 4 \n");
	input_set_abs_params(ts->input_dev, ABS_X, TMG120_ABS_X_MIN, TMG120_ABS_X_MAX, TMG120_FUZZ, TMG120_FLAT);
	input_set_abs_params(ts->input_dev, ABS_Y, TMG120_ABS_Y_MIN, TMG120_ABS_Y_MAX, TMG120_FUZZ, TMG120_FLAT);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, fuzz_p, 0);

	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, TMG120_ABS_X_MIN, TMG120_ABS_X_MAX, TMG120_FUZZ, TMG120_FLAT);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, TMG120_ABS_Y_MIN, TMG120_ABS_Y_MAX, TMG120_FUZZ, TMG120_FLAT);

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 3, fuzz_w, 0);

	tmg120_dump("tmg120: 5 \n");
	ret = input_register_device(ts->input_dev);
	if (ret) {
		tmg120_dump(KERN_ERR "==>: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	tmg120_dump("tmg120: 6 \n");

	if (client->irq) {
		ret = request_irq(client->irq, tmg120_irq_handler, irqflags, client->name, ts);

	tmg120_dump("tmg120: 7 \n");
                if (ret == 0) {
                        ts->use_irq = 1;

			/* Init timer */
			hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			ts->timer.function = tmg120_timer_func;
		}
                else {
			ts->use_irq = 0;
			free_irq(client->irq, ts);
			dev_err(&client->dev, "request_irq failed with value %d\n", ret);
		}
	}
	tmg120_dump("tmg120: 8 \n");
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = tmg120_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	tmg120_dump("tmg120: 9 \n");
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = tmg120_early_suspend;
	ts->early_suspend.resume = tmg120_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	tmg120_dump("==>:Touchscreen Up %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	tmg120_dump("tmg120: 10 \n");
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

static int tmg120_remove(struct i2c_client *client)
{
	struct tmg120_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int tmg120_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct tmg120_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0); /* disable interrupt */
	if (ret < 0)
		tmg120_dump(KERN_ERR "tmg120_suspend: i2c_smbus_write_byte_data failed\n");

	ret = i2c_smbus_write_byte_data(client, 0xf0, 0x86); /* deep sleep */
	if (ret < 0)
		tmg120_dump(KERN_ERR "tmg120_suspend: i2c_smbus_write_byte_data failed\n");

#if defined(CONFIG_MACH_OMAP3621_EVT1A)
    regulator_disable(boxer_tp_regulator);
#endif

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			tmg120_dump(KERN_ERR "tmg120_resume power off failed\n");
	}
	return 0;
}

static int tmg120_resume(struct i2c_client *client)
{
	int ret;
	struct tmg120_data *ts = i2c_get_clientdata(client);

#if defined(CONFIG_MACH_OMAP3621_EVT1A)
    ret = regulator_enable(boxer_tp_regulator);
    if (ret) {
        printk(KERN_ERR "Failed to enable regulator vtp on resume\n");
        return ret;
    }
#endif

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			tmg120_dump(KERN_ERR "tmg120_resume power on failed\n");
	}

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else
		i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tmg120_early_suspend(struct early_suspend *h)
{
	struct tmg120_data *ts;
	ts = container_of(h, struct tmg120_data, early_suspend);
	tmg120_suspend(ts->client, PMSG_SUSPEND);
}

static void tmg120_late_resume(struct early_suspend *h)
{
	struct tmg120_data *ts;
	ts = container_of(h, struct tmg120_data, early_suspend);
	tmg120_resume(ts->client);
}
#endif

static const struct i2c_device_id tmg120_id[] = {
	{ CYPRESS_CY8CTMG120_NAME, 0 },
	{ }
};

static struct i2c_driver tmg120_driver = {
	.probe		= tmg120_probe,
	.remove		= tmg120_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= tmg120_suspend,
	.resume		= tmg120_resume,
#endif
	.id_table	= tmg120_id,
	.driver = {
		.name	= CYPRESS_CY8CTMG120_NAME,
	},
};

static int __devinit cypress_tmg120_init(void)
{
    int ret = 0;

#if defined(CONFIG_MACH_OMAP3621_EVT1A)
    printk("Enabling power for touch panel\n");
    boxer_tp_regulator = regulator_get(NULL, "vtp");
    if (IS_ERR(boxer_tp_regulator)) {
        printk(KERN_ERR "Unable to get vtp regulator, reason: %ld!\n", IS_ERR(boxer_tp_regulator));
        ret = -ENODEV;
        goto out;
    }

    ret = regulator_enable(boxer_tp_regulator);
    if (ret) {
        printk(KERN_ERR "Failed to enable regulator vtp!\n");
        regulator_put(boxer_tp_regulator);
        goto out;
    }
#endif

	tmg120_dump("tmg120: cypress_tmg120_init is running!!\n");
	tmg120_wq = create_singlethread_workqueue("tmg120_wq");
	
	if (!tmg120_wq)
	{
		return -ENOMEM;
	}
        
    ret = i2c_add_driver(&tmg120_driver);

out:
	return ret;
}

static void __exit cypress_tmg120_exit(void)
{
#if defined(CONFIG_MACH_OMAP3621_EVT1A)
    regulator_disable(boxer_tp_regulator);
    regulator_put(boxer_tp_regulator);
#endif
    
	i2c_del_driver(&tmg120_driver);
	if (tmg120_wq)
		destroy_workqueue(tmg120_wq);
}

module_init(cypress_tmg120_init);
module_exit(cypress_tmg120_exit);

MODULE_DESCRIPTION("Cypress Touchscreen Driver");
MODULE_LICENSE("GPL");
