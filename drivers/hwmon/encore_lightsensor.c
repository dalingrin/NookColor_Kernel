/*
 * encore_lightsensor.c - Linux kernel module for Encore Light Sensor monitoring
 *
 * Copyright (C) 2010 Barnes and Noble Corporation
 *
 * Written by Dave Gallatin / Semiphore Systems, LLC.
 *
 * Inspired by omap34xx_temp.c
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/omap34xx.h>
#include <mach/control.h>
#include <linux/i2c/twl4030.h>
#include <linux/i2c/twl4030-madc.h>

#define MAX_HW_UPDATE_RATE	(HZ/5)

#ifdef CONFIG_MACH_OMAP3621_BOXER
#define GPIO_ALS_ENABLE		36
#define GPIO_ALS_GC2		38
#define ENCORE_ALS_SETTLE_TIME	1	//1ms
#else
#define GPIO_ALS_ENABLE		62
#define ENCORE_ALS_SETTLE_TIME	15	//15ms
#endif

struct encore_als_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	const char *name;
	char valid;
	unsigned long last_updated;
	u32 temp;
};

static struct platform_device encore_als_device = {
	.name 	= "encore_als",
	.id	= -1,
};

static void encore_als_update(struct encore_als_data *data)
{
	struct twl4030_madc_request req;

	mutex_lock(&data->update_lock);

	if (!data->valid
	    || time_after(jiffies, data->last_updated + MAX_HW_UPDATE_RATE)) {


		// enable sensor and wait until analog input settles	        
		gpio_set_value(GPIO_ALS_ENABLE, 1);
		mdelay(ENCORE_ALS_SETTLE_TIME);

		// light sensor analog output is connected to MADC 0
		req.channels = (1 << 0);
		req.do_avg = 0;
		req.method = TWL4030_MADC_SW1;
		req.active = 0;
		req.func_cb = NULL;
		twl4030_madc_conversion(&req);

		data->temp = (u16)req.rbuf[0];
		data->last_updated = jiffies;
		data->valid = 1;

		// disable sensor 
		gpio_set_value(GPIO_ALS_ENABLE, 0);
	}

	mutex_unlock(&data->update_lock);
}

static ssize_t show_name(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct encore_als_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", data->name);
}

static ssize_t show_raw(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
	struct encore_als_data *data = dev_get_drvdata(dev);

	encore_als_update(data);

	return sprintf(buf, "%d\n", data->temp);
}

static SENSOR_DEVICE_ATTR_2(als_input_raw, S_IRUGO, show_raw,
				NULL, 0, 0);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static int __devinit encore_als_probe(void)
{
	int err;
	struct encore_als_data *data;

	err = platform_device_register(&encore_als_device);
	if (err) {
		printk(KERN_ERR
			"Unable to register encore als device\n");
		goto exit;
	}

	data = kzalloc(sizeof(struct encore_als_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit_platform;
	}

	dev_set_drvdata(&encore_als_device.dev, data);
	mutex_init(&data->update_lock);
	data->name = "encore_als";

	err = device_create_file(&encore_als_device.dev,
				 &sensor_dev_attr_als_input_raw.dev_attr);
	if (err)
		goto exit_free;

	err = device_create_file(&encore_als_device.dev, &dev_attr_name);
	if (err)
		goto exit_remove_raw;

	data->hwmon_dev = hwmon_device_register(&encore_als_device.dev);

	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_all;
	}

        err = gpio_request(GPIO_ALS_ENABLE, data->name);
	if (err)
		goto exit_remove_all;

        err = gpio_direction_output(GPIO_ALS_ENABLE, 0);
	if (err)
		goto exit_free_enable_gpio;

#ifdef ENCORE_NFF
	// set GC2 low and use GC1 to for equivalent of enabling power
        err = gpio_request(GPIO_ALS_GC2, data->name);
	if (err)
		goto exit_free_enable_gpio;
        err = gpio_direction_output(GPIO_ALS_GC2, 0);
	if (err)
		goto exit_free_gc2_gpio;

	return 0;

exit_free_gc2_gpio:
	gpio_free(GPIO_ALS_GC2);

#else
	return 0;
#endif

exit_free_enable_gpio:
	gpio_free(GPIO_ALS_ENABLE);
	
exit_remove_all:
	device_remove_file(&encore_als_device.dev,
			   &dev_attr_name);
exit_remove_raw:
	device_remove_file(&encore_als_device.dev,
			   &sensor_dev_attr_als_input_raw.dev_attr);
exit_free:
	kfree(data);
exit_platform:
	platform_device_unregister(&encore_als_device);
exit:
	return err;
}

static int __init encore_als_init(void)
{
	return encore_als_probe();
}

static void __exit encore_als_exit(void)
{
	struct encore_als_data *data =
			dev_get_drvdata(&encore_als_device.dev);

	gpio_free(GPIO_ALS_ENABLE);

#ifdef ENCORE_NFF
	gpio_free(GPIO_ALS_GC2);
#endif

	hwmon_device_unregister(data->hwmon_dev);
	device_remove_file(&encore_als_device.dev,
			   &sensor_dev_attr_als_input_raw.dev_attr);
	device_remove_file(&encore_als_device.dev, &dev_attr_name);
	kfree(data);
	platform_device_unregister(&encore_als_device);
}

MODULE_AUTHOR("Dave Gallatin");
MODULE_DESCRIPTION("Encore ambient light sensor");
MODULE_LICENSE("GPL");

module_init(encore_als_init)
module_exit(encore_als_exit)

