/*
 * Maxim 9635 light sensor driver
 *
 * Copyright (c) 2010 Barnes & Noble. All rights reserved.
 * Written by David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <asm/atomic.h>

#include <linux/max9635.h>

/* Register addresses */
#define MAX9635_INTEN_REG           0x01
#define MAX9635_HIGHLUX_REG         0x03
#define MAX9635_LOWLUX_REG          0x04
#define MAX9635_UPPER_THRESHOLD_REG 0x05
#define MAX9635_LOWER_THRESHOLD_REG 0x06

/* Register values */
#define MAX9635_ENABLE_INT          0x01

/* Constants */
#define MAX9635_MIN_THRESHOLD       0x0
#define MAX9635_MAX_THRESHOLD       0xef

struct max9635_data {
    struct i2c_client *client;
    int gpio;
    struct work_struct irq_work;
    struct delayed_work timer_work;
    int poll_interval;
    int polling;
    atomic_t lux;
    int *thresholds;
    int thresholds_count;
    struct mutex thresholds_lock;
	int (*device_resource)(int); 
};

static int max9635_i2c_write(struct max9635_data *max9635, u8 reg, u8 byte)
{
    int err;
    u8 buf[2];

    struct i2c_msg msg = {
        .addr   = max9635->client->addr,
        .flags  = 0,
        .len    = 2,
        .buf    = buf,
    };

    buf[0] = reg;
    buf[1] = byte;

    err = i2c_transfer(max9635->client->adapter, &msg, 1);

    if (err != 1) {
        dev_err(&max9635->client->dev, "write xfer error\n");
    } else {
        err = 0;
    }

    return err;
}

static int max9635_i2c_read(struct max9635_data *max9635, u8 reg, u8 *byte)
{
    int err;

    struct i2c_msg msgs[] = {
        {
            .addr   = max9635->client->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &reg,
        },
        {
            .addr   = max9635->client->addr,
            .flags  = I2C_M_RD,
            .len    = 1,
            .buf    = byte,
        }
    };

    err = i2c_transfer(max9635->client->adapter, msgs, ARRAY_SIZE(msgs));

    if (err != ARRAY_SIZE(msgs)) {
        dev_err(&max9635->client->dev, "read xfer error\n");
    } else {
        err = 0;
    }

    return err;
}

static int max9635_irq_enable(struct max9635_data *max9635)
{
    return max9635_i2c_write(max9635, MAX9635_INTEN_REG, MAX9635_ENABLE_INT);
}

static int max9635_irq_disable(struct max9635_data *max9635)
{
    return max9635_i2c_write(max9635, MAX9635_INTEN_REG, 0x0);
}

static u8 max9635_next_lower_threshold(struct max9635_data *max9635)
{
    int i;
    int lux = atomic_read(&max9635->lux);   
    dev_info(&max9635->client->dev, "lower lux for %d\n", lux);
    
    for (i = (max9635->thresholds_count - 1); i >= 0; --i) {
        if (lux > max9635->thresholds[i]) {
            return max9635->thresholds[i];
        }
    }

    return MAX9635_MIN_THRESHOLD;
}

static u8 max9635_next_upper_threshold(struct max9635_data *max9635)
{
    int i;
    int lux = atomic_read(&max9635->lux);
    dev_info(&max9635->client->dev, "upper lux for %d\n", lux);

    for (i = 0; i < max9635->thresholds_count; ++i) {
        if (lux < max9635->thresholds[i]) {
            return max9635->thresholds[i];
        }
    }

    return MAX9635_MAX_THRESHOLD;
}

static int max9635_write_thresholds(struct max9635_data *max9635)
{
    int err;
    u8 upper;
    u8 lower;
   
    mutex_lock(&max9635->thresholds_lock);
    upper = max9635_next_upper_threshold(max9635);
    lower = max9635_next_lower_threshold(max9635);
    mutex_unlock(&max9635->thresholds_lock);
        
    err = max9635_i2c_write(max9635, MAX9635_UPPER_THRESHOLD_REG, upper);

    if (err) {
        dev_err(&max9635->client->dev, "error setting upper threshold\n");
        return err;
    }

    err = max9635_i2c_write(max9635, MAX9635_LOWER_THRESHOLD_REG, lower);

    if (err) {
        dev_err(&max9635->client->dev, "error setting lower threshold\n");
        return err;
    }

    return 0;
}

static int max9635_read_lux(struct max9635_data *max9635)
{
    int err;
    u8 high_lux;
    u8 low_lux;
    u16 exponent;
    u8 mantissa;

    err = max9635_i2c_read(max9635, MAX9635_HIGHLUX_REG, &high_lux);

    if (err) {
        dev_err(&max9635->client->dev, "error getting high lux byte\n");
        return err;
    }

    err = max9635_i2c_read(max9635, MAX9635_LOWLUX_REG, &low_lux);

    if (err) {
        dev_err(&max9635->client->dev, "error getting low lux byte\n");
        return err;
    }

    /* Following formula from Maxim datasheet 
        Lux = 2^(Exponent) x Mantissa / 20
    */

    exponent = 1 << ((high_lux & 0xf0) >> 4);
    mantissa = ((high_lux & 0xf) << 4) + (low_lux & 0xf);

    atomic_set(&max9635->lux, exponent * mantissa / 20);
    return err;
}

static void max9635_timer_work_func(struct work_struct *work)
{
    struct delayed_work *twork = container_of(work, struct delayed_work, work);
    struct max9635_data *max9635 = container_of(twork, struct max9635_data, timer_work);

    max9635_read_lux(max9635);
    schedule_delayed_work(&max9635->timer_work, max9635->poll_interval);
}

static void max9635_irq_work_func(struct work_struct *work)
{
    struct max9635_data *max9635 = container_of(work, struct max9635_data, irq_work);

    max9635_read_lux(max9635);
    max9635_write_thresholds(max9635);

    enable_irq(max9635->client->irq);    
}

static irqreturn_t max9635_isr(int irq, void *dev)
{
    struct max9635_data *max9635 = dev;
   
    disable_irq_nosync(irq);
    schedule_work(&max9635->irq_work);

    return IRQ_HANDLED;
}

static ssize_t max9635_poll_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", max9635->polling);
}

static ssize_t max9635_poll_store(struct device *dev, struct device_attribute *attr,
                                 const char *buf, size_t count)
{
    int err;
    unsigned int enable;
    struct i2c_client *client = to_i2c_client(dev);
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    enable = simple_strtoul(buf, NULL, 0);

    if (enable && !max9635->polling) {
        err = max9635_irq_disable(max9635);

        if (err) {
            dev_err(dev, "error disabling interrupt\n");
            return err;
        }

        max9635->polling = 1;
        schedule_delayed_work(&max9635->timer_work, 0);
    } else if (!enable && max9635->polling && max9635->gpio) {
        cancel_delayed_work_sync(&max9635->timer_work);

        err = max9635_irq_enable(max9635);

        if (err) {
            dev_err(dev, "error enabling interrupt, resuming polling\n");
            schedule_delayed_work(&max9635->timer_work, 0);
            return err;
        }

        max9635->polling = 0;
    }

    return count;
}

static ssize_t max9635_lux_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", atomic_read(&max9635->lux));
}

static ssize_t max9635_lux_store(struct device *dev, struct device_attribute *attr, 
                                const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t max9635_thresholds_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int i, c;
    ssize_t count = PAGE_SIZE - 2; // Reserve space for '\0' and '\n'
    char *buf_ptr = buf;

    struct i2c_client *client = to_i2c_client(dev);
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    mutex_lock(&max9635->thresholds_lock);

    for (i = 0; i < max9635->thresholds_count && count; ++i) {
        c = snprintf(buf_ptr, count, "%d ", max9635->thresholds[i]);
        buf_ptr += c;
        count -= c;
    }

    mutex_unlock(&max9635->thresholds_lock);
    sprintf(buf_ptr, "\n");

    return buf_ptr - buf;
}

static ssize_t max9635_thresholds_store(struct device *dev, struct device_attribute *attr, 
                                        const char *buf, size_t count)
{
    int argc, err; 
    char **args;
    int *ptr;

    struct i2c_client *client = to_i2c_client(dev);
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    args = argv_split(GFP_KERNEL, buf, &argc);

    if (args == NULL) {
        dev_err(dev, "error getting arguments\n");
        err = -ENOMEM;
        goto out0;
    }

    if (argc) {
        mutex_lock(&max9635->thresholds_lock);
        
        kfree(max9635->thresholds);
        max9635->thresholds_count = 0;
        max9635->thresholds = kzalloc(sizeof(int) * argc, GFP_KERNEL);

        if (max9635->thresholds == NULL) {
            dev_err(dev, "failed allocating new threshold\n");
            err = -ENOMEM;
            goto out1;
        }

        max9635->thresholds_count = argc;
        ptr = max9635->thresholds;

        while (argc--) {
            *ptr = simple_strtoul(*args, NULL, 0);
            ++args;
            ++ptr;
        }

        mutex_unlock(&max9635->thresholds_lock);
        max9635_write_thresholds(max9635);
    }

    return count;

out1:
    mutex_unlock(&max9635->thresholds_lock);
out0:
    return err;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, max9635_poll_show, max9635_poll_store);
static DEVICE_ATTR(lux, S_IRUGO, max9635_lux_show, max9635_lux_store); 
static DEVICE_ATTR(thresholds, S_IRUGO|S_IWUSR, max9635_thresholds_show, max9635_thresholds_store);

static struct attribute *max9635_attributes[] = {
    &dev_attr_poll,
    &dev_attr_lux,
    &dev_attr_thresholds,
    NULL
};

static struct attribute_group max9635_attribute_group = {
    .attrs = max9635_attributes,
};

static int __devinit max9635_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err;
    struct max9635_data *max9635 = NULL;
	unsigned char buf;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL\n");
		return -ENODEV;
	}


	max9635 = kzalloc(sizeof(*max9635), GFP_KERNEL);

    if (max9635 == NULL) {
        dev_err(&client->dev, "kzalloc failed\n");
        err = -ENOMEM;
        goto out0;
    }

    max9635->client = client;
    i2c_set_clientdata(client, max9635);


	max9635->device_resource = MAX9635_PDATA(client->dev.platform_data)->device_resource;
	if (max9635->device_resource) {
		if ((*max9635->device_resource)(1)<0)
		{
			max9635->device_resource = NULL;
			dev_err(&client->dev, "Cannot allocate resource for max9635\n");
			err = -ENODEV;
			goto out1;
		}
	}

	printk("----------max9635 i2c address scan %d\n",client->addr );
	buf=00;
	i2c_master_send(client, &buf, 1);
	err = i2c_master_recv(client, &buf, 1);
	if (err < 0){
		printk("max9635 not found; exiting probe.\n");
		err = -ENODEV;		
		goto out1;
	}

    mutex_init(&max9635->thresholds_lock);
    INIT_DELAYED_WORK(&max9635->timer_work, max9635_timer_work_func);
    max9635->poll_interval = MAX9635_PDATA(client->dev.platform_data)->poll_interval;
    max9635->gpio = MAX9635_PDATA(client->dev.platform_data)->gpio;

    err = sysfs_create_group(&client->dev.kobj, &max9635_attribute_group);

    if (err) {
        dev_err(&client->dev, "error creating sysfs entries\n");
        goto out1;
    }

    if (max9635->gpio) {
        dev_info(&client->dev, "operating in interrupt mode: gpio %d\n", max9635->gpio);

        INIT_WORK(&max9635->irq_work, max9635_irq_work_func);
        err = request_irq(max9635->client->irq, max9635_isr, IRQF_TRIGGER_FALLING, "max9635-irq", max9635);

        if (err < 0) {
            dev_err(&client->dev, "failed to get irq: %d\n", max9635->client->irq);
            goto out2;
        }

        err = max9635_irq_enable(max9635);

        if (err) {
            dev_err(&client->dev, "failed to enable interrupt\n");
            goto out3; 
        }
    } else {
        dev_info(&client->dev, "operating in polled mode: polling interval %d ms\n", max9635->poll_interval);
        max9635->polling = 1;
        schedule_delayed_work(&max9635->timer_work, max9635->poll_interval); 
    }

    return err;
out3:
    free_irq(max9635->client->irq, max9635);
out2:
	sysfs_remove_group(&client->dev.kobj, &max9635_attribute_group);
out1:
	if (max9635->device_resource) {
		(*max9635->device_resource)(0);
	}
    kfree(max9635);
out0:
    return err;
}

static int __devexit max9635_remove(struct i2c_client *client)
{
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    if (max9635_irq_disable(max9635)) {
        dev_err(&client->dev, "err disabling irq\n");
    }
    sysfs_remove_group(&client->dev.kobj, &max9635_attribute_group);
    
    cancel_work_sync(&max9635->irq_work);
    cancel_delayed_work_sync(&max9635->timer_work);

    free_irq(client->irq, max9635);
    gpio_free(max9635->gpio);
	if (max9635->device_resource) {
		(*max9635->device_resource)(0);
	}
    kfree(max9635);

    return 0;
}

static int max9635_suspend(struct i2c_client *client, pm_message_t mesg)
{ 
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    if (!max9635->polling) {
        return max9635_irq_disable(max9635);
    } else {
        return 0;
    }
}

static int max9635_resume(struct i2c_client *client)
{
    struct max9635_data *max9635 = i2c_get_clientdata(client);

    if (!max9635->polling) {
        return max9635_irq_enable(max9635);
    } else {
        return 0;
    }
}

static const struct i2c_device_id max9635_id[] = {
    { MAX9635_NAME, 0 },
    {},
};

MODULE_DEVICE_TABLE(i2c, max9635_id);

static struct i2c_driver max9635_driver = {
    .driver     = {
        .name = MAX9635_NAME,
    },
    .probe      = max9635_probe,
    .remove     = __devexit_p(max9635_remove),
    .resume     = max9635_resume,
    .suspend    = max9635_suspend,
    .id_table   = max9635_id,
};

static int __init max9635_init(void)
{
    return i2c_add_driver(&max9635_driver);
}

static void __exit max9635_exit(void)
{
    i2c_del_driver(&max9635_driver);
}

MODULE_AUTHOR("David Bolcsfoldi <dbolcsfoldi@intrinsyc.com>");
MODULE_DESCRIPTION(MAX9635_NAME " driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MAX9635_VERSION);

module_init(max9635_init)
module_exit(max9635_exit)
