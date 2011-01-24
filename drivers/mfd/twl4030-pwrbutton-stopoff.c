/**
 * twl4030-btn-stopoff.c - TWL4030 Power Button force offr
 *
 * Copyright (C) 2010 Barns and Noble
 *
 * Written by Bennett Chanr <bchan@book.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl4030.h>

		
#define DEVICENAME		"twl4030-btn-stopoff"


#define PWR_P1_SW_EVENTS	0x10
#define PWR_P2_SW_EVENTS	0x11
#define PWR_P3_SW_EVENTS	0x12
#define STOPON_PWRON (1<<6)
#define R_PROTECT_KEY		0x0E
/* tps65921 for edp1 */
#if defined(CONFIG_MACH_OMAP3621_EDP1) || defined(CONFIG_MACH_OMAP3621_EVT1A) || defined(CONFIG_MACH_OMAP3621_BOXER)
#define TWL4030_LOCK_KEY_1			0xFC
#define TWL4030_LOCK_KEY_2			0x96
#else
#define TWL4030_LOCK_KEY_1			0xC0
#define TWL4030_LOCK_KEY_2			0x0C
#endif /* CONFIG_MACH_OMAP3621_EDP1 */


static int twl4030_button_stop_off_enable(void)
{
	int err = 0;
	u8 uninitialized_var(rd_data);

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, TWL4030_LOCK_KEY_1,
			R_PROTECT_KEY);
	if (err) {
		pr_warning("twl4030: %s unable to unlock PROTECT_KEY 1, %x\n",__FUNCTION__,TWL4030_LOCK_KEY_1);
		return err;
	}

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, TWL4030_LOCK_KEY_2,
			R_PROTECT_KEY);
	if (err) {
		pr_warning("twl4030:  %s unable to unlock PROTECT_KEY 2, %x\n",__FUNCTION__,TWL4030_LOCK_KEY_2);
		return err;
	}

	err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data, PWR_P1_SW_EVENTS);
	rd_data |= STOPON_PWRON;
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data, PWR_P1_SW_EVENTS);
#if 0
	err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data, PWR_P2_SW_EVENTS);
	rd_data |= STOPON_PWRON;
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data, PWR_P2_SW_EVENTS);
	err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data, PWR_P3_SW_EVENTS);
	rd_data |= STOPON_PWRON;
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data, PWR_P3_SW_EVENTS);
#endif
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, R_PROTECT_KEY);


	if (err)
		printk(KERN_ERR "TWL4030 button off config error\n");

	return err;
}

static int twl4030_button_stop_off_disable(void)
{
	int err = 0;
	u8 uninitialized_var(rd_data);

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, TWL4030_LOCK_KEY_1,
			R_PROTECT_KEY);
	if (err) {
		pr_warning("twl4030: button off unable to unlock PROTECT_KEY 1, %x\n",TWL4030_LOCK_KEY_1);
		return err;
	}

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, TWL4030_LOCK_KEY_2,
			R_PROTECT_KEY);
	if (err) {
		pr_warning("twl4030:  button off unable to unlock PROTECT_KEY 2, %x\n",TWL4030_LOCK_KEY_2);
		return err;
	}

	err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data, PWR_P1_SW_EVENTS);
	rd_data &= ~STOPON_PWRON;
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data, PWR_P1_SW_EVENTS);

	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, R_PROTECT_KEY);

	if (err)
		printk(KERN_ERR "TWL4030 button off config error\n");
	return err;
}


static ssize_t button_stopoff_state_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	int err;
	u8 uninitialized_var(rd_data);
	err = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data, PWR_P1_SW_EVENTS);
	rd_data &= STOPON_PWRON;
	return sprintf(buf, "%s\n", (rd_data?"enabled":"disabled"));
}

static ssize_t button_stopoff_state_set(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	if (strncmp(buf,"enable",6)==0 ) {
		twl4030_button_stop_off_enable();
	} else if (strncmp(buf,"disable",7)==0 ) {
		twl4030_button_stop_off_disable();
	} else {
		printk(KERN_WARNING "button state set only takes enable or disable\n");
	}

	return count;
}


static DEVICE_ATTR(button_stopoff, S_IRUGO|S_IWUGO, button_stopoff_state_show, button_stopoff_state_set);


static int __devinit twl4030_pwrbutton_stopoff_probe(struct platform_device *pdev)
{
	int err;

	err = device_create_file(&pdev->dev, &dev_attr_button_stopoff);
	if (unlikely(err)) {
		dev_err(&pdev->dev, "Failed creating device attrs\n");
		err = -EINVAL;
	}
	else {
		err=twl4030_button_stop_off_enable();
	}

	return err;
}

static int __devexit twl4030_pwrbutton_stopoff_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_button_stopoff);
	return 0;
}


struct platform_driver twl4030_pwrbutton_stopoff_driver = {
//	.probe		= twl4030_pwrbutton_stopoff_probe,
	.remove		= __devexit_p(twl4030_pwrbutton_stopoff_remove),
	.driver		= {
		.name	= "twl4030_pwrbutton_stopoff",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device twl4030_pwrbutton_stopoff_device = {
	.name = DEVICENAME,
};


static int __init twl4030_pwrbutton_stopoff_init(void)
{
	int ret;

	ret=platform_driver_register(&twl4030_pwrbutton_stopoff_driver);

	if ((ret = platform_device_register(&twl4030_pwrbutton_stopoff_device)) != 0) {
		pr_debug( "Unable to register platform device twl4030_pwrbutton_stopoff_device\n");
		return -ENODEV;
	} else
	{
		twl4030_pwrbutton_stopoff_probe(&twl4030_pwrbutton_stopoff_device);
	}
	return ret;
}
module_init(twl4030_pwrbutton_stopoff_init);

static void __exit twl4030_pwrbutton_stopoff_exit(void)
{
	platform_device_unregister(&twl4030_pwrbutton_stopoff_device);
	platform_driver_unregister(&twl4030_pwrbutton_stopoff_driver);

}
module_exit(twl4030_pwrbutton_stopoff_exit);

MODULE_ALIAS("platform:twl4030_pwrbutton_stopoff");
MODULE_DESCRIPTION("Triton2 Power Button stopoff");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bennett Chan<bchan@book.com>");

