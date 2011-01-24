/*
 * isp1301_host - ISP 1301 USB transceiver, talking to OMAP OHCI controller
 *
 * Copyright (C) 2004 Texas Instruments
 * Copyright (C) 2004 David Brownell
 * Copyright (C) 2008 Anand Gadiyar
 *
 * Based on isp1301_omap.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* The ISP1301 is used with the OHCI controller in the OMAP3430 SDP.
 * The other isp1301 driver does otg_set_transceiver if CONFIG_OTG is
 * defined. For the OMAP3, the MUSB controller is the OTG controller
 * (and therefore CONFIG_OTG would likely be defined) but it very likely
 * would be used with a different transceiver (TWL4030, ISP1504, ...)
 *
 * This driver is a stub driver meant to put the ISP1301 in host mode.
 * It is based on the original isp1301 driver.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>


#define DRIVER_VERSION	"10 February 2009"
#define DRIVER_NAME	(isp1301_driver.driver.name)

MODULE_DESCRIPTION("ISP1301 USB HOST Transceiver Driver");
MODULE_LICENSE("GPL");

struct isp1301 {
	struct i2c_client	*client;
	void			(*i2c_release)(struct device *dev);

	int			irq_type;

	unsigned		working:1;

};



/*-------------------------------------------------------------------------*/

static struct i2c_driver isp1301_driver;

/* smbus apis are used for portability */

static inline u8 isp1301_get_u8(struct isp1301 *isp, u8 reg)
{
	return i2c_smbus_read_byte_data(isp->client, reg + 0);
}

static inline int isp1301_get_u16(struct isp1301 *isp, u8 reg)
{
	return i2c_smbus_read_word_data(isp->client, reg);
}

static inline int isp1301_set_bits(struct isp1301 *isp, u8 reg, u8 bits)
{
	return i2c_smbus_write_byte_data(isp->client, reg + 0, bits);
}

static inline int isp1301_clear_bits(struct isp1301 *isp, u8 reg, u8 bits)
{
	return i2c_smbus_write_byte_data(isp->client, reg + 1, bits);
}

/*-------------------------------------------------------------------------*/

/* identification */
#define	ISP1301_VENDOR_ID		0x00	/* u16 read */
#define	ISP1301_PRODUCT_ID		0x02	/* u16 read */
#define	ISP1301_BCD_DEVICE		0x14	/* u16 read */

#define	I2C_VENDOR_ID_PHILIPS		0x04cc
#define	I2C_PRODUCT_ID_PHILIPS_1301	0x1301

/* operational registers */
#define	ISP1301_MODE_CONTROL_1		0x04	/* u8 read, set, +1 clear */
#	define	MC1_SPEED		(1 << 0)
#	define	MC1_SUSPEND		(1 << 1)
#	define	MC1_DAT_SE0		(1 << 2)
#	define	MC1_TRANSPARENT		(1 << 3)
#	define	MC1_BDIS_ACON_EN	(1 << 4)
#	define	MC1_OE_INT_EN		(1 << 5)
#	define	MC1_UART_EN		(1 << 6)
#	define	MC1_MASK		0x7f
#define	ISP1301_MODE_CONTROL_2		0x12	/* u8 read, set, +1 clear */
#	define	MC2_GLOBAL_PWR_DN	(1 << 0)
#	define	MC2_SPD_SUSP_CTRL	(1 << 1)
#	define	MC2_BI_DI		(1 << 2)
#	define	MC2_TRANSP_BDIR0	(1 << 3)
#	define	MC2_TRANSP_BDIR1	(1 << 4)
#	define	MC2_AUDIO_EN		(1 << 5)
#	define	MC2_PSW_EN		(1 << 6)
#	define	MC2_EN2V7		(1 << 7)
#define	ISP1301_OTG_CONTROL_1		0x06	/* u8 read, set, +1 clear */
#	define	OTG1_DP_PULLUP		(1 << 0)
#	define	OTG1_DM_PULLUP		(1 << 1)
#	define	OTG1_DP_PULLDOWN	(1 << 2)
#	define	OTG1_DM_PULLDOWN	(1 << 3)
#	define	OTG1_ID_PULLDOWN	(1 << 4)
#	define	OTG1_VBUS_DRV		(1 << 5)
#	define	OTG1_VBUS_DISCHRG	(1 << 6)
#	define	OTG1_VBUS_CHRG		(1 << 7)

#define	ISP1301_INTERRUPT_SOURCE	0x08	/* u8 read */
#define	ISP1301_INTERRUPT_LATCH		0x0A	/* u8 read, set, +1 clear */

#define	ISP1301_INTERRUPT_FALLING	0x0C	/* u8 read, set, +1 clear */
#define	ISP1301_INTERRUPT_RISING	0x0E	/* u8 read, set, +1 clear */

/*-------------------------------------------------------------------------*/

static void isp1301_release(struct device *dev)
{
	struct isp1301	*isp;

	isp = dev_get_drvdata(dev);

	/* ugly -- i2c hijacks our memory hook to wait_for_completion() */
	if (isp->i2c_release)
		isp->i2c_release(dev);
	kfree(isp);
}

static int __exit isp1301_remove(struct i2c_client *i2c)
{
	struct isp1301	*isp;

	isp = i2c_get_clientdata(i2c);

	isp1301_clear_bits(isp, ISP1301_INTERRUPT_FALLING, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_RISING, ~0);

	put_device(&i2c->dev);

	return 0;
}

/*-------------------------------------------------------------------------*/

static int __init
isp1301_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int			status;
	struct isp1301		*isp;


	isp = kzalloc(sizeof *isp, GFP_KERNEL);
	if (!isp)
		return 0;

	i2c_set_clientdata(i2c, isp);
	isp->client = i2c;

	/* verify the chip (shouldn't be necesary) */
	status = isp1301_get_u16(isp, ISP1301_VENDOR_ID);
	if (status != I2C_VENDOR_ID_PHILIPS) {
		dev_dbg(&i2c->dev, "not philips id: %d\n", status);
		goto fail;
	}
	status = isp1301_get_u16(isp, ISP1301_PRODUCT_ID);
	if (status != I2C_PRODUCT_ID_PHILIPS_1301) {
		dev_dbg(&i2c->dev, "not isp1301, %d\n", status);
		goto fail;
	}
	isp->i2c_release = i2c->dev.release;
	i2c->dev.release = isp1301_release;

	/* initial development used chiprev 2.00 */
	status = i2c_smbus_read_word_data(i2c, ISP1301_BCD_DEVICE);
	dev_info(&i2c->dev, "chiprev %x.%02x, driver " DRIVER_VERSION "\n",
		status >> 8, status & 0xff);

	/* make like power-on reset */
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1, MC1_MASK);

	isp1301_set_bits(isp, ISP1301_MODE_CONTROL_2, MC2_BI_DI);
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_2, ~MC2_BI_DI);

	isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1,
				OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN);
	isp1301_clear_bits(isp, ISP1301_OTG_CONTROL_1,
				~(OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN));

	isp1301_clear_bits(isp, ISP1301_INTERRUPT_LATCH, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_FALLING, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_RISING, ~0);

	/* ISP1301 is put in host mode by default for OHCI host */

	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1, MC1_SUSPEND);
	isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1, MC1_DAT_SE0);
	isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1, OTG1_VBUS_DRV);
	return 0;
fail:
	kfree(isp);
	return -ENODEV;
}

static const struct i2c_device_id isp1301_id[] = {
	{ "isp1301_host", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isp1301_id);

static struct i2c_driver isp1301_driver = {
	.driver = {
		.name	= "isp1301_host",
	},
	.probe		= isp1301_probe,
	.remove		= __exit_p(isp1301_remove),
	.id_table	= isp1301_id,
};

/*-------------------------------------------------------------------------*/

static int __init isp_init(void)
{
	return i2c_add_driver(&isp1301_driver);
}
module_init(isp_init);

static void __exit isp_exit(void)
{
	i2c_del_driver(&isp1301_driver);
}
module_exit(isp_exit);

