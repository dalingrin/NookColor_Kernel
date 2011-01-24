/*
 * twl4030_usb - TWL4030 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004-2007 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
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
 *
 * Current status:
 *	- HS USB ULPI mode works.
 *	- 3-pin mode support may be added in future.
 */

#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/i2c/twl4030.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/wakelock.h>

/* Register defines */

#define VENDOR_ID_LO			0x00
#define VENDOR_ID_HI			0x01
#define PRODUCT_ID_LO			0x02
#define PRODUCT_ID_HI			0x03

#define FUNC_CTRL			0x04
#define FUNC_CTRL_SET			0x05
#define FUNC_CTRL_CLR			0x06
#define FUNC_CTRL_SUSPENDM		(1 << 6)
#define FUNC_CTRL_RESET			(1 << 5)
#define FUNC_CTRL_OPMODE_MASK		(3 << 3) /* bits 3 and 4 */
#define FUNC_CTRL_OPMODE_NORMAL		(0 << 3)
#define FUNC_CTRL_OPMODE_NONDRIVING	(1 << 3)
#define FUNC_CTRL_OPMODE_DISABLE_BIT_NRZI	(2 << 3)
#define FUNC_CTRL_TERMSELECT		(1 << 2)
#define FUNC_CTRL_XCVRSELECT_MASK	(3 << 0) /* bits 0 and 1 */
#define FUNC_CTRL_XCVRSELECT_HS		(0 << 0)
#define FUNC_CTRL_XCVRSELECT_FS		(1 << 0)
#define FUNC_CTRL_XCVRSELECT_LS		(2 << 0)
#define FUNC_CTRL_XCVRSELECT_FS4LS	(3 << 0)

#define IFC_CTRL			0x07
#define IFC_CTRL_SET			0x08
#define IFC_CTRL_CLR			0x09
#define IFC_CTRL_INTERFACE_PROTECT_DISABLE	(1 << 7)
#define IFC_CTRL_AUTORESUME		(1 << 4)
#define IFC_CTRL_CLOCKSUSPENDM		(1 << 3)
#define IFC_CTRL_CARKITMODE		(1 << 2)
#define IFC_CTRL_FSLSSERIALMODE_3PIN	(1 << 1)

#define TWL4030_OTG_CTRL		0x0A
#define TWL4030_OTG_CTRL_SET		0x0B
#define TWL4030_OTG_CTRL_CLR		0x0C
#define TWL4030_OTG_CTRL_DRVVBUS	(1 << 5)
#define TWL4030_OTG_CTRL_CHRGVBUS	(1 << 4)
#define TWL4030_OTG_CTRL_DISCHRGVBUS	(1 << 3)
#define TWL4030_OTG_CTRL_DMPULLDOWN	(1 << 2)
#define TWL4030_OTG_CTRL_DPPULLDOWN	(1 << 1)
#define TWL4030_OTG_CTRL_IDPULLUP	(1 << 0)

#define USB_INT_EN_RISE			0x0D
#define USB_INT_EN_RISE_SET		0x0E
#define USB_INT_EN_RISE_CLR		0x0F
#define USB_INT_EN_FALL			0x10
#define USB_INT_EN_FALL_SET		0x11
#define USB_INT_EN_FALL_CLR		0x12
#define USB_INT_STS			0x13
#define USB_INT_LATCH			0x14
#define USB_INT_IDGND			(1 << 4)
#define USB_INT_SESSEND			(1 << 3)
#define USB_INT_SESSVALID		(1 << 2)
#define USB_INT_VBUSVALID		(1 << 1)
#define USB_INT_HOSTDISCONNECT		(1 << 0)

#define CARKIT_CTRL			0x19
#define CARKIT_CTRL_SET			0x1A
#define CARKIT_CTRL_CLR			0x1B
#define CARKIT_CTRL_MICEN		(1 << 6)
#define CARKIT_CTRL_SPKRIGHTEN		(1 << 5)
#define CARKIT_CTRL_SPKLEFTEN		(1 << 4)
#define CARKIT_CTRL_RXDEN		(1 << 3)
#define CARKIT_CTRL_TXDEN		(1 << 2)
#define CARKIT_CTRL_IDGNDDRV		(1 << 1)
#define CARKIT_CTRL_CARKITPWR		(1 << 0)
#define CARKIT_PLS_CTRL			0x22
#define CARKIT_PLS_CTRL_SET		0x23
#define CARKIT_PLS_CTRL_CLR		0x24
#define CARKIT_PLS_CTRL_SPKRRIGHT_BIASEN	(1 << 3)
#define CARKIT_PLS_CTRL_SPKRLEFT_BIASEN	(1 << 2)
#define CARKIT_PLS_CTRL_RXPLSEN		(1 << 1)
#define CARKIT_PLS_CTRL_TXPLSEN		(1 << 0)

#define CARKIT_ANA_CTRL			0xBB
#define SEL_MADC_MCPC			(1 << 3)

#define MCPC_CTRL			0x30
#define MCPC_CTRL_SET			0x31
#define MCPC_CTRL_CLR			0x32
#define MCPC_CTRL_RTSOL			(1 << 7)
#define MCPC_CTRL_EXTSWR		(1 << 6)
#define MCPC_CTRL_EXTSWC		(1 << 5)
#define MCPC_CTRL_VOICESW		(1 << 4)
#define MCPC_CTRL_OUT64K		(1 << 3)
#define MCPC_CTRL_RTSCTSSW		(1 << 2)
#define MCPC_CTRL_HS_UART		(1 << 0)

#define MCPC_IO_CTRL			0x33
#define MCPC_IO_CTRL_SET		0x34
#define MCPC_IO_CTRL_CLR		0x35
#define MCPC_IO_CTRL_MICBIASEN		(1 << 5)
#define MCPC_IO_CTRL_CTS_NPU		(1 << 4)
#define MCPC_IO_CTRL_RXD_PU		(1 << 3)
#define MCPC_IO_CTRL_TXDTYP		(1 << 2)
#define MCPC_IO_CTRL_CTSTYP		(1 << 1)
#define MCPC_IO_CTRL_RTSTYP		(1 << 0)

#define MCPC_CTRL2			0x36
#define MCPC_CTRL2_SET			0x37
#define MCPC_CTRL2_CLR			0x38
#define MCPC_CTRL2_MCPC_CK_EN		(1 << 0)

#define OTHER_FUNC_CTRL			0x80
#define OTHER_FUNC_CTRL_SET		0x81
#define OTHER_FUNC_CTRL_CLR		0x82
#define OTHER_FUNC_CTRL_BDIS_ACON_EN	(1 << 4)
#define OTHER_FUNC_CTRL_FIVEWIRE_MODE	(1 << 2)

#define OTHER_IFC_CTRL			0x83
#define OTHER_IFC_CTRL_SET		0x84
#define OTHER_IFC_CTRL_CLR		0x85
#define OTHER_IFC_CTRL_OE_INT_EN	(1 << 6)
#define OTHER_IFC_CTRL_CEA2011_MODE	(1 << 5)
#define OTHER_IFC_CTRL_FSLSSERIALMODE_4PIN	(1 << 4)
#define OTHER_IFC_CTRL_HIZ_ULPI_60MHZ_OUT	(1 << 3)
#define OTHER_IFC_CTRL_HIZ_ULPI		(1 << 2)
#define OTHER_IFC_CTRL_ALT_INT_REROUTE	(1 << 0)

#define OTHER_INT_EN_RISE		0x86
#define OTHER_INT_EN_RISE_SET		0x87
#define OTHER_INT_EN_RISE_CLR		0x88
#define OTHER_INT_EN_FALL		0x89
#define OTHER_INT_EN_FALL_SET		0x8A
#define OTHER_INT_EN_FALL_CLR		0x8B
#define OTHER_INT_STS			0x8C
#define OTHER_INT_LATCH			0x8D
#define OTHER_INT_VB_SESS_VLD		(1 << 7)
#define OTHER_INT_DM_HI			(1 << 6) /* not valid for "latch" reg */
#define OTHER_INT_DP_HI			(1 << 5) /* not valid for "latch" reg */
#define OTHER_INT_BDIS_ACON		(1 << 3) /* not valid for "fall" regs */
#define OTHER_INT_MANU			(1 << 1)
#define OTHER_INT_ABNORMAL_STRESS	(1 << 0)

#define ID_STATUS			0x96
#define ID_RES_FLOAT			(1 << 4)
#define ID_RES_440K			(1 << 3)
#define ID_RES_200K			(1 << 2)
#define ID_RES_102K			(1 << 1)
#define ID_RES_GND			(1 << 0)

#define POWER_CTRL			0xAC
#define POWER_CTRL_SET			0xAD
#define POWER_CTRL_CLR			0xAE
#define POWER_CTRL_OTG_ENAB		(1 << 5)

#define OTHER_IFC_CTRL2			0xAF
#define OTHER_IFC_CTRL2_SET		0xB0
#define OTHER_IFC_CTRL2_CLR		0xB1
#define OTHER_IFC_CTRL2_ULPI_STP_LOW	(1 << 4)
#define OTHER_IFC_CTRL2_ULPI_TXEN_POL	(1 << 3)
#define OTHER_IFC_CTRL2_ULPI_4PIN_2430	(1 << 2)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_MASK	(3 << 0) /* bits 0 and 1 */
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT1N	(0 << 0)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT2N	(1 << 0)

#define REG_CTRL_EN			0xB2
#define REG_CTRL_EN_SET			0xB3
#define REG_CTRL_EN_CLR			0xB4
#define REG_CTRL_ERROR			0xB5
#define ULPI_I2C_CONFLICT_INTEN		(1 << 0)

#define OTHER_FUNC_CTRL2		0xB8
#define OTHER_FUNC_CTRL2_SET		0xB9
#define OTHER_FUNC_CTRL2_CLR		0xBA
#define OTHER_FUNC_CTRL2_VBAT_TIMER_EN	(1 << 0)

/* following registers do not have separate _clr and _set registers */
#define VBUS_DEBOUNCE			0xC0
#define ID_DEBOUNCE			0xC1
#define VBAT_TIMER			0xD3
#define PHY_PWR_CTRL			0xFD
#define PHY_PWR_PHYPWD			(1 << 0)
#define PHY_CLK_CTRL			0xFE
#define PHY_CLK_CTRL_CLOCKGATING_EN	(1 << 2)
#define PHY_CLK_CTRL_CLK32K_EN		(1 << 1)
#define REQ_PHY_DPLL_CLK		(1 << 0)
#define PHY_CLK_CTRL_STS		0xFF
#define PHY_DPLL_CLK			(1 << 0)

/* In module TWL4030_MODULE_PM_MASTER */
#define PROTECT_KEY			0x0E
#define STS_HW_CONDITIONS		0x0F

/* In module TWL4030_MODULE_PM_RECEIVER */
#define VUSB_DEDICATED1			0x7D
#define VUSB_DEDICATED2			0x7E
#define VUSB1V5_DEV_GRP			0x71
#define VUSB1V5_TYPE			0x72
#define VUSB1V5_REMAP			0x73
#define VUSB1V8_DEV_GRP			0x74
#define VUSB1V8_TYPE			0x75
#define VUSB1V8_REMAP			0x76
#define VUSB3V1_DEV_GRP			0x77
#define VUSB3V1_TYPE			0x78
#define VUSB3V1_REMAP			0x79

/* In module TWL4030_MODULE_INTBR */
#define PMBR1				0x0D
#define GPIO_USB_4PIN_ULPI_2430C	(3 << 0)

#define TPS65921_USB_DTCT_CTRL		0x02
#define TPS65921_USB_CHG_DET_EN_SW	(1 << 7)
#define TPS65921_USB_DET_STS_MASK	(3 << 2)
#define TPS65921_USB_DET_STS_100MA	(1 << 2)
#define TPS65921_USB_DET_STS_500MA	(2 << 2)
#define TPS65921_USB_HW_CHRG_DET_EN	(1 << 0)

#define TPS65921_USB_SW_CHRG_CTRL	0x03
#define TPS65921_CHGD_SERX_DM_LOWV	(1 << 5)
#define TPS65921_CHGD_SERX_DP_LOWV	(1 << 4)

#define IRQ_WAKE_LOCK_TIMEOUT       (5*HZ)

enum linkstat {
	USB_LINK_UNKNOWN = 0,
	USB_LINK_NONE,
	USB_LINK_VBUS,
	USB_LINK_ID,
};

struct twl4030_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

	/* TWL4030 internal USB regulator supplies */
	struct regulator	*usb1v5;
	struct regulator	*usb1v8;
	struct regulator	*usb3v1;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	/* pin configuration */
	enum twl4030_usb_mode	usb_mode;

	int			irq;
	u8			linkstat;
	u8			asleep;
	bool		irq_enabled;
	struct work_struct irq_work;
    struct wake_lock irq_wake_lock;
};

#if  defined(CONFIG_REGULATOR_BQ24073)
static struct regulator *bci_regulator;
#endif

#ifdef CONFIG_CHARGER_MAX8903
extern void max8903_charger_enable(int);
#endif
/* internal define on top of container_of */
#define xceiv_to_twl(x)		container_of((x), struct twl4030_usb, otg);

/*-------------------------------------------------------------------------*/

static int twl4030_i2c_write_u8_verify(struct twl4030_usb *twl,
		u8 module, u8 data, u8 address)
{
	u8 check;

	if ((twl4030_i2c_write_u8(module, data, address) >= 0) &&
	    (twl4030_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	dev_dbg(twl->dev, "Write%d[%d,0x%x] wrote %02x but read %02x\n",
			1, module, address, check, data);

	/* Failed once: Try again */
	if ((twl4030_i2c_write_u8(module, data, address) >= 0) &&
	    (twl4030_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	dev_dbg(twl->dev, "Write%d[%d,0x%x] wrote %02x but read %02x\n",
			2, module, address, check, data);

	/* Failed again: Return error */
	return -EBUSY;
}

#define twl4030_usb_write_verify(twl, address, data)	\
	twl4030_i2c_write_u8_verify(twl, TWL4030_MODULE_USB, (data), (address))

static inline int twl4030_usb_write(struct twl4030_usb *twl,
		u8 address, u8 data)
{
	int ret = 0;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_USB, data, address);
	if (ret < 0)
		dev_dbg(twl->dev,
			"TWL4030:USB:Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline int twl4030_readb(struct twl4030_usb *twl, u8 module, u8 address)
{
	u8 data;
	int ret = 0;

	ret = twl4030_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_dbg(twl->dev,
			"TWL4030:readb[0x%x,0x%x] Error %d\n",
					module, address, ret);

	return ret;
}

static inline int twl4030_usb_read(struct twl4030_usb *twl, u8 address)
{
	return twl4030_readb(twl, TWL4030_MODULE_USB, address);
}

/*-------------------------------------------------------------------------*/
#if 0
static void dump_regs(struct twl4030_usb *twl)
{
	printk("--------- TWL4030 regs ---------\n");
	printk("FUNC_CTRL: 0x%02x\n", twl4030_usb_read(twl, FUNC_CTRL));
	printk("IFC_CTRL: 0x%02x\n", twl4030_usb_read(twl, IFC_CTRL));
	printk("OTG_CTRL: 0x%02x\n", twl4030_usb_read(twl, TWL4030_OTG_CTRL));
	printk("USB_INT_EN_RISE: 0x%02x\n", twl4030_usb_read(twl, USB_INT_EN_RISE));
	printk("OTHER_FUNC_CTRL: 0x%02x\n", twl4030_usb_read(twl, OTHER_FUNC_CTRL));
	printk("OTHER_IFC_CTRL: 0x%02x\n", twl4030_usb_read(twl, OTHER_IFC_CTRL));
	printk("ID_STATUS: 0x%02x\n", twl4030_usb_read(twl, ID_STATUS));
}
#endif

static inline int
twl4030_usb_set_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, reg + 1, bits);
}

static inline int
twl4030_usb_clear_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(twl, reg + 2, bits);
}

/*-------------------------------------------------------------------------*/

static enum linkstat twl4030_usb_linkstat(struct twl4030_usb *twl)
{
	int	status;
	int	linkstat = USB_LINK_UNKNOWN;

	/*
	 * For ID/VBUS sensing, see manual section 15.4.8 ...
	 * except when using only battery backup power, two
	 * comparators produce VBUS_PRES and ID_PRES signals,
	 * which don't match docs elsewhere.  But ... BIT(7)
	 * and BIT(2) of STS_HW_CONDITIONS, respectively, do
	 * seem to match up.  If either is true the USB_PRES
	 * signal is active, the OTG module is activated, and
	 * its interrupt may be raised (may wake the system).
	 */
	status = twl4030_readb(twl, TWL4030_MODULE_PM_MASTER,
			STS_HW_CONDITIONS);
	if (status < 0)
		dev_err(twl->dev, "USB link status err %d\n", status);
	else if (status & (BIT(7) | BIT(2))) {
		if (status & BIT(2))
			linkstat = USB_LINK_ID;
		else
			linkstat = USB_LINK_VBUS;
	} else
		linkstat = USB_LINK_NONE;

	dev_dbg(twl->dev, "HW_CONDITIONS 0x%02x/%d; link %d\n",
			status, status, linkstat);

	/* REVISIT this assumes host and peripheral controllers
	 * are registered, and that both are active...
	 */

	spin_lock_irq(&twl->lock);
	twl->linkstat = linkstat;
	if (linkstat == USB_LINK_ID) {
		twl->otg.default_a = true;
		twl->otg.state = OTG_STATE_A_IDLE;
	} else {
		twl->otg.default_a = false;
		twl->otg.state = OTG_STATE_B_IDLE;
	}
	spin_unlock_irq(&twl->lock);

	return linkstat;
}

static void twl4030_usb_set_mode(struct twl4030_usb *twl, int mode)
{
	twl->usb_mode = mode;

	switch (mode) {
	case T2_USB_MODE_ULPI:
		twl4030_usb_clear_bits(twl, IFC_CTRL, IFC_CTRL_CARKITMODE);
		twl4030_usb_set_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);
		twl4030_usb_clear_bits(twl, FUNC_CTRL,
					FUNC_CTRL_XCVRSELECT_MASK |
					FUNC_CTRL_OPMODE_MASK);
		break;
	case -1:
		/* FIXME: power on defaults */
		break;
	default:
		dev_err(twl->dev, "unsupported T2 transceiver mode %d\n",
				mode);
		break;
	};
}

static void twl4030_i2c_access(struct twl4030_usb *twl, int on)
{
	unsigned long timeout;
	int val = twl4030_usb_read(twl, PHY_CLK_CTRL);

	if (val >= 0) {
		if (on) {
			/* enable DPLL to access PHY registers over I2C */
			val |= REQ_PHY_DPLL_CLK;
			WARN_ON(twl4030_usb_write_verify(twl, PHY_CLK_CTRL,
						(u8)val) < 0);

			timeout = jiffies + HZ;
			while (!(twl4030_usb_read(twl, PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK)
				&& time_before(jiffies, timeout))
					udelay(10);
			if (!(twl4030_usb_read(twl, PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK))
				dev_err(twl->dev, "Timeout setting T2 HSUSB "
						"PHY DPLL clock\n");
		} else {
			/* let ULPI control the DPLL clock */
			val &= ~REQ_PHY_DPLL_CLK;
			WARN_ON(twl4030_usb_write_verify(twl, PHY_CLK_CTRL,
						(u8)val) < 0);
		}
	}
}

static void twl4030_phy_power(struct twl4030_usb *twl, int on)
{
	u8 pwr;

	pwr = twl4030_usb_read(twl, PHY_PWR_CTRL);
	if (on) {
		regulator_enable(twl->usb3v1);
		regulator_enable(twl->usb1v8);
		/*
		 * Disabling usb3v1 regulator (= writing 0 to VUSB3V1_DEV_GRP
		 * in twl4030) resets the VUSB_DEDICATED2 register. This reset
		 * enables VUSB3V1_SLEEP bit that remaps usb3v1 ACTIVE state to
		 * SLEEP. We work around this by clearing the bit after usv3v1
		 * is re-activated. This ensures that VUSB3V1 is really active.
		 */
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0,
							VUSB_DEDICATED2);
		regulator_enable(twl->usb1v5);
		pwr &= ~PHY_PWR_PHYPWD;
		WARN_ON(twl4030_usb_write_verify(twl, PHY_PWR_CTRL, pwr) < 0);
		twl4030_usb_write(twl, PHY_CLK_CTRL,
				  twl4030_usb_read(twl, PHY_CLK_CTRL) |
					(PHY_CLK_CTRL_CLOCKGATING_EN |
						PHY_CLK_CTRL_CLK32K_EN));
	} else  {
		pwr |= PHY_PWR_PHYPWD;
		WARN_ON(twl4030_usb_write_verify(twl, PHY_PWR_CTRL, pwr) < 0);
		regulator_disable(twl->usb1v5);
		regulator_disable(twl->usb1v8);
		regulator_disable(twl->usb3v1);
	}
}

static void twl4030_phy_suspend(struct twl4030_usb *twl, int controller_off)
{
	if (twl->asleep)
		return;

	if (twl->otg.gadget) {
        dev_dbg(twl->dev, "notifying gadget of disconnect\n");
		usb_gadget_disconnect(twl->otg.gadget);
    }

	if (twl->otg.link_save_context)
		twl->otg.link_save_context(&twl->otg);

	twl4030_phy_power(twl, 0);
	twl->asleep = 1;
}

static void twl4030_phy_resume(struct twl4030_usb *twl)
{
	if (!twl->asleep)
		return;

	twl4030_phy_power(twl, 1);
	twl4030_i2c_access(twl, 1);
	twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(twl, 0);

	twl->asleep = 0;

	if (twl->otg.link_restore_context)
		twl->otg.link_restore_context(&twl->otg);

	if (twl->otg.gadget) {
        dev_dbg(twl->dev, "Notifying gadget of connect\n");
		usb_gadget_connect(twl->otg.gadget);
    }
}

static int twl4030_usb_ldo_init(struct twl4030_usb *twl)
{
	const uint8_t key1 = twl_rev_is_tps65921() ? 0xFC : 0xC0;
	const uint8_t key2 = twl_rev_is_tps65921() ? 0x96 : 0x0C;

	/* Enable writing to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, key1, PROTECT_KEY);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, key2, PROTECT_KEY);

	/* put VUSB3V1 LDO in active state */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB_DEDICATED2);

	/* input to VUSB3V1 LDO is from VBAT, not VBUS */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x14, VUSB_DEDICATED1);

	/* Initialize 3.1V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_DEV_GRP);

	twl->usb3v1 = regulator_get(twl->dev, "usb3v1");
	if (IS_ERR(twl->usb3v1))
		return -ENODEV;

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_TYPE);

	/* Initialize 1.5V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_DEV_GRP);

	twl->usb1v5 = regulator_get(twl->dev, "usb1v5");
	if (IS_ERR(twl->usb1v5))
		goto fail1;

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_TYPE);

	/* Initialize 1.8V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_DEV_GRP);

	twl->usb1v8 = regulator_get(twl->dev, "usb1v8");
	if (IS_ERR(twl->usb1v8))
		goto fail2;

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_TYPE);

	/* disable access to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, PROTECT_KEY);

	return 0;

fail2:
	regulator_put(twl->usb1v5);
	twl->usb1v5 = NULL;
fail1:
	regulator_put(twl->usb3v1);
	twl->usb3v1 = NULL;
	return -ENODEV;
}

static ssize_t twl4030_usb_vbus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&twl->lock, flags);
	ret = sprintf(buf, "%s\n",
			(twl->linkstat == USB_LINK_VBUS) ? "on" : "off");
	spin_unlock_irqrestore(&twl->lock, flags);

	return ret;
}
static DEVICE_ATTR(vbus, 0444, twl4030_usb_vbus_show, NULL);

static void twl4030_irq_work(struct work_struct *work)
{
	struct twl4030_usb *twl = container_of(work, struct twl4030_usb, irq_work);
	int status;

	/* FIXME add a set_power() method so that B-devices can
	 * configure the charger appropriately.  It's not always
	 * correct to consume VBUS power, and how much current to
	 * consume is a function of the USB configuration chosen
	 * by the host.
	 *
	 * REVISIT usb_gadget_vbus_connect(...) as needed, ditto
	 * its disconnect() sibling, when changing to/from the
	 * USB_LINK_VBUS state.  musb_hdrc won't care until it
	 * starts to handle softconnect right.
	 */

	status = twl4030_usb_linkstat(twl);
	if (status != USB_LINK_UNKNOWN) {
		if (status == USB_LINK_NONE) {
#ifdef CONFIG_CHARGER_MAX8903
		    max8903_charger_enable(0);
#endif
		    if (twl->otg.link_force_active)
			    twl->otg.link_force_active(0);

			twl4030_phy_suspend(twl, 0);
		} else {
            if (twl->otg.link_force_active)
                twl->otg.link_force_active(1);

			twl4030_phy_resume(twl);
#ifdef CONFIG_CHARGER_MAX8903
			max8903_charger_enable(1);
#endif
		}

#if defined(CONFIG_TWL4030_BCI_BATTERY)
		twl4030charger_usb_en(status == USB_LINK_VBUS);
#endif
#if  defined(CONFIG_REGULATOR_BQ24073)
		
        twl4030_i2c_write_u8(TWL4030_MODULE_MAIN_CHARGE,
                            TPS65921_USB_HW_CHRG_DET_EN,
                            TPS65921_USB_DTCT_CTRL);
        if(bci_regulator == NULL || IS_ERR(bci_regulator)) {
            bci_regulator = regulator_get(twl->dev, "bq24073");
		}

        if(!IS_ERR(bci_regulator)) {
            if(USB_LINK_VBUS == status) {
			    u8 chg_pres = 0;
			    int check_cnt = 0;
                int limit;
                int i;

				/* FIXME: This is ugly hack, but as the charger detection module
				 * is not wanting to set any interrupt flag, currently this
				 * appears to be the only way to do it for now.
				 * We don't care about delayed USB IRQs, as the VBUS and ID pin
				 * debounce timers are large enough, and we will handle the IRQ
				 * anyway after we return from the handler.
				 */
                msleep(150);

				chg_pres = twl4030_readb(twl, TWL4030_MODULE_MAIN_CHARGE,
                                         TPS65921_USB_DTCT_CTRL);
                /* if usb is connected to usb host 500ma, otherwise 1500ma limit */
                if((chg_pres & TPS65921_USB_DET_STS_MASK) != TPS65921_USB_DET_STS_500MA)
                    limit = 500000;
                else
                    limit = 1500000;

                if(!regulator_is_enabled(bci_regulator))
                    regulator_enable(bci_regulator);
				
                regulator_set_current_limit(bci_regulator, limit, limit);

                //printk("Set USB Charger limit to %duA\n",limit);
            } else {
			    /* No VBUS or VBus from charge pump (ID pin low and,
                 * and device conneted to OTG port)
                 */

                if(regulator_is_enabled(bci_regulator))
                    regulator_disable(bci_regulator);
                //printk("Disable USB Charger\n");
            }
        }
#endif
	}

	sysfs_notify(&twl->dev->kobj, NULL, "vbus");
}

static irqreturn_t twl4030_usb_irq(int irq, void *_twl)
{
	struct twl4030_usb *twl = _twl;
    wake_lock_timeout(&twl->irq_wake_lock, IRQ_WAKE_LOCK_TIMEOUT);
	schedule_work(&twl->irq_work);
	return IRQ_HANDLED;
}

static int twl4030_set_suspend(struct otg_transceiver *x, int suspend)
{
#ifndef CONFIG_MACH_OMAP3621_EVT1A
	struct twl4030_usb *twl = xceiv_to_twl(x);

	// For Encore this is done on the VBUS interrupt
	if (suspend)
		twl4030_phy_suspend(twl, 1);
	else
		twl4030_phy_resume(twl);
#endif 

	return 0;
}

static int twl4030_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct twl4030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.gadget = gadget;
	if (!gadget)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int twl4030_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct twl4030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.host = host;
	if (!host)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int __init twl4030_usb_probe(struct platform_device *pdev)
{
	struct twl4030_usb_data *pdata = pdev->dev.platform_data;
	struct twl4030_usb	*twl;
	int			status, err;

	if (!pdata) {
		dev_dbg(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	twl = kzalloc(sizeof *twl, GFP_KERNEL);
	if (!twl)
		return -ENOMEM;

	twl->dev		= &pdev->dev;
	if (pdata->bci_supply)
		pdata->bci_supply->dev	= twl->dev;
	twl->irq		= platform_get_irq(pdev, 0);
	twl->otg.dev		= twl->dev;
	twl->otg.label		= "twl4030";
	twl->otg.set_host	= twl4030_set_host;
	twl->otg.set_peripheral	= twl4030_set_peripheral;
	twl->otg.set_suspend	= twl4030_set_suspend;
	twl->usb_mode		= pdata->usb_mode;
	twl->asleep		= 1;

    wake_lock_init(&twl->irq_wake_lock, WAKE_LOCK_SUSPEND, "twl4030-irq");
	INIT_WORK(&twl->irq_work, twl4030_irq_work);

	/* init spinlock for workqueue */
	spin_lock_init(&twl->lock);

	err = twl4030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		kfree(twl);
		return err;
	}
	otg_set_transceiver(&twl->otg);

	platform_set_drvdata(pdev, twl);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	/*
	 * One time configuration to route MCPC pins to the MADC for
	 * monitoring */
	regulator_enable(twl->usb3v1);
	regulator_enable(twl->usb1v8);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0,
			VUSB_DEDICATED2);
	regulator_enable(twl->usb1v5);
	twl4030_usb_write(twl, CARKIT_ANA_CTRL,
		twl4030_usb_read(twl, CARKIT_ANA_CTRL) | SEL_MADC_MCPC);
	regulator_disable(twl->usb1v5);
	regulator_disable(twl->usb1v8);
	regulator_disable(twl->usb3v1);

	/* Our job is to use irqs and status from the power module
	 * to keep the transceiver disabled when nothing's connected.
	 *
	 * FIXME we actually shouldn't start enabling it until the
	 * USB controller drivers have said they're ready, by calling
	 * set_host() and/or set_peripheral() ... OTG_capable boards
	 * need both handles, otherwise just one suffices.
	 */
	twl->irq_enabled = true;
	status = request_irq(twl->irq, twl4030_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl4030_usb", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq, status);
		kfree(twl);
		return status;
	}

	/* The IRQ handler just handles changes from the previous states
	 * of the ID and VBUS pins ... in probe() we must initialize that
	 * previous state.  The easy way:  fake an IRQ.
	 *
	 * REVISIT:  a real IRQ might have happened already, if PREEMPT is
	 * enabled.  Else the IRQ may not yet be configured or enabled,
	 * because of scheduling delays.
	 */
	twl4030_phy_suspend(twl, 0);
	twl4030_usb_irq(twl->irq, twl);

	dev_info(&pdev->dev, "Initialized TWL4030 USB module\n");
	return 0;
}

static int __exit twl4030_usb_remove(struct platform_device *pdev)
{
	struct twl4030_usb *twl = platform_get_drvdata(pdev);
	int val;

    cancel_work_sync(&twl->irq_work);
	free_irq(twl->irq, twl);
	device_remove_file(twl->dev, &dev_attr_vbus);

	/* set transceiver mode to power on defaults */
	twl4030_usb_set_mode(twl, -1);

	/* autogate 60MHz ULPI clock,
	 * clear dpll clock request for i2c access,
	 * disable 32KHz
	 */
	val = twl4030_usb_read(twl, PHY_CLK_CTRL);
	if (val >= 0) {
		val |= PHY_CLK_CTRL_CLOCKGATING_EN;
		val &= ~(PHY_CLK_CTRL_CLK32K_EN | REQ_PHY_DPLL_CLK);
		twl4030_usb_write(twl, PHY_CLK_CTRL, (u8)val);
	}

	/* disable complete OTG block */
	twl4030_usb_clear_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);

	twl4030_phy_power(twl, 0);
	regulator_put(twl->usb1v5);
	regulator_put(twl->usb1v8);
	regulator_put(twl->usb3v1);

    wake_lock_destroy(&twl->irq_wake_lock);
	kfree(twl);

	return 0;
}

static struct platform_driver twl4030_usb_driver = {
	.probe		= twl4030_usb_probe,
	.remove		= __exit_p(twl4030_usb_remove),
	.driver		= {
		.name	= "twl4030_usb",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_usb_init(void)
{
	return platform_driver_register(&twl4030_usb_driver);
}
subsys_initcall(twl4030_usb_init);

static void __exit twl4030_usb_exit(void)
{
	platform_driver_unregister(&twl4030_usb_driver);
}
module_exit(twl4030_usb_exit);

MODULE_ALIAS("platform:twl4030_usb");
MODULE_AUTHOR("Texas Instruments, Inc, Nokia Corporation");
MODULE_DESCRIPTION("TWL4030 USB transceiver driver");
MODULE_LICENSE("GPL");
