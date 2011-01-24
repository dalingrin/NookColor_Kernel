/*
 * linux/arch/arm/mach-omap2/usb-ehci.c
 *
 * This file will contain the board specific details for the
 * Synopsys EHCI host controller on OMAP3430
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * Generalization by:
 * Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <mach/mux.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/usb.h>

static struct resource ehci_resources[] = {
	[0] = {
		.start   = OMAP34XX_HSUSB_HOST_BASE + 0x800,
		.end     = OMAP34XX_HSUSB_HOST_BASE + 0x800 + SZ_1K - 1,
		.flags   = IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start   = INT_34XX_EHCI_IRQ,
		.flags   = IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name           = "ehci-omap",
	.id             = 0,
	.dev = {
		.dma_mask               = &ehci_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.platform_data          = NULL,
	},
	.num_resources  = ARRAY_SIZE(ehci_resources),
	.resource       = ehci_resources,
};

/* MUX settings for EHCI pins */
/*
 * setup_ehci_io_mux - initialize IO pad mux for USBHOST
 */
static void setup_ehci_io_mux(void)
{
#ifdef CONFIG_OMAP_EHCI_PHY_MODE
	/* PHY mode of operation for board: 750-2083-001
	 * ISP1504 connected to Port1 and Port2
	 * Do Func Mux setting for 12-pin ULPI PHY mode
	 */

	/* Port1 */
	omap_cfg_reg(Y9_3430_USB1HS_PHY_STP);
	omap_cfg_reg(Y8_3430_USB1HS_PHY_CLK);
	omap_cfg_reg(AA14_3430_USB1HS_PHY_DIR);
	omap_cfg_reg(AA11_3430_USB1HS_PHY_NXT);
	omap_cfg_reg(W13_3430_USB1HS_PHY_DATA0);
	omap_cfg_reg(W12_3430_USB1HS_PHY_DATA1);
	omap_cfg_reg(W11_3430_USB1HS_PHY_DATA2);
	omap_cfg_reg(Y11_3430_USB1HS_PHY_DATA3);
	omap_cfg_reg(W9_3430_USB1HS_PHY_DATA4);
	omap_cfg_reg(Y12_3430_USB1HS_PHY_DATA5);
	omap_cfg_reg(W8_3430_USB1HS_PHY_DATA6);
	omap_cfg_reg(Y13_3430_USB1HS_PHY_DATA7);

	/* Port2 */
	omap_cfg_reg(AA10_3430_USB2HS_PHY_STP);
	omap_cfg_reg(AA8_3430_USB2HS_PHY_CLK);
	omap_cfg_reg(AA9_3430_USB2HS_PHY_DIR);
	omap_cfg_reg(AB11_3430_USB2HS_PHY_NXT);
	omap_cfg_reg(AB10_3430_USB2HS_PHY_DATA0);
	omap_cfg_reg(AB9_3430_USB2HS_PHY_DATA1);
	omap_cfg_reg(W3_3430_USB2HS_PHY_DATA2);
	omap_cfg_reg(T4_3430_USB2HS_PHY_DATA3);
	omap_cfg_reg(T3_3430_USB2HS_PHY_DATA4);
	omap_cfg_reg(R3_3430_USB2HS_PHY_DATA5);
	omap_cfg_reg(R4_3430_USB2HS_PHY_DATA6);
	omap_cfg_reg(T2_3430_USB2HS_PHY_DATA7);

#else
	/* Set Func mux for :
	 * TLL mode of operation
	 * 12-pin ULPI SDR TLL mode for Port1/2/3
	 */

	/* Port1 */
	omap_cfg_reg(Y9_3430_USB1HS_TLL_STP);
	omap_cfg_reg(Y8_3430_USB1HS_TLL_CLK);
	omap_cfg_reg(AA14_3430_USB1HS_TLL_DIR);
	omap_cfg_reg(AA11_3430_USB1HS_TLL_NXT);
	omap_cfg_reg(W13_3430_USB1HS_TLL_DATA0);
	omap_cfg_reg(W12_3430_USB1HS_TLL_DATA1);
	omap_cfg_reg(W11_3430_USB1HS_TLL_DATA2);
	omap_cfg_reg(Y11_3430_USB1HS_TLL_DATA3);
	omap_cfg_reg(W9_3430_USB1HS_TLL_DATA4);
	omap_cfg_reg(Y12_3430_USB1HS_TLL_DATA5);
	omap_cfg_reg(W8_3430_USB1HS_TLL_DATA6);
	omap_cfg_reg(Y13_3430_USB1HS_TLL_DATA7);

	/* Port2 */
	omap_cfg_reg(AA10_3430_USB2HS_TLL_STP);
	omap_cfg_reg(AA8_3430_USB2HS_TLL_CLK);
	omap_cfg_reg(AA9_3430_USB2HS_TLL_DIR);
	omap_cfg_reg(AB11_3430_USB2HS_TLL_NXT);
	omap_cfg_reg(AB10_3430_USB2HS_TLL_DATA0);
	omap_cfg_reg(AB9_3430_USB2HS_TLL_DATA1);
	omap_cfg_reg(W3_3430_USB2HS_TLL_DATA2);
	omap_cfg_reg(T4_3430_USB2HS_TLL_DATA3);
	omap_cfg_reg(T3_3430_USB2HS_TLL_DATA4);
	omap_cfg_reg(R3_3430_USB2HS_TLL_DATA5);
	omap_cfg_reg(R4_3430_USB2HS_TLL_DATA6);
	omap_cfg_reg(T2_3430_USB2HS_TLL_DATA7);

	/* Port3 */
	omap_cfg_reg(AB3_3430_USB3HS_TLL_STP);
	omap_cfg_reg(AA6_3430_USB3HS_TLL_CLK);
	omap_cfg_reg(AA3_3430_USB3HS_TLL_DIR);
	omap_cfg_reg(Y3_3430_USB3HS_TLL_NXT);
	omap_cfg_reg(AA5_3430_USB3HS_TLL_DATA0);
	omap_cfg_reg(Y4_3430_USB3HS_TLL_DATA1);
	omap_cfg_reg(Y5_3430_USB3HS_TLL_DATA2);
	omap_cfg_reg(W5_3430_USB3HS_TLL_DATA3);
	omap_cfg_reg(AB12_3430_USB3HS_TLL_DATA4);
	omap_cfg_reg(AB13_3430_USB3HS_TLL_DATA5);
	omap_cfg_reg(AA13_3430_USB3HS_TLL_DATA6);
	omap_cfg_reg(AA12_3430_USB3HS_TLL_DATA7);
#endif /* CONFIG_OMAP_EHCI_PHY_MODE */

	return;
}

void __init usb_ehci_init(void)
{
	/* Setup Pin IO MUX for EHCI */
	if (cpu_is_omap34xx())
		setup_ehci_io_mux();

	if (platform_device_register(&ehci_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (EHCI) device\n");
		return;
	}
}


