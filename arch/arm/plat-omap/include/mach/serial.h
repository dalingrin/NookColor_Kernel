/*
 *  arch/arm/plat-omap/include/mach/serial.h
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H

#if defined(CONFIG_ARCH_OMAP1)
/* OMAP1 serial ports */
#define OMAP_UART1_BASE		0xfffb0000
#define OMAP_UART2_BASE		0xfffb0800
#define OMAP_UART3_BASE		0xfffb9800
#elif defined(CONFIG_ARCH_OMAP2)
/* OMAP2 serial ports */
#define OMAP_UART1_BASE		0x4806a000
#define OMAP_UART2_BASE		0x4806c000
#define OMAP_UART3_BASE		0x4806e000
#elif defined(CONFIG_ARCH_OMAP3)
/* OMAP3 serial ports */
#define OMAP_UART1_BASE		0x4806a000
#define OMAP_UART2_BASE		0x4806c000
#define OMAP_UART3_BASE		0x49020000
#define OMAP_UART_EXT_BASE	0x10000000
#endif

#if defined(CONFIG_MACH_OMAP_ZOOM2) || defined(CONFIG_MACH_OMAP_ZOOM3)
#define OMAP_MAX_NR_PORTS	4
#elif defined(CONFIG_MACH_OMAP3621_BOXER) || defined (CONFIG_MACH_OMAP3621_EVT1A) || defined (CONFIG_MACH_OMAP_3621_EDP)
#define OMAP_MAX_NR_PORTS	2
#else
#define OMAP_MAX_NR_PORTS	3
#endif
#define OMAP1510_BASE_BAUD	(12000000/16)
#define OMAP16XX_BASE_BAUD	(48000000/16)
#define OMAP24XX_BASE_BAUD	(48000000/16)

#define is_omap_port(pt)	({int __ret = 0;			\
			if ((pt)->port.mapbase == OMAP_UART1_BASE ||	\
			    (pt)->port.mapbase == OMAP_UART2_BASE ||	\
			    (pt)->port.mapbase == OMAP_UART3_BASE)	\
				__ret = 1;				\
			__ret;						\
			})

#ifndef __ASSEMBLER__
extern void omap_serial_init(void);
extern int omap_uart_can_sleep(void);
extern void omap_uart_check_wakeup(void);
extern void omap_uart_prepare_suspend(void);
extern void omap_uart_prepare_idle(int num, int power_state);
extern void omap_uart_resume_idle(int num);
extern int omap_uart_enable_irqs(int enable);
#endif

#endif
