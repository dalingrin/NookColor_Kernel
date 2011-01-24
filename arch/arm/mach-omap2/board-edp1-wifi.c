/* linux/arch/arm/mach-omap2/board-edp1-wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>

#define EDP1_WIFI_PMENA_GPIO	22
#define EDP1_WIFI_IRQ_GPIO	15
#define EDP1_WIFI_EN_POW	16

static int edp1_wifi_cd = 0;		/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

int omap_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

int edp1_wifi_status(int irq)
{
	return edp1_wifi_cd;
}

int edp1_wifi_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	edp1_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(edp1_wifi_set_carddetect);
#endif

static int edp1_wifi_power_state;

int edp1_wifi_power(int on)
{
	printk("%s: %d\n", __func__, on);
	gpio_set_value(EDP1_WIFI_PMENA_GPIO, on);
	edp1_wifi_power_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(edp1_wifi_power);
#endif

static int edp1_wifi_reset_state;
int edp1_wifi_reset(int on)
{
	printk("%s: %d\n", __func__, on);
	edp1_wifi_reset_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(edp1_wifi_reset);
#endif

struct wifi_platform_data edp1_wifi_control = {
        .set_power	= edp1_wifi_power,
	.set_reset	= edp1_wifi_reset,
	.set_carddetect	= edp1_wifi_set_carddetect,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource edp1_wifi_resources[] = {
	[0] = {
		.name		= "device_wifi_irq",
		.start		= OMAP_GPIO_IRQ(EDP1_WIFI_IRQ_GPIO),
		.end		= OMAP_GPIO_IRQ(EDP1_WIFI_IRQ_GPIO),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device edp1_wifi_device = {
        .name           = "device_wifi",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(edp1_wifi_resources),
        .resource       = edp1_wifi_resources,
        .dev            = {
                .platform_data = &edp1_wifi_control,
        },
};
#endif

static int __init edp1_wifi_init(void)
{
	int ret;

	printk("%s: start\n", __func__);
	ret = gpio_request(EDP1_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			EDP1_WIFI_IRQ_GPIO);
		goto out;
	}
	ret = gpio_request(EDP1_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			EDP1_WIFI_PMENA_GPIO);
		gpio_free(EDP1_WIFI_IRQ_GPIO);
		goto out;
	}
	ret = gpio_request(EDP1_WIFI_EN_POW, "wifi_en_pow");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			EDP1_WIFI_EN_POW);
		gpio_free(EDP1_WIFI_EN_POW);
		goto out;
	}

	gpio_direction_input(EDP1_WIFI_IRQ_GPIO);
	gpio_direction_output(EDP1_WIFI_PMENA_GPIO, 0);
	gpio_direction_output(EDP1_WIFI_EN_POW, 1);
#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&edp1_wifi_device);
#endif
out:
        return ret;
}

device_initcall(edp1_wifi_init);
