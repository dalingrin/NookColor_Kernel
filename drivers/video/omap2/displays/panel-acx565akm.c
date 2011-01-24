#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <mach/display.h>
#include <mach/dma.h>

#include "panel-acx565akm.h"

#define MIPID_CMD_READ_DISP_ID         0x04
#define MIPID_CMD_READ_RED             0x06
#define MIPID_CMD_READ_GREEN           0x07
#define MIPID_CMD_READ_BLUE            0x08
#define MIPID_CMD_READ_DISP_STATUS     0x09
#define MIPID_CMD_RDDSDR               0x0F
#define MIPID_CMD_SLEEP_IN             0x10
#define MIPID_CMD_SLEEP_OUT            0x11
#define MIPID_CMD_DISP_OFF             0x28
#define MIPID_CMD_DISP_ON              0x29
#define MIPID_CMD_WRITE_DISP_BRIGHTNESS        0x51
#define MIPID_CMD_READ_DISP_BRIGHTNESS 0x52
#define MIPID_CMD_WRITE_CTRL_DISP      0x53

#define CTRL_DISP_BRIGHTNESS_CTRL_ON   (1 << 5)
#define CTRL_DISP_AMBIENT_LIGHT_CTRL_ON        (1 << 4)
#define CTRL_DISP_BACKLIGHT_ON         (1 << 2)
#define CTRL_DISP_AUTO_BRIGHTNESS_ON   (1 << 1)

#define MIPID_CMD_READ_CTRL_DISP       0x54
#define MIPID_CMD_WRITE_CABC           0x55
#define MIPID_CMD_READ_CABC            0x56

#define MIPID_VER_LPH8923              3
#define MIPID_VER_LS041Y3              4
#define MIPID_VER_L4F00311             8
#define MIPID_VER_ACX565AKM            9

struct acx565akm_device {
       struct backlight_device *bl_dev;
       int             enabled;
       int             model;
       int             revision;
       u8              display_id[3];
       int             has_bc:1;
       int             has_cabc:1;
       unsigned int    saved_bklight_level;
       unsigned long   hw_guard_end;           /* next value of jiffies
                                                  when we can issue the
                                                  next sleep in/out command */
       unsigned long   hw_guard_wait;          /* max guard time in jiffies */

       struct spi_device       *spi;
       struct mutex            mutex;
       struct omap_panel       panel;
       struct omap_display     *display;
};

static int acx565akm_bl_update_status(struct backlight_device *dev);

static void acx565akm_transfer(struct acx565akm_device *md, int cmd,
                             const u8 *wbuf, int wlen, u8 *rbuf, int rlen)
{
       struct spi_message      m;
       struct spi_transfer     *x, xfer[5];
       int                     r;

       BUG_ON(md->spi == NULL);

       spi_message_init(&m);

       memset(xfer, 0, sizeof(xfer));
       x = &xfer[0];

       cmd &=  0xff;
       x->tx_buf       = &cmd;
       x->bits_per_word = 9;
       x->len          = 2;

       if (rlen > 1 && wlen == 0) {
               /*
                * Between the command and the response data there is a
                * dummy clock cycle. Add an extra bit after the command
                * word to account for this.
                */
               x->bits_per_word = 10;
               cmd <<= 1;
       }
       spi_message_add_tail(x, &m);

       if (wlen) {
               x++;
               x->tx_buf       = wbuf;
               x->len          = wlen;
               x->bits_per_word = 9;
               spi_message_add_tail(x, &m);
       }

       if (rlen) {
               x++;
               x->rx_buf       = rbuf;
               x->len          = rlen;
               spi_message_add_tail(x, &m);
       }

       r = spi_sync(md->spi, &m);
       if (r < 0)
               dev_dbg(&md->spi->dev, "spi_sync %d\n", r);
}

static inline void acx565akm_cmd(struct acx565akm_device *md, int cmd)
{
       acx565akm_transfer(md, cmd, NULL, 0, NULL, 0);
}

static inline void acx565akm_write(struct acx565akm_device *md,
                              int reg, const u8 *buf, int len)
{
       acx565akm_transfer(md, reg, buf, len, NULL, 0);
}

static inline void acx565akm_read(struct acx565akm_device *md,
                             int reg, u8 *buf, int len)
{
       acx565akm_transfer(md, reg, NULL, 0, buf, len);
}

static void hw_guard_start(struct acx565akm_device *md, int guard_msec)
{
       md->hw_guard_wait = msecs_to_jiffies(guard_msec);
       md->hw_guard_end = jiffies + md->hw_guard_wait;
}

static void hw_guard_wait(struct acx565akm_device *md)
{
       unsigned long wait = md->hw_guard_end - jiffies;

       if ((long)wait > 0 && wait <= md->hw_guard_wait) {
               set_current_state(TASK_UNINTERRUPTIBLE);
               schedule_timeout(wait);
       }
}

static void set_sleep_mode(struct acx565akm_device *md, int on)
{
       int cmd, sleep_time = 50;

       if (on)
               cmd = MIPID_CMD_SLEEP_IN;
       else
               cmd = MIPID_CMD_SLEEP_OUT;
       hw_guard_wait(md);
       acx565akm_cmd(md, cmd);
       hw_guard_start(md, 120);
       /*
        * When we enable the panel, it seems we _have_ to sleep
        * 120 ms before sending the init string. When disabling the
        * panel we'll sleep for the duration of 2 frames, so that the
        * controller can still provide the PCLK,HS,VS signals. */
       if (!on)
               sleep_time = 120;
       msleep(sleep_time);
}

static void set_display_state(struct acx565akm_device *md, int enabled)
{
       int cmd = enabled ? MIPID_CMD_DISP_ON : MIPID_CMD_DISP_OFF;

       acx565akm_cmd(md, cmd);
}

static int panel_enabled(struct acx565akm_device *md)
{
       u32 disp_status;
       int enabled;

       acx565akm_read(md, MIPID_CMD_READ_DISP_STATUS, (u8 *)&disp_status, 4);
       disp_status = __be32_to_cpu(disp_status);
       enabled = (disp_status & (1 << 17)) && (disp_status & (1 << 10));
       dev_dbg(&md->spi->dev,
               "LCD panel %senabled by bootloader (status 0x%04x)\n",
               enabled ? "" : "not ", disp_status);
       return enabled;
}

static void enable_backlight_ctrl(struct acx565akm_device *md, int enable)
{
       u16 ctrl;

       acx565akm_read(md, MIPID_CMD_READ_CTRL_DISP, (u8 *)&ctrl, 1);
       if (enable) {
               ctrl |= CTRL_DISP_BRIGHTNESS_CTRL_ON |
                       CTRL_DISP_BACKLIGHT_ON;
       } else {
               ctrl &= ~(CTRL_DISP_BRIGHTNESS_CTRL_ON |
                         CTRL_DISP_BACKLIGHT_ON);
       }

       ctrl |= 1 << 8;
       acx565akm_write(md, MIPID_CMD_WRITE_CTRL_DISP, (u8 *)&ctrl, 2);
}

static void set_cabc_mode(struct acx565akm_device *md, int mode)
{
       u16 cabc_ctrl;

       cabc_ctrl = 0;
       acx565akm_read(md, MIPID_CMD_READ_CABC, (u8 *)&cabc_ctrl, 1);
       cabc_ctrl &= ~3;
       cabc_ctrl |= (1 << 8) | (mode & 3);
       acx565akm_write(md, MIPID_CMD_WRITE_CABC, (u8 *)&cabc_ctrl, 2);
}

static int get_cabc_mode(struct acx565akm_device *md)
{
       u8 cabc_ctrl;

       acx565akm_read(md, MIPID_CMD_READ_CABC, &cabc_ctrl, 1);
       return cabc_ctrl & 3;
}

static int panel_detect(struct acx565akm_device *md)
{
       acx565akm_read(md, MIPID_CMD_READ_DISP_ID, md->display_id, 3);
       dev_dbg(&md->spi->dev, "MIPI display ID: %02x%02x%02x\n",
               md->display_id[0], md->display_id[1], md->display_id[2]);

       switch (md->display_id[0]) {
       case 0x10:
               md->model = MIPID_VER_ACX565AKM;
               md->panel.name = "acx565akm";
               md->has_bc = 1;
               md->has_cabc = 1;
               break;
       case 0x29:
               md->model = MIPID_VER_L4F00311;
               md->panel.name = "l4f00311";
               break;
       case 0x45:
               md->model = MIPID_VER_LPH8923;
               md->panel.name = "lph8923";
               break;
       case 0x83:
               md->model = MIPID_VER_LS041Y3;
               md->panel.name = "ls041y3";
               break;
       default:
               md->panel.name = "unknown";
               dev_err(&md->spi->dev, "invalid display ID\n");
               return -ENODEV;
       }

       md->revision = md->display_id[1];

       pr_info("omapfb: %s rev %02x LCD detected\n",
                       md->panel.name, md->revision);

       return 0;
}

static int acx565akm_panel_enable(struct omap_display *display)
{
       struct acx565akm_device *md =
               (struct acx565akm_device *)display->panel->priv;

       dev_dbg(&md->spi->dev, "%s\n", __func__);

       mutex_lock(&md->mutex);

       if (display->hw_config.panel_enable)
               display->hw_config.panel_enable(display);

       md->enabled = panel_enabled(md);

       if (md->enabled) {
               dev_dbg(&md->spi->dev, "panel already enabled\n");
               mutex_unlock(&md->mutex);
               return 0;
       }

       set_sleep_mode(md, 0);
       md->enabled = 1;
       set_display_state(md, 1);

       mutex_unlock(&md->mutex);

       return acx565akm_bl_update_status(md->bl_dev);
}

static void acx565akm_panel_disable(struct omap_display *display)
{
       struct acx565akm_device *md =
               (struct acx565akm_device *)display->panel->priv;

       dev_dbg(&md->spi->dev, "%s\n", __func__);

       mutex_lock(&md->mutex);

       if (!md->enabled) {
               mutex_unlock(&md->mutex);
               return;
       }
       set_display_state(md, 0);
       set_sleep_mode(md, 1);
       md->enabled = 0;

       if (display->hw_config.panel_disable)
               display->hw_config.panel_disable(display);

       mutex_unlock(&md->mutex);
}

#if 0
static void acx565akm_set_mode(struct omap_display *display,
                             int x_res, int y_res, int bpp)
{
       struct acx565akm_device *md =
               (struct acx565akm_device *)display->panel->priv;
       u16 par;

       switch (bpp) {
       case 16:
               par = 0x150;
               break;
       case 18:
               par = 0x160;
               break;
       case 24:
               par = 0x170;
               break;
       }

       acx565akm_write(md, 0x3a, (u8 *)&par, 2);
}
#endif

static int acx565akm_panel_suspend(struct omap_display *display)
{
       acx565akm_panel_disable(display);
       return 0;
}

static int acx565akm_panel_resume(struct omap_display *display)
{
       return acx565akm_panel_enable(display);
}

static void acx565akm_set_brightness(struct acx565akm_device *md, int level)
{
       int bv;

       bv = level | (1 << 8);
       acx565akm_write(md, MIPID_CMD_WRITE_DISP_BRIGHTNESS, (u8 *)&bv, 2);

       if (level)
               enable_backlight_ctrl(md, 1);
       else
               enable_backlight_ctrl(md, 0);
}

static int acx565akm_get_actual_brightness(struct acx565akm_device *md)
{
       u8 bv;

       acx565akm_read(md, MIPID_CMD_READ_DISP_BRIGHTNESS, &bv, 1);

       return bv;
}

static int acx565akm_bl_update_status(struct backlight_device *dev)
{
       struct acx565akm_device *md = dev_get_drvdata(&dev->dev);
       struct omap_display *display = md->display;
       int r;
       int level;

       dev_dbg(&md->spi->dev, "%s\n", __func__);

       if (display->hw_config.set_backlight == NULL)
               return -ENODEV;

       mutex_lock(&md->mutex);

       if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
                       dev->props.power == FB_BLANK_UNBLANK)
               level = dev->props.brightness;
       else
               level = 0;

       r = 0;
       if (md->has_bc)
               acx565akm_set_brightness(md, level);
       else
               if (display->hw_config.set_backlight != NULL)
                       r = display->hw_config.set_backlight(display, level);
       else
               r = -ENODEV;

       mutex_unlock(&md->mutex);

       return r;
}

static int acx565akm_bl_get_intensity(struct backlight_device *dev)
{
       struct acx565akm_device *md = dev_get_drvdata(&dev->dev);
       struct omap_display *display = md->display;

       dev_dbg(&dev->dev, "%s\n", __func__);

       if (md->has_bc && display->hw_config.set_backlight == NULL)
               return -ENODEV;

       if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
                       dev->props.power == FB_BLANK_UNBLANK) {
               if (md->has_bc)
                       return acx565akm_get_actual_brightness(md);
               else
                       return dev->props.brightness;
       }

       return 0;
}

static struct backlight_ops acx565akm_bl_ops = {
       .get_brightness = acx565akm_bl_get_intensity,
       .update_status  = acx565akm_bl_update_status,
};

static const char *cabc_modes[] = {
       "off",          /* used also always when CABC is not supported */
       "ui",
       "still-image",
       "moving-image",
};

static ssize_t show_cabc_mode(struct device *dev,
               struct device_attribute *attr,
               char *buf)
{
       struct acx565akm_device *md = dev_get_drvdata(dev);
       const char *mode_str;
       int mode;
       int len;

       if (!md->has_cabc)
               mode = 0;
       else
               mode = get_cabc_mode(md);
       mode_str = "unknown";
       if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
               mode_str = cabc_modes[mode];
       len = snprintf(buf, PAGE_SIZE, "%s\n", mode_str);

       return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t store_cabc_mode(struct device *dev,
               struct device_attribute *attr,
               const char *buf, size_t count)
{
       struct acx565akm_device *md = dev_get_drvdata(dev);
       int i;

       for (i = 0; i < ARRAY_SIZE(cabc_modes); i++) {
               const char *mode_str = cabc_modes[i];
               int cmp_len = strlen(mode_str);

               if (count > 0 && buf[count - 1] == '\n')
                       count--;
               if (count != cmp_len)
                       continue;

               if (strncmp(buf, mode_str, cmp_len) == 0)
                       break;
       }

       if (i == ARRAY_SIZE(cabc_modes))
               return -EINVAL;

       if (!md->has_cabc && i != 0)
               return -EINVAL;

       mutex_lock(&md->mutex);
       set_cabc_mode(md, i);
       mutex_unlock(&md->mutex);

       return count;
}

static ssize_t show_cabc_available_modes(struct device *dev,
               struct device_attribute *attr,
               char *buf)
{
       struct acx565akm_device *md = dev_get_drvdata(dev);
       int len;
       int i;

       if (!md->has_cabc)
               return snprintf(buf, PAGE_SIZE, "%s\n", cabc_modes[0]);

       for (i = 0, len = 0;
            len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
               len += snprintf(&buf[len], PAGE_SIZE - len, "%s%s%s",
                       i ? " " : "", cabc_modes[i],
                       i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

       return len < PAGE_SIZE ? len : PAGE_SIZE - 1;
}

static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR,
               show_cabc_mode, store_cabc_mode);
static DEVICE_ATTR(cabc_available_modes, S_IRUGO,
               show_cabc_available_modes, NULL);

static struct attribute *bldev_attrs[] = {
       &dev_attr_cabc_mode.attr,
       &dev_attr_cabc_available_modes.attr,
       NULL,
};

static struct attribute_group bldev_attr_group = {
       .attrs = bldev_attrs,
};

static int acx565akm_panel_init(struct omap_display *display)
{
       struct omap_panel *panel = display->panel;
       struct acx565akm_panel_data *panel_data = display->hw_config.panel_data;
       struct acx565akm_device *md = (struct acx565akm_device *)panel->priv;

       struct backlight_device *bldev;
       int brightness;
       int max_brightness;
       int r;

       dev_dbg(&md->spi->dev, "%s\n", __func__);

       if (!panel_data) {
               dev_err(&md->spi->dev, "no panel data\n");
               return -ENODEV;
       }

       mutex_init(&md->mutex);
       md->display = display;

       if (display->hw_config.panel_enable)
               display->hw_config.panel_enable(display);

       md->enabled = panel_enabled(md);

       r = panel_detect(md);
       if (r) {
               if (!md->enabled && display->hw_config.panel_disable)
                       display->hw_config.panel_disable(display);
               mutex_unlock(&md->mutex);
               return r;
       }

       if (!panel_data->bc_connected) {
               md->has_bc = 0;
               md->has_cabc = 0;
       }

#if 0
       acx565akm_set_mode(display, panel->timings.x_res, panel->timings.y_res,
                         panel->bpp);
#endif

       if (!md->enabled)
               display->hw_config.panel_disable(display);

       bldev = backlight_device_register("acx565akm", &md->spi->dev,
                       md, &acx565akm_bl_ops);
       md->bl_dev = bldev;

       if (md->has_cabc) {
               r = sysfs_create_group(&bldev->dev.kobj, &bldev_attr_group);
               if (r) {
                       dev_err(&bldev->dev, "failed to create sysfs files\n");
                       backlight_device_unregister(bldev);
                       return r;
               }
       }

       bldev->props.fb_blank = FB_BLANK_UNBLANK;
       bldev->props.power = FB_BLANK_UNBLANK;

       if (md->has_bc)
               max_brightness = 255;
       else
               max_brightness = display->hw_config.max_backlight_level;

       if (md->has_bc)
               brightness = acx565akm_get_actual_brightness(md);
       else {
               if (display->hw_config.get_backlight != NULL)
                       brightness = display->hw_config.get_backlight(display);
               else
                       brightness = 0;
       }

       bldev->props.max_brightness = max_brightness;
       bldev->props.brightness = brightness;
       acx565akm_bl_update_status(bldev);

       return 0;
}

static struct omap_panel acx565akm_panel = {
       .name           = "panel-acx565akm",
       .init           = acx565akm_panel_init,
       .suspend        = acx565akm_panel_suspend,
       .resume         = acx565akm_panel_resume,
       .enable         = acx565akm_panel_enable,
       .disable        = acx565akm_panel_disable,

       .timings = {
               .x_res = 800,
               .y_res = 480,

               .pixel_clock    = 24000,

               .hsw            = 4,
               .hfp            = 16,
               .hbp            = 12,

               .vsw            = 3,
               .vfp            = 3,
               .vbp            = 3,
       },

       .config         = OMAP_DSS_LCD_TFT,

       .recommended_bpp = 16,

       /*
        * supported modes: 12bpp(444), 16bpp(565), 18bpp(666),  24bpp(888)
        * resolutions.
        */
};

static int acx565akm_spi_probe(struct spi_device *spi)
{
       struct acx565akm_device *md;

       dev_dbg(&md->spi->dev, "%s\n", __func__);

       md = kzalloc(sizeof(*md), GFP_KERNEL);
       if (md == NULL) {
               dev_err(&spi->dev, "out of memory\n");
               return -ENOMEM;
       }

       spi->mode = SPI_MODE_3;
       md->spi = spi;
       dev_set_drvdata(&spi->dev, md);
       md->panel = acx565akm_panel;
       acx565akm_panel.priv = md;

       omap_dss_register_panel(&acx565akm_panel);

       return 0;
}

static int acx565akm_spi_remove(struct spi_device *spi)
{
       struct acx565akm_device *md = dev_get_drvdata(&spi->dev);

       dev_dbg(&md->spi->dev, "%s\n", __func__);

       sysfs_remove_group(&md->bl_dev->dev.kobj, &bldev_attr_group);
       backlight_device_unregister(md->bl_dev);
       omap_dss_unregister_panel(&acx565akm_panel);

       kfree(md);

       return 0;
}

static struct spi_driver acx565akm_spi_driver = {
       .driver = {
               .name   = "acx565akm",
               .bus    = &spi_bus_type,
               .owner  = THIS_MODULE,
       },
       .probe  = acx565akm_spi_probe,
       .remove = __devexit_p(acx565akm_spi_remove),
};

static int __init acx565akm_init(void)
{
       return spi_register_driver(&acx565akm_spi_driver);
}

static void __exit acx565akm_exit(void)
{
       spi_unregister_driver(&acx565akm_spi_driver);
}

module_init(acx565akm_init);
module_exit(acx565akm_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@nokia.com>");
MODULE_DESCRIPTION("acx565akm LCD Driver");
MODULE_LICENSE("GPL");
