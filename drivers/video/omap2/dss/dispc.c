/*
 * linux/drivers/video/omap2/dss/dispc.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "DISPC"

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <mach/sram.h>
#include <mach/board.h>
#include <mach/clock.h>

#include <mach/display.h>

#include "dss.h"

/* DISPC */
#define DISPC_BASE			0x48050400

#define DISPC_SZ_REGS			SZ_1K

struct dispc_reg { u16 idx; };

#define DISPC_REG(idx)			((const struct dispc_reg) { idx })

/* DISPC common */
#define DISPC_REVISION			DISPC_REG(0x0000)
#define DISPC_SYSCONFIG			DISPC_REG(0x0010)
#define DISPC_SYSSTATUS			DISPC_REG(0x0014)
#define DISPC_IRQSTATUS			DISPC_REG(0x0018)
#define DISPC_IRQENABLE			DISPC_REG(0x001C)
#define DISPC_CONTROL			DISPC_REG(0x0040)
#define DISPC_CONFIG			DISPC_REG(0x0044)
#define DISPC_CAPABLE			DISPC_REG(0x0048)
#define DISPC_DEFAULT_COLOR0		DISPC_REG(0x004C)
#define DISPC_DEFAULT_COLOR1		DISPC_REG(0x0050)
#define DISPC_TRANS_COLOR0		DISPC_REG(0x0054)
#define DISPC_TRANS_COLOR1		DISPC_REG(0x0058)
#define DISPC_LINE_STATUS		DISPC_REG(0x005C)
#define DISPC_LINE_NUMBER		DISPC_REG(0x0060)
#define DISPC_TIMING_H			DISPC_REG(0x0064)
#define DISPC_TIMING_V			DISPC_REG(0x0068)
#define DISPC_POL_FREQ			DISPC_REG(0x006C)
#define DISPC_DIVISOR			DISPC_REG(0x0070)
#define DISPC_GLOBAL_ALPHA		DISPC_REG(0x0074)
#define DISPC_SIZE_DIG			DISPC_REG(0x0078)
#define DISPC_SIZE_LCD			DISPC_REG(0x007C)

/* DISPC GFX plane */
#define DISPC_GFX_BA0			DISPC_REG(0x0080)
#define DISPC_GFX_BA1			DISPC_REG(0x0084)
#define DISPC_GFX_POSITION		DISPC_REG(0x0088)
#define DISPC_GFX_SIZE			DISPC_REG(0x008C)
#define DISPC_GFX_ATTRIBUTES		DISPC_REG(0x00A0)
#define DISPC_GFX_FIFO_THRESHOLD	DISPC_REG(0x00A4)
#define DISPC_GFX_FIFO_SIZE_STATUS	DISPC_REG(0x00A8)
#define DISPC_GFX_ROW_INC		DISPC_REG(0x00AC)
#define DISPC_GFX_PIXEL_INC		DISPC_REG(0x00B0)
#define DISPC_GFX_WINDOW_SKIP		DISPC_REG(0x00B4)
#define DISPC_GFX_TABLE_BA		DISPC_REG(0x00B8)

#define DISPC_DATA_CYCLE1		DISPC_REG(0x01D4)
#define DISPC_DATA_CYCLE2		DISPC_REG(0x01D8)
#define DISPC_DATA_CYCLE3		DISPC_REG(0x01DC)

#define DISPC_CPR_COEF_R		DISPC_REG(0x0220)
#define DISPC_CPR_COEF_G		DISPC_REG(0x0224)
#define DISPC_CPR_COEF_B		DISPC_REG(0x0228)

#define DISPC_GFX_PRELOAD		DISPC_REG(0x022C)

/* DISPC Video plane, n = 0 for VID1 and n = 1 for VID2 */
#define DISPC_VID_REG(n, idx)		DISPC_REG(0x00BC + (n)*0x90 + idx)

#define DISPC_VID_BA0(n)		DISPC_VID_REG(n, 0x0000)
#define DISPC_VID_BA1(n)		DISPC_VID_REG(n, 0x0004)
#define DISPC_VID_POSITION(n)		DISPC_VID_REG(n, 0x0008)
#define DISPC_VID_SIZE(n)		DISPC_VID_REG(n, 0x000C)
#define DISPC_VID_ATTRIBUTES(n)		DISPC_VID_REG(n, 0x0010)
#define DISPC_VID_FIFO_THRESHOLD(n)	DISPC_VID_REG(n, 0x0014)
#define DISPC_VID_FIFO_SIZE_STATUS(n)	DISPC_VID_REG(n, 0x0018)
#define DISPC_VID_ROW_INC(n)		DISPC_VID_REG(n, 0x001C)
#define DISPC_VID_PIXEL_INC(n)		DISPC_VID_REG(n, 0x0020)
#define DISPC_VID_FIR(n)		DISPC_VID_REG(n, 0x0024)
#define DISPC_VID_PICTURE_SIZE(n)	DISPC_VID_REG(n, 0x0028)
#define DISPC_VID_ACCU0(n)		DISPC_VID_REG(n, 0x002C)
#define DISPC_VID_ACCU1(n)		DISPC_VID_REG(n, 0x0030)

/* coef index i = {0, 1, 2, 3, 4, 5, 6, 7} */
#define DISPC_VID_FIR_COEF_H(n, i)	DISPC_REG(0x00F0 + (n)*0x90 + (i)*0x8)
/* coef index i = {0, 1, 2, 3, 4, 5, 6, 7} */
#define DISPC_VID_FIR_COEF_HV(n, i)	DISPC_REG(0x00F4 + (n)*0x90 + (i)*0x8)
/* coef index i = {0, 1, 2, 3, 4} */
#define DISPC_VID_CONV_COEF(n, i)	DISPC_REG(0x0130 + (n)*0x90 + (i)*0x4)
/* coef index i = {0, 1, 2, 3, 4, 5, 6, 7} */
#define DISPC_VID_FIR_COEF_V(n, i)	DISPC_REG(0x01E0 + (n)*0x20 + (i)*0x4)

#define DISPC_VID_PRELOAD(n)		DISPC_REG(0x230 + (n)*0x04)


#define DISPC_IRQ_MASK_ERROR            (DISPC_IRQ_GFX_FIFO_UNDERFLOW | \
					 DISPC_IRQ_OCP_ERR | \
					 DISPC_IRQ_VID1_FIFO_UNDERFLOW | \
					 DISPC_IRQ_VID2_FIFO_UNDERFLOW | \
					 DISPC_IRQ_SYNC_LOST | \
					 DISPC_IRQ_SYNC_LOST_DIGIT)

#define DISPC_MAX_NR_ISRS		8


#define LPR_GFX_FIFO_HIGH_THRES		0xB9C
#define LPR_GFX_FIFO_LOW_THRES		0x7F8
#define DISPC_VID_ATTRIBUTES_ENABLE	(1 << 0)
#define DSS_CONTROL_APLL_CLK		1
static int lpr_enabled;
static int gfx_in_use;

struct omap_dispc_isr_data {
	omap_dispc_isr_t	isr;
	void			*arg;
	u32			mask;
};

#define REG_GET(idx, start, end) \
	FLD_GET(dispc_read_reg(idx), start, end)

#define REG_FLD_MOD(idx, val, start, end)				\
	dispc_write_reg(idx, FLD_MOD(dispc_read_reg(idx), val, start, end))

static const struct dispc_reg dispc_reg_att[] = { DISPC_GFX_ATTRIBUTES,
	DISPC_VID_ATTRIBUTES(0),
	DISPC_VID_ATTRIBUTES(1) };

static struct {
	void __iomem    *base;

	struct clk	*dpll4_m4_ck;

	unsigned long	cache_req_pck;
	unsigned long	cache_prate;
	struct dispc_clock_info cache_cinfo;

	u32	fifo_size[3];

	spinlock_t irq_lock;
	spinlock_t lpr_lock;
	u32 irq_error_mask;
	struct omap_dispc_isr_data registered_isr[DISPC_MAX_NR_ISRS];
	u32 error_irqs;
	struct work_struct error_work;

	u32		ctx[DISPC_SZ_REGS / sizeof(u32)];
} dispc;

static void _omap_dispc_set_irqs(void);
static int dispc_is_vdma_req(u8 rotation, enum omap_color_mode color_mode);

static inline void dispc_write_reg(const struct dispc_reg idx, u32 val)
{
	__raw_writel(val, dispc.base + idx.idx);
}

static inline u32 dispc_read_reg(const struct dispc_reg idx)
{
	return __raw_readl(dispc.base + idx.idx);
}

#define SR(reg) \
	dispc.ctx[(DISPC_##reg).idx / sizeof(u32)] = dispc_read_reg(DISPC_##reg)
#define RR(reg) \
	dispc_write_reg(DISPC_##reg, dispc.ctx[(DISPC_##reg).idx / sizeof(u32)])

void dispc_save_context(void)
{
	if (cpu_is_omap24xx())
		return;

	SR(SYSCONFIG);
	SR(IRQENABLE);
	SR(CONTROL);
	SR(CONFIG);
	SR(DEFAULT_COLOR0);
	SR(DEFAULT_COLOR1);
	SR(TRANS_COLOR0);
	SR(TRANS_COLOR1);
	SR(LINE_NUMBER);
	SR(TIMING_H);
	SR(TIMING_V);
	SR(POL_FREQ);
	SR(DIVISOR);
	SR(GLOBAL_ALPHA);
	SR(SIZE_DIG);
	SR(SIZE_LCD);

	SR(GFX_BA0);
	SR(GFX_BA1);
	SR(GFX_POSITION);
	SR(GFX_SIZE);
	SR(GFX_ATTRIBUTES);
	SR(GFX_FIFO_THRESHOLD);
	SR(GFX_ROW_INC);
	SR(GFX_PIXEL_INC);
	SR(GFX_WINDOW_SKIP);
	SR(GFX_TABLE_BA);

	SR(DATA_CYCLE1);
	SR(DATA_CYCLE2);
	SR(DATA_CYCLE3);

	SR(CPR_COEF_R);
	SR(CPR_COEF_G);
	SR(CPR_COEF_B);

	SR(GFX_PRELOAD);

	/* VID1 */
	SR(VID_BA0(0));
	SR(VID_BA1(0));
	SR(VID_POSITION(0));
	SR(VID_SIZE(0));
	SR(VID_ATTRIBUTES(0));
	SR(VID_FIFO_THRESHOLD(0));
	SR(VID_ROW_INC(0));
	SR(VID_PIXEL_INC(0));
	SR(VID_FIR(0));
	SR(VID_PICTURE_SIZE(0));
	SR(VID_ACCU0(0));
	SR(VID_ACCU1(0));

	SR(VID_FIR_COEF_H(0, 0));
	SR(VID_FIR_COEF_H(0, 1));
	SR(VID_FIR_COEF_H(0, 2));
	SR(VID_FIR_COEF_H(0, 3));
	SR(VID_FIR_COEF_H(0, 4));
	SR(VID_FIR_COEF_H(0, 5));
	SR(VID_FIR_COEF_H(0, 6));
	SR(VID_FIR_COEF_H(0, 7));

	SR(VID_FIR_COEF_HV(0, 0));
	SR(VID_FIR_COEF_HV(0, 1));
	SR(VID_FIR_COEF_HV(0, 2));
	SR(VID_FIR_COEF_HV(0, 3));
	SR(VID_FIR_COEF_HV(0, 4));
	SR(VID_FIR_COEF_HV(0, 5));
	SR(VID_FIR_COEF_HV(0, 6));
	SR(VID_FIR_COEF_HV(0, 7));

	SR(VID_CONV_COEF(0, 0));
	SR(VID_CONV_COEF(0, 1));
	SR(VID_CONV_COEF(0, 2));
	SR(VID_CONV_COEF(0, 3));
	SR(VID_CONV_COEF(0, 4));

	SR(VID_FIR_COEF_V(0, 0));
	SR(VID_FIR_COEF_V(0, 1));
	SR(VID_FIR_COEF_V(0, 2));
	SR(VID_FIR_COEF_V(0, 3));
	SR(VID_FIR_COEF_V(0, 4));
	SR(VID_FIR_COEF_V(0, 5));
	SR(VID_FIR_COEF_V(0, 6));
	SR(VID_FIR_COEF_V(0, 7));

	SR(VID_PRELOAD(0));

	/* VID2 */
	SR(VID_BA0(1));
	SR(VID_BA1(1));
	SR(VID_POSITION(1));
	SR(VID_SIZE(1));
	SR(VID_ATTRIBUTES(1));
	SR(VID_FIFO_THRESHOLD(1));
	SR(VID_ROW_INC(1));
	SR(VID_PIXEL_INC(1));
	SR(VID_FIR(1));
	SR(VID_PICTURE_SIZE(1));
	SR(VID_ACCU0(1));
	SR(VID_ACCU1(1));

	SR(VID_FIR_COEF_H(1, 0));
	SR(VID_FIR_COEF_H(1, 1));
	SR(VID_FIR_COEF_H(1, 2));
	SR(VID_FIR_COEF_H(1, 3));
	SR(VID_FIR_COEF_H(1, 4));
	SR(VID_FIR_COEF_H(1, 5));
	SR(VID_FIR_COEF_H(1, 6));
	SR(VID_FIR_COEF_H(1, 7));

	SR(VID_FIR_COEF_HV(1, 0));
	SR(VID_FIR_COEF_HV(1, 1));
	SR(VID_FIR_COEF_HV(1, 2));
	SR(VID_FIR_COEF_HV(1, 3));
	SR(VID_FIR_COEF_HV(1, 4));
	SR(VID_FIR_COEF_HV(1, 5));
	SR(VID_FIR_COEF_HV(1, 6));
	SR(VID_FIR_COEF_HV(1, 7));

	SR(VID_CONV_COEF(1, 0));
	SR(VID_CONV_COEF(1, 1));
	SR(VID_CONV_COEF(1, 2));
	SR(VID_CONV_COEF(1, 3));
	SR(VID_CONV_COEF(1, 4));

	SR(VID_FIR_COEF_V(1, 0));
	SR(VID_FIR_COEF_V(1, 1));
	SR(VID_FIR_COEF_V(1, 2));
	SR(VID_FIR_COEF_V(1, 3));
	SR(VID_FIR_COEF_V(1, 4));
	SR(VID_FIR_COEF_V(1, 5));
	SR(VID_FIR_COEF_V(1, 6));
	SR(VID_FIR_COEF_V(1, 7));

	SR(VID_PRELOAD(1));
}

void dispc_restore_context(void)
{
	RR(SYSCONFIG);
	RR(IRQENABLE);
	/*RR(CONTROL);*/
	RR(CONFIG);
	RR(DEFAULT_COLOR0);
	RR(DEFAULT_COLOR1);
	RR(TRANS_COLOR0);
	RR(TRANS_COLOR1);
	RR(LINE_NUMBER);
	RR(TIMING_H);
	RR(TIMING_V);
	RR(POL_FREQ);
	RR(DIVISOR);
	RR(GLOBAL_ALPHA);
	RR(SIZE_DIG);
	RR(SIZE_LCD);

	RR(GFX_BA0);
	RR(GFX_BA1);
	RR(GFX_POSITION);
	RR(GFX_SIZE);
	RR(GFX_ATTRIBUTES);
	RR(GFX_FIFO_THRESHOLD);
	RR(GFX_ROW_INC);
	RR(GFX_PIXEL_INC);
	RR(GFX_WINDOW_SKIP);
	RR(GFX_TABLE_BA);

	RR(DATA_CYCLE1);
	RR(DATA_CYCLE2);
	RR(DATA_CYCLE3);

	RR(CPR_COEF_R);
	RR(CPR_COEF_G);
	RR(CPR_COEF_B);

	RR(GFX_PRELOAD);

	/* VID1 */
	RR(VID_BA0(0));
	RR(VID_BA1(0));
	RR(VID_POSITION(0));
	RR(VID_SIZE(0));
	RR(VID_ATTRIBUTES(0));
	RR(VID_FIFO_THRESHOLD(0));
	RR(VID_ROW_INC(0));
	RR(VID_PIXEL_INC(0));
	RR(VID_FIR(0));
	RR(VID_PICTURE_SIZE(0));
	RR(VID_ACCU0(0));
	RR(VID_ACCU1(0));

	RR(VID_FIR_COEF_H(0, 0));
	RR(VID_FIR_COEF_H(0, 1));
	RR(VID_FIR_COEF_H(0, 2));
	RR(VID_FIR_COEF_H(0, 3));
	RR(VID_FIR_COEF_H(0, 4));
	RR(VID_FIR_COEF_H(0, 5));
	RR(VID_FIR_COEF_H(0, 6));
	RR(VID_FIR_COEF_H(0, 7));

	RR(VID_FIR_COEF_HV(0, 0));
	RR(VID_FIR_COEF_HV(0, 1));
	RR(VID_FIR_COEF_HV(0, 2));
	RR(VID_FIR_COEF_HV(0, 3));
	RR(VID_FIR_COEF_HV(0, 4));
	RR(VID_FIR_COEF_HV(0, 5));
	RR(VID_FIR_COEF_HV(0, 6));
	RR(VID_FIR_COEF_HV(0, 7));

	RR(VID_CONV_COEF(0, 0));
	RR(VID_CONV_COEF(0, 1));
	RR(VID_CONV_COEF(0, 2));
	RR(VID_CONV_COEF(0, 3));
	RR(VID_CONV_COEF(0, 4));

	RR(VID_FIR_COEF_V(0, 0));
	RR(VID_FIR_COEF_V(0, 1));
	RR(VID_FIR_COEF_V(0, 2));
	RR(VID_FIR_COEF_V(0, 3));
	RR(VID_FIR_COEF_V(0, 4));
	RR(VID_FIR_COEF_V(0, 5));
	RR(VID_FIR_COEF_V(0, 6));
	RR(VID_FIR_COEF_V(0, 7));

	RR(VID_PRELOAD(0));

	/* VID2 */
	RR(VID_BA0(1));
	RR(VID_BA1(1));
	RR(VID_POSITION(1));
	RR(VID_SIZE(1));
	RR(VID_ATTRIBUTES(1));
	RR(VID_FIFO_THRESHOLD(1));
	RR(VID_ROW_INC(1));
	RR(VID_PIXEL_INC(1));
	RR(VID_FIR(1));
	RR(VID_PICTURE_SIZE(1));
	RR(VID_ACCU0(1));
	RR(VID_ACCU1(1));

	RR(VID_FIR_COEF_H(1, 0));
	RR(VID_FIR_COEF_H(1, 1));
	RR(VID_FIR_COEF_H(1, 2));
	RR(VID_FIR_COEF_H(1, 3));
	RR(VID_FIR_COEF_H(1, 4));
	RR(VID_FIR_COEF_H(1, 5));
	RR(VID_FIR_COEF_H(1, 6));
	RR(VID_FIR_COEF_H(1, 7));

	RR(VID_FIR_COEF_HV(1, 0));
	RR(VID_FIR_COEF_HV(1, 1));
	RR(VID_FIR_COEF_HV(1, 2));
	RR(VID_FIR_COEF_HV(1, 3));
	RR(VID_FIR_COEF_HV(1, 4));
	RR(VID_FIR_COEF_HV(1, 5));
	RR(VID_FIR_COEF_HV(1, 6));
	RR(VID_FIR_COEF_HV(1, 7));

	RR(VID_CONV_COEF(1, 0));
	RR(VID_CONV_COEF(1, 1));
	RR(VID_CONV_COEF(1, 2));
	RR(VID_CONV_COEF(1, 3));
	RR(VID_CONV_COEF(1, 4));

	RR(VID_FIR_COEF_V(1, 0));
	RR(VID_FIR_COEF_V(1, 1));
	RR(VID_FIR_COEF_V(1, 2));
	RR(VID_FIR_COEF_V(1, 3));
	RR(VID_FIR_COEF_V(1, 4));
	RR(VID_FIR_COEF_V(1, 5));
	RR(VID_FIR_COEF_V(1, 6));
	RR(VID_FIR_COEF_V(1, 7));

	RR(VID_PRELOAD(1));

	/* enable last, because LCD & DIGIT enable are here */
	RR(CONTROL);
}

#undef SR
#undef RR

static inline void enable_clocks(bool enable)
{
	if (enable)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);
	else
		dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
}

bool dispc_go_busy(enum omap_channel channel)
{
	int bit;

	if (channel == OMAP_DSS_CHANNEL_LCD)
		bit = 5; /* GOLCD */
	else
		bit = 6; /* GODIGIT */

	return REG_GET(DISPC_CONTROL, bit, bit) == 1;
}

void dispc_go(enum omap_channel channel)
{
	int bit;

	enable_clocks(1);

	if (channel == OMAP_DSS_CHANNEL_LCD)
		bit = 0; /* LCDENABLE */
	else
		bit = 1; /* DIGITALENABLE */

	/* if the channel is not enabled, we don't need GO */
	if (REG_GET(DISPC_CONTROL, bit, bit) == 0)
		goto end;

	if (channel == OMAP_DSS_CHANNEL_LCD)
		bit = 5; /* GOLCD */
	else
		bit = 6; /* GODIGIT */

	if (REG_GET(DISPC_CONTROL, bit, bit) == 1) {
		DSSERR("GO bit not down for channel %d\n", channel);
		goto end;
	}

	DSSDBG("GO %s\n", channel == OMAP_DSS_CHANNEL_LCD ? "LCD" : "DIGIT");

	REG_FLD_MOD(DISPC_CONTROL, 1, bit, bit);
end:
	enable_clocks(0);
}

static void _dispc_write_firh_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_VID_FIR_COEF_H(plane-1, reg), value);
}

static void _dispc_write_firhv_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_VID_FIR_COEF_HV(plane-1, reg), value);
}

static void _dispc_write_firv_reg(enum omap_plane plane, int reg, u32 value)
{
	BUG_ON(plane == OMAP_DSS_GFX);

	dispc_write_reg(DISPC_VID_FIR_COEF_V(plane-1, reg), value);
}

/* New coefficients Begin */
static void _dispc_set_scale_coef(enum omap_plane plane,
			u16 orig_width, u16 out_width,
			u16 orig_height, u16 out_height,
			int five_taps, bool vdma,
			bool flicker_filter, int flicker_filter_level)
{
	unsigned long reg, mval, rem_ratio, default_mval;
	short int vc_3tap[3][8];
	short int vc[5][8];
	short int hc[5][8];
	int i = 0;

	/* 3-taps vertical filter coefficients */
	/* Downscaling matrix, if image width > 1024 */
	const static short int filter_coeff_vc_d[3][8] = {
		{36,	40,	45,	50,	18, 	23,	27, 	31},
		{56,	57,	56,	55,	55,	55,	56,	57},
		{36,	31,	27,	23,	55,	50,	45,	40} };

	/* Upscaling matrix, if image width > 1024 */
	const static short int filter_coeff_vc_u[3][8] = {
		{0,	16,	32,	48,	0,	0,	0,	0 },
		{128,	112,	96,	80,	64,	80,	96,	112 },
		{0,	0,	0,	0,	64,	48,	32,	16 } };

	/* 5-taps horizontal filter coefficients */
	/* Downscaling matrix, if image width > 1024 */
	const static short int filter_coeff_hc_d[5][8] = {
		{0,	4,	8,	12,	-9,	-7,	-5,	-2 },
		{36,	40,	44,	48,	17,	22,	27,	 31 },
		{56, 	55,	54,	53,	52,	53,	54,	55 },
		{36,	31,	27,	22,	51,	48,	44,	40 },
		{0,	-2,	-5,	-7,	17,	12,	8,	4 } };

	/* Upscaling matrix, if image width > 1024 */
	const static short int filter_coeff_hc_u[5][8] = {
		{0,	0,	0,	0,	0,	0,	0,	0 },
		{0,	16,	32,	48,	0,	0,	0,	0 },
		{128,	112,	96,	80,	64,	80,	96,	112 },
		{0,	0,	0,	0,	64,	48,	32,	16 },
		{0,	0,	0,	0,	0,	0,	0,	0 } };


	/* Filter coefficients designed for various scaling ratios */
	/* Below Co-effients are defined for 5 taps as
	* [0] = VCC22
	* [1] = VC2
	* [2] = VC1
	* [3] = VC0
	* [4] = VC00
	*/

	const static short int filter_coeff_M8[5][8] = {
		{17,	14,	5,	-6,	2,	9,	15,	19},
		{-20,	-4,	17,	47,	-18,	-27,	-30,	-27},
		{134,	127,	121,	105,	81,	105,	121,	127},
		{-20,	-27,	-30,	-27,	81,	47,	17,	-4},
		{17,	18,	15,	9,	-18,	-6,	5,	13 } };

	const static short int filter_coeff_M9[5][8] = {
		{8,	1,	-9,	-18,	14,	17,	17,	14},
		{-8,	8,	30,	56,	-26,	-30,	-27,	-21},
		{128,	126,	117,	103,	83,	103,	117,	126},
		{-8,	-21,	-27,	-30,	83,	56,	30,	8},
		{8,	14,	17,	17,	-26,	-18,	-9,	1} };

	const static short int filter_coeff_M10[5][8] = {
		{-2,	-10,	-18,	-24,	18,	15,	11,	5},
		{2,	20,	41,	62,	-28,	-27,	-22,	-12},
		{128,	125,	116,	102,	83,	102,	116,	125},
		{2,	-12,	-22,	-27,	83,	62,	41,	20},
		{-2,	5,	11,	15,	-28,	-24,	-18,	-10} };

	const static short int filter_coeff_M11[5][8] = {
		{-12,	-19,	-24,	-27,	14,	9,	3,	-4},
		{12,	30,	49,	67,	-26,	-22,	-15,	-3},
		{128,	124,	115,	101,	83,	101,	115,	124},
		{12,	-3,	-15,	-22,	83,	67,	49,	30},
		{-12,	-4,	3,	9,	-26,	-27,	-24,	-19} };

	const static short int filter_coeff_M12[5][8] = {
		{-19,	-24,	-26,	-25,	6,	1,	-6,	-12},
		{21,	38,	55,	70,	-21,	-16,	-7,	6},
		{124,	120,	112,	98,	82,	98,	112,	120},
		{21,	6,	-7,	-16,	82,	70,	55,	38},
		{-19,	-12,	-6,	1,	-21,	-25,	-26,	-24} };

	const static short int filter_coeff_M13[5][8] = {
		{-22,	-25,	-25,	-22,	0,	-6,	-12,	-18 },
		{27,	43,	58,	71,	-17,	-10,	0,	13 },
		{118,	115,	107,	95,	81,	95,	107,	115 },
		{27,	13,	0,	-10,	81,	71,	58,	43 },
		{-22,	-18,	-12,	-6,	-17,	-22,	-25,	-25} };

	const static short int filter_coeff_M14[5][8] = {
		{-23,	-24,	-22,	-18,	-6,	-11,	-16,	-20 },
		{32,	46,	59,	70,	-11,	-4,	6,	18 },
		{110,	108,	101,	91,	78,	91,	101,	108 },
		{32,	18,	6,	-4,	78,	70,	59,	46 },
		{-23,	-20,	-16,	-11,	-11,	-18,	-22,	-24} };

	const static short int filter_coeff_M16[5][8] = {
		{-20,	-18,	-14,	-9,	-14,	-17,	-19,	-21},
		{37,	48,	58,	66,	-2,	6,	15,	26},
		{94,	93,	88,	82,	73,	82,	88,	93},
		{37,	26,	15,	6,	73,	66,	58,	48},
		{-20,	-21,	-19,	-17,	-2,	-9,	-14,	-18} };

	const static short int filter_coeff_M19[5][8] = {
		{-12,	-8,	-4,	1,	-16,	-16,	-16,	-13},
		{38,	47,	53,	59,	8,	15,	22,	31},
		{76,	72,	73,	69,	64,	69,	73,	72},
		{38,	31,	22,	15,	64,	59,	53,	47},
		{-12,	-14,	-16,	-16,	8,	1,	-4,	-9} };

	const static short int filter_coeff_M22[5][8] = {
		{-6,	-1,	3,	8,	-14,	-13,	-11,	-7},
		{37,	44,	48,	53,	13,	19,	25,	32},
		{66,	61,	63,	61,	58,	61,	63,	61},
		{37,	32,	25,	19,	58,	53,	48,	44},
		{-6,	-8,	-11,	-13,	13,	8,	3,	-2} };

	const static short int filter_coeff_M26[5][8] = {
		{1,	4,	8,	13,	-10,	-8,	-5,	-2},
		{36,	40,	44,	48,	18,	22,	27,	31},
		{54,	55,	54,	53,	51,	53,	54,	55},
		{36,	31,	27,	22,	51,	48,	44,	40},
		{1,	-2,	-5,	-8,	18,	13,	8,	4 } };

	const static short int filter_coeff_M32[5][8] =	{
		{7,	10,	14,	17,	-4,	-1,	1,	4},
		{34,	37,	39,	42,	21,	24,	28,	31},
		{46,	46,	46,	46,	45,	46,	46,	46},
		{34,	31,	27,	24,	45,	42,	39,	37},
		{7,	4,	1,	-1,	21,	17,	14,	10} };

	const static short int coeff_mvals[12] =
		{8, 9, 10, 11, 12, 13, 14, 16, 19, 22, 26, 32};

	/* Select the coefficients based on the ratio - height/vertical */
	if (out_height != 0 && five_taps) {
		if ((out_height != orig_height) || vdma) {
			rem_ratio = 0;
			mval = 8; /* default */
			rem_ratio =  out_height / orig_height ;
			if (rem_ratio > 1) {
				mval = 8;
				DSSDBG("Coefficient class mval = %lu \n", mval);
			} else {
				mval = (8 * orig_height) / out_height;
			}

			/* if flicker filter is ON then calculate
			 * the corresponding vertical coeff table
			*/
			if (flicker_filter) {
				switch (mval) {
				case 8:
				case 9:
				case 10:
				case 11:
				case 12:
				case 13:
				case 14:
					default_mval = mval;
					break;
				case 15:
				case 16:
					default_mval = 16;
					break;
				case 17:
				case 18:
				case 19:
					default_mval = 19;
					break;
				case 20:
				case 21:
				case 22:
					default_mval = 22;
					break;
				case 23:
				case 24:
				case 25:
				case 26:
					default_mval = 26;
					break;
				case 27:
				case 28:
				case 29:
				case 30:
				case 31:
				case 32:
					default_mval = 32;
					break;
				default:
					default_mval = 8;

				}

				for (i = 0; i < sizeof(coeff_mvals); i++) {
					if (coeff_mvals[i] == default_mval)
						break;
				}
				/*get the requested frequency coeff table*/
				mval = coeff_mvals[i + flicker_filter_level];
			}

			DSSDBG("<%s> Applying flicker_filter [%d], "
				"Filter_level [%d], mval [%lu]\n",
				__func__, flicker_filter,
				flicker_filter_level, mval);

			switch (mval) {
			case 8:
				memcpy(vc, filter_coeff_M8, sizeof(vc));
				break;
			case 9:
				memcpy(vc, filter_coeff_M9, sizeof(vc));
				break;
			case 10:
				memcpy(vc, filter_coeff_M10, sizeof(vc));
				break;
			case 11:
				memcpy(vc, filter_coeff_M11, sizeof(vc));
				break;
			case 12:
				memcpy(vc, filter_coeff_M12, sizeof(vc));
				break;
			case 13:
				memcpy(vc, filter_coeff_M13, sizeof(vc));
				break;
			case 14:
				memcpy(vc, filter_coeff_M14, sizeof(vc));
				break;
			case 15:
			case 16:
				memcpy(vc, filter_coeff_M16, sizeof(vc));
				break;
			case 17:
			case 18:
			case 19:
				memcpy(vc, filter_coeff_M19, sizeof(vc));
				break;
			case 20:
			case 21:
			case 22:
				memcpy(vc, filter_coeff_M22, sizeof(vc));
				break;
			case 23:
			case 24:
			case 25:
			case 26:
				memcpy(vc, filter_coeff_M26, sizeof(vc));
				break;
			case 27:
			case 28:
			case 29:
			case 30:
			case 31:
			case 32:
				memcpy(vc, filter_coeff_M32, sizeof(vc));
				break;
			default:
				DSSDBG("Chosing default M val = 8 \n");
				memcpy(vc, filter_coeff_M8, sizeof(vc));
				break;
			};
		}
	} else if (out_height != 0 && !five_taps) {
		if (out_height != orig_height) {
			/* Vertical scaling */
			if (out_height > orig_height) {
				DSSDBG("Vertical upscaling \n");
				memcpy(vc_3tap, filter_coeff_vc_u,
					sizeof(vc_3tap));
			} else {
				DSSDBG("Vertical downscaling \n");
				memcpy(vc_3tap, filter_coeff_vc_d,
					sizeof(vc_3tap));
			}
		}
	}


	/* Select the coefficients based on the ratio - width / horizontal */
	if (out_width != 0 && five_taps) {
		if ((out_width != orig_width) || vdma) {
			rem_ratio = 0;
			mval = 8; /* default */
			rem_ratio =  out_width / orig_width ;
			if (rem_ratio > 1)
				mval = 8;
			else
				mval = (8 * orig_width) / out_width;

			switch (mval) {
			case 8:
				memcpy(hc, filter_coeff_M8, sizeof(hc));
				break;
			case 9:
				memcpy(hc, filter_coeff_M9, sizeof(hc));
				break;
			case 10:
				memcpy(hc, filter_coeff_M10, sizeof(hc));
				break;
			case 11:
				memcpy(hc, filter_coeff_M11, sizeof(hc));
				break;
			case 12:
				memcpy(hc, filter_coeff_M12, sizeof(hc));
				break;
			case 13:
				memcpy(hc, filter_coeff_M13, sizeof(hc));
				break;
			case 14:
				memcpy(hc, filter_coeff_M14, sizeof(hc));
				break;
			case 15:
			case 16:
				memcpy(hc, filter_coeff_M16, sizeof(hc));
				break;
			case 17:
			case 18:
			case 19:
				memcpy(hc, filter_coeff_M19, sizeof(hc));
				break;
			case 20:
			case 21:
			case 22:
				memcpy(hc, filter_coeff_M22, sizeof(hc));
				break;
			case 23:
			case 24:
			case 25:
			case 26:
				memcpy(hc, filter_coeff_M26, sizeof(hc));
				break;
			case 27:
			case 28:
			case 29:
			case 30:
			case 31:
			case 32:
				memcpy(hc, filter_coeff_M32, sizeof(hc));
				break;
			default:
				DSSDBG("Chosing default M val = 8 \n");
				memcpy(hc, filter_coeff_M8, sizeof(hc));
				break;
			};
		}
	} else if (out_width != 0 && !five_taps) {
		if (out_width != orig_width) {
			/* Horizontal scaling */
			if (out_height > orig_height) {
				DSSDBG("Horizontal upscaling \n");
				memcpy(hc, filter_coeff_hc_u, sizeof(hc));
			} else {
				DSSDBG("Horizontal downscaling \n");
				memcpy(hc, filter_coeff_hc_d, sizeof(hc));
			}
		}
	}

	/* Pack the coefficients - use fivetaps for all ratios */
	for (i = 0; i < 8; i++) {
		reg = 0;
		DSSDBG("Phase i = %d \n", (int)i);
		reg = ((hc[1][i] & 0xff) << 24)
			| ((hc[2][i] & 0xff) << 16)
			| ((hc[3][i] & 0xff) << 8)
			| (hc[4][i] & 0xff);
		_dispc_write_firh_reg(plane, i, reg);
		DSSDBG("H coefficients = 0x%x\n", (int)reg);

		if (!five_taps) {
			reg = 0;
			reg = (hc[0][i] & 0xff)
				| ((vc_3tap[2][i] & 0xff) << 8)
				| ((vc_3tap[1][i] & 0xff) << 16)
				| ((vc_3tap[0][i] & 0xff) << 24);
			_dispc_write_firhv_reg(plane, i, reg);
			DSSDBG("HV coefficients = 0x%x\n", (int)reg);

		} else {
			reg = 0;
			reg = (hc[0][i] & 0xff)
				| ((vc[3][i] & 0xff) << 8)
				| ((vc[2][i] & 0xff) << 16)
				| ((vc[1][i] & 0xff) << 24);
			_dispc_write_firhv_reg(plane, i, reg);
			DSSDBG("HV coefficients = 0x%x\n", (int)reg);

			reg = 0;
			reg =  ((vc[0][i] & 0xff) << 8) | (vc[4][i] & 0xff);
			_dispc_write_firv_reg(plane, i, reg);
			DSSDBG("V coefficients = 0x%x\n", (int)reg);
		}
	}
}

static void _dispc_setup_color_conv_coef(void)
{
	const struct color_conv_coef {
		int  ry,  rcr,  rcb,   gy,  gcr,  gcb,   by,  bcr,  bcb;
		int  full_range;
	}  ctbl_bt601_5 = {
		298,  409,    0,  298, -208, -100,  298,    0,  517, 0,
	};

	const struct color_conv_coef *ct;

#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))

	ct = &ctbl_bt601_5;

	dispc_write_reg(DISPC_VID_CONV_COEF(0, 0), CVAL(ct->rcr, ct->ry));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 1), CVAL(ct->gy,	 ct->rcb));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 2), CVAL(ct->gcb, ct->gcr));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 3), CVAL(ct->bcr, ct->by));
	dispc_write_reg(DISPC_VID_CONV_COEF(0, 4), CVAL(0,       ct->bcb));

	dispc_write_reg(DISPC_VID_CONV_COEF(1, 0), CVAL(ct->rcr, ct->ry));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 1), CVAL(ct->gy,	 ct->rcb));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 2), CVAL(ct->gcb, ct->gcr));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 3), CVAL(ct->bcr, ct->by));
	dispc_write_reg(DISPC_VID_CONV_COEF(1, 4), CVAL(0,       ct->bcb));

#undef CVAL

	REG_FLD_MOD(DISPC_VID_ATTRIBUTES(0), ct->full_range, 11, 11);
	REG_FLD_MOD(DISPC_VID_ATTRIBUTES(1), ct->full_range, 11, 11);
}


static void _dispc_set_plane_ba0(enum omap_plane plane, u32 paddr)
{
	const struct dispc_reg ba0_reg[] = { DISPC_GFX_BA0,
		DISPC_VID_BA0(0),
		DISPC_VID_BA0(1) };

	dispc_write_reg(ba0_reg[plane], paddr);
}

static void _dispc_set_plane_ba1(enum omap_plane plane, u32 paddr)
{
	const struct dispc_reg ba1_reg[] = { DISPC_GFX_BA1,
				      DISPC_VID_BA1(0),
				      DISPC_VID_BA1(1) };

	dispc_write_reg(ba1_reg[plane], paddr);
}

static void _dispc_set_plane_pos(enum omap_plane plane, int x, int y)
{
	const struct dispc_reg pos_reg[] = { DISPC_GFX_POSITION,
				      DISPC_VID_POSITION(0),
				      DISPC_VID_POSITION(1) };

	u32 val = FLD_VAL(y, 26, 16) | FLD_VAL(x, 10, 0);
	dispc_write_reg(pos_reg[plane], val);
}

static void _dispc_set_pic_size(enum omap_plane plane, int width, int height)
{
	const struct dispc_reg siz_reg[] = { DISPC_GFX_SIZE,
				      DISPC_VID_PICTURE_SIZE(0),
				      DISPC_VID_PICTURE_SIZE(1) };
	u32 val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(siz_reg[plane], val);
}

static void _dispc_set_vid_size(enum omap_plane plane, int width, int height)
{
	u32 val;
	const struct dispc_reg vsi_reg[] = { DISPC_VID_SIZE(0),
				      DISPC_VID_SIZE(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	dispc_write_reg(vsi_reg[plane-1], val);
}

static void _dispc_set_alpha_blend_attrs(enum omap_plane plane, bool enable)
{
	REG_FLD_MOD(dispc_reg_att[plane], enable ? 1 : 0, 28, 28);
}

static void _dispc_setup_global_alpha(enum omap_plane plane, u8 global_alpha)
{

	BUG_ON(plane == OMAP_DSS_VIDEO1);

	if (plane == OMAP_DSS_GFX)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 7, 0);
	else if (plane == OMAP_DSS_VIDEO2)
		REG_FLD_MOD(DISPC_GLOBAL_ALPHA, global_alpha, 23, 16);
}

static void _dispc_set_pix_inc(enum omap_plane plane, s32 inc)
{
	const struct dispc_reg ri_reg[] = { DISPC_GFX_PIXEL_INC,
				     DISPC_VID_PIXEL_INC(0),
				     DISPC_VID_PIXEL_INC(1) };

	dispc_write_reg(ri_reg[plane], inc);
}

static void _dispc_set_row_inc(enum omap_plane plane, s32 inc)
{
	const struct dispc_reg ri_reg[] = { DISPC_GFX_ROW_INC,
				     DISPC_VID_ROW_INC(0),
				     DISPC_VID_ROW_INC(1) };

	dispc_write_reg(ri_reg[plane], inc);
}

static void _dispc_set_color_mode(enum omap_plane plane,
		enum omap_color_mode color_mode)
{
	u32 m = 0;

	switch (color_mode) {
	case OMAP_DSS_COLOR_CLUT1:
		m = 0x0; break;
	case OMAP_DSS_COLOR_CLUT2:
		m = 0x1; break;
	case OMAP_DSS_COLOR_CLUT4:
		m = 0x2; break;
	case OMAP_DSS_COLOR_CLUT8:
		m = 0x3; break;
	case OMAP_DSS_COLOR_RGB12U:
		m = 0x4; break;
	case OMAP_DSS_COLOR_ARGB16:
		m = 0x5; break;
	case OMAP_DSS_COLOR_RGB16:
		m = 0x6; break;
	case OMAP_DSS_COLOR_RGB24U:
		m = 0x8; break;
	case OMAP_DSS_COLOR_RGB24P:
		m = 0x9; break;
	case OMAP_DSS_COLOR_YUV2:
		m = 0xa; break;
	case OMAP_DSS_COLOR_UYVY:
		m = 0xb; break;
	case OMAP_DSS_COLOR_ARGB32:
		m = 0xc; break;
	case OMAP_DSS_COLOR_RGBA32:
		m = 0xd; break;
	case OMAP_DSS_COLOR_RGBX32:
		m = 0xe; break;
	default:
		BUG(); break;
	}

	REG_FLD_MOD(dispc_reg_att[plane], m, 4, 1);
}

static void _dispc_set_channel_out(enum omap_plane plane,
		enum omap_channel channel)
{
	int shift;
	u32 val;

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 8;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
		shift = 16;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(dispc_reg_att[plane]);
	val = FLD_MOD(val, channel, shift, shift);
	dispc_write_reg(dispc_reg_att[plane], val);
}

void dispc_set_burst_size(enum omap_plane plane,
		enum omap_burst_size burst_size)
{
	int shift;
	u32 val;

	enable_clocks(1);

	switch (plane) {
	case OMAP_DSS_GFX:
		shift = 6;
		break;
	case OMAP_DSS_VIDEO1:
	case OMAP_DSS_VIDEO2:
		shift = 14;
		break;
	default:
		BUG();
		return;
	}

	val = dispc_read_reg(dispc_reg_att[plane]);
	val = FLD_MOD(val, burst_size, shift+1, shift);
	dispc_write_reg(dispc_reg_att[plane], val);

	enable_clocks(0);
}

static void _dispc_set_vid_color_conv(enum omap_plane plane, bool enable)
{
	u32 val;

	BUG_ON(plane == OMAP_DSS_GFX);

	val = dispc_read_reg(dispc_reg_att[plane]);
	val = FLD_MOD(val, enable, 9, 9);
	dispc_write_reg(dispc_reg_att[plane], val);
}

void dispc_enable_replication(enum omap_plane plane, bool enable)
{
	int bit;

	if (plane == OMAP_DSS_GFX)
		bit = 5;
	else
		bit = 10;

	enable_clocks(1);
	REG_FLD_MOD(dispc_reg_att[plane], enable, bit, bit);
	enable_clocks(0);
}

void dispc_set_lcd_size(u16 width, u16 height)
{
	u32 val;
	BUG_ON((width > (1 << 11)) || (height > (1 << 11)));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	enable_clocks(1);
	dispc_write_reg(DISPC_SIZE_LCD, val);
	enable_clocks(0);
}

void dispc_set_digit_size(u16 width, u16 height)
{
	u32 val;
	BUG_ON((width > (1 << 11)) || (height > (1 << 11)));
	val = FLD_VAL(height - 1, 26, 16) | FLD_VAL(width - 1, 10, 0);
	enable_clocks(1);
	dispc_write_reg(DISPC_SIZE_DIG, val);
	enable_clocks(0);
}

static void dispc_read_plane_fifo_sizes(void)
{
	const struct dispc_reg fsz_reg[] = { DISPC_GFX_FIFO_SIZE_STATUS,
				      DISPC_VID_FIFO_SIZE_STATUS(0),
				      DISPC_VID_FIFO_SIZE_STATUS(1) };
	u32 size;
	int plane;

	enable_clocks(1);

	for (plane = 0; plane < ARRAY_SIZE(dispc.fifo_size); ++plane) {
		if (cpu_is_omap24xx())
			size = FLD_GET(dispc_read_reg(fsz_reg[plane]), 8, 0);
		else if (cpu_is_omap34xx())
			size = FLD_GET(dispc_read_reg(fsz_reg[plane]), 10, 0);
		else
			BUG();

		dispc.fifo_size[plane] = size;
	}

	enable_clocks(0);
}

u32 dispc_get_plane_fifo_size(enum omap_plane plane)
{
	return dispc.fifo_size[plane];
}

void dispc_setup_plane_fifo(enum omap_plane plane, u32 low, u32 high)
{
	const struct dispc_reg ftrs_reg[] = { DISPC_GFX_FIFO_THRESHOLD,
				       DISPC_VID_FIFO_THRESHOLD(0),
				       DISPC_VID_FIFO_THRESHOLD(1) };
	enable_clocks(1);

	DSSDBG("fifo(%d) low/high old %u/%u, new %u/%u\n",
			plane,
			REG_GET(ftrs_reg[plane], 11, 0),
			REG_GET(ftrs_reg[plane], 27, 16),
			low, high);

	if (cpu_is_omap24xx())
		dispc_write_reg(ftrs_reg[plane],
				FLD_VAL(high, 24, 16) | FLD_VAL(low, 8, 0));
	else
		dispc_write_reg(ftrs_reg[plane],
				FLD_VAL(high, 27, 16) | FLD_VAL(low, 11, 0));

	enable_clocks(0);
}

void dispc_enable_fifomerge(bool enable)
{
	enable_clocks(1);

	DSSDBG("FIFO merge %s\n", enable ? "enabled" : "disabled");
	REG_FLD_MOD(DISPC_CONFIG, enable ? 1 : 0, 14, 14);

	enable_clocks(0);
}

static void _dispc_set_fir(enum omap_plane plane, int hinc, int vinc)
{
	u32 val;
	const struct dispc_reg fir_reg[] = { DISPC_VID_FIR(0),
				      DISPC_VID_FIR(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	if (cpu_is_omap24xx())
		val = FLD_VAL(vinc, 27, 16) | FLD_VAL(hinc, 11, 0);
	else
		val = FLD_VAL(vinc, 28, 16) | FLD_VAL(hinc, 12, 0);
	dispc_write_reg(fir_reg[plane-1], val);
}

static void _dispc_set_vid_accu0(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	const struct dispc_reg ac0_reg[] = { DISPC_VID_ACCU0(0),
				      DISPC_VID_ACCU0(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(vaccu, 25, 16) | FLD_VAL(haccu, 9, 0);
	dispc_write_reg(ac0_reg[plane-1], val);
}

static void _dispc_set_vid_accu1(enum omap_plane plane, int haccu, int vaccu)
{
	u32 val;
	const struct dispc_reg ac1_reg[] = { DISPC_VID_ACCU1(0),
				      DISPC_VID_ACCU1(1) };

	BUG_ON(plane == OMAP_DSS_GFX);

	val = FLD_VAL(vaccu, 25, 16) | FLD_VAL(haccu, 9, 0);
	dispc_write_reg(ac1_reg[plane-1], val);
}

static void _dispc_set_vdma_attrs(enum omap_plane plane, bool enable)
{
	REG_FLD_MOD(dispc_reg_att[plane], enable ? 1 : 0, 20, 20);
}

static void _dispc_set_scaling(enum omap_plane plane,
		u16 orig_width, u16 orig_height,
		u16 out_width, u16 out_height,
		bool ilace, bool five_taps,
		bool fieldmode, bool vdma,
		bool flicker_filter, int flicker_filter_level)
{
	int fir_hinc;
	int fir_vinc;
	int hscaleup, vscaleup;
	int accu0 = 0;
	int accu1 = 0;
	u32 l;

	BUG_ON(plane == OMAP_DSS_GFX);

	hscaleup = orig_width <= out_width;
	vscaleup = orig_height <= out_height;

	/* New filter coefficients integration */
	_dispc_set_scale_coef(plane, orig_width, out_width,
			orig_height, out_height, five_taps, vdma,
			flicker_filter, flicker_filter_level);

	if (!orig_width || (!vdma && (orig_width == out_width)))
		fir_hinc = 0;
	else
		fir_hinc = 1024 * orig_width / out_width;

	if (!orig_height || (!vdma && (orig_height == out_height)))
		fir_vinc = 0;
	else
		fir_vinc = 1024 * orig_height / out_height;

	_dispc_set_fir(plane, fir_hinc, fir_vinc);

	l = dispc_read_reg(dispc_reg_att[plane]);
	l &= ~((0x0f << 5) | (0x3 << 21));

	l |= fir_hinc ? (1 << 5) : 0;
	l |= fir_vinc ? (1 << 6) : 0;

	l |= hscaleup ? 0 : (1 << 7);
	l |= vscaleup ? 0 : (1 << 8);

	l |= five_taps ? (1 << 21) : 0;
	l |= five_taps ? (1 << 22) : 0;

	dispc_write_reg(dispc_reg_att[plane], l);

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	if (ilace && !fieldmode) {
		accu1 = 0;
		accu0 = (fir_vinc / 2) & 0x3ff;
		if (accu0 >= 1024/2) {
			accu1 = 1024/2;
			accu0 -= accu1;
		}
	}

	_dispc_set_vid_accu0(plane, 0, accu0);
	_dispc_set_vid_accu1(plane, 0, accu1);
}

static void _dispc_set_rotation_attrs(enum omap_plane plane, u8 rotation,
		bool mirroring, enum omap_color_mode color_mode)
{
	if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY) {
		int vidrot = 0;

		if (mirroring) {
			switch (rotation) {
			case 0:
				vidrot = 2;
				break;
			case 1:
				vidrot = 3;
				break;
			case 2:
				vidrot = 0;
				break;
			case 3:
				vidrot = 1;
				break;
			}
		} else {
			switch (rotation) {
			case 0:
				vidrot = 0;
				break;
			case 1:
				vidrot = 3;
				break;
			case 2:
				vidrot = 2;
				break;
			case 3:
				vidrot = 1;
				break;
			}
		}

		REG_FLD_MOD(dispc_reg_att[plane], vidrot, 13, 12);

		if (!dispc_is_vdma_req(rotation, color_mode) &&
			(rotation == 1 || rotation == 3))
			REG_FLD_MOD(dispc_reg_att[plane], 0x1, 18, 18);
		else
			REG_FLD_MOD(dispc_reg_att[plane], 0x0, 18, 18);

	} else {
		REG_FLD_MOD(dispc_reg_att[plane], 0, 13, 12);
		REG_FLD_MOD(dispc_reg_att[plane], 0, 18, 18);
	}
}

static s32 pixinc(int pixels, u8 ps)
{
	if (pixels == 1)
		return 1;
	else if (pixels > 1)
		return 1 + (pixels - 1) * ps;
	else if (pixels < 0)
		return 1 - (-pixels + 1) * ps;
	else
		BUG();
}

static void calc_vrfb_rotation_offset(u8 rotation, bool mirror,
		u16 screen_width,
		u16 width, u16 height,
		enum omap_color_mode color_mode, bool fieldmode,
		unsigned int field_offset,
		unsigned *offset0, unsigned *offset1,
		s32 *row_inc, s32 *pix_inc)
{
	u8 ps;

	switch (color_mode) {
	case OMAP_DSS_COLOR_RGB16:
	case OMAP_DSS_COLOR_ARGB16:
		ps = 2;
		break;

	case OMAP_DSS_COLOR_RGB24P:
		ps = 3;
		break;

	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		ps = 4;
		break;

	default:
		BUG();
		return;
	}

	DSSDBG("calc_rot(%d): scrw %d, %dx%d\n", rotation, screen_width,
			width, height);

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	switch (rotation + mirror * 4) {
	case 0:
	case 2:
		/*
		 * If the pixel format is YUV or UYVY divide the width
		 * of the image by 2 for 0 and 180 degree rotation.
		 */
		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY)
			width = width >> 1;
	case 1:
	case 3:
		*offset1 = 0;
		if (field_offset)
			*offset0 = field_offset * screen_width * ps;
		else
			*offset0 = 0;

		*row_inc = pixinc(1 + (screen_width - width) +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	case 4:
	case 6:
		/* If the pixel format is YUV or UYVY divide the width
		 * of the image by 2  for 0 degree and 180 degree
		 */
		if (color_mode == OMAP_DSS_COLOR_YUV2 ||
			color_mode == OMAP_DSS_COLOR_UYVY)
			width = width >> 1;
	case 5:
	case 7:
		*offset1 = 0;
		if (field_offset)
			*offset0 = field_offset * screen_width * ps;
		else
			*offset0 = 0;
		*row_inc = pixinc(1 - (screen_width + width) -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	default:
		BUG();
	}
}

static void calc_dma_rotation_offset(u8 rotation, bool mirror,
		u16 screen_width,
		u16 width, u16 height,
		enum omap_color_mode color_mode, bool fieldmode,
		unsigned int field_offset,
		unsigned *offset0, unsigned *offset1,
		s32 *row_inc, s32 *pix_inc)
{
	u8 ps;
	u16 fbw, fbh;

	switch (color_mode) {
	case OMAP_DSS_COLOR_RGB16:
	case OMAP_DSS_COLOR_ARGB16:
		ps = 2;
		break;

	case OMAP_DSS_COLOR_RGB24P:
		ps = 3;
		break;

	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
		ps = 4;
		break;

	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		ps = 2;
		break;
	default:
		BUG();
		return;
	}

	DSSDBG("calc_rot(%d): scrw %d, %dx%d\n", rotation, screen_width,
			width, height);

	/* width & height are overlay sizes, convert to fb sizes */

	if (rotation == 0 || rotation == 2) {
		fbw = width;
		fbh = height;
	} else {
		fbw = height;
		fbh = width;
	}

	/*
	 * field 0 = even field = bottom field
	 * field 1 = odd field = top field
	 */
	switch (rotation + mirror * 4) {
	case 0:
		*offset1 = 0;
		if (field_offset)
			*offset0 = *offset1 + field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(1 + (screen_width - fbw) +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;
	case 1:
		*offset1 = screen_width * (fbh - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 + field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * (fbh - 1) + 1 +
				(fieldmode ? 1 : 0), ps);
		*pix_inc = pixinc(-screen_width, ps);
		break;
	case 2:
		*offset1 = (screen_width * (fbh - 1) + fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-1 -
				(screen_width - fbw) -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(-1, ps);
		break;
	case 3:
		*offset1 = (fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-screen_width * (fbh - 1) - 1 -
				(fieldmode ? 1 : 0), ps);
		*pix_inc = pixinc(screen_width, ps);
		break;

	/* mirroring */
	case 0 + 4:
		*offset1 = (fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 + field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * 2 - 1 +
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(-1, ps);
		break;

	case 1 + 4:
		*offset1 = 0;
		if (field_offset)
			*offset0 = *offset1 + field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(-screen_width * (fbh - 1) + 1 +
				(fieldmode ? 1 : 0),
				ps);
		*pix_inc = pixinc(screen_width, ps);
		break;

	case 2 + 4:
		*offset1 = screen_width * (fbh - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * screen_width * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(1 - screen_width * 2 -
				(fieldmode ? screen_width : 0),
				ps);
		*pix_inc = pixinc(1, ps);
		break;

	case 3 + 4:
		*offset1 = (screen_width * (fbh - 1) + fbw - 1) * ps;
		if (field_offset)
			*offset0 = *offset1 - field_offset * ps;
		else
			*offset0 = *offset1;
		*row_inc = pixinc(screen_width * (fbh - 1) - 1 -
				(fieldmode ? 1 : 0),
				ps);
		*pix_inc = pixinc(-screen_width, ps);
		break;

	default:
		BUG();
	}
}

static unsigned long calc_fclk_five_taps(u16 width, u16 height,
		u16 out_width, u16 out_height, enum omap_color_mode color_mode)
{
	u32 fclk = 0;
	/* FIXME venc pclk? */
	u64 tmp, pclk = dispc_pclk_rate();

	if (height > out_height) {
		/* FIXME get real display PPL */
		unsigned int ppl = 800;

		tmp = pclk * height * out_width;
		do_div(tmp, 2 * out_height * ppl);
		fclk = tmp;

		if (height > 2 * out_height && ppl != out_width) {
			tmp = pclk * (height - 2 * out_height) * out_width;
			do_div(tmp, 2 * out_height * (ppl - out_width));
			fclk = max(fclk, (u32) tmp);
		}
	}

	if (width > out_width) {
		tmp = pclk * width;
		do_div(tmp, out_width);
		fclk = max(fclk, (u32) tmp);

		if (color_mode == OMAP_DSS_COLOR_RGB24U)
			fclk <<= 1;
	}

	return fclk;
}

static unsigned long calc_fclk(u16 width, u16 height,
		u16 out_width, u16 out_height)
{
	unsigned int hf, vf;

	/*
	 * FIXME how to determine the 'A' factor
	 * for the no downscaling case ?
	 */

	if (width > 3 * out_width)
		hf = 4;
	else if (width > 2 * out_width)
		hf = 3;
	else if (width > out_width)
		hf = 2;
	else
		hf = 1;

	if (height > out_height)
		vf = 2;
	else
		vf = 1;

	/* FIXME venc pclk? */
	return dispc_pclk_rate() * vf * hf;
}

static int dispc_is_vdma_req(u8 rotation, enum omap_color_mode color_mode)
{
	/* TODO: VDMA support for RGB16 mode */
	if (cpu_is_omap3630())
		if ((color_mode == OMAP_DSS_COLOR_YUV2) ||
			(color_mode == OMAP_DSS_COLOR_UYVY))
			if ((rotation == 1) || (rotation == 3))
				return true;
	return false;
}

void dispc_set_channel_out(enum omap_plane plane, enum omap_channel channel_out)
{
	enable_clocks(1);
	_dispc_set_channel_out(plane, channel_out);
	enable_clocks(0);
}

static int _dispc_setup_plane(enum omap_plane plane,
		u32 paddr, u16 screen_width,
		u16 pos_x, u16 pos_y,
		u16 width, u16 height,
		u16 out_width, u16 out_height,
		enum omap_color_mode color_mode,
		bool ilace,
		enum omap_dss_rotation_type rotation_type,
		u8 rotation, int mirror,
		u8 global_alpha,
		u8 pre_alpha_mult,
		bool flicker_filter, int flicker_filter_level)
{
	const int maxdownscale = (cpu_is_omap34xx() ||
				  cpu_is_omap3630()) ? 4 : 2;
	bool five_taps = 1;
	bool fieldmode = 0;
	int cconv = 0;
	unsigned offset0, offset1;
	s32 row_inc;
	s32 pix_inc;
	u16 frame_height = height;
	unsigned int field_offset = 0;

	if (paddr == 0)
		return -EINVAL;

	if (ilace && (height == out_height) && !flicker_filter)
		fieldmode = 1;

	if (ilace) {
		if (fieldmode)
			height /= 2;
		pos_y /= 2;
		out_height /= 2;

		DSSDBG("adjusting for ilace: height %d, pos_y %d, "
				"out_height %d\n",
				height, pos_y, out_height);
	}

	if (plane == OMAP_DSS_GFX) {
		if (width != out_width || height != out_height)
			return -EINVAL;

		switch (color_mode) {
		case OMAP_DSS_COLOR_ARGB16:
		case OMAP_DSS_COLOR_RGB16:
		case OMAP_DSS_COLOR_RGB24P:
		case OMAP_DSS_COLOR_RGB24U:
		case OMAP_DSS_COLOR_ARGB32:
		case OMAP_DSS_COLOR_RGBA32:
		case OMAP_DSS_COLOR_RGBX32:
			break;

		default:
			return -EINVAL;
		}
	} else {
		/* video plane */

		unsigned long fclk = 0;

		if (out_width < width / maxdownscale ||
		   out_width > width * 8)
			return -EINVAL;

		if (out_height < height / maxdownscale ||
		   out_height > height * 8)
			return -EINVAL;

		switch (color_mode) {
		case OMAP_DSS_COLOR_RGB16:
		case OMAP_DSS_COLOR_RGB24P:
		case OMAP_DSS_COLOR_RGB24U:
		case OMAP_DSS_COLOR_RGBX32:
			break;

		case OMAP_DSS_COLOR_ARGB16:
		case OMAP_DSS_COLOR_ARGB32:
		case OMAP_DSS_COLOR_RGBA32:
			if (plane == OMAP_DSS_VIDEO1)
				return -EINVAL;
			break;

		case OMAP_DSS_COLOR_YUV2:
		case OMAP_DSS_COLOR_UYVY:
			cconv = 1;
			break;

		default:
			return -EINVAL;
		}


		if (!five_taps) {
			fclk = calc_fclk(width, height,
					out_width, out_height);

			/* Try 5-tap filter if 3-tap fclk is too high */
			if (cpu_is_omap34xx() && height > out_height &&
					fclk > dispc_fclk_rate())
				five_taps = true;
			if (dispc_is_vdma_req(rotation, color_mode))
				five_taps = true;
		}

		if (width > (2048 >> five_taps))
			five_taps = 0;

		if (five_taps)
			fclk = calc_fclk_five_taps(width, height,
					out_width, out_height, color_mode);

		DSSDBG("required fclk rate = %lu Hz\n", fclk);
		DSSDBG("current fclk rate = %lu Hz\n", dispc_fclk_rate());

		if (fclk > dispc_fclk_rate())
			return -EINVAL;
	}

	if (ilace && !fieldmode) {
		/*
		 * when downscaling the bottom field may have to start several
		 * source lines below the top field. Unfortunately ACCUI
		 * registers will only hold the fractional part of the offset
		 * so the integer part must be added to the base address of the
		 * bottom field.
		 */
		if (!height || height == out_height)
			field_offset = 0;
		else
			field_offset = height / out_height / 2;
	}

	/* Fields are independent but interleaved in memory. */
	if (fieldmode)
		field_offset = 1;

	if (rotation_type == OMAP_DSS_ROT_DMA)
		calc_dma_rotation_offset(rotation, mirror,
				screen_width, width, frame_height, color_mode,
				fieldmode, field_offset,
				&offset0, &offset1, &row_inc, &pix_inc);
	else
		calc_vrfb_rotation_offset(rotation, mirror,
				screen_width, width, frame_height, color_mode,
				fieldmode, field_offset,
				&offset0, &offset1, &row_inc, &pix_inc);

	DSSDBG("offset0 %u, offset1 %u, row_inc %d, pix_inc %d\n",
			offset0, offset1, row_inc, pix_inc);

	_dispc_set_color_mode(plane, color_mode);

	_dispc_set_plane_ba0(plane, paddr + offset0);
	_dispc_set_plane_ba1(plane, paddr + offset1);

	_dispc_set_row_inc(plane, row_inc);
	_dispc_set_pix_inc(plane, pix_inc);

	DSSDBG("%d,%d %dx%d -> %dx%d\n", pos_x, pos_y, width, height,
			out_width, out_height);

	_dispc_set_plane_pos(plane, pos_x, pos_y);

	_dispc_set_pic_size(plane, width, height);

	if (plane != OMAP_DSS_GFX) {
		if (dispc_is_vdma_req(rotation, color_mode)) {
			_dispc_set_scaling(plane, width, height,
				out_width, out_height,
				ilace, five_taps, fieldmode, 1,
				flicker_filter, flicker_filter_level);
			_dispc_set_vdma_attrs(plane, 1);
		} else {
			_dispc_set_scaling(plane, width, height,
				out_width, out_height,
				ilace, five_taps, fieldmode, 0,
				flicker_filter, flicker_filter_level);
			_dispc_set_vdma_attrs(plane, 0);
		}
		_dispc_set_vid_size(plane, out_width, out_height);
		_dispc_set_vid_color_conv(plane, cconv);
	}

	_dispc_set_rotation_attrs(plane, rotation, mirror, color_mode);

	if (cpu_is_omap3630() && (plane != OMAP_DSS_VIDEO1))
		_dispc_set_alpha_blend_attrs(plane, pre_alpha_mult);

	if (plane != OMAP_DSS_VIDEO1)
		_dispc_setup_global_alpha(plane, global_alpha);

	return 0;
}

static void _dispc_enable_plane(enum omap_plane plane, bool enable)
{
	REG_FLD_MOD(dispc_reg_att[plane], enable ? 1 : 0, 0, 0);
}

static void dispc_disable_isr(void *data, u32 mask)
{
	struct completion *compl = data;
	complete(compl);
}

static void _enable_lcd_out(bool enable)
{
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 0, 0);
}

void dispc_enable_lcd_out(bool enable)
{
	struct completion frame_done_completion;
	bool is_on;
	int r;

	enable_clocks(1);

	/* When we disable LCD output, we need to wait until frame is done.
	 * Otherwise the DSS is still working, and turning off the clocks
	 * prevents DSS from going to OFF mode */
	is_on = REG_GET(DISPC_CONTROL, 0, 0);

	if (!enable && is_on) {
		init_completion(&frame_done_completion);

		r = omap_dispc_register_isr(dispc_disable_isr,
				&frame_done_completion,
				DISPC_IRQ_FRAMEDONE);

		if (r)
			DSSERR("failed to register FRAMEDONE isr\n");
	}

	_enable_lcd_out(enable);

	if (!enable && is_on) {
		if (!wait_for_completion_timeout(&frame_done_completion,
					msecs_to_jiffies(100)))
			DSSERR("timeout waiting for FRAME DONE\n");

		r = omap_dispc_unregister_isr(dispc_disable_isr,
				&frame_done_completion,
				DISPC_IRQ_FRAMEDONE);

		if (r)
			DSSERR("failed to unregister FRAMEDONE isr\n");
	}

	enable_clocks(0);
}

static void _enable_digit_out(bool enable)
{
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 1, 1);
}

void dispc_enable_digit_out(bool enable)
{
	struct completion frame_done_completion;
	int r;

	enable_clocks(1);

	if (REG_GET(DISPC_CONTROL, 1, 1) == enable) {
		enable_clocks(0);
		return;
	}

	if (enable) {
		unsigned long flags;
		/* When we enable digit output, we'll get an extra digit
		 * sync lost interrupt, that we need to ignore */
		spin_lock_irqsave(&dispc.irq_lock, flags);
		dispc.irq_error_mask &= ~DISPC_IRQ_SYNC_LOST_DIGIT;
		_omap_dispc_set_irqs();
		spin_unlock_irqrestore(&dispc.irq_lock, flags);
	}

	/* When we disable digit output, we need to wait until fields are done.
	 * Otherwise the DSS is still working, and turning off the clocks
	 * prevents DSS from going to OFF mode. And when enabling, we need to
	 * wait for the extra sync losts */
	init_completion(&frame_done_completion);

	r = omap_dispc_register_isr(dispc_disable_isr, &frame_done_completion,
			DISPC_IRQ_EVSYNC_EVEN | DISPC_IRQ_EVSYNC_ODD);
	if (r)
		DSSERR("failed to register EVSYNC isr\n");

	_enable_digit_out(enable);

	/* XXX I understand from TRM that we should only wait for the
	 * current field to complete. But it seems we have to wait
	 * for both fields */
	if (!wait_for_completion_timeout(&frame_done_completion,
				msecs_to_jiffies(100)))
		DSSERR("timeout waiting for EVSYNC\n");

	if (!wait_for_completion_timeout(&frame_done_completion,
				msecs_to_jiffies(100)))
		DSSERR("timeout waiting for EVSYNC\n");

	r = omap_dispc_unregister_isr(dispc_disable_isr,
			&frame_done_completion,
			DISPC_IRQ_EVSYNC_EVEN | DISPC_IRQ_EVSYNC_ODD);
	if (r)
		DSSERR("failed to unregister EVSYNC isr\n");

	if (enable) {
		unsigned long flags;
		spin_lock_irqsave(&dispc.irq_lock, flags);
		dispc.irq_error_mask = DISPC_IRQ_MASK_ERROR;
		dispc_write_reg(DISPC_IRQSTATUS, DISPC_IRQ_SYNC_LOST_DIGIT);
		_omap_dispc_set_irqs();
		spin_unlock_irqrestore(&dispc.irq_lock, flags);
	}

	enable_clocks(0);
}

void dispc_lcd_enable_signal_polarity(bool act_high)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, act_high ? 1 : 0, 29, 29);
	enable_clocks(0);
}

void dispc_lcd_enable_signal(bool enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 28, 28);
	enable_clocks(0);
}

void dispc_pck_free_enable(bool enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, enable ? 1 : 0, 27, 27);
	enable_clocks(0);
}

void dispc_enable_fifohandcheck(bool enable)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONFIG, enable ? 1 : 0, 16, 16);
	enable_clocks(0);
}


void dispc_set_lcd_display_type(enum omap_lcd_display_type type)
{
	int mode;

	switch (type) {
	case OMAP_DSS_LCD_DISPLAY_STN:
		mode = 0;
		break;

	case OMAP_DSS_LCD_DISPLAY_TFT:
		mode = 1;
		break;

	default:
		BUG();
		return;
	}

	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, mode, 3, 3);
	enable_clocks(0);
}

void dispc_set_loadmode(enum omap_dss_load_mode mode)
{
	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONFIG, mode, 2, 1);
	enable_clocks(0);
}


void dispc_set_default_color(enum omap_channel channel, u32 color)
{
	const struct dispc_reg def_reg[] = { DISPC_DEFAULT_COLOR0,
				DISPC_DEFAULT_COLOR1 };

	enable_clocks(1);
	dispc_write_reg(def_reg[channel], color);
	enable_clocks(0);
}

u32 dispc_get_default_color(enum omap_channel channel)
{
	const struct dispc_reg def_reg[] = { DISPC_DEFAULT_COLOR0,
				DISPC_DEFAULT_COLOR1 };
	u32 l;

	BUG_ON(channel != OMAP_DSS_CHANNEL_DIGIT &&
	       channel != OMAP_DSS_CHANNEL_LCD);

	enable_clocks(1);
	l = dispc_read_reg(def_reg[channel]);
	enable_clocks(0);

	return l;
}

void dispc_set_trans_key(enum omap_channel ch,
		enum omap_dss_trans_key_type type,
		u32 trans_key)
{
	const struct dispc_reg tr_reg[] = {
		DISPC_TRANS_COLOR0, DISPC_TRANS_COLOR1 };

	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, type, 11, 11);
	else /* OMAP_DSS_CHANNEL_DIGIT */
		REG_FLD_MOD(DISPC_CONFIG, type, 13, 13);

	dispc_write_reg(tr_reg[ch], trans_key);
	enable_clocks(0);
}

void dispc_get_trans_key(enum omap_channel ch,
		enum omap_dss_trans_key_type *type,
		u32 *trans_key)
{
	const struct dispc_reg tr_reg[] = {
		DISPC_TRANS_COLOR0, DISPC_TRANS_COLOR1 };

	enable_clocks(1);
	if (type) {
		if (ch == OMAP_DSS_CHANNEL_LCD)
			*type = REG_GET(DISPC_CONFIG, 11, 11);
		else if (ch == OMAP_DSS_CHANNEL_DIGIT)
			*type = REG_GET(DISPC_CONFIG, 13, 13);
		else
			BUG();
	}

	if (trans_key)
		*trans_key = dispc_read_reg(tr_reg[ch]);
	enable_clocks(0);
}

void dispc_enable_trans_key(enum omap_channel ch, bool enable)
{
	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 10, 10);
	else /* OMAP_DSS_CHANNEL_DIGIT */
		REG_FLD_MOD(DISPC_CONFIG, enable, 12, 12);
	enable_clocks(0);
}
void dispc_enable_alpha_blending(enum omap_channel ch, bool enable)
{
	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		REG_FLD_MOD(DISPC_CONFIG, enable, 18, 18);
	else /* OMAP_DSS_CHANNEL_DIGIT */
		REG_FLD_MOD(DISPC_CONFIG, enable, 19, 19);
	enable_clocks(0);
}
bool dispc_alpha_blending_enabled(enum omap_channel ch)
{
	bool enabled;

	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		enabled = REG_GET(DISPC_CONFIG, 18, 18);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		enabled = REG_GET(DISPC_CONFIG, 18, 18);
	else
		BUG();
	enable_clocks(0);

	return enabled;

}


bool dispc_trans_key_enabled(enum omap_channel ch)
{
	bool enabled;

	enable_clocks(1);
	if (ch == OMAP_DSS_CHANNEL_LCD)
		enabled = REG_GET(DISPC_CONFIG, 10, 10);
	else if (ch == OMAP_DSS_CHANNEL_DIGIT)
		enabled = REG_GET(DISPC_CONFIG, 12, 12);
	else
		BUG();
	enable_clocks(0);

	return enabled;
}


void dispc_set_tft_data_lines(u8 data_lines)
{
	int code;

	switch (data_lines) {
	case 12:
		code = 0;
		break;
	case 16:
		code = 1;
		break;
	case 18:
		code = 2;
		break;
	case 24:
		code = 3;
		break;
	default:
		BUG();
		return;
	}

	enable_clocks(1);
	REG_FLD_MOD(DISPC_CONTROL, code, 9, 8);
	enable_clocks(0);
}

void dispc_set_parallel_interface_mode(enum omap_parallel_interface_mode mode)
{
	u32 l;
	int stallmode;
	int gpout0 = 1;
	int gpout1;

	switch (mode) {
	case OMAP_DSS_PARALLELMODE_BYPASS:
		stallmode = 0;
		gpout1 = 1;
		break;

	case OMAP_DSS_PARALLELMODE_RFBI:
		stallmode = 1;
		gpout1 = 0;
		break;

	case OMAP_DSS_PARALLELMODE_DSI:
		stallmode = 1;
		gpout1 = 1;
		break;

	default:
		BUG();
		return;
	}

	enable_clocks(1);

	l = dispc_read_reg(DISPC_CONTROL);

	l = FLD_MOD(l, stallmode, 11, 11);
	l = FLD_MOD(l, gpout0, 15, 15);
	l = FLD_MOD(l, gpout1, 16, 16);

	dispc_write_reg(DISPC_CONTROL, l);

	enable_clocks(0);
}

static bool _dispc_lcd_timings_ok(int hsw, int hfp, int hbp,
		int vsw, int vfp, int vbp)
{
	if (cpu_is_omap24xx() || omap_rev() < OMAP3430_REV_ES3_0) {
		if (hsw < 1 || hsw > 64 ||
				hfp < 1 || hfp > 256 ||
				hbp < 1 || hbp > 256 ||
				vsw < 1 || vsw > 64 ||
				vfp < 0 || vfp > 255 ||
				vbp < 0 || vbp > 255)
			return false;
	} else {
		if (hsw < 1 || hsw > 256 ||
				hfp < 1 || hfp > 4096 ||
				hbp < 1 || hbp > 4096 ||
				vsw < 1 || vsw > 256 ||
				vfp < 0 || vfp > 4095 ||
				vbp < 0 || vbp > 4095)
			return false;
	}

	return true;
}

bool dispc_lcd_timings_ok(struct omap_video_timings *timings)
{
	return _dispc_lcd_timings_ok(timings->hsw, timings->hfp,
			timings->hbp, timings->vsw,
			timings->vfp, timings->vbp);
}

static void _dispc_set_lcd_timings(int hsw, int hfp, int hbp,
				   int vsw, int vfp, int vbp)
{
	u32 timing_h, timing_v;

	if (cpu_is_omap24xx() || omap_rev() < OMAP3430_REV_ES3_0) {
		timing_h = FLD_VAL(hsw-1, 5, 0) | FLD_VAL(hfp-1, 15, 8) |
			FLD_VAL(hbp-1, 27, 20);

		timing_v = FLD_VAL(vsw-1, 5, 0) | FLD_VAL(vfp, 15, 8) |
			FLD_VAL(vbp, 27, 20);
	} else {
		timing_h = FLD_VAL(hsw-1, 7, 0) | FLD_VAL(hfp-1, 19, 8) |
			FLD_VAL(hbp-1, 31, 20);

		timing_v = FLD_VAL(vsw-1, 7, 0) | FLD_VAL(vfp, 19, 8) |
			FLD_VAL(vbp, 31, 20);
	}

	enable_clocks(1);
	dispc_write_reg(DISPC_TIMING_H, timing_h);
	dispc_write_reg(DISPC_TIMING_V, timing_v);
	enable_clocks(0);
}

/* change name to mode? */
void dispc_set_lcd_timings(struct omap_video_timings *timings)
{
	unsigned xtot, ytot;
	unsigned long ht, vt;

	if (!_dispc_lcd_timings_ok(timings->hsw, timings->hfp,
				timings->hbp, timings->vsw,
				timings->vfp, timings->vbp))
		BUG();

	_dispc_set_lcd_timings(timings->hsw, timings->hfp, timings->hbp,
			timings->vsw, timings->vfp, timings->vbp);

	dispc_set_lcd_size(timings->x_res, timings->y_res);

	xtot = timings->x_res + timings->hfp + timings->hsw + timings->hbp;
	ytot = timings->y_res + timings->vfp + timings->vsw + timings->vbp;

	ht = (timings->pixel_clock * 1000) / xtot;
	vt = (timings->pixel_clock * 1000) / xtot / ytot;

	DSSDBG("xres %u yres %u\n", timings->x_res, timings->y_res);
	DSSDBG("pck %u\n", timings->pixel_clock);
	DSSDBG("hsw %d hfp %d hbp %d vsw %d vfp %d vbp %d\n",
			timings->hsw, timings->hfp, timings->hbp,
			timings->vsw, timings->vfp, timings->vbp);

	DSSDBG("hsync %luHz, vsync %luHz\n", ht, vt);
}

void dispc_set_lcd_divisor(u16 lck_div, u16 pck_div)
{
	BUG_ON(lck_div < 1);
	BUG_ON(pck_div < 2);

	enable_clocks(1);
	dispc_write_reg(DISPC_DIVISOR,
			FLD_VAL(lck_div, 23, 16) | FLD_VAL(pck_div, 7, 0));
	enable_clocks(0);
}

static void dispc_get_lcd_divisor(int *lck_div, int *pck_div)
{
	u32 l;
	l = dispc_read_reg(DISPC_DIVISOR);
	*lck_div = FLD_GET(l, 23, 16);
	*pck_div = FLD_GET(l, 7, 0);
}

unsigned long dispc_fclk_rate(void)
{
	unsigned long r = 0;

	if (dss_get_dispc_clk_source() == 0)
		r = dss_clk_get_rate(DSS_CLK_FCK1);
	else
#ifdef CONFIG_OMAP2_DSS_DSI
		r = dsi_get_dsi1_pll_rate();
#else
	BUG();
#endif
	return r;
}

unsigned long dispc_lclk_rate(void)
{
	int lcd;
	unsigned long r;
	u32 l;

	l = dispc_read_reg(DISPC_DIVISOR);

	lcd = FLD_GET(l, 23, 16);

	r = dispc_fclk_rate();

	return r / lcd;
}

unsigned long dispc_pclk_rate(void)
{
	int lcd, pcd;
	unsigned long r;
	u32 l;

	l = dispc_read_reg(DISPC_DIVISOR);

	lcd = FLD_GET(l, 23, 16);
	pcd = FLD_GET(l, 7, 0);

	r = dispc_fclk_rate();

	return r / lcd / pcd;
}

void dispc_dump_clocks(struct seq_file *s)
{
	int lcd, pcd;

	enable_clocks(1);

	dispc_get_lcd_divisor(&lcd, &pcd);

	seq_printf(s, "- dispc -\n");

	seq_printf(s, "dispc fclk source = %s\n",
			dss_get_dispc_clk_source() == 0 ?
			"dss1_alwon_fclk" : "dsi1_pll_fclk");

	seq_printf(s, "pixel clk = %lu / %d / %d = %lu\n",
			dispc_fclk_rate(),
			lcd, pcd,
			dispc_pclk_rate());

	enable_clocks(0);
}

void dispc_dump_regs(struct seq_file *s)
{
#define DUMPREG(r) seq_printf(s, "%-35s %08x\n", #r, dispc_read_reg(r))

	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	DUMPREG(DISPC_REVISION);
	DUMPREG(DISPC_SYSCONFIG);
	DUMPREG(DISPC_SYSSTATUS);
	DUMPREG(DISPC_IRQSTATUS);
	DUMPREG(DISPC_IRQENABLE);
	DUMPREG(DISPC_CONTROL);
	DUMPREG(DISPC_CONFIG);
	DUMPREG(DISPC_CAPABLE);
	DUMPREG(DISPC_DEFAULT_COLOR0);
	DUMPREG(DISPC_DEFAULT_COLOR1);
	DUMPREG(DISPC_TRANS_COLOR0);
	DUMPREG(DISPC_TRANS_COLOR1);
	DUMPREG(DISPC_LINE_STATUS);
	DUMPREG(DISPC_LINE_NUMBER);
	DUMPREG(DISPC_TIMING_H);
	DUMPREG(DISPC_TIMING_V);
	DUMPREG(DISPC_POL_FREQ);
	DUMPREG(DISPC_DIVISOR);
	DUMPREG(DISPC_GLOBAL_ALPHA);
	DUMPREG(DISPC_SIZE_DIG);
	DUMPREG(DISPC_SIZE_LCD);

	DUMPREG(DISPC_GFX_BA0);
	DUMPREG(DISPC_GFX_BA1);
	DUMPREG(DISPC_GFX_POSITION);
	DUMPREG(DISPC_GFX_SIZE);
	DUMPREG(DISPC_GFX_ATTRIBUTES);
	DUMPREG(DISPC_GFX_FIFO_THRESHOLD);
	DUMPREG(DISPC_GFX_FIFO_SIZE_STATUS);
	DUMPREG(DISPC_GFX_ROW_INC);
	DUMPREG(DISPC_GFX_PIXEL_INC);
	DUMPREG(DISPC_GFX_WINDOW_SKIP);
	DUMPREG(DISPC_GFX_TABLE_BA);

	DUMPREG(DISPC_DATA_CYCLE1);
	DUMPREG(DISPC_DATA_CYCLE2);
	DUMPREG(DISPC_DATA_CYCLE3);

	DUMPREG(DISPC_CPR_COEF_R);
	DUMPREG(DISPC_CPR_COEF_G);
	DUMPREG(DISPC_CPR_COEF_B);

	DUMPREG(DISPC_GFX_PRELOAD);

	DUMPREG(DISPC_VID_BA0(0));
	DUMPREG(DISPC_VID_BA1(0));
	DUMPREG(DISPC_VID_POSITION(0));
	DUMPREG(DISPC_VID_SIZE(0));
	DUMPREG(DISPC_VID_ATTRIBUTES(0));
	DUMPREG(DISPC_VID_FIFO_THRESHOLD(0));
	DUMPREG(DISPC_VID_FIFO_SIZE_STATUS(0));
	DUMPREG(DISPC_VID_ROW_INC(0));
	DUMPREG(DISPC_VID_PIXEL_INC(0));
	DUMPREG(DISPC_VID_FIR(0));
	DUMPREG(DISPC_VID_PICTURE_SIZE(0));
	DUMPREG(DISPC_VID_ACCU0(0));
	DUMPREG(DISPC_VID_ACCU1(0));

	DUMPREG(DISPC_VID_BA0(1));
	DUMPREG(DISPC_VID_BA1(1));
	DUMPREG(DISPC_VID_POSITION(1));
	DUMPREG(DISPC_VID_SIZE(1));
	DUMPREG(DISPC_VID_ATTRIBUTES(1));
	DUMPREG(DISPC_VID_FIFO_THRESHOLD(1));
	DUMPREG(DISPC_VID_FIFO_SIZE_STATUS(1));
	DUMPREG(DISPC_VID_ROW_INC(1));
	DUMPREG(DISPC_VID_PIXEL_INC(1));
	DUMPREG(DISPC_VID_FIR(1));
	DUMPREG(DISPC_VID_PICTURE_SIZE(1));
	DUMPREG(DISPC_VID_ACCU0(1));
	DUMPREG(DISPC_VID_ACCU1(1));

	DUMPREG(DISPC_VID_FIR_COEF_H(0, 0));
	DUMPREG(DISPC_VID_FIR_COEF_H(0, 1));
	DUMPREG(DISPC_VID_FIR_COEF_H(0, 2));
	DUMPREG(DISPC_VID_FIR_COEF_H(0, 3));
	DUMPREG(DISPC_VID_FIR_COEF_H(0, 4));
	DUMPREG(DISPC_VID_FIR_COEF_H(0, 5));
	DUMPREG(DISPC_VID_FIR_COEF_H(0, 6));
	DUMPREG(DISPC_VID_FIR_COEF_H(0, 7));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 0));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 1));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 2));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 3));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 4));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 5));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 6));
	DUMPREG(DISPC_VID_FIR_COEF_HV(0, 7));
	DUMPREG(DISPC_VID_CONV_COEF(0, 0));
	DUMPREG(DISPC_VID_CONV_COEF(0, 1));
	DUMPREG(DISPC_VID_CONV_COEF(0, 2));
	DUMPREG(DISPC_VID_CONV_COEF(0, 3));
	DUMPREG(DISPC_VID_CONV_COEF(0, 4));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 0));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 1));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 2));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 3));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 4));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 5));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 6));
	DUMPREG(DISPC_VID_FIR_COEF_V(0, 7));

	DUMPREG(DISPC_VID_FIR_COEF_H(1, 0));
	DUMPREG(DISPC_VID_FIR_COEF_H(1, 1));
	DUMPREG(DISPC_VID_FIR_COEF_H(1, 2));
	DUMPREG(DISPC_VID_FIR_COEF_H(1, 3));
	DUMPREG(DISPC_VID_FIR_COEF_H(1, 4));
	DUMPREG(DISPC_VID_FIR_COEF_H(1, 5));
	DUMPREG(DISPC_VID_FIR_COEF_H(1, 6));
	DUMPREG(DISPC_VID_FIR_COEF_H(1, 7));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 0));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 1));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 2));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 3));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 4));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 5));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 6));
	DUMPREG(DISPC_VID_FIR_COEF_HV(1, 7));
	DUMPREG(DISPC_VID_CONV_COEF(1, 0));
	DUMPREG(DISPC_VID_CONV_COEF(1, 1));
	DUMPREG(DISPC_VID_CONV_COEF(1, 2));
	DUMPREG(DISPC_VID_CONV_COEF(1, 3));
	DUMPREG(DISPC_VID_CONV_COEF(1, 4));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 0));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 1));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 2));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 3));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 4));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 5));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 6));
	DUMPREG(DISPC_VID_FIR_COEF_V(1, 7));

	DUMPREG(DISPC_VID_PRELOAD(0));
	DUMPREG(DISPC_VID_PRELOAD(1));

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
#undef DUMPREG
}

static void _dispc_set_pol_freq(bool onoff, bool rf, bool ieo, bool ipc,
				bool ihs, bool ivs, u8 acbi, u8 acb)
{
	u32 l = 0;

	DSSDBG("onoff %d rf %d ieo %d ipc %d ihs %d ivs %d acbi %d acb %d\n",
			onoff, rf, ieo, ipc, ihs, ivs, acbi, acb);

	l |= FLD_VAL(onoff, 17, 17);
	l |= FLD_VAL(rf, 16, 16);
	l |= FLD_VAL(ieo, 15, 15);
	l |= FLD_VAL(ipc, 14, 14);
	l |= FLD_VAL(ihs, 13, 13);
	l |= FLD_VAL(ivs, 12, 12);
	l |= FLD_VAL(acbi, 11, 8);
	l |= FLD_VAL(acb, 7, 0);

	enable_clocks(1);
	dispc_write_reg(DISPC_POL_FREQ, l);
	enable_clocks(0);
}

void dispc_set_pol_freq(enum omap_panel_config config, u8 acbi, u8 acb)
{
	_dispc_set_pol_freq((config & OMAP_DSS_LCD_ONOFF) != 0,
			(config & OMAP_DSS_LCD_RF) != 0,
			(config & OMAP_DSS_LCD_IEO) != 0,
			(config & OMAP_DSS_LCD_IPC) != 0,
			(config & OMAP_DSS_LCD_IHS) != 0,
			(config & OMAP_DSS_LCD_IVS) != 0,
			acbi, acb);
}

void find_lck_pck_divs(bool is_tft, unsigned long req_pck, unsigned long fck,
		u16 *lck_div, u16 *pck_div)
{
	u16 pcd_min = is_tft ? 2 : 3;
	unsigned long best_pck;
	u16 best_ld, cur_ld;
	u16 best_pd, cur_pd;

	best_pck = 0;
	best_ld = 0;
	best_pd = 0;

	for (cur_ld = 1; cur_ld <= 255; ++cur_ld) {
		unsigned long lck = fck / cur_ld;

		for (cur_pd = pcd_min; cur_pd <= 255; ++cur_pd) {
			unsigned long pck = lck / cur_pd;
			long old_delta = abs(best_pck - req_pck);
			long new_delta = abs(pck - req_pck);

			if (best_pck == 0 || new_delta < old_delta) {
				best_pck = pck;
				best_ld = cur_ld;
				best_pd = cur_pd;

				if (pck == req_pck)
					goto found;
			}

			if (pck < req_pck)
				break;
		}

		if (lck / pcd_min < req_pck)
			break;
	}

found:
	*lck_div = best_ld;
	*pck_div = best_pd;
}

int dispc_calc_clock_div(bool is_tft, unsigned long req_pck,
		struct dispc_clock_info *cinfo)
{
	unsigned long prate;
	struct dispc_clock_info cur, best;
	int match = 0;
	int min_fck_per_pck;
	unsigned long fck_rate = dss_clk_get_rate(DSS_CLK_FCK1);

	if (cpu_is_omap34xx())
		prate = clk_get_rate(clk_get_parent(dispc.dpll4_m4_ck));
	else
		prate = 0;

	if (req_pck == dispc.cache_req_pck &&
			((cpu_is_omap34xx() && prate == dispc.cache_prate) ||
			 dispc.cache_cinfo.fck == fck_rate)) {
		DSSDBG("dispc clock info found from cache.\n");
		*cinfo = dispc.cache_cinfo;
		return 0;
	}

	min_fck_per_pck = CONFIG_OMAP2_DSS_MIN_FCK_PER_PCK;

	if (min_fck_per_pck &&
		req_pck * min_fck_per_pck > DISPC_MAX_FCK) {
		DSSERR("Requested pixel clock not possible with the current "
				"OMAP2_DSS_MIN_FCK_PER_PCK setting. Turning "
				"the constraint off.\n");
		min_fck_per_pck = 0;
	}

retry:
	memset(&cur, 0, sizeof(cur));
	memset(&best, 0, sizeof(best));

	if (cpu_is_omap24xx()) {
		/* XXX can we change the clock on omap2? */
		cur.fck = dss_clk_get_rate(DSS_CLK_FCK1);
		cur.fck_div = 1;

		match = 1;

		find_lck_pck_divs(is_tft, req_pck, cur.fck,
				&cur.lck_div, &cur.pck_div);

		cur.lck = cur.fck / cur.lck_div;
		cur.pck = cur.lck / cur.pck_div;

		best = cur;

		goto found;
	} else if (cpu_is_omap34xx()) {
		if (cpu_is_omap3630())
			cur.fck_div = 32;
		else
			cur.fck_div = 16;

		for ( ; cur.fck_div > 0; --cur.fck_div) {
			if (cpu_is_omap3630())
				cur.fck = prate / cur.fck_div ;
			else
				cur.fck = prate / cur.fck_div * 2;

			if (cur.fck > DISPC_MAX_FCK)
				continue;

			if (min_fck_per_pck &&
					cur.fck < req_pck * min_fck_per_pck)
				continue;

			match = 1;

			find_lck_pck_divs(is_tft, req_pck, cur.fck,
					&cur.lck_div, &cur.pck_div);

			cur.lck = cur.fck / cur.lck_div;
			cur.pck = cur.lck / cur.pck_div;

			if (abs(cur.pck - req_pck) < abs(best.pck - req_pck)) {
				best = cur;

				if (cur.pck == req_pck)
					goto found;
			}
		}
	} else {
		BUG();
	}

found:
	if (!match) {
		if (min_fck_per_pck) {
			DSSERR("Could not find suitable clock settings.\n"
					"Turning FCK/PCK constraint off and"
					"trying again.\n");
			min_fck_per_pck = 0;
			goto retry;
		}

		DSSERR("Could not find suitable clock settings.\n");

		return -EINVAL;
	}

	if (cinfo)
		*cinfo = best;

	dispc.cache_req_pck = req_pck;
	dispc.cache_prate = prate;
	dispc.cache_cinfo = best;

	return 0;
}

int dispc_set_clock_div(struct dispc_clock_info *cinfo)
{
	unsigned long prate;
	int r;

	if (cpu_is_omap34xx()) {
		prate = clk_get_rate(clk_get_parent(dispc.dpll4_m4_ck));
		DSSDBG("dpll4_m4 = %ld\n", prate);
	}

	DSSDBG("fck = %ld (%d)\n", cinfo->fck, cinfo->fck_div);
	DSSDBG("lck = %ld (%d)\n", cinfo->lck, cinfo->lck_div);
	DSSDBG("pck = %ld (%d)\n", cinfo->pck, cinfo->pck_div);

	if (cpu_is_omap34xx()) {
		r = clk_set_rate(dispc.dpll4_m4_ck, prate / cinfo->fck_div);
		if (r)
			return r;
	}

	dispc_set_lcd_divisor(cinfo->lck_div, cinfo->pck_div);

	return 0;
}

int dispc_get_clock_div(struct dispc_clock_info *cinfo)
{
	cinfo->fck = dss_clk_get_rate(DSS_CLK_FCK1);

	if (cpu_is_omap34xx()) {
		unsigned long prate;
		prate = clk_get_rate(clk_get_parent(dispc.dpll4_m4_ck));
		cinfo->fck_div = prate / (cinfo->fck / 2);
	} else {
		cinfo->fck_div = 0;
	}

	cinfo->lck_div = REG_GET(DISPC_DIVISOR, 23, 16);
	cinfo->pck_div = REG_GET(DISPC_DIVISOR, 7, 0);

	cinfo->lck = cinfo->fck / cinfo->lck_div;
	cinfo->pck = cinfo->lck / cinfo->pck_div;

	return 0;
}

/* dispc.irq_lock has to be locked by the caller */
static void _omap_dispc_set_irqs(void)
{
	u32 mask;
	u32 old_mask;
	int i;
	struct omap_dispc_isr_data *isr_data;

	mask = dispc.irq_error_mask;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];

		if (isr_data->isr == NULL)
			continue;

		mask |= isr_data->mask;
	}

	enable_clocks(1);

	old_mask = dispc_read_reg(DISPC_IRQENABLE);
	/* clear the irqstatus for newly enabled irqs */
	dispc_write_reg(DISPC_IRQSTATUS, (mask ^ old_mask) & mask);

	dispc_write_reg(DISPC_IRQENABLE, mask);

	enable_clocks(0);
}

int omap_dispc_register_isr(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int i;
	int ret;
	unsigned long flags;
	struct omap_dispc_isr_data *isr_data;

	if (isr == NULL)
		return -EINVAL;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	/* check for duplicate entry */
	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];
		if (isr_data->isr == isr && isr_data->arg == arg &&
				isr_data->mask == mask) {
			ret = -EINVAL;
			goto err;
		}
	}

	isr_data = NULL;
	ret = -EBUSY;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];

		if (isr_data->isr != NULL)
			continue;

		isr_data->isr = isr;
		isr_data->arg = arg;
		isr_data->mask = mask;
		ret = 0;

		break;
	}

	_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return 0;
err:
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return ret;
}
EXPORT_SYMBOL(omap_dispc_register_isr);

int omap_dispc_unregister_isr(omap_dispc_isr_t isr, void *arg, u32 mask)
{
	int i;
	unsigned long flags;
	int ret = -EINVAL;
	struct omap_dispc_isr_data *isr_data;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &dispc.registered_isr[i];
		if (isr_data->isr != isr || isr_data->arg != arg ||
				isr_data->mask != mask)
			continue;

		/* found the correct isr */

		isr_data->isr = NULL;
		isr_data->arg = NULL;
		isr_data->mask = 0;

		ret = 0;
		break;
	}

	if (ret == 0)
		_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	return ret;
}
EXPORT_SYMBOL(omap_dispc_unregister_isr);

/* This functions adds the OMAPDSS power saving capability by
 * FIFO Merge when display is not in use, thus ehancing the DSS for
 * Screen saver support.
 */
int omap_dispc_lpr_enable(void)
{
	int rc = 0;
	unsigned long flags;
	struct omap_overlay *ovl;
	int v_attr;

	/* Cannot enable lpr if DSS is inactive */
	if (!gfx_in_use) {
		DSSDBG("\nGRX plane in use\n");
		return -1;
	}

	spin_lock_irqsave(&dispc.lpr_lock, flags);

	if (lpr_enabled) {
		DSSDBG("\nLPR is enabled\n");
		goto lpr_out;
	}

	/* Check whether LPR can be triggered
	* - gfx pipeline is routed to LCD
	* - both video pipelines are disabled (this allows FIFOs merge)
	*/

	ovl = omap_dss_get_overlay(0);

	if (ovl->manager->id != OMAP_DSS_CHANNEL_LCD) {
		rc = -1;
		goto lpr_out;
	}

	v_attr = dispc_read_reg(DISPC_VID_ATTRIBUTES(0)) |
			dispc_read_reg(DISPC_VID_ATTRIBUTES(1));

	if (v_attr & DISPC_VID_ATTRIBUTES_ENABLE) {
		rc = -1;
		goto lpr_out;
	}

	dispc_setup_plane_fifo(ovl->id, LPR_GFX_FIFO_LOW_THRES,
				LPR_GFX_FIFO_HIGH_THRES);

	dispc_enable_fifomerge(1);

	/* Enable LCD */

	dispc_enable_lcd_out(1);

	spin_unlock_irqrestore(&dispc.lpr_lock, flags);

	/*Let LPR settings take an effect */

	dispc_go(ovl->manager->id);

	lpr_enabled = 1;

	return 0;

lpr_out:
	spin_unlock_irqrestore(&dispc.lpr_lock, flags);
	return rc;

}
EXPORT_SYMBOL(omap_dispc_lpr_enable);

int omap_dispc_lpr_disable(void)
{
	unsigned long flags;
	struct omap_overlay *ovl;
	u32 fifo_low, fifo_high;
	enum omap_burst_size burst_size;

	if (!gfx_in_use)
		return -1;

	spin_lock_irqsave(&dispc.lpr_lock, flags);

	ovl = omap_dss_get_overlay(0);

	if (!lpr_enabled) {
		spin_unlock_irqrestore(&dispc.lpr_lock, flags);
		return 0;
	}
	/* Disable Fifo Merge */
	dispc_enable_fifomerge(0);

	default_get_overlay_fifo_thresholds(ovl->id, dispc.fifo_size[ovl->id],
				&burst_size, &fifo_low, &fifo_high);

	/* Restore default fifo size*/
	dispc_setup_plane_fifo(ovl->id, fifo_low, fifo_high);

	lpr_enabled = 0;

	spin_unlock_irqrestore(&dispc.lpr_lock, flags);

	/* Let DSS take an effect */
	dispc_go(ovl->manager->id);

	return 0;
}
EXPORT_SYMBOL(omap_dispc_lpr_disable);

#ifdef DEBUG
static void print_irq_status(u32 status)
{
	if ((status & dispc.irq_error_mask) == 0)
		return;

	printk(KERN_DEBUG "DISPC IRQ: 0x%x: ", status);

#define PIS(x) \
	if (status & DISPC_IRQ_##x) \
		printk(#x " ");
	PIS(GFX_FIFO_UNDERFLOW);
	PIS(OCP_ERR);
	PIS(VID1_FIFO_UNDERFLOW);
	PIS(VID2_FIFO_UNDERFLOW);
	PIS(SYNC_LOST);
	PIS(SYNC_LOST_DIGIT);
#undef PIS

	printk("\n");
}
#endif

/* Called from dss.c. Note that we don't touch clocks here,
 * but we presume they are on because we got an IRQ. However,
 * an irq handler may turn the clocks off, so we may not have
 * clock later in the function. */
void dispc_irq_handler(void)
{
	int i;
	u32 irqstatus;
	u32 handledirqs = 0;
	u32 unhandled_errors;
	struct omap_dispc_isr_data *isr_data;
	struct omap_dispc_isr_data registered_isr[DISPC_MAX_NR_ISRS];

	spin_lock(&dispc.irq_lock);

	irqstatus = dispc_read_reg(DISPC_IRQSTATUS);

#ifdef DEBUG
	if (dss_debug)
		print_irq_status(irqstatus);
#endif
	/* Ack the interrupt. Do it here before clocks are possibly turned
	 * off */
	dispc_write_reg(DISPC_IRQSTATUS, irqstatus);

	/* flushed posted write */
	dispc_read_reg(DISPC_IRQSTATUS);

	/* make a copy and unlock, so that isrs can unregister
	 * themselves */
	memcpy(registered_isr, dispc.registered_isr,
			sizeof(registered_isr));

	spin_unlock(&dispc.irq_lock);

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		isr_data = &registered_isr[i];

		if (!isr_data->isr)
			continue;

		if (isr_data->mask & irqstatus) {
			isr_data->isr(isr_data->arg, irqstatus);
			handledirqs |= isr_data->mask;

			if (isr_data->mask & irqstatus & DISPC_IRQ_VSYNC)
				if (dispc_go_busy(OMAP_DSS_CHANNEL_LCD))
					break;
		}
	}

	spin_lock(&dispc.irq_lock);

	unhandled_errors = irqstatus & ~handledirqs & dispc.irq_error_mask;

	if (unhandled_errors) {
		dispc.error_irqs |= unhandled_errors;

		dispc.irq_error_mask &= ~unhandled_errors;
		_omap_dispc_set_irqs();

		schedule_work(&dispc.error_work);
	}

	spin_unlock(&dispc.irq_lock);
}

static void dispc_error_worker(struct work_struct *work)
{
	int i;
	u32 errors;
	unsigned long flags;

	spin_lock_irqsave(&dispc.irq_lock, flags);
	errors = dispc.error_irqs;
	dispc.error_irqs = 0;
	spin_unlock_irqrestore(&dispc.irq_lock, flags);

	if (errors & DISPC_IRQ_GFX_FIFO_UNDERFLOW) {
		DSSERR("GFX_FIFO_UNDERFLOW, disabling GFX\n");
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			struct omap_overlay *ovl;
			ovl = omap_dss_get_overlay(i);

			if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
				continue;

			if (ovl->id == 0) {
				dispc_enable_plane(ovl->id, 0);
				dispc_go(ovl->manager->id);
				mdelay(50);
				break;
			}
		}
	}

	if (errors & DISPC_IRQ_VID1_FIFO_UNDERFLOW) {
		DSSERR("VID1_FIFO_UNDERFLOW, disabling VID1\n");
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			struct omap_overlay *ovl;
			ovl = omap_dss_get_overlay(i);

			if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
				continue;

			if (ovl->id == 1) {
				dispc_enable_plane(ovl->id, 0);
				dispc_go(ovl->manager->id);
				mdelay(50);
				break;
			}
		}
	}

	if (errors & DISPC_IRQ_VID2_FIFO_UNDERFLOW) {
		DSSERR("VID2_FIFO_UNDERFLOW, disabling VID2\n");
		for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
			struct omap_overlay *ovl;
			ovl = omap_dss_get_overlay(i);

			if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
				continue;

			if (ovl->id == 2) {
				dispc_enable_plane(ovl->id, 0);
				dispc_go(ovl->manager->id);
				mdelay(50);
				break;
			}
		}
	}

	if (errors & DISPC_IRQ_SYNC_LOST) {
		struct omap_overlay_manager *manager = NULL;
		bool enable = false;

		DSSERR("SYNC_LOST, disabling LCD\n");

		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);
			if (mgr == NULL)
				break;

			if (mgr->id == OMAP_DSS_CHANNEL_LCD) {
				manager = mgr;
				enable = mgr->device->state ==
						OMAP_DSS_DISPLAY_ACTIVE;
				mgr->device->disable(mgr->device);
				break;
			}
		}

		if (manager) {
			for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
				struct omap_overlay *ovl;
				ovl = omap_dss_get_overlay(i);

				if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
					continue;

				if (ovl->id != 0 && ovl->manager == manager)
					dispc_enable_plane(ovl->id, 0);
			}

			dispc_go(manager->id);
			mdelay(50);
			if (enable)
				manager->device->enable(manager->device);
		}
	}

	if (errors & DISPC_IRQ_SYNC_LOST_DIGIT) {
		struct omap_overlay_manager *manager = NULL;
		bool enable = false;

		DSSERR("SYNC_LOST_DIGIT, disabling TV\n");

		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);
			if (mgr == NULL)
				break;

			if (mgr->id == OMAP_DSS_CHANNEL_DIGIT) {
				manager = mgr;
				enable = mgr->device->state ==
						OMAP_DSS_DISPLAY_ACTIVE;
				mgr->device->disable(mgr->device);
				break;
			}
		}

		if (manager) {
			for (i = 0; i < omap_dss_get_num_overlays(); ++i) {
				struct omap_overlay *ovl;
				ovl = omap_dss_get_overlay(i);

				if (!(ovl->caps & OMAP_DSS_OVL_CAP_DISPC))
					continue;

				if (ovl->id != 0 && ovl->manager == manager)
					dispc_enable_plane(ovl->id, 0);
			}

			dispc_go(manager->id);
			mdelay(50);
			if (enable)
				manager->device->enable(manager->device);
		}
	}

	if (errors & DISPC_IRQ_OCP_ERR) {
		DSSERR("OCP_ERR\n");
		for (i = 0; i < omap_dss_get_num_overlay_managers(); ++i) {
			struct omap_overlay_manager *mgr;
			mgr = omap_dss_get_overlay_manager(i);

			if (mgr->caps & OMAP_DSS_OVL_CAP_DISPC)
				mgr->device->disable(mgr->device);
		}
	}

	spin_lock_irqsave(&dispc.irq_lock, flags);
	dispc.irq_error_mask |= errors;
	_omap_dispc_set_irqs();
	spin_unlock_irqrestore(&dispc.irq_lock, flags);
}

int omap_dispc_wait_for_irq_timeout(u32 irqmask, unsigned long timeout)
{
	void dispc_irq_wait_handler(void *data, u32 mask)
	{
		complete((struct completion *)data);
	}

	int r;
	DECLARE_COMPLETION_ONSTACK(completion);

	r = omap_dispc_register_isr(dispc_irq_wait_handler, &completion,
			irqmask);

	if (r)
		return r;

	timeout = wait_for_completion_timeout(&completion, timeout);

	omap_dispc_unregister_isr(dispc_irq_wait_handler, &completion, irqmask);

	if (timeout == 0)
		return -ETIMEDOUT;

	if (timeout == -ERESTARTSYS)
		return -ERESTARTSYS;

	return 0;
}

int omap_dispc_wait_for_irq_interruptible_timeout(u32 irqmask,
		unsigned long timeout)
{
	void dispc_irq_wait_handler(void *data, u32 mask)
	{
		complete((struct completion *)data);
	}

	int r;
	DECLARE_COMPLETION_ONSTACK(completion);

	r = omap_dispc_register_isr(dispc_irq_wait_handler, &completion,
			irqmask);

	if (r)
		return r;

	timeout = wait_for_completion_interruptible_timeout(&completion,
			timeout);

	omap_dispc_unregister_isr(dispc_irq_wait_handler, &completion, irqmask);

	if (timeout == 0)
		return -ETIMEDOUT;

	if (timeout == -ERESTARTSYS)
		return -ERESTARTSYS;

	return 0;
}

#ifdef CONFIG_OMAP2_DSS_FAKE_VSYNC
void dispc_fake_vsync_irq(void)
{
	u32 irqstatus = DISPC_IRQ_VSYNC;
	int i;

	for (i = 0; i < DISPC_MAX_NR_ISRS; i++) {
		struct omap_dispc_isr_data *isr_data;
		isr_data = &dispc.registered_isr[i];

		if (!isr_data->isr)
			continue;

		if (isr_data->mask & irqstatus)
			isr_data->isr(isr_data->arg, irqstatus);
	}
}
#endif

static void _omap_dispc_initialize_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dispc.irq_lock, flags);

	memset(dispc.registered_isr, 0, sizeof(dispc.registered_isr));

	dispc.irq_error_mask = DISPC_IRQ_MASK_ERROR;

	/* there's SYNC_LOST_DIGIT waiting after enabling the DSS,
	 * so clear it */
	dispc_write_reg(DISPC_IRQSTATUS, dispc_read_reg(DISPC_IRQSTATUS));

	_omap_dispc_set_irqs();

	spin_unlock_irqrestore(&dispc.irq_lock, flags);
}

void dispc_enable_sidle(void)
{
	REG_FLD_MOD(DISPC_SYSCONFIG, 2, 4, 3);	/* SIDLEMODE: smart idle */
}

void dispc_disable_sidle(void)
{
	REG_FLD_MOD(DISPC_SYSCONFIG, 1, 4, 3);	/* SIDLEMODE: no idle */
}

static void _omap_dispc_initial_config(void)
{
	u32 l;

	l = dispc_read_reg(DISPC_SYSCONFIG);
	l = FLD_MOD(l, 2, 13, 12);	/* MIDLEMODE: smart standby */
	l = FLD_MOD(l, 2, 4, 3);	/* SIDLEMODE: smart idle */
	l = FLD_MOD(l, 1, 2, 2);	/* ENWAKEUP */
	l = FLD_MOD(l, 1, 0, 0);	/* AUTOIDLE */
	dispc_write_reg(DISPC_SYSCONFIG, l);

	/* FUNCGATED */
	REG_FLD_MOD(DISPC_CONFIG, 1, 9, 9);

	/* L3 firewall setting: enable access to OCM RAM */
	if (cpu_is_omap24xx())
		__raw_writel(0x402000b0, IO_ADDRESS(0x680050a0));

	_dispc_setup_color_conv_coef();

	dispc_set_loadmode(OMAP_DSS_LOAD_FRAME_ONLY);

	dispc_read_plane_fifo_sizes();
}


#define DISPC_LOADMODE			(BIT(1) | BIT(2))
#define DISPC_PALETTEGAMMA_TABLE	(BIT(3))
int dispc_setup_clut(u32 phy_addr)
{
	u32 temp;

	if (!phy_addr)
		return -ENODEV;

	/* Write the address in the GFX_TABLE_BA reg */
	dispc_write_reg(DISPC_GFX_TABLE_BA, (u32)phy_addr);

	/* set the LOAD palette command */
	temp = dispc_read_reg(DISPC_CONFIG);
	temp |= DISPC_PALETTEGAMMA_TABLE | DISPC_LOADMODE;
	dispc_write_reg(DISPC_CONFIG, temp);

	return 0;
}
int dispc_init()
{
	u32 rev;

	spin_lock_init(&dispc.irq_lock);

	INIT_WORK(&dispc.error_work, dispc_error_worker);

	dispc.base = ioremap(DISPC_BASE, DISPC_SZ_REGS);
	if (!dispc.base) {
		DSSERR("can't ioremap DISPC\n");
		return -ENOMEM;
	}

	if (cpu_is_omap34xx()) {
		dispc.dpll4_m4_ck = clk_get(NULL, "dpll4_m4_ck");
		if (IS_ERR(dispc.dpll4_m4_ck)) {
			DSSERR("Failed to get dpll4_m4_ck\n");
			return -ENODEV;
		}
	}

	enable_clocks(1);

	_omap_dispc_initial_config();

	_omap_dispc_initialize_irq();

	dispc_save_context();

	rev = dispc_read_reg(DISPC_REVISION);
	printk(KERN_INFO "OMAP DISPC rev %d.%d\n",
	       FLD_GET(rev, 7, 4), FLD_GET(rev, 3, 0));

	enable_clocks(0);

	return 0;
}

void dispc_exit(void)
{
	if (cpu_is_omap34xx())
		clk_put(dispc.dpll4_m4_ck);
	iounmap(dispc.base);
}

int dispc_enable_plane(enum omap_plane plane, bool enable)
{
	DSSDBG("dispc_enable_plane %d, %d\n", plane, enable);

	enable_clocks(1);
	_dispc_enable_plane(plane, enable);

	if (plane == OMAP_DSS_GFX) {
		if (enable)
			gfx_in_use = 1;
		else
			gfx_in_use = 0;
	}

	enable_clocks(0);

	return 0;
}

int dispc_setup_plane(enum omap_plane plane,
		       u32 paddr, u16 screen_width,
		       u16 pos_x, u16 pos_y,
		       u16 width, u16 height,
		       u16 out_width, u16 out_height,
		       enum omap_color_mode color_mode,
		       bool ilace,
		       enum omap_dss_rotation_type rotation_type,
		       u8 rotation, bool mirror, u8 global_alpha,
		       u8 pre_alpha_mult,
		       bool flicker_filter, int flicker_filter_level)
{
	int r = 0;

	DSSDBG("dispc_setup_plane %d, pa %x, sw %d, %d,%d, %dx%d -> "
	       "%dx%d, ilace %d, cmode %x, rot %d, mir %d "
	       "flicker_filter %d flicker_filter_level %d\n",
	       plane, paddr, screen_width, pos_x, pos_y,
	       width, height,
	       out_width, out_height,
	       ilace, color_mode,
	       rotation, mirror,
	       flicker_filter, flicker_filter_level);

	enable_clocks(1);

	r = _dispc_setup_plane(plane,
			   paddr, screen_width,
			   pos_x, pos_y,
			   width, height,
			   out_width, out_height,
			   color_mode, ilace,
			   rotation_type,
			   rotation, mirror,
			   global_alpha,
			   pre_alpha_mult,
			   flicker_filter, flicker_filter_level);

	enable_clocks(0);

	return r;
}
