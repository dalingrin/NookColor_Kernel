/*
 * drivers/mmc/host/omap_hsmmc.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * Authors:
 *	Syed Mohammed Khasim	<x0khasim@ti.com>

 *	Mohit Jalori		<mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/mmc.h>
#include <mach/cpu.h>

/* OMAP HSMMC Host Controller Registers */
#define OMAP_HSMMC_SYSCONFIG	0x0010
#define OMAP_HSMMC_SYSSTATUS	0x0014
#define OMAP_HSMMC_CON		0x002C
#define OMAP_HSMMC_BLK		0x0104
#define OMAP_HSMMC_ARG		0x0108
#define OMAP_HSMMC_CMD		0x010C
#define OMAP_HSMMC_RSP10	0x0110
#define OMAP_HSMMC_RSP32	0x0114
#define OMAP_HSMMC_RSP54	0x0118
#define OMAP_HSMMC_RSP76	0x011C
#define OMAP_HSMMC_DATA		0x0120
#define OMAP_HSMMC_HCTL		0x0128
#define OMAP_HSMMC_SYSCTL	0x012C
#define OMAP_HSMMC_STAT		0x0130
#define OMAP_HSMMC_IE		0x0134
#define OMAP_HSMMC_ISE		0x0138
#define OMAP_HSMMC_CAPA		0x0140

#define VS18			(1 << 26)
#define VS30			(1 << 25)
#define SDVS18			(0x5 << 9)
#define SDVS30			(0x6 << 9)
#define SDVS33			(0x7 << 9)
#define SDVSCLR			0xFFFFF1FF
#define SDVSDET			0x00000400
#define AUTOIDLE		0x1
#define SDBP			(1 << 8)
#define DTO			0xe
#define ICE			0x1
#define ICS			0x2
#define CEN			(1 << 2)
#define CLKD_MASK		0x0000FFC0
#define CLKD_SHIFT		6
#define DTO_MASK		0x000F0000
#define DTO_SHIFT		16
#define INT_EN_MASK		0x307F0033
#define INIT_STREAM		(1 << 1)
#define DP_SELECT		(1 << 21)
#define DDIR			(1 << 4)
#define DMA_EN			0x1
#define MSBS			(1 << 5)
#define BCE			(1 << 1)
#define FOUR_BIT		(1 << 1)
#define DW8			(1 << 5)
#define CC			0x1
#define TC			0x02
#define OD			0x1
#define ERR			(1 << 15)
#define CMD_TIMEOUT		(1 << 16)
#define DATA_TIMEOUT		(1 << 20)
#define CMD_CRC			(1 << 17)
#define CMD_CEB			(1 << 18)
#define CMD_CIE			(1 << 19)
#define DATA_CRC		(1 << 21)
#define DATA_DEB		(1 << 22)
#define CARD_ERR		(1 << 28)
#define STAT_CLEAR		0xFFFFFFFF
#define INIT_STREAM_CMD		0x00000000
#define DUAL_VOLT_OCR_BIT	7
#define SRC			(1 << 25)
#define SRD			(1 << 26)
#define RESETDONE		(1 << 0)

/*
 * FIXME: Most likely all the data using these _DEVID defines should come
 * from the platform_data, or implemented in controller and slot specific
 * functions.
 */
#define OMAP_MMC1_DEVID		0
#define OMAP_MMC2_DEVID		1

#define OMAP_MMC_DATADIR_NONE	0
#define OMAP_MMC_DATADIR_READ	1
#define OMAP_MMC_DATADIR_WRITE	2
#define MMC_TIMEOUT_MS		20
#define OMAP_MMC_MASTER_CLOCK	96000000
#define DRIVER_NAME		"mmci-omap-hs"

/*
 * One controller can have multiple slots, like on some omap boards using
 * omap.c controller driver. Luckily this is not currently done on any known
 * omap_hsmmc.c device.
 */
#define mmc_slot(host)		(host->pdata->slots[host->slot_id])

/*
 * MMC Host controller read/write API's
 */
#define OMAP_HSMMC_READ(base, reg)	\
	__raw_readl((base) + OMAP_HSMMC_##reg)

#define OMAP_HSMMC_WRITE(base, reg, val) \
	__raw_writel((val), (base) + OMAP_HSMMC_##reg)

struct mmc_omap_host {
	struct	device		*dev;
	struct	mmc_host	*mmc;
	struct	mmc_request	*mrq;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
	struct	clk		*fclk;
	struct	clk		*iclk;
	struct	clk		*dbclk;
	struct	semaphore	sem;
	struct	work_struct	mmc_carddetect_work;
	struct  work_struct	mmc_opp_set_work;
	void	__iomem		*base;
	resource_size_t		mapbase;
	unsigned int		id;
	unsigned int		dma_len;
	unsigned int		dma_dir;
	unsigned char		bus_mode;
	unsigned char		datadir;
	u32			*buffer;
	u32			bytesleft;
	int			suspended;
	int			irq;
	int			carddetect;
	int			use_dma, dma_ch;
	int			initstr;
	int			slot_id;
	int			dbclk_enabled;
	int 			clks_enabled;
	int			inactive;
	unsigned long		max_vdd1_opp;
	unsigned long		min_vdd1_opp;
	unsigned		off_counter;
	spinlock_t		clk_lock;
	struct timer_list	inact_timer;
	struct	omap_mmc_platform_data	*pdata;
	int			shutdown;
};

struct omap_hsmmc_regs {
	u32 hctl;
	u32 capa;
	u32 sysconfig;
	u32 ise;
	u32 ie;
	u32 con;
	u32 sysctl;
};
static struct omap_hsmmc_regs hsmmc_ctx[3];

static void omap2_hsmmc_save_ctx(struct mmc_omap_host *host)
{
	/* MMC : context save */
	hsmmc_ctx[host->id].hctl = OMAP_HSMMC_READ(host->base, HCTL);
	hsmmc_ctx[host->id].capa = OMAP_HSMMC_READ(host->base, CAPA);
	hsmmc_ctx[host->id].ise = OMAP_HSMMC_READ(host->base, ISE);
	hsmmc_ctx[host->id].ie = OMAP_HSMMC_READ(host->base, IE);
	hsmmc_ctx[host->id].con = OMAP_HSMMC_READ(host->base, CON);
	hsmmc_ctx[host->id].sysctl = OMAP_HSMMC_READ(host->base, SYSCTL);
}

static void omap2_hsmmc_restore_ctx(struct mmc_omap_host *host)
{
	/* MMC : context restore */
	OMAP_HSMMC_WRITE(host->base, HCTL, hsmmc_ctx[host->id].hctl);
	OMAP_HSMMC_WRITE(host->base, CAPA, hsmmc_ctx[host->id].capa);
	OMAP_HSMMC_WRITE(host->base, CON, hsmmc_ctx[host->id].con);
	OMAP_HSMMC_WRITE(host->base, ISE, hsmmc_ctx[host->id].ise);
	OMAP_HSMMC_WRITE(host->base, IE, hsmmc_ctx[host->id].ie);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, hsmmc_ctx[host->id].sysctl);
	OMAP_HSMMC_WRITE(host->base, HCTL, OMAP_HSMMC_READ(host->base,
		HCTL) | SDBP);
}

static int omap_hsmmc_enable_clks(struct mmc_omap_host *host)
{
	unsigned long flags, timeout;
	int ret = 0;

	spin_lock_irqsave(&host->clk_lock, flags);

	if (host->clks_enabled)
		goto done;

	ret = clk_enable(host->iclk);
	if (ret)
		goto clk_en_err1;

	ret = clk_enable(host->fclk);
	if (ret)
		goto clk_en_err2;

	host->clks_enabled = 1;

	if (cpu_is_omap2430()) {
		/*
		 * MMC can still work without debounce clock.
		 */
		if (IS_ERR(host->dbclk))
			dev_warn(mmc_dev(host->mmc),
			"Failed to get debounce clock\n");
		else
			if (clk_enable(host->dbclk) != 0)
				dev_dbg(mmc_dev(host->mmc), "Enabling debounce"
						" clk failed\n");
			else
				host->dbclk_enabled = 1;
	}

	if (!(host->pdata->context_loss) ||
		(host->pdata->context_loss(host->dev) != host->off_counter)) {
			/* Coming out of OFF:
			 * The SRA bit of SYSCTL reg has a wrong reset
			 * value.
			 * The bit resets automatically in subsequent
			 * reads. Idealy it should have been 0 as per
			 * the reset value of the register.
			 * Wait for the reset to complete */
			timeout = jiffies +
				msecs_to_jiffies(MMC_TIMEOUT_MS);
			while ((OMAP_HSMMC_READ(host->base, SYSSTATUS)
				& RESETDONE) != RESETDONE
				&& time_before(jiffies, timeout))
				;
			omap2_hsmmc_restore_ctx(host);
	}

done:
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return ret;
clk_en_err2:
	clk_disable(host->iclk);
clk_en_err1:
	dev_dbg(mmc_dev(host->mmc),
		"Unable to enable MMC clocks \n");
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return ret;
}

static void omap_hsmmc_disable_clks(struct mmc_omap_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);

	if (!host->clks_enabled)
		goto done;

	omap2_hsmmc_save_ctx(host);
	if (host->pdata->context_loss)
		host->off_counter = host->pdata->context_loss(host->dev);

	clk_disable(host->fclk);
	clk_disable(host->iclk);
	host->clks_enabled = 0;
	if (cpu_is_omap2430()) {
		if (host->dbclk_enabled) {
			clk_disable(host->dbclk);
			host->dbclk_enabled = 0;
		}
	}
done:
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

/*
 * Stop clock to the card
 */
static void omap_mmc_stop_clock(struct mmc_omap_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		dev_dbg(mmc_dev(host->mmc), "MMC Clock is not stoped\n");
}

/*
 * Send init stream sequence to card
 * before sending IDLE command
 */
static void send_init_stream(struct mmc_omap_host *host)
{
	int reg = 0;
	unsigned long timeout;

	disable_irq(host->irq);
	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((reg != CC) && time_before(jiffies, timeout))
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;

	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);
	enable_irq(host->irq);
}

static inline
int mmc_omap_cover_is_closed(struct mmc_omap_host *host)
{
	int r = 1;

	if (host->pdata->slots[host->slot_id].get_cover_state)
		r = host->pdata->slots[host->slot_id].get_cover_state(host->dev,
			host->slot_id);
	return r;
}

static ssize_t
mmc_omap_show_cover_switch(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct mmc_omap_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n", mmc_omap_cover_is_closed(host) ? "closed" :
		       "open");
}

static DEVICE_ATTR(cover_switch, S_IRUGO, mmc_omap_show_cover_switch, NULL);

static ssize_t
mmc_omap_show_slot_name(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct mmc_omap_host *host = mmc_priv(mmc);
	struct omap_mmc_slot_data slot = host->pdata->slots[host->slot_id];

	return sprintf(buf, "slot:%s\n", slot.name);
}

static DEVICE_ATTR(slot_name, S_IRUGO, mmc_omap_show_slot_name, NULL);

/*
 * Configure the response type and send the cmd.
 */
static void
mmc_omap_start_command(struct mmc_omap_host *host, struct mmc_command *cmd,
	struct mmc_data *data)
{
	int cmdreg = 0, resptype = 0, cmdtype = 0;

	dev_dbg(mmc_dev(host->mmc), "%s: CMD%d, argument 0x%08x\n",
		mmc_hostname(host->mmc), cmd->opcode, cmd->arg);
	host->cmd = cmd;

	/*
	 * Clear status bits and enable interrupts
	 */
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE, INT_EN_MASK);
	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			resptype = 1;
		else
			resptype = 2;
	}

#ifdef CONFIG_HC_Broken_eMMC_ZOOM2
	if (host->id == OMAP_MMC2_DEVID) {
		/*
		 * HACK:
		 * eMMC on Zoom2 seems fail init sequence without this
		 * delay on SWITCH CMD.
		 */
		if (cmd->opcode == 6)
			msleep(200);
	}
#endif

	/*
	 * Unlike OMAP1 controller, the cmdtype does not seem to be based on
	 * ac, bc, adtc, bcr. Only commands ending an open ended transfer need
	 * a val of 0x3, rest 0x0.
	 */
	if (cmd == host->mrq->stop)
		cmdtype = 0x3;

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

	if (data) {
		cmdreg |= DP_SELECT | MSBS | BCE;
		if (data->flags & MMC_DATA_READ)
			cmdreg |= DDIR;
		else
			cmdreg &= ~(DDIR);
	}

	if (host->use_dma)
		cmdreg |= DMA_EN;

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
}

/*
 * Notify the transfer complete to MMC core
 */
static void
mmc_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	host->data = NULL;

	if (host->use_dma && host->dma_ch != -1)
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma_len,
			host->dma_dir);

	host->datadir = OMAP_MMC_DATADIR_NONE;

	if (!data->error)
		data->bytes_xfered += data->blocks * (data->blksz);
	else
		data->bytes_xfered = 0;

	if (!data->stop) {
		mod_timer(&host->inact_timer, jiffies + msecs_to_jiffies(1000));
		host->mrq = NULL;
		mmc_request_done(host->mmc, data->mrq);
		return;
	}
	mmc_omap_start_command(host, data->stop, NULL);
}

/*
 * Notify the core about command completion
 */
static void
mmc_omap_cmd_done(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
		}
	}
	if (host->data == NULL || cmd->error) {
		mod_timer(&host->inact_timer, jiffies + msecs_to_jiffies(1000));
		host->mrq = NULL;
		mmc_request_done(host->mmc, cmd->mrq);
	}
}

static void
omap_hsmmc_inact_timer(unsigned long data)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *) data;

	omap_hsmmc_disable_clks(host);
	schedule_work(&host->mmc_opp_set_work);
}

/*
 * DMA clean up for command errors
 */
static void mmc_dma_cleanup(struct mmc_omap_host *host, int errno)
{
	host->data->error = errno;

	if (host->use_dma && host->dma_ch != -1) {
		dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->dma_len,
			host->dma_dir);
		omap_free_dma(host->dma_ch);
		host->dma_ch = -1;
		up(&host->sem);
	}
	host->data = NULL;
	host->datadir = OMAP_MMC_DATADIR_NONE;
}

/*
 * Readable error output
 */
#ifdef CONFIG_MMC_DEBUG
static void mmc_omap_report_irq(struct mmc_omap_host *host, u32 status)
{
	/* --- means reserved bit without definition at documentation */
	static const char *mmc_omap_status_bits[] = {
		"CC", "TC", "BGE", "---", "BWR", "BRR", "---", "---", "CIRQ",
		"OBI", "---", "---", "---", "---", "---", "ERRI", "CTO", "CCRC",
		"CEB", "CIE", "DTO", "DCRC", "DEB", "---", "ACE", "---",
		"---", "---", "---", "CERR", "CERR", "BADA", "---", "---", "---"
	};
	char res[256];
	char *buf = res;
	int len, i;

	len = sprintf(buf, "MMC IRQ 0x%x :", status);
	buf += len;

	for (i = 0; i < ARRAY_SIZE(mmc_omap_status_bits); i++)
		if (status & (1 << i)) {
			len = sprintf(buf, " %s", mmc_omap_status_bits[i]);
			buf += len;
		}

	dev_dbg(mmc_dev(host->mmc), "%s\n", res);
}
#endif  /* CONFIG_MMC_DEBUG */

/*
 * MMC controller internal state machines reset
 *
 * Used to reset command or data internal state machines, using respectively
 *  SRC or SRD bit of SYSCTL register
 * Can be called from interrupt context
 */
static inline void mmc_omap_reset_controller_fsm(struct mmc_omap_host *host,
		unsigned long bit)
{
	unsigned long i = 0;
	unsigned long limit = (loops_per_jiffy *
				msecs_to_jiffies(MMC_TIMEOUT_MS));

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			 OMAP_HSMMC_READ(host->base, SYSCTL) | bit);

	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & bit) &&
		(i++ < limit))
		cpu_relax();

	if (OMAP_HSMMC_READ(host->base, SYSCTL) & bit)
		dev_err(mmc_dev(host->mmc),
			"Timeout waiting on controller reset in %s\n",
			__func__);
}

/*
 * MMC controller IRQ handler
 */
static irqreturn_t mmc_omap_irq(int irq, void *dev_id)
{
	struct mmc_omap_host *host = dev_id;
	struct mmc_data *data;
	int end_cmd = 0, end_trans = 0, status;

	omap_hsmmc_enable_clks(host);

	if (host->cmd == NULL && host->data == NULL) {
		OMAP_HSMMC_WRITE(host->base, STAT,
			OMAP_HSMMC_READ(host->base, STAT));
		return IRQ_HANDLED;
	}

	data = host->data;
	status = OMAP_HSMMC_READ(host->base, STAT);
	dev_dbg(mmc_dev(host->mmc), "IRQ Status is %x\n", status);

	if (status & ERR) {
#ifdef CONFIG_MMC_DEBUG
		mmc_omap_report_irq(host, status);
#endif
		if ((status & CMD_TIMEOUT) ||
			(status & CMD_CRC) ||
			(status & CMD_CEB) ||
			(status & CMD_CIE)) {
			if (host->cmd) {
				if (status & CMD_TIMEOUT) {
					mmc_omap_reset_controller_fsm(host, SRC);
					host->cmd->error = -ETIMEDOUT;
				} else {
					host->cmd->error = -EILSEQ;
				}
				end_cmd = 1;
			}
			if (host->data) {
				mmc_dma_cleanup(host, -ETIMEDOUT);
				mmc_omap_reset_controller_fsm(host, SRD);
			}
		}
		if ((status & DATA_TIMEOUT) ||
			(status & DATA_CRC) ||
			(status & DATA_DEB)) {
			if (host->data) {
				if (status & DATA_TIMEOUT)
					mmc_dma_cleanup(host, -ETIMEDOUT);
				else
					mmc_dma_cleanup(host, -EILSEQ);
				mmc_omap_reset_controller_fsm(host, SRD);
				end_trans = 1;
			}
		}
		if (status & CARD_ERR) {
			dev_dbg(mmc_dev(host->mmc),
				"Ignoring card err CMD%d\n", host->cmd->opcode);
			if (host->cmd)
				end_cmd = 1;
			if (host->data)
				end_trans = 1;
		}
	}

	OMAP_HSMMC_WRITE(host->base, STAT, status);

	if (end_cmd || (status & CC))
		mmc_omap_cmd_done(host, host->cmd);
	if (end_trans || (status & TC))
		mmc_omap_xfer_done(host, data);

	return IRQ_HANDLED;
}

/*
 * Switch MMC interface voltage ... only relevant for MMC1.
 *
 * MMC2 and MMC3 use fixed 1.8V levels, and maybe a transceiver.
 * The MMC2 transceiver controls are used instead of DAT4..DAT7.
 * Some chips, like eMMC ones, use internal transceivers.
 */
static int omap_mmc_switch_opcond(struct mmc_omap_host *host, int vdd)
{
	u32 reg_val = 0;
	int ret;

	if (host->id != OMAP_MMC1_DEVID)
		return 0;

	/* Disable the clocks */
	omap_hsmmc_disable_clks(host);

	/* Turn the power off */
	ret = mmc_slot(host).set_power(host->dev, host->slot_id, 0, 0);
	if (ret != 0)
		goto err;

	/* Turn the power ON with given VDD 1.8 or 3.0v */
	ret = mmc_slot(host).set_power(host->dev, host->slot_id, 1, vdd);
	if (ret != 0)
		goto err;

	omap_hsmmc_enable_clks(host);

	OMAP_HSMMC_WRITE(host->base, HCTL,
		OMAP_HSMMC_READ(host->base, HCTL) & SDVSCLR);
	reg_val = OMAP_HSMMC_READ(host->base, HCTL);

	/*
	 * If a MMC dual voltage card is detected, the set_ios fn calls
	 * this fn with VDD bit set for 1.8V. Upon card removal from the
	 * slot, omap_mmc_set_ios sets the VDD back to 3V on MMC_POWER_OFF.
	 *
	 * Cope with a bit of slop in the range ... per data sheets:
	 *  - "1.8V" for vdds_mmc1/vdds_mmc1a can be up to 2.45V max,
	 *    but recommended values are 1.71V to 1.89V
	 *  - "3.0V" for vdds_mmc1/vdds_mmc1a can be up to 3.5V max,
	 *    but recommended values are 2.7V to 3.3V
	 *
	 * Board setup code shouldn't permit anything very out-of-range.
	 * TWL4030-family VMMC1 and VSIM regulators are fine (avoiding the
	 * middle range) but VSIM can't power DAT4..DAT7 at more than 3V.
	 */
	if ((1 << vdd) <= MMC_VDD_23_24)
		reg_val |= SDVS18;
	else
		reg_val |= SDVS30;

	OMAP_HSMMC_WRITE(host->base, HCTL, reg_val);

	OMAP_HSMMC_WRITE(host->base, HCTL,
		OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	return 0;
err:
	dev_dbg(mmc_dev(host->mmc), "Unable to switch operating voltage\n");
	return ret;
}

/*
 * Work Item to notify the core about card insertion/removal
 */
static void mmc_omap_detect(struct work_struct *work)
{
	struct mmc_omap_host *host = container_of(work, struct mmc_omap_host,
						mmc_carddetect_work);
	struct omap_mmc_slot_data *slot = &mmc_slot(host);

	if (host->suspended)
		return;

	omap_hsmmc_enable_clks(host);

	host->carddetect = slot->card_detect(slot->card_detect_irq);

	sysfs_notify(&host->mmc->class_dev.kobj, NULL, "cover_switch");
	if (host->carddetect) {
		mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	} else {
		mmc_detect_change(host->mmc, (HZ * 50) / 1000);
	}
}

static void mmc_omap_opp_setup(struct work_struct *work)
{
	struct mmc_omap_host *host = container_of(work, struct mmc_omap_host,
		mmc_opp_set_work);

	if (host->pdata->set_vdd1_opp) {
		host->pdata->set_vdd1_opp(host->dev, host->min_vdd1_opp);
		host->inactive = 1;
	}
}

/*
 * ISR for handling card insertion and removal
 */
static irqreturn_t omap_mmc_cd_handler(int irq, void *dev_id)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *)dev_id;

	if (host->suspended)
		return IRQ_HANDLED;

	schedule_work(&host->mmc_carddetect_work);

	return IRQ_HANDLED;
}

/*
 * DMA call back function
 */
static void mmc_omap_dma_cb(int lch, u16 ch_status, void *data)
{
	struct mmc_omap_host *host = data;

	if (!(ch_status & OMAP_DMA_BLOCK_IRQ)) {
		dev_warn(mmc_dev(host->mmc), "unexpected dma status %x\n",
			ch_status);
		return;
	}

	if (host->dma_ch < 0)
		return;

	omap_free_dma(host->dma_ch);
	host->dma_ch = -1;
	/*
	 * DMA Callback: run in interrupt context.
	 * mutex_unlock will through a kernel warning if used.
	 */
	up(&host->sem);
}

/*
 * Configure dma src and destination parameters
 */
static int mmc_omap_config_dma_param(int sync_dir, struct mmc_omap_host *host,
				struct mmc_data *data)
{
	if (sync_dir == 0) {
		omap_set_dma_dest_params(host->dma_ch, 0,
			OMAP_DMA_AMODE_CONSTANT,
			(host->mapbase + OMAP_HSMMC_DATA), 0, 0);
		omap_set_dma_src_params(host->dma_ch, 0,
			OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(&data->sg[0]), 0, 0);
	} else {
		omap_set_dma_src_params(host->dma_ch, 0,
			OMAP_DMA_AMODE_CONSTANT,
			(host->mapbase + OMAP_HSMMC_DATA), 0, 0);
		omap_set_dma_dest_params(host->dma_ch, 0,
			OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(&data->sg[0]), 0, 0);
	}
	return 0;
}
/*
 * Routine to configure and start DMA for the MMC card
 */
static int
mmc_omap_start_dma_transfer(struct mmc_omap_host *host, struct mmc_request *req)
{
	int sync_dev, sync_dir = 0;
	int dma_ch = 0, ret = 0, err = 1;
	struct mmc_data *data = req->data;

	/*
	 * If for some reason the DMA transfer is still active,
	 * we wait for timeout period and free the dma
	 */
	if (host->dma_ch != -1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(100);
		if (down_trylock(&host->sem)) {
			omap_free_dma(host->dma_ch);
			host->dma_ch = -1;
			up(&host->sem);
			return err;
		}
	} else {
		if (down_trylock(&host->sem))
			return err;
	}

	if (!(data->flags & MMC_DATA_WRITE)) {
		host->dma_dir = DMA_FROM_DEVICE;
		if (host->id == OMAP_MMC1_DEVID)
			sync_dev = OMAP24XX_DMA_MMC1_RX;
		else if (host->id == OMAP_MMC2_DEVID)
			sync_dev = OMAP24XX_DMA_MMC2_RX;
		else
#ifdef CONFIG_OMAP_HS_MMC3
			sync_dev = OMAP34XX_DMA_MMC3_RX;
#else
			sync_dev = OMAP24XX_DMA_MMC2_RX;
#endif
	} else {
		host->dma_dir = DMA_TO_DEVICE;
		if (host->id == OMAP_MMC1_DEVID)
			sync_dev = OMAP24XX_DMA_MMC1_TX;
		else if(host->id == OMAP_MMC2_DEVID)
			sync_dev = OMAP24XX_DMA_MMC2_TX;
		else
#ifdef CONFIG_OMAP_HS_MMC3
			sync_dev = OMAP34XX_DMA_MMC3_TX;
#else
			sync_dev = OMAP24XX_DMA_MMC2_TX;
#endif
	}

	ret = omap_request_dma(sync_dev, "MMC/SD", mmc_omap_dma_cb,
			host, &dma_ch);
	if (ret != 0) {
		dev_dbg(mmc_dev(host->mmc),
			"%s: omap_request_dma() failed with %d\n",
			mmc_hostname(host->mmc), ret);
		return ret;
	}

	host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg,
			data->sg_len, host->dma_dir);
	host->dma_ch = dma_ch;

	if (!(data->flags & MMC_DATA_WRITE))
		mmc_omap_config_dma_param(1, host, data);
	else
		mmc_omap_config_dma_param(0, host, data);

	if ((data->blksz % 4) == 0)
		omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S32,
			(data->blksz / 4), data->blocks, OMAP_DMA_SYNC_FRAME,
			sync_dev, sync_dir);
	else
		/* REVISIT: The MMC buffer increments only when MSB is written.
		 * Return error for blksz which is non multiple of four.
		 */
		return -EINVAL;

	omap_start_dma(dma_ch);
	return 0;
}

static void set_data_timeout(struct mmc_omap_host *host,
			     struct mmc_request *req)
{
	unsigned int timeout, cycle_ns;
	uint32_t reg, clkd, dto = 0;

	reg = OMAP_HSMMC_READ(host->base, SYSCTL);
	clkd = (reg & CLKD_MASK) >> CLKD_SHIFT;
	if (clkd == 0)
		clkd = 1;

	cycle_ns = 1000000000 / (clk_get_rate(host->fclk) / clkd);
	timeout = req->data->timeout_ns / cycle_ns;
	timeout += req->data->timeout_clks;
	if (timeout) {
		while ((timeout & 0x80000000) == 0) {
			dto += 1;
			timeout <<= 1;
		}
		dto = 31 - dto;
		timeout <<= 1;
		if (timeout && dto)
			dto += 1;
		if (dto >= 13)
			dto -= 13;
		else
			dto = 0;
		if (dto > 14)
			dto = 14;
	}

	reg &= ~DTO_MASK;
	reg |= dto << DTO_SHIFT;
	OMAP_HSMMC_WRITE(host->base, SYSCTL, reg);
}

/*
 * Configure block length for MMC/SD cards and initiate the transfer.
 */
static int
mmc_omap_prepare_data(struct mmc_omap_host *host, struct mmc_request *req)
{
	int ret;
	host->data = req->data;

	if (req->data == NULL) {
		host->datadir = OMAP_MMC_DATADIR_NONE;
		OMAP_HSMMC_WRITE(host->base, BLK, 0);
		return 0;
	}

	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz)
					| (req->data->blocks << 16));
	set_data_timeout(host, req);

	host->datadir = (req->data->flags & MMC_DATA_WRITE) ?
			OMAP_MMC_DATADIR_WRITE : OMAP_MMC_DATADIR_READ;

	if (host->use_dma) {
		ret = mmc_omap_start_dma_transfer(host, req);
		if (ret != 0) {
			dev_dbg(mmc_dev(host->mmc), "MMC start dma failure\n");
			return ret;
		}
	}
	return 0;
}

/*
 * Request function. for read/write operation
 */
static void omap_mmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_omap_host *host = mmc_priv(mmc);

	WARN_ON(host->mrq != NULL);
	host->mrq = req;

	if (host->shutdown) {
		return;
	}

	if (host->inactive)
		if (host->pdata->set_vdd1_opp)
			host->pdata->set_vdd1_opp(host->dev,
				host->max_vdd1_opp);

	del_timer_sync(&host->inact_timer);
	host->inactive = 0;
	omap_hsmmc_enable_clks(host);

	mmc_omap_prepare_data(host, req);
	mmc_omap_start_command(host, req->cmd, req->data);
}


/* Routine to configure clock values. Exposed API to core */
static void omap_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmc_omap_host *host = mmc_priv(mmc);
	u16 dsor = 0;
	unsigned long regval;
	unsigned long timeout;
	u32 con;

	del_timer_sync(&host->inact_timer);
	omap_hsmmc_enable_clks(host);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		mmc_slot(host).set_power(host->dev, host->slot_id, 0, 0);
		/*
		 * Reset interface voltage to 3V if it's 1.8V now;
		 * only relevant on MMC-1, the others always use 1.8V.
		 *
		 * REVISIT: If we are able to detect cards after unplugging
		 * a 1.8V card, this code should not be needed.
		 */
		if (host->id != OMAP_MMC1_DEVID)
			break;
		if (!(OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET)) {
			int vdd = fls(host->mmc->ocr_avail) - 1;
			if (omap_mmc_switch_opcond(host, vdd) != 0)
				host->mmc->ios.vdd = vdd;
		}
		break;
	case MMC_POWER_UP:
		mmc_slot(host).set_power(host->dev, host->slot_id, 1, ios->vdd);
		break;
	}

	switch (mmc->ios.bus_width) {
	case MMC_BUS_WIDTH_8:
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) | DW8);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~FOUR_BIT);
		break;
	}

	if (host->id == OMAP_MMC1_DEVID) {
		/* Only MMC1 can interface at 3V without some flavor
		 * of external transceiver; but they all handle 1.8V.
		 */
		if ((OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET) &&
			(ios->vdd == DUAL_VOLT_OCR_BIT)) {
				/*
				 * The mmc_select_voltage fn of the core does
				 * not seem to set the power_mode to
				 * MMC_POWER_UP upon recalculating the voltage.
				 * vdd 1.8v.
				 */
				if (omap_mmc_switch_opcond(host, ios->vdd) != 0)
					dev_dbg(mmc_dev(host->mmc),
						"Switch operation failed\n");
		}
	}

	if (ios->clock) {
		dsor = OMAP_MMC_MASTER_CLOCK / ios->clock;
		if (dsor < 1)
			dsor = 1;

		if (OMAP_MMC_MASTER_CLOCK / dsor > ios->clock)
			dsor++;

		if (dsor > 250)
			dsor = 250;
	}
	omap_mmc_stop_clock(host);
	regval = OMAP_HSMMC_READ(host->base, SYSCTL);
	regval = regval & ~(CLKD_MASK);
	regval = regval | (dsor << 6) | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regval);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* Wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != 0x2
		&& time_before(jiffies, timeout))
		msleep(1);

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);

	if (ios->power_mode == MMC_POWER_ON)
		send_init_stream(host);

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		OMAP_HSMMC_WRITE(host->base, CON, con | OD);
	else
		OMAP_HSMMC_WRITE(host->base, CON, con & ~OD);

	omap_hsmmc_disable_clks(host);
}

static int omap_hsmmc_get_cd(struct mmc_host *mmc)
{
	struct mmc_omap_host *host = mmc_priv(mmc);
	struct omap_mmc_platform_data *pdata = host->pdata;

	if (!pdata->slots[0].card_detect)
		return -ENOSYS;
	return pdata->slots[0].card_detect(pdata->slots[0].card_detect_irq);
}

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static void omap_hsmmc_status_notify_cb(int card_present, void *dev_id)
{
	struct mmc_omap_host *host = dev_id;
	struct omap_mmc_slot_data *slot = &mmc_slot(host);

        printk(KERN_DEBUG "%s: card_present %d\n", mmc_hostname(host->mmc),
		               card_present);

	host->carddetect = slot->card_detect(slot->card_detect_irq);

	sysfs_notify(&host->mmc->class_dev.kobj, NULL, "cover_switch");
	if (host->carddetect) {
		mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	} else {
		mmc_omap_reset_controller_fsm(host, SRD);
		mmc_detect_change(host->mmc, (HZ * 50) / 1000);
	}
}
#endif

static int omap_hsmmc_get_ro(struct mmc_host *mmc)
{
	struct mmc_omap_host *host = mmc_priv(mmc);
	struct omap_mmc_platform_data *pdata = host->pdata;

	if (!pdata->slots[0].get_ro)
		return -ENOSYS;
	return pdata->slots[0].get_ro(host->dev, 0);
}

static struct mmc_host_ops mmc_omap_ops = {
	.request = omap_mmc_request,
	.set_ios = omap_mmc_set_ios,
	.get_cd = omap_hsmmc_get_cd,
	.get_ro = omap_hsmmc_get_ro,
	/* NYET -- enable_sdio_irq */
};

static int __init omap_mmc_probe(struct platform_device *pdev)
{
	struct omap_mmc_platform_data *pdata = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct mmc_omap_host *host = NULL;
	struct resource *res;
	int ret = 0, irq;
	u32 hctl, capa;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "Platform Data is missing\n");
		return -ENXIO;
	}

	if (pdata->nr_slots == 0) {
		dev_err(&pdev->dev, "No Slots\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENXIO;

	res = request_mem_region(res->start, res->end - res->start + 1,
							pdev->name);
	if (res == NULL)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct mmc_omap_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto err;
	}

	host		= mmc_priv(mmc);
	host->mmc	= mmc;
	host->pdata	= pdata;
	host->dev	= &pdev->dev;
	host->use_dma	= 1;
	host->dev->dma_mask = &pdata->dma_mask;
	host->dma_ch	= -1;
	host->irq	= irq;
	host->id	= pdev->id;
	host->slot_id	= 0;
	host->mapbase	= res->start;
	host->base	= ioremap(host->mapbase, SZ_4K);
	host->shutdown = 0;

	host->max_vdd1_opp = pdata->max_vdd1_opp;
	host->min_vdd1_opp = pdata->min_vdd1_opp;

#ifdef CONFIG_HC_Broken_eMMC_ZOOM2
	/*
	 * HACK:
	 * The HC eMMC card on Zoom2 is broken. It reports wrong ext_csd
	 * version. This is a hack to make the eMMC card useble on Zoom2.
	 * Without this hack the MMC core fails to detect the correct size
	 * of the card and hence accesses beyond the detected boundary results
	 * in DATA CRC errors.
	 * Make use of the unused bit to indicate the host controller ID to
	 * the MMC core.
	 */
	if (host->id == OMAP_MMC2_DEVID)
		host->mmc->unused = 1;
#endif

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (pdata->slots[0].embedded_sdio)
		mmc_set_embedded_sdio_data(mmc,
				&pdata->slots[0].embedded_sdio->cis,
				&pdata->slots[0].embedded_sdio->cccr,
				pdata->slots[0].embedded_sdio->funcs,
				pdata->slots[0].embedded_sdio->num_funcs);
#endif
	platform_set_drvdata(pdev, host);
	INIT_WORK(&host->mmc_carddetect_work, mmc_omap_detect);
	INIT_WORK(&host->mmc_opp_set_work, mmc_omap_opp_setup);

	mmc->ops	= &mmc_omap_ops;
	mmc->f_min	= 400000;
	mmc->f_max	= 52000000;

	sema_init(&host->sem, 1);
	spin_lock_init(&host->clk_lock);

	init_timer(&host->inact_timer);
	host->inact_timer.function = omap_hsmmc_inact_timer;
	host->inact_timer.data = (unsigned long) host;

	host->clks_enabled = 0;
	host->off_counter = 0;
	host->inactive = 0;

	host->iclk = clk_get(&pdev->dev, "mmchs_ick");
	if (IS_ERR(host->iclk)) {
		ret = PTR_ERR(host->iclk);
		host->iclk = NULL;
		goto err1;
	}
	host->fclk = clk_get(&pdev->dev, "mmchs_fck");
	if (IS_ERR(host->fclk)) {
		ret = PTR_ERR(host->fclk);
		host->fclk = NULL;
		clk_put(host->iclk);
		goto err1;
	}

	if (cpu_is_omap2430()) {
		host->dbclk_enabled = 0;
		host->dbclk = clk_get(&pdev->dev, "mmchsdb_fck");
	}

	if (omap_hsmmc_enable_clks(host) != 0) {
		clk_put(host->iclk);
		clk_put(host->fclk);
		goto err1;
	}

#ifdef CONFIG_MMC_BLOCK_BOUNCE
	mmc->max_phys_segs = 1;
	mmc->max_hw_segs = 1;
#endif
	mmc->max_blk_size = 512;       /* Block Length at max can be 1024 */
	mmc->max_blk_count = 0xFFFF;    /* No. of Blocks is 16 bits */
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	if (mmc_slot(host).wires >= 8)
		mmc->caps |= MMC_CAP_8_BIT_DATA;
	else if (pdata->slots[host->slot_id].wires >= 4)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	/* Only MMC1 supports 3.0V */
	if (host->id == OMAP_MMC1_DEVID) {
		hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | hctl);

	OMAP_HSMMC_WRITE(host->base, CAPA,
			OMAP_HSMMC_READ(host->base, CAPA) | capa);

	/* Set the controller to AUTO IDLE mode */
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG,
			OMAP_HSMMC_READ(host->base, SYSCONFIG) | AUTOIDLE);

	/* Set SD bus power bit */
	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, mmc_omap_irq, IRQF_DISABLED,
			mmc_hostname(mmc), host);
	if (ret) {
		dev_dbg(mmc_dev(host->mmc), "Unable to grab HSMMC IRQ\n");
		goto err_irq;
	}

	/* initialize power supplies, gpios, etc */
	if (pdata->init != NULL) {
		if (pdata->init(&pdev->dev) != 0) {
			dev_dbg(mmc_dev(host->mmc), "late init error\n");
			goto err_irq_cd_init;
		}
	}
	mmc->ocr_avail = mmc_slot(host).ocr_mask;

	/* Request IRQ for card detect */
	if ((mmc_slot(host).card_detect_irq) && (mmc_slot(host).card_detect)) {
		ret = request_irq(mmc_slot(host).card_detect_irq,
				  omap_mmc_cd_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
					  | IRQF_DISABLED,
				  mmc_hostname(mmc), host);
		if (ret) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to grab MMC CD IRQ\n");
			goto err_irq_cd;
		}
	}
#ifdef CONFIG_MMC_EMBEDDED_SDIO
	else if (mmc_slot(host).register_status_notify) {
		mmc_slot(host).register_status_notify(omap_hsmmc_status_notify_cb, host);
	}
#endif

	OMAP_HSMMC_WRITE(host->base, ISE, INT_EN_MASK);
	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);

	mmc_add_host(mmc);

	if (host->pdata->slots[host->slot_id].name != NULL) {
		ret = device_create_file(&mmc->class_dev, &dev_attr_slot_name);
		if (ret < 0)
			goto err_slot_name;
	}
	if (mmc_slot(host).card_detect_irq && mmc_slot(host).card_detect &&
			host->pdata->slots[host->slot_id].get_cover_state) {
		ret = device_create_file(&mmc->class_dev,
					&dev_attr_cover_switch);
		if (ret < 0)
			goto err_cover_switch;
	}

	return 0;

err_cover_switch:
	device_remove_file(&mmc->class_dev, &dev_attr_cover_switch);
err_slot_name:
	mmc_remove_host(mmc);
err_irq_cd:
	free_irq(mmc_slot(host).card_detect_irq, host);
err_irq_cd_init:
	free_irq(host->irq, host);
err_irq:
	omap_hsmmc_disable_clks(host);
	clk_put(host->fclk);
	clk_put(host->iclk);
	if (cpu_is_omap2430())
		clk_put(host->dbclk);

err1:
	iounmap(host->base);
err:
	dev_dbg(mmc_dev(host->mmc), "Probe Failed\n");
	release_mem_region(res->start, res->end - res->start + 1);
	if (host)
		mmc_free_host(mmc);
	return ret;
}

static int omap_mmc_remove(struct platform_device *pdev)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);
	struct resource *res;

	if (host) {
		omap_hsmmc_enable_clks(host);
		mmc_remove_host(host->mmc);
		if (host->pdata->cleanup)
			host->pdata->cleanup(&pdev->dev);
		free_irq(host->irq, host);
		if (mmc_slot(host).card_detect_irq)
			free_irq(mmc_slot(host).card_detect_irq, host);
		flush_scheduled_work();

		omap_hsmmc_disable_clks(host);
		clk_put(host->fclk);
		clk_put(host->iclk);
		if (cpu_is_omap2430())
			clk_put(host->dbclk);

		mmc_free_host(host->mmc);
		iounmap(host->base);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, res->end - res->start + 1);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int omap_mmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	if (host && host->suspended)
		return 0;

	if (host) {
		host->suspended = 1;
		if (host->pdata->suspend) {
			ret = host->pdata->suspend(&pdev->dev,
						host->slot_id);
			if (ret) {
				dev_dbg(mmc_dev(host->mmc),
					"Unable to handle MMC board"
					" level suspend\n");
				host->suspended = 0;
				return ret;
			}
		}
		cancel_work_sync(&host->mmc_carddetect_work);

		ret = mmc_suspend_host(host->mmc, state);
		if (ret == 0) {
			omap_hsmmc_enable_clks(host);

			OMAP_HSMMC_WRITE(host->base, ISE, 0);
			OMAP_HSMMC_WRITE(host->base, IE, 0);

			if (host->id == OMAP_MMC1_DEVID
					&& !(OMAP_HSMMC_READ(host->base, HCTL)
							& SDVSDET)) {
				OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base, HCTL)
					& SDVSCLR);
				OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base, HCTL)
					| SDVS30);
				OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base, HCTL)
					| SDBP);
			}

			omap_hsmmc_disable_clks(host);
		} else {
			host->suspended = 0;
			if (host->pdata->resume) {
				ret = host->pdata->resume(&pdev->dev,
					host->slot_id);
				if (ret)
					dev_dbg(mmc_dev(host->mmc),
						"Unmask interrupt failed\n");
			}
		}

	}
	return ret;
}

/* Routine to resume the MMC device */
static int omap_mmc_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	if (host && !host->suspended)
		return 0;

	if (host) {
		if (host->pdata->resume) {
			ret = host->pdata->resume(&pdev->dev, host->slot_id);
			if (ret)
				dev_dbg(mmc_dev(host->mmc),
					"Unmask interrupt failed\n");
		}

		/* Notify the core to resume the host */
		ret = mmc_resume_host(host->mmc);
		if (ret == 0)
			host->suspended = 0;
	}

	return ret;
}

#else
#define omap_mmc_suspend	NULL
#define omap_mmc_resume		NULL
#endif

static void omap_mmc_shutdown(struct platform_device *pdev)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "shutting down mmc\n");
	mmc_flush_scheduled_work();
	host->shutdown = 1;
	cancel_delayed_work(&host->mmc->detect);
	mmc_flush_scheduled_work();

	msleep(1600);

}	

static struct platform_driver omap_mmc_driver = {
	.probe		= omap_mmc_probe,
	.remove		= omap_mmc_remove,
	.suspend	= omap_mmc_suspend,
	.resume		= omap_mmc_resume,
	.shutdown	= omap_mmc_shutdown,
	.driver		= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init omap_mmc_init(void)
{
	/* Register the MMC driver */
	return platform_driver_register(&omap_mmc_driver);
}

static void __exit omap_mmc_cleanup(void)
{
	/* Unregister MMC driver */
	platform_driver_unregister(&omap_mmc_driver);
}

module_init(omap_mmc_init);
module_exit(omap_mmc_cleanup);

MODULE_DESCRIPTION("OMAP High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
