/*
 *  LowRISC SD Host Controller Interface driver
 *
 *  Ported for Sonata SoC (RV32, TL-UL bus, 32-bit register access)
 *
 *  Copyright (C) 2018 LowRISC CIC
 *  Copyright (C) 2024 Jonathan Kimmitt
 *
 *    Based on toshsd.c
 *    Copyright (C) 2014 Ondrej Zary
 *    Copyright (C) 2007 Richard Betts, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "lowrisc_sd.h"

#define DRIVER_NAME "lowrisc-sd"
#define DEBUG

static inline void sd_write(struct lowrisc_sd_host *host, u32 reg, u32 val)
{
	writel(val, host->ioaddr + reg);
}

static inline u32 sd_read(struct lowrisc_sd_host *host, u32 reg)
{
	return readl(host->ioaddr + reg);
}

static void sd_irq_en(struct lowrisc_sd_host *host, int mask)
{
	sd_write(host, SD_IRQ_EN_REG, mask);
	host->int_en = mask;
}

static void lowrisc_sd_init(struct lowrisc_sd_host *host)
{
	/* Mask all IRQs and clear any pending status */
	sd_write(host, SD_IRQ_EN_REG, 0);
	sd_write(host, SD_IRQ_CLR_REG, 0xf);
	host->int_en = 0;

	/* Reset the SD protocol engine (assert then deassert resets) */
	sd_write(host, SD_RESET_REG, 0);  /* Assert all resets */
	udelay(10);
	sd_write(host, SD_RESET_REG, 0x7);  /* Deassert: clk_rst | data_rst | cmd_rst */
}

static void __lowrisc_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		mdelay(1);
		break;
	case MMC_POWER_UP:
	case MMC_POWER_ON:
		break;
	}

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		host->width_setting = 0;
		break;
	case MMC_BUS_WIDTH_4:
		host->width_setting = 0x20;
		break;
	}
}

static void lowrisc_sd_finish_request(struct lowrisc_sd_host *host)
{
	struct mmc_request *mrq = host->mrq;

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	sd_write(host, SD_RESET_REG, 0);    /* Assert all resets */
	sd_write(host, SD_START_REG, 0);
	sd_write(host, SD_RESET_REG, 0x7);  /* Deassert all resets */
	mmc_request_done(host->mmc, mrq);
}

static void lowrisc_sd_cmd_irq(struct lowrisc_sd_host *host)
{
	struct mmc_command *cmd = host->cmd;

	if (!cmd) {
		dev_warn(&host->pdev->dev, "Spurious CMD irq\n");
		return;
	}
	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT && cmd->flags & MMC_RSP_136) {
		int i;
		/* R2 — 136-bit response */
		for (i = 0; i < 4; i++) {
			cmd->resp[i] = sd_read(host, SD_RESP0 + (3 - i) * 4) << 8;
			if (i != 3)
				cmd->resp[i] |= sd_read(host, SD_RESP0 + (2 - i) * 4) >> 24;
		}
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		/* R1, R1B, R3, R6, R7 */
		cmd->resp[0] = sd_read(host, SD_RESP0);
	}

	if (host->data)
		host->int_en |= SD_CARD_RW_END;
	else
		lowrisc_sd_finish_request(host);
}

static void lowrisc_sd_next_block(struct lowrisc_sd_host *host);

static void lowrisc_sd_read_block(struct lowrisc_sd_host *host)
{
	void __iomem *buf_base = host->ioaddr + SD_DATA_BUF_OFFSET;
	size_t blksize = host->data->blksz;
	int len;

	BUG_ON(!sg_miter_next(&host->sg_miter));
	BUG_ON(host->sg_miter.length < blksize);

	if (!((sizeof(u32) - 1) & (size_t)(host->sg_miter.addr))) {
		u32 *buf = (u32 *)(host->sg_miter.addr);
		for (len = blksize; len > 0; len -= sizeof(u32))
			*buf++ = readl(buf_base + (blksize - len));
	} else {
		u8 *buf = host->sg_miter.addr;
		for (len = blksize; len > 0; len -= sizeof(u32)) {
			u32 scratch = readl(buf_base + (blksize - len));
			memcpy(buf, &scratch, sizeof(u32));
			buf += sizeof(u32);
		}
	}
	host->sg_miter.consumed = blksize;
	sg_miter_stop(&host->sg_miter);
}

static void lowrisc_sd_data_end_irq(struct lowrisc_sd_host *host)
{
	struct mmc_data *data = host->data;

	if (!data) {
		dev_warn(&host->pdev->dev, "Spurious data end IRQ\n");
		return;
	}

	if (data->flags & MMC_DATA_READ)
		lowrisc_sd_read_block(host);

	host->blocks_remaining--;
	host->block_offset++;

	if (host->blocks_remaining > 0 && data->error == 0) {
		/* More blocks to transfer — issue next single-block cmd */
		lowrisc_sd_next_block(host);
		return;
	}

	/* All blocks done (or error) */
	host->data = NULL;
	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	lowrisc_sd_finish_request(host);
}

static irqreturn_t lowrisc_sd_irq(int irq, void *dev_id)
{
	struct lowrisc_sd_host *host = dev_id;
	u32 int_status, int_reg;
	int error = 0;
	irqreturn_t ret = IRQ_HANDLED;

	spin_lock(&host->lock);
	int_status = sd_read(host, SD_IRQ_STAT_RESP);
	int_reg = int_status & host->int_en;

	if (!int_reg) {
		ret = IRQ_NONE;
		goto irq_end;
	}

	/* Clear handled interrupt bits (write-1-to-clear) */
	sd_write(host, SD_IRQ_CLR_REG, int_reg);

	/* Check for timeout only on command completion — the wait counter
	 * is only meaningful for the command that just finished.
	 */
	if ((int_reg & SD_CARD_RESP_END) &&
	    sd_read(host, SD_WAIT_RESP) >= sd_read(host, SD_TIMEOUT_RESP)) {
		error = -ETIMEDOUT;
		dev_info(&host->pdev->dev, "IRQ: timeout error\n");
		if (host->cmd)
			host->cmd->error = error;
		sd_write(host, SD_START_REG, 0);
		sd_write(host, SD_SETTING_REG, 0);
	}

	/* Card insert/remove */
	if (int_reg & SD_CARD_CARD_REMOVED) {
		int mask = (host->int_en & ~SD_CARD_CARD_REMOVED) | SD_CARD_CARD_INSERTED;
		sd_irq_en(host, mask);
		mmc_detect_change(host->mmc, 1);
	}

	if (int_reg & SD_CARD_CARD_INSERTED) {
		int mask = (host->int_en & ~SD_CARD_CARD_INSERTED) | SD_CARD_CARD_REMOVED;
		sd_irq_en(host, mask);
		lowrisc_sd_init(host);
		mmc_detect_change(host->mmc, 1);
	}

	/* Command completion */
	if (int_reg & SD_CARD_RESP_END) {
		lowrisc_sd_cmd_irq(host);
		host->int_en &= ~SD_CARD_RESP_END;
	}

	/* Data transfer completion */
	if (int_reg & SD_CARD_RW_END) {
		lowrisc_sd_data_end_irq(host);
		host->int_en &= ~SD_CARD_RW_END;
	}

irq_end:
	sd_irq_en(host, host->int_en);
	spin_unlock(&host->lock);
	return ret;
}

static void lowrisc_sd_start_cmd(struct lowrisc_sd_host *host,
				 struct mmc_command *cmd)
{
	int setting = 0;
	int timeout = 100000;  /* ~0.5s at 200kHz init clock */
	struct mmc_data *data = host->data;

	if (!(cmd->flags & MMC_RSP_PRESENT))
		setting = 0;
	else if (cmd->flags & MMC_RSP_136)
		setting = 3;
	else
		setting = 1;

	setting |= host->width_setting;
	host->cmd = cmd;

	if (data) {
		setting |= 0x4;
		if (data->flags & MMC_DATA_READ)
			setting |= 0x10;
		else
			setting |= 0x8;
	}

	/* Reset cmd (and data if applicable) to clear finish signals */
	sd_write(host, SD_RESET_REG, 0);  /* Assert all resets */
	sd_write(host, SD_START_REG, 0);
	sd_write(host, SD_RESET_REG, 0x7);  /* Deassert all resets */

	/* Set up the command */
	sd_write(host, SD_ALIGN_REG, 0);
	sd_write(host, SD_ARG_REG, cmd->arg);
	sd_write(host, SD_CMD_REG, cmd->opcode);
	sd_write(host, SD_SETTING_REG, setting);
	sd_write(host, SD_TIMEOUT_REG, timeout);
	/* Start the transaction */
	sd_write(host, SD_START_REG, 1);
	sd_irq_en(host, sd_read(host, SD_IRQ_EN_RESP) | SD_CARD_RESP_END);
}

static void lowrisc_sd_write_block(struct lowrisc_sd_host *host)
{
	void __iomem *buf_base = host->ioaddr + SD_DATA_BUF_OFFSET;
	size_t blksize = host->data->blksz;
	int len;

	if (sg_miter_next(&host->sg_miter)) {
		BUG_ON(host->sg_miter.length < blksize);

		if (!((sizeof(u32) - 1) & (size_t)(host->sg_miter.addr))) {
			u32 *buf = (u32 *)(host->sg_miter.addr);
			for (len = blksize; len > 0; len -= sizeof(u32))
				writel(*buf++, buf_base + (blksize - len));
		} else {
			u8 *buf = host->sg_miter.addr;
			for (len = blksize; len > 0; len -= sizeof(u32)) {
				u32 scratch;
				memcpy(&scratch, buf, sizeof(u32));
				buf += sizeof(u32);
				writel(scratch, buf_base + (blksize - len));
			}
		}
		host->sg_miter.consumed = blksize;
		sg_miter_stop(&host->sg_miter);
	}
}

static void lowrisc_sd_start_data(struct lowrisc_sd_host *host,
				  struct mmc_data *data)
{
	unsigned int flags = SG_MITER_ATOMIC;

	host->data = data;
	host->blocks_remaining = data->blocks;
	host->block_offset = 0;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;

	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);

	/* Always tell hardware single block — we iterate in the driver */
	sd_write(host, SD_BLKCNT_REG, 1);
	sd_write(host, SD_BLKSIZE_REG, data->blksz);

	if (!(data->flags & MMC_DATA_READ))
		lowrisc_sd_write_block(host);
}

/* Issue the next single-block command for a multi-block transfer */
static void lowrisc_sd_next_block(struct lowrisc_sd_host *host)
{
	struct mmc_command *cmd = &host->block_cmd;

	/* Set up hardware for next single block */
	sd_write(host, SD_BLKCNT_REG, 1);
	sd_write(host, SD_BLKSIZE_REG, host->data->blksz);

	/* Fill buffer for writes */
	if (!(host->data->flags & MMC_DATA_READ))
		lowrisc_sd_write_block(host);

	/* Synthesise a single-block command */
	memset(cmd, 0, sizeof(*cmd));
	if (host->data->flags & MMC_DATA_READ)
		cmd->opcode = MMC_READ_SINGLE_BLOCK;
	else
		cmd->opcode = MMC_WRITE_BLOCK;
	cmd->arg = host->orig_arg + host->block_offset;
	cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	lowrisc_sd_start_cmd(host, cmd);
}

static void lowrisc_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	unsigned long flags;

	/* Abort if card not present */
	if (sd_read(host, SD_DETECT_RESP)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);
	host->mrq = mrq;

	if (mrq->data) {
		host->orig_arg = mrq->cmd->arg;
		lowrisc_sd_start_data(host, mrq->data);

		/* Convert multi-block commands to single-block */
		if (mrq->cmd->opcode == MMC_READ_MULTIPLE_BLOCK)
			mrq->cmd->opcode = MMC_READ_SINGLE_BLOCK;
		else if (mrq->cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)
			mrq->cmd->opcode = MMC_WRITE_BLOCK;
	}

	lowrisc_sd_start_cmd(host, mrq->cmd);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void lowrisc_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	__lowrisc_sd_set_ios(mmc, ios);
	spin_unlock_irqrestore(&host->lock, flags);
}

static int lowrisc_sd_get_ro(struct mmc_host *mmc)
{
	/* No write-protect detection on Sonata */
	return 0;
}

static int lowrisc_sd_get_cd(struct mmc_host *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	return !sd_read(host, SD_DETECT_RESP);
}

static struct mmc_host_ops lowrisc_sd_ops = {
	.request = lowrisc_sd_request,
	.set_ios = lowrisc_sd_set_ios,
	.get_ro  = lowrisc_sd_get_ro,
	.get_cd  = lowrisc_sd_get_cd,
};

static int lowrisc_sd_probe(struct platform_device *pdev)
{
	int ret;
	struct lowrisc_sd_host *host;
	struct mmc_host *mmc;
	struct resource *iomem;
	int irq;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	mmc = mmc_alloc_host(sizeof(struct lowrisc_sd_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;

	host->ioaddr = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		goto free_host;
	}

	{
		u32 ver = readl(host->ioaddr + SD_VERSION_RESP);
		dev_info(&pdev->dev, "SD controller mapped at %pR, RTL version 0x%08x\n",
			 iomem, ver);
	}

	/* Set MMC host parameters — 1-bit bus only (DAT1/DAT2 not connected) */
	mmc->ops = &lowrisc_sd_ops;
	mmc->caps = 0;  /* No 4-bit support */
	mmc->caps2 = MMC_CAP2_NO_SDIO;  /* Skip SDIO probe (CMD52) */
	mmc->ocr_avail = MMC_VDD_32_33;
	mmc->f_min = 400000;     /* 400 kHz for init */
	mmc->f_max = 15000000;   /* 30 MHz / 2 */
	mmc->max_blk_count = 8;  /* Multi-block broken into single-block HW ops */
	mmc->max_blk_size = 512;
	mmc->max_req_size = 4096;  /* Must be >= PAGE_SIZE for block layer */
	mmc->max_seg_size = 4096;

	spin_lock_init(&host->lock);
	platform_set_drvdata(pdev, host);

	lowrisc_sd_init(host);

	ret = devm_request_irq(&pdev->dev, irq, lowrisc_sd_irq, IRQF_SHARED,
			       DRIVER_NAME, host);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ %d\n", irq);
		goto free_host;
	}

	ret = mmc_add_host(mmc);
	if (ret)
		goto free_host;

	dev_info(&pdev->dev, "LowRISC SD host controller, IRQ %d\n", irq);
	sd_irq_en(host, SD_CARD_CARD_INSERTED | SD_CARD_CARD_REMOVED);
	return 0;

free_host:
	mmc_free_host(mmc);
	return ret;
}

static void lowrisc_sd_remove(struct platform_device *pdev)
{
	struct lowrisc_sd_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
	sd_write(host, SD_IRQ_EN_REG, 0);  /* Mask all interrupts */
	mmc_free_host(host->mmc);
}

static const struct of_device_id lowrisc_sd_of_match[] = {
	{ .compatible = "lowrisc,sd-host" },
	{ }
};

MODULE_DEVICE_TABLE(of, lowrisc_sd_of_match);

static struct platform_driver lowrisc_sd_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = lowrisc_sd_of_match,
	},
	.probe  = lowrisc_sd_probe,
	.remove = lowrisc_sd_remove,
};

module_platform_driver(lowrisc_sd_driver);

MODULE_AUTHOR("Jonathan Kimmitt");
MODULE_DESCRIPTION("LowRISC SD Host Controller Interface driver");
MODULE_LICENSE("GPL");
