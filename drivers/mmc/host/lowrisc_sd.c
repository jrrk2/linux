/*
 *  LowRISC SD Host Controller Interface driver
 *
 *  Ported for Sonata SoC (RV32, TL-UL bus, 32-bit register access)
 *  Polling mode — all SD transactions are synchronous.
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
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "lowrisc_sd.h"

#define DRIVER_NAME "lowrisc-sd"

static inline void sd_write(struct lowrisc_sd_host *host, u32 reg, u32 val)
{
	writel(val, host->ioaddr + reg);
}

static inline u32 sd_read(struct lowrisc_sd_host *host, u32 reg)
{
	return readl(host->ioaddr + reg);
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

/* System clock frequency feeding the SD clock divider */
#define SD_SYS_CLK_HZ	30000000

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

	/* Set SD clock divider: sd_clk = SYS_CLK / (2 * (divider + 1)) */
	if (ios->clock) {
		unsigned int divider = SD_SYS_CLK_HZ / (2 * ios->clock);

		if (divider > 0)
			divider--;
		if (divider > 255)
			divider = 255;
		sd_write(host, SD_CLK_DIV_REG, divider);
		dev_info(&host->pdev->dev, "clock %u Hz, divider %u\n",
			 SD_SYS_CLK_HZ / (2 * (divider + 1)), divider);
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

static void lowrisc_sd_read_response(struct lowrisc_sd_host *host,
				     struct mmc_command *cmd)
{
	if (cmd->flags & MMC_RSP_PRESENT && cmd->flags & MMC_RSP_136) {
		int i;
		/* R2 -- 136-bit response */
		for (i = 0; i < 4; i++) {
			cmd->resp[i] = sd_read(host, SD_RESP0 + (3 - i) * 4) << 8;
			if (i != 3)
				cmd->resp[i] |= sd_read(host, SD_RESP0 + (2 - i) * 4) >> 24;
		}
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		/* R1, R1B, R3, R6, R7 */
		cmd->resp[0] = sd_read(host, SD_RESP0);
	}
}

/*
 * Poll for command completion (RESP_END bit in IRQ status).
 * Returns 0 on success, -ETIMEDOUT on timeout.
 */
static int lowrisc_sd_poll_cmd(struct lowrisc_sd_host *host)
{
	int i;

	for (i = 0; i < 500000; i++) {
		u32 status = sd_read(host, SD_IRQ_STAT_RESP);

		if (status & SD_CARD_RESP_END) {
			sd_write(host, SD_IRQ_CLR_REG, SD_CARD_RESP_END);
			/* Check hardware timeout counter */
			if (sd_read(host, SD_WAIT_RESP) >=
			    sd_read(host, SD_TIMEOUT_RESP))
				return -ETIMEDOUT;
			return 0;
		}
		udelay(1);
	}
	return -ETIMEDOUT;
}

/*
 * Poll for data transfer completion (RW_END bit in IRQ status).
 * Returns 0 on success, -ETIMEDOUT on timeout.
 */
static int lowrisc_sd_poll_data(struct lowrisc_sd_host *host)
{
	int i;

	/* Data transfers can take much longer (multi-block reads) */
	for (i = 0; i < 2000000; i++) {
		u32 status = sd_read(host, SD_IRQ_STAT_RESP);

		if (status & SD_CARD_RW_END) {
			sd_write(host, SD_IRQ_CLR_REG, SD_CARD_RW_END);
			return 0;
		}
		udelay(1);
	}
	return -ETIMEDOUT;
}

/*
 * Send a command to the SD controller and start it.
 * Does NOT wait for completion — caller must poll.
 */
static void lowrisc_sd_send_cmd(struct lowrisc_sd_host *host,
				struct mmc_command *cmd,
				struct mmc_data *data)
{
	int setting = 0;
	int timeout = 100000;  /* ~0.5s at 200kHz init clock */

	if (!(cmd->flags & MMC_RSP_PRESENT))
		setting = 0;
	else if (cmd->flags & MMC_RSP_136)
		setting = 3;
	else
		setting = 1;

	setting |= host->width_setting;

	if (data) {
		setting |= 0x4;
		if (data->flags & MMC_DATA_READ)
			setting |= 0x10;
		else
			setting |= 0x8;
	}

	/* Reset cmd (and data if applicable) to clear finish signals */
	sd_write(host, SD_RESET_REG, 0);    /* Assert all resets */
	sd_write(host, SD_START_REG, 0);
	sd_write(host, SD_RESET_REG, 0x7);  /* Deassert all resets */
	sd_write(host, SD_IRQ_CLR_REG, 0xf); /* Clear any stale status */

	/* Set up the command */
	sd_write(host, SD_ALIGN_REG, 0);
	sd_write(host, SD_ARG_REG, cmd->arg);
	sd_write(host, SD_CMD_REG, cmd->opcode);
	sd_write(host, SD_SETTING_REG, setting);
	sd_write(host, SD_TIMEOUT_REG, timeout);
	/* Start the transaction */
	sd_write(host, SD_START_REG, 1);
}

static void lowrisc_sd_write_data(struct lowrisc_sd_host *host,
				  struct mmc_data *data,
				  struct sg_mapping_iter *sg_miter)
{
	void __iomem *buf_base = host->ioaddr + SD_DATA_BUF_OFFSET;
	size_t total = data->blocks * data->blksz;
	size_t offset = 0;

	while (sg_miter_next(sg_miter) && offset < total) {
		size_t len = min(sg_miter->length, total - offset);
		u32 *src = sg_miter->addr;
		size_t i;

		for (i = 0; i < len; i += sizeof(u32))
			writel(*src++, buf_base + offset + i);
		sg_miter->consumed = len;
		offset += len;
	}
	sg_miter_stop(sg_miter);
}

static void lowrisc_sd_read_data(struct lowrisc_sd_host *host,
				 struct mmc_data *data,
				 struct sg_mapping_iter *sg_miter)
{
	void __iomem *buf_base = host->ioaddr + SD_DATA_BUF_OFFSET;
	size_t total = data->blocks * data->blksz;
	size_t offset = 0;

	while (sg_miter_next(sg_miter) && offset < total) {
		size_t len = min(sg_miter->length, total - offset);
		u32 *dst = sg_miter->addr;
		size_t i;

		for (i = 0; i < len; i += sizeof(u32))
			*dst++ = readl(buf_base + offset + i);
		sg_miter->consumed = len;
		offset += len;
	}
	sg_miter_stop(sg_miter);
}

static void lowrisc_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct sg_mapping_iter sg_miter;
	int ret;

	/* Abort if card not present */
	if (sd_read(host, SD_DETECT_RESP)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	/* CMD23 (set block count) if present */
	if (mrq->sbc) {
		lowrisc_sd_send_cmd(host, mrq->sbc, NULL);
		ret = lowrisc_sd_poll_cmd(host);
		if (ret) {
			mrq->sbc->error = ret;
			dev_err(&host->pdev->dev, "CMD%d timeout\n",
				mrq->sbc->opcode);
			goto done;
		}
		lowrisc_sd_read_response(host, mrq->sbc);
	}

	/* Set up data transfer if present */
	if (data) {
		unsigned int flags = SG_MITER_ATOMIC;

		if (data->flags & MMC_DATA_READ)
			flags |= SG_MITER_TO_SG;
		else
			flags |= SG_MITER_FROM_SG;

		sg_miter_start(&sg_miter, data->sg, data->sg_len, flags);

		sd_write(host, SD_BLKCNT_REG, data->blocks);
		sd_write(host, SD_BLKSIZE_REG, data->blksz);

		/* For writes, fill the buffer before sending the command */
		if (!(data->flags & MMC_DATA_READ))
			lowrisc_sd_write_data(host, data, &sg_miter);
	}

	/* Send the main command */
	lowrisc_sd_send_cmd(host, mrq->cmd, data);
	ret = lowrisc_sd_poll_cmd(host);
	if (ret) {
		mrq->cmd->error = ret;
		if (mrq->cmd->opcode != 8 /* SD_SEND_IF_COND */ &&
		    mrq->cmd->opcode != MMC_APP_CMD)
			dev_err(&host->pdev->dev, "CMD%d timeout\n",
				mrq->cmd->opcode);
		if (data)
			sg_miter_stop(&sg_miter);
		goto done;
	}
	lowrisc_sd_read_response(host, mrq->cmd);

	/* Wait for data transfer completion */
	if (data) {
		ret = lowrisc_sd_poll_data(host);
		if (ret) {
			data->error = ret;
			dev_err(&host->pdev->dev,
				"data timeout on CMD%d (%u blocks)\n",
				mrq->cmd->opcode, data->blocks);
			sg_miter_stop(&sg_miter);
			goto done;
		}

		/* Read data from buffer after transfer */
		if (data->flags & MMC_DATA_READ)
			lowrisc_sd_read_data(host, data, &sg_miter);

		data->bytes_xfered = data->blocks * data->blksz;
	}

	/* Stop command (CMD12) if present — skip when CMD23 was used,
	 * since the card auto-stops after the pre-defined block count.
	 */
	if (mrq->stop && !mrq->sbc) {
		lowrisc_sd_send_cmd(host, mrq->stop, NULL);
		ret = lowrisc_sd_poll_cmd(host);
		if (ret) {
			mrq->stop->error = ret;
			dev_err(&host->pdev->dev, "CMD%d (stop) timeout\n",
				mrq->stop->opcode);
			goto done;
		}
		lowrisc_sd_read_response(host, mrq->stop);
	}

done:
	/* Reset the SD engine */
	sd_write(host, SD_RESET_REG, 0);
	sd_write(host, SD_START_REG, 0);
	sd_write(host, SD_RESET_REG, 0x7);
	mmc_request_done(mmc, mrq);
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

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem)
		return -EINVAL;

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

	/* Set MMC host parameters */
	mmc->ops = &lowrisc_sd_ops;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_CMD23;
	mmc->caps2 = MMC_CAP2_NO_SDIO;  /* Skip SDIO probe (CMD52) */
	mmc->ocr_avail = MMC_VDD_32_33;
	mmc->f_min = 400000;     /* 400 kHz for init */
	mmc->f_max = 15000000;   /* 30 MHz / 2 */
	mmc->max_blk_count = 8;  /* 8 x 512 = 4KB hardware buffer limit */
	mmc->max_blk_size = 512;
	mmc->max_req_size = 4096;  /* Must be >= PAGE_SIZE for block layer */
	mmc->max_seg_size = 4096;

	spin_lock_init(&host->lock);
	platform_set_drvdata(pdev, host);

	lowrisc_sd_init(host);

	ret = mmc_add_host(mmc);
	if (ret)
		goto free_host;

	dev_info(&pdev->dev, "LowRISC SD host controller (polling mode)\n");
	return 0;

free_host:
	mmc_free_host(mmc);
	return ret;
}

static void lowrisc_sd_remove(struct platform_device *pdev)
{
	struct lowrisc_sd_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
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
