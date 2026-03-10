// SPDX-License-Identifier: GPL-2.0
/*
 * LiteX SD card driver — polling mode
 *
 * Replaces the upstream litex_mmc driver which hangs/fails due to:
 *   1. IRQ wait_for_completion that never completes
 *   2. PHY bus width never written after ACMD6 (data corruption)
 *   3. litex_read8/write8 mismatches with native-32-bit CSR registers
 *
 * Uses direct writel/readl for native 32-bit CSR registers.
 * Register map and init sequence proven by sdtest userspace tool.
 *
 * Requires FPGA built with --with-coherent-dma so the SD card DMA
 * engine routes through the CPU cache. Supports direct scatter-gather
 * DMA (zero-copy) when buffers are physically contiguous, falling back
 * to a coherent bounce buffer otherwise.
 */

#define LITEX_SD_VERSION "1.2"

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

/* Register offsets from base (flat map, proven by sdtest) */
#define SD_PHY_CARD_DET   0x00
#define SD_PHY_CLK_DIV    0x04
#define SD_PHY_INIT       0x08
#define SD_PHY_SETTINGS   0x18  /* 0=1-bit, 1=4-bit */

#define SD_CMD_ARG        0x1C
#define SD_CMD_CMD        0x20  /* opcode<<8 | xfer<<5 | rsp_type */
#define SD_CMD_SEND       0x24
#define SD_CMD_RSP0       0x28
#define SD_CMD_RSP1       0x2C
#define SD_CMD_RSP2       0x30
#define SD_CMD_RSP3       0x34  /* Short response value */
#define SD_CMD_EVENT      0x38
#define SD_DATA_EVENT     0x3C

#define SD_BLK_LENGTH     0x40
#define SD_BLK_COUNT      0x44

#define SD_RD_BASE_HI     0x48
#define SD_RD_BASE_LO     0x4C
#define SD_RD_LENGTH      0x50
#define SD_RD_ENABLE      0x54
#define SD_RD_DONE        0x58

#define SD_WR_BASE_HI     0x64
#define SD_WR_BASE_LO     0x68
#define SD_WR_LENGTH      0x6C
#define SD_WR_ENABLE      0x70
#define SD_WR_DONE        0x74

/* Transfer types (bits [6:5] of CMD register) */
#define SD_XFER_NONE      0
#define SD_XFER_READ      1
#define SD_XFER_WRITE     2

/* Response types (bits [1:0] of CMD register) */
#define SD_RSP_NONE       0
#define SD_RSP_SHORT      1
#define SD_RSP_LONG       2
#define SD_RSP_SHORT_BUSY 3

/* Event register bits */
#define SD_EVT_DONE       BIT(0)
#define SD_EVT_TIMEOUT    BIT(2)
#define SD_EVT_CRC_ERR    BIT(3)

#define SD_CMD_TIMEOUT_US  100000   /* 100ms */
#define SD_DATA_TIMEOUT_US 2000000  /* 2s */

#define DMA_BUF_SIZE       PAGE_SIZE

struct litex_sd_host {
	struct mmc_host *mmc;
	void __iomem *regs;
	void *dma_buf;         /* Coherent bounce buffer */
	dma_addr_t dma_phys;   /* Physical address of bounce buffer */
	unsigned int ref_clk;
};

static inline void sd_write(struct litex_sd_host *host, u32 off, u32 val)
{
	writel(val, host->regs + off);
}

static inline u32 sd_read(struct litex_sd_host *host, u32 off)
{
	return readl(host->regs + off);
}

static int sd_wait_evt(struct litex_sd_host *host, u32 reg, u32 timeout_us)
{
	u32 evt;
	int ret;

	ret = readl_poll_timeout(host->regs + reg, evt, evt & SD_EVT_DONE,
				 5, timeout_us);
	if (ret)
		return -ETIMEDOUT;
	if (evt & SD_EVT_TIMEOUT)
		return -ETIMEDOUT;
	if (evt & SD_EVT_CRC_ERR)
		return -EILSEQ;
	return 0;
}

static u32 sd_rsp_type(struct mmc_command *cmd)
{
	if (cmd->flags & MMC_RSP_136)
		return SD_RSP_LONG;
	if (!(cmd->flags & MMC_RSP_PRESENT))
		return SD_RSP_NONE;
	if (cmd->flags & MMC_RSP_BUSY)
		return SD_RSP_SHORT_BUSY;
	return SD_RSP_SHORT;
}

static void litex_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct litex_sd_host *host = mmc_priv(mmc);
	struct device *dev = mmc_dev(mmc);
	struct mmc_command *sbc = mrq->sbc;
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	struct mmc_command *stop = mrq->stop;
	u32 rsp = sd_rsp_type(cmd);
	u32 xfer = SD_XFER_NONE;
	unsigned int len = 0;
	unsigned int retries = cmd->retries;
	dma_addr_t dma_addr = host->dma_phys;
	bool direct = false;
	int ret;

	/* Check card presence */
	if (sd_read(host, SD_PHY_CARD_DET)) {
		cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	/* Send set-block-count (CMD23) if provided */
	if (sbc) {
		sd_write(host, SD_CMD_ARG, sbc->arg);
		sd_write(host, SD_CMD_CMD,
			 (sbc->opcode << 8) | sd_rsp_type(sbc));
		sd_write(host, SD_CMD_SEND, 1);
		sbc->error = sd_wait_evt(host, SD_CMD_EVENT,
					 SD_CMD_TIMEOUT_US);
		if (sbc->error) {
			mmc_request_done(mmc, mrq);
			return;
		}
		sbc->resp[0] = sd_read(host, SD_CMD_RSP3);
	}

	/* Set up DMA for data transfers */
	if (data) {
		int sg_count;

		len = data->blksz * data->blocks;
		sd_write(host, SD_BLK_LENGTH, data->blksz);
		sd_write(host, SD_BLK_COUNT, data->blocks);

		/* Try direct DMA to/from sg buffer (zero-copy).
		 * With max_segs=1, MMC core always gives a single segment,
		 * so this path should always succeed.
		 */
		sg_count = dma_map_sg(dev, data->sg, data->sg_len,
				      mmc_get_dma_dir(data));
		if (sg_count == 1 && sg_dma_len(data->sg) >= len) {
			dma_addr = sg_dma_address(data->sg);
			direct = true;
		} else if (len > DMA_BUF_SIZE) {
			dev_err(dev, "no direct DMA and xfer too large: %u\n",
				len);
			dma_unmap_sg(dev, data->sg, data->sg_len,
				     mmc_get_dma_dir(data));
			cmd->error = -EINVAL;
			mmc_request_done(mmc, mrq);
			return;
		}

		if (data->flags & MMC_DATA_READ) {
			xfer = SD_XFER_READ;
			sd_write(host, SD_RD_ENABLE, 0);
			sd_write(host, SD_RD_BASE_HI, 0);
			sd_write(host, SD_RD_BASE_LO, dma_addr);
			sd_write(host, SD_RD_LENGTH, len);
			sd_write(host, SD_RD_ENABLE, 1);
		} else {
			xfer = SD_XFER_WRITE;
			if (!direct)
				sg_copy_to_buffer(data->sg, data->sg_len,
						  host->dma_buf, len);
			sd_write(host, SD_WR_ENABLE, 0);
			sd_write(host, SD_WR_BASE_HI, 0);
			sd_write(host, SD_WR_BASE_LO, dma_addr);
			sd_write(host, SD_WR_LENGTH, len);
			sd_write(host, SD_WR_ENABLE, 1);
		}
	}

	/* Send command with retries */
	do {
		sd_write(host, SD_CMD_ARG, cmd->arg);
		sd_write(host, SD_CMD_CMD,
			 (cmd->opcode << 8) | (xfer << 5) | rsp);
		sd_write(host, SD_CMD_SEND, 1);
		ret = sd_wait_evt(host, SD_CMD_EVENT, SD_CMD_TIMEOUT_US);
	} while (ret && retries-- > 0);

	if (ret) {
		dev_dbg(dev, "CMD%d error: %d\n", cmd->opcode, ret);
		cmd->error = ret;
		goto out;
	}

	/* Read response */
	if (rsp == SD_RSP_SHORT || rsp == SD_RSP_SHORT_BUSY)
		cmd->resp[0] = sd_read(host, SD_CMD_RSP3);
	else if (rsp == SD_RSP_LONG) {
		cmd->resp[0] = sd_read(host, SD_CMD_RSP0);
		cmd->resp[1] = sd_read(host, SD_CMD_RSP1);
		cmd->resp[2] = sd_read(host, SD_CMD_RSP2);
		cmd->resp[3] = sd_read(host, SD_CMD_RSP3);
	}

	/* Handle data transfer */
	if (data) {
		u32 done;

		ret = sd_wait_evt(host, SD_DATA_EVENT, SD_DATA_TIMEOUT_US);
		if (ret) {
			dev_err(dev, "CMD%d data error: %d\n",
				cmd->opcode, ret);
			data->error = ret;
			goto out;
		}

		/* Wait for DMA completion */
		if (xfer == SD_XFER_READ)
			ret = readl_poll_timeout(host->regs + SD_RD_DONE,
						 done, done & 1,
						 5, SD_DATA_TIMEOUT_US);
		else
			ret = readl_poll_timeout(host->regs + SD_WR_DONE,
						 done, done & 1,
						 5, SD_DATA_TIMEOUT_US);
		if (ret) {
			dev_err(dev, "CMD%d DMA timeout\n", cmd->opcode);
			data->error = -ETIMEDOUT;
			goto out;
		}

		if (xfer == SD_XFER_READ && !direct)
			sg_copy_from_buffer(data->sg, data->sg_len,
					    host->dma_buf, len);
		data->bytes_xfered = len;
	}

out:
	if (data)
		dma_unmap_sg(dev, data->sg, data->sg_len,
			     mmc_get_dma_dir(data));

	if (stop && (cmd->error || !sbc)) {
		sd_write(host, SD_CMD_ARG, stop->arg);
		sd_write(host, SD_CMD_CMD,
			 (stop->opcode << 8) | SD_RSP_SHORT_BUSY);
		sd_write(host, SD_CMD_SEND, 1);
		stop->error = sd_wait_evt(host, SD_CMD_EVENT,
					  SD_CMD_TIMEOUT_US);
		if (!stop->error)
			stop->resp[0] = sd_read(host, SD_CMD_RSP3);
	}

	mmc_request_done(mmc, mrq);
}

static void litex_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct litex_sd_host *host = mmc_priv(mmc);

	if (ios->clock) {
		u32 div = DIV_ROUND_UP(host->ref_clk, ios->clock);

		div = roundup_pow_of_two(div);
		div = clamp(div, 2U, 256U);
		sd_write(host, SD_PHY_CLK_DIV, div);
	}

	/* Set PHY bus width — the critical fix missing from upstream */
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		sd_write(host, SD_PHY_SETTINGS, 1);
	else
		sd_write(host, SD_PHY_SETTINGS, 0);
}

static int litex_sd_get_cd(struct mmc_host *mmc)
{
	struct litex_sd_host *host = mmc_priv(mmc);

	/* PHY_CARD_DET: 0=inserted, 1=removed */
	return !sd_read(host, SD_PHY_CARD_DET);
}

static const struct mmc_host_ops litex_sd_ops = {
	.request = litex_sd_request,
	.set_ios = litex_sd_set_ios,
	.get_cd  = litex_sd_get_cd,
};

static int litex_sd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct litex_sd_host *host;
	struct mmc_host *mmc;
	struct clk *clk;
	int ret;

	mmc = mmc_alloc_host(sizeof(*host), dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->regs)) {
		ret = PTR_ERR(host->regs);
		goto err_free;
	}

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		ret = dev_err_probe(dev, PTR_ERR(clk), "no clock\n");
		goto err_free;
	}
	host->ref_clk = clk_get_rate(clk);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto err_free;

	host->dma_buf = dma_alloc_coherent(dev, DMA_BUF_SIZE,
					    &host->dma_phys, GFP_KERNEL);
	if (!host->dma_buf) {
		ret = -ENOMEM;
		goto err_free;
	}

	mmc->ops        = &litex_sd_ops;
	mmc->f_min      = 400000;
	mmc->f_max      = 50000000;
	mmc->ocr_avail  = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps       = MMC_CAP_4_BIT_DATA | MMC_CAP_NEEDS_POLL |
			  MMC_CAP_CMD23 | MMC_CAP_WAIT_WHILE_BUSY;
	mmc->caps2      = MMC_CAP2_NO_SDIO | MMC_CAP2_NO_WRITE_PROTECT |
			  MMC_CAP2_NO_MMC;
	mmc->max_blk_size  = 512;
	mmc->max_blk_count = 128;              /* 64KB per request */
	mmc->max_req_size  = 128 * 512;
	mmc->max_seg_size  = 128 * 512;
	mmc->max_segs      = 1;               /* Single sg = always direct DMA */

	/* Initialize hardware: 1-bit, slow clock, DMA off */
	sd_write(host, SD_PHY_SETTINGS, 0);
	sd_write(host, SD_RD_ENABLE, 0);
	sd_write(host, SD_WR_ENABLE, 0);
	sd_write(host, SD_PHY_CLK_DIV, 128);
	sd_write(host, SD_PHY_INIT, 1);
	usleep_range(1000, 2000);

	platform_set_drvdata(pdev, host);

	ret = mmc_add_host(mmc);
	if (ret)
		goto err_dma;

	dev_info(dev, "LiteX SD v" LITEX_SD_VERSION
		 " (polling, coherent DMA, ref_clk=%uHz)\n", host->ref_clk);
	return 0;

err_dma:
	dma_free_coherent(dev, DMA_BUF_SIZE, host->dma_buf, host->dma_phys);
err_free:
	mmc_free_host(mmc);
	return ret;
}

static void litex_sd_remove(struct platform_device *pdev)
{
	struct litex_sd_host *host = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	mmc_remove_host(host->mmc);
	sd_write(host, SD_RD_ENABLE, 0);
	sd_write(host, SD_WR_ENABLE, 0);
	dma_free_coherent(dev, DMA_BUF_SIZE, host->dma_buf, host->dma_phys);
	mmc_free_host(host->mmc);
}

static const struct of_device_id litex_sd_match[] = {
	{ .compatible = "litex,mmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, litex_sd_match);

static struct platform_driver litex_sd_driver = {
	.probe      = litex_sd_probe,
	.remove_new = litex_sd_remove,
	.driver = {
		.name           = "litex-mmc",
		.of_match_table = litex_sd_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(litex_sd_driver);

MODULE_DESCRIPTION("LiteX SD card driver (polling mode)");
MODULE_AUTHOR("Jonathan Sheridan");
MODULE_LICENSE("GPL v2");
