// SPDX-License-Identifier: GPL-2.0
//
// SPI controller driver for OpenTitan-derived SoCs (lowRISC Sonata)
//
// Uses memory-mapped TX/RX BRAMs for bulk data transfer.
// Hardware: spi_wb block with 2KB TX BRAM and 2KB RX BRAM.

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "spi-opentitan"

/* Register offsets */
#define OT_SPI_CFG		0x0C
#define OT_SPI_CONTROL		0x10
#define OT_SPI_STATUS		0x14
#define OT_SPI_START		0x18
#define OT_SPI_CS0		0x28

/* CFG register bits */
#define OT_SPI_CFG_CPOL		BIT(31)
#define OT_SPI_CFG_CPHA		BIT(30)
#define OT_SPI_CFG_MSB_FIRST	BIT(29)

/* STATUS register bits */
#define OT_SPI_STATUS_IDLE	BIT(26)

/* CONTROL register bits */
#define OT_SPI_CONTROL_SW_RESET	BIT(31)

/* BRAM offsets from base */
#define OT_SPI_TX_BRAM		0x1000
#define OT_SPI_RX_BRAM		0x1800

/* Max transfer size: 11-bit byte count field */
#define OT_SPI_MAX_XFER		2047

/* Polling timeout in microseconds */
#define OT_SPI_TIMEOUT_US	1000000

struct ot_spi {
	void __iomem *base;
	void __iomem *tx_buf;
	void __iomem *rx_buf;
	unsigned long clk_rate;
};

static void ot_spi_tx_bram_write(struct ot_spi *sp, const u8 *buf, int len)
{
	int i;
	u32 word;

	for (i = 0; i < len; i += 4) {
		word = 0;
		if (i < len)
			word |= buf[i];
		if (i + 1 < len)
			word |= (u32)buf[i + 1] << 8;
		if (i + 2 < len)
			word |= (u32)buf[i + 2] << 16;
		if (i + 3 < len)
			word |= (u32)buf[i + 3] << 24;
		iowrite32(word, sp->tx_buf + i);
	}
}

static void ot_spi_tx_bram_zero(struct ot_spi *sp, int len)
{
	int i;

	for (i = 0; i < len; i += 4)
		iowrite32(0, sp->tx_buf + i);
}

static void ot_spi_rx_bram_read(struct ot_spi *sp, u8 *buf, int len)
{
	int i;
	u32 word;

	for (i = 0; i < len; i += 4) {
		word = ioread32(sp->rx_buf + i);
		if (i < len)
			buf[i] = word & 0xff;
		if (i + 1 < len)
			buf[i + 1] = (word >> 8) & 0xff;
		if (i + 2 < len)
			buf[i + 2] = (word >> 16) & 0xff;
		if (i + 3 < len)
			buf[i + 3] = (word >> 24) & 0xff;
	}
}

static void ot_spi_set_cfg(struct ot_spi *sp, struct spi_device *spi,
			    u32 speed_hz)
{
	u32 cfg = OT_SPI_CFG_MSB_FIRST;
	u16 half_clk;

	if (spi->mode & SPI_CPOL)
		cfg |= OT_SPI_CFG_CPOL;
	if (spi->mode & SPI_CPHA)
		cfg |= OT_SPI_CFG_CPHA;

	/* half_clk_period = clk_rate / (2 * speed) - 1 */
	if (speed_hz >= sp->clk_rate / 2)
		half_clk = 0;
	else
		half_clk = DIV_ROUND_UP(sp->clk_rate, 2 * speed_hz) - 1;

	cfg |= half_clk;
	iowrite32(cfg, sp->base + OT_SPI_CFG);
}

static int ot_spi_wait_idle(struct ot_spi *sp)
{
	u32 val;

	return readl_poll_timeout(sp->base + OT_SPI_STATUS, val,
				  val & OT_SPI_STATUS_IDLE, 0,
				  OT_SPI_TIMEOUT_US);
}

static int ot_spi_transfer_one_message(struct spi_controller *ctlr,
				       struct spi_message *msg)
{
	struct ot_spi *sp = spi_controller_get_devdata(ctlr);
	struct spi_device *spi = msg->spi;
	struct spi_transfer *xfer;
	int ret = 0;

	/* Configure speed/mode from first transfer (or device defaults) */
	ot_spi_set_cfg(sp, spi, msg->transfers.next ?
		       list_first_entry(&msg->transfers, struct spi_transfer,
					transfer_list)->speed_hz ?: spi->max_speed_hz
		       : spi->max_speed_hz);

	/* Assert CS */
	iowrite32(0, sp->base + OT_SPI_CS0);

	msg->actual_length = 0;

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		int len = xfer->len;

		if (len > OT_SPI_MAX_XFER) {
			ret = -EMSGSIZE;
			break;
		}

		/* Update speed if this transfer specifies one */
		if (xfer->speed_hz)
			ot_spi_set_cfg(sp, spi, xfer->speed_hz);

		/* Write TX data to BRAM */
		if (xfer->tx_buf)
			ot_spi_tx_bram_write(sp, xfer->tx_buf, len);
		else
			ot_spi_tx_bram_zero(sp, len);

		/* Trigger transfer */
		iowrite32(len, sp->base + OT_SPI_START);

		/* Wait for completion */
		ret = ot_spi_wait_idle(sp);
		if (ret) {
			dev_err(&spi->dev, "transfer timeout\n");
			break;
		}

		/* Read RX data from BRAM */
		if (xfer->rx_buf)
			ot_spi_rx_bram_read(sp, xfer->rx_buf, len);

		msg->actual_length += len;

		/* Handle cs_change: toggle CS between transfers */
		if (xfer->cs_change) {
			if (!list_is_last(&xfer->transfer_list,
					  &msg->transfers)) {
				iowrite32(1, sp->base + OT_SPI_CS0);
				iowrite32(0, sp->base + OT_SPI_CS0);
			}
		}

		spi_transfer_delay_exec(xfer);
	}

	/* Deassert CS */
	iowrite32(1, sp->base + OT_SPI_CS0);

	msg->status = ret;
	spi_finalize_current_message(ctlr);

	return 0;
}

static const struct of_device_id ot_spi_match[] = {
	{ .compatible = "lowrisc,opentitan-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, ot_spi_match);

static int ot_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct ot_spi *sp;
	void __iomem *base;
	struct clk *clk;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(clk);
	}

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(*sp));
	if (!ctlr)
		return -ENOMEM;

	sp = spi_controller_get_devdata(ctlr);
	sp->base = base;
	sp->tx_buf = base + OT_SPI_TX_BRAM;
	sp->rx_buf = base + OT_SPI_RX_BRAM;
	sp->clk_rate = clk_get_rate(clk);

	/* Software reset the SPI controller */
	iowrite32(OT_SPI_CONTROL_SW_RESET, sp->base + OT_SPI_CONTROL);

	/* Toggle CS to abort any in-progress slave transaction (e.g. from
	 * a bare-metal bootloader that used SPI before Linux booted).
	 * KSZ8851SNL datasheet: RSTN needs 10ms min after power stable.
	 */
	iowrite32(1, sp->base + OT_SPI_CS0);
	mdelay(1);
	iowrite32(0, sp->base + OT_SPI_CS0);
	mdelay(1);
	iowrite32(1, sp->base + OT_SPI_CS0);
	mdelay(10);

	ctlr->mode_bits = SPI_CPOL | SPI_CPHA;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->transfer_one_message = ot_spi_transfer_one_message;
	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->num_chipselect = 1;

	dev_info(&pdev->dev, "OpenTitan SPI controller, clk=%lu Hz\n",
		 sp->clk_rate);

	return devm_spi_register_controller(&pdev->dev, ctlr);
}

static struct platform_driver ot_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ot_spi_match,
	},
	.probe = ot_spi_probe,
};

module_platform_driver(ot_spi_driver);

MODULE_DESCRIPTION("SPI controller driver for OpenTitan-derived SoCs");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
