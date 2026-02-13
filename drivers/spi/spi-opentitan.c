// SPDX-License-Identifier: GPL-2.0
/*
 * OpenTitan SPI host controller driver for lowRISC Sonata.
 *
 * This drives the OpenTitan SPI host IP block, which has:
 * - 128-byte TX and RX FIFOs
 * - Manual chip-select control
 * - Separate TX/RX enable bits for half/full-duplex
 * - Transfer length programmed via START register
 *
 * Copyright (C) 2026 Jonathan
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

/* Register offsets */
#define OT_SPI_CFG		0x0C
#define OT_SPI_CONTROL		0x10
#define OT_SPI_STATUS		0x14
#define OT_SPI_START		0x18
#define OT_SPI_RX_FIFO		0x1C
#define OT_SPI_TX_FIFO		0x20
#define OT_SPI_INFO		0x24
#define OT_SPI_CS		0x28

/* CONTROL bits */
#define OT_SPI_CTRL_TX_EN	BIT(2)
#define OT_SPI_CTRL_RX_EN	BIT(3)

/* STATUS bits */
#define OT_SPI_STATUS_TX_LVL_MASK	0xFF
#define OT_SPI_STATUS_RX_LVL_SHIFT	8
#define OT_SPI_STATUS_RX_LVL_MASK	0xFF
#define OT_SPI_STATUS_IDLE		BIT(18)

#define OT_SPI_FIFO_DEPTH	128

struct ot_spi {
	void __iomem *base;
	struct clk *clk;
};

static inline u32 ot_spi_read(struct ot_spi *spi, unsigned int off)
{
	return readl(spi->base + off);
}

static inline void ot_spi_write(struct ot_spi *spi, unsigned int off, u32 val)
{
	writel(val, spi->base + off);
}

static void ot_spi_wait_idle(struct ot_spi *spi)
{
	while (!(ot_spi_read(spi, OT_SPI_STATUS) & OT_SPI_STATUS_IDLE))
		cpu_relax();
}

static int ot_spi_tx_avail(struct ot_spi *spi)
{
	return OT_SPI_FIFO_DEPTH -
	       (ot_spi_read(spi, OT_SPI_STATUS) & OT_SPI_STATUS_TX_LVL_MASK);
}

static int ot_spi_rx_avail(struct ot_spi *spi)
{
	return (ot_spi_read(spi, OT_SPI_STATUS) >> OT_SPI_STATUS_RX_LVL_SHIFT)
	       & OT_SPI_STATUS_RX_LVL_MASK;
}

static void ot_spi_set_cs(struct spi_device *device, bool is_high)
{
	struct ot_spi *spi = spi_controller_get_devdata(device->controller);
	u32 cs_val;
	int cs_num = spi_get_chipselect(device, 0);

	if (device->mode & SPI_CS_HIGH)
		is_high = !is_high;

	cs_val = ot_spi_read(spi, OT_SPI_CS);
	if (is_high)
		cs_val |= BIT(cs_num);   /* Deassert (CS inactive high) */
	else
		cs_val &= ~BIT(cs_num);  /* Assert (CS active low) */

	ot_spi_write(spi, OT_SPI_CS, cs_val);
}

static int ot_spi_transfer_one(struct spi_controller *host,
			       struct spi_device *device,
			       struct spi_transfer *t)
{
	struct ot_spi *spi = spi_controller_get_devdata(host);
	const u8 *tx_ptr = t->tx_buf;
	u8 *rx_ptr = t->rx_buf;
	unsigned int remaining = t->len;
	u32 ctrl;

	while (remaining > 0) {
		unsigned int chunk = min_t(unsigned int, remaining,
					   OT_SPI_FIFO_DEPTH);
		unsigned int i;

		ot_spi_wait_idle(spi);

		/* Set direction */
		ctrl = 0;
		if (tx_ptr)
			ctrl |= OT_SPI_CTRL_TX_EN;
		if (rx_ptr)
			ctrl |= OT_SPI_CTRL_RX_EN;
		/* If neither, default to TX (clock out zeros) */
		if (!ctrl)
			ctrl = OT_SPI_CTRL_TX_EN;

		ot_spi_write(spi, OT_SPI_CONTROL, ctrl);

		/* Fill TX FIFO */
		if (tx_ptr) {
			for (i = 0; i < chunk; i++) {
				while (ot_spi_tx_avail(spi) == 0)
					cpu_relax();
				ot_spi_write(spi, OT_SPI_TX_FIFO, tx_ptr[i]);
			}
		}

		/* Start transfer */
		ot_spi_write(spi, OT_SPI_START, chunk);

		/* Wait for completion */
		ot_spi_wait_idle(spi);

		/* Drain RX FIFO */
		if (rx_ptr) {
			for (i = 0; i < chunk; i++) {
				while (ot_spi_rx_avail(spi) == 0)
					cpu_relax();
				rx_ptr[i] = (u8)ot_spi_read(spi,
							     OT_SPI_RX_FIFO);
			}
		}

		if (tx_ptr)
			tx_ptr += chunk;
		if (rx_ptr)
			rx_ptr += chunk;
		remaining -= chunk;
	}

	spi_finalize_current_transfer(host);
	return 0;
}

static int ot_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *host;
	struct ot_spi *spi;
	int ret;

	host = devm_spi_alloc_host(&pdev->dev, sizeof(*spi));
	if (!host)
		return -ENOMEM;

	spi = spi_controller_get_devdata(host);

	spi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spi->base))
		return PTR_ERR(spi->base);

	spi->clk = devm_clk_get_optional_enabled(&pdev->dev, NULL);
	if (IS_ERR(spi->clk))
		return PTR_ERR(spi->clk);

	host->dev.of_node = pdev->dev.of_node;
	host->bus_num = -1;
	host->num_chipselect = 4;
	host->mode_bits = SPI_CPOL | SPI_CPHA;
	host->bits_per_word_mask = SPI_BPW_MASK(8);
	host->transfer_one = ot_spi_transfer_one;
	host->set_cs = ot_spi_set_cs;
	host->max_speed_hz = 25000000;

	/* Deassert all CS lines */
	ot_spi_write(spi, OT_SPI_CS, 0xF);

	ret = devm_spi_register_controller(&pdev->dev, host);
	if (ret)
		dev_err(&pdev->dev, "failed to register SPI controller\n");

	return ret;
}

static const struct of_device_id ot_spi_of_match[] = {
	{ .compatible = "lowrisc,opentitan-spi" },
	{}
};
MODULE_DEVICE_TABLE(of, ot_spi_of_match);

static struct platform_driver ot_spi_driver = {
	.probe	= ot_spi_probe,
	.driver	= {
		.name		= "opentitan-spi",
		.of_match_table	= ot_spi_of_match,
	},
};
module_platform_driver(ot_spi_driver);

MODULE_AUTHOR("Jonathan");
MODULE_DESCRIPTION("OpenTitan SPI host controller driver for lowRISC Sonata");
MODULE_LICENSE("GPL");
