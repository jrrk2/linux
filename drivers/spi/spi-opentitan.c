// SPDX-License-Identifier: GPL-2.0
/*
 * OpenTitan SPI host controller driver for lowRISC Sonata.
 *
 * This drives the OpenTitan SPI host IP block, which has:
 * - 8-entry TX and RX FIFOs (byte-wide)
 * - Manual chip-select control
 * - Separate TX/RX enable bits supporting full-duplex
 * - Transfer length programmed via START register
 *
 * The hardware only supports a single START per CS assertion, so
 * multi-transfer SPI messages are merged into one START command
 * via the transfer_one_message callback.
 *
 * Register map:
 *   0x0C  CFG      [RW]  half_clk_period[15:0], msb_first[29], cpha[30], cpol[31]
 *   0x10  CONTROL  [RW]  tx_clear[0], rx_clear[1], tx_en[2], rx_en[3]
 *   0x14  STATUS   [RO]  tx_lvl[7:0], rx_lvl[15:8], tx_full[16], rx_empty[17], idle[18]
 *   0x18  START    [WO]  byte_count[10:0]
 *   0x1C  RX_FIFO  [RO]  data[7:0]
 *   0x20  TX_FIFO  [WO]  data[7:0]
 *   0x28  CS       [RW]  per-bit chip-select (active low)
 *
 * Copyright (C) 2026 Jonathan
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/jiffies.h>
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
#define OT_SPI_CTRL_TX_FLUSH	BIT(0)
#define OT_SPI_CTRL_RX_FLUSH	BIT(1)
#define OT_SPI_CTRL_TX_EN	BIT(2)
#define OT_SPI_CTRL_RX_EN	BIT(3)

/* STATUS bits */
#define OT_SPI_STATUS_TX_LVL_MASK	0xFF
#define OT_SPI_STATUS_RX_LVL_SHIFT	8
#define OT_SPI_STATUS_RX_LVL_MASK	0xFF
#define OT_SPI_STATUS_IDLE		BIT(18)

struct ot_spi {
	void __iomem *base;
	struct clk *clk;
	u32 cs_state;		/* Shadow of CS register (active-low) */
#ifdef CONFIG_GPIOLIB
	struct gpio_chip gc;
#endif
};

static inline u32 ot_spi_read(struct ot_spi *spi, unsigned int off)
{
	return readl(spi->base + off);
}

static inline void ot_spi_write(struct ot_spi *spi, unsigned int off, u32 val)
{
	writel(val, spi->base + off);
}

static int ot_spi_wait_idle(struct ot_spi *spi)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	while (!(ot_spi_read(spi, OT_SPI_STATUS) & OT_SPI_STATUS_IDLE)) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}
	return 0;
}

static int ot_spi_rx_avail(struct ot_spi *spi)
{
	return (ot_spi_read(spi, OT_SPI_STATUS) >> OT_SPI_STATUS_RX_LVL_SHIFT)
	       & OT_SPI_STATUS_RX_LVL_MASK;
}

static void ot_spi_set_cs(struct spi_device *device, bool is_high)
{
	struct ot_spi *spi = spi_controller_get_devdata(device->controller);
	int cs_num = spi_get_chipselect(device, 0);

	if (device->mode & SPI_CS_HIGH)
		is_high = !is_high;

	if (is_high)
		spi->cs_state |= BIT(cs_num);   /* Deassert (CS inactive high) */
	else
		spi->cs_state &= ~BIT(cs_num);  /* Assert (CS active low) */

	ot_spi_write(spi, OT_SPI_CS, spi->cs_state);
}

/*
 * Handle an entire SPI message as a single hardware transaction.
 *
 * The OpenTitan SPI host does not support issuing multiple START commands
 * within one CS assertion.  We therefore sum the byte counts of every
 * transfer in the message, issue one START, and stream TX/RX data across
 * the transfer boundaries.
 *
 * Because the hardware FIFOs are only 8 bytes deep, TX writes and RX reads
 * must be interleaved for messages longer than 8 bytes when RX is enabled.
 * Otherwise the RX FIFO fills up, the hardware stalls, the TX FIFO fills,
 * and we deadlock.
 */
static int ot_spi_transfer_one_message(struct spi_controller *host,
				       struct spi_message *msg)
{
	struct ot_spi *spi = spi_controller_get_devdata(host);
	struct spi_transfer *xfer;
	struct spi_transfer *tx_xfer, *rx_xfer;
	unsigned int total_len = 0;
	unsigned int tx_off = 0, rx_off = 0;
	unsigned int tx_done = 0, rx_done = 0;
	unsigned int rx_total;
	bool need_rx = false;
	unsigned long timeout;
	u32 ctrl;
	int ret;

	/* Calculate total byte count and check if any transfer needs RX */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		total_len += xfer->len;
		if (xfer->rx_buf)
			need_rx = true;
	}

	if (total_len == 0) {
		ret = 0;
		goto done;
	}

	rx_total = need_rx ? total_len : 0;

	ret = ot_spi_wait_idle(spi);
	if (ret) {
		dev_err(&host->dev, "SPI not idle before message\n");
		goto flush;
	}

	/* Flush FIFOs to ensure clean state */
	ot_spi_write(spi, OT_SPI_CONTROL,
		     OT_SPI_CTRL_TX_FLUSH | OT_SPI_CTRL_RX_FLUSH);

	/* Assert chip select */
	ot_spi_set_cs(msg->spi, false);

	/* Always TX_EN for clock generation; RX_EN if any transfer reads */
	ctrl = OT_SPI_CTRL_TX_EN;
	if (need_rx)
		ctrl |= OT_SPI_CTRL_RX_EN;
	ot_spi_write(spi, OT_SPI_CONTROL, ctrl);

	/* Single START for the entire message */
	ot_spi_write(spi, OT_SPI_START, total_len);

	/*
	 * Interleave TX writes and RX reads.  For every byte we push into
	 * the TX FIFO the hardware clocks one byte in from MISO (if RX_EN).
	 * With only 8-byte FIFOs we must drain RX while feeding TX to
	 * prevent either FIFO from blocking the shift engine.
	 */
	tx_xfer = list_first_entry(&msg->transfers,
				   struct spi_transfer, transfer_list);
	rx_xfer = tx_xfer;
	timeout = jiffies + msecs_to_jiffies(500);

	while (tx_done < total_len || rx_done < rx_total) {
		u32 status = ot_spi_read(spi, OT_SPI_STATUS);

		/* Feed TX FIFO if not full and we have more to send */
		if (tx_done < total_len && !(status & BIT(16))) {
			const u8 *tx = tx_xfer->tx_buf;

			ot_spi_write(spi, OT_SPI_TX_FIFO,
				     tx ? tx[tx_off] : 0x00);
			tx_off++;
			tx_done++;
			if (tx_off >= tx_xfer->len && tx_done < total_len) {
				tx_xfer = list_next_entry(tx_xfer,
							  transfer_list);
				tx_off = 0;
			}
		}

		/* Drain RX FIFO while data is available */
		if (rx_done < rx_total) {
			unsigned int rx_lvl;

			rx_lvl = (status >> OT_SPI_STATUS_RX_LVL_SHIFT)
				 & OT_SPI_STATUS_RX_LVL_MASK;
			while (rx_lvl && rx_done < rx_total) {
				u8 *rx = rx_xfer->rx_buf;
				u8 val = (u8)ot_spi_read(spi,
							 OT_SPI_RX_FIFO);
				if (rx)
					rx[rx_off] = val;
				rx_off++;
				rx_done++;
				rx_lvl--;
				if (rx_off >= rx_xfer->len &&
				    rx_done < rx_total) {
					rx_xfer = list_next_entry(rx_xfer,
								  transfer_list);
					rx_off = 0;
				}
			}
		}

		if (time_after(jiffies, timeout)) {
			dev_err(&host->dev,
				"SPI timeout, STATUS=0x%08x tx=%u/%u rx=%u/%u\n",
				ot_spi_read(spi, OT_SPI_STATUS),
				tx_done, total_len, rx_done, rx_total);
			ret = -ETIMEDOUT;
			goto cs_off;
		}

		cpu_relax();
	}

	/* Wait for hardware to finish clocking the last bytes */
	ret = ot_spi_wait_idle(spi);
	if (ret) {
		dev_err(&host->dev, "SPI idle timeout, STATUS=0x%08x\n",
			ot_spi_read(spi, OT_SPI_STATUS));
		goto cs_off;
	}

	msg->actual_length = total_len;

cs_off:
	ot_spi_set_cs(msg->spi, true);
flush:
	if (ret) {
		/* Flush FIFOs so the next transaction starts clean */
		ot_spi_write(spi, OT_SPI_CONTROL,
			     OT_SPI_CTRL_TX_FLUSH | OT_SPI_CTRL_RX_FLUSH);
	}
done:
	msg->status = ret;
	spi_finalize_current_message(host);
	return ret;
}

static size_t ot_spi_max_message_size(struct spi_device *spi)
{
	return 2047; /* START register is 11 bits */
}

#ifdef CONFIG_GPIOLIB
static int ot_spi_gpio_set(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct ot_spi *spi = gpiochip_get_data(gc);

	if (val)
		spi->cs_state |= BIT(offset);
	else
		spi->cs_state &= ~BIT(offset);
	ot_spi_write(spi, OT_SPI_CS, spi->cs_state);
	return 0;
}

static int ot_spi_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct ot_spi *spi = gpiochip_get_data(gc);

	return !!(spi->cs_state & BIT(offset));
}

static int ot_spi_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int val)
{
	ot_spi_gpio_set(gc, offset, val);
	return 0;
}
#endif

static int ot_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *host;
	struct ot_spi *spi;
	u32 reset_cs, reset_ms;
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
	host->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	host->bits_per_word_mask = SPI_BPW_MASK(8);
	host->transfer_one_message = ot_spi_transfer_one_message;
	host->set_cs = ot_spi_set_cs;
	host->max_speed_hz = 25000000;
	host->max_message_size = ot_spi_max_message_size;

	/* Flush FIFOs — controller won't report IDLE without this */
	ot_spi_write(spi, OT_SPI_CONTROL,
		     OT_SPI_CTRL_TX_FLUSH | OT_SPI_CTRL_RX_FLUSH);

	/* Deassert all CS lines (active low) */
	spi->cs_state = 0xF;
	ot_spi_write(spi, OT_SPI_CS, spi->cs_state);

	/* Optional DT-driven reset: pulse a CS line low for a device reset */
	if (!of_property_read_u32(pdev->dev.of_node, "reset-cs", &reset_cs)) {
		if (of_property_read_u32(pdev->dev.of_node,
					 "reset-duration-ms", &reset_ms))
			reset_ms = 150;
		dev_info(&pdev->dev, "reset via CS[%u] for %u ms\n",
			 reset_cs, reset_ms);
		spi->cs_state &= ~BIT(reset_cs);
		ot_spi_write(spi, OT_SPI_CS, spi->cs_state);
		mdelay(reset_ms);
		spi->cs_state |= BIT(reset_cs);
		ot_spi_write(spi, OT_SPI_CS, spi->cs_state);
		mdelay(100);
	}

#ifdef CONFIG_GPIOLIB
	if (of_property_read_bool(pdev->dev.of_node, "gpio-controller")) {
		spi->gc.label = dev_name(&pdev->dev);
		spi->gc.parent = &pdev->dev;
		spi->gc.owner = THIS_MODULE;
		spi->gc.base = -1;
		spi->gc.ngpio = host->num_chipselect;
		spi->gc.set = ot_spi_gpio_set;
		spi->gc.get = ot_spi_gpio_get;
		spi->gc.direction_output = ot_spi_gpio_direction_output;
		spi->gc.fwnode = dev_fwnode(&pdev->dev);

		ret = devm_gpiochip_add_data(&pdev->dev, &spi->gc, spi);
		if (ret) {
			dev_err(&pdev->dev, "failed to add GPIO chip\n");
			return ret;
		}
	}
#endif

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
