// SPDX-License-Identifier: GPL-2.0
/*
 * OpenTitan SPI host controller driver for lowRISC Sonata.
 *
 * This drives the OpenTitan SPI host IP block with memory-mapped
 * TX/RX buffers (2048 bytes each) for bulk transfers.
 *
 * The hardware only supports a single START per CS assertion, so
 * multi-transfer SPI messages are merged into one START command
 * via the transfer_one_message callback.
 *
 * Register map:
 *   0x0C  CFG      [RW]  half_clk_period[15:0], msb_first[29], cpha[30], cpol[31]
 *   0x10  CONTROL  [RW]  tx_clear[0], rx_clear[1], tx_en[2], rx_en[3], sw_reset[31]
 *   0x14  STATUS   [RO]  tx_lvl[11:0], rx_lvl[23:12], tx_full[24], rx_empty[25], idle[26]
 *   0x18  START    [WO]  byte_count[10:0]
 *   0x1C  RX_FIFO  [RO]  data[7:0]
 *   0x20  TX_FIFO  [WO]  data[7:0]
 *   0x28  CS       [RW]  per-bit chip-select (active low)
 *
 * Buffer map (within same 8KB region):
 *   0x1000-0x17FF  TX buffer (2048 bytes, CPU writes, HW reads)
 *   0x1800-0x1FFF  RX buffer (2048 bytes, HW writes, CPU reads)
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

/* Memory-mapped buffer offsets */
#define OT_SPI_TX_BUF		0x1000
#define OT_SPI_RX_BUF		0x1800
#define OT_SPI_BUF_SIZE	2048

/* CONTROL bits */
#define OT_SPI_CTRL_TX_FLUSH	BIT(0)
#define OT_SPI_CTRL_RX_FLUSH	BIT(1)
#define OT_SPI_CTRL_TX_EN	BIT(2)
#define OT_SPI_CTRL_RX_EN	BIT(3)
#define OT_SPI_CTRL_SW_RESET	BIT(31)

/* STATUS bits */
#define OT_SPI_STATUS_IDLE	BIT(26)

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
 * Handle an entire SPI message as a single hardware transaction using
 * the memory-mapped TX/RX buffers.
 *
 * All transfers in the message are concatenated into the TX buffer,
 * a single START is issued, and RX data is read back from the RX buffer.
 */
static int ot_spi_transfer_one_message(struct spi_controller *host,
				       struct spi_message *msg)
{
	struct ot_spi *spi = spi_controller_get_devdata(host);
	struct spi_transfer *xfer;
	unsigned int total_len = 0;
	unsigned int off;
	bool need_rx = false;
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

	if (total_len > OT_SPI_BUF_SIZE) {
		dev_err(&host->dev, "message too large (%u > %u)\n",
			total_len, OT_SPI_BUF_SIZE);
		ret = -EMSGSIZE;
		goto done;
	}

	ret = ot_spi_wait_idle(spi);
	if (ret) {
		dev_err(&host->dev, "SPI not idle before message\n");
		goto flush;
	}

	/* Fill TX buffer from all transfers */
	off = 0;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (xfer->tx_buf)
			memcpy_toio(spi->base + OT_SPI_TX_BUF + off,
				    xfer->tx_buf, xfer->len);
		else
			memset_io(spi->base + OT_SPI_TX_BUF + off,
				  0, xfer->len);
		off += xfer->len;
	}

	/* Flush FIFOs, enable TX (+RX if needed) */
	ot_spi_write(spi, OT_SPI_CONTROL,
		     OT_SPI_CTRL_TX_FLUSH | OT_SPI_CTRL_RX_FLUSH);
	ot_spi_write(spi, OT_SPI_CONTROL,
		     OT_SPI_CTRL_TX_EN |
		     (need_rx ? OT_SPI_CTRL_RX_EN : 0));

	/* Assert chip select */
	ot_spi_set_cs(msg->spi, false);

	/* Single START for the entire message */
	ot_spi_write(spi, OT_SPI_START, total_len);

	/* Wait for transfer to complete */
	ret = ot_spi_wait_idle(spi);
	if (ret) {
		dev_err(&host->dev, "SPI transfer timeout, STATUS=0x%08x\n",
			ot_spi_read(spi, OT_SPI_STATUS));
		goto cs_off;
	}

	/* Read RX buffer back to transfers that need it */
	if (need_rx) {
		off = 0;
		list_for_each_entry(xfer, &msg->transfers, transfer_list) {
			if (xfer->rx_buf)
				memcpy_fromio(xfer->rx_buf,
					      spi->base + OT_SPI_RX_BUF + off,
					      xfer->len);
			off += xfer->len;
		}
	}

	msg->actual_length = total_len;

cs_off:
	ot_spi_set_cs(msg->spi, true);
flush:
	if (ret) {
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
	return OT_SPI_BUF_SIZE;
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

	/* Software reset + flush */
	ot_spi_write(spi, OT_SPI_CONTROL, OT_SPI_CTRL_SW_RESET);
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
