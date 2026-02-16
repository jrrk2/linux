// SPDX-License-Identifier: GPL-2.0
/*
 * Framebuffer driver for the ST7735 LCD on the lowRISC Sonata board.
 *
 * The display is connected to the SPI LCD controller at 0x80300000:
 *   CS[0] = chip select (directly controlled, active low)
 *   CS[1] = D/C (data/command select: 0=command, 1=data)
 *   CS[2] = reset (active low)
 *
 * This driver talks directly to the SPI hardware registers rather than
 * going through the Linux SPI framework, since the display requires
 * per-byte D/C toggling that doesn't map well to SPI messages.
 *
 * The SPI routines are an exact port of the bare-metal spi.c / main.c
 * which is proven working on this hardware.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>

/* PWM register for LCD backlight (channel 6 at PWM base 0x80001000) */
#define PWM_BASE	0x80001000
#define PWM_LCD_CH	6
#define PWM_CH_SIZE	8 /* 8 bytes per channel */

/* SPI register offsets — matching bare-metal spi.h */
#define SPI_CFG		0x0C
#define SPI_CONTROL	0x10
#define SPI_STATUS	0x14
#define SPI_START	0x18
#define SPI_RX_FIFO	0x1C
#define SPI_TX_FIFO	0x20
#define SPI_INFO	0x24
#define SPI_CS		0x28

#define SPI_STATUS_IDLE		0x40000

/* ST7735 commands */
#define ST7735_SWRESET	0x01
#define ST7735_SLPOUT	0x11
#define ST7735_NORON	0x13
#define ST7735_INVOFF	0x20
#define ST7735_DISPON	0x29
#define ST7735_CASET	0x2A
#define ST7735_RASET	0x2B
#define ST7735_RAMWR	0x2C
#define ST7735_COLMOD	0x3A
#define ST7735_MADCTL	0x36
#define ST7735_FRMCTR1	0xB1
#define ST7735_FRMCTR2	0xB2
#define ST7735_FRMCTR3	0xB3
#define ST7735_INVCTR	0xB4
#define ST7735_PWCTR1	0xC0
#define ST7735_PWCTR2	0xC1
#define ST7735_PWCTR3	0xC2
#define ST7735_PWCTR4	0xC3
#define ST7735_PWCTR5	0xC4
#define ST7735_VMCTR1	0xC5
#define ST7735_GMCTRP1	0xE0
#define ST7735_GMCTRN1	0xE1

#define LCD_WIDTH	160
#define LCD_HEIGHT	128

struct sonata_lcd {
	void __iomem *base;
	struct fb_info *info;
	u16 *vmem;
	struct delayed_work work;
};

/*
 * SPI helpers — exact port of bare-metal spi.c
 *
 * The bare-metal code does NOT write to SPI_CFG at all, relying on
 * hardware defaults (MSB_FIRST=1, HALF_CLK_PERIOD=0 → ~20MHz SPI).
 * We replicate that here.
 */

static int spi_wait_idle(void __iomem *base)
{
	int timeout = 1000000;
	u32 status;

	while (timeout-- > 0) {
		status = readl(base + SPI_STATUS);
		if (status & SPI_STATUS_IDLE)
			return 0;
	}
	pr_err("sonata-lcd: spi_wait_idle TIMEOUT, status=0x%08x\n", status);
	return -ETIMEDOUT;
}

/*
 * Port of bare-metal spi_tx() from spi.c.
 *
 * The SPI_START register is 11 bits wide (max 2047), so large
 * transfers must be split into chunks.  The bare-metal code never
 * calls spi_tx with >2047 bytes (pixels are sent 2 at a time).
 *
 * The TX FIFO is 8 entries deep (from SPI_INFO register).  The
 * bare-metal code assumes 128 but works because it only sends
 * small amounts.  We poll TX_FIFO_FULL (bit 16) instead.
 */
static void spi_tx(void __iomem *base, const u8 *data, int len)
{
	while (len > 0) {
		int chunk = (len > 2047) ? 2047 : len;
		int i;

		spi_wait_idle(base);
		writel(0x4, base + SPI_CONTROL); /* TX_ENABLE */
		writel(chunk, base + SPI_START);

		for (i = 0; i < chunk; i++) {
			/* Wait until TX FIFO has space (not full) */
			while (readl(base + SPI_STATUS) & (1u << 16))
				;
			writel(data[i], base + SPI_TX_FIFO);
		}

		data += chunk;
		len -= chunk;
	}
}

/*
 * CS/DC/RST control via SPI_CS register — exact port of bare-metal
 * gpio_write() callback from main.c.
 *
 * Bare-metal uses spi_set_cs() which does read-modify-write per bit.
 * We write all bits at once since we control all three signals.
 */
static void spi_set_cs(void __iomem *base, int cs_line, int level)
{
	u32 cs = readl(base + SPI_CS);

	if (level)
		cs |= (1u << cs_line);
	else
		cs &= ~(1u << cs_line);
	writel(cs, base + SPI_CS);
}

/* CS line assignments — matching bare-metal main.c */
#define LCD_CS_LINE	0
#define LCD_DC_LINE	1
#define LCD_RST_LINE	2

/*
 * gpio_write callback equivalent — sets DC then CS,
 * exactly matching bare-metal gpio_write() order.
 */
static void lcd_gpio_write(void __iomem *base, bool cs, bool dc)
{
	spi_set_cs(base, LCD_DC_LINE, dc);
	spi_set_cs(base, LCD_CS_LINE, cs);
}

/*
 * SPI write callback equivalent — exact port of bare-metal spi_write().
 * The bare-metal spi_write() calls spi_tx then spi_wait_idle.
 */
static void lcd_spi_write(void __iomem *base, const u8 *data, int len)
{
	spi_tx(base, data, len);
	spi_wait_idle(base);
}

/*
 * ST7735 command helpers — matching the bare-metal lcd_st7735 library's
 * internal write_command / write_data flow.
 *
 * Bare-metal flow for a command with data:
 *   1. gpio_write(cs=0, dc=0)  — CS low, DC low (command mode)
 *   2. spi_write(cmd, 1)       — send command byte, wait idle
 *   3. gpio_write(cs=1, dc=0)  — CS high (deselect)
 *   4. gpio_write(cs=0, dc=1)  — CS low, DC high (data mode)
 *   5. spi_write(data, len)    — send data bytes, wait idle
 *   6. gpio_write(cs=1, dc=1)  — CS high (deselect)
 */
static void lcd_write_cmd(struct sonata_lcd *lcd, u8 cmd)
{
	lcd_gpio_write(lcd->base, false, false); /* CS low, DC low */
	lcd_spi_write(lcd->base, &cmd, 1);
	lcd_gpio_write(lcd->base, true, false);  /* CS high, DC low */
}

static void lcd_write_data(struct sonata_lcd *lcd, const u8 *data, int len)
{
	lcd_gpio_write(lcd->base, false, true);  /* CS low, DC high */
	lcd_spi_write(lcd->base, data, len);
	lcd_gpio_write(lcd->base, true, true);   /* CS high, DC high */
}

static void lcd_cmd(struct sonata_lcd *lcd, u8 cmd)
{
	lcd_write_cmd(lcd, cmd);
}

static void lcd_cmd_data(struct sonata_lcd *lcd, u8 cmd,
			 const u8 *data, int len)
{
	lcd_write_cmd(lcd, cmd);
	if (len > 0)
		lcd_write_data(lcd, data, len);
}

static void lcd_set_window(struct sonata_lcd *lcd,
			   u16 x0, u16 y0, u16 x1, u16 y1)
{
	u8 buf[4];

	buf[0] = x0 >> 8; buf[1] = x0;
	buf[2] = x1 >> 8; buf[3] = x1;
	lcd_cmd_data(lcd, ST7735_CASET, buf, 4);

	buf[0] = y0 >> 8; buf[1] = y0;
	buf[2] = y1 >> 8; buf[3] = y1;
	lcd_cmd_data(lcd, ST7735_RASET, buf, 4);
}

/*
 * Hardware init — ported from bare-metal init_script_r + init_script_r3.
 *
 * The bare-metal init flow:
 *   1. spi_set_cs(DC, 0), spi_set_cs(CS, 0)  — initial pin state
 *   2. RST pulse low for 150ms
 *   3. run_script(init_script_r)  — SWRESET, SLPOUT, frame rate, power, etc.
 *   4. run_script(init_script_r3) — gamma, NORON, DISPON
 */
static void lcd_hw_init(struct sonata_lcd *lcd)
{
	u8 buf[16];

	/* Set initial pin state — matching bare-metal main.c lines 62-63 */
	spi_set_cs(lcd->base, LCD_DC_LINE, 0);
	spi_set_cs(lcd->base, LCD_CS_LINE, 0);

	/* Hardware reset — matching bare-metal main.c lines 66-68 */
	spi_set_cs(lcd->base, LCD_RST_LINE, 0);
	mdelay(150);
	spi_set_cs(lcd->base, LCD_RST_LINE, 1);
	mdelay(150);

	/* Software reset */
	lcd_cmd(lcd, ST7735_SWRESET);
	mdelay(150);

	/* Sleep out */
	lcd_cmd(lcd, ST7735_SLPOUT);
	mdelay(500);

	/* Frame rate control - normal mode */
	buf[0] = 0x01; buf[1] = 0x2C; buf[2] = 0x2D;
	lcd_cmd_data(lcd, ST7735_FRMCTR1, buf, 3);

	/* Frame rate control - idle mode */
	buf[0] = 0x01; buf[1] = 0x2C; buf[2] = 0x2D;
	lcd_cmd_data(lcd, ST7735_FRMCTR2, buf, 3);

	/* Frame rate control - partial mode */
	buf[0] = 0x01; buf[1] = 0x2C; buf[2] = 0x2D;
	buf[3] = 0x01; buf[4] = 0x2C; buf[5] = 0x2D;
	lcd_cmd_data(lcd, ST7735_FRMCTR3, buf, 6);

	/* Display inversion control */
	buf[0] = 0x07;
	lcd_cmd_data(lcd, ST7735_INVCTR, buf, 1);

	/* Power control */
	buf[0] = 0xA2; buf[1] = 0x02; buf[2] = 0x84;
	lcd_cmd_data(lcd, ST7735_PWCTR1, buf, 3);

	buf[0] = 0xC5;
	lcd_cmd_data(lcd, ST7735_PWCTR2, buf, 1);

	buf[0] = 0x0A; buf[1] = 0x00;
	lcd_cmd_data(lcd, ST7735_PWCTR3, buf, 2);

	buf[0] = 0x8A; buf[1] = 0x2A;
	lcd_cmd_data(lcd, ST7735_PWCTR4, buf, 2);

	buf[0] = 0x8A; buf[1] = 0xEE;
	lcd_cmd_data(lcd, ST7735_PWCTR5, buf, 2);

	/* VCOM control */
	buf[0] = 0x0E;
	lcd_cmd_data(lcd, ST7735_VMCTR1, buf, 1);

	/* Inversion off */
	lcd_cmd(lcd, ST7735_INVOFF);

	/* Memory access control: MV | MX | BGR → landscape, 160x128 */
	buf[0] = 0x68;
	lcd_cmd_data(lcd, ST7735_MADCTL, buf, 1);

	/* Color mode: 16-bit */
	buf[0] = 0x05;
	lcd_cmd_data(lcd, ST7735_COLMOD, buf, 1);

	/* Positive gamma — from init_script_r3 */
	memcpy(buf, "\x02\x1c\x07\x12\x37\x32\x29\x2d"
		    "\x29\x25\x2B\x39\x00\x01\x03\x10", 16);
	lcd_cmd_data(lcd, ST7735_GMCTRP1, buf, 16);

	/* Negative gamma — from init_script_r3 */
	memcpy(buf, "\x03\x1d\x07\x06\x2E\x2C\x29\x2D"
		    "\x2E\x2E\x37\x3F\x00\x00\x02\x10", 16);
	lcd_cmd_data(lcd, ST7735_GMCTRN1, buf, 16);

	/* Normal display on */
	lcd_cmd(lcd, ST7735_NORON);
	mdelay(10);

	/* Display on */
	lcd_cmd(lcd, ST7735_DISPON);
	mdelay(100);
}

/*
 * Push vmem to the display — matching the bare-metal
 * lcd_st7735_rgb565_start / _put / _finish flow.
 */
static void lcd_update(struct sonata_lcd *lcd)
{
	lcd_set_window(lcd, 0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
	lcd_write_cmd(lcd, ST7735_RAMWR);
	/* Data phase: CS low, DC high for the entire pixel block */
	lcd_gpio_write(lcd->base, false, true);
	spi_tx(lcd->base, (u8 *)lcd->vmem, LCD_WIDTH * LCD_HEIGHT * 2);
	spi_wait_idle(lcd->base);
	lcd_gpio_write(lcd->base, true, true);
}

/* Deferred I/O — write dirty pages to display */

static void sonata_lcd_deferred_work(struct work_struct *work)
{
	struct sonata_lcd *lcd = container_of(work, struct sonata_lcd,
					      work.work);

	lcd_update(lcd);
}

/* FB ops */

static ssize_t sonata_lcd_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct sonata_lcd *lcd = info->par;
	ssize_t ret;

	ret = fb_sys_write(info, buf, count, ppos);
	if (ret > 0)
		schedule_delayed_work(&lcd->work, msecs_to_jiffies(50));
	return ret;
}

static void sonata_lcd_fillrect(struct fb_info *info,
				const struct fb_fillrect *rect)
{
	struct sonata_lcd *lcd = info->par;

	sys_fillrect(info, rect);
	schedule_delayed_work(&lcd->work, msecs_to_jiffies(50));
}

static void sonata_lcd_copyarea(struct fb_info *info,
				const struct fb_copyarea *area)
{
	struct sonata_lcd *lcd = info->par;

	sys_copyarea(info, area);
	schedule_delayed_work(&lcd->work, msecs_to_jiffies(50));
}

static void sonata_lcd_imageblit(struct fb_info *info,
				 const struct fb_image *image)
{
	struct sonata_lcd *lcd = info->par;

	sys_imageblit(info, image);
	schedule_delayed_work(&lcd->work, msecs_to_jiffies(50));
}

static const struct fb_ops sonata_lcd_ops = {
	.owner		= THIS_MODULE,
	__FB_DEFAULT_SYSMEM_OPS_RDWR,
	.fb_write	= sonata_lcd_write,
	.fb_fillrect	= sonata_lcd_fillrect,
	.fb_copyarea	= sonata_lcd_copyarea,
	.fb_imageblit	= sonata_lcd_imageblit,
};

static int sonata_lcd_probe(struct platform_device *pdev)
{
	struct sonata_lcd *lcd;
	struct fb_info *info;
	int vmem_size = LCD_WIDTH * LCD_HEIGHT * 2; /* 16bpp */
	int x, y;

	lcd = devm_kzalloc(&pdev->dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	lcd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lcd->base))
		return PTR_ERR(lcd->base);

	lcd->vmem = devm_kzalloc(&pdev->dev, vmem_size, GFP_KERNEL);
	if (!lcd->vmem)
		return -ENOMEM;

	/*
	 * Reset SPI controller to a known state.  When reloading the kernel
	 * via GDB without a board reset, the controller may be mid-transaction
	 * with IDLE deasserted, causing spi_wait_idle to spin forever.
	 *
	 * Per the SPI spec: clear TX FIFO first, then SW_RESET to force
	 * the state machine to IDLE, then clear RX FIFO.
	 */
	writel(0x1, lcd->base + SPI_CONTROL);  /* TX_CLEAR */
	writel(1u << 31, lcd->base + SPI_CONTROL);  /* SW_RESET */
	writel(0x2, lcd->base + SPI_CONTROL);  /* RX_CLEAR */

	dev_info(&pdev->dev, "initializing ST7735 display...\n");
	lcd_hw_init(lcd);

	/* Send a test pattern to verify the display works */
	dev_info(&pdev->dev, "sending test pattern...\n");
	for (y = 0; y < LCD_HEIGHT; y++)
		for (x = 0; x < LCD_WIDTH; x++)
			lcd->vmem[y * LCD_WIDTH + x] =
				(x < LCD_WIDTH / 2) ? 0x0000 : 0xFFFF;
	lcd_update(lcd);
	/* Clear vmem for framebuffer use */
	memset(lcd->vmem, 0, LCD_WIDTH * LCD_HEIGHT * 2);

	/* Turn on LCD backlight via PWM channel 6 */
	{
		void __iomem *pwm = ioremap(PWM_BASE + PWM_LCD_CH * PWM_CH_SIZE, 8);
		if (pwm) {
			writel(1, pwm + 4);   /* counter top = 1 */
			writel(255, pwm + 0); /* pulse width = 255 (full brightness) */
			iounmap(pwm);
			dev_info(&pdev->dev, "backlight enabled\n");
		}
	}

	dev_info(&pdev->dev, "ST7735 display initialized\n");

	/* Allocate framebuffer */
	info = framebuffer_alloc(0, &pdev->dev);
	if (!info)
		return -ENOMEM;

	info->fbops = &sonata_lcd_ops;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = LCD_WIDTH * 2;
	info->fix.smem_start = (unsigned long)lcd->vmem;
	info->fix.smem_len = vmem_size;
	strscpy(info->fix.id, "sonata-lcd");

	info->var.xres = LCD_WIDTH;
	info->var.yres = LCD_HEIGHT;
	info->var.xres_virtual = LCD_WIDTH;
	info->var.yres_virtual = LCD_HEIGHT;
	info->var.bits_per_pixel = 16;
	/* BGR565 — matches MADCTL 0x68 which sets the BGR bit */
	info->var.red.offset = 0;
	info->var.red.length = 5;
	info->var.green.offset = 5;
	info->var.green.length = 6;
	info->var.blue.offset = 11;
	info->var.blue.length = 5;

	info->screen_buffer = (char *)lcd->vmem;
	info->screen_size = vmem_size;
	info->par = lcd;

	lcd->info = info;
	INIT_DELAYED_WORK(&lcd->work, sonata_lcd_deferred_work);

	if (register_framebuffer(info)) {
		framebuffer_release(info);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, lcd);
	dev_info(&pdev->dev, "fb%d: %dx%d 16bpp ST7735 LCD\n",
		 info->node, LCD_WIDTH, LCD_HEIGHT);

	return 0;
}

static void sonata_lcd_remove(struct platform_device *pdev)
{
	struct sonata_lcd *lcd = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&lcd->work);
	unregister_framebuffer(lcd->info);
	framebuffer_release(lcd->info);
}

static const struct of_device_id sonata_lcd_of_match[] = {
	{ .compatible = "lowrisc,sonata-lcd" },
	{}
};
MODULE_DEVICE_TABLE(of, sonata_lcd_of_match);

static struct platform_driver sonata_lcd_driver = {
	.probe	= sonata_lcd_probe,
	.remove	= sonata_lcd_remove,
	.driver	= {
		.name		= "sonata-lcd",
		.of_match_table	= sonata_lcd_of_match,
	},
};
module_platform_driver(sonata_lcd_driver);

MODULE_AUTHOR("Jonathan");
MODULE_DESCRIPTION("ST7735 LCD framebuffer driver for lowRISC Sonata");
MODULE_LICENSE("GPL");
