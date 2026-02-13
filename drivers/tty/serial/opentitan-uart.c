// SPDX-License-Identifier: GPL-2.0
/*
 * OpenTitan UART serial driver for lowRISC Sonata.
 *
 * Register-compatible with the OpenTitan UART IP block.
 * Reference: OpenTitan UART Hardware Interface Specification.
 *
 * Copyright (C) 2026 Jonathan
 */

#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/tty_flip.h>

/* OpenTitan UART register offsets */
#define OT_UART_INTR_STATE	0x00	/* W1C interrupt status */
#define OT_UART_INTR_ENABLE	0x04	/* Interrupt enable */
#define OT_UART_INTR_TEST	0x08	/* Interrupt test (write-only) */
#define OT_UART_CTRL		0x10	/* Control: NCO[31:16], parity_odd[6],
					   parity_en[5], rxblvl[4:2],
					   nf[1], rx_en, tx_en[0] */
#define OT_UART_STATUS		0x14	/* Status register */
#define OT_UART_RDATA		0x18	/* RX data */
#define OT_UART_WDATA		0x1C	/* TX data */
#define OT_UART_FIFO_CTRL	0x20	/* FIFO control */
#define OT_UART_FIFO_STATUS	0x24	/* FIFO levels */
#define OT_UART_OVRD		0x28	/* TX override */
#define OT_UART_VAL		0x2C	/* Oversampled RX value */
#define OT_UART_TIMEOUT_CTRL	0x30	/* RX timeout */

/* CTRL register bits */
#define OT_UART_CTRL_TX_EN	BIT(0)
#define OT_UART_CTRL_RX_EN	BIT(1)
#define OT_UART_CTRL_NCO_SHIFT	16

/* STATUS register bits */
#define OT_UART_STATUS_TXFULL	BIT(0)
#define OT_UART_STATUS_RXFULL	BIT(1)
#define OT_UART_STATUS_TXEMPTY	BIT(2)
#define OT_UART_STATUS_TXIDLE	BIT(3)
#define OT_UART_STATUS_RXIDLE	BIT(4)
#define OT_UART_STATUS_RXEMPTY	BIT(5)

/* Interrupt bits (INTR_STATE / INTR_ENABLE) */
#define OT_UART_INTR_TX_WATERMARK	BIT(0)
#define OT_UART_INTR_RX_WATERMARK	BIT(1)
#define OT_UART_INTR_TX_DONE		BIT(2)
#define OT_UART_INTR_RX_OVERFLOW	BIT(3)
#define OT_UART_INTR_RX_FRAME_ERR	BIT(4)
#define OT_UART_INTR_RX_BREAK_ERR	BIT(5)
#define OT_UART_INTR_RX_TIMEOUT	BIT(6)
#define OT_UART_INTR_RX_PARITY_ERR	BIT(7)
#define OT_UART_INTR_TX_EMPTY		BIT(8)

/* FIFO_STATUS */
#define OT_UART_FIFO_STATUS_TXLVL_MASK	0xFF
#define OT_UART_FIFO_STATUS_RXLVL_SHIFT 16
#define OT_UART_FIFO_STATUS_RXLVL_MASK	0xFF

#define OT_UART_MAX_PORTS	3
#define OT_UART_FIFO_SIZE	32
#define OT_UART_DEV_NAME	"ttyOT"
#define OT_UART_DRIVER_NAME	"opentitan-uart"

struct ot_uart_port {
	struct uart_port port;
	struct timer_list timer;
	struct clk *clk;
	unsigned long clk_freq;
};

#define to_ot_uart_port(p)	container_of(p, struct ot_uart_port, port)

static struct ot_uart_port ot_uart_ports[OT_UART_MAX_PORTS];

static inline u32 ot_uart_read(struct uart_port *port, unsigned int off)
{
	return readl(port->membase + off);
}

static inline void ot_uart_write(struct uart_port *port, unsigned int off,
				 u32 val)
{
	writel(val, port->membase + off);
}

/*
 * Calculate NCO value for baud rate.
 * NCO = (baud_rate << 20) / clk_freq
 */
static u32 ot_uart_calc_nco(unsigned long clk_freq, unsigned int baud)
{
	return (u32)(((u64)baud << 20) / clk_freq);
}

static void ot_uart_set_baud(struct uart_port *port, unsigned int baud)
{
	struct ot_uart_port *uart = to_ot_uart_port(port);
	u32 nco = ot_uart_calc_nco(uart->clk_freq, baud);
	u32 ctrl;

	ctrl = (nco << OT_UART_CTRL_NCO_SHIFT) |
	       OT_UART_CTRL_TX_EN | OT_UART_CTRL_RX_EN;
	ot_uart_write(port, OT_UART_CTRL, ctrl);
}

/* --- TX / RX helpers --- */

static void ot_uart_rx_chars(struct uart_port *port)
{
	u32 status;
	u8 ch;

	status = ot_uart_read(port, OT_UART_STATUS);
	while (!(status & OT_UART_STATUS_RXEMPTY)) {
		ch = ot_uart_read(port, OT_UART_RDATA) & 0xFF;
		port->icount.rx++;

		if (!uart_handle_sysrq_char(port, ch))
			uart_insert_char(port, 0, 0, ch, TTY_NORMAL);

		status = ot_uart_read(port, OT_UART_STATUS);
	}

	tty_flip_buffer_push(&port->state->port);
}

static void ot_uart_tx_chars(struct uart_port *port)
{
	u8 ch;

	uart_port_tx(port, ch,
		!(ot_uart_read(port, OT_UART_STATUS) & OT_UART_STATUS_TXFULL),
		ot_uart_write(port, OT_UART_WDATA, ch));
}

/* --- Interrupt handler --- */

static irqreturn_t ot_uart_interrupt(int irq, void *data)
{
	struct ot_uart_port *uart = data;
	struct uart_port *port = &uart->port;
	unsigned long flags;
	u32 isr;

	uart_port_lock_irqsave(port, &flags);

	isr = ot_uart_read(port, OT_UART_INTR_STATE);
	/* Clear all pending interrupts (W1C) */
	ot_uart_write(port, OT_UART_INTR_STATE, isr);

	if (isr & (OT_UART_INTR_RX_WATERMARK | OT_UART_INTR_RX_TIMEOUT))
		ot_uart_rx_chars(port);

	if (isr & (OT_UART_INTR_TX_WATERMARK | OT_UART_INTR_TX_EMPTY))
		ot_uart_tx_chars(port);

	uart_port_unlock_irqrestore(port, flags);

	return IRQ_RETVAL(isr);
}

/* Polling fallback timer */
static void ot_uart_timer(struct timer_list *t)
{
	struct ot_uart_port *uart = timer_container_of(uart, t, timer);
	struct uart_port *port = &uart->port;

	ot_uart_interrupt(0, uart);
	mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
}

/* --- uart_ops --- */

static unsigned int ot_uart_tx_empty(struct uart_port *port)
{
	u32 status = ot_uart_read(port, OT_UART_STATUS);

	return (status & OT_UART_STATUS_TXEMPTY) ? TIOCSER_TEMT : 0;
}

static void ot_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int ot_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void ot_uart_stop_tx(struct uart_port *port)
{
	u32 ie = ot_uart_read(port, OT_UART_INTR_ENABLE);

	ie &= ~(OT_UART_INTR_TX_WATERMARK | OT_UART_INTR_TX_EMPTY);
	ot_uart_write(port, OT_UART_INTR_ENABLE, ie);
}

static void ot_uart_start_tx(struct uart_port *port)
{
	u32 ie = ot_uart_read(port, OT_UART_INTR_ENABLE);

	ie |= OT_UART_INTR_TX_WATERMARK;
	ot_uart_write(port, OT_UART_INTR_ENABLE, ie);

	/* Kick off TX immediately */
	ot_uart_tx_chars(port);
}

static void ot_uart_stop_rx(struct uart_port *port)
{
	u32 ie = ot_uart_read(port, OT_UART_INTR_ENABLE);

	ie &= ~(OT_UART_INTR_RX_WATERMARK | OT_UART_INTR_RX_TIMEOUT);
	ot_uart_write(port, OT_UART_INTR_ENABLE, ie);
}

static int ot_uart_startup(struct uart_port *port)
{
	struct ot_uart_port *uart = to_ot_uart_port(port);
	int ret;

	/* Clear any pending interrupts */
	ot_uart_write(port, OT_UART_INTR_STATE, 0x1FF);

	if (port->irq) {
		ret = request_irq(port->irq, ot_uart_interrupt, 0,
				  OT_UART_DRIVER_NAME, uart);
		if (ret) {
			dev_warn(port->dev,
				 "line %d irq %d failed: using polling\n",
				 port->line, port->irq);
			port->irq = 0;
		}
	}

	/* Enable RX interrupts */
	ot_uart_write(port, OT_UART_INTR_ENABLE,
		      OT_UART_INTR_RX_WATERMARK | OT_UART_INTR_RX_TIMEOUT);

	if (!port->irq) {
		timer_setup(&uart->timer, ot_uart_timer, 0);
		mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
	}

	return 0;
}

static void ot_uart_shutdown(struct uart_port *port)
{
	struct ot_uart_port *uart = to_ot_uart_port(port);

	/* Disable all interrupts */
	ot_uart_write(port, OT_UART_INTR_ENABLE, 0);
	ot_uart_write(port, OT_UART_INTR_STATE, 0x1FF);

	if (port->irq)
		free_irq(port->irq, uart);
	else
		timer_delete_sync(&uart->timer);
}

static void ot_uart_set_termios(struct uart_port *port,
				struct ktermios *new,
				const struct ktermios *old)
{
	unsigned int baud;
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	baud = uart_get_baud_rate(port, new, old, 0, 460800);
	uart_update_timeout(port, new->c_cflag, baud);
	ot_uart_set_baud(port, baud);
	uart_port_unlock_irqrestore(port, flags);
}

static const char *ot_uart_type(struct uart_port *port)
{
	return OT_UART_DRIVER_NAME;
}

static void ot_uart_config_port(struct uart_port *port, int flags)
{
	port->type = 1;
}

static int ot_uart_verify_port(struct uart_port *port,
			       struct serial_struct *ser)
{
	if (port->type != PORT_UNKNOWN && ser->type != 1)
		return -EINVAL;
	return 0;
}

static const struct uart_ops ot_uart_ops = {
	.tx_empty	= ot_uart_tx_empty,
	.set_mctrl	= ot_uart_set_mctrl,
	.get_mctrl	= ot_uart_get_mctrl,
	.stop_tx	= ot_uart_stop_tx,
	.start_tx	= ot_uart_start_tx,
	.stop_rx	= ot_uart_stop_rx,
	.startup	= ot_uart_startup,
	.shutdown	= ot_uart_shutdown,
	.set_termios	= ot_uart_set_termios,
	.type		= ot_uart_type,
	.config_port	= ot_uart_config_port,
	.verify_port	= ot_uart_verify_port,
};

/* --- Console support --- */

static void ot_uart_putchar(struct uart_port *port, unsigned char ch)
{
	while (ot_uart_read(port, OT_UART_STATUS) & OT_UART_STATUS_TXFULL)
		cpu_relax();
	ot_uart_write(port, OT_UART_WDATA, ch);
}

#ifdef CONFIG_SERIAL_OPENTITAN_CONSOLE

static struct uart_driver ot_uart_driver;

static void ot_uart_console_putchar(unsigned char ch)
{
	volatile unsigned int *uart = (volatile unsigned int *)0x80100000UL;

	while (uart[0x14/4] & 1) /* TX_FULL */
		;
	uart[0x1c/4] = ch;
}

static void ot_uart_console_write(struct console *co, const char *s,
				  unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		if (s[i] == '\n')
			ot_uart_console_putchar('\r');
		ot_uart_console_putchar(s[i]);
	}
}

static int ot_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index >= OT_UART_MAX_PORTS || co->index < 0)
		return -EINVAL;

	port = &ot_uart_ports[co->index].port;
	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console ot_uart_console = {
	.name	= OT_UART_DEV_NAME,
	.write	= ot_uart_console_write,
	.device	= uart_console_device,
	.setup	= ot_uart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &ot_uart_driver,
};

/*
 * Disabled: console_initcall drains the printk buffer which was causing
 * issues.  earlycon provides output for now.
 */
#if 0
static int __init ot_uart_console_init(void)
{
	register_console(&ot_uart_console);
	return 0;
}
console_initcall(ot_uart_console_init);
#endif

#define OT_UART_CONSOLE	(&ot_uart_console)

/* Earlycon support for very early boot output */
static void early_ot_uart_write(struct console *console, const char *s,
				unsigned int count)
{
	struct earlycon_device *device = console->data;
	struct uart_port *port = &device->port;

	uart_console_write(port, s, count, ot_uart_putchar);
}

static int __init early_ot_uart_setup(struct earlycon_device *device,
				      const char *options)
{
	struct uart_port *port = &device->port;
	unsigned long clk = device->baud ? device->port.uartclk : 40000000;
	unsigned int baud = device->baud ? device->baud : 115200;
	u32 nco, ctrl;

	if (!port->membase)
		return -ENODEV;

	/* Configure baud rate — there is no bootloader to do this */
	nco = (u32)(((u64)baud << 20) / clk);
	ctrl = (nco << OT_UART_CTRL_NCO_SHIFT) |
	       OT_UART_CTRL_TX_EN | OT_UART_CTRL_RX_EN;
	writel(ctrl, port->membase + OT_UART_CTRL);

	device->con->write = early_ot_uart_write;
	return 0;
}

OF_EARLYCON_DECLARE(opentitan_uart, "lowrisc,opentitan-uart",
		    early_ot_uart_setup);

#else
#define OT_UART_CONSOLE	NULL
#endif /* CONFIG_SERIAL_OPENTITAN_CONSOLE */

static struct uart_driver ot_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= OT_UART_DRIVER_NAME,
	.dev_name	= OT_UART_DEV_NAME,
	.major		= 0,
	.minor		= 0,
	.nr		= OT_UART_MAX_PORTS,
	.cons		= OT_UART_CONSOLE,
};

/* --- Platform driver --- */

static int ot_uart_probe(struct platform_device *pdev)
{
	struct ot_uart_port *uart;
	struct uart_port *port;
	struct resource *res;
	int dev_id, ret;

	dev_id = of_alias_get_id(pdev->dev.of_node, "serial");
	if (dev_id < 0)
		dev_id = pdev->id;
	if (dev_id < 0) {
		/* Auto-assign based on device count */
		static int next_id;
		dev_id = next_id++;
	}
	if (dev_id >= OT_UART_MAX_PORTS)
		return -EINVAL;

	uart = &ot_uart_ports[dev_id];
	port = &uart->port;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	port->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	port->mapbase = res->start;

	ret = platform_get_irq_optional(pdev, 0);
	if (ret > 0)
		port->irq = ret;

	uart->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(uart->clk))
		return PTR_ERR(uart->clk);

	if (uart->clk) {
		ret = clk_prepare_enable(uart->clk);
		if (ret)
			return ret;
		uart->clk_freq = clk_get_rate(uart->clk);
	} else {
		/* Default to 40 MHz if no clock provider */
		uart->clk_freq = 40000000;
	}

	port->dev = &pdev->dev;
	port->iotype = UPIO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->ops = &ot_uart_ops;
	port->fifosize = OT_UART_FIFO_SIZE;
	port->type = PORT_UNKNOWN;
	port->line = dev_id;
	port->uartclk = uart->clk_freq;
	spin_lock_init(&port->lock);

	platform_set_drvdata(pdev, port);

	/* Set default baud rate so the console works immediately */
	ot_uart_set_baud(port, 115200);

	ret = uart_add_one_port(&ot_uart_driver, port);
	if (ret) {
		if (uart->clk)
			clk_disable_unprepare(uart->clk);
		return ret;
	}

	return 0;
}

static void ot_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct ot_uart_port *uart = to_ot_uart_port(port);

	uart_remove_one_port(&ot_uart_driver, port);
	if (uart->clk)
		clk_disable_unprepare(uart->clk);
}

static const struct of_device_id ot_uart_of_match[] = {
	{ .compatible = "lowrisc,opentitan-uart" },
	{}
};
MODULE_DEVICE_TABLE(of, ot_uart_of_match);

static struct platform_driver ot_uart_platform_driver = {
	.probe	= ot_uart_probe,
	.remove	= ot_uart_remove,
	.driver	= {
		.name		= OT_UART_DRIVER_NAME,
		.of_match_table	= ot_uart_of_match,
	},
};

static int __init ot_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&ot_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&ot_uart_platform_driver);
	if (ret)
		uart_unregister_driver(&ot_uart_driver);

	return ret;
}

static void __exit ot_uart_exit(void)
{
	platform_driver_unregister(&ot_uart_platform_driver);
	uart_unregister_driver(&ot_uart_driver);
}

module_init(ot_uart_init);
module_exit(ot_uart_exit);

MODULE_AUTHOR("Jonathan");
MODULE_DESCRIPTION("OpenTitan UART serial driver for lowRISC Sonata");
MODULE_LICENSE("GPL");
