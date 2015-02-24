/*
 * drivers/serial/serial_cortina.c
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifdef CONFIG_MAGIC_SYSRQ
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <mach/hardware.h>
#include <asm/io.h>

#if defined(CONFIG_KGDB_SERIAL_CONSOLE) || \
    defined(CONFIG_KGDB_SERIAL_CONSOLE_MODULE)
#include <linux/kgdb.h>
#include <asm/irq_regs.h>
#endif

/***************************************
 *	UART Related registers
 ****************************************/
/* register definitions */
#define	 CFG		UCFG
#define	 FC		UFC
#define	 RX_SAMPLE	URX_SAMPLE
#define	 TX_DAT		UTX_DATA
#define	 RX_DAT		URX_DATA
#define	 INFO		UINFO
#define	 IE		UINT_EN
#define	 INT	 	UINT_CLR
#define	 STATUS		UINT_STAT

/* CFG */
#define	 CFG_STOP_2BIT   (1<<2)
#define	 CFG_PARITY_EVEN (1<<3)
#define	 CFG_PARITY_EN   (1<<4)
#define	 CFG_TX_EN		(1<<5)
#define	 CFG_RX_EN   	(1<<6)
#define	 CFG_UART_EN 	(1<<7)
#define	 CFG_BAUD_SART   8

/* INFO */
#define		INFO_TX_EMPTY   (1<<3)
#define	 INFO_TX_FULL		(1<<2)
#define	 INFO_RX_EMPTY   	(1<<1)
#define	 INFO_RX_FULL		(1<<0)


#define PORT_CORTINA 10
#define CONFIG_SERIAL_CS_CORTINA_CONSOLE
/* Forward decl.s */
static irqreturn_t cortina_uart_int(int irq, void *dev_id);

/* uart_ops functions */
static unsigned int cortina_uart_tx_empty(struct uart_port *port)
{
	/* Return 0 on FIXO condition, TIOCSER_TEMT otherwise */

	return (readl(port->membase + INFO) & 0x8) ?  TIOCSER_TEMT : 0;

}

static void cortina_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* This is your basic 3-wire serial port. None of these signals exist. */
}

static unsigned int cortina_uart_get_mctrl(struct uart_port *port)
{
	/* Claim unimplemented signals asserted, as per Documentation/serial/driver */
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CD;
}

static void cortina_uart_stop_tx(struct uart_port *port)
{
	/* Turn off Tx interrupts. The port lock is held at this point */
	unsigned int temp;
	temp = readl(port->membase + IE);
	writel(temp & ~0x20, port->membase + IE);

}

static inline void cortina_transmit_buffer(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	while (!(readl(port->membase + INFO) & INFO_TX_FULL)) {
		/* send xmit->buf[xmit->tail]
		 * out the port here */
		writel(xmit->buf[xmit->tail], port->membase + TX_DAT);
		xmit->tail = (xmit->tail + 1) &
				 (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_empty(xmit))
		cortina_uart_stop_tx(port);
}
static void cortina_uart_start_tx(struct uart_port *port)
{
	/* Turn on Tx interrupts. The port lock is held at this point */
	unsigned long temp;

	temp = readl(port->membase + IE);
	writel((temp | 0x20), port->membase + IE);

	if(readl(port->membase + INFO) & INFO_TX_EMPTY)
	cortina_transmit_buffer(port);
}

static void cortina_uart_stop_rx(struct uart_port *port)
{
	/* Turn off Rx interrupts. The port lock is held at this point */
	unsigned long temp;

	temp = readl(port->membase + IE);
	writel(temp & ~0x40, port->membase + IE);

}

static void cortina_uart_enable_ms(struct uart_port *port)
{
	/* Nope, you really can't hope to attach a modem to this */
}
/* FIXME no such utility found in cortina hardware */
static void cortina_uart_break_ctl(struct uart_port *port, int ctl)
{
	/* N.A */
}

static int cortina_uart_startup(struct uart_port *port)
{
	unsigned long temp;
	int retval;
	struct platform_clk clk;

	temp = readl(port->membase + IE);
	writel(temp & 0, port->membase + IE);

	retval = request_irq(port->irq, cortina_uart_int, IRQF_DISABLED|IRQF_SAMPLE_RANDOM, "cortina_uart", port);
	if (retval)
		return retval;
	/* The serial core uses this as a cookie, so we should set it even though
		* it doesn't mean much here. */
/*	sport->mapbase =  FIXME check what value should go here */


	/* this may need to be changed Keeps the serial_core calculations happy... */
	get_platform_clk(&clk);
	port->uartclk = clk.apb_clk;	//APB_CLOCK;

	temp = readl(port->membase + CFG);
	temp |= (CFG_UART_EN | CFG_TX_EN| CFG_RX_EN | 0x3 /* data 8 */ );
	writel(temp, port->membase + CFG);
	temp = readl(port->membase + IE);
	writel(temp | 0x60, port->membase + IE);
	return 0;
}

static void cortina_uart_shutdown(struct uart_port *port)
{
	cortina_uart_stop_tx(port);
	cortina_uart_stop_rx(port);
	free_irq(port->irq,port);
}

static void cortina_uart_set_termios(struct uart_port *port, struct ktermios *termios,
								  struct ktermios *old)
{
	unsigned long flags,temp;
	int baud;
	unsigned sample_fre = 0;
	struct platform_clk clk;
	
	get_platform_clk(&clk);
	port->uartclk = clk.apb_clk;
	baud = uart_get_baud_rate(port, termios, old, 0, 115200);
	temp = readl(port->membase + CFG);
	/* mask off the baud settings */
	temp &= 0xff;
	switch (baud) {
		case 9600:
			temp |= (port->uartclk / 9600) << CFG_BAUD_SART ;
			break;
		case 19200:
			temp |= (port->uartclk / 19200) << CFG_BAUD_SART ;
			break;
		case 38400:
			temp |= (port->uartclk / 38400) << CFG_BAUD_SART ;
			break;
		case 57600:
			temp |= (port->uartclk / 57600) << CFG_BAUD_SART ;
			break;
		case 115200:
			temp |= (port->uartclk / 115200) << CFG_BAUD_SART ;
			break;
		default:
			temp |= (port->uartclk / 38400) << CFG_BAUD_SART ;
			break;
	}

	/* Sampling rate should be half of baud count */
	sample_fre = (temp >> CFG_BAUD_SART) / 2;
	/* mask off the data width */
	temp &= 0xfffffffc;
	switch(termios->c_cflag & CSIZE) {
		case CS5:
			temp |= 0x0;
			break;
		case CS6:
			temp |= 0x1;
			break;
		case CS7:
			temp |= 0x2;
			break;
		case CS8:
			default:
			temp |= 0x3;
			break;
	}

	/* mask off Stop bits */
	temp &= ~(CFG_STOP_2BIT);
	if (termios->c_cflag & CSTOPB) {
		temp |= CFG_STOP_2BIT;
	}
	/* Parity */
	temp &= ~(CFG_PARITY_EN);
	temp |= CFG_PARITY_EVEN;
	if (termios->c_cflag & PARENB) {
		temp |= CFG_PARITY_EN;
		if (termios->c_cflag & PARODD)
			temp &= ~(CFG_PARITY_EVEN);
	}

	spin_lock_irqsave(&port->lock, flags);
	writel(temp, port->membase + CFG);
	writel(sample_fre, port->membase + RX_SAMPLE);
	spin_unlock_irqrestore(&port->lock, flags);

}

static const char *cortina_uart_type(struct uart_port *port)
{
	return port->type == PORT_CORTINA ? "Cortina UART" : NULL;
}

static void cortina_uart_release_port(struct uart_port *port)
{
	/* Easy enough */
}

static int cortina_uart_request_port(struct uart_port *port)
{
	return 0;			/* How can we fail? */
}

static void cortina_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_CORTINA;
}

static int cortina_uart_verify_port(struct uart_port *port,
								 struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_CORTINA)
		return -EINVAL;
	return 0;
}

#if defined(CONFIG_KGDB_SERIAL_CONSOLE) || \
    defined(CONFIG_KGDB_SERIAL_CONSOLE_MODULE)

static int cortina_poll_get_char(struct uart_port *port)
{
        unsigned int rx;

        while(readl(port->membase + INFO) & INFO_RX_EMPTY)
                ;
        rx = readl(port->membase + RX_DAT);

        return(rx);
}

static void cortina_poll_put_char(struct uart_port *port, unsigned char c)
{
#define wait_tx_done()  while (!(readl(port->membase + INFO) & INFO_TX_EMPTY));
        unsigned int ie = readl(port->membase + IE);

        // disable the interrupts
        writel(0, port->membase + IE);
        wait_tx_done();

        writel(c, port->membase + TX_DAT);
        wait_tx_done();

        //enable the interrupt
        writel(ie, port->membase + IE);
        return;
}

#endif

static struct uart_ops cortina_uart_ops = {
	.tx_empty = cortina_uart_tx_empty,
	.set_mctrl = cortina_uart_set_mctrl,
	.get_mctrl = cortina_uart_get_mctrl,
	.stop_tx = cortina_uart_stop_tx,
	.start_tx = cortina_uart_start_tx,
	.stop_rx = cortina_uart_stop_rx,
	.enable_ms = cortina_uart_enable_ms,
	.break_ctl = cortina_uart_break_ctl,
	.startup = cortina_uart_startup,
	.shutdown = cortina_uart_shutdown,
	.set_termios = cortina_uart_set_termios,
	.type = cortina_uart_type,
	.release_port = cortina_uart_release_port,
	.request_port = cortina_uart_request_port,
	.config_port = cortina_uart_config_port,
	.verify_port = cortina_uart_verify_port,
#if defined(CONFIG_KGDB_SERIAL_CONSOLE) || \
    defined(CONFIG_KGDB_SERIAL_CONSOLE_MODULE)
        .poll_get_char = cortina_poll_get_char,
        .poll_put_char = cortina_poll_put_char,
#endif
};

/* This is on-chip so there is only ever exactly one of them */
static struct uart_port cortina_uart_port0 = {
	.ops = &cortina_uart_ops,
	.line = 0,
	.irq = GOLDENGATE_IRQ_UART0,
	.membase = (void *)IO_ADDRESS(UART0_BASE_ADDR),
	.mapbase = IO_ADDRESS(UART0_BASE_ADDR),
	.uartclk = G2_UART_CLOCK, /* Peripheral clock == APB clock */
	.type = PORT_CORTINA
};

static struct uart_port cortina_uart_port1 = {
	.ops = &cortina_uart_ops,
	.line = 1,
	.irq = GOLDENGATE_IRQ_UART1,
	.membase = (void *)IO_ADDRESS(UART1_BASE_ADDR),
	.mapbase = IO_ADDRESS(UART1_BASE_ADDR),
	.uartclk = G2_UART_CLOCK, /* Peripheral clock == APB clock */
	.type = PORT_CORTINA
};
static struct uart_port cortina_uart_port2 = {
	.ops = &cortina_uart_ops,
	.line = 2,
	.irq = GOLDENGATE_IRQ_UART2,
	.membase = (void *)IO_ADDRESS(UART2_BASE_ADDR),
	.mapbase = IO_ADDRESS(UART2_BASE_ADDR),
	.uartclk = G2_UART_CLOCK, /* Peripheral clock == APB clock */
	.type = PORT_CORTINA
};

static struct uart_port cortina_uart_port3 = {
	.ops = &cortina_uart_ops,
	.line = 3,
	.irq = GOLDENGATE_IRQ_UART3,
	.membase = (void *)IO_ADDRESS(UART3_BASE_ADDR),
	.mapbase = IO_ADDRESS(UART3_BASE_ADDR),
	.uartclk = G2_UART_CLOCK, /* Peripheral clock == APB clock */
	.type = PORT_CORTINA
};

static inline void cortina_uart_int_rx_chars(struct uart_port *port)
{
	struct tty_struct *tty = port->state->port.tty;
	unsigned int ch;
	unsigned int rx,flg;

	rx = readl(port->membase + INFO);
	if(INFO_RX_EMPTY & rx)
		return ;

	/* Read the character while FIFO is not empty */
	do{
		flg = TTY_NORMAL;
		port->icount.rx++;
		ch = readl(port->membase + RX_DAT);
		if(!(ch & 0x100)) { /* RX char is not valid */
			goto ignore;
		}
		if (uart_handle_sysrq_char(port, (unsigned char)ch)){
			goto ignore;
		}
		tty_insert_flip_char(tty, ch, flg);
ignore:
		rx = readl(port->membase + INFO);
   } while(!(INFO_RX_EMPTY & rx)) ;

	tty_flip_buffer_push(tty);

}


static inline void cortina_uart_int_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

	/* Process out of band chars */
	if (port->x_char) {
		/* Send next char */
		writel(port->x_char, port->membase + TX_DAT);
		goto done;
	}

	/* Nothing to do ? */
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		cortina_uart_stop_tx(port);
		goto done;
	}

	cortina_transmit_buffer(port);

	/* Wake up */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	/* Maybe we're done after all */
	if (uart_circ_empty(xmit)) {
		cortina_uart_stop_tx(port);
		goto done;
	}
	/* Ack the interrupt */
done:
	return;
}

irqreturn_t cortina_uart_int(int irq, void *dev_id)
{
	struct uart_port *port = (struct uart_port *)dev_id;
	unsigned long flags,temp;

	spin_lock_irqsave(&port->lock,flags);

	/* Must clear interrupt first! */
	temp = readl(port->membase + INT);
	writel(temp , port->membase + INT);

	/* Process any Rx chars first */
	cortina_uart_int_rx_chars(port);
	/* Then use any Tx space */
	cortina_uart_int_tx_chars(port);

	spin_unlock_irqrestore(&port->lock,flags);

	return IRQ_HANDLED;
}

#ifdef CONFIG_SERIAL_CS_CORTINA_CONSOLE

void cortina_console_write(struct console *co, const char *s,
							unsigned int count)
{
	struct uart_port *port = &cortina_uart_port0;
	unsigned long previous;
	int i;

	/* Save current state */
	previous = readl(port->membase + IE);
	/* Disable Tx interrupts so this all goes out in one go */
	cortina_uart_stop_tx(port);

	/* Write all the chars */
	for (i = 0; i < count; i++) {

		/* Wait the TX buffer to be empty, which can't take forever:
			* there's no flow control on the UART */
		while (!(readl(port->membase + INFO) & INFO_TX_EMPTY))
			udelay(1);

		/* Send the char */
		writel(*s, port->membase + TX_DAT);

		/* CR/LF stuff */
		if (*s++ == '\n') {
			/* Wait the TX buffer to be empty */
			while (!(readl(port->membase + INFO) & INFO_TX_EMPTY))
					udelay(1);
			writel('\r', port->membase + TX_DAT);
		}
	}

	writel(previous, port->membase + IE); /* Put it all back */
}

static int __init cortina_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &cortina_uart_port0;
	int ret;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	/* This isn't going to do much, but it might change the baud rate. */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	ret = uart_set_options(port, co, baud, parity, bits, flow);

	return 0;
}

static struct uart_driver cortina_uart_driver;	/* Forward decl. */

static struct console cortina_console = {
	.name = "ttyS",
	.write = cortina_console_write,
	.device = uart_console_device,
	.setup = cortina_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,			/* Only possible option. */
	.data = &cortina_uart_driver,
};

static int __init cortina_console_init(void)
{
	cortina_console.index = 0;
	register_console(&cortina_console);
	return 0;
}

console_initcall(cortina_console_init);

#endif

static struct uart_driver cortina_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "cortina_uart",
	.dev_name = "ttyS",
	.major = TTY_MAJOR,
	.minor = 64,
	.nr = 4,
	.cons = &cortina_console,
};

static int __init cortina_uart_init(void)
{
	int ret;
	struct platform_clk clk;

	printk(KERN_INFO "Cortina on-chip UART init.\n");

	ret = readl(IO_ADDRESS(GLOBAL_GPIO_MUX_2));
	ret &= ~0x00003F00;	/* Bit[13:8] */
	writel(ret, IO_ADDRESS(GLOBAL_GPIO_MUX_2));

	cortina_uart_port0.ops = &cortina_uart_ops;
	ret = uart_register_driver(&cortina_uart_driver);
	if (ret == 0) {
		cortina_uart_port0.ops = &cortina_uart_ops;
		cortina_uart_port0.line = 0;
		cortina_uart_port0.irq = GOLDENGATE_IRQ_UART0;
		cortina_uart_port0.membase = (void *)IO_ADDRESS(UART0_BASE_ADDR);
		cortina_uart_port0.mapbase = IO_ADDRESS(UART0_BASE_ADDR);
		get_platform_clk(&clk);
		cortina_uart_port0.uartclk = clk.apb_clk;
		ret = uart_add_one_port(&cortina_uart_driver, &cortina_uart_port0);
		if (ret == 0) {
			cortina_uart_port1.ops = &cortina_uart_ops;
			cortina_uart_port1.line = 1;
			cortina_uart_port1.irq = GOLDENGATE_IRQ_UART1;
			cortina_uart_port1.membase = (void *)IO_ADDRESS(UART1_BASE_ADDR);
			cortina_uart_port1.mapbase = IO_ADDRESS(UART1_BASE_ADDR);
			cortina_uart_port1.uartclk = clk.apb_clk;
			ret = uart_add_one_port(&cortina_uart_driver, &cortina_uart_port1);
		}
		if (ret == 0) {
			cortina_uart_port2.ops = &cortina_uart_ops;
			cortina_uart_port2.line = 2;
			cortina_uart_port2.irq = GOLDENGATE_IRQ_UART2;
			cortina_uart_port2.membase = (void *)IO_ADDRESS(UART2_BASE_ADDR);
			cortina_uart_port2.mapbase = IO_ADDRESS(UART2_BASE_ADDR);
			cortina_uart_port2.uartclk = clk.apb_clk;
			ret = uart_add_one_port(&cortina_uart_driver, &cortina_uart_port2);
		}
		if (ret == 0) {
			cortina_uart_port3.ops = &cortina_uart_ops;
			cortina_uart_port3.line = 3;
			cortina_uart_port3.irq = GOLDENGATE_IRQ_UART3;
			cortina_uart_port3.membase = (void *)IO_ADDRESS(UART3_BASE_ADDR);
			cortina_uart_port3.mapbase = IO_ADDRESS(UART3_BASE_ADDR);
			cortina_uart_port3.uartclk = clk.apb_clk;
			ret = uart_add_one_port(&cortina_uart_driver, &cortina_uart_port3);
		}
	}
	return ret;
}

static void __exit cortina_uart_exit(void)
{
		uart_remove_one_port(&cortina_uart_driver,&cortina_uart_port0);
		uart_remove_one_port(&cortina_uart_driver,&cortina_uart_port1);
		uart_remove_one_port(&cortina_uart_driver,&cortina_uart_port2);
		uart_remove_one_port(&cortina_uart_driver,&cortina_uart_port3);
		uart_unregister_driver(&cortina_uart_driver);
}

module_init(cortina_uart_init);
module_exit(cortina_uart_exit);

MODULE_AUTHOR("Cortina-Systems");
MODULE_DESCRIPTION(" Cortina UART driver");
MODULE_LICENSE("GPL");

