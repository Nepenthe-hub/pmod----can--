#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include "ch9434.h"

int ch943x_set_baud(struct uart_port *port, int baud)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 lcr;
	unsigned long clk = port->uartclk;
	unsigned long div;
	uint8_t dll = 0;
	uint8_t dlm = 0;
	uint32_t x;
	uint64_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	dev_dbg(s->dev, "%s - %d\n", __func__, baud);

	if (s->chiptype == CHIP_CH9434D) {
		integerdivider = (((u64)25 * 96000000) / (4 * baud));
		x = (integerdivider / 100) << 4;
		fractionaldivider = integerdivider - (100 * (x >> 4));
		x |= ((((fractionaldivider * 16) + 50) / 100)) &
		     ((uint8_t)0x0F);
		dll = x & 0xff;
		dlm = (x >> 8) & 0xff;
	} else {
		div = clk * 10 / 8 / baud;
		div = (div + 5) / 10;
	}

	lcr = ch943x_port_read(port, CH943X_LCR_REG);

	/* Open the LCR divisors for configuration */
	ch943x_port_write(port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A);

	/* Write the new divisor */
	if (s->chiptype == CHIP_CH9434D) {
		ch943x_port_write(port, CH943X_DLL_REG, dll);
		ch943x_port_write(port, CH943X_DLH_REG, dlm);
	} else {
		ch943x_port_write(port, CH943X_DLH_REG, div / 256);
		ch943x_port_write(port, CH943X_DLL_REG, div % 256);
	}

	/* Put LCR back to the normal mode */
	ch943x_port_write(port, CH943X_LCR_REG, lcr);

	return DIV_ROUND_CLOSEST(clk / 16, div);
}

int ch943x_dump_register(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 lcr;
	u8 i, reg;

	lcr = ch943x_port_read(port, CH943X_LCR_REG);
	ch943x_port_write(port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A);
	dev_dbg(s->dev, "******Dump register at LCR=DLAB\n");
	for (i = 0; i < 1; i++) {
		reg = ch943x_port_read(port, i);
		dev_dbg(s->dev, "Reg[0x%02x] = 0x%02x\n", i, reg);
	}

	ch943x_port_update(port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A,
			   0);
	dev_dbg(s->dev, "******Dump register at LCR=Normal\n");
	for (i = 0; i < 8; i++) {
		reg = ch943x_port_read(port, i);
		dev_dbg(s->dev, "Reg[0x%02x] = 0x%02x\n", i, reg);
	}

	/* Put LCR back to the normal mode */
	ch943x_port_write(port, CH943X_LCR_REG, lcr);

	return 0;
}

void ch943x_handle_rx(struct uart_port *port, unsigned int rxlen,
		      unsigned int iir)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	unsigned int lsr = 0, ch, flag, bytes_read = 0, i;
	bool read_lsr = (iir == CH943X_IIR_RLSE_SRC) ? true : false;

	dev_dbg(s->dev, "%s\n", __func__);

	if (unlikely(rxlen >= sizeof(s->buf))) {
		port->icount.buf_overrun++;
		rxlen = sizeof(s->buf);
	}

	/* Only read lsr if there are possible errors in FIFO */
	if (read_lsr) {
		lsr = ch943x_port_read(port, CH943X_LSR_REG);
		/* No errors left in FIFO */
		if (!(lsr & CH943X_LSR_FIFOE_BIT))
			read_lsr = false;
	} else
		lsr = 0;

	if (s->spi_contmode) {
		ch943x_raw_read(port, CH943X_RHR_REG, s->buf, rxlen);
		bytes_read += rxlen;
	} else {
		for (i = 0; i < rxlen; i++) {
			s->buf[i] = ch943x_port_read(port, CH943X_RHR_REG);
			bytes_read++;
		}
	}

	flag = TTY_NORMAL;
	port->icount.rx++;

	if (unlikely(lsr & CH943X_LSR_BRK_ERROR_MASK)) {
		dev_err(s->dev, "%s - lsr error detect\n", __func__);
		if (lsr & CH943X_LSR_BI_BIT) {
			lsr &= ~(CH943X_LSR_FE_BIT | CH943X_LSR_PE_BIT);
			port->icount.brk++;
			if (uart_handle_break(port))
				goto ignore_char;
		} else if (lsr & CH943X_LSR_PE_BIT)
			port->icount.parity++;
		else if (lsr & CH943X_LSR_FE_BIT)
			port->icount.frame++;
		else if (lsr & CH943X_LSR_OE_BIT)
			port->icount.overrun++;

		lsr &= port->read_status_mask;
		if (lsr & CH943X_LSR_BI_BIT)
			flag = TTY_BREAK;
		else if (lsr & CH943X_LSR_PE_BIT)
			flag = TTY_PARITY;
		else if (lsr & CH943X_LSR_FE_BIT)
			flag = TTY_FRAME;

		if (lsr & CH943X_LSR_OE_BIT)
			dev_err(s->dev, "%s - overrun detect\n", __func__);
	}

	for (i = 0; i < bytes_read; i++) {
		ch = s->buf[i];

		if (uart_handle_sysrq_char(port, ch))
			continue;

		if (lsr & port->ignore_status_mask)
			continue;

		uart_insert_char(port, lsr, CH943X_LSR_OE_BIT, ch, flag);
	}

	dev_dbg(s->dev, "%s - bytes_read:%d\n", __func__, bytes_read);

ignore_char:
	tty_flip_buffer_push(&port->state->port);
}

void ch943x_handle_tx(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int txlen, to_send, i;
	unsigned char thr_reg;

	dev_dbg(s->dev, "%s\n", __func__);
	/* xon/xoff char */
	if (unlikely(port->x_char)) {
		ch943x_port_write(port, CH943X_THR_REG, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		dev_dbg(s->dev, "ch943x_handle_tx stopped\n");
		ch943x_port_update(port, CH943X_IER_REG,
				   CH943X_IER_THRI_BIT, 0);
		return;
	}

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);

	if (likely(to_send)) {
		/* Limit to size of TX FIFO */
		txlen = CH943X_FIFO_SIZE;
		to_send = (to_send > txlen) ? txlen : to_send;

		/* Add data to send */
		port->icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			s->buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) &
				     (UART_XMIT_SIZE - 1);
		}
		dev_dbg(s->dev, "ch943x_handle_tx %d bytes\n", to_send);
		thr_reg = (0x80 | CH943X_THR_REG) + (port->line * 0x10);
		ch943x_raw_write(port, &thr_reg, s->buf, to_send);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

void ch943x_port_irq(struct ch943x_port *s, int portno)
{
	struct uart_port *port = &s->p[portno].port;
	unsigned int iir, msr, rxlen = 0;
	unsigned char lsr;

	lsr = ch943x_port_read(port, CH943X_LSR_REG);
	if (lsr & 0x02) {
		dev_err(port->dev, "Rx Overrun portno = %d, lsr = 0x%2x\n",
			portno, lsr);
	}

	iir = ch943x_port_read(port, CH943X_IIR_REG);
	if (iir & CH943X_IIR_NO_INT_BIT) {
		dev_dbg(s->dev, "%s no int or port not open, quit\n",
			__func__);
		return;
	}
	iir &= CH943X_IIR_ID_MASK;
	switch (iir) {
	case CH943X_IIR_RDI_SRC:
	case CH943X_IIR_RLSE_SRC:
	case CH943X_IIR_RTOI_SRC:
		ch943x_reg_write(s, CH943X_FIFO_REG,
				 port->line | CH943X_FIFO_RD_BIT);
		rxlen = ch943x_reg_read(s, CH943X_FIFOCL_REG);
		rxlen |= ch943x_reg_read(s, CH943X_FIFOCH_REG) << 8;
		dev_dbg(s->dev, "%s rxlen = %d\n", __func__, rxlen);

		if (rxlen == 0xFFFF) {
			dev_err(port->dev, "Illegal length:%d", rxlen);
			break;
		}
		if (rxlen > 2048)
			rxlen = 2048;
		ch943x_handle_rx(port, rxlen, iir);
		break;
	case CH943X_IIR_MSI_SRC:
		msr = ch943x_port_read(port, CH943X_MSR_REG);
		s->p[portno].msr_reg = msr;
		uart_handle_cts_change(port, !!(msr & CH943X_MSR_CTS_BIT));
		dev_dbg(s->dev, "uart_handle_modem_change = 0x%02x\n",
			msr);
		break;
	case CH943X_IIR_THRI_SRC:
		mutex_lock(&s->mutex);
		ch943x_handle_tx(port);
		mutex_unlock(&s->mutex);
		break;
	default:
		dev_err(port->dev, "Port %i: Unexpected interrupt: %x",
			port->line, iir);
		break;
	}
}

void ch943x_wq_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, tx_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(s->dev, "%s\n", __func__);
	mutex_lock(&s->mutex);
	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_THRI_BIT,
			   CH943X_IER_THRI_BIT);
	mutex_unlock(&s->mutex);
}

void ch943x_stop_tx(struct uart_port *port)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(s->dev, "%s\n", __func__);
	schedule_work(&one->stop_tx_work);
}

void ch943x_stop_rx(struct uart_port *port)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(s->dev, "%s\n", __func__);
	schedule_work(&one->stop_rx_work);
}

void ch943x_start_tx(struct uart_port *port)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(s->dev, "%s\n", __func__);

	/* handle rs485 */
	if ((one->rs485.flags & SER_RS485_ENABLED) &&
	    (one->rs485.delay_rts_before_send > 0)) {
		mdelay(one->rs485.delay_rts_before_send);
	}
	if (!work_pending(&one->tx_work)) {
		dev_dbg(s->dev, "%s schedule\n", __func__);
		schedule_work(&one->tx_work);
	}
}

void ch943x_stop_rx_work_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, stop_rx_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(s->dev, "%s\n", __func__);
	mutex_lock(&s->mutex);
	one->port.read_status_mask &= ~CH943X_LSR_DR_BIT;
	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_RDI_BIT,
			   0);
	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_RLSI_BIT,
			   0);
	mutex_unlock(&s->mutex);
}

void ch943x_stop_tx_work_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, stop_tx_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);
	struct circ_buf *xmit = &one->port.state->xmit;

	dev_dbg(s->dev, "%s\n", __func__);
	mutex_lock(&s->mutex);
	/* handle rs485 */
	if (one->rs485.flags & SER_RS485_ENABLED) {
		/* do nothing if current tx not yet completed */
		int lsr = ch943x_port_read(&one->port, CH943X_LSR_REG);
		if (!(lsr & CH943X_LSR_TEMT_BIT)) {
			mutex_unlock(&s->mutex);
			return;
		}
		if (uart_circ_empty(xmit) &&
		    (one->rs485.delay_rts_after_send > 0))
			mdelay(one->rs485.delay_rts_after_send);
	}

	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_THRI_BIT,
			   0);
	mutex_unlock(&s->mutex);
}

unsigned int ch943x_tx_empty(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	unsigned int lsr;
	unsigned int result;

	dev_dbg(s->dev, "%s\n", __func__);
	lsr = ch943x_port_read(port, CH943X_LSR_REG);
	result = (lsr & CH943X_LSR_THRE_BIT) ? TIOCSER_TEMT : 0;

	return result;
}

unsigned int ch943x_get_mctrl(struct uart_port *port)
{
	unsigned int status, ret;
	struct ch943x_port *s = dev_get_drvdata(port->dev);

	dev_dbg(s->dev, "%s\n", __func__);
	status = s->p[port->line].msr_reg;
	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;

	return ret;
}

void ch943x_md_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, md_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);
	unsigned int mctrl = one->port.mctrl;
	unsigned char mcr = 0;

	if (s->chiptype == CHIP_CH9434D) {
		if (one->port.line == 3) { // RTS3/DTR3
			dev_err(s->dev,
				"%s - port %d RTS/DTR is unavailable.\n",
				__func__, one->port.line);
			return;
		} else if (one->port.line == 0) { // RTS0/DTR0
#if defined(CH9434D_TNOW1_ON) || defined(CH9434D_CAN_ON)
			dev_err(s->dev,
				"%s - port %d RTS/DTR is unavailable.\n",
				__func__, one->port.line);
			return;
#endif
		}
	}

	if (mctrl & TIOCM_RTS) {
		mcr |= UART_MCR_RTS;
	}
	if (mctrl & TIOCM_DTR) {
		mcr |= UART_MCR_DTR;
	}
	if (mctrl & TIOCM_OUT1) {
		mcr |= UART_MCR_OUT1;
	}
	if (mctrl & TIOCM_OUT2) {
		mcr |= UART_MCR_OUT2;
	}
	if (mctrl & TIOCM_LOOP) {
		mcr |= UART_MCR_LOOP;
	}

	mcr |= one->mcr_force;

	dev_dbg(s->dev, "%s - mcr:0x%x, force:0x%2x\n", __func__, mcr,
		one->mcr_force);

	ch943x_port_write(&one->port, CH943X_MCR_REG, mcr);
}

void ch943x_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(s->dev, "%s - mctrl:0x%x\n", __func__, mctrl);

	schedule_work(&one->md_work);
}

void ch943x_break_ctl(struct uart_port *port, int break_state)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);

	dev_dbg(s->dev, "%s\n", __func__);
	if (s->chiptype == CHIP_CH9434D)
		return;
	else
		ch943x_port_update(
			port, CH943X_LCR_REG, CH943X_LCR_TXBREAK_BIT,
			break_state ? CH943X_LCR_TXBREAK_BIT : 0);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
void ch943x_set_termios(struct uart_port *port, struct ktermios *termios,
			const struct ktermios *old)
#else
void ch943x_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old)
#endif
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	unsigned int lcr;
	int baud;
	u8 bParityType;

	dev_dbg(s->dev, "%s\n", __func__);

	/* Word size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = CH943X_LCR_WORD_LEN_5;
		break;
	case CS6:
		lcr = CH943X_LCR_WORD_LEN_6;
		break;
	case CS7:
		lcr = CH943X_LCR_WORD_LEN_7;
		break;
	case CS8:
		lcr = CH943X_LCR_WORD_LEN_8;
		break;
	default:
		lcr = CH943X_LCR_WORD_LEN_8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
		break;
	}

	bParityType = termios->c_cflag & PARENB ?
			      (termios->c_cflag & PARODD ? 1 : 2) +
				      (termios->c_cflag & CMSPAR ? 2 : 0) :
			      0;
	lcr |= CH943X_LCR_PARITY_BIT;

	switch (bParityType) {
	case 0x01:
		lcr |= CH943X_LCR_ODDPARITY_BIT;
		dev_dbg(s->dev, "parity = odd\n");
		break;
	case 0x02:
		lcr |= CH943X_LCR_EVENPARITY_BIT;
		dev_dbg(s->dev, "parity = even\n");
		break;
	case 0x03:
		lcr |= CH943X_LCR_MARKPARITY_BIT;
		dev_dbg(s->dev, "parity = mark\n");
		break;
	case 0x04:
		lcr |= CH943X_LCR_SPACEPARITY_BIT;
		dev_dbg(s->dev, "parity = space\n");
		break;
	default:
		lcr &= ~CH943X_LCR_PARITY_BIT;
		dev_dbg(s->dev, "parity = none\n");
		break;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= CH943X_LCR_STOPLEN_BIT; /* 2 stops */

	/* Set read status mask */
	port->read_status_mask = CH943X_LSR_OE_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= CH943X_LSR_PE_BIT |
					  CH943X_LSR_FE_BIT;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= CH943X_LSR_BI_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= CH943X_LSR_BI_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= CH943X_LSR_BRK_ERROR_MASK;

	/* Update LCR register */
	if (s->chiptype == CHIP_CH9434D) {
		if ((bParityType == 0x01) || (bParityType == 0x02)) {
			lcr &= ~(BIT(0) | BIT(1)); // 9位模式
		}
	}
	ch943x_port_write(port, CH943X_LCR_REG, lcr);

	/* Configure flow control */
	if (termios->c_cflag & CRTSCTS) {
		dev_dbg(s->dev, "ch943x_set_termios enable rts/cts\n");
		ch943x_port_update(port, CH943X_MCR_REG,
				   CH943X_MCR_AFE | CH943X_MCR_RTS_BIT,
				   CH943X_MCR_AFE | CH943X_MCR_RTS_BIT);
		one->mcr_force |= CH943X_MCR_AFE | CH943X_MCR_RTS_BIT;

		// add on 20200608 suppose cts status is always valid here
		uart_handle_cts_change(port, 1);
	} else {
		dev_dbg(s->dev, "ch943x_set_termios disable rts/cts\n");
		ch943x_port_update(port, CH943X_MCR_REG, CH943X_MCR_AFE,
				   0);
		one->mcr_force &= ~(CH943X_MCR_AFE | CH943X_MCR_RTS_BIT);
	}

	/* Get baud rate generator configuration */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 16 * 24);
	/* Setup baudrate generator */
	baud = ch943x_set_baud(port, baud);
	uart_update_timeout(port, termios->c_cflag, baud);
}

int ch943x_startup(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	u8 val;

	dev_dbg(s->dev, "%s\n", __func__);

	/* kmalloc spi transfer buffer when startup */
	one->rxbuf = kmalloc(2048 * 2, GFP_KERNEL);
	if (!one->rxbuf) {
		dev_err(s->dev, "kmalloc rxbuf failed\n");
		return -1;
	}

	one->txbuf = kmalloc(2048 * 2, GFP_KERNEL);
	if (!one->txbuf) {
		dev_err(s->dev, "kmalloc txbuf failed\n");
		return -1;
	}

	/* Reset FIFOs and configure RX-FIFO levels to 8 */
	ch943x_port_write(port, CH943X_FCR_REG,
			  CH943X_FCR_RXRESET_BIT | CH943X_FCR_TXRESET_BIT |
				  CH943X_FCR_RXLVLH_BIT |
				  CH943X_FCR_FIFO_BIT);
	udelay(2000);

	/* Initialize the UART */
	ch943x_port_write(port, CH943X_LCR_REG, CH943X_LCR_WORD_LEN_8);

	/* Enable RX, TX, CTS change interrupts */
	val = CH943X_IER_RDI_BIT | CH943X_IER_RLSI_BIT |
	      CH943X_IER_MSI_BIT;
	ch943x_port_write(port, CH943X_IER_REG, val);

	/* Enable Uart interrupts */
	ch943x_port_write(port, CH943X_MCR_REG, CH943X_MCR_OUT2);
	one->mcr_force = CH943X_MCR_OUT2;

	atomic_set(&s->p[port->line].isopen, 1);

	return 0;
}

void ch943x_shutdown(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	dev_dbg(s->dev, "%s\n", __func__);

	/* Disable all interrupts */
	ch943x_port_write(port, CH943X_IER_REG, 0);
	ch943x_port_write(port, CH943X_MCR_REG, 0);

	if (one->rxbuf)
		kfree(one->rxbuf);
	if (one->txbuf)
		kfree(one->txbuf);
	one->mcr_force = 0;

	atomic_set(&s->p[port->line].isopen, 0);
}

const char *ch943x_type(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	return (port->type == PORT_SC16IS7XX) ? s->devtype->name : NULL;
}

int ch943x_request_port(struct uart_port *port)
{
	/* Do nothing */
	return 0;
}

void ch943x_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_SC16IS7XX;
}

int ch943x_verify_port(struct uart_port *port, struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_SC16IS7XX))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

void ch943x_pm(struct uart_port *port, unsigned int state,
	       unsigned int oldstate)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);

	dev_dbg(s->dev, "%s\n", __func__);
}

void ch943x_null_void(struct uart_port *port)
{
	/* Do nothing */
}

void ch943x_enable_ms(struct uart_port *port)
{
	/* Do nothing */
}

const struct uart_ops ch943x_ops = {
	.tx_empty = ch943x_tx_empty,
	.set_mctrl = ch943x_set_mctrl,
	.get_mctrl = ch943x_get_mctrl,
	.stop_tx = ch943x_stop_tx,
	.start_tx = ch943x_start_tx,
	.stop_rx = ch943x_stop_rx,
	.break_ctl = ch943x_break_ctl,
	.startup = ch943x_startup,
	.shutdown = ch943x_shutdown,
	.set_termios = ch943x_set_termios,
	.type = ch943x_type,
	.request_port = ch943x_request_port,
	.release_port = ch943x_null_void,
	.config_port = ch943x_config_port,
	.verify_port = ch943x_verify_port,
	.enable_ms = ch943x_enable_ms,
	.pm = ch943x_pm,
};

int ch943x_register_uart_driver(struct ch943x_port *s)
{
	int ret;

	s->uart.owner = THIS_MODULE;
	s->uart.dev_name = "ttyWCH";
	s->uart.nr = s->devtype->nr_uart;
	ret = uart_register_driver(&s->uart);
	if (ret) {
		dev_err(s->dev, "Registering UART driver failed\n");
		return -1;
	}

	return 0;
}

int ch943x_register_uart_port(struct ch943x_port *s)
{
	int i;
	u8 clkdiv = 13;
	unsigned long freq;

	dev_dbg(s->dev, "%s\n", __func__);

	freq = 32 * 1000000 * 15 / clkdiv;

	for (i = 0; i < s->devtype->nr_uart; i++) {
		/* Initialize port data */
		s->p[i].port.line = i;
		s->p[i].port.dev = s->dev;
		s->p[i].port.irq = s->irq;
		s->p[i].port.type = PORT_SC16IS7XX;
		s->p[i].port.fifosize = CH943X_FIFO_SIZE;
		s->p[i].port.flags = UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		s->p[i].port.iotype = UPIO_PORT;
		s->p[i].port.ops = &ch943x_ops;

		atomic_set(&s->p[i].isopen, 0);
		/* Disable all interrupts */
		ch943x_port_write(&s->p[i].port, CH943X_IER_REG, 0);

		/* Disable uart interrupts */
		ch943x_port_write(&s->p[i].port, CH943X_MCR_REG, 0);

		s->p[i].msr_reg =
			ch943x_port_read(&s->p[i].port, CH943X_MSR_REG);

		/* Initialize queue for start TX */
		INIT_WORK(&s->p[i].tx_work, ch943x_wq_proc);
		/* Initialize queue for changing mode */
		INIT_WORK(&s->p[i].md_work, ch943x_md_proc);

		INIT_WORK(&s->p[i].stop_rx_work, ch943x_stop_rx_work_proc);
		INIT_WORK(&s->p[i].stop_tx_work, ch943x_stop_tx_work_proc);

		/* Register port */
		uart_add_one_port(&s->uart, &s->p[i].port);
	}

	for (i = 0; i < s->devtype->nr_uart; i++) {
		if (s->chiptype == CHIP_CH9434D)
			s->p[i].port.uartclk = 96000000;
		else
			s->p[i].port.uartclk = 32 * 1000000 * 15 / clkdiv;
	}

	if ((s->chiptype == CHIP_CH9434A) ||
	    (s->chiptype == CHIP_CH9434M)) {
#if defined(CH943X_TNOW0_ON)
		s->p[0].port.rs485.flags |= SER_RS485_ENABLED;
		s->reg485 |= BIT(0);
		ch943x_reg_write(s, CH943X_RS485_REG, s->reg485);
#endif
#if defined(CH943X_TNOW1_ON)
		s->p[1].port.rs485.flags |= SER_RS485_ENABLED;
		s->reg485 |= BIT(1);
		ch943x_reg_write(s, CH943X_RS485_REG, s->reg485);
#endif
#if defined(CH943X_TNOW2_ON)
		s->p[2].port.rs485.flags |= SER_RS485_ENABLED;
		s->reg485 |= BIT(2);
		ch943x_reg_write(s, CH943X_RS485_REG, s->reg485);
#endif
#if defined(CH943X_TNOW3_ON)
		s->p[3].port.rs485.flags |= SER_RS485_ENABLED;
		s->reg485 |= BIT(3);
		ch943x_reg_write(s, CH943X_RS485_REG, s->reg485);
#endif
	}

	return 0;
}

void ch943x_uart_remove(struct ch943x_port *s)
{
	int i;

	for (i = 0; i < s->uart.nr; i++) {
		cancel_work_sync(&s->p[i].tx_work);
		cancel_work_sync(&s->p[i].md_work);
		cancel_work_sync(&s->p[i].stop_rx_work);
		cancel_work_sync(&s->p[i].stop_tx_work);
		uart_remove_one_port(&s->uart, &s->p[i].port);
	}
	uart_unregister_driver(&s->uart);
}
