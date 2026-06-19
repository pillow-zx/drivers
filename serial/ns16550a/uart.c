// SPDX-License-Identifier: GPL-2.0
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kfifo.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/property.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/*
 * serial_uart16550: serial@1000000 {
 *	compatible = "demo,uart16550";
 *	reg = <0x1000000 0x100>;
 *	interrupts = <GIC_SPI 42 IRQ_TYPE_LEVEL_HIGH>;
 *	clock-frequency = <1843200>;
 *	current-speed = <115200>;
 *	reg-shift = <0>;
 *	reg-io-width = <16>;
 *	status = "okey";
 * };
 */

#define UART16550_DRV_NAME		"platform-16550"
#define UART16550_FIFO_SIZE		4096
#define UART16550_TX_BATCH_MAX		64
#define UART16550_USER_CHUNK		256
#define UART16550_DEFAULT_BAUD		115200
#define UART16550_DEFAULT_UARTCLK	1843200
#define UART16550_DEFAULT_FIFO_DEPTH	16
#define UART16550_IRQ_LOOP_LIMIT	256

#define UART16550_PARITY_NONE		0
#define UART16550_PARITY_ODD		1
#define UART16550_PARITY_EVEN		2

struct uart16550_config {
	__u32 baud;
	__u8 data_bits;
	__u8 stop_bits;
	__u8 parity;
	__u8 loopback;
};

struct uart16550_status {
	__u32 rx_pending;
	__u32 tx_pending;
	__u32 rx_overrun;
	__u32 lsr_errors;
};

#define UART16550_IOC_MAGIC		'u'
#define UART16550_IOC_GET_CONFIG	_IOR(UART16550_IOC_MAGIC, 0x00, struct uart16550_config)
#define UART16550_IOC_SET_CONFIG	_IOW(UART16550_IOC_MAGIC, 0x01, struct uart16550_config)
#define UART16550_IOC_FLUSH		_IO(UART16550_IOC_MAGIC, 0x02)
#define UART16550_IOC_GET_STATUS	_IOR(UART16550_IOC_MAGIC, 0x03, struct uart16550_status)

struct uart16550_port {
	struct device *dev;
	void __iomem *membase;
	struct clk *clk;
	int irq;
	int line;

	u32 uartclk;
	u32 tx_fifo_depth;
	u32 reg_shift;
	u32 reg_io_width;

	spinlock_t lock;
	struct mutex config_lock;
	wait_queue_head_t rx_wait;
	wait_queue_head_t tx_wait;
	DECLARE_KFIFO(rx_fifo, u8, UART16550_FIFO_SIZE);
	DECLARE_KFIFO(tx_fifo, u8, UART16550_FIFO_SIZE);

	struct uart16550_config config;
	u8 ier;
	bool removed;
	u32 rx_overrun;
	u32 lsr_errors;

	struct miscdevice miscdev;
};

static DEFINE_IDA(uart16550_ida);

static void uart16550_ida_free(void *data)
{
	struct uart16550_port *port = data;

	ida_free(&uart16550_ida, port->line);
}

static void __iomem *uart16550_reg_addr(struct uart16550_port *port,
					unsigned int reg)
{
	return port->membase + (reg << port->reg_shift);
}

static u8 uart16550_read(struct uart16550_port *port, unsigned int reg)
{
	void __iomem *addr = uart16550_reg_addr(port, reg);

	switch (port->reg_io_width) {
	case 1:
		return ioread8(addr);
	case 2:
		return ioread16(addr) & 0xff;
	case 4:
		return ioread32(addr) & 0xff;
	default:
		return 0;
	}
}

static void uart16550_write(struct uart16550_port *port, unsigned int reg, u8 val)
{
	void __iomem *addr = uart16550_reg_addr(port, reg);

	switch (port->reg_io_width) {
	case 1:
		iowrite8(val, addr);
		break;
	case 2:
		iowrite16(val, addr);
		break;
	case 4:
		iowrite32(val, addr);
		break;
	}
}

static bool uart16550_removed(struct uart16550_port *port)
{
	return READ_ONCE(port->removed);
}

static bool uart16550_rx_available(struct uart16550_port *port)
{
	unsigned long flags;
	bool available;

	spin_lock_irqsave(&port->lock, flags);
	available = !kfifo_is_empty(&port->rx_fifo);
	spin_unlock_irqrestore(&port->lock, flags);

	return available;
}

static bool uart16550_tx_has_space(struct uart16550_port *port)
{
	unsigned long flags;
	bool available;

	spin_lock_irqsave(&port->lock, flags);
	available = kfifo_avail(&port->tx_fifo) > 0;
	spin_unlock_irqrestore(&port->lock, flags);

	return available;
}

static int uart16550_config_to_lcr(const struct uart16550_config *config, u8 *lcr)
{
	u8 val;

	switch (config->data_bits) {
	case 5:
		val = UART_LCR_WLEN5;
		break;
	case 6:
		val = UART_LCR_WLEN6;
		break;
	case 7:
		val = UART_LCR_WLEN7;
		break;
	case 8:
		val = UART_LCR_WLEN8;
		break;
	default:
		return -EINVAL;
	}

	switch (config->stop_bits) {
	case 1:
		break;
	case 2:
		val |= UART_LCR_STOP;
		break;
	default:
		return -EINVAL;
	}

	switch (config->parity) {
	case UART16550_PARITY_NONE:
		break;
	case UART16550_PARITY_ODD:
		val |= UART_LCR_PARITY;
		break;
	case UART16550_PARITY_EVEN:
		val |= UART_LCR_PARITY | UART_LCR_EPAR;
		break;
	default:
		return -EINVAL;
	}

	*lcr = val;
	return 0;
}

static int uart16550_apply_config_locked(struct uart16550_port *port,
					 const struct uart16550_config *config)
{
	unsigned int divisor;
	u64 denominator;
	u8 lcr;
	u8 mcr;
	int ret;

	if (!config->baud || !port->uartclk)
		return -EINVAL;

	ret = uart16550_config_to_lcr(config, &lcr);
	if (ret)
		return ret;

	denominator = 16ULL * config->baud;
	divisor = DIV_ROUND_CLOSEST_ULL(port->uartclk, denominator);
	if (!divisor || divisor > UART_DIV_MAX)
		return -EINVAL;

	uart16550_write(port, UART_LCR, lcr | UART_LCR_DLAB);
	uart16550_write(port, UART_DLL, divisor & 0xff);
	uart16550_write(port, UART_DLM, (divisor >> 8) & 0xff);
	uart16550_write(port, UART_LCR, lcr);

	mcr = UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2;
	if (config->loopback)
		mcr |= UART_MCR_LOOP;
	uart16550_write(port, UART_MCR, mcr);

	port->config = *config;
	return 0;
}

static int uart16550_apply_config(struct uart16550_port *port,
				  const struct uart16550_config *config)
{
	unsigned long flags;
	int ret;

	mutex_lock(&port->config_lock);
	spin_lock_irqsave(&port->lock, flags);
	ret = uart16550_apply_config_locked(port, config);
	spin_unlock_irqrestore(&port->lock, flags);
	mutex_unlock(&port->config_lock);

	return ret;
}

static void uart16550_update_ier_locked(struct uart16550_port *port, u8 clear, u8 set)
{
	port->ier &= ~clear;
	port->ier |= set;
	uart16550_write(port, UART_IER, port->ier);
}

static unsigned int uart16550_rx_chars_locked(struct uart16550_port *port)
{
	unsigned int queued = 0;
	unsigned int drained = 0;
	u8 lsr;

	do {
		lsr = uart16550_read(port, UART_LSR);
		if (lsr & UART_LSR_BRK_ERROR_BITS)
			port->lsr_errors++;
		if (!(lsr & UART_LSR_DR))
			break;

		if (kfifo_avail(&port->rx_fifo)) {
			u8 ch = uart16550_read(port, UART_RX);

			kfifo_in(&port->rx_fifo, &ch, 1);
			queued++;
		} else {
			uart16550_read(port, UART_RX);
			port->rx_overrun++;
		}
		drained++;
	} while (drained < UART16550_USER_CHUNK);

	return queued;
}

static unsigned int uart16550_tx_chars_locked(struct uart16550_port *port)
{
	unsigned int sent = 0;
	unsigned int limit = min_t(u32, port->tx_fifo_depth, UART16550_TX_BATCH_MAX);
	u8 ch;

	if (!(uart16550_read(port, UART_LSR) & UART_LSR_THRE)) {
		if (!kfifo_is_empty(&port->tx_fifo))
			uart16550_update_ier_locked(port, 0, UART_IER_THRI);
		return 0;
	}

	while (sent < limit && !kfifo_is_empty(&port->tx_fifo)) {
		if (kfifo_out(&port->tx_fifo, &ch, 1) != 1)
			break;
		uart16550_write(port, UART_TX, ch);
		sent++;
	}

	if (kfifo_is_empty(&port->tx_fifo))
		uart16550_update_ier_locked(port, UART_IER_THRI, 0);
	else
		uart16550_update_ier_locked(port, 0, UART_IER_THRI);

	return sent;
}

static void uart16550_kick_tx(struct uart16550_port *port)
{
	unsigned long flags;
	unsigned int sent;

	spin_lock_irqsave(&port->lock, flags);
	sent = uart16550_tx_chars_locked(port);
	spin_unlock_irqrestore(&port->lock, flags);

	if (sent)
		wake_up_interruptible(&port->tx_wait);
}

static irqreturn_t uart16550_irq(int irq, void *dev_id)
{
	struct uart16550_port *port = dev_id;
	bool rx_wake = false;
	bool tx_wake = false;
	bool handled = false;
	unsigned int loops = 0;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	while (loops++ < UART16550_IRQ_LOOP_LIMIT) {
		u8 iir = uart16550_read(port, UART_IIR);
		u8 id = iir & UART_IIR_ID;

		if (iir & UART_IIR_NO_INT)
			break;

		handled = true;

		switch (id) {
		case UART_IIR_RLSI:
		case UART_IIR_RDI:
		case UART_IIR_RX_TIMEOUT:
			if (uart16550_rx_chars_locked(port))
				rx_wake = true;
			break;
		case UART_IIR_THRI:
			if (uart16550_tx_chars_locked(port))
				tx_wake = true;
			break;
		case UART_IIR_MSI:
			uart16550_read(port, UART_MSR);
			break;
		default:
			if (uart16550_rx_chars_locked(port))
				rx_wake = true;
			if (uart16550_tx_chars_locked(port))
				tx_wake = true;
			break;
		}
	}

	spin_unlock_irqrestore(&port->lock, flags);

	if (rx_wake)
		wake_up_interruptible(&port->rx_wait);
	if (tx_wake)
		wake_up_interruptible(&port->tx_wait);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int uart16550_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct uart16550_port *port = container_of(misc, struct uart16550_port, miscdev);

	if (uart16550_removed(port))
		return -ENODEV;

	file->private_data = port;
	return 0;
}

static ssize_t uart16550_read_file(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct uart16550_port *port = file->private_data;
	u8 tmp[UART16550_USER_CHUNK];
	size_t copied = 0;
	int ret;

	if (!count)
		return 0;

	while (!uart16550_rx_available(port)) {
		if (uart16550_removed(port))
			return -ENODEV;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(port->rx_wait,
					       uart16550_rx_available(port) ||
					       uart16550_removed(port));
		if (ret)
			return ret;
	}

	while (copied < count) {
		unsigned long flags;
		unsigned int n;

		spin_lock_irqsave(&port->lock, flags);
		n = min_t(size_t, count - copied, sizeof(tmp));
		n = kfifo_out(&port->rx_fifo, tmp, n);
		spin_unlock_irqrestore(&port->lock, flags);

		if (!n)
			break;
		if (copy_to_user(buf + copied, tmp, n))
			return copied ? copied : -EFAULT;

		copied += n;
	}

	return copied;
}

static ssize_t uart16550_write_file(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct uart16550_port *port = file->private_data;
	u8 tmp[UART16550_USER_CHUNK];
	size_t copied = 0;
	int ret;

	if (!count)
		return 0;

	while (copied < count) {
		unsigned long flags;
		unsigned int space;
		unsigned int n;

		if (uart16550_removed(port))
			return copied ? copied : -ENODEV;

		while (!uart16550_tx_has_space(port)) {
			if (uart16550_removed(port))
				return copied ? copied : -ENODEV;
			if ((file->f_flags & O_NONBLOCK) || copied)
				return copied ? copied : -EAGAIN;

			ret = wait_event_interruptible(port->tx_wait,
						       uart16550_tx_has_space(port) ||
						       uart16550_removed(port));
			if (ret)
				return ret;
		}

		spin_lock_irqsave(&port->lock, flags);
		space = kfifo_avail(&port->tx_fifo);
		spin_unlock_irqrestore(&port->lock, flags);

		n = min_t(size_t, count - copied, sizeof(tmp));
		n = min_t(unsigned int, n, space);
		if (copy_from_user(tmp, buf + copied, n))
			return copied ? copied : -EFAULT;

		spin_lock_irqsave(&port->lock, flags);
		n = kfifo_in(&port->tx_fifo, tmp, n);
		spin_unlock_irqrestore(&port->lock, flags);

		copied += n;
		uart16550_kick_tx(port);
	}

	return copied;
}

static __poll_t uart16550_poll(struct file *file, poll_table *wait)
{
	struct uart16550_port *port = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &port->rx_wait, wait);
	poll_wait(file, &port->tx_wait, wait);

	if (uart16550_removed(port))
		return EPOLLERR | EPOLLHUP;
	if (uart16550_rx_available(port))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (uart16550_tx_has_space(port))
		mask |= EPOLLOUT | EPOLLWRNORM;

	return mask;
}

static void uart16550_flush(struct uart16550_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset(&port->rx_fifo);
	kfifo_reset(&port->tx_fifo);
	uart16550_write(port, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT |
			UART_FCR_TRIGGER_14);
	uart16550_update_ier_locked(port, UART_IER_THRI, 0);
	spin_unlock_irqrestore(&port->lock, flags);

	wake_up_interruptible(&port->tx_wait);
}

static long uart16550_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct uart16550_port *port = file->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case UART16550_IOC_GET_CONFIG:
	{
		struct uart16550_config config;

		mutex_lock(&port->config_lock);
		config = port->config;
		mutex_unlock(&port->config_lock);

		if (copy_to_user(argp, &config, sizeof(config)))
			return -EFAULT;
		return 0;
	}
	case UART16550_IOC_SET_CONFIG:
	{
		struct uart16550_config config;

		if (copy_from_user(&config, argp, sizeof(config)))
			return -EFAULT;
		if (uart16550_removed(port))
			return -ENODEV;

		return uart16550_apply_config(port, &config);
	}
	case UART16550_IOC_FLUSH:
		if (uart16550_removed(port))
			return -ENODEV;
		uart16550_flush(port);
		return 0;
	case UART16550_IOC_GET_STATUS:
	{
		struct uart16550_status status;
		unsigned long flags;

		spin_lock_irqsave(&port->lock, flags);
		status.rx_pending = kfifo_len(&port->rx_fifo);
		status.tx_pending = kfifo_len(&port->tx_fifo);
		status.rx_overrun = port->rx_overrun;
		status.lsr_errors = port->lsr_errors;
		spin_unlock_irqrestore(&port->lock, flags);

		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations uart16550_fops = {
	.owner = THIS_MODULE,
	.open = uart16550_open,
	.read = uart16550_read_file,
	.write = uart16550_write_file,
	.poll = uart16550_poll,
	.unlocked_ioctl = uart16550_ioctl,
	.llseek = noop_llseek,
};

static int uart16550_parse_properties(struct platform_device *pdev,
				      struct uart16550_port *port)
{
	struct device *dev = &pdev->dev;
	u32 baud = UART16550_DEFAULT_BAUD;
	u32 uartclk = 0;
	int ret;

	port->reg_shift = 0;
	port->reg_io_width = 1;
	port->tx_fifo_depth = UART16550_DEFAULT_FIFO_DEPTH;

	device_property_read_u32(dev, "reg-shift", &port->reg_shift);
	device_property_read_u32(dev, "reg-io-width", &port->reg_io_width);
	device_property_read_u32(dev, "fifo-size", &port->tx_fifo_depth);
	device_property_read_u32(dev, "current-speed", &baud);

	if (port->reg_io_width != 1 && port->reg_io_width != 2 &&
	    port->reg_io_width != 4)
		return dev_err_probe(dev, -EINVAL, "unsupported reg-io-width %u\n",
				     port->reg_io_width);

	if (!port->tx_fifo_depth)
		port->tx_fifo_depth = UART16550_DEFAULT_FIFO_DEPTH;

	port->clk = devm_clk_get_optional_enabled(dev, "baudclk");
	if (IS_ERR(port->clk))
		return dev_err_probe(dev, PTR_ERR(port->clk),
				     "failed to enable baud clock\n");

	if (!port->clk) {
		port->clk = devm_clk_get_optional_enabled(dev, NULL);
		if (IS_ERR(port->clk))
			return dev_err_probe(dev, PTR_ERR(port->clk),
					     "failed to enable clock\n");
	}

	if (port->clk)
		uartclk = clk_get_rate(port->clk);
	if (!uartclk)
		device_property_read_u32(dev, "clock-frequency", &uartclk);
	if (!uartclk)
		uartclk = UART16550_DEFAULT_UARTCLK;

	port->uartclk = uartclk;
	port->config.baud = baud;
	port->config.data_bits = 8;
	port->config.stop_bits = 1;
	port->config.parity = UART16550_PARITY_NONE;
	port->config.loopback = 0;

	ret = uart16550_config_to_lcr(&port->config, &(u8){ 0 });
	if (ret)
		return ret;

	return 0;
}

static void uart16550_hw_shutdown(struct uart16550_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	WRITE_ONCE(port->removed, true);
	port->ier = 0;
	uart16550_write(port, UART_IER, 0);
	uart16550_write(port, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	spin_unlock_irqrestore(&port->lock, flags);

	wake_up_interruptible(&port->rx_wait);
	wake_up_interruptible(&port->tx_wait);
}

static int uart16550_hw_init(struct uart16550_port *port)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&port->lock, flags);
	port->ier = 0;
	uart16550_write(port, UART_IER, 0);
	uart16550_write(port, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT |
			UART_FCR_TRIGGER_14);
	(void)uart16550_read(port, UART_LSR);
	(void)uart16550_read(port, UART_RX);
	(void)uart16550_read(port, UART_IIR);
	(void)uart16550_read(port, UART_MSR);
	ret = uart16550_apply_config_locked(port, &port->config);
	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

static void uart16550_hw_enable_irq(struct uart16550_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	uart16550_update_ier_locked(port, 0, UART_IER_RDI | UART_IER_RLSI);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int uart16550_probe(struct platform_device *pdev)
{
	struct uart16550_port *port;
	struct resource *res;
	char *name;
	int ret;

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->dev = &pdev->dev;
	spin_lock_init(&port->lock);
	mutex_init(&port->config_lock);
	init_waitqueue_head(&port->rx_wait);
	init_waitqueue_head(&port->tx_wait);
	INIT_KFIFO(port->rx_fifo);
	INIT_KFIFO(port->tx_fifo);

	port->line = ida_alloc(&uart16550_ida, GFP_KERNEL);
	if (port->line < 0)
		return port->line;

	ret = devm_add_action_or_reset(&pdev->dev, uart16550_ida_free, port);
	if (ret)
		return ret;

	port->membase = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	ret = uart16550_parse_properties(pdev, port);
	if (ret)
		return ret;

	ret = uart16550_hw_init(port);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to initialize UART\n");

	ret = devm_request_irq(&pdev->dev, port->irq, uart16550_irq, 0,
			       dev_name(&pdev->dev), port);
	if (ret) {
		uart16550_hw_shutdown(port);
		return dev_err_probe(&pdev->dev, ret, "failed to request IRQ\n");
	}
	uart16550_hw_enable_irq(port);

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "ttyS16550p%d", port->line);
	if (!name) {
		uart16550_hw_shutdown(port);
		return -ENOMEM;
	}

	port->miscdev.minor = MISC_DYNAMIC_MINOR;
	port->miscdev.name = name;
	port->miscdev.fops = &uart16550_fops;
	port->miscdev.parent = &pdev->dev;
	port->miscdev.mode = 0600;

	ret = misc_register(&port->miscdev);
	if (ret) {
		uart16550_hw_shutdown(port);
		return dev_err_probe(&pdev->dev, ret, "failed to register misc device\n");
	}

	platform_set_drvdata(pdev, port);

	dev_info(&pdev->dev,
		 "registered /dev/%s: mem %pa irq %d uartclk %u baud %u reg-shift %u width %u\n",
		 port->miscdev.name, &res->start, port->irq, port->uartclk,
		 port->config.baud, port->reg_shift, port->reg_io_width);

	return 0;
}

static void uart16550_remove(struct platform_device *pdev)
{
	struct uart16550_port *port = platform_get_drvdata(pdev);

	misc_deregister(&port->miscdev);
	uart16550_hw_shutdown(port);
}

static const struct of_device_id uart16550_of_match[] = {
	{ .compatible = "demo,uart16550" },
	{ .compatible = "ns16550a" },
	{ .compatible = "ns16550" },
	{ .compatible = "uart16550" },
	{ }
};
MODULE_DEVICE_TABLE(of, uart16550_of_match);

static const struct platform_device_id uart16550_id_table[] = {
	{ "platform-16550", 0 },
	{ "ns16550a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(platform, uart16550_id_table);

static struct platform_driver uart16550_driver = {
	.probe = uart16550_probe,
	.remove_new = uart16550_remove,
	.driver = {
		.name = UART16550_DRV_NAME,
		.of_match_table = uart16550_of_match,
	},
	.id_table = uart16550_id_table,
};

module_platform_driver(uart16550_driver);

MODULE_AUTHOR("pillow");
MODULE_DESCRIPTION("Interrupt-driven platform 16550 UART misc driver");
MODULE_LICENSE("GPL");
