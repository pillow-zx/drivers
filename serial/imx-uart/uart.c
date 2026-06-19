// SPDX-License-Identifier: GPL-2.0
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/property.h>
#include <linux/rational.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/*
 * Original DTS notes:
 *
 * #define IMX6UL_CLK_UART1_IPG          189
 * #define IMX6UL_CLK_UART1_SERIAL       190
 *
 * uart1 : serial @2020000 {
 *         compatible = "fsl,imx6ul-uart", "fsl,imx6q-uart";
 *         reg = <0x02020000 0x4000>;
 *         interrupts = <GIC_SPI 26 IRQ_TYPE_LEVEL_HIGH>;
 *         clocks = <&clks IMX6UL_CLK_UART1_IPG>,
 *                  <&clks IMX6UL_CLK_UART1_SERIAL>;
 *         clock-names = "ipg", "per";
 *         status = "disabled";
 * };
 *
 * MX6UL_PAD_UART1_TX_DATA__UART1_DCE_TX 0x0084 0x0310 0x0000 0 0
 * MX6UL_PAD_UART1_TX_DATA__UART1_DTE_RX 0x0084 0x0310 0x0624 0 2
 *
 * &iomuxc {
 *         pinctrl_uart1: uart1grp {
 *                 fsl,pins = <
 *                         MX6UL_PAD_UART1_TX_DATA__UART1_DCE_TX 0x1b0b1
 *                         MX6UL_PAD_UART1_RX_DATA__UART1_DCE_RX 0x1b0b1
 *                 >;
 *         };
 * };
 *
 * &uart1 {
 *         pinctrl-names = "default";
 *         pinctrl-0 = <&pinctrl_uart1>;
 *         status = "okay";
 * };
 */

#define IMX_UART_DRV_NAME		"imx6ul-uart-lite"
#define IMX_UART_DEV_NAME		"imxuart"
#define IMX_UART_FIFO_SIZE		4096            // 软件 FIFO 大小
#define IMX_UART_USER_CHUNK		256             // 每次用户读写的最大块
#define IMX_UART_TX_FIFO_DEPTH		32              // 硬件 TX FIFO 深度
#define IMX_UART_RX_LOOP_LIMIT		256             // 中断单次最多处理字节数，防止中断占用过长
#define IMX_UART_DEFAULT_BAUD		115200          // 默认波特率
#define IMX_UART_TXTL_DEFAULT		8
#define IMX_UART_RXTL_DEFAULT		8

/* i.MX UART register offsets. */
// 接收数据，含状态位
#define URXD0				0x00
// 发送数据
#define URTX0				0x40
// 控制寄存器
#define UCR1				0x80
#define UCR2				0x84
#define UCR3				0x88
#define UCR4				0x8c
// FIFO 控制
#define UFCR				0x90
// 状态寄存器
#define USR1				0x94
#define USR2				0x98
// 波特率分频寄存器
#define UBIR				0xa4
#define UBMR				0xa8
#define IMX21_ONEMS			0xb0
// 测试寄存器
#define IMX21_UTS			0xb4

#define URXD_CHARRDY                    BIT(15)  // 字符就绪
#define URXD_ERR                        BIT(14)  // 有错误（需检查低位）
#define URXD_OVRRUN                     BIT(13)  // 溢出
#define URXD_FRMERR                     BIT(12)  // 帧错误
#define URXD_PRERR                      BIT(10)  // 奇偶错误
#define URXD_RX_DATA                    0xff     // 实际数据在低 8 位

#define UCR1_ADEN			BIT(15)
#define UCR1_TRDYEN			BIT(13)
#define UCR1_IDEN			BIT(12)
#define UCR1_RRDYEN			BIT(9)
#define UCR1_RTSDEN			BIT(5)
#define UCR1_SNDBRK			BIT(4)
#define UCR1_UARTEN			BIT(0)

#define UCR2_IRTS			BIT(14)
#define UCR2_PREN			BIT(8)
#define UCR2_PROE			BIT(7)
#define UCR2_STPB			BIT(6)
#define UCR2_WS				BIT(5)
#define UCR2_ATEN			BIT(3)
#define UCR2_TXEN			BIT(2)
#define UCR2_RXEN			BIT(1)
#define UCR2_SRST			BIT(0)

#define UCR3_DSR			BIT(10)
#define UCR3_ADNIMP			BIT(7)
#define IMX21_UCR3_RXDMUXSEL		BIT(2)

#define UCR4_OREN			BIT(1)
#define UCR4_DREN			BIT(0)

#define UFCR_RXTL_MASK			0x3f
#define UFCR_DCEDTE			BIT(6)
#define UFCR_RFDIV			(7 << 7)
#define UFCR_RFDIV_REG(x)		(((x) < 7 ? 6 - (x) : 6) << 7)
#define UFCR_TXTL_SHF			10

#define USR1_PARITYERR			BIT(15)
#define USR1_TRDY			BIT(13)
#define USR1_FRAMERR			BIT(10)
#define USR1_RRDY			BIT(9)
#define USR1_AGTIM			BIT(8)

#define USR2_TXDC			BIT(3)
#define USR2_BRCD			BIT(2)
#define USR2_ORE			BIT(1)
#define USR2_RDR			BIT(0)

#define UTS_LOOP			BIT(12)
#define UTS_TXFULL			BIT(4)

#define IMX_UART_PARITY_NONE		0
#define IMX_UART_PARITY_ODD		1
#define IMX_UART_PARITY_EVEN		2

/**
 * @brief 串口线路参数配置(8N1)
 *
 * @baud:      波特率
 * @data_bits: 数据位数
 * @stop_bits: 停止位
 * @parity:    奇偶校验
 * @loopback:  回环测试
 */
struct imx_uart_config {
	__u32 baud;
	__u8 data_bits;
	__u8 stop_bits;
	__u8 parity;
	__u8 loopback;
};

/**
 * @brief 运行时快照
 *
 * @rx_pending: rx_fifo 当前待读字节数
 * @tx_pending: tx_fifo 当前待发字节数
 * @rx_overrun: 累计接收溢出次数
 * @rx_errors:  累计接收错误次数
 */
struct imx_uart_status {
	__u32 rx_pending;
	__u32 tx_pending;
	__u32 rx_overrun;
	__u32 rx_errors;
};

#define IMX_UART_IOC_MAGIC		'u'

/*
 * IMX_UART_IOC_GET_CONFIG     _IOR：从驱动读配置
 * IMX_UART_IOC_SET_CONFIG     _IOW：向驱动写配置
 * IMX_UART_IOC_FLUSH          _IO：无数据传输
 * IMX_UART_IOC_GET_STATUS     _IOR：读取队列深度和错误计数
 */
#define IMX_UART_IOC_GET_CONFIG \
	_IOR(IMX_UART_IOC_MAGIC, 0x00, struct imx_uart_config)
#define IMX_UART_IOC_SET_CONFIG \
	_IOW(IMX_UART_IOC_MAGIC, 0x01, struct imx_uart_config)
#define IMX_UART_IOC_FLUSH		_IO(IMX_UART_IOC_MAGIC, 0x02)
#define IMX_UART_IOC_GET_STATUS \
	_IOR(IMX_UART_IOC_MAGIC, 0x03, struct imx_uart_status)

/**
 * Per-UART state. This driver exposes a UART as a misc character device
 * instead of registering with serial_core/tty, so it owns the software FIFOs,
 * blocking semantics, and configuration ABI directly.
 */
struct imx_uart_port {
	struct device *dev;
	void __iomem *membase;
	struct clk *clk_ipg;
	struct clk *clk_per;
	int irq;
	int line;
	u32 uartclk;

	spinlock_t lock;             // 用于中断上下文
	struct mutex config_lock;    // 用于配置操作
	wait_queue_head_t rx_wait;
	wait_queue_head_t tx_wait;
	DECLARE_KFIFO(rx_fifo, u8, IMX_UART_FIFO_SIZE);
	DECLARE_KFIFO(tx_fifo, u8, IMX_UART_FIFO_SIZE);

	struct imx_uart_config config;
	bool removed;
	u32 rx_overrun;
	u32 rx_errors;

	struct miscdevice miscdev;
};

static DEFINE_IDA(imx_uart_ida);

/**
 * @brief Release the numeric line id allocated for one UART instance.
 *
 * This function is registered as a devm cleanup action so the IDA slot is
 * returned automatically if probe fails or when the device is removed.
 *
 * @param data Pointer to the struct imx_uart_port that owns the line id.
 */
static void imx_uart_ida_free(void *data)
{
	struct imx_uart_port *port = data;

	ida_free(&imx_uart_ida, port->line);
}

/**
 * @brief Disable the peripheral clocks used by the UART block.
 *
 * The action is paired with imx_uart_enable_clocks() through devm so clock
 * state is unwound on probe failure and device removal.
 *
 * @param data Pointer to the struct imx_uart_port containing prepared clocks.
 */
static void imx_uart_disable_clocks(void *data)
{
	struct imx_uart_port *port = data;

	clk_disable_unprepare(port->clk_ipg);
	clk_disable_unprepare(port->clk_per);
}

/**
 * @brief Read a 32-bit UART register.
 *
 * @param port UART port state containing the mapped register base.
 * @param offset Register offset from the UART MMIO base.
 * @return The 32-bit value currently stored in the register.
 */
static u32 imx_uart_read(struct imx_uart_port *port, u32 offset)
{
	return readl(port->membase + offset);
}

/**
 * @brief Write a 32-bit UART register.
 *
 * @param port UART port state containing the mapped register base.
 * @param val Value to write.
 * @param offset Register offset from the UART MMIO base.
 */
static void imx_uart_write(struct imx_uart_port *port, u32 val, u32 offset)
{
	writel(val, port->membase + offset);
}

/**
 * @brief Test whether the UART instance is being removed or shut down.
 *
 * READ_ONCE pairs with WRITE_ONCE in shutdown so file operations observe the
 * removal flag without relying on compiler-generated repeated loads.
 *
 * @param port UART port state.
 * @return true if the device is no longer usable, false otherwise.
 */
static bool imx_uart_removed(struct imx_uart_port *port)
{
	return READ_ONCE(port->removed);
}

/**
 * @brief Check whether userspace can read at least one queued RX byte.
 *
 * The RX FIFO is shared by interrupt context and file operations, so the check
 * is protected by the port spinlock.
 *
 * @param port UART port state.
 * @return true if rx_fifo is not empty, false if a read would block.
 */
static bool imx_uart_rx_available(struct imx_uart_port *port)
{
	unsigned long flags;
	bool available;

	spin_lock_irqsave(&port->lock, flags);
	available = !kfifo_is_empty(&port->rx_fifo);
	spin_unlock_irqrestore(&port->lock, flags);

	return available;
}

/**
 * @brief Check whether userspace can queue at least one TX byte.
 *
 * The TX FIFO is shared by interrupt context and file operations, so the check
 * is protected by the port spinlock.
 *
 * @param port UART port state.
 * @return true if tx_fifo has free space, false if a write would block.
 */
static bool imx_uart_tx_has_space(struct imx_uart_port *port)
{
	unsigned long flags;
	bool available;

	spin_lock_irqsave(&port->lock, flags);
	available = kfifo_avail(&port->tx_fifo) > 0;
	spin_unlock_irqrestore(&port->lock, flags);

	return available;
}

/**
 * @brief Convert the public UART configuration into the i.MX UCR2 value.
 *
 * This validates line format fields exposed through ioctl and builds the bits
 * that control data width, stop bits, parity, TX/RX enable, software reset
 * state, and ignored RTS flow-control behavior.
 *
 * @param config Userspace-visible UART configuration to apply.
 * @param preserved Previous UCR2 value; only selected runtime bits are kept.
 * @param ucr2 Output location receiving the completed UCR2 value.
 * @return 0 on success, -EINVAL if the requested format is unsupported.
 */
static int imx_uart_config_to_ucr2(const struct imx_uart_config *config,
				   u32 preserved, u32 *ucr2)
{
        /* 保留运行时状态 */
	u32 val = preserved & UCR2_ATEN;

	if (config->data_bits == 8)
		val |= UCR2_WS;
	else if (config->data_bits != 7)
		return -EINVAL;

	switch (config->stop_bits) {
	case 1:
		break;
	case 2:
		val |= UCR2_STPB;
		break;
	default:
		return -EINVAL;
	}

	switch (config->parity) {
	case IMX_UART_PARITY_NONE:
		break;
	case IMX_UART_PARITY_ODD:
		val |= UCR2_PREN | UCR2_PROE;
		break;
	case IMX_UART_PARITY_EVEN:
		val |= UCR2_PREN;
		break;
	default:
		return -EINVAL;
	}

	*ucr2 = val | UCR2_SRST | UCR2_IRTS | UCR2_TXEN | UCR2_RXEN;
	return 0;
}

/**
 * @brief Program the UART baud-rate generator.
 *
 * The i.MX UART uses a reference divider in UFCR and a fractional
 * numerator/denominator pair in UBIR/UBMR. This routine chooses the closest
 * representable fraction for the requested baud rate.
 *
 * @param port UART port state. The caller must hold port->lock.
 * @param baud Requested baud rate in bits per second.
 * @return 0 on success, -EINVAL if the clock or requested baud is invalid.
 */
static int imx_uart_apply_baud_locked(struct imx_uart_port *port, u32 baud)
{

        /*
         * baud = uartclk /  (deiv * 16) * (num / denom)
         */
	unsigned long num, denom;
	u32 ufcr;
	u32 div;

	if (!baud || !port->uartclk || baud > port->uartclk / 16)
		return -EINVAL;

	div = port->uartclk / (baud * 16);
	if (div > 7)
		div = 7;
	if (!div)
		div = 1;

	/*
	 * i.MX UART baud rate is generated by a reference divider plus a
	 * numerator/denominator pair. Pick the closest fraction that fits the
	 * 16-bit UBIR/UBMR fields.
	 */
	rational_best_approximation(16 * div * baud, port->uartclk,
				    1 << 16, 1 << 16, &num, &denom);
	if (!num || !denom)
		return -EINVAL;

	ufcr = imx_uart_read(port, UFCR);
	ufcr = (ufcr & ~UFCR_RFDIV) | UFCR_RFDIV_REG(div);
	imx_uart_write(port, ufcr, UFCR);
	imx_uart_write(port, num - 1, UBIR);
	imx_uart_write(port, denom - 1, UBMR);
	imx_uart_write(port, port->uartclk / div / 1000, IMX21_ONEMS);

	return 0;
}

/**
 * @brief Enable or disable internal UART loopback.
 *
 * Loopback connects the transmitter path back to the receiver path inside the
 * controller, which is useful for local diagnostics without external wiring.
 *
 * @param port UART port state. The caller must hold port->lock.
 * @param on true to enable loopback, false to disable it.
 */
static void imx_uart_apply_loopback_locked(struct imx_uart_port *port, bool on)
{
	u32 uts = imx_uart_read(port, IMX21_UTS);

	if (on)
		uts |= UTS_LOOP;
	else
		uts &= ~UTS_LOOP;

	imx_uart_write(port, uts, IMX21_UTS);
}

/**
 * @brief Apply a complete UART configuration while locks are already held.
 *
 * This helper validates the public configuration, updates the baud-rate
 * registers, writes UCR2 line-format bits, applies loopback state, and caches
 * the accepted configuration for later GET_CONFIG or flush operations.
 *
 * @param port UART port state. The caller must hold port->lock.
 * @param config Configuration to validate and apply.
 * @return 0 on success or a negative errno value on invalid configuration.
 */
static int imx_uart_apply_config_locked(struct imx_uart_port *port,
					const struct imx_uart_config *config)
{
	u32 ucr2;
	int ret;

        /* 先计算 ucr2 */
	ret = imx_uart_config_to_ucr2(config, imx_uart_read(port, UCR2), &ucr2);
	if (ret)
		return ret;

        /* 设置波特率 */
	ret = imx_uart_apply_baud_locked(port, config->baud);
	if (ret)
		return ret;

        /* 最后一起写 UCR2，避免中间状态 */
	imx_uart_write(port, ucr2, UCR2);
	imx_uart_apply_loopback_locked(port, config->loopback);
	port->config = *config;

	return 0;
}

/**
 * @brief Apply a UART configuration from process context.
 *
 * The mutex serializes configuration ioctls, while the spinlock protects
 * registers and FIFO state against the interrupt handler.
 *
 * @param port UART port state.
 * @param config Configuration to validate and apply.
 * @return 0 on success or a negative errno value on invalid configuration.
 */
static int imx_uart_apply_config(struct imx_uart_port *port,
				 const struct imx_uart_config *config)
{
	unsigned long flags;
	int ret;

        /*
         * mutex 与 spinlock 顺序不可修改，否则会死锁
         *
         * mutex 在外层，防止两个进程同时执行 SET_CONFIG
         * spinlock 在内层，保护寄存器访问和 FIFO 状态不被并发修改
         */

	mutex_lock(&port->config_lock);
	spin_lock_irqsave(&port->lock, flags);
	ret = imx_uart_apply_config_locked(port, config);
	spin_unlock_irqrestore(&port->lock, flags);
	mutex_unlock(&port->config_lock);

	return ret;
}

/**
 * @brief Issue a software reset to the UART controller.
 *
 * The controller acknowledges reset completion by setting UCR2_SRST again.
 * The function waits up to roughly 1 ms for that acknowledgement.
 *
 * @param port UART port state.
 * @return 0 when reset completes, -ETIMEDOUT if the controller does not ack.
 */
static int imx_uart_soft_reset(struct imx_uart_port *port)
{
	u32 ucr2 = imx_uart_read(port, UCR2);
	int i;

	/* Hardware sets SRST again once the reset sequence has completed. */
	imx_uart_write(port, ucr2 & ~UCR2_SRST, UCR2);

	for (i = 0; i < 1000; i++) {
		if (imx_uart_read(port, UCR2) & UCR2_SRST)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

/**
 * @brief Configure hardware FIFO trigger levels.
 *
 * The routine preserves the reference divider and DCE/DTE mode bits in UFCR
 * while installing this driver's TX and RX trigger thresholds.
 *
 * @param port UART port state.
 */
static void imx_uart_setup_fifo(struct imx_uart_port *port)
{
	u32 val;

	val = imx_uart_read(port, UFCR) & (UFCR_RFDIV | UFCR_DCEDTE);
	val |= IMX_UART_TXTL_DEFAULT << UFCR_TXTL_SHF;
	val |= IMX_UART_RXTL_DEFAULT & UFCR_RXTL_MASK;
	imx_uart_write(port, val, UFCR);
}

/**
 * @brief Drain received bytes from hardware into the software RX FIFO.
 *
 * This function is called from IRQ context with port->lock held. It records
 * hardware receive errors and software FIFO overrun events, but still queues
 * the received data byte when FIFO space is available.
 *
 * @param port UART port state. The caller must hold port->lock.
 * @return Number of bytes queued into rx_fifo.
 */
static unsigned int imx_uart_rx_chars_locked(struct imx_uart_port *port)
{
	unsigned int queued = 0;
	unsigned int drained = 0;
	u32 rx;

	/* Drain hardware RX FIFO, but cap the loop so one IRQ cannot spin forever. */
	while (drained++ < IMX_UART_RX_LOOP_LIMIT) {
		rx = imx_uart_read(port, URXD0);
		if (!(rx & URXD_CHARRDY))
			break;

		if (rx & URXD_ERR) {
			port->rx_errors++;
			if (rx & URXD_OVRRUN)
				port->rx_overrun++;
		}

		if (kfifo_avail(&port->rx_fifo)) {
			u8 ch = rx & URXD_RX_DATA;

			kfifo_in(&port->rx_fifo, &ch, 1);
			queued++;
		} else {
			port->rx_overrun++;
		}
	}

	return queued;
}

/**
 * @brief Move queued transmit bytes from software FIFO to hardware FIFO.
 *
 * This function is called with port->lock held. It writes as many bytes as the
 * hardware can accept, then enables or disables TX-ready interrupts depending
 * on whether software data remains queued.
 *
 * @param port UART port state. The caller must hold port->lock.
 * @return Number of bytes written to the hardware transmitter.
 */
static unsigned int imx_uart_tx_chars_locked(struct imx_uart_port *port)
{
	unsigned int sent = 0;
	u8 ch;

	/* Push pending software FIFO bytes until the hardware TX FIFO is full. */
	while (sent < IMX_UART_TX_FIFO_DEPTH && !kfifo_is_empty(&port->tx_fifo)) {
		if (imx_uart_read(port, IMX21_UTS) & UTS_TXFULL)
			break;
		if (kfifo_out(&port->tx_fifo, &ch, 1) != 1)
			break;

		imx_uart_write(port, ch, URTX0);
		sent++;
	}

	/* Only request TX-ready interrupts while there is software data left. */
	if (kfifo_is_empty(&port->tx_fifo)) {
		u32 ucr1 = imx_uart_read(port, UCR1);

		imx_uart_write(port, ucr1 & ~UCR1_TRDYEN, UCR1);
	} else {
		u32 ucr1 = imx_uart_read(port, UCR1);

		imx_uart_write(port, ucr1 | UCR1_TRDYEN, UCR1);
	}

	return sent;
}

/**
 * @brief Start or advance transmission after userspace queues data.
 *
 * This tries to write immediately to the hardware TX FIFO so small writes do
 * not have to wait for a later TX-ready interrupt.
 *
 * @param port UART port state.
 */
static void imx_uart_kick_tx(struct imx_uart_port *port)
{
	unsigned long flags;
	unsigned int sent;

	spin_lock_irqsave(&port->lock, flags);
	sent = imx_uart_tx_chars_locked(port);
	spin_unlock_irqrestore(&port->lock, flags);

	if (sent)
		wake_up_interruptible(&port->tx_wait);
}

/**
 * @brief Handle UART receive, transmit, and error interrupts.
 *
 * The handler snapshots status/control registers, filters out disabled sticky
 * status bits, drains RX data, advances TX data, clears handled error/status
 * bits, and wakes readers or writers whose wait conditions changed.
 *
 * @param irq Linux IRQ number delivered by the interrupt core.
 * @param dev_id Pointer to the struct imx_uart_port registered with the IRQ.
 * @return IRQ_HANDLED if this UART generated work, IRQ_NONE otherwise.
 */
static irqreturn_t imx_uart_irq(int irq, void *dev_id)
{
	struct imx_uart_port *port = dev_id;
	unsigned int rx_queued = 0;
	unsigned int tx_sent = 0;
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;
	u32 usr1, usr2, ucr1, ucr2, ucr4;

        /* 进入中断处理首先需要加锁并一次性读取所有相关寄存器
         * 保证后续判断基于同一时刻硬件状态 */
	spin_lock_irqsave(&port->lock, flags);

	usr1 = imx_uart_read(port, USR1);
	usr2 = imx_uart_read(port, USR2);
	ucr1 = imx_uart_read(port, UCR1);
	ucr2 = imx_uart_read(port, UCR2);
	ucr4 = imx_uart_read(port, UCR4);

        /* i.MX UART 使用“粘性”状态位，即使中断源禁用硬件仍会置位状态标记
         * 需要进行手动过滤，这里屏蔽未启动中断源的状态位
         *
         * 控制位（启用中断）	状态位（事件发生）
         * UCR1_RRDYEN	      USR1_RRDY（RX FIFO 达到水位）
         * UCR2_ATEN	      USR1_AGTIM（接收超时）
         * UCR1_TRDYEN	      USR1_TRDY（TX FIFO 低于水位）
         * UCR4_OREN	      USR2_ORE（接收溢出）
         */
	if (!(ucr1 & UCR1_RRDYEN))
		usr1 &= ~USR1_RRDY;
	if (!(ucr2 & UCR2_ATEN))
		usr1 &= ~USR1_AGTIM;
	if (!(ucr1 & UCR1_TRDYEN))
		usr1 &= ~USR1_TRDY;
	if (!(ucr4 & UCR4_OREN))
		usr2 &= ~USR2_ORE;


        /*
         * 处理接收
         *
         * USR1_RRDY 表示积累的字节数达到了水位线
         * USR1_AGTIM 表示 aging timer 超时，当接受字节但不够水位线时，
         * 超时机制在一段时间后触发中断，防止数据长时间滞留在硬件 FIFO
         */
	if (usr1 & (USR1_RRDY | USR1_AGTIM)) {
                /* 清理aging timer 标志*/
		imx_uart_write(port, USR1_AGTIM, USR1);
		rx_queued = imx_uart_rx_chars_locked(port);
		ret = IRQ_HANDLED;
	}

        /*
         * 处理发送和接收错误
         *
         * USR1_TRDY 表示 TX FIFO 已经低于水位线，可以继续填充数据
         */
	if (usr1 & (USR1_TRDY | USR1_PARITYERR | USR1_FRAMERR)) { // i.MX UART 错误与 TRDY 在同一寄存器
		tx_sent = imx_uart_tx_chars_locked(port);
                /* 奇偶错误与帧错误数据接收侧错误，累加到 rx_errors 然后清除 */
		if (usr1 & (USR1_PARITYERR | USR1_FRAMERR)) {
			port->rx_errors++;
			imx_uart_write(port, usr1 & (USR1_PARITYERR | USR1_FRAMERR),
				       USR1);
		}
		ret = IRQ_HANDLED;
	}

        /*
         * 溢出和 break 检测
         *
         * USR2_ORE 接受溢出
         * USR2_BRCD break 检测
         */
	if (usr2 & (USR2_ORE | USR2_BRCD)) {
		if (usr2 & USR2_ORE)
			port->rx_overrun++;
		imx_uart_write(port, usr2 & (USR2_ORE | USR2_BRCD), USR2);
		ret = IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&port->lock, flags);

        /*
         * 锁外唤醒
         *
         * 因为唤醒的进程会立刻尝试获取 lock，
         * 如果在持锁时唤醒，被唤醒的进程抢锁失败，白白自旋一次
         * 先解锁后进程一旦唤醒可以立刻拿到锁，效率更高
         */

	if (rx_queued)
		wake_up_interruptible(&port->rx_wait);
	if (tx_sent)
		wake_up_interruptible(&port->tx_wait);

	return ret;
}

/**
 * @brief Open the misc character device for one UART instance.
 *
 * misc_open() places the miscdevice in file->private_data before this callback.
 * The driver converts it back to imx_uart_port and stores that pointer for all
 * later file operations.
 *
 * @param inode VFS inode for the misc device node.
 * @param file Open file object to initialize.
 * @return 0 on success or -ENODEV if the device has been removed.
 */
static int imx_uart_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct imx_uart_port *port;

	port = container_of(misc, struct imx_uart_port, miscdev);
	if (imx_uart_removed(port))
		return -ENODEV;

	file->private_data = port;
	return 0;
}

/**
 * @brief Read received UART data from the software RX FIFO.
 *
 * Blocking reads sleep until the interrupt handler queues data or the device
 * is removed. Non-blocking reads return -EAGAIN when no data is available.
 *
 * @param file Open file whose private_data points to struct imx_uart_port.
 * @param buf Userspace buffer that receives bytes.
 * @param count Maximum number of bytes requested.
 * @param ppos Unused file position pointer.
 * @return Number of bytes copied, 0 for zero-length reads, or a negative errno.
 */
static ssize_t imx_uart_read_file(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct imx_uart_port *port = file->private_data;
	u8 tmp[IMX_UART_USER_CHUNK];
	size_t copied = 0;
	int ret;

	if (!count)
		return 0;

	while (!imx_uart_rx_available(port)) {

                /* 必须先检查设备是否存在，再检查调用者意愿，最后真正睡眠 */
		if (imx_uart_removed(port))
			return -ENODEV; // 热拔出
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN; // 非阻塞

		ret = wait_event_interruptible(port->rx_wait,
					       imx_uart_rx_available(port) ||
					       imx_uart_removed(port));
		if (ret)
			return ret; // 信号打断
	}

	while (copied < count) {
		unsigned long flags;
		unsigned int n;

                /*
                 * 将持 spinlock 与 copy_to_user 分开
                 *
                 * copy_to_user 访问用户空间可能触发缺页异常，
                 * 缺页处理需要睡眠等待 IO， spinlock 不允许睡眠，
                 * 造成死锁/BUG_ON
                 *
                 * 持锁时制作内核内存操作，锁外做用户内存操作
                 */

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

/**
 * @brief Queue UART transmit data from userspace.
 *
 * Data is copied into tx_fifo in bounded chunks and imx_uart_kick_tx() starts
 * hardware transmission. Blocking writes wait for software FIFO space only
 * before the first byte; partial writes return the number already queued.
 *
 * @param file Open file whose private_data points to struct imx_uart_port.
 * @param buf Userspace buffer containing bytes to transmit.
 * @param count Number of bytes requested.
 * @param ppos Unused file position pointer.
 * @return Number of bytes queued or a negative errno if no byte was queued.
 */
static ssize_t imx_uart_write_file(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct imx_uart_port *port = file->private_data;
	u8 tmp[IMX_UART_USER_CHUNK];
	size_t copied = 0;
	int ret;

	if (!count)
		return 0;

	while (copied < count) {
		unsigned long flags;
		unsigned int space;
		unsigned int n;

		if (imx_uart_removed(port))
			return copied ? copied : -ENODEV;

                /* FIFO 满了就进入睡眠 */
		while (!imx_uart_tx_has_space(port)) {

			if (imx_uart_removed(port))
				return copied ? copied : -ENODEV;

                        /* 即使是阻塞模式，只要写了数据，FIFO 满也不等待，
                         * 直接返回写入字节数 */
			if ((file->f_flags & O_NONBLOCK) || copied)
				return copied ? copied : -EAGAIN;

			ret = wait_event_interruptible(port->tx_wait,
						       imx_uart_tx_has_space(port) ||
						       imx_uart_removed(port));
			if (ret)
				return ret;
		}

                /*
                 * 将持 spinlock 与 copy_to_user 分开
                 *
                 * copy_from_user 访问用户空间可能触发缺页异常，
                 * 缺页处理需要睡眠等待 IO， spinlock 不允许睡眠，
                 * 造成死锁/BUG_ON
                 *
                 * 持锁时制作内核内存操作，锁外做用户内存操作
                 */

		spin_lock_irqsave(&port->lock, flags);
		space = kfifo_avail(&port->tx_fifo); // 读 FIFO 剩余空间
		spin_unlock_irqrestore(&port->lock, flags);

		n = min_t(size_t, count - copied, sizeof(tmp));
		n = min_t(unsigned int, n, space);
		if (copy_from_user(tmp, buf + copied, n))
			return copied ? copied : -EFAULT;

		spin_lock_irqsave(&port->lock, flags);
		n = kfifo_in(&port->tx_fifo, tmp, n); // 把 tmp 写入 FIFO
		spin_unlock_irqrestore(&port->lock, flags);

		copied += n;
		imx_uart_kick_tx(port);
	}

	return copied;
}

/**
 * @brief Report readiness for poll/select/epoll.
 *
 * The VFS poll path registers both RX and TX wait queues, then returns readable
 * or writable readiness according to current software FIFO state.
 *
 * @param file Open file whose private_data points to struct imx_uart_port.
 * @param wait Poll table used by the VFS to subscribe to wait queues.
 * @return EPOLL* readiness bits, including HUP/ERR when the device is removed.
 */
static __poll_t imx_uart_poll(struct file *file, poll_table *wait)
{
	struct imx_uart_port *port = file->private_data;
	__poll_t mask = 0;

        /*
         * 必须先注册，后检查，否则会有竟态
         */

	poll_wait(file, &port->rx_wait, wait);
	poll_wait(file, &port->tx_wait, wait);

	if (imx_uart_removed(port))
		return EPOLLERR | EPOLLHUP; // 设备消失直接返回
	if (imx_uart_rx_available(port))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (imx_uart_tx_has_space(port))
		mask |= EPOLLOUT | EPOLLWRNORM;

	return mask;
}

/**
 * @brief Flush software queues and reset the UART hardware.
 *
 * The current accepted UART configuration is preserved, while pending RX/TX
 * data and sticky status bits are discarded. Writers are awakened because TX
 * software FIFO space becomes available after the reset.
 *
 * @param port UART port state.
 */
static void imx_uart_flush(struct imx_uart_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset(&port->rx_fifo);
	kfifo_reset(&port->tx_fifo);
	imx_uart_soft_reset(port);
	imx_uart_setup_fifo(port);
	imx_uart_apply_config_locked(port, &port->config);
	imx_uart_write(port, USR1_AGTIM, USR1);
	imx_uart_write(port, USR2_ORE | USR2_BRCD, USR2);
	imx_uart_write(port, imx_uart_read(port, UCR1) & ~UCR1_TRDYEN, UCR1);
	spin_unlock_irqrestore(&port->lock, flags);

	wake_up_interruptible(&port->tx_wait);
}

/**
 * @brief Handle private ioctl commands for this misc UART device.
 *
 * Supported commands get or set line configuration, flush the port, and return
 * queue/error status. Unknown commands are rejected with -ENOTTY as required by
 * the ioctl convention.
 *
 * @param file Open file whose private_data points to struct imx_uart_port.
 * @param cmd ioctl command number.
 * @param arg Userspace pointer or integer argument encoded by the command.
 * @return 0 on success or a negative errno value on failure.
 */
static long imx_uart_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct imx_uart_port *port = file->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case IMX_UART_IOC_GET_CONFIG:
	{
		struct imx_uart_config config;

		mutex_lock(&port->config_lock);
		config = port->config;
		mutex_unlock(&port->config_lock);

		if (copy_to_user(argp, &config, sizeof(config)))
			return -EFAULT;
		return 0;
	}
	case IMX_UART_IOC_SET_CONFIG:
	{
		struct imx_uart_config config;

		if (copy_from_user(&config, argp, sizeof(config)))
			return -EFAULT;
		if (imx_uart_removed(port))
			return -ENODEV;

		return imx_uart_apply_config(port, &config);
	}
	case IMX_UART_IOC_FLUSH:
		if (imx_uart_removed(port))
			return -ENODEV;
		imx_uart_flush(port);
		return 0;
	case IMX_UART_IOC_GET_STATUS:
	{
		struct imx_uart_status status;
		unsigned long flags;

		spin_lock_irqsave(&port->lock, flags);
		status.rx_pending = kfifo_len(&port->rx_fifo);
		status.tx_pending = kfifo_len(&port->tx_fifo);
		status.rx_overrun = port->rx_overrun;
		status.rx_errors = port->rx_errors;
		spin_unlock_irqrestore(&port->lock, flags);

		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations imx_uart_fops = {
	.owner = THIS_MODULE,
	.open = imx_uart_open,
	.read = imx_uart_read_file,
	.write = imx_uart_write_file,
	.poll = imx_uart_poll,
	.unlocked_ioctl = imx_uart_ioctl,
	.llseek = noop_llseek,
};

/**
 * @brief Initialize UART controller registers for this driver.
 *
 * The routine resets the controller, configures FIFO thresholds, clears pending
 * sticky status, enables the UART block, routes RX data, applies the cached
 * line configuration, and leaves interrupt sources masked until IRQ setup is
 * complete.
 *
 * @param port UART port state with clocks enabled and register base mapped.
 * @return 0 on success or a negative errno value from reset/configuration.
 */
static int imx_uart_hw_init(struct imx_uart_port *port)
{
	u32 ucr1, ucr2, ucr3, ucr4;
	int ret;

	/* Start from a quiet UART, then enable only the features this driver uses. */
	imx_uart_write(port, 0, UCR1);
	ret = imx_uart_soft_reset(port);
	if (ret)
		return ret;

	imx_uart_setup_fifo(port);

	imx_uart_write(port, USR1_AGTIM, USR1);
	imx_uart_write(port, USR2_ORE | USR2_BRCD, USR2);

	ucr1 = imx_uart_read(port, UCR1);
	ucr1 &= ~(UCR1_ADEN | UCR1_TRDYEN | UCR1_IDEN |
		  UCR1_RRDYEN | UCR1_RTSDEN | UCR1_SNDBRK);
	ucr1 |= UCR1_UARTEN;
	imx_uart_write(port, ucr1, UCR1);

	ucr4 = imx_uart_read(port, UCR4);
	ucr4 &= ~(UCR4_DREN | UCR4_OREN);
	imx_uart_write(port, ucr4, UCR4);

	/* Route RX data into the UART block and keep modem-status inputs inactive. */
	ucr3 = IMX21_UCR3_RXDMUXSEL | UCR3_ADNIMP | UCR3_DSR;
	imx_uart_write(port, ucr3, UCR3);

	ret = imx_uart_apply_config_locked(port, &port->config);
	if (ret)
		return ret;

	ucr2 = imx_uart_read(port, UCR2);
	ucr2 &= ~UCR2_ATEN;
	imx_uart_write(port, ucr2, UCR2);

	return 0;
}

/**
 * @brief Unmask the UART interrupt sources used by this driver.
 *
 * RX-ready, aging-timeout, and overrun interrupts are enabled only after the
 * Linux IRQ handler has been registered, so hardware cannot signal before the
 * driver is ready to service it.
 *
 * @param port UART port state.
 */
static void imx_uart_hw_enable_irq(struct imx_uart_port *port)
{
	unsigned long flags;
	u32 ucr1, ucr2, ucr4;

	/* Enable RX-ready, aging-timeout, and overrun interrupts after IRQ setup. */
	spin_lock_irqsave(&port->lock, flags);
	ucr1 = imx_uart_read(port, UCR1);
	ucr1 |= UCR1_RRDYEN;
	imx_uart_write(port, ucr1, UCR1);

	ucr2 = imx_uart_read(port, UCR2);
	ucr2 |= UCR2_ATEN;
	imx_uart_write(port, ucr2, UCR2);

	ucr4 = imx_uart_read(port, UCR4);
	ucr4 |= UCR4_OREN;
	imx_uart_write(port, ucr4, UCR4);
	spin_unlock_irqrestore(&port->lock, flags);
}

/**
 * @brief Stop the UART hardware and wake blocked users.
 *
 * This marks the port removed, disables interrupt sources and TX/RX engines,
 * and wakes wait queues so blocked read/write/poll callers can return.
 *
 * @param port UART port state.
 */
static void imx_uart_hw_shutdown(struct imx_uart_port *port)
{
	unsigned long flags;
	u32 ucr1, ucr2;

	spin_lock_irqsave(&port->lock, flags);
	/* Make blocked file operations return promptly during remove/error paths. */
	WRITE_ONCE(port->removed, true);

	ucr1 = imx_uart_read(port, UCR1);
	ucr1 &= ~(UCR1_ADEN | UCR1_TRDYEN | UCR1_IDEN |
		  UCR1_RRDYEN | UCR1_RTSDEN | UCR1_SNDBRK | UCR1_UARTEN);
	imx_uart_write(port, ucr1, UCR1);

	ucr2 = imx_uart_read(port, UCR2);
	ucr2 &= ~(UCR2_ATEN | UCR2_TXEN | UCR2_RXEN);
	imx_uart_write(port, ucr2, UCR2);
	spin_unlock_irqrestore(&port->lock, flags);

	wake_up_interruptible(&port->rx_wait);
	wake_up_interruptible(&port->tx_wait);
}

/**
 * @brief Acquire, enable, and record the clocks required by the UART.
 *
 * The device tree must provide clocks named "ipg" and "per". The peripheral
 * clock rate is cached as uartclk because baud-rate programming depends on it.
 *
 * @param pdev Platform device created from the UART device tree node.
 * @param port UART port state to receive clock handles and rate.
 * @return 0 on success or a negative errno value reported through dev_err_probe().
 */
static int imx_uart_enable_clocks(struct platform_device *pdev,
				  struct imx_uart_port *port)
{
	int ret;

	port->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(port->clk_ipg))
		return dev_err_probe(&pdev->dev, PTR_ERR(port->clk_ipg),
				     "failed to get ipg clock\n");

	port->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(port->clk_per))
		return dev_err_probe(&pdev->dev, PTR_ERR(port->clk_per),
				     "failed to get per clock\n");

	ret = clk_prepare_enable(port->clk_per);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to enable per clock\n");

	ret = clk_prepare_enable(port->clk_ipg);
	if (ret) {
		clk_disable_unprepare(port->clk_per);
		return dev_err_probe(&pdev->dev, ret,
				     "failed to enable ipg clock\n");
	}

	ret = devm_add_action_or_reset(&pdev->dev, imx_uart_disable_clocks, port);
	if (ret)
		return ret;

	port->uartclk = clk_get_rate(port->clk_per);
	if (!port->uartclk)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "per clock has zero rate\n");

	return 0;
}

/**
 * @brief Bind one matched platform device to this UART driver.
 *
 * Probe allocates per-port state, maps MMIO resources, obtains IRQ and clocks,
 * applies the default or device-tree baud rate, initializes hardware, requests
 * the interrupt, and registers the /dev/imxuartN misc device.
 *
 * @param pdev Platform device whose compatible string matched imx_uart_of_match.
 * @return 0 on successful registration or a negative errno value on failure.
 */
static int imx_uart_probe(struct platform_device *pdev)
{
	struct imx_uart_port *port;
	struct resource *res;
	u32 baud = IMX_UART_DEFAULT_BAUD;
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

	port->line = ida_alloc(&imx_uart_ida, GFP_KERNEL);
	if (port->line < 0)
		return port->line;

        /* 托管清理动作，probe 失败自动归还 id */
	ret = devm_add_action_or_reset(&pdev->dev, imx_uart_ida_free, port);
	if (ret)
		return ret;

	port->membase = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	ret = imx_uart_enable_clocks(pdev, port);
	if (ret)
		return ret;

        /* 读取 DTS 中的 current-speed 属性，不存在时报纸 baud 为 115200 */
	device_property_read_u32(&pdev->dev, "current-speed", &baud);
	port->config.baud = baud;

        /* 8N1 */
	port->config.data_bits = 8;
	port->config.stop_bits = 1;
	port->config.parity = IMX_UART_PARITY_NONE;
	port->config.loopback = 0;

        /* 配置硬件 */
	ret = imx_uart_hw_init(port);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to initialize UART\n");

	/* 先注册中断处理函数，再打开硬件中断源 */
	ret = devm_request_irq(&pdev->dev, port->irq, imx_uart_irq, 0,
			       dev_name(&pdev->dev), port);
	if (ret) {
		imx_uart_hw_shutdown(port);
		return dev_err_probe(&pdev->dev, ret, "failed to request IRQ\n");
	}
	imx_uart_hw_enable_irq(port);


        /* 注册 misc 设备 */
	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s%d",
			      IMX_UART_DEV_NAME, port->line);
	if (!name) {
		imx_uart_hw_shutdown(port);
		return -ENOMEM;
	}

	port->miscdev.minor = MISC_DYNAMIC_MINOR; // 内核分配设备号
	port->miscdev.name = name;                // /dev/ 下文件名
	port->miscdev.fops = &imx_uart_fops;      // 操作函数集
	port->miscdev.parent = &pdev->dev;        // 建立 sysfs 父子关系
	port->miscdev.mode = 0600;                // 仅 root 可读写

	ret = misc_register(&port->miscdev);
	if (ret) {
		imx_uart_hw_shutdown(port);
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register misc device\n");
	}

        /* 把 port 挂载到 pdev 上，remove 时取回 */
	platform_set_drvdata(pdev, port);

	dev_info(&pdev->dev,
		 "registered /dev/%s: mem %pa irq %d uartclk %u baud %u\n",
		 port->miscdev.name, &res->start, port->irq, port->uartclk,
		 port->config.baud);

	return 0;
}

/**
 * @brief Unbind a platform device from this UART driver.
 *
 * Removing the misc device first prevents new opens. Hardware shutdown then
 * marks the port removed, disables the controller, and wakes any blocked file
 * operations that still hold an open descriptor.
 *
 * @param pdev Platform device previously initialized by imx_uart_probe().
 */
static void imx_uart_remove(struct platform_device *pdev)
{
	struct imx_uart_port *port = platform_get_drvdata(pdev);

        /* 顺序不可变，先撤销节点，再关硬件 */
	misc_deregister(&port->miscdev);
	imx_uart_hw_shutdown(port);
}

static const struct of_device_id imx_uart_of_match[] = {
	{ .compatible = "fsl,imx6ul-uart" },
	{ .compatible = "fsl,imx6q-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx_uart_of_match);

static const struct platform_device_id imx_uart_id_table[] = {
	{ "imx6ul-uart-lite", 0 },
	{ }
};
MODULE_DEVICE_TABLE(platform, imx_uart_id_table);

static struct platform_driver imx_uart_driver = {
	.probe = imx_uart_probe,
	.remove_new = imx_uart_remove,
	.driver = {
		.name = IMX_UART_DRV_NAME,
		.of_match_table = imx_uart_of_match,
	},
	.id_table = imx_uart_id_table,
};

module_platform_driver(imx_uart_driver);

MODULE_AUTHOR("pillow");
MODULE_DESCRIPTION("Interrupt-driven i.MX6UL UART misc driver");
MODULE_LICENSE("GPL");
