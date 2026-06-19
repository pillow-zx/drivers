# Linux Driver Learning Notes

这个仓库用于记录我在学习 Linux 驱动开发过程中编写的示例代码。

## 主要内容

### `serial/imx-uart`

i.MX6UL UART 的简化 platform 驱动示例。

主要内容：

- 通过设备树匹配 `fsl,imx6ul-uart`、`fsl,imx6q-uart`
- 使用 `devm_platform_get_and_ioremap_resource()` 映射寄存器
- 获取并使能 `ipg`、`per` 两路时钟
- 使用中断驱动收发数据
- 使用 `kfifo` 管理软件 RX/TX 缓冲
- 注册 misc 字符设备，设备名形如 `/dev/imxuart0`
- 支持 `read`、`write`、`poll` 和 `ioctl`
- 支持波特率、数据位、停止位、奇偶校验、回环模式配置

### `serial/ns16550a`

16550/NS16550A 兼容 UART 的 platform 驱动示例。

主要内容：

- 通过设备树匹配 `demo,uart16550`、`ns16550a`、`ns16550`、`uart16550`
- 支持 `reg-shift` 和 `reg-io-width`
- 支持可选时钟和 `clock-frequency`
- 配置 16550 LCR/DLL/DLM/MCR/IER/FCR 等寄存器
- 使用中断和 `kfifo` 实现收发路径
- 注册 misc 字符设备，设备名形如 `/dev/ttyS16550p0`
- 支持 `read`、`write`、`poll` 和 `ioctl`

## 备注

本仓库是个人学习代码集合，代码会随着学习进度持续调整。提交记录和目录内容更适合作为学习轨迹参考，而不是稳定 API 或完整驱动框架。
