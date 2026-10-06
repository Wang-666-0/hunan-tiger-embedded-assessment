# VOFA 示例：模块阅读与验收

## 从哪里开始

`Core/Src/main.c` 保留 HAL、时钟和外设初始化，主循环只调用 `VofaDemo_RunCycle()`。

| 文件 / 函数 | 职责与参数 |
| --- | --- |
| vofa_demo.c / VofaDemo_RunCycle(void) | 每轮发送两帧，两次发送后分别延时 500 ms |
| uart_dma_observer.c / HAL_UARTEx_RxEventCallback(huart, Size) | USART1 的 DMA 接收事件；huart 指定 UART，Size 是本次接收长度 |
| uart_dma_observer.c / HAL_UART_TxCpltCallback(huart) | USART1 的中断或 DMA 发送完成时设置 tx_done |

## 要看懂的部分

- `channels:10,90\r\n` 和 `channels:90,10\r\n` 是两通道 FireWater 数值帧。
- `sizeof(vofa_data_1)-1` 排除字符串末尾的 `\0`；发送长度只包含实际帧内容。
- 当前示例使用 `HAL_UART_Transmit` 阻塞发送。DMA 回调和原有 Watch 状态集中在 uart_dma_observer 模块，但主循环没有启动 DMA 接收，不能用 rx_ready 验收当前曲线发送。

## 验收

烧录后在 VOFA+ 选 FireWater，按原来的串口参数连接。两通道应每约 500 ms 在 10/90 与 90/10 之间切换。修改演示数值和节奏，先看 vofa_demo.c。
