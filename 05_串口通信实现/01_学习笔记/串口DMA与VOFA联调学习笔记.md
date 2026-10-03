# 串口 DMA 与 VOFA+ 联调学习笔记

姓名：王世伟　创建日期：2026-10-03

## 1. 任务与实验环境

本次完成串口发送、接收与原样回传，再使用 DMA 和串口空闲检测接收不同长度的数据，最后用逻辑分析仪观察串口波形.

| 项目 | 本次使用 |
| --- | --- |
| 开发板 | STM32F103C8T6 |
| 开发工具 | STM32CubeMX、Keil |
| 串口连接 | USB 转 TTL，CH340 |
| 串口上位机 | VOFA+ 1.3.10，RawData 数据引擎 |
| 逻辑分析仪软件 | Saleae Logic 2 |
| 界面说明 | 软件界面经过本地汉化处理，菜单名称按本地界面记录 |
| 串口参数 | USART1，115200 bit/s，8N1，无流控 |

## 2. 接线与配置

CH340 的 RXD 接 PA9（USART1_TX），TXD 接 PA10（USART1_RX），GND 接开发板 GND. 串口模块需支持 3.3 V 逻辑电平；开发板单独供电时，模块 VCC 不连接.

USART1 选择 Asynchronous，数据方向为 Receive and Transmit. 8N1 表示8位数据、无校验、1位停止；在无校验时，界面的 8 Bits (including Parity) 就是8位有效数据. 系统时钟使用 HSE 8 MHz，经 PLL ×9 得到72 MHz.

USART1 参数配置图：________

| DMA 项目 | 接收 | 发送 |
| --- | --- | --- |
| 请求 | USART1_RX | USART1_TX |
| 通道 | DMA1 Channel 5 | DMA1 Channel 4 |
| 方向 | Peripheral to Memory | Memory to Peripheral |
| 模式 | Normal | Normal |
| 外设地址递增 | Disable | Disable |
| 内存地址递增 | Enable | Enable |
| 两端数据宽度 | Byte | Byte |

外设地址固定，是因为每次访问相同的串口数据寄存器；内存地址递增，是为了把字节依次放入缓冲区或依次取出发送.

DMA 配置图：________

启用 DMA1 Channel 4、DMA1 Channel 5 和 USART1 global interrupt. 截图中的 DMA 项灰色且打勾，表示已启用并由 CubeMX 锁定，不是关闭. USART1 中断用于空闲检测、错误和发送完成等处理；DMA 中断处理搬运完成等事件.

NVIC 中断配置图：________

生成代码后的初始化顺序为：

```c
MX_GPIO_Init();
MX_DMA_Init();
MX_USART1_UART_Init();
```

## 3. 从普通收发到 DMA 不定长回传

先用 HAL_UART_Transmit 每秒发送 Hello VOFA!，确认电脑能收到. 然后用 HAL_UART_Receive 轮询接收1字节并回传，验证两个方向接线与参数正确. 最后替换为 DMA 版本.

最终工程使用256字节缓冲区和三个状态变量：

```c
uint8_t uart_buffer[256];
volatile uint16_t rx_length = 0;
volatile uint8_t rx_ready = 0;
volatile uint8_t tx_done = 0;
```

接收使用 HAL_UARTEx_ReceiveToIdle_DMA，256是容量上限，不是每次必须收满的长度. 启动后关闭 DMA_IT_HT，避免半缓冲区事件提前通知. 当前 Normal 模式会在空闲检测或缓冲区收满时结束这次接收并通知软件.

```text
启动接收 DMA → 数据进入缓冲区 → 空闲或收满
→ 接收回调记录长度 → 主循环启动发送 DMA
→ 发送完成回调置标志 → 主循环重新启动接收
```

发送使用 HAL_UART_Transmit_DMA，并传入实际 rx_length，因此不同长度的数据都可原样回传. HAL 接口启动失败时进入 Error_Handler.

### 两个回调有什么作用？

回调可以理解为 HAL 给程序的“完成通知”. 程序不直接调用它们，由中断处理流程调用.

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        rx_length = Size;
        rx_ready = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_done = 1;
    }
}
```

- 接收回调：Size 是实际接收字节数，记录长度并通知主循环可以回传. Hello 不带换行是5字节，附加 \r\n 是7字节.
- 发送完成回调：通知串口发送已完成，可以重新使用缓冲区接收. 启动发送 DMA 后函数立即返回，不等于数据已全部发出.
- USART1 判断：只处理本次使用的串口. 回调中仅修改状态，主要操作放在主循环.
- volatile：提醒编译器变量可能被中断修改，不把读取结果一直缓存；它不是通用的线程安全机制.

这版使用同一缓冲区，回传结束前不重新接收，避免发送数据被覆盖. 因此测试时手动发一条，等回传结束再发下一条，不开自动循环发送. 它是基础演示方案，不保证连续高速收发不丢数据.

空闲只代表字节间出现停顿，不保证一段就是完整应用报文. 更复杂通信还需用长度、帧尾或其他组帧规则判断边界.

## 4. VOFA+ 联调结果

选择 CH340 对应串口，本次为 COM3，设置115200、8N1、无流控，数据引擎选择 RawData. 串口号会随电脑和连接变化，不固定为 COM3.

分别发送 A、Hello、STM32 DMA test，截图中每条均有发送与相同内容的接收记录，验证了不同长度文本的回传. 区分软件的发送记录与实际接收记录，不能只看到发送日志就认定成功.

VOFA+ 不同长度数据回传图：________

现有构建日志为0 Error(s)、0 Warning(s)，下载截图显示 Programming Done、Verify OK 和 Application running. 工程代码已核对为 DMA 版本；上位机截图验证数据结果，不能单靠截图判断内部是否使用 DMA.

## 5. Saleae 串口波形验证

逻辑分析仪 GND 接开发板 GND，D0 并接 PA10，观察电脑发给 STM32 的数据；D1 并接 PA9，观察 STM32 回传. 保留 CH340 接线，不连接逻辑分析仪电源输出.

使用12 MS/s 数字采样，D0 下降沿触发，触发后采集1秒. 两路分别配置 Async Serial：115200、8位、无校验、1停止位、LSB First、不反相.

VOFA+ 发送字符 U，不附加换行. U 的 ASCII 为0x55，便于观察交替电平：

```text
空闲高 → 起始0 → 数据1 0 1 0 1 0 1 0 → 停止1
```

D0 先解码为0x55，D1 随后也解码为0x55，表明接收与回传内容一致.

Saleae 解码与位宽测量图：________

### 测量中的几个数值怎么理解？

| 截图数值 | 解释 |
| --- | --- |
| 8.667 μs | 单个位宽，理论值为1/115200≈8.681 μs |
| 115.385 kHz（width⁻¹） | 位宽倒数，普通二进制 UART 中对应约115385 bit/s |
| 17.333 μs | 0x55交替高低，一个完整高低周期含两个位 |
| 57.692 kHz | 上述波形周期的频率，不能当成波特率 |
| 82.417 μs | 解码器显示的标注区间，不能直接当完整帧时长 |
| 185.833 μs | D0起始到D1起始的间隔，包含接收、空闲检测与处理，不是单独的DMA搬运耗时 |

测得位速率与115200相差约0.16%，采样量化和实际时钟均可能影响读数，不能仅凭一次测量分离原因. 8N1一帧为1起始＋8数据＋1停止，共10位，理论时长约86.81 μs；测完整帧应覆盖停止位全部时间.

## 6. 学习收获

本次验证了串口双向通信、DMA接收不同长度数据及回传，并能通过波形解释起始位、数据位、停止位和波特率. 我理解了DMA负责搬运、空闲检测提供事件通知、回调记录状态、主循环组织后续操作，也区分了位宽倒数与交替波形频率.

最终演示：用VOFA+发送不同长度文本并检查回传，再在Saleae中展示U的双向解码和8.667 μs位宽. DMA实现以工程代码为依据，逻辑分析仪验证外部信号与数据.

---

本笔记由王世伟完成实验，Codex 根据问答、工程代码和实验截图辅助整理.
