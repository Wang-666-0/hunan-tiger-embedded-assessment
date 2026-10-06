# CAN 通信考核学习笔记

姓名：王世伟。日期：2026-10-06。

使用 STM32F405RGT6 云台板，实现 CAN1、CAN2 经板载收发器双向通信，通过 Keil Watch 验收。本文合并原理、配置、代码设计、调试和运行结果，按当前分文件工程整理，由 Codex 辅助编写。

## 1. 先掌握这五个重点

1. **RX/TX 是控制器与收发器之间的逻辑信号；CAN_H/CAN_L 是两根差分总线。**
2. **CAN 位速率按 PCLK1=42 MHz 计算，结果为 500 kbit/s；100 ms 是任务发送周期。**
3. **中断搬运报文，队列复制数据，任务负责校验和保存。**
4. **提交邮箱成功、硬件发送完成、对方有效接收，是三层不同的证据。**
5. **验收看两路计数持续增长，再核对 ID、8 字节数据和序号。数组附带字符显示是正常现象。**

重构前已完成编译烧录，截图核对到 CAN1 收到 0x202/A2、CAN2 收到 0x201/A1，两路收发内容正确。分文件版本通过 EIDE 构建和 Keil 完整重编译，Keil 为 0 错误、0 警告；保存的截图属于拆分前，拆分后还需重新烧录复验。未进行长期丢帧率或周期抖动测量。

## 2. CAN 原理：控制器、收发器与总线

```text
CAN1 控制器 ⇄ RX/TX ⇄ CAN1 收发器
                            ⇅
                    CAN_H / CAN_L 总线
                            ⇅
CAN2 控制器 ⇄ RX/TX ⇄ CAN2 收发器
```

| 部分 | 作用 |
| --- | --- |
| 控制器 | 生成/解析帧，完成仲裁、CRC、ACK 和错误检测 |
| 收发器 | 在逻辑电平与差分总线电平之间转换 |
| CAN_H、CAN_L | 共同表示差分状态，连接为 H 接 H、L 接 L |

**H/L 不是分别发送和接收的两根线。** CubeMX 配置 MCU 的 CAN_RX/CAN_TX，板子外部接口标记 CAN_H/CAN_L，两者处于不同层次。两个控制器经各自收发器构成两个通信节点。

本次使用经典 CAN 标准数据帧：ID 为 11 位，范围 0x000～0x7FF；DLC 是数据长度，本次为 8 字节。ID 标识报文并参与仲裁，不是接收设备地址。在相同帧格式下，较小 ID 优先级更高。

Normal 模式用于实际总线通信，发送需要其他接收节点给出 ACK。ACK 表示节点正确接收了帧，不保证接收任务已经处理数据，所以还要看有效接收计数。回环模式用于控制器内部自测，不能单凭回环证明外部接线正确。

高速 CAN 常用总线两端各 120 Ω 的终端电阻，板上是否提供要结合电路核对。收发器原理参考 [Analog Devices 官方说明](https://www.analog.com/en/resources/technical-articles/understanding-can-transceiver-how-validate-multinode-can-system-performance.html)。

## 3. CubeMX 配置与参数计算

### 3.1 引脚和时钟

| 项目 | 当前配置 |
| --- | --- |
| MCU | STM32F405RGT6，LQFP64 |
| SWD | PA13/SWDIO、PA14/SWCLK；SYS → Debug → Serial Wire |
| CAN1_RX / TX | PA11 / PA12，AF9 |
| CAN2_RX / TX | PB12 / PB13，AF9 |
| HSE / PLL | 25 MHz；M=25、N=336、P=2，系统时钟选择 PLLCLK |
| SYSCLK / HCLK | 168 MHz / 168 MHz |
| PCLK1 / PCLK2 | 42 MHz / 84 MHz |
| HAL 时基 / FreeRTOS tick | TIM2 / SysTick；内核 1000 Hz，即 1 tick=1 ms |

```text
PLL VCO = 25 MHz / 25 × 336 = 336 MHz
SYSCLK  = 336 MHz / 2 = 168 MHz
PCLK1   = 168 MHz / 4 = 42 MHz
```

**CAN 使用 APB1 外设时钟 42 MHz；图中的 APB1 Timer clocks=84 MHz 是定时器时钟。**

本次两个易错点：设置 M/N/P 后仍显示 25 MHz，是 System Clock Mux 还选着 HSE，要改选 PLLCLK；PLLQ 不供 CAN 使用，本次没有启用要求 48 MHz 的外设，因此 Q 不是本次配置重点。

### 3.2 500 kbit/s 怎样算出来

| 参数 | 值 | 含义 |
| --- | --- | --- |
| Prescaler | 6 | 决定时间量子 TQ 的分频 |
| Sync Segment | 固定 1 TQ | 同步段 |
| BS1 / BS2 | 11 / 2 TQ | 采样点前后时间段 |
| SJW | 1 TQ | 重同步最大调整长度 |
| Mode | Normal | 正常总线通信 |

```text
TQ = 6 / 42 MHz ≈ 142.857 ns
每 bit = 1 + 11 + 2 = 14 TQ = 2 μs
波特率 = 42 MHz / [6 × (1 + 11 + 2)] = 500000 bit/s
采样点 = (1 + 11) / 14 ≈ 85.7%
```

CAN Prescaler 按值 6 计算，不能套用定时器 PSC+1 的公式。500 kbit/s 决定每一位的速度，100 ms 决定应用多久发一轮。

### 3.3 开关、内核与中断

| 配置 | 当前值及作用 |
| --- | --- |
| AutoBusOff | Enable，满足恢复条件后自动退出 bus-off |
| AutoRetransmission | Enable，未正常完成的发送由硬件按规则重试 |
| AutoWakeUp / TimeTriggeredMode | Disable，本次不使用 |
| ReceiveFifoLocked / TransmitFifoPriority | Disable，未启用 FIFO 锁定与按入队顺序发送 |
| FreeRTOS 接口 / 堆 | CMSIS_V2；heap_4，10240 字节 |
| 栈溢出检查 / malloc 失败钩子 | 2 / Enabled |
| CAN1、CAN2 RX0 与 TX 中断 | 开启，抢占优先级 5、子优先级 0 |
| CAN SCE / RX1 中断 | 本次未开启 |
| configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY | 5，接收中断可调用 FromISR API |

中断优先级数值越小越紧急，任务优先级数值越大越高。NVIC 允许 CPU 响应中断，ActivateNotification 允许外设产生指定事件，两部分都要设置。

## 4. 分文件框架与任务分工

工程位于 `Output/CAN_FreeRTOS`。main.c 初始化系统和外设、启动内核；freertos.c 保留创建入口和错误钩子，业务分成四个模块。

| 文件 | 职责与设计意义 |
| --- | --- |
| Core/Src/can.c | 位时序、GPIO、时钟、NVIC，集中查看外设配置 |
| Core/Src/stm32f4xx_it.c | 中断入口，调用 HAL_CAN_IRQHandler |
| App/can_tasks.c | 两个任务、周期和优先级，集中管理工作节奏 |
| App/can_transport.c | 队列、过滤器、启动、发送、HAL 回调，集中处理硬件和中断交接 |
| App/can_protocol.c | 编码、解码和格式校验，让数据规则独立于硬件与调度 |
| App/can_debug.c | 最近报文、计数、错误和 ESR，保留统一 Watch 入口 |

```text
HAL、时钟、GPIO、CAN1、CAN2 初始化
  → osKernelInitialize
  → MX_FREERTOS_Init：先创建队列，再创建任务
  → osKernelStart
  → CAN_RX 等数据；CAN_TX 调用 CanStart 后周期发送
```

**队列必须先于接收通知创建，避免中断向不存在的队列写数据。**

| 任务 | 优先级 | 栈 | 工作方式 |
| --- | --- | --- | --- |
| CAN_RX | 4 | 256 words=1024 字节 | 无报文阻塞，有报文保存/校验 |
| CAN_TX | 3 | 256 words=1024 字节 | 每 100 ms 两路各提交一帧，采样硬件状态 |
| defaultTask | CMSIS Normal | 512 字节 | 每 1000 tick 延时一次，不负责 CAN 业务 |

接收任务比发送任务优先级高，但没有数据就阻塞，让其他任务运行。原生 xTaskCreate 的栈单位是 word，CMSIS stack_size 的单位是字节。

```c
xTaskCreate(CanRxTask, "CAN_RX", 256, NULL, 4, NULL);
```

六个参数是入口、调试名称、栈深度、用户参数、优先级、句柄输出地址。两个 NULL 分别表示不传参数、不保存句柄；实际代码检查返回值是否为 pdPASS。

## 5. 过滤器、邮箱、FIFO、队列怎样区分

| 对象 | 位置 | 本次用途 |
| --- | --- | --- |
| 过滤器 | CAN 硬件 | 决定哪些帧进入 FIFO |
| 发送邮箱 | 每路 CAN 硬件，3 个 | 暂存待发帧，等待仲裁和发送 |
| FIFO0 | 每路 CAN 硬件 | 暂存收到的帧，等中断读取 |
| can_rx_queue | FreeRTOS 软件，16 帧 | 将中断读出的报文交给任务 |

**硬件 FIFO 和软件队列是不同的缓冲，增加软件队列长度不会扩大硬件 FIFO。**

```c
filter.SlaveStartFilterBank = 14;
filter.FilterBank = 0;   // 配置 CAN1
filter.FilterBank = 14;  // 配置 CAN2
```

双 CAN 共用 28 个过滤器组。分界 14 后，0～13 分给 CAN1，14～27 分给 CAN2，本次每路只启用一个组。CAN2 使用公共过滤器资源时还需要 CAN1 相关时钟。参考 [ST RM0090：bxCAN](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。

本次选择 IDMASK、32BIT、FIFO0、ENABLE，ID 和掩码全 0。掩码全 0 让硬件接收全部 ID，中断筛选标准数据帧，任务再校验本次测试内容。

CanStart 顺序：配置过滤器 → HAL_CAN_Start → 开启 FIFO0 消息/溢出和 TX 邮箱通知 → can_ready=1。自动重发不保证错误总线也能发送成功，仍要检查完成计数与对端接收。

## 6. 报文格式与关键收发流程

### 6.1 8 字节测试数据

| 方向 | 标准 ID | 前四字节 | 后四字节 |
| --- | --- | --- | --- |
| CAN1 → CAN2 | 0x201 | A1 12 34 56 | uint32 小端序号 |
| CAN2 → CAN1 | 0x202 | A2 12 34 56 | uint32 小端序号 |

两路每轮使用同一序号，初值 0，轮末加 1。小端是低有效字节在前：1 为 01 00 00 00，256 为 00 01 00 00。

```c
data[4] = (uint8_t)sequence;
data[5] = (uint8_t)(sequence >> 8);
data[6] = (uint8_t)(sequence >> 16);
data[7] = (uint8_t)(sequence >> 24);
```

解码先转 uint32_t，再移位和按位或拼接，避免依赖数组对齐或指针强转。序号用于观察更新，本次未额外检查序号连续性或统计丢帧。

### 6.2 发送：三层成功证据

```c
CanSend(&hcan1, CAN1_TEST_ID, CAN1_TEST_MARKER, sequence);
HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
```

CanSend 的参数是控制器句柄、标准 ID、发送方标记、序号。它检查空闲邮箱，构造标准数据帧，设置 DLC=8、TransmitGlobalTime=DISABLE；邮箱满就记录 busy 并跳过，避免忙等。

AddTxMessage 中 header 指向帧属性，data 指向数据，mailbox 返回所选邮箱。**HAL_OK 只表示提交成功，实际完成由 TxMailbox0/1/2CompleteCallback 计数。**

| 层次 | 看什么 | 证明什么 |
| --- | --- | --- |
| 提交邮箱 | 本端 tx_queued_count 增长 | 软件交给硬件的请求成功 |
| 实际发送 | 本端 tx_count 增长 | 硬件完成正常发送 |
| 应用接收 | 对端 rx_valid_count 增长，ID/数据正确 | 对方任务收到并校验测试数据 |

周期用 `vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100))`：last_wake 保存基准，100 ms 换成 tick，避免累计每轮执行时间。抢占、超时和总线仲裁仍会引入抖动，不能理解成帧上总线时刻绝对精确。

### 6.3 接收：中断搬运，任务处理

```text
FIFO0 → CANx_RX0_IRQHandler → HAL_CAN_IRQHandler
  → HAL_CAN_RxFifo0MsgPendingCallback：取帧、筛选、入队
  → CanRxTask：阻塞取队列
  → CanDebug_RecordRx：保存、校验、计数
```

CanRxMessage 包含 source、id、dlc、data[8]。**source 是接收控制器编号，不是发送方；source=1 应收到 CAN2 的 0x202/A2。**

```c
xQueueSendFromISR(can_rx_queue, &message, &task_woken);
portYIELD_FROM_ISR(task_woken);
CanTransport_Receive(&message, portMAX_DELAY);
```

队列复制整个结构体，局部 message 在中断结束后消失，也不会影响队列中的报文。task_woken 记录是否唤醒更高优先级任务，退出中断时按需切换。接收任务没有数据就阻塞，不轮询忙等。

中断不打印、不等待互斥锁，将校验交给任务。任务检查对方 ID、DLC=8、A1/A2 标记、12/34/56 固定内容，增加有效或无效计数，再增加处理总数。能被硬件接收不代表格式一定有效。

## 7. Keil 调试与实际验收

### 7.1 操作步骤

1. 打开 `Output/CAN_FreeRTOS/MDK-ARM/CAN_FreeRTOS.uvprojx`，重新编译。
2. Options for Target → Debug 选择 ST-Link Debugger，Settings 选 SW 接口并确认识别芯片。
3. Flash → Download 下载，进入 Debug Session 后点击 Run。
4. View → Watch Windows → Watch 1，双击 Enter expression 添加下面的变量。
5. View → Periodic Window Update 用于运行中刷新；不能实时刷新时暂停读取，再继续运行。
6. Watch 右键 Hexadecimal Display：勾选为十六进制，取消为十进制；ID/数组看十六进制，计数看十进制。

EIDE 的 axf 在 `MDK-ARM/build/CAN_FreeRTOS`，Keil 的 axf 在 `MDK-ARM/CAN_FreeRTOS`。调试符号必须对应板上程序，用当前 Keil 工程编译、下载后调试最直接。操作参考 [Keil 官方 Watch 说明](https://www.keil.com/support/man/docs/uv4/uv4_db_dbg_watchwin.asp)。

### 7.2 核心 Watch 变量

| 变量 | 预期 |
| --- | --- |
| can_ready / can_fault_code | 1 / 0 |
| can1_tx_count、can2_tx_count | 实际发送计数持续增长，正常约每秒 10 次 |
| can1_rx_valid_count、can2_rx_valid_count | 有效接收计数持续增长，正常约每秒 10 次 |
| can1_last_rx_id / can2_last_rx_id | 0x202 / 0x201，十进制 514 / 513 |
| can1_last_rx_dlc、can2_last_rx_dlc | 8 |
| can1_last_rx_data | 前四字节 A2 12 34 56 |
| can2_last_rx_data | 前四字节 A1 12 34 56 |
| can1_last_rx_sequence、can2_last_rx_sequence | 随新报文增长 |

**运行 5～10 秒，检查两个方向持续更新、内容正确，没有提交失败、无效数据、丢弃或 bus-off。只看编译通过或 queued 增长不够。**

### 7.3 数组与序号没有出错

一次截图中，两路发送和有效接收计数均为 0x212=530，ready=1、fault=0，接收 ID 正确。另一张数组截图记录：

```text
CAN1：ID=0x202，A2 12 34 56 6C 01 00 00
CAN2：ID=0x201，A1 12 34 56 6C 01 00 00
```

6C 01 00 00 是小端序号 0x16C=364；有效接收数 0x16D=365。序号从 0 开始，0～364 共 365 帧，两者对应。这个关系以本轮前面的帧都正常处理为前提，不是任何时刻都必须成立。

**数组地址、字符摘要和字节后的 '4'/'V'，只是 Keil 对 uchar 的附加显示。** 展开后核对八个十六进制字节即可。volatile 便于观察内存，但不保证多个字段原子更新；核对同一帧全部数据时可以暂停读取。

两张截图来自不同调试时刻，不能把 530 与 365 相减计算同一次连续运行的帧数。

## 8. 错误定位对照

| 变量 | 含义 / 正常现象 |
| --- | --- |
| can_tx_task_cycle_count | 发送轮数，持续增长 |
| can1_tx_queued_count / can2_tx_queued_count | 邮箱提交成功数，与发送完成数分开 |
| can1_tx_busy_count / can2_tx_busy_count | 无空闲邮箱而跳过，正常为 0 |
| can1_tx_submit_error_count / can2_tx_submit_error_count | HAL 提交失败，正常为 0 |
| can1_rx_count / can2_rx_count | 任务处理的标准数据帧总数 |
| can1_rx_invalid_count / can2_rx_invalid_count | 不符合测试格式，正常为 0 |
| can_rx_read_error_count | FIFO 读取失败/长度异常，正常为 0 |
| can_rx_queue_full_count | 软件队列满而丢弃，正常为 0 |
| can_rx_ignored_count | 忽略的扩展/远程帧，本测试正常为 0 |
| can1_error_callback_count / can2_error_callback_count | 已启用通知触发的错误回调，正常为 0 |
| can1_last_hal_error / can2_last_hal_error | HAL 错误记录，正常为 0 |
| can1_error_status / can2_error_status | ESR 快照，适合十六进制 |
| can1_bus_off / can2_bus_off | 每 100 ms 采样的 bus-off，正常为 0 |
| can1_free_mailboxes / can2_free_mailboxes | 空闲邮箱 0～3，刚提交后可为 2 |

未开启 SCE 中断，错误回调不增加不等于没有总线错误，还要看 ESR/bus-off。ESR 中 TEC 在 23:16、REC 在 31:24、BOFF 是 bit 2；HAL 累积错误记录和当前硬件状态不同。

| 现象 | 优先检查 |
| --- | --- |
| ready=0 | 是否点击 Run、进入任务，再看故障码和调用栈 |
| queued 增加，tx 不增加，随后 busy 增加 | 帧在邮箱但无法发完，查看 ESR、对方状态和总线 |
| tx 增加，对方 valid 不增加 | 对方总接收数、无效数、ID、DLC、原始数据 |
| invalid 增加 | 帧到了应用，但格式不符 |
| 队列满/FIFO 溢出 | 任务是否及时处理、是否长时间停断点、是否有高频节点 |

| can_fault_code | 原因 |
| --- | --- |
| 0 | 未记录应用致命错误 |
| 1 | 接收队列创建失败 |
| 2 / 3 / 4 | 接收 / 发送 / 默认任务创建失败 |
| 11 / 12 | CAN1 / CAN2 过滤器配置失败 |
| 13 / 14 | CAN1 / CAN2 启动失败 |
| 15 / 16 | CAN1 / CAN2 通知开启失败 |
| 21 / 22 | 栈溢出 / 内存分配失败钩子 |

内存分配失败可能先进入钩子记录 22。若失败发生在外设初始化阶段，fault_code 可能仍为 0，此时看停止函数和调用栈。

## 9. 过程图片

图 02 · 168 MHz 时钟配置：[GitHub 直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/%E5%87%BD%E6%95%B0%E5%88%86%E6%96%87%E4%BB%B6%E7%BC%96%E5%86%99/11_CAN%E9%80%9A%E4%BF%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/02_CubeMX_168MHz%E6%97%B6%E9%92%9F%E9%85%8D%E7%BD%AE.png)

图 03 · CAN 500 kbit/s、自动 bus-off 管理与自动重发：[GitHub 直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/%E5%87%BD%E6%95%B0%E5%88%86%E6%96%87%E4%BB%B6%E7%BC%96%E5%86%99/11_CAN%E9%80%9A%E4%BF%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/03_CAN_500k%E4%B8%8E%E8%87%AA%E5%8A%A8%E9%87%8D%E5%8F%91%E9%85%8D%E7%BD%AE.png)

图 04 · CAN1/CAN2 引脚与 RX0 中断优先级：[GitHub 直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/%E5%87%BD%E6%95%B0%E5%88%86%E6%96%87%E4%BB%B6%E7%BC%96%E5%86%99/11_CAN%E9%80%9A%E4%BF%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/04_CAN%E5%BC%95%E8%84%9A%E4%B8%8E%E6%8E%A5%E6%94%B6%E4%B8%AD%E6%96%AD%E9%85%8D%E7%BD%AE.png)

图 05 · Keil 收发计数与十六进制显示：[GitHub 直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/%E5%87%BD%E6%95%B0%E5%88%86%E6%96%87%E4%BB%B6%E7%BC%96%E5%86%99/11_CAN%E9%80%9A%E4%BF%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/05_Keil%E6%94%B6%E5%8F%91%E8%AE%A1%E6%95%B0%E4%B8%8E%E5%8D%81%E5%85%AD%E8%BF%9B%E5%88%B6%E6%98%BE%E7%A4%BA.png)

图 06 · Keil 双向接收报文、数组与小端序号：[GitHub 直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/%E5%87%BD%E6%95%B0%E5%88%86%E6%96%87%E4%BB%B6%E7%BC%96%E5%86%99/11_CAN%E9%80%9A%E4%BF%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/06_Keil%E5%8F%8C%E5%90%91%E6%8E%A5%E6%94%B6%E6%8A%A5%E6%96%87%E4%B8%8E%E5%BA%8F%E5%8F%B7.png)
