# CAN1、CAN2 双向通信：Keil 调试与验收

## 当前代码实现

STM32F405RGT6，HSE 25 MHz，SYSCLK 168 MHz，PCLK1 42 MHz。CAN1、CAN2 都使用 Normal 模式，Prescaler=6、BS1=11 TQ、BS2=2 TQ、SJW=1 TQ：

```text
波特率 = 42 MHz / [6 × (1 + 11 + 2)] = 500 kbit/s
```

CAN1 使用 PA11/RX、PA12/TX，CAN2 使用 PB12/RX、PB13/TX。TX、RX0 中断优先级均为 5；FreeRTOS 允许调用 FromISR API 的优先级边界也是 5。

应用代码已拆分到 `Output/CAN_FreeRTOS/App`：can_tasks 负责调度，can_transport 负责硬件与回调，can_protocol 负责帧格式，can_debug 负责 Watch 状态。freertos.c 的 USER CODE 区域保留启动调用和故障钩子。代码使用带 BOM 的 UTF-8，VS Code 工程工作区默认编码也设置为 UTF-8。模块阅读顺序见 [App 模块说明](../Output/CAN_FreeRTOS/App/README.md)。

| 工作部分 | 职责 |
| --- | --- |
| CAN_TX 任务，优先级 3 | 启动两路 CAN，每 100 ms 各提交一帧，采样硬件状态 |
| CAN_RX 任务，优先级 4 | 阻塞等待队列，收到报文后保存数据、解码序号、校验内容 |
| FIFO0 接收中断 | 从硬件 FIFO 取报文，通过 xQueueSendFromISR 复制到队列 |
| 三个发送完成回调 | 在硬件确认发送成功后增加 tx_count |
| defaultTask | 保留 CubeMX 默认任务，每 1000 个 RTOS tick 唤醒一次，不承担 CAN 收发 |

系统 tick 为 1000 Hz，因此这里 1000 tick 对应 1 s。发送任务用 vTaskDelayUntil 固定唤醒时间基准；中断和调度仍可能引入短暂抖动，100 ms 不等于保证每帧恰好在对应时刻出现在总线上。

## 报文格式

两路都发送标准数据帧，DLC=8，序号从 0 开始，每轮增加 1。

| 发送方向 | 标准 ID | 前四字节 | 后四字节 |
| --- | --- | --- | --- |
| CAN1 → CAN2 | 0x201 | A1 12 34 56 | 32 位序号，小端排列 |
| CAN2 → CAN1 | 0x202 | A2 12 34 56 | 32 位序号，小端排列 |

例如序号 1 的 CAN1 报文是 `A1 12 34 56 01 00 00 00`。序号 256 的后四字节是 `00 01 00 00`。

滤波器接受所有 ID；接收中断只转交标准数据帧，接收任务再验证对方的 ID、DLC、标记和固定三个字节。当前序号用于显示持续更新，没有额外实现丢帧或序号连续性统计。

## 打开正确的 Keil 工程

打开仓库下这个工程：

```text
11_CAN通信/Output/CAN_FreeRTOS/MDK-ARM/CAN_FreeRTOS.uvprojx
```

VS Code 的 EIDE 与 Keil 使用不同输出目录。调试前在 Keil 编译并下载当前工程，保证烧录程序和调试符号来自同一次构建：

- EIDE：`MDK-ARM/build/CAN_FreeRTOS/CAN_FreeRTOS.axf`
- Keil：`MDK-ARM/CAN_FreeRTOS/CAN_FreeRTOS.axf`

这里已完成两套构建检查，Keil 完整重编译为 0 错误、0 警告。硬件是否双向收发成功，需要以下 Watch 验收确认。

## 进入硬件调试

1. 若已经处于调试状态，先通过 **Debug → Start/Stop Debug Session** 退出。
2. 编译当前工程，在 **Options for Target → Debug** 确认使用 **ST-Link Debugger**。
3. 在 Debug 的 **Settings** 中确认使用 **SW** 接口，并能识别目标芯片。当前工程已保存 ST-Link 配置。
4. 通过 **Flash → Download** 下载当前 Keil 构建。
5. 通过 **Debug → Start/Stop Debug Session** 进入调试。
6. 如果停在 main，点击 **Debug → Run**，让初始化和 FreeRTOS 调度器继续运行。
7. 打开 **View → Watch Windows → Watch 1**。双击 `<Enter expression>`，逐行添加下面的变量。
8. 勾选 **View → Periodic Window Update**，运行时观察变化。如果无法实时刷新，短暂停止 CPU 后读取，再点击 Run 继续。

官方说明：[开始调试](https://www.keil.com/support/man/docs/uv4/uv4_db_dbg_startdebug.asp)、[Watch 窗口与运行时刷新](https://www.keil.com/support/man/docs/uv4/uv4_db_dbg_watchwin.asp)。

## 第一组：最基本的验收变量

```text
can_ready
can_fault_code
can1_tx_count
can2_tx_count
can1_rx_valid_count
can2_rx_valid_count
can1_last_rx_id
can2_last_rx_id
can1_last_rx_data
can2_last_rx_data
can1_last_rx_sequence
can2_last_rx_sequence
```

| 变量 | 正常现象 |
| --- | --- |
| can_ready | 1，表示两路 CAN 已启动且中断通知已开启 |
| can_fault_code | 0 |
| can1_tx_count、can2_tx_count | 各自持续增长，正常约每秒 10 次 |
| can1_rx_valid_count、can2_rx_valid_count | 各自持续增长，正常约每秒 10 次 |
| can1_last_rx_id | 0x202，十进制 514 |
| can2_last_rx_id | 0x201，十进制 513 |
| can1_last_rx_data | 前四字节 A2 12 34 56 |
| can2_last_rx_data | 前四字节 A1 12 34 56 |
| 两路 last_rx_sequence | 持续增长 |

ID 和数组建议在 Watch 中右键选择十六进制显示；数组可以展开查看八个元素。计数器建议使用十进制。

运行时各字段不是一次性原子快照，收发计数短暂相差少量是可能的。关注两个方向是否持续更新、内容是否正确。

## 第二组：有异常时检查

| 变量 | 含义及预期 |
| --- | --- |
| can_tx_task_cycle_count | 发送任务轮数，应持续增加 |
| can1_tx_queued_count、can2_tx_queued_count | 成功提交到邮箱的数量；不等于实际发送成功 |
| can1_tx_busy_count、can2_tx_busy_count | 三个邮箱都占用而跳过本帧的次数，正常保持 0 |
| can1_tx_submit_error_count、can2_tx_submit_error_count | HAL 提交失败次数，正常保持 0 |
| can1_rx_count、can2_rx_count | 接收任务处理的标准数据帧总数 |
| can1_rx_invalid_count、can2_rx_invalid_count | ID、长度或固定数据不符合本次测试协议的数量，正常保持 0 |
| can_rx_read_error_count | 硬件 FIFO 读取失败或长度异常次数，正常保持 0 |
| can_rx_queue_full_count | 软件队列满导致丢弃的次数，正常保持 0 |
| can_rx_ignored_count | 忽略的扩展帧或远程帧数量，本次双向测试应保持 0 |
| can1_last_hal_error、can2_last_hal_error | HAL 累积错误位，正常为 0 |
| can1_error_callback_count、can2_error_callback_count | 发送邮箱或接收 FIFO 等已使能通知触发的错误回调次数，正常保持 0 |
| can1_bus_off、can2_bus_off | 每 100 ms 采样的 bus-off 标志，正常为 0 |
| can1_error_status、can2_error_status | CAN ESR 寄存器快照，用十六进制查看 |
| can1_free_mailboxes、can2_free_mailboxes | 每轮提交后采样的空闲邮箱数，范围 0～3；刚提交后可以为 2 |

未开启 SCE 错误状态中断，因此并非所有总线错误都会即时触发 HAL 错误回调。bus-off 和 ESR 由发送任务定期读取；HAL 错误位为累积记录，不等同于当前硬件状态。ESR 的 TEC 位于 23:16，REC 位于 31:24，BOFF 位为第 2 位。

典型判断：

- `can_ready=0`：先看是否停在 main、是否点击 Run，再检查 can_fault_code 和当前停住位置。
- queued 已增长但 tx_count 不增长，随后 busy 增长：邮箱里有待发报文，发送尚未成功；结合两路 ESR、bus_off 和已连接的总线检查。
- 两路 tx_count 增长但某一路 rx_valid_count 不增长：看该路 rx_count、invalid_count、最新 ID 和数组。
- invalid_count 增长：报文已到接收任务，但内容不符合当前测试格式；查看原始报文。
- 队列满或 FIFO 溢出：检查是否频繁停断点、是否存在其他高频发送节点。

## can_fault_code

| 数值 | 含义 |
| --- | --- |
| 0 | 未记录应用致命错误 |
| 1 | 接收队列创建失败 |
| 2、3、4 | 接收任务、发送任务、默认任务创建失败 |
| 11、12 | CAN1、CAN2 滤波器配置失败 |
| 13、14 | CAN1、CAN2 启动失败 |
| 15、16 | CAN1、CAN2 中断通知开启失败 |
| 21 | FreeRTOS 检测到任务栈溢出 |
| 22 | FreeRTOS 内存分配失败 |

开启 malloc 失败钩子后，队列或任务内存分配失败可能先进入钩子，此时会记录 22。致命错误会进入 Error_Handler 并停止运行。若错误发生在任务启动前的 CubeMX 外设初始化中，can_fault_code 可能仍是 0，此时看 Keil 当前停止的函数和调用栈。

## 本次硬件验收标准

让程序连续运行 5～10 秒：两路实际发送计数、有效接收计数和序号持续增长；CAN1 收到 0x202/A2，CAN2 收到 0x201/A1；没有提交失败、队列丢弃、数据校验失败或 bus-off。满足这些现象，就能证明两路 CAN 经已连接的物理总线互发互收。

不要把“编译成功”“加入邮箱成功”单独当作硬件通信验收结果。
