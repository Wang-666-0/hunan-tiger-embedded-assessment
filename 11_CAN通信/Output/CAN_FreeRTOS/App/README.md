# CAN 双向通信：模块阅读与验收

## 调用顺序

`freertos.c` 创建软件接收队列、默认任务和两个 CAN 任务。CAN_TX 任务启动控制器后，定期发送；FIFO0 中断搬运到队列，CAN_RX 任务校验并更新 Watch。

| 文件 / 函数 | 职责与参数 |
| --- | --- |
| can_tasks.c / CanTasks_Create(void) | 创建 CAN_RX（优先级 4）和 CAN_TX（优先级 3），各 256 words 栈 |
| can_tasks.c / CanTxTask(argument) | 私有任务函数；每 100 ms 两路各提交一帧、推进序号、采样硬件状态 |
| can_tasks.c / CanRxTask(argument) | 私有任务函数；阻塞等待队列，交给调试模块保存/校验 |
| can_transport.c / CanTransport_CreateQueue(void) | 创建 16 帧软件队列，在开启接收中断前调用 |
| can_transport.c / CanStart(void) | 配置共用滤波器组，启动两路 CAN 并开启 RX/TX 通知 |
| can_transport.c / CanSend(hcan, id, marker, sequence) | hcan 为 CAN1/2 句柄；id 为标准 ID；marker 标记发送方；sequence 为 32 位序号；无空闲邮箱则记录并跳过 |
| can_transport.c / CanTransport_Receive(message, wait_ticks) | message 是输出报文；wait_ticks 单位是 RTOS tick；任务用 portMAX_DELAY 等待 |
| can_transport.c / HAL_CAN_RxFifo0MsgPendingCallback(hcan) | 从对应控制器 FIFO0 取帧，筛选标准数据帧，复制进软件队列 |
| can_transport.c / HAL_CAN_TxMailbox0/1/2CompleteCallback(hcan) | 各邮箱实际发送完成的回调，调用私有 CanTxCompleteCount 计数 |
| can_transport.c / HAL_CAN_ErrorCallback(hcan) | 记录错误回调次数与 HAL 错误位 |
| can_protocol.c / CanProtocol_Encode(marker, sequence, data) | data 为至少 8 字节输出缓冲；编码标记、固定内容、小端序号 |
| can_protocol.c / CanDecodeSequence(data) | data 为至少 8 字节输入缓冲；读取第 4..7 字节的小端序号 |
| can_protocol.c / CanProtocol_IsValid(message) | 检查对方 ID、长度、标记、固定内容；返回非零表示符合测试格式 |
| can_debug.c / CanDebug_RecordRx(message) | 保存最近报文、序号和有效/无效计数 |
| can_debug.c / CanDebug_SampleHardware(void) | 采样 ESR、bus-off、HAL 错误、空闲发送邮箱数 |
| can_debug.c / CanFail(code) | code 写入 can_fault_code，再进入 Error_Handler；故障码定义在头文件 |

## 要看懂的部分

- CAN1/CAN2 共用 28 个滤波器组，分界 14：CAN1 用第 0 组，CAN2 用第 14 组。
- 掩码为 0 接收全部 ID；中断筛选帧类型，任务按本次测试的 ID/内容校验。
- `CanRxMessage.source` 是接收控制器编号。因此 source=1 应收到 CAN2 的 0x202/A2。
- HAL_OK 表示成功提交邮箱；发送完成回调增加 tx_count，才能表示实际完成。
- 中断只搬运数据，不阻塞打印；队列复制局部结构体，不保存局部变量指针。
- volatile 便于调试观察，但不能保证多个字段同时更新。暂停后更适合比较一帧全部数据。
- 数组是二进制数据，Keil 在字节后显示字符是正常附加显示，展开看十六进制字节即可。

## 验收

正常时 can_ready=1、can_fault_code=0；两路 tx_count、rx_valid_count 和序号持续增加。

| 接收端 | 最近 ID | 数据前四字节 |
| --- | --- | --- |
| CAN1 | 0x202 | A2 12 34 56 |
| CAN2 | 0x201 | A1 12 34 56 |

无提交失败、内容无效、队列丢弃或 bus-off。原有 Watch 名称全部保留，详细操作见考核目录下 `01_学习笔记/Keil调试与收发验收.md`。

阅读顺序建议：can_tasks → can_protocol → can_transport → can_debug。
