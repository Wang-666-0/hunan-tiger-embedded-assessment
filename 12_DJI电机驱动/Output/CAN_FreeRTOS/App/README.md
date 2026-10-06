# GM6020 应用模块

建议按“配置 → 任务 → 反馈协议 → 共享反馈 → 输出控制 → 串口 → 硬件收发”的顺序阅读。每个 .c 文件头说明功能，关键代码行保留并补充中文注释。

| 文件 | 职责 |
| --- | --- |
| app_config.h | ID、协议、限幅、周期、超时、优先级、栈与诊断开关 |
| can_tasks.c/.h | 调度接收、控制和串口任务，直接调用电机反馈解析 |
| gm6020_codec.c | 纯反馈解包、在线判断、控制报文编码 |
| can_protocol.c/.h | 消息类型与大端 16 位读写 |
| gm6020.c/.h | 反馈发布、接收时间戳、临界区快照 |
| motor_control.c/.h | 输出请求、周期检查、发送与异常撤销 |
| motor_console.c/.h | UART 单字节接收、恢复接收、组行、命令解析与遥测 |
| can_transport.c/.h | CAN1 启动、FIFO/队列、提交和取消邮箱、HAL 回调 |
| can_debug.c/.h | 基础统计与故障码；可选诊断不承担电机正常功能 |

启动入口为 Core/Src/freertos.c 的 USER CODE：先建两个队列，再建三个应用任务。额外诊断 APP_ENABLE_DIAGNOSTICS 默认0；开启时才编译原始帧副本、硬件采样与栈余量。

反馈流程：CanRxTask → GM6020_ParseFeedback → GM6020_DecodeFeedback → 发布反馈。

命令流程：UART队列 → MotorConsole_HandleByte → MotorConsole_ProcessLine → 保存请求 → MotorControl_Step → 编码并发送。

应用功能不写入 CubeMX 自动生成的外设初始化区。重新生成后核对 App 源码分组和包含路径。

[完整学习笔记](../../../01_学习笔记/GM6020工程解析与验收.md)
