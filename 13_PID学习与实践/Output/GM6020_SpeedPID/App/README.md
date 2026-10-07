# GM6020_SpeedPID 应用模块

本阶段只做转速单环。阅读顺序从参数、PID 数学到控制和串口，最后看任务调度。

| 文件 | 实现功能 |
| --- | --- |
| app_config.h | 电机 ID、时序、限幅、PID 初值和在线参数范围 |
| pid.c / pid.h | 自己封装的 PID：P/I/D、微分滤波、条件积分与限幅；无硬件依赖 |
| motor_control.c / .h | 锁存目标 rpm、连续运行、保留历史的在线增益、保护与周期闭环；统一发送 CAN |
| motor_console.c / .h | USART1 按行收命令、S/P/I/D/K/STOP/T0/T1、FireWater 曲线 |
| gm6020_codec.c | GM6020 八字节反馈解析及控制报文槽位编码 |
| gm6020.c / .h | 发布并复制一致电机反馈，判断反馈在线 |
| can_protocol.c / .h | 大端 16 位数据读写 |
| can_transport.c / .h | CAN1 过滤器、队列、HAL 回调、提交和取消报文 |
| can_tasks.c / .h | 创建接收、控制、串口任务，安排 2 ms / 5 ms / 10 ms 周期 |
| can_debug.c / .h | 基本初始化失败及收发统计；额外原始帧、硬件采样默认关闭 |

`Core/Src/freertos.c` 只创建应用队列与任务。`Core/Src/can.c` 和 `usart.c` 是 CubeMX 外设初始化，业务逻辑集中在 App。

数学 PID 在控制任务中计算；串口修改请求，不直接改积分状态。目标与实际两曲线读取同一份控制快照。S 一次发送后持续运行；参数滑块不需要循环 K。具体命令见[任务入口](../../../README.md)，控件绑定与返回值见[VOFA 使用步骤](../../../03_VOFA调参/README.md)。

重新生成 CubeMX 代码可能重建工程文件：应确认 App 的九个 C 文件仍在工程中，App 仍在包含路径中，FreeRTOS 用户区仍调用 CanTasks_Create。当前阶段无需重新生成。
