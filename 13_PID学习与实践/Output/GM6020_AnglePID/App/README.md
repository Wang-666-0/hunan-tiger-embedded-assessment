# GM6020_AnglePID 应用模块

角度外环输出目标 rpm，转速内环输出 CAN 原始电压给定。两环使用自己封装的 PID，共同以 2 ms 周期更新；S 命令仍能单独运行转速环。

| 文件 | 实现功能 |
| --- | --- |
| app_config.h | 电机 ID、时序、限幅、PID 初值和在线参数范围 |
| pid.c / pid.h | 自己封装的 PID：P/I/D、微分滤波、条件积分与限幅；无硬件依赖 |
| motor_control.c / .h | 角度/转速模式、双环 PID、在线增益、一次性制动清 I、保护、阶跃指标；统一发送 CAN |
| motor_console.c / .h | USART1 按行收 A/Z/AP/AI/AD/V 与旧速度命令，21 通道 FireWater 遥测 |
| angle_tracker.c / .h | 编码器跨零累计，支持正反多圈；失联、跳变后锁定无效，静止 Z 重建参考 |
| gm6020_codec.c | GM6020 八字节反馈解析及控制报文槽位编码 |
| gm6020.c / .h | 发布一致速度/多圈反馈，超时锁定位置无效，执行静止置零 |
| can_protocol.c / .h | 大端 16 位数据读写 |
| can_transport.c / .h | CAN1 过滤器、队列、HAL 回调、提交和取消报文 |
| can_tasks.c / .h | 创建接收、控制、串口任务，安排 2 ms / 5 ms / 20 ms 周期 |
| can_debug.c / .h | 基本初始化失败及收发统计；额外原始帧、硬件采样默认关闭 |

`Core/Src/freertos.c` 只创建应用队列与任务。`Core/Src/can.c` 和 `usart.c` 是 CubeMX 外设初始化，业务逻辑集中在 App。

数学 PID 在控制任务中计算；串口只修改请求，不直接改积分状态。控制副本在请求版本与反馈保护均通过后提交，旧计算不能覆盖 STOP 或新目标。新 A 清外环 I，保留微分历史；制动入口只清一次内环 I，不因每次外环输出变化重建内环。微分作用于测量值，目标阶跃没有直接 D 冲击。

第一帧反馈建立临时零点。正式实验先 STOP、静止后 Z。位置失效后不猜圈数，不自动恢复；重新 STOP/Z 后再发新 A。A720 是相对零点的绝对目标，不是每收到一次就再加两圈。详细步骤见[角度学习笔记](../../../01_学习笔记/角度双环与VOFA调参.md)、[VOFA 面板](../../../04_VOFA角度双环/README.md)。关闭 VOFA 不会自动停机。

重新生成 CubeMX 代码可能重建工程文件：应确认 App 的十个 C 文件（包含 angle_tracker.c）仍在 EIDE/Keil 工程中，App 在包含路径中，FreeRTOS 用户区仍调用 CanTasks_Create。当前无需重新生成。
