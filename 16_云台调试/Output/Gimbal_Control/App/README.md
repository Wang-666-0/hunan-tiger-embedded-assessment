# App 代码导航

## 主调用链

`main → MX_FREERTOS_Init → AppTasks_Create → ControlTask → AppLogic_Step → Backend_Apply → Controller_Update → GimbalPid → CAN`。

遥控和 CAN 中断只搬运接收数据，由独立任务解析。控制任务统一协调输出许可和目标；控制台负责调参、结果与模拟显示。真实反馈、控制算法和显示演示分别维护状态。

## 从这些函数开始

| 目录/文件 | 先看函数 | 主要职责 |
| --- | --- | --- |
| system/app_tasks.c | ControlTask | 2 ms 主调度、状态采样及各后端执行 |
| system/app_logic.c | AppLogic_Step | 遥控映射、安全状态机和目标生成 |
| system/remote_mapper.c | RemoteMapper_Map | 摇杆死区、模式、扳机和测试轴选择 |
| system/target_manager.c | TargetManager_Step | 手动请求、单次阶跃事件、50 ms 发射节拍 |
| system/safety_manager.c | SafetyManager_Step | 失联停止、故障撤权和新 OFF 帧重解锁 |
| system/gimbal_startup.c | GimbalStartup_Update | 水平基准上的双轴准备 |
| motors/yaw_backend.c | YawBackend_StartStep | 相对阶跃、实测记录、归位与执行会话 |
| motors/pitch_backend.c | PitchBackend_StartStep | Pitch 阶跃、归位、锁轴及 CAN 提交 |
| motors/yaw_controller.c | YawController_Update | 软件零点、目标积分、Yaw 闭环 |
| motors/pitch_controller.c | PitchController_StartMove | 直接阶跃、双环、重力补偿和制动 |
| control/gimbal_pid.c | GimbalPid_UpdateWithFeedforward | 两轴共用位置—速度 PID 与抗积分饱和 |
| control/yaw_step.c | YawStep_Start / YawStep_Sample | 真实阶跃统计及完整 PASS 判定 |
| motors/feeder_controller.c | Feeder_Run | 单发、连发跟踪、减速及原始电流计算 |
| motors/feeder_backend.c | FeederBackend_Apply | 拨盘准备、事件执行与 CAN 电流提交 |
| motors/flywheel_speed.c | FlywheelSpeed_Step | 6000 rpm 目标爬升和速度 PI |
| motors/flywheel_jog.c | FlywheelJog_Step | 双轮运行会话、反馈与电流请求 |
| motors/motor_backend.c | MotorBackend_Apply | 双轮实际执行与稳速互锁 |
| drivers/ibus.c | ibus_feed_byte | 帧头、校验和、通道范围与重同步 |
| drivers/remote_input.c | RemoteInput_Poll | DMA 字节队列消费与有效遥控发布 |
| drivers/can_bus.c | HAL_CAN_RxFifo0MsgPendingCallback | CAN 队列、固定设备命令、发送邮箱归属 |
| drivers/gl60_ii.c / dm3507.c | BuildTorque / MakeTorque | 电机协议与物理量量化 |
| console/app_console.c | HandleLine / AppConsole_Poll | 串口路由、视图、SIM 与遥测 |
| console/*_console.c | *_Console_Handle | 电机调参与实测结果命令 |

每个 `.c` 的文件头给出完整函数和文件级变量清单；函数定义的 `)` 与 `{` 之间给出作用、参数、单位、返回值，复杂函数另讲处理顺序。主要执行函数排在前面，辅助判断与异常处理排在后面。

## 配置和保留功能

`app_config.h` 保留集中调参入口。整数使用枚举，浮点使用带类型常量；仅拨盘独立测试保留模式宏。当前模式为 1，摩擦轮不输出，拨盘保留原安全准备、单发和连发。

模拟曲线四个头文件保持原样，SIM 命令和遥测通道位置保留；控制器和 RESULT 仍只读真实反馈。移除可选轨迹整形后，原轨迹遥测列保持目标/零值，占位不移动后续列。

本次删除：关闭单个电机的兼容分支、旧逐字节串口接收、Pitch 可选轨迹、落点快照、全局撤权追踪、清错门禁快照及重复诊断输出。调试串口通信异常只影响命令组装/统计，不再作为整机遥控撤权的来源。

保留：当前 PID 与补偿参数、遥控映射、基本行程与反馈检查、使能/停止握手、真实阶跃统计、VIEW 锁轴、单发/连发和必要串口调试。
