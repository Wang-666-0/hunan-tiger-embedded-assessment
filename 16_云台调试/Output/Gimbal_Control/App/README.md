# App 框架说明

当前生产配置与使用入口见 [工程说明](../README.md)。本目录继续保持平铺模块，已有 32 个 C 文件全部参与工程构建。新增 `console_parse.h` 只共享纯解析函数，无新增任务或 C 编译单元。

## 调用链与模块边界

`main → freertos → AppTasks_Create` 创建遥控、CAN 接收、控制、控制台四个任务。控制任务在原临界区内取得快照、执行保护、更新目标并提交各执行器命令；串口格式化由控制台任务处理。

| 层次 | 模块 | 职责 |
| --- | --- | --- |
| 任务与故障 | app_tasks、app_fault | 任务创建、状态发布、统一停机与人工重解锁屏障 |
| 遥控输入 | ibus、remote_input、remote_mapper | 帧校验、DMA接收/时间戳、通道映射、模式与回中条件 |
| 业务与许可 | app_logic、safety_manager、target_manager | 保护优先、许可组合、轮子/拨盘请求、阶跃事件与轴记忆 |
| 双轴准备 | gimbal_startup | 安全 OFF 自动准备及显式重新建立基准 |
| CAN | can_frame、can_bus | 双总线收发、白名单、故障代次、邮箱撤销与停机接口 |
| 只读采集 | dm_diag、vesc_diag、app_diagnostics | 参数查询、协议拼接、超时与结果交付 |
| 双摩擦轮 | motor_backend、flywheel_jog、flywheel_speed | 双轮会话、反馈健康、6000 rpm 速度环和停机确认 |
| 拨盘 | feeder_backend、feeder_controller、feeder_console | C610 反馈、位置/速度控制、发射互锁及串口接口 |
| Yaw | gl60_ii、yaw_backend、yaw_controller、yaw_console | GL60 II 协议、许可/准备/停机、角度参考和双环入口 |
| Pitch | dm3507、pitch_backend、pitch_controller、pitch_console | DM3507 协议、坐标/水平标定、机械边界和重力补偿 |
| 共用控制数学 | yaw_cascade | 两轴的速度观测、位置/速度 PID 与抗积分饱和 |
| 阶跃测量 | yaw_step、pitch_step | 共用统计核心、独立实例与 250/300 ms 判定 |
| 控制台 | app_console、console_parse.h | 命令分发、DMA 回调、遥测/报告、双轴共用数值解析 |

## 本次等价精简

- 删除 `app_console` 中嵌在 Yaw 关闭分支内、却又要求 Yaw 开启的恒假旧遥测块。
- Yaw/Pitch 数字与三参数 PID 解析共用 `console_parse.h`，保持空格、尾字符、有限值和错误输入处理。
- 控制任务的四处阶跃编号同步改用 `YawBackend_GetStepId` / `PitchBackend_GetStepId`。原测量结构为 192 字节，原每周期为读编号复制四份完整结构；新接口只读取编号，原 `GetStep` 和报告接口继续保留。
- Yaw 阶跃参数冻结按单环/双环二选一，去掉双环模式下必被覆盖的单环赋值。
- Pitch 删除被 100 ms 超时判断完全覆盖的半周期判断；反馈接收时的时间方向判断保持。
- 遥控回中复用已计算的左摇杆中心标志，条件不变。

控制任务在 AppLogic 更新前和两轴 Apply 后的编号同步位置全部保留，分别覆盖串口接受与本周期遥控接受的新阶跃；不能把它们误当重复逻辑删除。

## 保留的功能与接口

旧单环、限幅双环、Yaw 实验规划/前馈和历史遥控/摩擦轮档按用户确认保留。生产配置默认关闭实验路径，测试显式启用；不应为缩短源码删掉这些可选能力。

Pitch 使用 `YawCascade_UpdateWithFeedforward` 实现重力补偿，使用 `YawStep` 统计基础；不能因为模块名含 Yaw 就删除共用字段或接口。两轴准备/停机握手和重力坐标失效规则也不等价，继续分开处理。

本次没有调整 PID、滤波、2 ms 控制周期、Yaw 10 ms / Pitch 2 ms 力矩提交、机械限位、遥控解锁、故障撤权、CAN 重发设置、遥测列序或成绩判据。软件回归用于验证这些路径仍有效，实际阶跃性能需要实机曲线验证。

原文档及历次开发描述保存在 [精简前 App 说明](../../../02_过程记录/框架等价精简_20261019/before/Output/Gimbal_Control/App/README.md)；本次逐项改动与验证证据见 [实现与验证](../../../02_过程记录/框架等价精简_20261019/实现与验证.md)。
