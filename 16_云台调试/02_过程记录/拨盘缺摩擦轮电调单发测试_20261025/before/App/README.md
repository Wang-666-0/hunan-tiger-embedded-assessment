# App 代码导航

App 的实现已从 32 个 C 文件整理为 **10 个 C 文件**。原头文件继续保留公开类型和函数接口，避免其他模块、Core 回调或现有测试改用另一套 API。每个 `.c`、`.h` 文件开头均注明职责、主要函数、调用关系和维护约束。

## 去哪里改代码

| 实现文件 | 内容 | 常用入口 |
| --- | --- | --- |
| `app_tasks.c` | RTOS 任务、应用逻辑、双轴启动、致命故障 | `AppTasks_Create`、`AppLogic_Step`、`GimbalStartup_Update` |
| `app_console.c` | USART1 收发、总命令分发、VOFA 视图和遥测 | `AppConsole_Poll`、`HandleLine`、`PublishTelemetry` |
| `motor_console.c` | Yaw、Pitch、拨盘各自的串口命令和报告 | `YawConsole_Handle`、`PitchConsole_Handle`、`FeederConsole_Handle` |
| `can_bus.c` | 双 CAN、通用帧、GL60 II 和 DM3507 协议 | `CanBus_*`、`Gl60_*`、`Dm3507_*` |
| `app_diagnostics.c` | 只读设备查询、VESC 分片与诊断会话 | `AppDiagnostics_*`、`DmDiag_*`、`VescDiag_*` |
| `remote_input.c` | iBUS、遥控映射、安全管理、目标与发射节拍 | `RemoteInput_Poll`、`RemoteMapper_Map`、`SafetyManager_Step`、`TargetManager_Step` |
| `motor_backend.c` | 左右摩擦轮、速度 PI、点动及停机确认 | `MotorBackend_Apply`、`FlywheelJog_*`、`FlywheelSpeed_*` |
| `feeder_controller.c` | 拨盘位置/速度控制、单发/连发及执行后端 | `Feeder_Update`、`FeederBackend_Apply` |
| `yaw_controller.c` | Yaw、两轴共用串级 PID、共用阶跃统计和 Yaw 后端 | `YawCascade_*`、`YawController_Update`、`YawStep_*`、`YawBackend_Apply` |
| `pitch_controller.c` | Pitch、重力补偿、制动、归位、锁轴和实测适配 | `PitchController_Update`、`PitchController_ReturnTo`、`PitchBackend_Apply` |

实现文件内用中文分区注明每一段职责。`APP_SECTION_BEGIN/END` 注释是测试定位边界，不能只删除其中一端；正常固件直接编译这 10 个 C 文件，不包含其他 `.c` 文件，也不需要生成源码。

## 参数统一入口

**参数及常量宏统一定义在 `app_config.h`**，其他头文件只保留防重复包含宏。既有的 150 个分散宏已迁入，并将拨盘初始化参数、保留单环默认值、串口缓冲等原先写在函数中的默认值改为同值宏。

| 要修改的内容 | 在 `app_config.h` 中搜索 |
| --- | --- |
| Yaw 双环 PID | `APP_YAW_POSITION_`、`APP_YAW_SPEED_` |
| Pitch 双环 PID | `APP_PITCH_POSITION_`、`APP_PITCH_SPEED_` |
| 重力与制动 | `APP_PITCH_GRAVITY_`、`BRAKE` |
| 拨盘每发角度、减速比、方向、单发/连发 | `APP_FEEDER_` |
| 摩擦轮目标、极对数、电流和速度环 | `APP_FLYWHEEL_` |
| 遥控通道、死区、档位 | `RC_` |
| 任务周期、栈和优先级 | `PERIOD_MS`、`STACK_WORDS`、`PRIORITY` |
| 机械边界、反馈超时 | `CONTROLLER_`、`TIMEOUT_MS` |
| 真实阶跃判据 | `YAW_STEP_`、`PITCH_STEP_` |
| 带标记的演示显示 | `DISPLAY` |

单位写在名称后缀或行尾注释中。`MS` 是毫秒，`DEG` 是度，`DPS` 是度/秒，`NM` 是牛·米；ERPM 和机械 RPM 分开，C610 电流原始值不等于 mA。协议帧长度、寄存器编号、编码端点及故障位也集中保存，但应按协议维护。

修改宏后需重新编译和烧录；串口运行时设置仍只更新原先的 RAM 状态。带 `#ifndef` 的构建开关继续允许测试或原工程的 `-D` 参数覆盖，不能随意去掉这些包装。算法内的 0/1、数学系数和状态枚举不全部参数化，避免把程序语义误当调参项。

## 调用链和约束

`main → freertos → AppTasks_Create` 创建遥控、CAN 接收、控制、控制台四个任务。遥控输入先形成有效帧，再依次进入通道映射、安全判定及目标管理；控制任务调用各 Backend，Backend 经过会话、健康和许可检查后才调用控制器并提交 CAN 命令。

- 控制周期仍为 2 ms，遥控轮询 2 ms，控制台和波形调度 5 ms；没有调整任务优先级、PID 数值或滤波参数。
- Yaw 力矩提交仍为 10 ms，Pitch 和拨盘仍为 2 ms。减少文件数本身不会改变这些节拍。
- Pitch 仍使用 `YawCascade` 共用数学接口和 `YawStep` 统计核心。它们放在 `yaw_controller.c`，不能因名称含 Yaw 就当作 Pitch 无关代码删除。
- 安全判定、许可和输出提交仍在原临界区。重构没有取消故障撤权、停机、重新解锁或反馈超时检查。
- 归位可中断当前动作，普通正式阶跃保留原门禁；CH7 上拨是“当前目标减 30°”，`YAW HOME` / `PITCH HOME` 才是绝对回零。
- VIEW 的另一轴保持逻辑、串口命令、组名、通道顺序和测试判据保持原样。
- 演示数据继续有独立组名、实测角度和状态标记；不得用于物理闭环或替代真实成绩。

## 编译和验证

原 `MDK-ARM/Gimbal_Control.code-workspace` 及 `.eide/eide.yml` 已指向这 10 个文件，Keil `.uvprojx` 也已同步。继续使用原 VS Code 工程编译、烧录，不要重新添加已合并的旧 C 文件。

测试夹具仍按逻辑边界提供 HAL/RTOS 模拟。`Tests/source_layout.py` 从当前正式文件的分区读取所需代码，`Tests/app_source_map.json` 记录对应关系；测试报告哈希追踪完整的正式文件。完整固件构建则验证这 10 个文件合并编译后的链接。

整理前备份、前后回归、预处理语句等价检查和编译证据见 `../../../02_过程记录/App十模块整理_20261025/`。硬件阶跃性能和实机时序仍需要烧录后确认。
