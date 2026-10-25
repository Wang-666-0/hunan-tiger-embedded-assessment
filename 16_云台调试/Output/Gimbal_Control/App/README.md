# App 维护导航

当前按职责分为五组；上电参数与常量统一在 `app_config.h`。每个实现直接参与编译，对应头文件放在同一目录。每个函数体的 `{` 前有中文功能说明；每个 C 文件头列出全部函数、文件级变量及其用途。

| 目录 | 内容 | 从哪里开始 |
| --- | --- | --- |
| system | 任务、启动、遥控业务和安全状态机 | app_tasks → app_logic → safety_manager / target_manager |
| drivers | CAN、UART4/iBUS 和电机协议 | can_bus、remote_input、ibus、gl60_ii、dm3507 |
| control | 两轴共用 PID 和真实阶跃统计 | yaw_cascade、yaw_step、pitch_step |
| motors | 各电机控制器与执行后端 | yaw / pitch / feeder 的 controller、backend；flywheel_speed / flywheel_jog / motor_backend |
| console | 系统/电机命令、VOFA 和显示演示 | app_console 分发给 yaw_console、pitch_console、feeder_console |

## 运行链

`main → freertos → AppTasks_Create` 创建遥控、CAN、控制和控制台任务。遥控有效帧经通道映射、安全判定、目标管理后，由各 Backend 执行。控制器负责算法，Backend 负责使能/停止、反馈时效、请求接管和 CAN 提交。

`YawCascade` 与 `YawStep` 是共用接口：Pitch 也依赖它们。名称继续兼容现用 API，实际实现已放入 control。控制器状态按电机分别保存。

## 修改参数

打开 `app_config.h`，按分区或名称搜索：

- Yaw：`APP_YAW_POSITION_` / `APP_YAW_SPEED_`。
- Pitch：`APP_PITCH_POSITION_` / `APP_PITCH_SPEED_`、`GRAVITY`、`BRAKE`。
- 拨盘：`APP_FEEDER_`；每发 40°，连发每 50 ms 推进目标，控制周期 2 ms。
- 摩擦轮：`APP_FLYWHEEL_`；保留现用速度 PI、电流上限、反馈/温度/超速保护。
- 通信/调度：`PERIOD_MS`、`BAUDRATE`、`STACK_WORDS`、`PRIORITY`；遥控通道为 `RC_`。

MS=毫秒，DEG=度，DPS=度/秒，NM=牛·米。ERPM 和机械 RPM 不同；C610 电流原始值也不是 mA。编译默认值在重新烧录后生效，串口调参只改变本次 RAM 会话。

## 当前拨盘独立测试

`APP_FEEDER_STANDALONE_TEST_ENABLED=1`：摩擦轮动力输出关闭，拨盘单发/连发不等待两侧 VESC 就绪；真实摩擦轮状态继续原样报告。电调装回后改为 0，重新编译，恢复正常双轮稳速互锁。

1. CH5/CH6 置 OFF，CH8 释放、摇杆回中，等待云台/拨盘原安全准备完成；必要时发送 `FEEDER READY`。
2. CH5 中档正常模式，CH6 中档单发或高档连发；选择模式后，CH8 先释放再扣动。
3. 连发保持扣动，松开后按原逻辑完成已接受的尾步；不会排队补发。
4. `FEEDER MODE` 查配置，`FEEDER ANGLE VIEW` 看实际/目标角度 I0/I1；反号显示保留。`FEEDER RESULT`、`FEEDER CHECK` 查真实结果和故障。

## 现用命令与显示

`YAW VIEW` 锁定 Pitch；`PITCH VIEW` 锁定 Yaw。`WHEEL VIEW`、`FEEDER ANGLE VIEW`、`FEEDER SPEED VIEW` 和各电机调参/状态/成绩命令保留。`GIMBAL VIEW FREE` 释放视图锁定。CH7 上拨仍为当前轴目标回退 30°；绝对回零使用 `YAW HOME` / `PITCH HOME`。

现有显示演示放在 console：`YAW SIM ON/OFF`、`PITCH SIM ON/OFF`，带独立演示组名、真实角度和状态标记。控制器与 RESULT 只读取真实反馈。保留当前 VOFA 通道位置；Yaw 旧实验参数所在列保持固定/停用值。

## 已归档的功能

旧遥控映射、摩擦轮定时低功率/开环观察、Yaw 单环和实验轨迹/加速度前馈已从正式实现清理。初期 `DIAG ON/OFF`、通用 `STATUS`、`READ DM`、`READ VESC` 和被动 CAN 抓包已移除。当前 `YAW/PITCH STATUS`、`BUS`、`GUARD`、`CHECK` 等运行信息查询保留。

PID 数值、重力与刹车参数、运行保护、归位、锁轴、单发/连发和各真实成绩判据维持原配置。Pitch 可选 SHAPE 与回退详细快照保留。

## 编译、测试与维护

继续打开原 `MDK-ARM/Gimbal_Control.code-workspace`，使用 EIDE 编译、烧录；EIDE 与 Keil 已按五个目录同步源文件和包含路径。辅助构建脚本与 VS Code 均使用原 AC5/O0 配置。普通构建直接编译真实文件。

离线测试通过 `Tests/runtime_groups.json` 直接选择正式源文件，不再抽取大文件中的注释分区。修改后运行相应组，并按变更影响进行整机编译。文件开头说明作用和入口，函数旁说明单位、状态约束与原因；历史调参过程集中归档。

本轮完整备份、旧用例、审阅与验证记录位于工程根目录的 `02_过程记录/工程精简审阅_20261025/`。本轮不连接串口、不烧录。
