# Gimbal_Control

本工程实现云台调试考核：遥控保护、Yaw/Pitch 30°阶跃、双摩擦轮 6000 rpm、拨盘单发与 20 Hz 连发请求、Pitch 重力前馈、串口调参与真实成绩统计。全部功能保留在同一个 FreeRTOS 工程中。

归档日期为 **2026-10-29**，考核完成状态依据作者反馈。此文描述当前源码快照，旧过程记录按各自日期解读；本次整理不新增实机性能结论。

复盘先看[工程思维导图与答辩复习](云台工程代码思维导图与答辩复习.md)，再按[App 代码导航](App/README.md)沿数据流读源码；答辩清单见[答辩准备](答辩准备_2026-10-29.md)。

## 当前配置

- 当前为拨盘独立测试：`APP_FEEDER_STANDALONE_TEST_ENABLED=1`。摩擦轮动力输出关闭，拨盘不等待双轮稳速；轮速反馈仍如实显示。
- 修改该模式常量为 0，可恢复双摩擦轮输出与稳速互锁。其余电机和 DMA 接收不再提供编译裁剪开关。
- 两轴模拟曲线、SIM 命令和现有 VOFA 通道位置保留。模拟值仅用于显示，电机控制和 RESULT 始终读取真实反馈。
- 参数集中在 `App/app_config.h`：整数使用枚举常量，PID/角度/力矩使用 `static const float`。串口调参只修改当前 RAM，会在重启后恢复源码默认值。

| 项目 | Yaw | Pitch |
| --- | --- | --- |
| 电机 | CAN1 ID1，GL60 II | CAN1 ID2，DM3507 |
| 位置 PID | 13.768 / 3.214 / 0.139 | 11.500 / 24.775 / 0.002 |
| 速度 PID | 0.00504 / 0.0008 / 0.000074 | 0.00408 / 0.0005 / 0.000001 |
| 命令提交周期 | 10 ms | 2 ms |
| 目标角度范围 | ±150° | −18°～+33° |
| 实测硬边界 | ±160° | −19°～+34° |
| 其他 | 直接目标、共用串级 PID | 重力 Kg=−0.13、直接目标、末段与回退制动 |

控制任务周期 2 ms；UART4 iBUS 为 115200，USART1 为 921600/8N1，遥测配置间隔 5 ms。手动满杆使目标按 90°/s 变化，不等于电机阶跃运动的速度上限。

## 代码阅读

`ControlTask → AppLogic_Step → 各 Backend_Apply → Controller_Update → GimbalPid_UpdateWithFeedforward → CAN`。

`App/control/gimbal_pid.c/.h` 是两轴共用 PID，已替代原 `yaw_cascade.c/.h`；类型和接口统一使用 `GimbalPid`。目录导航见 `App/README.md`。

每个 App `.c` 以主流程/阶跃/控制函数开头，内部在线判断、校验和错误处理放后面；静态函数的前置声明只解决调用顺序，不执行任何逻辑。文件头列出职责、函数和文件级变量，各函数参数列表后解释作用、参数/单位、返回值，关键函数补充处理步骤。

## 遥控与调试

保持原遥控映射：CH1/2 手动双轴，CH5 OFF/手动/测试，CH6 关闭/单发/连发，CH7 阶跃确认或相对回退，CH8 释放/扳机及安全锁，CH3/4 选择测试轴。

上电时在确认过的水平机械基准、安全 OFF、摇杆回中条件下完成自动准备，再通过新的 CH5 解锁操作运行。故障恢复仍需新的 OFF 帧及人工解锁。遥控失联、基本行程边界、设备反馈时效和 CAN 停机处理保留。

常用命令：

```text
GIMBAL
GIMBAL READY
YAW PID
PITCH PID
YAW VIEW
PITCH VIEW
GIMBAL VIEW FREE
YAW STEP
PITCH STEP
YAW HOME
PITCH HOME
YAW RESULT
PITCH RESULT
FEEDER MODE
FEEDER ANGLE VIEW
FEEDER RESULT
WHEEL VIEW
TELEM OFF
TELEM ON
STOP
```

`YAW VIEW` 锁 Pitch，`PITCH VIEW` 锁 Yaw；释放互锁用 `GIMBAL VIEW FREE`。Yaw STEP 为当前目标相对 ±30°；Pitch STEP 仍要求近软件零点、静止且不在测量中。Pitch −30°超出现有限位，会被拒绝。HOME 是软件回零，CH7 低值档是当前目标减 30°。

实测展示前使用 `YAW SIM OFF`、`PITCH SIM OFF`；保留 `YAW SIM`、`PITCH SIM` 查询。开机默认开启演示显示，重启后需要重新核对。

## 测量与编译

真实阶跃窗口为 5 s，末 1 s 按时间加权统计稳态误差；正式 PASS 同时检查最终稳定时间、超调和有效稳态误差。首次入带不等于最终稳定。可选全程轨迹整形、落点诊断快照、全局停机追踪和旧逐字节接收兼容已移除，原 VOFA 保留列使用固定值。

继续使用 `MDK-ARM/Gimbal_Control.code-workspace` 或 Keil 工程；AC5/O0 配置保持。`Scripts/build.ps1` 仅完整编译，不连接串口、不烧录。

离线验证入口为 `Tests/run_selftests.py`，说明见 `Tests/README.md`。完整软件构建和离线通过不能代替真实电机阶跃与射频验收。

2026-10-29 归档时已重新完成 AC5 完整构建及全部离线回归，均通过；当前离线证据见 [Tests/selftest_results.json](Tests/selftest_results.json)，具体源码哈希、检查项和结果以报告为准。

精简前源码、配置、旧说明和回归基线备份位于 `MDK-ARM/build/refactor_20261028/before.zip`；该目录仅供本次审阅，不参与固件编译。
