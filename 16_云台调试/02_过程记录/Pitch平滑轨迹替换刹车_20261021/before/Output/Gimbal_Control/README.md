# Gimbal_Control

当前控制配置：Yaw `GL60II_FIXED_PID_V2Q`、Pitch `DM3507_RETURN_BRAKE_V1K`、拨盘 `C610_FEEDBACK_TIME_V1F`。Pitch保持用户最新七参，回退刹车增益0.035、上限0.45协议Nm、建立斜率35协议Nm/s，默认开启；使用与通道见[Pitch回退刹车](../../01_学习笔记/Pitch到位回退刹车_20261020.md)。2026-10-19框架精简与下文Yaw截图说明保留。

2026-10-20 按用户截图及确认，仅固化已达标的 Yaw 六个 PID 为上电默认值：位置 15.549 / 0.734 / 0.139，速度 0.00517 / 0.0035 / 0.000074。截图这一轮为最终稳定 168 ms、超调 0.130461°、稳态误差 −0.0965366°、I62=1；用户确认符合预期。仍使用原双环 PID，实验规划/前馈关闭，沿用现有面板。来源、适用范围和重新构建/烧录后的回读方法见 [六参数固化记录](../../01_学习笔记/Yaw达标六参数固化_20261020.md)。

## 工程与框架

- VS Code/EIDE 入口：`MDK-ARM/Gimbal_Control.code-workspace`，目标 `Gimbal_Control`，ARM Compiler 5，当前 EIDE 配置为 O0。
- Keil 入口：`MDK-ARM/Gimbal_Control.uvprojx`；CubeMX 入口：`Gimbal_Control.ioc`。
- App 保持平铺模块；32 个 App C 文件均加入工程，工程共 69 个源项。模块依赖见 [App 说明](App/README.md)。
- 已有遥控、安全保护、双摩擦轮 6000 rpm、拨盘、双轴手动控制、自动准备、串口调参和阶跃统计均保留。

## 双轴控制与阶跃

| 项目 | Yaw | Pitch |
| --- | --- | --- |
| 驱动 | CAN1 ID1，GL60 II | CAN1 ID2，DM3507 |
| 位置 PID | 15.549 / 0.734 / 0.139 | 13.55 / 0.001 / 0.026 |
| 速度 PID | 0.00517 / 0.0035 / 0.000074 | 0.0062 / 0.1019 / 0.000050 |
| 前馈 | 生产关闭实验规划/前馈 | 重力系数 −0.48，需有效水平标定；独立回退阻尼默认开启 |
| 手动目标范围 | ±50° | −18°～+33° |
| 越界保护 | ±60° | −19°～+34° |
| 力矩提交周期 | 10 ms | 2 ms |
| 正式阶跃稳定时间上限 | 250 ms | 300 ms |

控制任务周期仍为 2 ms。双轴使用同一双环数学实现 `yaw_cascade`、同一测量核心 `yaw_step`，实例独立；Pitch 的包装层保留独立 300 ms 判定与重力参数快照。

阶跃按 CAN 实际接收时间统计：观察 5 s、末 1 s 时间加权稳态误差，误差带、超调和稳态误差上限均为 0.2°。PASS 要求最终进入误差带并保持到窗口结束，使用 Yaw 250 ms / Pitch 300 ms 时限；首次入带快不代表总成绩通过。

`STEP` 是相对软件零点的绝对 ±30°目标，需满足已准备、人工解锁、许可、近零静止和测量窗口门禁。Pitch −30°超出现有限位，会按原设计拒绝；不能通过改限位实现反向考核。

## 常用操作

继续使用原工程，在正确机械基准、安全 OFF 下完成双轴准备，再按原遥控流程用新的 CH5 OFF→ON 边沿解锁。正常关停重开保留参考；`GIMBAL READY` 是显式重建双轴基准的操作。完整启动条件可查询 `GIMBAL HELP`、`YAW HELP`、`PITCH HELP`。

| 目的 | 命令 |
| --- | --- |
| 状态 / 参数 / 成绩 | `YAW`、`YAW PID`、`YAW RESULT`；Pitch 同名命令 |
| 正向阶跃 / 回参考 | `YAW STEP`、`YAW HOME`；`PITCH STEP`、`PITCH HOME` |
| 两环调参 | `YAW POS kp ki kd`、`YAW VEL kp ki kd`；Pitch 同名命令 |
| 切换波形 | `YAW VIEW`、`PITCH VIEW`、`FEEDER VIEW` |
| 暂停 / 恢复波形 | `TELEM OFF`、`TELEM ON` |
| 统一停机 | `STOP` |
| 只读诊断 | `REMOTE INFO`、`REMOTE RX`、`WHEEL STATUS`、`WHEEL SPEED`、`YAW BUS` |

遥控仍为 CH1/CH2 双轴手动，CH5 总模式，CH6 摩擦轮/射击模式，CH7 阶跃确认/回参考，CH8 扳机与失联锁，CH3/CH4 选择 Pitch/Yaw 阶跃。CH5 测试模式禁止摩擦轮和拨盘。

USART1 保持 921600、5 ms 遥测周期：Yaw 72 列、Pitch 60 列、拨盘 39 列。Pitch原I0～I57保持，追加I58刹车力矩、I59首次到位锁存。Yaw 考核看 I60 首次入带、I61 最终稳定时间、I62 综合通过；Pitch 看 I26、I27、I32。

## 构建与验证

在原工作区执行“eide: 重新构建”，成功后按原 ST-Link 流程烧录。当前配置和构建路径不变，未另行交付 HEX。

`Tests/run_selftests.py` 使用 ARM GCC 与 Unicorn 执行真实 C 代码、模拟 HAL/RTOS，无硬件输出；省略 `--groups` 执行完整回归。2026-10-19 框架精简的验证结果和与改前基线的对比见 [框架精简记录](../../02_过程记录/框架等价精简_20261019/实现与验证.md)。2026-10-20 六参数固化的测试与构建结果尚待验证，不能沿用历史结果视为本次通过。

注意：`Scripts/build.ps1` 自行指定 O3，不能把它的结果当作当前 EIDE O0 构建。2026-10-19 的框架验证直接使用 EIDE 的 O0 builder 参数。

## 历史资料

原说明包含历次参数、通道和实现过程，其中“尚未接入 Pitch/拨盘”等是历史状态。精简前全文保存在 [原工程说明](../../02_过程记录/框架等价精简_20261019/before/Output/Gimbal_Control/README.md)，已有学习笔记、过程记录和测试均保留。实际当前配置以 `App/app_config.h` 和控制器头文件为准。
