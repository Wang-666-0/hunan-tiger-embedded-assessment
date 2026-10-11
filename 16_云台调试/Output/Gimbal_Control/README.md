# Gimbal_Control 工程预留目录

本目录用于后续建立一套完整的 STM32F405 / FreeRTOS 固件。当前阶段仅完成规划，没有可编译工程或可烧录 HEX。

## 计划结构

```text
Gimbal_Control/
├─ App/
│  ├─ Input/        # iBUS、输入快照和遥控逻辑映射
│  ├─ Safety/       # 统一状态机与输出门禁
│  ├─ Can/          # 双CAN、标准/扩展帧路由
│  ├─ Drivers/      # GL60、DM3507、VESC、C610协议
│  ├─ Estimation/   # 连续角度、BMI088和反馈融合
│  ├─ Control/      # 两轴、摩擦轮和拨盘控制
│  ├─ App/          # 云台/发射机构任务编排
│  └─ Telemetry/    # 遥测、阶跃记录和指标计算
├─ Core/            # CubeMX生成的初始化与中断入口
├─ Drivers/         # STM32 HAL/CMSIS
├─ Middlewares/     # FreeRTOS
├─ Tests/           # 协议编解码、状态机和控制逻辑主机测试
└─ MDK-ARM/         # Keil/EIDE工程
```

## 可复用边界

- 从 `15_遥控使用` 复用 `ibus`、带时间戳的 `remote_input` 和保护状态机原则。
- 从 `13_PID学习与实践` 复用不依赖 HAL 的 PID 数学与阶跃记录思路。
- 从 `14_板载IMU读取` 复用 BMI088 驱动、校准和 Mahony 数学模块。
- 重新实现双 CAN 传输与四类设备协议。旧 `can_transport` 只支持 CAN1 标准帧、DLC8，不能直接用于 VESC 扩展帧或本任务混合总线。

工程建立时先完成保护与协议的最小版本，按 `02_过程记录/实机验收记录.md` 逐阶段开启动力功能。
