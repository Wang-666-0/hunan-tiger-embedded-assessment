# App 模块说明

业务代码按 `.c/.h` 配对放在此目录。`main.c` 负责 HAL、时钟和外设初始化；`freertos.c` 保留 CubeMX 的任务创建和入口；传感器、算法、校准和协议分别封装，不在生成文件中堆放业务实现。

## 模块导航

| 文件 | 职责 | 重点阅读 |
| --- | --- | --- |
| bmi088_port.c/.h | SPI1、PA4/PC4 双片选、DWT 微秒时间与短延时 | 哪个片选选中哪部分传感器；超时后如何释放片选 |
| bmi088.c/.h | 启动、软复位、芯片 ID、寄存器配置/检查、六轴采样 | 加速度计 SPI dummy byte、量程灵敏度、rad/s 与 m/s² |
| imu_calibration.c/.h | 静止条件、连续采样、陀螺仪零偏和初始重力平均 | 运动重置累计；不能从加速度中减掉重力 |
| mahony.c/.h | 六轴 Mahony 状态、实际 dt 更新、重力初始化、欧拉角提取 | 四元数只初始化一次；反馈符号；归一化和奇异点保护 |
| imu_app.c/.h | FreeRTOS 业务入口和 5 ms 周期 | 初始化/校准/运行/重试流程；不发布故障数据 |
| vofa_telemetry.c/.h | 三通道 JustFloat 打包、USART1 发送 | 通道顺序、16 字节帧、固定帧尾和有限超时 |

重要代码行的中文注释说明“为什么这样设置”和“单位是什么”，避免只重复函数名。纯算法/校准部分不依赖 HAL；板级相关代码集中到 port 和 telemetry 模块。

## 如何调用

应用已在 `ImuApp_Run()` 中完成以下串接，不需要再把参考源码的 `MahonyAHRSupdateIMU()` 调用粘贴进任务：

| 本工程接口 | 用法 |
| --- | --- |
| BMI088_Init() | 初始化并验证本板传感器；成功才开始校准 |
| BMI088_Read(&sample) | 取得一次完整样本；gyro_rad_s 与 accel_m_s2 明示单位 |
| ImuCalibration_Reset / Push / GetResult | 持续收集静止窗口，得到三轴 bias 和平均 gravity |
| Mahony_Init(&attitude, gravity) | 初始化一次姿态与反馈状态 |
| Mahony_UpdateIMU(&attitude, gyro, accel, dt_s) | 每个有效周期更新四元数，gyro 已扣除 bias |
| Mahony_GetEulerDegrees(&attitude, euler_deg) | 输出顺序固定为 Roll/Pitch/Yaw，单位度 |
| VOFA_SendEuler(euler_deg) | 打包并发送当前周期的新角度 |

业务入口使用 `ImuApp_Run()`；不要在 `main` 调度器启动前直接调用它，因为传感器初始化和任务等待依赖 CMSIS-RTOS2。

## 调用关系

```text
main：HAL + Clock + GPIO + SPI1 + USART1 → 启动 FreeRTOS
freertos：StartImuTask → imu_app
                          ├─ bmi088 → bmi088_port → HAL SPI/GPIO/DWT
                          ├─ imu_calibration
                          ├─ mahony
                          └─ vofa_telemetry → HAL USART1
```

所有 BMI088 和 USART1 业务由同一任务拥有，当前不需要跨任务互斥锁。以后若其他任务共用 SPI1/USART1，必须增加总线/串口管理，不可直接并发调用。

## 时序与单位

| 项目 | 约定 |
| --- | --- |
| 任务周期 | 5 ms / 200 Hz，固定目标时刻等待，不在处理结束后简单再睡 5 ms |
| HAL / RTOS 时基 | HAL TIM2；RTOS SysTick 1000 Hz |
| 算法时间增量 | 实际经过的秒数；不得用毫秒直接代替秒 |
| 加速度 | m/s²，保留重力分量 |
| 角速度 | rad/s，先减静止零偏 |
| 四元数 | w、x、y、z，无量纲，更新后归一化 |
| 欧拉角 | 右手坐标的 ZYX 提取，输出 Roll/Pitch/Yaw，单位为度 |
| USART1 | 115200、8N1，三通道每 5 ms 一帧 |

不是每次循环重新初始化姿态；启动用平均重力方向确定 Roll/Pitch，并设 Yaw = 0。任务停止/重新校准后，相对 Yaw 参考会重新建立。

## JustFloat 帧

```text
偏移 0～3：Roll float
偏移 4～7：Pitch float
偏移 8～11：Yaw float
偏移 12～15：00 00 80 7F
```

不在帧内加入时间戳、温度、调试字符串或换行。本版没有上位机命令协议。VOFA 必须选 JustFloat；I0/I1/I2 对应 Roll/Pitch/Yaw。

## 异常处理与范围

初始化检查芯片型号和必要寄存器；故障后重试而不是假设读出的字节有效。校准需要连续静止，明显运动会重置累计。算法拒绝无效输入/不合理时间跨度，发布前检查角度有效性。异常过程中暂停发布新姿态，VOFA 可能保留上一个数值，所以“仍有数值”不等于采样仍在进行。

本工程不包含磁力计、温控闭环、电机/CAN 控制或 USB 协议。Yaw 会漂移；Pitch 接近 ±90° 时欧拉角表示会奇异。若要改变模型朝向，优先核对实际传感器坐标和 VOFA 模型偏置，不在驱动中盲目翻转单轴。

## 修改后的检查

1. 核对 `.h` 中的接口、单位与调用方一致。
2. 检查 SPI 模式、片选、传感器量程/灵敏度配对。
3. 检查 EIDE/Keil 包含所有 App 源文件和 `../App` 头文件路径。
4. 运行项目提供的软件测试，再完整构建。
5. 重新烧录，在真实板上验证静止、X/Y/Z 单轴、组合转动、复位和异常恢复。

若重新生成 CubeMX，保持接入调用在 `USER CODE` 中，并检查生成器没有移除 App 源文件组。软件测试状态由最终构建日志记录；实机状态记录在任务的 `02_过程记录`。
