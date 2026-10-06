# IMU 读取：模块阅读与验收

## 调用顺序

`main.c → ImuApp_Init → ImuReader_Init`；主循环是 `ImuApp_Process → ImuReader_Process → AttitudeFilter_Update → ImuTelemetry_Process`。

| 文件 / 函数 | 职责与参数 |
| --- | --- |
| imu_app.c / ImuApp_Init(void) | main 完成外设初始化后调用一次 |
| imu_app.c / ImuApp_Process(void) | 协调读取、输出和原有 1 ms 延时 |
| imu_reader.c / ImuReader_Init(void) | 调用原 DMP 初始化，读取加速度及陀螺仪灵敏度 |
| imu_reader.c / ImuReader_Process(void) | 循环读取 FIFO 待处理包，校验数据标志，换算 g 和 deg/s |
| attitude_filter.c / AttitudeFilter_Update(ax_g, ay_g, az_g, gx_dps, gy_dps, dt) | 三轴加速度单位 g，X/Y 角速度单位 deg/s，采样间隔 dt 单位 s；更新 roll/pitch |
| imu_telemetry.c / ImuTelemetry_ReportInit(dmp_result) | 输出 DMP 初始化状态；dmp_result=0 表示初始化成功 |
| imu_telemetry.c / ImuTelemetry_Process(void) | 每 100 ms 最多输出一次 attitude:roll,pitch 数值帧 |

## 要看懂的部分

- FIFO 的 more 标志表示还有待处理包，不能只读一包就长期不处理后面的数据。
- 每个包的滤波 dt 是 `1/DEFAULT_MPU_HZ`，当前 100 Hz 对应 0.01 s，与 100 ms 打印间隔不同。
- 加速度通过重力方向求倾角；陀螺仪用角速度乘 dt 求角度增量。alpha=0.98，预测占 98%，加速度修正占 2%。
- 第一包直接建立初始角度；它是重力倾角初始化，并非桌面姿态归零。
- 此分支保留原来的两角互补滤波。读取的 DMP 四元数仍保存在 dmp_quat，但当前滤波没有使用四元数，也没有输出 yaw。
- `Mpu6050/` 内原来的驱动与 DMP 库已经分文件。App 负责应用流程，底层库继续负责传感器访问。

## 验收

VOFA+ FireWater 应出现两个姿态通道；倾斜传感器时 roll/pitch 随之变化。Keil 可继续观察 roll_deg、pitch_deg、dmp_accel、dmp_gyro、dmp_more 等原变量。算法复盘先看 attitude_filter.c，再看 imu_reader.c。
