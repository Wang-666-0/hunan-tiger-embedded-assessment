# MPU6050 IMU 读取与互补滤波学习笔记

姓名：王世伟. 日期：2026-10-05.

根据本次 CubeMX 配置、VS Code 编译烧录、Git 分支练习与 VOFA 验证整理，由 Codex 辅助编写.

## 1. 考核完成情况

| 要求 | 实际完成情况 |
| --- | --- |
| 自备 MPU6050 和最小系统板 | 已接线并通信成功 |
| IIC 读取原始数据 | 已读取身份寄存器和原始六轴数据 |
| 使用 DMP 库获得六轴数据 | 已完成 HAL 移植并上板输出六轴数据 |
| 互补滤波解算姿态，加分 | 已实现 Roll、Pitch，并验证倾斜响应 |

当前 main.c 打印的是互补滤波 Roll、Pitch. 原始六轴和 DMP 六轴输出是此前阶段验证结果，不是三个程序同时打印. 当前滤波适合缓慢、小角度倾斜实验，没有实现绝对航向 Yaw.

## 2. 硬件与 CubeMX 配置

STM32F103C8T6，外部晶振 8 MHz，PLL ×9，系统时钟 72 MHz. 本工程没有 FreeRTOS，HAL 毫秒计时使用 SysTick.

| 连接/配置 | 本次设置 |
| --- | --- |
| MPU6050 SCL | PB6，I2C1_SCL |
| MPU6050 SDA | PB7，I2C1_SDA |
| I2C 模式 | I2C，Standard Mode，100000 Hz |
| 地址 | 7 位地址 0x68，AD0 接地 |
| 串口 | USART1，PA9 TX、PA10 RX，115200，8N1，无流控 |
| 电脑连接 | CH340，TX/RX 交叉连接，共地；本次端口 COM10 |
| 上位机 | VOFA RawData，接收带标签的文本 |

图片重点：PB6/PB7 功能和 100 kHz 参数，而不是其他未使用的外设.

图 01 · I2C1引脚与参数配置：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/01_I2C1%E5%BC%95%E8%84%9A%E4%B8%8E%E5%8F%82%E6%95%B0%E9%85%8D%E7%BD%AE.png).

## 3. 先读取 WHO_AM_I，确认通信

```c
HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
    &hi2c1, 0x68 << 1, 0x75,
    I2C_MEMADD_SIZE_8BIT, &mpu_id, 1, 100
);
```

| 参数 | 意义 |
| --- | --- |
| &hi2c1 | 使用 I2C1 句柄 |
| 0x68 << 1 | HAL 要求左移一位的设备地址，不是寄存器地址 |
| 0x75 | WHO_AM_I 身份寄存器 |
| I2C_MEMADD_SIZE_8BIT | 寄存器地址占 8 位，不是六轴数据位宽 |
| &mpu_id | 接收数据的变量地址 |
| 1 | 读取一个字节；身份寄存器只占一字节 |
| 100 | 阻塞操作超时时间 100 ms |

返回值是通信状态，读取内容写入 mpu_id. HAL_OK=0，HAL_ERROR=1，HAL_BUSY=2，HAL_TIMEOUT=3. 身份读取成功时，本次输出 status=0、ID=0x68. 初次曾出现 BUSY，后来连续成功；单次 BUSY 不能解释为传感器身份错误.

图 02 · WHO_AM_I通信验证：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/02_WHO_AM_I%E9%80%9A%E4%BF%A1%E9%AA%8C%E8%AF%81.png).

## 4. 原始六轴读取与内部低通滤波

最初手动初始化配置如下，后来切换 DMP 时删除了这段应用层初始化，由 DMP 库管理配置.

| 寄存器 | 写入值 | 作用 |
| --- | --- | --- |
| 0x6B，电源管理 | 0x01 | 唤醒，选 X 轴陀螺仪 PLL 时钟 |
| 0x1A，CONFIG | 0x03 | DLPF_CFG=3，陀螺仪带宽约 42 Hz，加速度约 44 Hz |
| 0x19，采样分频 | 9 | 此配置下 1000/(1+9)=100 Hz |
| 0x1B，陀螺仪量程 | 0x00 | ±250 °/s，灵敏度 131 LSB/(°/s) |
| 0x1C，加速度量程 | 0x00 | ±2 g，灵敏度 16384 LSB/g |

低通滤波保留较慢变化、衰减较快变化，适合减轻传感器高频噪声和振动影响，同时会带来延迟. 它不等于静止零偏校准，也不能把固定偏差自动归零.

从 0x3B 连续读取 14 字节：

| 数组位置 | 数据 |
| --- | --- |
| 0、1 / 2、3 / 4、5 | 加速度 X / Y / Z |
| 6、7 | 温度，本次跳过 |
| 8、9 / 10、11 / 12、13 | 陀螺仪 X / Y / Z |

高字节在前，组合成有符号 16 位数：

```c
ax = (int16_t)(((uint16_t)mpu_data[0] << 8) | mpu_data[1]);
```

左移高字节、按位或拼接低字节，再转换为 int16_t 保留正负方向. 原始阶段加速度除以 16384.0f 得到 g，角速度除以 131.0f 得到 °/s. 传感器采样 100 Hz，但旧循环 HAL_Delay(100) 的显示读取约 10 Hz，两者不是同一频率.

图 03 · I2C原始六轴数据：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/03_I2C%E5%8E%9F%E5%A7%8B%E5%85%AD%E8%BD%B4%E6%95%B0%E6%8D%AE.png).

## 5. 静止陀螺仪零偏校准

最初静止时角速度约为 -4.1、0.7、-0.2 °/s. 理想静止角速度是零，多次平均可以估计固定偏差.

1. 初始化后保持静止，采集 200 次陀螺仪原始值，每次间隔约 10 ms.
2. 使用 int32_t 累加，按有效采样数求浮点平均.
3. 后续输出先减去原始计数偏差，再除以灵敏度.

```c
gx_dps = ((float)gx - gx_bias) / 131.0f;
```

校准必须静止，但不要求水平. 不能用这种方法将静止加速度的三轴都减成零，因为其中包含重力. 校准后本次角速度接近零，仍有少量噪声；温度变化会导致偏差变化.

这段手动校准属于原始读取阶段. 当前 DMP 阶段启用了 DMP_GYRO_CAL，没有继续执行原来的 200 次应用层校准.

图 05 · 陀螺仪静止零偏校准：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/05_%E9%99%80%E8%9E%BA%E4%BB%AA%E9%9D%99%E6%AD%A2%E9%9B%B6%E5%81%8F%E6%A0%A1%E5%87%86.png).

## 6. DMP 库的移植

库来自桌面的“源码资料/Mpu6050”，原版是 STM32F1 标准外设库加软件 I2C，原引脚 PB10/PB11，不能直接放入 HAL 工程使用.

移植到工程的 Mpu6050 目录：保留 inv_mpu、inv_mpu_dmp_motion_driver 和 DMP 固件数组等核心文件；底层通信改用 hi2c1 的 HAL_I2C_Mem_Read/Write，延时改用 HAL_Delay，时间查询改用 HAL_GetTick. 没有导入旧 system 和软件 I2C 驱动.

同时修正初始化失败仍返回成功、get_ms 空实现和 FIFO 时间戳未赋值的问题. 时间戳是主机读取时间，不能当作精确采样时间. 增加初始化步骤错误码和角度计算保护，更新 EIDE、Keil 的源码及头文件配置.

移植时曾强制链接 DMP 初始化与读取入口，避免 main.c 未调用导致链接器裁剪后掩盖缺失依赖. 当时检查固件 ROM 26492 字节，静态 RAM 含配置的栈堆 2072 字节，分别小于工程配置的 64 KB、20 KB. 此数字属于当时移植检查版本，不代表当前增加滤波后的最终容量，也不证明运行时栈峰值.

Flash 保存代码、只读常量和初始化数据；RAM 保存可变数据、零初始化数据、栈与堆. 初始化的全局变量通常同时占用 Flash 初值和 RAM 运行空间.

## 7. DMP 六轴数据的完整流程

```text
MX_I2C1_Init → MPU6050_DMP_Init
→ MPU6050 内部采样和 DMP 处理
→ 数据包进入 FIFO
→ dmp_read_fifo 读取并拆包
→ 检查有效标志 → 单位换算 → 串口输出
```

| 重要函数 | 作用 |
| --- | --- |
| mpu_init | 芯片基础初始化 |
| mpu_set_sensors | 开启加速度计和陀螺仪 |
| mpu_configure_fifo | 配置传感器 FIFO 数据路径 |
| mpu_set_sample_rate | 设置传感器采样率 |
| dmp_load_motion_driver_firmware | 将 DMP 固件经 I2C 装载到 MPU6050 |
| dmp_set_orientation | 设置坐标轴对应关系 |
| dmp_enable_feature | 选择四元数、原始加速度、校准后角速度等功能 |
| dmp_set_fifo_rate | 本次设置 DMP 输出 100 Hz |
| mpu_set_dmp_state(1) | 启动 DMP |
| mpu_get_gyro_sens / mpu_get_accel_sens | 获取当前量程换算系数 |
| dmp_read_fifo | 读取一包加速度、角速度、四元数和数据标记 |

初始化入口 MPU6050_DMP_Init 返回 0 才成功；1~9 表示对应初始化步骤失败. DMP 算法在 MPU6050 内部执行，STM32 负责配置与读结果.

```c
dmp_read_fifo(dmp_gyro, dmp_accel, dmp_quat,
              &dmp_timestamp, &dmp_sensors, &dmp_more);
```

数组分别保存三轴角速度、三轴加速度、四元数；dmp_sensors 是有效数据标志，dmp_more 是剩余数据包数量. 返回非零时本轮退出读取，主循环下一轮继续；它可能表示暂时无数据，也可能表示读取错误.

### 有效数据标志的判断

```c
if ((dmp_sensors & INV_XYZ_ACCEL) &&
    ((dmp_sensors & INV_XYZ_GYRO) == INV_XYZ_GYRO))
```

| 标志 | 二进制 |
| --- | --- |
| INV_X_GYRO=0x40 | 0100 0000 |
| INV_Y_GYRO=0x20 | 0010 0000 |
| INV_Z_GYRO=0x10 | 0001 0000 |
| INV_XYZ_GYRO=0x70 | 0111 0000 |
| INV_XYZ_ACCEL=0x08 | 0000 1000 |

& 是按位与，提取指定标志；&& 是逻辑与，要求两边条件都成立. 加速度是一个整体标志，非零就通过. 角速度是三个标志的组合，必须与 0x70 相等才能确认三轴全部存在，只判断非零只能确认至少一个轴.

### 换算和读取频率

```c
ax_g = (float)dmp_accel[0] / accel_sensitivity;
gx_dps = (float)dmp_gyro[0] / gyro_sensitivity;
```

(float) 避免加速度整数除法丢失小数. 当前 DMP 默认加速度 ±2 g、陀螺仪 ±2000 °/s，驱动灵敏度分别为 16384 和 16.4. 不再使用旧阶段的 131.

DMP 每 10 ms 产生一包. do...while(dmp_more != 0) 及时读完积压数据，串口只每 100 ms 打印一次. 不能每 100 ms 只读一包，否则 FIFO 会积压. 当前循环未读到新包时仍可能打印旧值，所以需要观察转动时读数变化，不能只凭重复输出判断持续采样成功.

图 06 · DMP六轴输出验证：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/06_DMP%E5%85%AD%E8%BD%B4%E8%BE%93%E5%87%BA%E9%AA%8C%E8%AF%81.png).

## 8. 互补滤波解算 Roll 和 Pitch

### 为什么既用加速度，也用角速度

角速度积分能跟踪角度变化，但不知道初始姿态，固定角速度误差也会不断累积. 例如偏差 0.1 °/s，积分一分钟误差可达 6°.

静止时加速度计的读数主要由重力方向决定，可以直接计算倾斜角，不依赖历史积分；运动加速度和振动则会干扰它. 所以陀螺仪跟踪短时变化，加速度提供长期倾斜参考.

模块正面朝上且只绕 X 轴倾斜 θ 时：ay≈sinθ g，az≈cosθ g. 倾斜 30° 对应 ay≈0.5 g、az≈0.866 g，atan2(ay,az) 反算得到 30°. 具体正负号取决于轴向和倾斜方向.

```c
roll_acc_deg = atan2f(ay_g, az_g) * 57.2957795f;
pitch_acc_deg = atan2f(-ax_g,
    sqrtf(ay_g * ay_g + az_g * az_g)) * 57.2957795f;
```

atan2f 根据两个分量计算弧度并区分象限，sqrtf 求 YZ 平面合成分量，180/π≈57.2957795 将弧度转换为度.

### 初值与逐包更新

第一包有效数据直接使用加速度角度初始化 roll_deg、pitch_deg，并将 attitude_ready 设为 1. 这不是上电归零，而是建立实际测得的倾斜起点.

```c
const float dt = 1.0f / DEFAULT_MPU_HZ; /* 0.01 秒/包 */
const float alpha = 0.98f;
roll_deg = alpha * (roll_deg + gx_dps * dt)
         + (1.0f - alpha) * roll_acc_deg;
pitch_deg = alpha * (pitch_deg + gy_dps * dt)
          + (1.0f - alpha) * pitch_acc_deg;
```

上次角度+角速度×时间，是陀螺仪预测. 每包将其以 98% 权重和加速度角度的 2% 权重融合. 修正持续执行，不能理解成只修正一次. alpha 越大越依赖陀螺仪，越小越容易受到运动加速度影响；0.98 是本次 100 Hz 下的入门参数.

每次成功读取有效六轴数据后计算滤波，打印分支只显示结果. FIFO 连续读积压包时使用每包 0.01 秒，不能拿主机两次紧邻读取的时间差代替.

### 验证与限制

用户已验证倾斜可以检测. 截图展示角度变化，但不是与标准角度仪比较的精度测试. 缓慢单轴倾斜约 ±30°，停住后观察稳定，回到原姿态后观察恢复.

图 07 · 互补滤波倾斜响应：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/07_%E4%BA%92%E8%A1%A5%E6%BB%A4%E6%B3%A2%E5%80%BE%E6%96%9C%E5%93%8D%E5%BA%94.png).

静止截图 Roll 约 -2.19°，Pitch 约 -4.47°，短期波动较小，更像固定零点偏差，而不能仅凭截图判断长期漂移. 可能来自实际安装倾斜、加速度计偏差等，现有资料不能分离各原因.

| 现象 | 区别 |
| --- | --- |
| 固定偏差 | 静止稳定在某个非零角度 |
| 漂移 | 静止角度随时间持续偏离 |
| 噪声 | 在一个数值附近来回波动 |

图 08 · 静止角度固定偏差：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/08_%E9%9D%99%E6%AD%A2%E8%A7%92%E5%BA%A6%E5%9B%BA%E5%AE%9A%E5%81%8F%E5%B7%AE.png).

若要将当前安装姿态定义为零，可静止多次采样求平均作为 offset，显示时 angle_show=angle-offset. 本次只讨论了方法，尚未加入该功能. 它不能消除真实漂移，也不等同于传感器校准.

当前算法直接用 gx、gy 近似欧拉角变化率，适合小角度缓慢运动；大角度多轴转动需考虑耦合. 快速平移会干扰加速度参考. 重力无法确定绕竖直方向的航向，因此没有实现绝对 Yaw.

## 9. VS Code 与 Git 实践

本次在 VS Code 的 EIDE 中完成编译、烧录，通过 VOFA 查看串口输出. 在“mpu读取数据处理”分支进行单位换算、零偏校准、DMP 移植等练习，并练习切回 main 合并分支.

截图中的 main 比 origin/main 超前，表示本地有尚未推送的提交；不代表合并失败. 当前整理笔记前已在 main，工作区干净，互补滤波代码已在当前 main 中.

图 04 · VSCode烧录与分支合并：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/09_IMU%E8%AF%BB%E5%8F%96%E4%BB%BB%E5%8A%A1/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/04_VSCode%E7%83%A7%E5%BD%95%E4%B8%8E%E5%88%86%E6%94%AF%E5%90%88%E5%B9%B6.png).

提交说明可以修改. 最新提交可用 git commit --amend -m "新的说明"，但暂存的改动也会并入该次提交. Amend 改变提交哈希；已推送的提交需协调重写历史，不能把普通改名当成无影响操作.

## 10. 现场验收顺序

1. 说明 I2C1 接线、设备地址与身份寄存器验证方法.
2. 展示原始数据读取与高低字节组合、量程换算.
3. 说明 DMP 库移植接口，展示六轴输出阶段的验证记录.
4. 运行当前程序，倾斜模块展示 Roll、Pitch；解释陀螺仪积分和加速度参考如何互补.
5. 说明固定零点偏差与漂移的区别，明确小角度实验和 Yaw 的边界.