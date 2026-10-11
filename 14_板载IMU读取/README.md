# 板载 IMU 读取考核

## 考核目标

- 使用实验室 STM32F405RGT6 云台板，通过硬件 SPI1 读取板载 BMI088。
- 基于 FreeRTOS 实现，沿用已有考核工程的 CMSIS_V2 接口，由 IMU 任务负责初始化、周期采样与串口输出。
- 完成 BMI088 初始化、三轴角速度、三轴加速度及温度读取，启动时进行静止零偏校准。
- 用六轴 Mahony 算法得到四元数和欧拉角，正常状态通过 USART1 每 5 ms 向 VOFA+ 输出 Roll/Pitch/Yaw，显示数值、曲线和 Cube 姿态。

本任务与 `09_IMU读取任务` 中的 MPU6050/I²C 考核分别保存。

## 当前进度

已生成 `Output/BMI088_Read` FreeRTOS 基础工程，使用 CMSIS_V2、TIM2 HAL 时基和 `imuTask`（Normal，512 words）。FreeRTOS 总堆为 10240 字节，时钟、SPI1、片选 GPIO 和 USART1 引脚已核对。

EIDE 导入后 IROM1/IRAM1 地址与容量为空、IRAM2 为零，导致 `L6406E/L6407E` 链接失败。已补齐 Flash 和 SRAM 布局并禁用未使用的 IROM2；2026-10-07 使用现有 EIDE/ARM Compiler 5.06 完整重编译 37 个源文件成功，当前编译日志无错误、无警告，已生成 HEX。Flash 占用约 12.1 KB，RAM 占用约 15.2 KB。

2026-10-08 按用户要求，将先前误建在第 15 项的完整代码合入本目录的 `Output/BMI088_Read`，保留原工程名、CubeMX 配置、EIDE 工程标识和本地芯片包。现在不是空任务：`App` 包含 BMI088 驱动、硬件适配、静止校准、Mahony、任务调度及 JustFloat 六个配对模块。

合并后完整重编译 43 个源文件，0 错误、0 警告；4316 项模拟检查全部通过。当前 ROM 占用 20100 字节，RAM 占用 15552 字节，HEX 与合并前的已验证完整固件一致。前述 37 文件/12.1 KB 是空任务基础工程的历史结果，不是当前固件大小。

教程、算法来源、模拟测试和验证记录已统一合入本任务。合并前会修改的原文件，以及第 15 项原目录，保存在 `02_过程记录/合并前备份/2026-10-08`，可恢复；不再单列第 15 项考核入口。构建复核结果见 [合并记录](02_过程记录/合并记录.md)。真实烧录、SPI 电气/时序、RTOS 周期和 VOFA 模型方向仍需实机验收。

使用顺序：打开 `Output/BMI088_Read/MDK-ARM/BMI088_Read.code-workspace`，在 EIDE 完整构建并烧录；复位后静止 3～5 秒；VOFA 使用 USART1 对应 COM、115200/8N1、JustFloat，Cube 选择 Euler/Degree，X/Y/Z 绑定 I0/I1/I2。详细步骤见 [从烧录到 VOFA 姿态显示](01_学习笔记/从烧录到VOFA姿态显示.md)，代码导航见 [App 模块说明](Output/BMI088_Read/App/README.md)。

## 目录

| 目录 | 用途 |
| --- | --- |
| 01_学习笔记 | FreeRTOS、BMI088 原理、SPI 通信、CubeMX 配置、驱动移植与姿态解算笔记 |
| 02_过程记录 | 考核要求、配置截图、编译烧录和 VOFA+ 验证记录 |
| 03_参考资料 | 云台板原理图、硬件图片与后续驱动参考资料 |
| Output | 基于 FreeRTOS 的 CubeMX/Keil 工程与业务模块 |

工程名保持为 `BMI088_Read`，直接打开 `Output/BMI088_Read/BMI088_Read.ioc` 配置，不需要重新建工程。

## FreeRTOS 配置方案

| 项目 | 本次设置 |
| --- | --- |
| 接口 | CMSIS_V2，与任务 08、10、11 一致 |
| HAL 时基 | TIM2 |
| FreeRTOS 时基 | SysTick，1000 Hz |
| 内存管理 | heap_4，动态任务分配 |
| FreeRTOS 总堆 | 10240 字节 |
| 用户任务 | imuTask，入口 StartImuTask，osPriorityNormal |
| 任务栈 | CubeMX 填 512 words，对应 2048 字节 |
| 第一版数据流程 | IMU 任务初始化 BMI088、静止校准，再每 5 ms 读取、Mahony 解算并发送欧拉角 |

TIM2 专用于 HAL 时基，生成器自动配置其计数与中断。任务调度等待使用 RTOS 延时；BMI088 初始化所需的短延时按驱动要求处理。SPI1 与 USART1 首版采用任务内阻塞传输，PB0、PC9 保持输入。

UART 每帧三个小端 float，按 Roll/Pitch/Yaw 排列，再加 `00 00 80 7F` 帧尾，共 16 字节。正常 200 帧/秒在 115200、8N1 下约占 28% 带宽。错误/重校准/重同步时不承诺仍有连续 200 Hz 有效帧。六轴 Yaw 是相对航向，会漂移；Roll/Pitch 由重力初始化，不是任意初始倾斜全部清零。

## 原始资料

| 文件 | 来源与说明 |
| --- | --- |
| 03_参考资料/IMU_V4.7.4_BMI088_ICM42688.pdf | 复制自任务 11 的参考资料，内容保持一致 |
| 03_参考资料/云台板硬件图_01.jpg | 云台板硬件正反面图之一，复制自任务 11 |
| 03_参考资料/云台板硬件图_02.jpg | 云台板硬件正反面图之二，复制自任务 11 |

## EIDE 内存布局

持久配置保存在 `Output/BMI088_Read/MDK-ARM/.eide/eide.yml`，EIDE 根据该配置自动生成构建目录下的链接文件。布局与原始 Keil 工程一致：

| 区域 | 起始地址 | 容量 | 状态 |
| --- | --- | --- | --- |
| IROM1 | 0x08000000 | 0x00100000 | 启用，Startup 勾选 |
| IRAM1 | 0x20000000 | 0x0001C000 | 启用 |
| IRAM2 | 0x2001C000 | 0x00004000 | 启用 |
| IROM2 | 不使用 | 不使用 | 禁用 |

HEX 输出为 `Output/BMI088_Read/MDK-ARM/build/BMI088_Read/BMI088_Read.hex`。

## 已核对的硬件连接

| 功能 | 引脚 |
| --- | --- |
| SPI1 SCK / MISO / MOSI | PA5 / PA6 / PA7 |
| BMI088 陀螺仪片选 | PA4，GPIO 推挽输出，初始高电平 |
| BMI088 加速度计片选 | PC4，GPIO 推挽输出，初始高电平 |
| BMI088 加速度计 / 陀螺仪中断 | PB0 / PC9，第一版轮询时配置为输入 |
| USART1 TX / RX | PA9 / PA10，115200、8N1 |
| SWDIO / SWCLK | PA13 / PA14 |

外部晶振为 25 MHz，系统时钟为 168 MHz，APB1 为 42 MHz，APB2 为 84 MHz。SPI1 使用 Mode 0、8 位、MSB First、软件片选，32 分频时 SCK 为 2.625 MHz。
