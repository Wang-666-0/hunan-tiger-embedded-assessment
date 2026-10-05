# FreeRTOS 进阶学习笔记

日期：2026-10-05. 本次考核已完成，使用 VS Code 编译、烧录并练习 Git 分支管理.

## 1. 考核要求与实现

手动创建 PWM 呼吸灯、串口收发、IMU 读取三个任务，使用消息队列，通过上位机修改最大亮度和呼吸周期.

主要代码在 `Output/FreeRTOS_Advanced/Core/Src/freertos.c`. `main.c` 负责外设初始化、内核初始化及启动调度器；`MX_FREERTOS_Init()` 创建互斥锁、队列和三个任务. 默认任务的创建已注释.

| 任务 | 优先级 | 主要工作 | 等待方式 |
| --- | --- | --- | --- |
| PWM | 4，最高 | 接收控制命令，计算亮度并更新 TIM1 CH1 比较值 | 绝对延时，5 ms 更新 |
| IMU | 3 | 初始化 MPU6050，读取原始六轴并发布最新数据 | 绝对延时，10 ms 读取 |
| UART | 2 | 拼接命令、校验参数、回复结果，并约每 200 ms 打印最新 IMU 数据 | 每轮相对延时 1 ms |

STM32 单核上通过调度交替执行任务. 高优先级就绪任务先运行，任务等待时让出 CPU. UART 不承担精确采样；串口打印与 IMU 读取分离，减少对采样周期的影响.

## 2. 配置要点

- 系统时钟 72 MHz；TIM1 CH1 使用 PA8，PSC=71、ARR=999，PWM 载波频率为 1 kHz.
- USART1：PA9 TX、PA10 RX，115200、8N1；接收中断抢占优先级 5，使用 FreeRTOS 的 FromISR 接口.
- I2C1：PB6 SCL、PB7 SDA，100 kHz，MPU6050 的 7 位地址为 0x68.
- FreeRTOS Tick=1000 Hz，1 Tick=1 ms，heap_4 堆大小 10240 字节. SysTick 用于 RTOS，TIM2 用于 HAL 毫秒计时.
- 本次 IMU 读取原始数据，没有使用 DMP. 从 0x3B 连续读取 14 字节，合并出三个加速度和三个角速度值，跳过两个温度字节.

## 3. 消息队列如何连接任务

```text
USART1 接收中断 → 字节队列 → UART 任务 → PWM 命令队列 → PWM 任务
IMU 任务 → 最新数据队列 → UART 任务 → 上位机
```

| 队列 | 容量 | 用法 |
| --- | --- | --- |
| uart_rx_queue | 128 个字节 | 中断调用 xQueueSendFromISR，UART 任务取出字节 |
| pwm_command_queue | 8 条 PwmCommand | UART 发送命令类型与数值，PWM 任务负责修改自身参数 |
| imu_data_queue | 1 条 ImuMessage | IMU 使用 xQueueOverwrite 覆盖旧数据，UART 读取最新样本 |

队列复制消息内容，发送局部结构体的地址不会让队列保存一个失效指针. 长度为 1 的 IMU 队列用于显示最新状态，不保存全部采样历史. 串口发送还使用互斥锁，避免不同任务发送内容交叉.

## 4. 上位机控制

发送大写命令，并添加换行 `\n`，也支持 `\r\n`.

| 命令 | 效果 |
| --- | --- |
| B500 | 最大亮度设为 500，仍然呼吸 |
| B0 | 最大亮度为 0，熄灭 |
| B1200 | 超出 0～1000 范围，回复 ERR range，保留原设置 |
| T500 | 完整的变亮、变暗周期设为 500 ms |

T 的允许范围是 200～10000 ms. 频率与周期关系为 f=1000/T，所以 T500 对应 2 Hz. 回复 `OK queued` 表示命令已入队，PWM 任务随后处理，不是已经测量到灯光变化.

PWM 将周期分成变亮和变暗两段，按阶段进度计算比较值. 每次推进 5 ms，因此任意输入周期存在 5 ms 的量化；PWM 的 1 kHz 载波频率和呼吸频率是两个概念.

## 5. 周期精确的关键

```c
TickType_t last_wake = xTaskGetTickCount();
for (;;)
{
    /* 本轮工作 */
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
}
```

`last_wake` 在循环外初始化一次，函数自动推进下一次目标时刻. `pdMS_TO_TICKS(10)` 把 10 ms 转成 Tick；本工程结果为 10 Tick.

- 相对延时 vTaskDelay：工作结束后再等待，工作时间会加入周期.
- 绝对延时 vTaskDelayUntil：按固定时间表等待，避免工作时间持续累积到周期中.
- 实际执行仍会受中断、高优先级任务及超时影响，绝对延时不代表没有抖动. 本次完成固定周期设计和功能验证，没有进行硬件时序测量.

PWM 每轮最多处理 8 条命令，UART 每轮最多处理 32 字节，避免持续收消息拖住其他工作. IMU 发布数据不等待 UART，串口打印放在低优先级任务.

## 6. 验证与易错点

已验证编译烧录、呼吸灯运行、IMU 六轴输出及 T500 命令回复. 截图为阶段验证记录；最后将 IMU 打印移至 UART 的调整已编译通过.

曾遇到大量 L6406/L6407 链接报错，原因是 EIDE 存储区域设置不正确. 配置为 Flash 起点 0x08000000、大小 0x10000（64 KB），RAM 起点 0x20000000、大小 0x5000（20 KB），关闭无效 IROM2 后解决.

记住：中断用 FromISR 接口；周期任务用绝对延时；命令末尾添加换行；B 控制最大亮度，T 控制呼吸周期. CubeMX 重新生成代码后，应检查默认任务创建是否仍被禁用.

## 7. 关键截图

图 01 · 启用周期等待函数：重点查看 vTaskDelayUntil 的 Enabled 状态. [GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/FreeRTOS%E6%B6%88%E6%81%AF%E9%98%9F%E5%88%97/10_FreeRTOS%E8%BF%9B%E9%98%B6/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/01_%E5%90%AF%E7%94%A8%E5%91%A8%E6%9C%9F%E7%AD%89%E5%BE%85%E5%87%BD%E6%95%B0.png).

图 02 · PWM 任务与烧录验证：重点查看周期基准、5 ms 换算和 Verified OK. [GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/FreeRTOS%E6%B6%88%E6%81%AF%E9%98%9F%E5%88%97/10_FreeRTOS%E8%BF%9B%E9%98%B6/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/02_PWM%E4%BB%BB%E5%8A%A1%E4%B8%8E%E7%83%A7%E5%BD%95%E9%AA%8C%E8%AF%81.png).

图 03 · IMU 输出与串口命令联调：重点查看 n 持续增加、err=0，以及 T500 和换行设置. 命令回复由实际操作确认，截图没有显示回复行. [GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/FreeRTOS%E6%B6%88%E6%81%AF%E9%98%9F%E5%88%97/10_FreeRTOS%E8%BF%9B%E9%98%B6/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/03_IMU%E8%BE%93%E5%87%BA%E4%B8%8E%E4%B8%B2%E5%8F%A3%E5%91%BD%E4%BB%A4%E8%81%94%E8%B0%83.png).