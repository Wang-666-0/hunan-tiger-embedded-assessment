# FreeRTOS 进阶：三个任务、消息队列与固定周期

姓名：王世伟. 日期：2026-10-05.

根据本次 CubeMX 配置、VS Code EIDE 编译烧录、VOFA 联调和 Git 分支练习整理，由 Codex 辅助编写. 内容按当前工程记录，重点解释实现方法、重要函数和本次遇到的问题.

## 1. 考核要求与完成情况

| 考核要求 | 本次实现 |
| --- | --- |
| 了解多线程机制与任务调度 | 在单核 STM32 上创建三个用户任务，使用抢占式调度 |
| 手动创建 PWM、串口、IMU 任务 | 在 MX_FREERTOS_Init 中调用 xTaskCreate |
| 要求任务周期精确 | PWM 和 IMU 使用 vTaskDelayUntil，分别按 5 ms、10 ms 时间表运行 |
| 任务间使用消息队列 | 使用串口字节队列、PWM 命令队列、IMU 最新数据队列 |
| 上位机改变亮度或呼吸频率 | 解析 B/T 命令，将控制消息发给 PWM 任务 |
| 后续软件任务基于 FreeRTOS | 应用功能在任务中执行，main.c 负责初始化和启动内核 |

已完成编译烧录、呼吸灯运行、IMU 六轴输出和串口命令回复验证. 最后将周期性的 IMU 打印移入 UART 任务，调整已编译通过；已有联调截图记录的是该调整之前的阶段. 固定周期设计已实现，没有进行示波器或逻辑分析仪的周期抖动测量.

最终接线验证：PA8 接 LED 负极时，B0 使灯停止呼吸但仍点亮；改为 PA8 接 LED 正极、负极侧接 GND 后，B0 可以熄灭. 当前代码与高电平有效的接法匹配，已确认问题来自点亮电平与控制方向的对应关系，详见第 11 节.

## 2. 工程目录与代码分工

本次任务目录为 `10_FreeRTOS进阶`.

| 文件或目录 | 作用 |
| --- | --- |
| 01_学习笔记 | 本学习笔记 |
| 02_过程记录 | 关键截图 |
| Output/FreeRTOS_Advanced | CubeMX 生成及后续编写的工程 |
| Core/Src/main.c | 初始化 HAL、时钟、外设，初始化内核并启动调度 |
| Core/Src/freertos.c | 创建三个任务、队列、互斥锁；实现应用逻辑 |
| Core/Src/tim.c | TIM1 PWM 配置 |
| Core/Src/usart.c | USART1 参数、引脚和中断配置 |
| Core/Src/i2c.c | I2C1 参数与引脚配置 |
| Core/Src/stm32f1xx_it.c | 硬件中断入口，调用对应 HAL 中断处理函数 |
| Core/Src/stm32f1xx_hal_timebase_tim.c | 使用 TIM2 提供 HAL 毫秒计时 |
| Core/Inc/FreeRTOSConfig.h | Tick、堆大小、调度及可用接口配置 |
| MDK-ARM/.eide/eide.yml | EIDE 构建、存储区域等配置 |

业务代码主要写在 freertos.c，不能把针对该文件的指导直接套进 main.c. 两个文件的默认头文件和 USER CODE 标记不同，修改前要核对实际内容.

## 3. 硬件与 CubeMX 配置

### 3.1 引脚与时钟

| 项目 | 本次设置 |
| --- | --- |
| MCU | STM32F103C8T6，按 64 KB Flash、20 KB SRAM 配置 |
| 系统时钟 | 外部晶振 8 MHz，PLL ×9，SYSCLK 72 MHz |
| 调试 | Serial Wire，PA13/PA14 |
| 呼吸灯 | PA8，TIM1_CH1；最终 PA8 接 LED 正极，负极侧接 GND，串联限流电阻 |
| MPU6050 SCL / SDA | PB6 / PB7，I2C1 |
| USART1 TX / RX | PA9 / PA10 |
| 串口参数 | 115200，8 数据位，无校验，1 停止位，无硬件流控 |
| I2C | Standard Mode，100000 Hz，7 位地址 |

MPU6050 的 AD0 接地时，7 位设备地址为 0x68. HAL 访问时传入 `0x68 << 1`. 串口连接电脑时 TX/RX 交叉连接，并确保共地.

### 3.2 PWM 的三个时间概念

TIM1 输入时钟为 72 MHz，Prescaler=71，Period=999，PWM Mode 1，输出极性 High.

```text
计数频率 = 72000000 / (71 + 1) = 1000000 Hz
PWM 载波频率 = 1000000 / (999 + 1) = 1000 Hz
```

| 时间或频率 | 本次含义 |
| --- | --- |
| PWM 载波 1 kHz | 定时器硬件不断输出高低电平脉冲 |
| PWM 任务 5 ms 更新一次 | 软件调整比较值，使占空比逐渐变化 |
| 默认呼吸周期 2000 ms | 从起点经过变亮、变暗再回到起点的完整过程 |

改变 T 命令修改的是呼吸周期，不是定时器的 1 kHz 载波. 比较值决定高电平占比，但高电平是否点亮 LED 取决于接法.

### 3.3 FreeRTOS 配置

1. FREERTOS 的 Interface 选择 CMSIS_V2.
2. Config parameters 中启用抢占式调度，Tick Rate 设为 1000 Hz.
3. TOTAL_HEAP_SIZE=10240 Bytes，使用 heap_4.
4. Include parameters 中 vTaskDelayUntil 设为 Enabled. 界面显示函数名，生成文件中对应 INCLUDE_vTaskDelayUntil=1.
5. SYS 的 Timebase Source 选择 TIM2，让 HAL 毫秒计时与 FreeRTOS SysTick 分工.
6. USART1 global interrupt 启用，抢占优先级 5、子优先级 0，允许调用 FreeRTOS 中断接口.
7. 生成工程后，在代码里手动创建三个任务，禁用 defaultTask 的创建.

本工程 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`. 使用 FreeRTOS FromISR 接口的可配置外设中断，其优先级数值不能比这一阈值更小；USART1 使用 5 满足本次配置要求. Cortex-M 中断优先级数值越小越紧急，而 FreeRTOS 任务优先级数值越大越高，不要混淆.

图 01 · 启用周期等待函数：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/FreeRTOS%E6%B6%88%E6%81%AF%E9%98%9F%E5%88%97/10_FreeRTOS%E8%BF%9B%E9%98%B6/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/01_%E5%90%AF%E7%94%A8%E5%91%A8%E6%9C%9F%E7%AD%89%E5%BE%85%E5%87%BD%E6%95%B0.png).

图片重点：Include parameters 中 vTaskDelayUntil 的 Enabled 状态.

## 4. 从启动到三个任务运行

main.c 中的启动顺序如下.

```text
HAL_Init
  → SystemClock_Config
  → GPIO、I2C1、TIM1、USART1 初始化
  → osKernelInitialize
  → MX_FREERTOS_Init
  → osKernelStart
  → 调度器运行各任务
```

`osKernelInitialize()` 初始化内核，`MX_FREERTOS_Init()` 创建应用对象，`osKernelStart()` 启动调度. 正常运行时，业务逻辑不会再依靠 main.c 的空 while 循环.

CMSIS-RTOS2 在本工程中封装 FreeRTOS 内核. 启动使用 CMSIS 接口，三个用户任务和队列使用 FreeRTOS 原生接口.

### 4.1 手动创建任务

```c
xTaskCreate(PwmTask,  "PWM",  256, NULL, 4, NULL);
xTaskCreate(UartTask, "UART", 384, NULL, 2, NULL);
xTaskCreate(ImuTask,  "IMU",  512, NULL, 3, NULL);
```

实际工程逐个检查返回值，不是忽略创建失败.

以 PWM 为例，六个参数依次为：任务函数、调试名称、栈深度、传给任务的参数、优先级、用于返回任务句柄的地址.

| 参数 | 本次解释 |
| --- | --- |
| PwmTask | 任务入口函数，类型为 void 函数，接收 void * 参数 |
| "PWM" | 任务名称，方便调试 |
| 256 | 栈深度单位为 StackType_t，本 STM32 为 4 字节，因此是 1024 字节 |
| 第一个 NULL | 不传入用户参数 |
| 4 | 任务优先级 |
| 第二个 NULL | 不保存该任务的句柄 |

UART 栈为 384×4=1536 字节，IMU 栈为 512×4=2048 字节. 这些动态任务栈从 FreeRTOS 堆中分配，不能在统计 SRAM 时把已经计入的堆再加一次. 堆还用于任务控制块、队列等对象，不能只看三个任务的栈总和.

创建成功返回 pdPASS，资源不足等情况会失败. 工程检查失败后调用 Error_Handler.

### 4.2 调度如何分工

| 任务 | 优先级 | 工作节奏 |
| --- | --- | --- |
| PWM | 4 | 每 5 ms 更新比较值，并取出待处理命令 |
| IMU | 3 | 初始化后，每 10 ms 读取原始六轴 |
| UART | 2 | 接收命令、回复消息，约每 200 ms 打印 IMU 状态 |

STM32 是单核，任务通过保存与恢复运行上下文交替执行. 当前优先级最高的就绪任务获得 CPU；高优先级任务进入等待状态后，低优先级任务才有机会执行. 这不是固定顺序轮流执行三个完整循环.

“三个任务”指三个用户任务，内核仍有 Idle 等内部任务. defaultTask 的函数和属性可能留在文件里，但创建语句被注释后，它不会作为额外用户任务运行.

## 5. 消息结构与三条队列

### 5.1 为什么用消息队列

UART 只负责理解上位机意图，把“修改什么、改成多少”发给 PWM；实际灯光参数由 PWM 任务自己管理. IMU 只负责读取，把数据发给 UART；打印由 UART 负责. 这样减少任务之间直接修改变量的耦合.

```text
USART1 接收中断
  → uart_rx_queue：原始字节
  → UART 任务：拼接、校验命令
  → pwm_command_queue：命令类型和数值
  → PWM 任务：改变参数、更新输出

IMU 任务
  → imu_data_queue：最新六轴及计数
  → UART 任务：格式化、串口发送
  → 上位机
```

### 5.2 结构体定义

```c
typedef enum
{
    PWM_SET_MAX_BRIGHTNESS,
    PWM_SET_BREATH_PERIOD
} PwmCommandType;

typedef struct
{
    PwmCommandType type;
    uint32_t value;
} PwmCommand;
```

`type` 区分亮度命令和周期命令，`value` 保存数值. 同一个结构体可以表示 B500 或 T500，但二者的 type 不同.

IMU 消息包含 ax、ay、az、gx、gy、gz 六个有符号 16 位值，以及 sample_count、read_errors 两个计数.

### 5.3 队列配置和操作

| 队列 | 创建参数 | 本次用途 |
| --- | --- | --- |
| pwm_command_queue | 8，sizeof(PwmCommand) | 按先入先出顺序保存 PWM 命令 |
| uart_rx_queue | 128，sizeof(uint8_t) | 接收中断与 UART 任务之间暂存字节 |
| imu_data_queue | 1，sizeof(ImuMessage) | 保存最新一份 IMU 状态 |

`xQueueCreate(数量, 每项字节数)` 返回队列句柄，失败返回 NULL. sizeof 包含结构体实际占用和可能的对齐空间，不需要手工相加猜大小.

```c
xQueueSend(pwm_command_queue, &command, 0);
xQueueReceive(pwm_command_queue, &command, 0);
xQueueOverwrite(imu_data_queue, &message);
```

- Send 的三个参数：队列、待复制消息的地址、队列满时最多等待多少 Tick.
- Receive 的三个参数：队列、接收缓冲区地址、队列空时最多等待多少 Tick.
- 最后参数为 0 表示立即返回，不等待. Send/Receive 成功返回 pdPASS.
- 队列复制指定字节数的内容，本次不是把局部结构体指针长期保存在队列中.
- Overwrite 只用于本次长度为 1 的队列，直接用新消息替换旧消息. 上位机取得最新状态，不保证取得所有历史样本.

## 6. PWM 任务：接命令并计算呼吸波形

### 6.1 初始化

```c
TickType_t last_wake = xTaskGetTickCount();
const TickType_t update_period = pdMS_TO_TICKS(5);
uint32_t max_brightness = 1000;
uint32_t breath_period_ms = 2000;
uint32_t phase_ms = 0;
```

last_wake 是周期基准，update_period 是更新间隔，phase_ms 表示当前处于呼吸过程中的哪一位置. 开始循环前调用 HAL_TIM_PWM_Start 启动 TIM1 通道 1 的硬件输出.

### 6.2 每轮接收命令

PWM 每轮最多接收 8 条消息；如果队列为空立即退出接收循环. 这样避免持续处理消息妨碍更新输出.

按 command.type 分别处理：亮度在 0～1000 内则更新 max_brightness；周期在 200～10000 ms 内则更新 breath_period_ms，并将 phase_ms 清零. 修改周期后从周期起点重新开始.

UART 已做一次校验，PWM 再检查范围，为实际控制端保留边界保护.

### 6.3 三角波计算

将完整周期分成两个阶段.

```c
uint32_t half_period_ms = breath_period_ms / 2;

if (phase_ms < half_period_ms)
    brightness = max_brightness * phase_ms / half_period_ms;
else
    brightness = max_brightness * (breath_period_ms - phase_ms)
               / (breath_period_ms - half_period_ms);
```

以 max_brightness=1000、完整周期 2000 ms 为例.

| phase_ms | brightness | 含义 |
| --- | --- | --- |
| 0 | 0 | 起点 |
| 500 | 500 | 上升阶段中点 |
| 1000 | 1000 | 峰值 |
| 1500 | 500 | 下降阶段中点 |
| 接近 2000 | 接近 0 | 回到起点附近 |

计算使用整数除法，会截断小数. 每次 phase_ms 增加 5，达到周期末尾时清零. 不是 5 的整数倍的输入周期存在步进量化；实际波形也取决于任务是否按计划得到执行.

```c
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);
vTaskDelayUntil(&last_wake, update_period);
```

设置比较寄存器后，硬件继续输出 PWM；任务可以等待，不需要 CPU 手动维持每一个高低电平.

图 02 · PWM 任务与烧录验证：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/FreeRTOS%E6%B6%88%E6%81%AF%E9%98%9F%E5%88%97/10_FreeRTOS%E8%BF%9B%E9%98%B6/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/02_PWM%E4%BB%BB%E5%8A%A1%E4%B8%8E%E7%83%A7%E5%BD%95%E9%AA%8C%E8%AF%81.png).

图片重点：周期基准、5 ms 换算、启动 PWM 和终端 Verified OK. 截图记录早期 PWM 实现阶段，不代表截图中的每行代码都是最终版本.

## 7. 串口接收：中断搬字节，任务解析命令

### 7.1 启动一次接收

UART 任务先调用：

```c
HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
```

使用 USART1，通过中断接收 1 字节，保存到 uart_rx_byte. 函数启动接收后返回，收到数据时由中断处理流程通知完成.

### 7.2 接收完成回调

HAL_UART_RxCpltCallback 检查中断来源是不是 USART1，然后执行三件事.

1. 用 xQueueSendFromISR 把字节复制到接收队列.
2. 再次调用 HAL_UART_Receive_IT，启动下一个字节的接收.
3. 调用 portYIELD_FROM_ISR，按需要请求任务切换.

```c
BaseType_t higher_priority_task_woken = pdFALSE;
xQueueSendFromISR(uart_rx_queue, &uart_rx_byte,
                  &higher_priority_task_woken);
portYIELD_FROM_ISR(higher_priority_task_woken);
```

第三个参数用于报告：该发送是否唤醒了需要及时调度的高优先级任务. 当前 UART 任务轮询取队列，通常没有因等待该队列而阻塞，不能把这里理解为每个字节必然唤醒 UART.

队列满时本次代码增加 uart_rx_dropped，不会在中断里等待. 中断中不能使用普通队列发送的阻塞等待，也不在这里执行复杂字符串解析和串口打印.

### 7.3 拼接一行命令

UART 使用 char line[32]，最多存 31 个有效字符，最后一个空间留给字符串结束符 `\0`.

| 收到的字符 | 处理 |
| --- | --- |
| 普通字符 | 存入 line，length 增加 |
| `\r` | 忽略，允许 CRLF |
| `\n` | 认为本条命令结束，补 `\0` 后调用 ProcessCommand |
| 超出容量 | 标记 overflow，丢弃本行剩余内容，遇换行后回复错误并复位 |

所以在 VOFA 中输入 T500 还不够，要选择追加换行. `\n` 是协议说明中的写法，实际发送的是一个换行字节.

每轮最多取 32 个接收字节，再处理遥测打印，最后 vTaskDelay 1 ms. UART 任务的工作量随输入变化，本工程没有将它设计为精确 1 ms 执行一次；其串口接收由硬件中断持续完成.

## 8. ProcessCommand：验证 B/T，再发送控制消息

处理顺序是：识别首字母 → 检查数字长度 → 确认每一位都是数字 → 转数值 → 检查范围 → 入 PWM 队列 → 回复结果.

```c
const char *number = &line[1];
size_t digits = strlen(number);
uint32_t value = (uint32_t)strtoul(number, NULL, 10);
```

line[0] 是 B 或 T，&line[1] 指向后面的数字字符串. strlen 不包含末尾的 `\0`；strtoul 的最后一个参数 10 指定十进制转换. 本次先检查数字合法性和长度，再进行转换.

| 命令 | 数值含义 | 范围 |
| --- | --- | --- |
| B500 | 比较值峰值设为 500 | 0～1000 |
| B1200 | 超范围，不修改旧设置 | 回复 ERR range |
| T500 | 完整呼吸周期 500 ms | 200～10000 ms |
| T2000 | 完整呼吸周期 2 s | 默认值 |

呼吸频率 f=1000/T，T 单位为 ms. T500 对应 2 Hz. B 修改峰值，没有把程序切换为恒定亮度模式.

| 回复 | 意义 |
| --- | --- |
| OK queued | 控制消息成功入队，PWM 任务随后执行 |
| ERR command | 首字母不是大写 B/T |
| ERR number | 数字为空、长度不合适，或含非数字字符 |
| ERR range | 数值超出允许范围 |
| ERR queue full | PWM 命令队列已满，本条未入队 |
| ERR line too long | 整行超过接收缓冲区容量 |

OK queued 是队列发送成功，不是硬件执行结果确认. 错误命令保留原有灯光参数，因此 B1200 被拒绝后仍然呼吸是符合代码逻辑的.

## 9. IMU 任务：初始化、读取、发布最新数据

### 9.1 初始化流程

任务等待 100 ms，让模块上电稳定，然后读取 0x75 WHO_AM_I；通信成功且 ID=0x68 才继续.

身份失败时发送 ERR IMU identity，并调用 vTaskSuspend(NULL) 挂起自己，其他任务仍可运行. 当前代码没有恢复该任务的逻辑. 后续配置写入失败则调用 Error_Handler，与仅挂起 IMU 的处理不同.

| 寄存器 | 写入值 | 作用 |
| --- | --- | --- |
| 0x6B | 0x01 | 退出休眠，选择 X 轴陀螺仪 PLL 时钟 |
| 0x1A | 0x03 | 配置内部数字低通滤波 |
| 0x19 | 9 | 当前配置下，1000/(1+9)=100 Hz 输出采样率 |
| 0x1B | 0x00 | 陀螺仪量程 ±250 °/s |
| 0x1C | 0x00 | 加速度量程 ±2 g |

传感器内部输出 100 Hz，与 MCU 每 10 ms 读取是两个时钟下的动作. 软件读取没有使用数据就绪同步，因此不能仅凭这两个数值就保证每次恰好取得一份不同的新样本.

### 9.2 连续读取 14 字节

```c
HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B,
                I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 5);
```

| 参数 | 解释 |
| --- | --- |
| &hi2c1 | I2C1 句柄 |
| 0x68 << 1 | HAL 接口要求的设备地址形式 |
| 0x3B | 起始寄存器地址 |
| I2C_MEMADD_SIZE_8BIT | 寄存器地址宽度为 8 位 |
| data | 接收缓冲区 |
| sizeof(data) | 数组为 14 字节，本次读取 14 字节 |
| 5 | HAL 阻塞调用的超时参数，单位 ms |

函数返回 HAL 通信状态，数据写入缓冲区；返回值不是加速度. 超时参数不是任务周期，也不自动让任务在 5 ms 后按计划运行.

| 缓冲区下标 | 数据 |
| --- | --- |
| 0、1 | AX，高字节、低字节 |
| 2、3 | AY |
| 4、5 | AZ |
| 6、7 | 温度，本次忽略 |
| 8、9 | GX |
| 10、11 | GY |
| 12、13 | GZ |

```c
int16_t ax = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
```

高字节左移 8 位，再用按位或拼入低字节，最后按有符号 16 位数解释. 负方向的数据可以得到负数. A/G 打印的是原始计数，不是 g 和 °/s；当前量程下分别除以 16384 和 131 才可换算物理单位.

### 9.3 成功计数与数据发布

成功时增加 sample_count，构造 ImuMessage，用 xQueueOverwrite 发布最新数据；失败时增加 read_errors. 末尾使用绝对延时等待下个读取时刻.

n 表示 MCU 成功读取次数，err 表示读取失败次数. err=0 能说明这些读取没有报告 HAL 失败，不能单独证明每次采样时间严格准确.

IMU 常规循环里不再 snprintf 或发送串口. UART 每约 200 ms 取最新消息，生成如下文本.

```text
IMU n=15441 err=0 A:1356,-590,16528 G:-537,97,-27
```

串口文本用 snprintf 写入固定缓冲区. 返回值是原本需要写入的字符数，不包含末尾 `\0`；只有大于 0 且小于缓冲区大小才发送，避免把截断后的长度误当实际可发送长度.

图 03 · IMU 输出与串口命令联调：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/FreeRTOS%E6%B6%88%E6%81%AF%E9%98%9F%E5%88%97/10_FreeRTOS%E8%BF%9B%E9%98%B6/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/03_IMU%E8%BE%93%E5%87%BA%E4%B8%8E%E4%B8%B2%E5%8F%A3%E5%91%BD%E4%BB%A4%E8%81%94%E8%B0%83.png).

图片重点：n 持续增加、err=0、A/G 六轴输出，以及 T500 输入和换行设置. 用户已确认收到命令回复，但截图没有显示回复行. 该截图记录最后调整前的联调阶段.

## 10. 为什么使用互斥锁保护串口

UART 任务发送命令回复和常规 IMU 状态；IMU 初始化失败时也可能发送错误消息. 同一个 UART 句柄需要避免同时被两个任务使用.

```c
uart_tx_mutex = xSemaphoreCreateMutex();
```

UartSend 封装的流程如下.

```text
xSemaphoreTake 等待取得互斥锁
  → HAL_UART_Transmit 发送
  → xSemaphoreGive 释放锁
  → 返回发送状态
```

取得锁最多等待 100 ms；实际发送的超时由 UartSend 的 timeout 参数决定. 这两个等待是分别发生的，不能把 timeout 当整个函数的唯一时间上限.

互斥锁提供资源互斥，并具有优先级继承机制，缓解优先级反转. 它不会让串口发送自动变成 DMA 或非阻塞发送. 本次 HAL_UART_Transmit 仍是阻塞式发送，较低优先级的 UART 任务可以被就绪的 PWM/IMU 任务抢占.

UartSend 只在任务中使用，不在串口接收中断回调中使用. 发送结束后即使 HAL 返回错误，也先释放锁，避免把资源永久占住.

## 11. LED 接法与 B0：本次实际验证结论

本次最初 PA8 接 LED 负极，发送 B0 后停止呼吸但没有熄灭. 改为 PA8 接 LED 正极后，发送 B0 可以熄灭. 这说明需要把“GPIO 输出电平”与“LED 点亮条件”一起考虑.

### 11.1 为什么同一条命令出现不同灯光效果

LED 有电流从正极流向负极时才点亮. 在串联限流电阻的前提下，两种常见接法如下.

| 接法 | 点亮时的电流路径 | 点亮电平 |
| --- | --- | --- |
| PA8 接正极，负极侧接 GND | PA8 → 电阻/LED → GND | 高电平 |
| 正极侧接 3.3 V，PA8 接负极 | 3.3 V → 电阻/LED → PA8 | 低电平 |

当前 TIM1 是 PWM Mode 1、High 极性. B0 将 max_brightness 设为 0，每一轮计算的比较值都是 0，最终 CCR1=0，PA8 持续为低电平.

- 接负极时，PA8 拉低形成电流路径，灯持续亮着，所以只是没有呼吸变化.
- 接正极时，PA8 拉低不能提供正向点亮电压，灯熄灭，符合本次代码设计.

这里停止的是占空比变化，不是 PWM 任务被挂起. PWM 任务仍按 5 ms 时间表运行，phase_ms 也仍然推进.

### 11.2 比较值与亮度的方向

在当前硬件配置下，高电平占比约为 CCR/(ARR+1)=CCR/1000.

| CCR | 高电平占比 | 高电平有效接法 | 低电平有效接法 |
| --- | --- | --- | --- |
| 0 | 0% | 熄灭 | 持续点亮 |
| 500 | 50% | 中间占空比 | 中间占空比 |
| 1000 | 100% | 持续点亮 | 熄灭 |

表中的占空比不是人眼感受到的精确亮度百分比. 两种接法的亮度方向相反，原本名为 max_brightness 的数值本质上控制的是高电平占空比峰值；只有与高电平有效接法配合，才直接对应本次“最大亮度”的设计.

本次最终采用 PA8 接正极、负极侧接 GND 的高电平有效接法，保留原来的 PWM 代码. 若以后使用低电平有效的灯，需要在输出端统一反转占空比或调整 PWM 极性，再重新验证 B0、B500、B1000 的行为，不能只凭变量名判断亮灭.

## 12. 周期精确：绝对延时与相对延时

### 12.1 相对延时

```c
/* 先执行本轮工作 */
vTaskDelay(pdMS_TO_TICKS(10));
```

从调用延时的位置开始等待. 若工作耗时约 2 ms，再等约 10 ms，循环周期约为 12 ms，还会受到调度影响. 适合上电等待、轮询让出 CPU 等场景.

### 12.2 绝对延时

```c
TickType_t last_wake = xTaskGetTickCount();
for (;;)
{
    /* 本轮工作 */
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
}
```

last_wake 在循环外初始化一次. 函数自动把下一次目标时刻推进一个 period，形成类似 0、10、20、30 ms 的时间表. 若本轮工作花约 2 ms，就等待剩余约 8 ms；若花约 4 ms，就等待剩余约 6 ms.

不要每轮先把 last_wake 重设为当前时刻，否则失去原本固定时间表的意义. IMU 在初始化完成后才建立基准，避免将上电等待和寄存器初始化耗时算入周期循环.

### 12.3 pdMS_TO_TICKS 是什么

这是毫秒转 Tick 的宏，概念换算关系如下.

```text
Tick 数 = 毫秒数 × configTICK_RATE_HZ / 1000
```

本工程 configTICK_RATE_HZ=1000，因此 1 Tick=1 ms，pdMS_TO_TICKS(10)=10 Tick. 若 Tick 为 100 Hz，则 1 Tick=10 ms，同一表达式得到 1 Tick.

函数接收的是 Tick，不是毫秒. 直接写 10 表示 10 Tick，使用宏才能清楚表达想等 10 ms. 整数换算可能截断，较低 Tick 频率下过短的毫秒数可能转成 0.

### 12.4 本次如何减少周期偏差

| 措施 | 目的 |
| --- | --- |
| PWM 和 IMU 使用绝对延时 | 避免工作时间不断累积到周期中 |
| PWM 优先级 4、IMU 3、UART 2 | 周期任务就绪后优先运行 |
| PWM 每轮最多处理 8 条命令 | 避免连续命令占住更新循环 |
| IMU 队列发送不等待 | 不被显示任务的处理速度拖住 |
| IMU 的常规串口打印移至 UART | 将字符串格式化与串口传输移出采样循环 |
| UART 每轮最多处理 32 字节 | 给其遥测显示保留处理机会 |

绝对延时不能弥补超期执行. 若本轮工作或调度延误超过周期，目标时刻已经过去，可能不再等待. 因此“按固定时间表等待”不等于“硬件上完全无抖动”；周期设计和实际时序测量要分开记录.

## 13. 本次排错与容易混淆的问题

### 13.1 大量 L6406/L6407 链接报错

当时链接器报告大量段没有空间，最后报约 188 条错误. 根因是 EIDE 执行区域配置不正确，并不是 188 处 C 语法错误.

| 区域 | 起始地址 | 大小 |
| --- | --- | --- |
| IROM1，Flash | 0x08000000 | 0x10000，64 KB |
| IRAM1，SRAM | 0x20000000 | 0x5000，20 KB |

修正区域地址和大小，并关闭无效 IROM2 后编译成功. 即使下载器报告 128 KiB，本次仍按 STM32F103C8 工程声明的 64 KB 配置，没有通过扩大容量掩盖问题.

Flash 主要保存程序与只读数据；SRAM 保存可写变量、堆、栈等. 构建 RAM 统计不代表所有运行时峰值都已验证，但 FreeRTOS 静态保留的堆数组已经属于 SRAM 占用.

### 13.2 修改文件与头文件重复

main.c 生成内容已经包含 tim.h，并不意味着 freertos.c 也包含同样的头文件. 本次 freertos.c 的 USER CODE BEGIN Includes 中加入任务真正使用的 tim.h、queue.h、usart.h、i2c.h、semphr.h、标准库头文件；修改前应读实际文件，不机械重复添加.

### 13.3 重新生成 CubeMX 代码

用户逻辑尽量放入 USER CODE 区域. defaultTask 创建语句被注释的位置在生成区域中，重新生成后可能被恢复，必须检查是否意外多创建一个默认任务. 时钟、引脚、Tick 和堆设置也应以当前生成文件为准.

### 13.4 最终代码与旧注释

IMU 内有一处“每 200 ms 显示一次”的旧注释，源自打印仍在 IMU 任务时的版本. 最终实际代码在每次成功读取后发布数据，由 UART 约每 200 ms 打印. 判断职责时应读实际语句，不仅看旧注释.

## 14. 常用函数速查

| 函数或宏 | 本次作用 | 关键点 |
| --- | --- | --- |
| xTaskCreate | 手动创建任务 | 栈单位为元素数，本平台每元素 4 字节 |
| xTaskGetTickCount | 获取内核 Tick 计数 | 建立周期基准 |
| pdMS_TO_TICKS | 毫秒转 Tick | 宏，不是延时动作本身 |
| vTaskDelay | 相对等待 | 用于初始化等待和 UART 让出 CPU |
| vTaskDelayUntil | 按周期时间表等待 | 基准在循环外初始化 |
| xQueueCreate | 创建队列 | 数量与单项字节数分开指定 |
| xQueueSend / Receive | 任务中发消息、取消息 | 本工程等待参数为 0 |
| xQueueSendFromISR | 中断中发送字节 | 使用 FromISR 接口 |
| xQueueOverwrite | 发布最新 IMU 状态 | 本次队列长度必须为 1 |
| xSemaphoreCreateMutex | 创建 UART 互斥锁 | 检查是否返回 NULL |
| xSemaphoreTake / Give | 获取、释放发送权限 | 发送失败也要释放已取得的锁 |
| HAL_UART_Receive_IT | 启动中断接收 | 完成后重新启动下一次接收 |
| HAL_UART_Transmit | 发送回复和遥测文本 | 本工程为阻塞发送 |
| HAL_I2C_Mem_Read / Write | 访问 MPU6050 寄存器 | 状态由返回值给出，数据由缓冲区传递 |
| __HAL_TIM_SET_COMPARE | 更新 PWM 比较值 | 比较值与实际亮度的方向依接法而定 |
| snprintf / strtoul | 文本格式化、数字转换 | 检查长度，先校验输入再转换 |

## 15. 本次实践收获

完成了“硬件中断接收 → 队列搬运 → 任务解析 → 控制队列 → PWM 输出”的完整链路，同时加入周期 IMU 读取. 在 VS Code 中完成编译烧录，并练习创建、提交与合并分支.

复习时按五个问题检查理解：三个任务各自负责什么；队列里具体存什么；中断为什么不直接解析命令；绝对延时如何避免周期累计偏差；为什么 B0 对应的低电平在两种 LED 接法中产生不同效果.
## 16. 考核提交与演示梳理

| 演示项 | 本次结果与解释 |
| --- | --- |
| 工程编译、烧录 | 已完成，使用 VS Code EIDE 和 OpenOCD |
| 三个用户任务 | 在 freertos.c 中手动创建，PWM、IMU、UART 优先级分别为 4、3、2 |
| PWM 呼吸 | 已运行；硬件载波 1 kHz，软件每 5 ms 更新比较值 |
| 亮度命令 | B 数值经 UART 解析、队列传递，由 PWM 处理；B1200 超范围保留旧设置 |
| B0 熄灭 | 最终 PA8 接 LED 正极的接法下已实际验证 |
| 周期命令 | T500 已收到回复；T 表示完整呼吸周期，500 ms 对应 2 Hz |
| IMU 读取 | 六轴原始数值持续输出，阶段截图中 err=0 |
| 周期设计 | PWM 5 ms、IMU 10 ms 使用绝对延时，UART 负责非精确周期的命令与显示 |
| 数据传递 | 中断字节队列、任务命令队列、IMU 最新数据队列均有明确职责 |

按本次要求，考核功能已完成. 介绍工程时可按“main 启动 → 创建三个任务 → 中断接字节 → UART 解析命令 → 队列控制 PWM → IMU 发布数据 → UART 显示”的顺序讲解. 周期准确性应表述为已采用固定时间表和减少阻塞的设计，不把功能演示说成已完成硬件时序测量.

学习笔记、原始截图和工程分别放在本任务的三个目录，代码在 FreeRTOS消息队列 分支进行开发与记录. Git 分支练习和 VS Code 烧录属于本次实践过程，功能验证结论以实际观察和对应记录为准.