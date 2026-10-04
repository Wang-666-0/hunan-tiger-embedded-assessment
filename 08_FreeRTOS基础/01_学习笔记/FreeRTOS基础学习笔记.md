# FreeRTOS 基础：PWM 呼吸灯与串口任务

姓名：王世伟　日期：2026-10-04.

根据本次 CubeMX、Keil 和串口联调过程整理，由 Codex 辅助编写. 重点记录配置和使用，不展开内核实现.

## 1. 本次目标与当前结果

使用 STM32F103C8T6 创建两个任务：PWM 任务控制 PA8 外接 LED 呼吸，串口任务通过 USART1 与电脑通信.

已完成 CubeMX 配置、任务代码编写和串口回显实验. 截图记录了 PWM 代码编译时 0 Errors、0 Warnings. 串口实际存在连续发送四个字符只回复部分字符的问题；当前代码是入门轮询版本，不能作为稳定连续收发已完成的证明. 呼吸灯与串口同时运行应在现场再次观察确认.

工程入口：[FreeRTOS_Demo.uvprojx](../Output/FreeRTOS_Demo/MDK-ARM/FreeRTOS_Demo.uvprojx).

任务代码：[freertos.c](../Output/FreeRTOS_Demo/Src/freertos.c).

## 2. CubeMX 配置步骤

1. 从原来的 PWM 配置建立 FreeRTOS_Demo 工程，保留 STM32F103C8T6、外部 8 MHz 晶振、72 MHz 系统时钟、Serial Wire 调试接口.
2. TIM1 使用 Internal Clock，CH1 选择 PWM Generation CH1，输出 PA8. PSC=71、ARR=999，PWM 为 1 kHz.
3. SYS → Timebase Source 选择 TIM2，给 HAL 提供计时.
4. 中间件 → FREERTOS → Interface 选择 CMSIS_V2.
5. Tasks and Queues 中将默认任务改为 pwmTask，并添加 uartTask，最终保留两个用户任务. FreeRTOS 仍有内部 Idle 任务等，这里的“两任务”指用户创建的任务.

| 设置 | pwmTask | uartTask |
| --- | --- | --- |
| Entry Function | StartPwmTask | StartUartTask |
| 当前工程优先级 | osPriorityNormal | osPriorityLow |
| Stack Size | 256 Words | 256 Words |
| Allocation | Dynamic | Dynamic |
| Code Generation | Default | Default |
| Parameter | NULL | NULL |

最初教学建议两者 Normal，实际生成工程中串口为 Low，本笔记按工程记录. 优先级数值更高的就绪任务先执行，PWM 等待时串口任务可以运行；调度不是严格一人轮流执行一次. 栈 256 Words 对应生成代码中的 256×4=1024 字节，每个任务分别使用自己的栈.

图 01 · FreeRTOS 双任务配置：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/08_FreeRTOS%E5%9F%BA%E7%A1%80/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/01_FreeRTOS%E5%8F%8C%E4%BB%BB%E5%8A%A1%E9%85%8D%E7%BD%AE.png).

6. USART1 → Asynchronous，关闭 Hardware Flow Control，配置如下.

| 参数 | 值 |
| --- | --- |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| TX / RX | PA9 / PA10 |

本次使用轮询，不开启串口接收中断和 DMA.

图 02 · USART1 参数与引脚：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/08_FreeRTOS%E5%9F%BA%E7%A1%80/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/02_USART1%E5%8F%82%E6%95%B0%E4%B8%8E%E5%BC%95%E8%84%9A.png).

7. FREERTOS → Config parameters：TOTAL_HEAP_SIZE=8192 Bytes，Memory Management scheme=heap_4，TICK_RATE_HZ=1000. 节拍频率已核对生成的 Inc/FreeRTOSConfig.h，参数截图只显示了堆配置.
8. Project Manager：工程名 FreeRTOS_Demo，工具链 MDK-ARM，勾选 Keep User Code when re-generating，然后生成代码并打开 Keil.

图 03 · FreeRTOS 堆内存配置：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/08_FreeRTOS%E5%9F%BA%E7%A1%80/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/03_FreeRTOS%E5%86%85%E5%AD%98%E9%85%8D%E7%BD%AE.png).

## 3. 时钟怎么分工

```text
HSE 外部晶振 8 MHz → PLL ×9 → SYSCLK 72 MHz
  → AHB ÷1 → HCLK 72 MHz：CPU、存储器、DMA
      → SysTick 输入 72 MHz：FreeRTOS 节拍
      → APB1 ÷2 → PCLK1 36 MHz：APB1 外设
          → 定时器时钟 ×2 =72 MHz：TIM2，HAL 计时
      → APB2 ÷1 → PCLK2 72 MHz：GPIO、USART1 等
          → 定时器时钟 ×1 =72 MHz：TIM1，PWM
```

系统时钟仍是 72 MHz. 改 SYS 的 Timebase Source 不会改变芯片运行速度，只是把 HAL 的计时来源从 SysTick 改成 TIM2.

- SysTick 用于 FreeRTOS 节拍，当前 1000 Hz，即每节拍 1 ms. 输入 72 MHz 不等于每秒调度 7200 万次，实际节拍由重装值决定.
- TIM2 提供 HAL 毫秒计时，供 HAL_GetTick、HAL_Delay 和 HAL 串口超时使用. CubeMX 自动生成 stm32f1xx_hal_timebase_tim.c，无需为它配置输出引脚.
- TIM1 硬件持续产生 PWM，任务仅修改比较值. CPU 执行串口任务时，PWM 输出仍在继续.
- HSI 是内部高速 RC；LSE 是可选 32.768 kHz 外部低速晶振；LSI 是内部低速 RC，可给 RTC 和独立看门狗供时钟. 图上选中某时钟源，不代表对应外设已启用.
- MCO 用于把内部时钟输出到引脚，不是呼吸灯 PWM.
- 本次不使用 USB、ADC. 图中灰色 USB 72 MHz 和 ADC 36 MHz 不是可直接使用的配置：启用 USB 应配置 48 MHz，STM32F103 ADC 时钟应不超过 14 MHz.

图 06 · CubeMX 时钟树原图：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/08_FreeRTOS%E5%9F%BA%E7%A1%80/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/06_CubeMX%E6%97%B6%E9%92%9F%E6%A0%91%E5%8E%9F%E5%9B%BE.png).

图 07 · 时钟树用途中文批注：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/08_FreeRTOS%E5%9F%BA%E7%A1%80/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/07_%E6%97%B6%E9%92%9F%E6%A0%91%E4%B8%AD%E6%96%87%E6%89%B9%E6%B3%A8.png).

图 07 为辅助生成的教学批注图，以图 06 和实际配置为准. 重点定位原图右侧 To Cortex System timer（SysTick 输入）及 APB1 Timer clocks（TIM2 所属分支）；图中没有直接显示 TIM2 名称.

## 4. PWM 任务代码

在 Src/freertos.c 的 USER CODE BEGIN Includes 区域添加：

```c
#include "tim.h"
#include "usart.h"
```

在 StartPwmTask 的 USER CODE 区域编写：

```c
uint16_t brightness = 0;
int16_t step = 5;
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

for (;;)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);
    osDelay(5);

    if (brightness >= 1000)
        step = -5;
    else if (brightness == 0)
        step = 5;

    brightness = (uint16_t)((int16_t)brightness + step);
}
```

ARR=999 时，比较值 0～1000 对应约 0%～100% 占空比. 每次改变 5，每步等待 5 个节拍，渐亮、渐暗各约 1 秒，实际受调度影响. LED 接法决定占空比增大时亮度增加还是减少.

for (;;) 是无限循环，每个任务有自己的循环. osDelay 会让当前任务进入等待状态，其他就绪任务可以运行. HAL_Delay 是忙等待，任务中优先使用 osDelay. CMSIS_V2 的 osDelay 参数单位是节拍，不是永远等于毫秒；本工程 1000 Hz 时才对应 1 ms/节拍.

图 04 · PWM 任务代码与编译通过：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/08_FreeRTOS%E5%9F%BA%E7%A1%80/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/04_PWM%E4%BB%BB%E5%8A%A1%E7%BC%96%E8%AF%91%E9%80%9A%E8%BF%87.png).

## 5. 串口轮询任务代码

在 StartUartTask 的 USER CODE 区域编写：

```c
uint8_t rx_byte;
uint8_t welcome[] = "FreeRTOS UART ready\r\n";

HAL_UART_Transmit(&huart1, welcome, sizeof(welcome) - 1, 100);

for (;;)
{
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 1) == HAL_OK)
    {
        HAL_UART_Transmit(&huart1, &rx_byte, 1, 10);
    }
    osDelay(1);
}
```

rx_byte 是一个 uint8_t 变量，保存一个接收字节. welcome 是 uint8_t 数组，保存文本及末尾的字符串终止符. sizeof(welcome)-1 不发送末尾的 '\0'.

HAL_UART_Receive 的参数分别是串口句柄、接收地址、接收字节数、超时毫秒数. HAL_UART_Transmit 的参数分别是串口句柄、数据地址、发送字节数、超时毫秒数. 超时参数不是发送间隔.

此代码每次接收一个字节，再把该字节原样回传，称为回显. HAL 的轮询等待仍会占用 CPU，osDelay 才会让当前任务等待；但抢占调度仍能让更高优先级的就绪任务执行.

## 6. 接线、下载与测试

| STM32 | CH340 / LED |
| --- | --- |
| PA9（TX） | CH340 RXD |
| PA10（RX） | CH340 TXD |
| GND | CH340 GND，共地 |
| PA8 | 串联限流电阻连接外接 LED |

CH340 使用与 STM32 匹配的 3.3 V 串口电平. PA8 接线沿用原呼吸灯实验.

1. Keil 按 F7 编译，确认没有错误，然后下载程序.
2. VOFA 选择 RawData，选择实际 CH340 COM 口，配置 115200、8N1、无流控并连接.
3. 若要观察启动提示，应先打开串口再按板子复位键，否则提示可能在软件连接前已发完.
4. 先单个发送英文字符，观察 Rx 回传，并确认呼吸灯持续变化.
5. 关闭本地回显，或区分 Tx 与 Rx，避免把发送显示误认为单片机回传.
6. 连续发送 ABCD 等文字，记录实际丢字节现象.

图 05 · 串口回显与丢字节现象：[GitHub 图片直达链接](https://github.com/Wang-666-0/hunan-tiger-embedded-assessment/blob/main/08_FreeRTOS%E5%9F%BA%E7%A1%80/02_%E8%BF%87%E7%A8%8B%E8%AE%B0%E5%BD%95/05_%E4%B8%B2%E5%8F%A3%E5%9B%9E%E6%98%BE%E4%B8%8E%E4%B8%A2%E5%AD%97%E8%8A%82%E7%8E%B0%E8%B1%A1.png).

## 7. 为什么发四个字符只回来两个

115200、8N1 的一个字符需要起始位、8 个数据位和停止位，共 10 位，传输时间约 10/115200=87 μs. 连续四个字符约 0.35 ms 就可以发完.

程序接收一个字节后要回传，并 osDelay(1) 等待约一个节拍. 期间其他字节已经到达，当前接收没有 DMA 或中断缓冲，可能产生 ORE 接收溢出和丢字节. 回复两个不是硬件的固定两字节限制，结果随到达时序变化. PWM 任务优先级更高也可能进一步推迟串口任务处理.

中文 UTF-8 通常一个汉字占多个字节，丢字节会破坏字符编码；发送与显示编码不一致也会乱码. 截图里的重复内容还可能混合了软件 Tx/Rx 显示，不能仅凭画面推断每个字符都已回传.

正确改进方向：用接收中断或 DMA 把收到的数据先存入缓冲区，再由串口任务处理发送. 必要时用队列或通知联系任务，中断中只做短时间工作. 本次尚未实现和验证这项改进，不能写成已解决. 简单删掉 osDelay 也不保证连续收发可靠.

## 8. 常用操作与验收复习

| 操作 / 函数 | 用途 |
| --- | --- |
| CMSIS_V2 | 本次使用的 RTOS 接口 |
| osThreadNew | 创建任务，CubeMX 已自动生成 |
| osKernelInitialize / osKernelStart | 初始化与启动内核，由生成代码调用 |
| osDelay | 让任务等待指定节拍 |
| HAL_TIM_PWM_Start | 启动 PWM 通道 |
| __HAL_TIM_SET_COMPARE | 改变比较值，调整占空比 |
| HAL_UART_Receive / HAL_UART_Transmit | 当前轮询版本收发 |
| HAL_GetTick | 获取 HAL 毫秒计数，本次由 TIM2 提供 |

验收时可先展示双任务配置、TIM2 HAL 时基、任务函数，再现场展示呼吸灯与串口交互. 连续收发可靠性应在改为中断/DMA 后再次测试. 不把当前丢字符版本描述成完整稳定通信.

参考：[ST CubeMX 配置说明](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.1/en/docs/markup/CubeMX_UserManual/chapters/04_4_stm32cubemx_user_interface.html)，[STM32F1 参考手册 RM0008](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).