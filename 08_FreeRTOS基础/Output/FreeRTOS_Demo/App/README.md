# FreeRTOS 基础：模块阅读与验收

## 从哪里开始

`Src/freertos.c` 保留 CubeMX 生成的线程属性、创建代码及 StartPwmTask/StartUartTask 入口；入口分别调用两个 App 模块。

| 文件 / 函数 | 职责与参数 |
| --- | --- |
| breath_led.c / BreathLed_Task(argument) | TIM1 CH1 呼吸灯任务；argument 是任务创建时的用户参数，当前不用 |
| uart_echo.c / UartEcho_Task(argument) | 轮询接收一个串口字节，以 rx:数值输出；argument 当前不用 |

## 要看懂的部分

- brightness 是 TIM1 的 CCR 比较值，step=±5 控制变亮/变暗方向，范围 0..1000。
- `osDelay(5)` 是相对延时，CMSIS_V2 参数单位为 tick；当前 tick=1000 Hz，对应约 5 ms。
- 基础任务保持原实现。对周期要求更高的写法见第 10 项的 `vTaskDelayUntil`。
- 串口任务每轮接收最多等待 1 ms，再主动延时，让 PWM 任务获得运行机会。
- snprintf 返回所需字符数，所以只有长度大于 0 且小于缓冲区容量时才发送。

## 验收

呼吸灯持续工作，同时通过原串口发送字符。例如发送 A，FireWater 数值帧为 `rx:65`，65 是 A 的字节值。启动的 INFO 行是提示日志。修改灯光算法看 breath_led.c，修改回显内容看 uart_echo.c。
