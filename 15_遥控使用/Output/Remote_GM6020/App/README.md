# 应用代码阅读顺序

本任务使用 FS-i6X / FS-iA10B 的 iBUS 舵机数据，经 P1 上层 PA1 / UART4_RX 读取遥控通道，经 CAN1 控制 ID 1 的 GM6020 转速。USART1 保留给 VOFA，115200 8N1。

| 文件 | 责任 |
| --- | --- |
| ibus.c / .h | 独立协议库：32字节帧、滑窗重同步、校验和、通道范围检查、帧统计 |
| remote_input.c / .h | UART4 中断将字节、真实毫秒时刻和故障代次入队；任务调用 iBUS 库；提供通道快照 |
| remote_control.c / .h | 不依赖 HAL 的启动许可状态机，OFF→ON、中位、CH6 failsafe、遥控100ms超时 |
| remote_app.c / .h | 把状态机接到反馈和电机目标；临界区处理 STOP，旧帧不能恢复 STOP |
| motor_control.c / .h | 每2ms速度PID，反馈20ms超时、80℃、控制迟到、CAN发送故障停机 |
| pid.c / .h | 数学PID、限幅、抗饱和、微分滤波；在线改增益保留历史 |
| gm6020_codec.c、gm6020.c / .h | 电机协议编解码和完整反馈快照 |
| can_transport.c / .h | CAN1 ISR与队列、邮箱提交与取消；错误IRQ产生故障代次并取消旧邮箱 |
| motor_console.c / .h | USART1 STOP和P/I/D命令；20ms一次固定19通道 FireWater 遥测 |
| can_tasks.c / .h | 原生FreeRTOS任务：CAN接收优先级4，遥控解包优先级4，控制优先级3，控制台优先级2 |
| app_config.h | 通道阈值、±100rpm上限、PID初值、周期、栈和队列大小 |

通道数组下标0对应CH1。CH1中心1500，±30死区；1000和2000分别为−100和+100rpm。CH5低位≤1300为OFF，高位>1700为ON；CH6必须>1700才有运行许可。先在遥控器 Aux channels 将CH5绑定SWA、CH6绑定三档SWB，并把接收机 failsafe 设置CH1=1500、CH5=1000、CH6=1000。iBUS帧本身没有可用的独立RF失联位，所以实际关闭遥控器必须验收CH5/6下降，不能只验收校验和正确。

上电、任何故障和USART1 STOP后，健康的新OFF帧→中位新ON帧才启动。直接上电ON、故障后一直保持ON、非中位ON均不会启动。摇杆回中属于零转速闭环，PID可能输出制动力；SWA OFF属于退出闭环并发送零给定，零给定不等于机械抱闸。

USART1只接受按行命令 STOP、P50、I200、D0.002，CR或LF结尾；没有S启动命令和Flash保存业务。参数掉电恢复app_config.h初值。命令结果可在调试器观察motor_console_debug.last_result；FireWater不混入文字应答。

固定遥测以 remote: 开头，以CRLF结尾，VOFA通道顺序（从0起）是：0–9 CH1–CH10，10目标rpm，11反馈rpm，12原始给定，13iBUS在线，14armed，15距合法帧毫秒数，16故障码，17合法帧计数，18坏帧计数。在线仅表示100ms内有合法帧，实际RF断联依赖上述failsafe许可。故障码：0运行，1上电，2OFF，3遥控超时/无帧，4CH6未许可，5CAN反馈失效，6过温，7UART错误，8控制迟到，9CAN发送/硬件错误，10STOP，11启动时未中位。

## CubeMX配置与重新生成

Remote_GM6020.ioc同步UART4 PA1 RX（AF8）、RX-only、115200 8N1、CAN1 PA11/PA12 1Mbps、USART1 PA9/PA10、CAN1_SCE和UART IRQ优先级5、HSE25MHz/SYSCLK168MHz。PA1配置必须为GPIO_NOPULL，接收5V逻辑时不可启用内部上拉/下拉，不启用PA1模拟功能。P1上层第4脚经100Ω直达PA1，第5脚5V、第6脚GND；TIM2只承担内部HAL时基，不配置PA1为PWM或PPM。

CubeMX只声明defaultTask；应用任务由 freertos.c 的USER CODE区调用 CanTasks_Create，不要再在CubeMX重复添加这些任务。该区同时创建CAN、USART1和iBUS字节队列。App文件内的HAL回调保留为唯一入口：USART1由motor_console处理、UART4转给remote_input；CAN1_SCE由HAL_CAN_IRQHandler处理。

重新生成后，检查main.c包含MX_UART4_Init、stm32f4xx_it.c包含UART4_IRQHandler/CAN1_SCE_IRQHandler、UART4为UART_MODE_RX且只配置PA1 AF8/NOPULL，以及freertos.c USER CODE仍包含三个队列和CanTasks_Create。本工程串口配置在usart.c/.h；若本地CubeMX版本产生uart.c/.h，须同步工程文件表与include，不重复定义huart4。若CubeMX更新Keil工程时遗漏App分组，在Keil重新加入本目录全部.c，并添加../App include目录；不要添加Tests源码。现有eide.yml和Scripts/build.ps1的源文件表已包含完整App。

执行 Scripts/build.ps1 做完整ARM5编译；它读取Remote_GM6020.uvprojx，生成MDK-ARM/build/Remote_GM6020/Remote_GM6020.hex，不连接串口或烧录器。EIDE中打开MDK-ARM/Remote_GM6020.code-workspace也可构建。Tests中的仿真只验证协议和许可状态，真实串口、CAN和断联停止需接线后实机验收。
