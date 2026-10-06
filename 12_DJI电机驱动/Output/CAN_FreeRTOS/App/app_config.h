#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 复盘或调整时，应用参数集中在这里。 */
#define GM6020_MODE_VOLTAGE 0U // 协议模式编号 0：电压给定。
#define GM6020_MODE_CURRENT 1U // 协议模式编号 1：电流给定。
#ifndef GM6020_MOTOR_ID
#define GM6020_MOTOR_ID 1U // 必须与电机前三位物理拨码一致；范围 1～7。
#endif
#ifndef GM6020_CONTROL_MODE
#define GM6020_CONTROL_MODE GM6020_MODE_VOLTAGE // 默认发送电压协议，不能改变电机内部电流环开关。
#endif

/* 此处只选择 MCU 发送的协议，不能替代 Assistant 中的电流环开关。
 * 默认电压模式；电流模式要求手册注明的固件版本及电流环设置。 */
#define MOTOR_VOLTAGE_DEMO_LIMIT 2000 // 教学电压给定范围为 ±2000，超限直接拒绝。
#define MOTOR_CURRENT_DEMO_LIMIT 1000 // 教学电流给定范围为 ±1000。
#define MOTOR_CONTROL_PERIOD_MS 2U // 控制周期 2 ms，名义 500 Hz。
#define MOTOR_FEEDBACK_TIMEOUT_MS 200U // 反馈年龄达到 200 ms 时判为离线。
#define MOTOR_COMMAND_TIMEOUT_MS 3000U // 三秒没有新给定或 K 续期则撤销请求。
#define MOTOR_TEMPERATURE_LIMIT_C 80U // 本工程温度阈值，不是电机内置保护参数。
#define MOTOR_TELEMETRY_PERIOD_MS 20U // 每 20 ms 发九通道遥测，名义 50 Hz。
#define MOTOR_CONSOLE_PERIOD_MS 5U // 每 5 ms 处理一轮 UART 字节队列。
#define UART_INPUT_TIMEOUT_MS 1000U /* 一条命令的字节不能积压或间隔太久。 */
#define UART_TRANSMIT_TIMEOUT_MS 20U /* 同步发送一帧文本最多等待的毫秒数。 */
#define CAN_RX_QUEUE_LENGTH 32U // 最多缓冲 32 条完整 CAN 报文。
#define UART_RX_QUEUE_LENGTH 128U // 最多缓冲 128 个字节及其时间和代次。
#define UART_COMMAND_BUFFER_SIZE 32U // 31 个命令字符加一个字符串结束符。

/* 原生 FreeRTOS 的栈单位为 word，F405 上一个 word 为 4 字节。 */
#define CAN_RX_STACK_WORDS 256U // 接收任务栈 1024 字节，原生 RTOS 单位为 word。
#define MOTOR_CONTROL_STACK_WORDS 384U // 控制任务栈 1536 字节。
#define MOTOR_CONSOLE_STACK_WORDS 512U // 串口任务栈 2048 字节，容纳格式化局部缓冲。

/* 任务优先级数字越大越紧急：先接收，再控制，最后处理串口。 */
#define CAN_RX_TASK_PRIORITY 4U // 接收优先级最高，及时处理反馈。
#define MOTOR_CONTROL_TASK_PRIORITY 3U // 控制优先级居中，保持周期给定。
#define MOTOR_CONSOLE_TASK_PRIORITY 2U // 串口优先级最低，格式化不抢占控制任务。

/* 额外诊断默认关闭；只在明确需要原始帧、硬件采样、栈余量时打开。
 * 关闭后仍保留电机反馈、串口结果和基本收发统计，不影响正常功能。 */
#ifndef APP_ENABLE_DIAGNOSTICS
#define APP_ENABLE_DIAGNOSTICS 0U // 默认不编译原始帧副本、硬件采样和栈余量记录。
#endif
#define APP_DIAGNOSTIC_PERIOD_MS 100U // 仅启用额外诊断时使用的采样周期。

#if GM6020_MOTOR_ID < 1U || GM6020_MOTOR_ID > 7U
#error GM6020_MOTOR_ID_must_be_1_to_7
#endif
#if GM6020_CONTROL_MODE != GM6020_MODE_VOLTAGE && GM6020_CONTROL_MODE != GM6020_MODE_CURRENT
#error Invalid_GM6020_CONTROL_MODE
#endif

#endif /* APP_CONFIG_H */
