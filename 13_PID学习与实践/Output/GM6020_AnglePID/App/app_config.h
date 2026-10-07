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
#define GM6020_VOLTAGE_RAW_LIMIT 25000 // 新版官方 CAN 电压给定协议上限，不是伏特或 PWM。
#define GM6020_CURRENT_RAW_LIMIT 16384 // 官方 CAN 电流给定协议上限。
#define MOTOR_VOLTAGE_DEMO_LIMIT 15000 // 速度实验电压给定限幅；为 100 rpm 留出比旧 ±2000 更大的余量。
#define MOTOR_CURRENT_DEMO_LIMIT 1000 // 教学电流给定范围为 ±1000。
#define MOTOR_CONTROL_PERIOD_MS 2U // 控制周期 2 ms，名义 500 Hz。
#define MOTOR_FEEDBACK_TIMEOUT_MS 20U // 反馈应每 1 ms 到达；闭环超过 20 ms 没有新反馈则停机。
/* S 命令锁存目标：持续运行到 STOP 或实际故障，不需要上位机循环发 K。
 * 上电仍默认停止；断开电脑本身不会停机，实验结束必须发送 STOP。 */
#define MOTOR_TEMPERATURE_LIMIT_C 80U // 本工程温度阈值，不是电机内置保护参数。
#define MOTOR_TELEMETRY_PERIOD_MS 20U // 21 通道文本每 20 ms 一帧，50 Hz；避免 115200 串口带宽不足。
#define MOTOR_TELEMETRY_DEFAULT_MODE 1U // 默认角度/转速/双环增益和状态共 21 通道。
#define MOTOR_CONSOLE_PERIOD_MS 5U // 每 5 ms 处理一轮 UART 字节队列。
#define UART_INPUT_TIMEOUT_MS 1000U /* 一条命令的字节不能积压或间隔太久。 */
#define UART_TRANSMIT_TIMEOUT_MS 20U /* 同步发送一帧文本最多等待的毫秒数。 */
#define CAN_RX_QUEUE_LENGTH 32U // 最多缓冲 32 条完整 CAN 报文。
#define UART_RX_QUEUE_LENGTH 128U // 最多缓冲 128 个字节及其时间和代次。
#define UART_COMMAND_BUFFER_SIZE 32U // 31 个命令字符加一个字符串结束符。

/* 转速单环参数：增益通过串口修改，掉电后恢复以下初值。
 * 输出是 GM6020 协议原始给定；初值仅用于开始实验，不保证已调好。 */
#define MOTOR_SPEED_LIMIT_RPM 100.0f // S 命令允许 ±100 rpm，先用低转速实验。
#define SPEED_PID_DEFAULT_KP 50.0f // 沿用当前实机调参值；原始给定 / rpm。
#define SPEED_PID_DEFAULT_KI 200.0f // 沿用当前实机调参值；原始给定 / (rpm·s)。
#define SPEED_PID_DEFAULT_KD 0.003f // 沿用最近实机截图的调参值；原始给定·s / rpm。
#define SPEED_PID_KP_MAX 200.0f // 串口参数范围上限，不代表推荐调到这个值。
#define SPEED_PID_KI_MAX 1000.0f // 积分参数输入上限。
#define SPEED_PID_KD_MAX 10.0f // 微分参数输入上限。
/* 零稳态误差时 P≈0、D≈0，需要由 I 项独立维持电机所需的给定。
 * 积分限幅跟随最终输出限幅，避免总输出还有余量，I 却提前卡在 1000。
 * Ki 决定累积速度，不是积分项上限；最终输出限幅和抗饱和判断仍保留。 */
#if GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE
#define SPEED_PID_INTEGRAL_LIMIT ((float)MOTOR_VOLTAGE_DEMO_LIMIT) // 电压模式 I 项跟随最终输出，当前 ±15000。
#else
#define SPEED_PID_INTEGRAL_LIMIT ((float)MOTOR_CURRENT_DEMO_LIMIT) // 电流模式跟随 ±1000 输出限制。
#endif
#define SPEED_PID_DERIVATIVE_TAU_S 0.02f // 微分一阶滤波时间常数 20 ms。
#define MOTOR_CONTROL_MAX_GAP_MS 10U // 运行中控制间隔超过 10 ms 则停机，避免迟到控制继续施力。

/* 多圈角度 = 连续编码器差分累计；第一帧/STOP 后 Z 建立当前零位。
 * 反馈丢失后不知道是否跨过整圈，角度有效标志锁定为无效，需停机 Z。
 * 400 rpm 是本工程判定编码器跳变的宽松上界，不是电机额定速度说明。
 */
#define ANGLE_TRACK_MAX_GAP_MS MOTOR_FEEDBACK_TIMEOUT_MS // >=20 ms 的反馈间断不再猜测多圈位置。
#define MOTOR_PHYSICAL_SPEED_LIMIT_RPM 400.0f // 用于排除不可信的编码器跳变。
#define ANGLE_TRACK_MAX_COUNTS 16384000LL // 多圈累计保护边界，正负各 2000 圈，防止无限累计。
#define ANGLE_ZERO_SPEED_RPM 2.0f // Z 要求已 STOP，且最新反馈 |转速|<=2 rpm。
#define ANGLE_TARGET_LIMIT_DEG 7200.0f // 目标相对于 Z 的绝对多圈角度；±20 圈，覆盖 720°要求。

/* 角度外环输出是 rpm，速度内环输出才是电压原始给定。
 * AP/AI/AD 只改角度外环；P/I/D 仍只改速度内环，两套增益单位不同。
 * 外环起始值需要实机调参，软件通过不代表 30°/300 ms 已达标。
 */
#define ANGLE_PID_DEFAULT_KP 4.0f // rpm/度：30°误差将请求120 rpm，再由V限速。
#define ANGLE_PID_DEFAULT_KI 0.0f // rpm/(度·秒)：外环先关闭积分。
#define ANGLE_PID_DEFAULT_KD 0.05f // rpm·秒/度：对实际角度变化进行阻尼。
#define ANGLE_PID_KP_MAX 20.0f
#define ANGLE_PID_KI_MAX 100.0f
#define ANGLE_PID_KD_MAX 2.0f
#define ANGLE_PID_INTEGRAL_LIMIT_RPM 30.0f // 外环积分产生的目标转速贡献限幅。
#define ANGLE_PID_DERIVATIVE_TAU_S 0.02f // 外环微分低通20 ms。
#define ANGLE_DEFAULT_SPEED_LIMIT_RPM 100.0f // V命令可把角度模式最大速度设为1～100 rpm。
#define ANGLE_BRAKE_SPEED_ERROR_RPM 5.0f // 角度减速时，实际速度比同方向请求高出此值才触发一次清内环I。
#define ANGLE_BRAKE_MIN_SPEED_RPM 2.0f // 低速小波动不反复触发制动积分清理。
#define ANGLE_SETTLE_ERROR_DEG 0.5f // 考核静态误差带；该指标不会强行锁定或改写角度反馈。
#define ANGLE_SETTLE_HOLD_MS 50U // 连续在误差带内50 ms后确认，报告首次进入的时间。

/* 原生 FreeRTOS 的栈单位为 word，F405 上一个 word 为 4 字节。 */
#define CAN_RX_STACK_WORDS 256U // 接收任务栈 1024 字节，原生 RTOS 单位为 word。
#define MOTOR_CONTROL_STACK_WORDS 512U // 控制任务栈 2048 字节，容纳两个PID副本和角度状态。
#define MOTOR_CONSOLE_STACK_WORDS 768U // 串口任务栈 3072 字节，容纳21通道文本帧。

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
#if MOTOR_VOLTAGE_DEMO_LIMIT <= 0 || MOTOR_VOLTAGE_DEMO_LIMIT > GM6020_VOLTAGE_RAW_LIMIT
#error Voltage_output_limit_must_be_within_GM6020_protocol
#endif
#if MOTOR_CURRENT_DEMO_LIMIT <= 0 || MOTOR_CURRENT_DEMO_LIMIT > GM6020_CURRENT_RAW_LIMIT
#error Current_output_limit_must_be_within_GM6020_protocol
#endif

#endif /* APP_CONFIG_H */
