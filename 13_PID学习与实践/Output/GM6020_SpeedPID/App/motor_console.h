/*
 * 文件功能：声明 USART1 转速 PID 命令、FireWater 遥测与串口接收接口。
 * 控制命令只在完整行收到后执行；串口错误统计中的故障代次还参与恢复逻辑。
 * 默认 T1 包含目标/实际、控制量、PID 参数和运行状态；T0 只输出两条曲线。
 * 转速保留两位小数，PID 增益保留四位小数；最近命令结果位于 T1 的通道 9。
 */
#ifndef MOTOR_CONSOLE_H
#define MOTOR_CONSOLE_H

#include <stdint.h>

typedef enum
{
    MOTOR_COMMAND_NONE = 0, // 尚未处理命令。
    MOTOR_COMMAND_OK = 1, // 接受命令。
    MOTOR_COMMAND_SYNTAX = 2, // 格式错误。
    MOTOR_COMMAND_MODE = 3, // 保留旧协议枚举值，本项工程不使用 V/C 模式命令。
    MOTOR_COMMAND_RANGE = 4, // 数值超目标转速或 PID 参数范围。
    MOTOR_COMMAND_NOT_READY = 5, // 在线/温度条件不满足，或停止时发送兼容 K。
    MOTOR_COMMAND_STALE = 6, // 命令字节陈旧或片段间隔过大。
    MOTOR_COMMAND_RX_FAULT = 7 // 接收丢字节或 HAL 错误。
} MotorCommandResult;

typedef struct
{
    uint32_t rx_byte_count; // 接收回调收到的字节数。
    uint32_t rx_drop_count; // 队列满时丢失的字节数。
    uint32_t rx_error_count; // UART 接收错误次数。
    uint32_t rx_rearm_fail_count; // 单字节续接收失败次数。
    uint32_t rx_fault_epoch; // 每次接收故障递增；用于隔离错误前后的命令片段。
    uint32_t last_uart_error; // 最近 HAL UART 错误标志。
    uint32_t command_count; // 交给整行解析的命令数。
    uint32_t rejected_count; // 被拒绝的命令或组行错误数。
    uint32_t last_result; // 最近命令结果，T1 扩展遥测的通道 9。
    uint32_t telemetry_mode; // 默认 1：参数和状态共十通道；0：目标/实际两通道。
    uint32_t tx_frame_count; // 成功发送的遥测帧数。
    uint32_t tx_error_count; // 遥测发送失败次数。
    uint32_t format_error_count; // snprintf 失败或截断次数。
} MotorConsole_Debug;

extern volatile MotorConsole_Debug motor_console_debug;

void MotorConsole_CreateQueue(void);
void MotorConsole_Start(void);
void MotorConsole_Poll(void);
void MotorConsole_SendTelemetry(void);
/* 行内不能包含换行。指令：S30 / S-30 / S0 / P5 / I0.5 / D0 / STOP / K / T0 / T1。
 * S 的单位是 rpm，S0 保持零转速闭环；STOP 才撤销控制请求。
 * P/I/D 只修改参数，不启动电机。S 一次发送后持续运行，不需要 K。
 * 每条串口指令用 CR 或 LF 结束。 */
void MotorConsole_ProcessLine(const char *line);

#endif /* MOTOR_CONSOLE_H */
