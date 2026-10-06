#ifndef MOTOR_CONSOLE_H
#define MOTOR_CONSOLE_H

#include <stdint.h>

typedef enum
{
    MOTOR_COMMAND_NONE = 0, // 尚未处理命令。
    MOTOR_COMMAND_OK = 1, // 接受命令。
    MOTOR_COMMAND_SYNTAX = 2, // 格式错误。
    MOTOR_COMMAND_MODE = 3, // 命令模式与编译模式不同。
    MOTOR_COMMAND_RANGE = 4, // 超教学限幅。
    MOTOR_COMMAND_NOT_READY = 5, // 在线/温度/续期条件不满足。
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
    uint32_t last_result; // 最近命令结果，FireWater 通道 8。
    uint32_t tx_frame_count; // 成功发送的遥测帧数。
    uint32_t tx_error_count; // 遥测发送失败次数。
    uint32_t format_error_count; // snprintf 失败或截断次数。
} MotorConsole_Debug;

extern volatile MotorConsole_Debug motor_console_debug;

void MotorConsole_CreateQueue(void);
void MotorConsole_Start(void);
void MotorConsole_Poll(void);
void MotorConsole_SendTelemetry(void);
/* 行内不能包含换行。指令格式：V500 / V-500 / C500 / STOP / K。 */
void MotorConsole_ProcessLine(const char *line);

#endif /* MOTOR_CONSOLE_H */
