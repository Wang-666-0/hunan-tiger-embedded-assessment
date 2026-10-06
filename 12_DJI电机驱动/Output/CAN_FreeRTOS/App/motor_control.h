#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "gm6020.h"

typedef enum
{
    MOTOR_STOP_NONE = 0, // 有效请求正在启用。
    MOTOR_STOP_POWER_ON = 1, // 上电默认停止。
    MOTOR_STOP_USER = 2, // 用户 STOP 或零给定。
    MOTOR_STOP_FEEDBACK = 3, // 反馈过期。
    MOTOR_STOP_COMMAND_TIMEOUT = 4, // 给定三秒未更新。
    MOTOR_STOP_TEMPERATURE = 5, // 温度达到阈值。
    MOTOR_STOP_CAN_TX = 6, // 发送失败或旧邮箱挂起。
    MOTOR_STOP_UART_RX = 7 // 输入可靠性失效。
} MotorStopReason;

typedef struct
{
    int16_t requested_raw; /* 用户请求。 */
    int16_t output_raw;    /* 本轮准备发送的给定，不是电机实测输出。 */
    int16_t last_sent_raw; /* 最近成功提交邮箱的给定，不保证电机已执行。 */
    uint32_t enabled; // 是否允许使用非零请求；停止后需要新的有效给定才能启用。
    uint32_t online; // 控制任务本轮判断的反馈在线状态。
    uint32_t mode; // 编译选择的电压/电流协议。
    uint32_t motor_id; // 编译选择的目标电机编号。
    uint32_t stop_reason; // MotorStopReason 枚举，串口输出其数值。
    uint32_t last_command_ms; // 最后有效给定或 K 续期的 HAL 毫秒时刻。
    uint32_t cycle_count; // 控制步骤累计执行次数。
    uint32_t last_tx_id; // 最近提交邮箱的标准 CAN ID。
    uint8_t last_tx_data[8]; // 最近提交邮箱的八字节命令，不代表电机已执行。
} MotorControl_State;

extern volatile MotorControl_State motor_control;
/* 以下接口仅供任务调用，不在 ISR 中调用。 */
uint8_t MotorControl_SetOutput(int16_t raw);
uint8_t MotorControl_KeepAlive(void);
void MotorControl_Stop(MotorStopReason reason);
void MotorControl_Step(void);
void MotorControl_GetState(MotorControl_State *snapshot);

#endif /* MOTOR_CONTROL_H */
