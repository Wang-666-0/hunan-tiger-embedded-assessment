#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "gm6020.h"

/* 转速单环的任务接口；完整控制过程由 MotorControl_Step 统一执行。 */
typedef enum
{
    MOTOR_STOP_NONE = 0,            // 转速闭环正在启用，包括目标为 0 rpm 的情况。
    MOTOR_STOP_POWER_ON = 1,        // 上电默认停止。
    MOTOR_STOP_USER = 2,            // 用户发送 STOP。
    MOTOR_STOP_FEEDBACK = 3,        // 反馈超时。
    MOTOR_STOP_RESERVED_4 = 4,      // 保留旧编号；本版取消命令超时，不会产生此原因。
    MOTOR_STOP_TEMPERATURE = 5,     // 温度达到限值。
    MOTOR_STOP_CAN_TX = 6,          // 发送失败或旧邮箱挂起。
    MOTOR_STOP_UART_RX = 7,         // 串口丢字节或输入已经过期。
    MOTOR_STOP_CONTROL_TIMING = 8   // 控制任务运行间隔过长。
} MotorStopReason;

typedef struct
{
    float target_rpm;     // 用户要求的转速，S 命令更新，单位 rpm。
    float actual_rpm;     // 本次控制使用的 CAN 反馈转速，单位 rpm。
    float error_rpm;      // target_rpm - actual_rpm。
    float kp, ki, kd;     // 当前设置的连续时间增益；串口可在线修改。
    float p_term;         // 本轮比例项，单位为协议原始给定。
    float i_term;         // 本轮积分项，已经包含 Ki，单位为原始给定。
    float d_term;         // 本轮微分项，来自实际转速变化，单位为原始给定。
    int16_t output_raw;   // 本轮 PID 限幅后的整数给定，不是实际转速。
    int16_t last_sent_raw; // 最近提交 CAN 邮箱的值，不保证已被电机执行。
    uint32_t enabled;     // 1 为闭环控制；STOP 或故障后为 0。
    uint32_t online;      // 当前反馈是否新鲜。
    uint32_t mode;        // GM6020 电压或电流协议，本工程默认电压。
    uint32_t motor_id;    // 电机拨码 ID。
    uint32_t stop_reason; // MotorStopReason 枚举。
    uint32_t cycle_count; // 执行过的控制步骤数。
    uint32_t last_tx_id;  // 最近提交的标准 CAN ID。
    uint8_t last_tx_data[8]; // 最近提交的八字节命令。
} MotorControl_State;

extern volatile MotorControl_State motor_control;
/* 下列接口供任务调用，不能直接在中断回调里执行。 */
void MotorControl_Init(void);
uint8_t MotorControl_SetSpeed(float rpm); // 锁存目标并持续运行；S0 也闭环，Stop 才完全停用。
uint8_t MotorControl_SetGain(char term, float value); // P/I/D 在线更新，保留历史，不启动电机。
uint8_t MotorControl_KeepAlive(void); // 兼容旧 K：仅返回是否运行，不再要求定时发送。
void MotorControl_Stop(MotorStopReason reason);
void MotorControl_Step(void);
void MotorControl_GetState(MotorControl_State *snapshot);

#endif /* MOTOR_CONTROL_H */
