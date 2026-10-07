#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "gm6020.h"

/* 双环任务接口：角度外环输出 rpm，转速内环输出协议 raw。
 * S 命令仍可单独运行转速内环，以便先调好内环再调角度环。
 */
typedef enum
{
    MOTOR_CONTROL_SPEED_MODE = 0,
    MOTOR_CONTROL_ANGLE_MODE = 1
} MotorControl_Mode;

typedef enum
{
    MOTOR_STOP_NONE = 0,            // 速度或角度闭环正在启用；S0/A0 都仍是闭环。
    MOTOR_STOP_POWER_ON = 1,        // 上电默认停止。
    MOTOR_STOP_USER = 2,            // 用户发送 STOP。
    MOTOR_STOP_FEEDBACK = 3,        // 反馈超时。
    MOTOR_STOP_RESERVED_4 = 4,      // 保留旧编号；本版取消命令超时，不会产生此原因。
    MOTOR_STOP_TEMPERATURE = 5,     // 温度达到限值。
    MOTOR_STOP_CAN_TX = 6,          // 发送失败或旧邮箱挂起。
    MOTOR_STOP_UART_RX = 7,         // 串口丢字节或输入已经过期。
    MOTOR_STOP_CONTROL_TIMING = 8,  // 控制任务运行间隔过长。
    MOTOR_STOP_ANGLE_INVALID = 9    // 多圈角度失效或零点代次变化；需停止后重新 Z。
} MotorStopReason;

typedef struct
{
    float target_rpm;     // S 模式为用户目标，A 模式为外环输出，单位 rpm。
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
    float target_angle_deg; // A 命令的多圈角度，单位度，相对于 Z 零点。
    float actual_angle_deg; // 本次使用的多圈反馈角度，不按 360 度取模。
    float error_angle_deg;  // target_angle_deg - actual_angle_deg。
    float angle_kp, angle_ki, angle_kd; // 外环连续时间增益，输出单位 rpm。
    float angle_p_term, angle_i_term, angle_d_term; // 外环三项，单位 rpm。
    float speed_limit_rpm; // V 命令限制角度外环，不改变 S 命令的范围。
    uint32_t control_mode; // MotorControl_Mode，区别于 GM6020 协议 mode。
    uint32_t angle_valid; // 新鲜且连续的多圈反馈；掉线后不能沿用旧圈数。
    uint32_t settled; // 本次 A 指令在 ±0.5 度内持续保持 50 ms 时为 1。
    uint32_t settling_time_ms; // 持续入带的首次入带时刻减命令时刻；settled=1 才有效。
    float overshoot_deg; // 沿本次阶跃方向超过目标的最大角度。
} MotorControl_State;

extern volatile MotorControl_State motor_control;
/* 下列接口供任务调用，不能直接在中断回调里执行。 */
void MotorControl_Init(void);
uint8_t MotorControl_SetSpeed(float rpm); // 锁存目标并持续运行；S0 也闭环，Stop 才完全停用。
uint8_t MotorControl_SetGain(char term, float value); // P/I/D 在线更新，保留历史，不启动电机。
uint8_t MotorControl_SetAngle(float degrees); // A：相对 Z 的绝对多圈目标，A720 是两整圈。
uint8_t MotorControl_ZeroAngle(void); // Z：必须先 STOP，并收到新鲜且静止的反馈。
uint8_t MotorControl_SetAngleGain(char term, float value); // AP/AI/AD：外环增益，不启动。
uint8_t MotorControl_SetSpeedLimit(float rpm); // V：角度环输出的最大转速，1～100 rpm。
uint8_t MotorControl_KeepAlive(void); // 兼容旧 K：仅返回是否运行，不再要求定时发送。
void MotorControl_Stop(MotorStopReason reason);
void MotorControl_Step(void);
void MotorControl_GetState(MotorControl_State *snapshot);

#endif /* MOTOR_CONTROL_H */
