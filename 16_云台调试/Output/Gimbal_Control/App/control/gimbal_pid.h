/* 两轴共用串级 PID */
#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H
#include "app_config.h"
#include <stdint.h>

typedef struct { float kp, ki, kd; } GimbalPidGains;
typedef struct {
    GimbalPidGains position, speed;
    float velocity_limit_dps, velocity_dps, acceleration_dps2, target_velocity_dps;
    /* 积分状态直接保存输出单位：外环deg/s，内环协议Nm。 */
    float position_i, speed_i, torque_nm, unsaturated_torque_nm;
    uint32_t revision, feedback_ms, sequence;
    float feedforward_nm;
    uint8_t have_update, torque_limited, velocity_limited, feedforward_limited;
} GimbalPid;

void GimbalPid_Init(GimbalPid *loop);
/* 清控制历史，保留参数及已观测速度。重复OFF不破坏速度估计。 */
void GimbalPid_Stop(GimbalPid *loop);
void GimbalPid_ResetObserver(GimbalPid *loop);
/* 只在新有效CAN反馈调用，delta来自已解环的位置差，时间来自CAN接收戳。 */
void GimbalPid_Observe(GimbalPid *loop, float delta_deg, uint32_t gap_ms);
/* P=位置环，V=速度环。位置：deg->deg/s；速度：deg/s->协议Nm。
 * D均对测量微分，避免目标阶跃的微分冲击；相同参数写入不清积分。 */
uint8_t GimbalPid_SetPid(GimbalPid *loop, char which, float kp, float ki, float kd);
/* 2ms调用可即时响应新目标，积分仅按新反馈实际dt推进一次。
 * 生产torque_limit_nm应为协议映射端点10，不是原0.10业务限制。 */
float GimbalPid_Update(GimbalPid *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm);
/* 前馈参加力矩求和及抗饱和；原Yaw入口始终传0。单位均为协议Nm。 */
float GimbalPid_UpdateWithFeedforward(GimbalPid *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm, float feedforward_nm);
#endif
