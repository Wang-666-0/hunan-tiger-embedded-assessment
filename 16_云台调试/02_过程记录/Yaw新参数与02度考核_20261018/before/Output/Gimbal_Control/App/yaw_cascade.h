#ifndef YAW_CASCADE_H
#define YAW_CASCADE_H
#include <stdint.h>

/* 用户明确取消双环180deg/s及0.10协议Nm业务限制。
 * 1仅供V2A离线对照，生产0；协议编码的±10范围不能无限扩展。 */
#ifndef YAW_CASCADE_OUTPUT_LIMITS_ENABLED
#define YAW_CASCADE_OUTPUT_LIMITS_ENABLED 0U
#endif

typedef struct { float kp, ki, kd; } YawCascadePid;
/* 调用方根据规划加速度计算、限幅后的前馈力矩。Pitch不使用此结构。 */
typedef struct {
    float torque_nm;
    uint8_t limited;
} YawCascadeFeedforward;
typedef struct {
    float velocity_dps;
    uint8_t freeze_position_i;
} YawCascadeReference;
typedef struct {
    YawCascadePid position, speed;
    float velocity_limit_dps, velocity_dps, acceleration_dps2, target_velocity_dps;
    /* 积分状态直接保存输出单位：外环deg/s，内环协议Nm。 */
    float position_i, speed_i, torque_nm, unsaturated_torque_nm;
    uint32_t revision, feedback_ms, sequence;
    float feedforward_nm;
    uint8_t have_update, torque_limited, velocity_limited, feedforward_limited;
} YawCascade;

#define YAW_CASCADE_VELOCITY_FILTER_MS 10U
#define YAW_CASCADE_ACCEL_FILTER_MS 20U
void YawCascade_Init(YawCascade *loop);
/* 清控制历史，保留参数及已观测速度。重复OFF不破坏速度估计。 */
void YawCascade_Stop(YawCascade *loop);
void YawCascade_ResetObserver(YawCascade *loop);
/* 只在新有效CAN反馈调用，delta来自已解环的位置差，时间来自CAN接收戳。 */
void YawCascade_Observe(YawCascade *loop, float delta_deg, uint32_t gap_ms);
/* P=位置环，V=速度环。位置：deg->deg/s；速度：deg/s->协议Nm。
 * D均对测量微分，避免目标阶跃的微分冲击；相同参数写入不清积分。 */
uint8_t YawCascade_SetPid(YawCascade *loop, char which, float kp, float ki, float kd);
uint8_t YawCascade_SetVelocityLimit(YawCascade *loop, float dps);
/* 2ms调用可即时响应新目标，积分仅按新反馈实际dt推进一次。
 * 生产torque_limit_nm应为协议映射端点10，不是原0.10业务限制。 */
float YawCascade_Update(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm);
/* 前馈参加力矩求和及抗饱和；原Yaw入口始终传0。单位均为协议Nm。 */
float YawCascade_UpdateWithFeedforward(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm, float feedforward_nm);
/* Yaw平滑阶跃专用：已知参考速度进入外环，D对速度跟踪误差作用。
 * 仅规划运动段冻结外环积分；内环积分/抗饱和仍按实际CAN反馈更新。 */
float YawCascade_UpdateTracking(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                         uint32_t sequence, float torque_limit_nm, const YawCascadeFeedforward *ff,
                         const YawCascadeReference *reference);
#endif
