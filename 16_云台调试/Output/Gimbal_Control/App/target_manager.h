#ifndef TARGET_MANAGER_H
#define TARGET_MANAGER_H
#include "remote_mapper.h"

typedef struct {
    float yaw_rate_dps, pitch_rate_dps, flywheel_rpm, step_delta_deg;
    uint32_t shot_sequence, step_sequence;
    FireMode fire_mode;
    GimbalMode gimbal_mode;
    uint8_t friction_requested, shot_event, step_event;
} TargetRequest;

typedef struct {
    TargetRequest request;
    uint32_t next_shot_ms;
    FireMode last_fire_mode;
    GimbalMode last_gimbal_mode;
    uint8_t trigger_low_seen, step_center_seen, firing;
} TargetManager;

void TargetManager_Init(TargetManager *manager);
/* 事件只在本周期有效；序号用于记录次数，不是待执行队列。
 * flywheels_ready必须来自正式驱动反馈，基础版恒为0。 */
void TargetManager_Step(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms);
#endif
