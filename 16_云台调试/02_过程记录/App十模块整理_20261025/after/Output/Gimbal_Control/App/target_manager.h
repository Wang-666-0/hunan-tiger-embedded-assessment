/**
 * @file target_manager.h
 * @brief 运动与发射目标管理
 * 职责：定义执行器请求、阶跃边沿序号及单发/连发目标节拍。
 * 主要接口：TargetManager_Init, TargetManager_RecordAcceptedTestAxis, TargetManager_Step。
 * 实现/调用：remote_input.c；目标请求不代表最终输出许可，后端仍逐项校验。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef TARGET_MANAGER_H
#define TARGET_MANAGER_H
#include "app_config.h"
#include "remote_mapper.h"

typedef struct {
    float yaw_rate_dps, pitch_rate_dps, flywheel_rpm, step_delta_deg;
    uint32_t shot_sequence, step_sequence;
    FireMode fire_mode;
    GimbalMode gimbal_mode;
    uint8_t friction_requested, shot_event, step_event;
    GimbalMode step_axis;
    uint8_t step_return_event, wheel_reset_allowed; /* CH7 up: relative -30, not HOME. */
    uint8_t continuous_fire; /* Live qualified hold, cleared on release/guard failure. */
} TargetRequest;

typedef struct {
    TargetRequest request;
    uint32_t next_shot_ms;
    FireMode last_fire_mode;
    GimbalMode last_gimbal_mode;
    uint8_t trigger_low_seen, step_center_seen, firing;
    uint8_t wheel_off_seen;
    RcSwitch last_step_switch;
    GimbalMode last_test_axis; /* 后端真正接受的阶跃轴；初始Yaw，选轴杆居中时用于CH7上拨后退。 */
} TargetManager;

void TargetManager_Init(TargetManager *manager);
/* 控制集成层仅在对应后端确实接受本次遥控阶跃后调用；拒绝事件不改变此轴。 */
void TargetManager_RecordAcceptedTestAxis(TargetManager *manager, GimbalMode axis);
/* 事件只在本周期有效；序号用于记录次数，不是待执行队列。
 * flywheels_ready必须来自正式驱动反馈，基础版恒为0。 */
void TargetManager_Step(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms);
#endif
