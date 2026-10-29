/* 遥控语义映射 */
#ifndef REMOTE_MAPPER_H
#define REMOTE_MAPPER_H
#include "app_config.h"
#include <stdint.h>

typedef enum { RC_LOW = 0, RC_MIDDLE = 1, RC_HIGH = 2 } RcSwitch;
typedef enum { FIRE_SAFE = 0, FIRE_SINGLE = 1, FIRE_CONTINUOUS = 2 } FireMode;
typedef enum { GIMBAL_MANUAL = 0, GIMBAL_STEP_YAW = 1, GIMBAL_STEP_PITCH = 2,
    GIMBAL_TEST = 3 } GimbalMode;

typedef struct {
    uint16_t channels[14];
    uint32_t received_ms, sequence;
    uint8_t has_frame;
} RemoteFrame;

typedef struct {
    float yaw_rate_dps, pitch_rate_dps;
    RcSwitch arm, trigger, step;
    FireMode fire_mode;
    GimbalMode gimbal_mode;
    uint8_t valid, failsafe_ok, friction_on, neutral;
    RcSwitch wheel_switch; /* CH6原始档位，用于遥控启停适配层的OFF确认。 */
    GimbalMode step_axis;
    float step_delta_deg;
    uint8_t step_valid; /* 左摇杆只允许单轴方向；斜向/回中不生成阶跃。 */
    uint8_t step_selection_centered; /* 只有CH3/4均在死区内才复用上次实际接受的轴。 */
} RemoteCommand;

/* 只转换通道含义，不使能电机；单位为云台角速度deg/s和逻辑开关。 */
void RemoteMapper_Map(const RemoteFrame *frame, RemoteCommand *command);
#endif
