#ifndef MOTOR_BACKEND_H
#define MOTOR_BACKEND_H
#include "can_frame.h"
#include "target_manager.h"
#include "flywheel_jog.h"
typedef struct {
    uint8_t ready, feedback_ready, flywheels_ready, actuator_fault;
} MotorHealth;
typedef struct {
    FlywheelJogHealth health;
    FlywheelJogSelection selection;
    int32_t requested_mA;
    FlywheelSpeed speed;
    FlywheelSpeed speed_right;
    int32_t requested_left_mA, requested_right_mA;
    FlywheelStopReason stop_reason;
    FlywheelJogSelection last_speed_selection;
    uint8_t active, rearm_ready, stop_pending;
} MotorJogState;
void MotorBackend_Init(void);
uint8_t MotorBackend_SelectJog(FlywheelJogSelection selection);
void MotorBackend_Stop(void);
uint8_t MotorBackend_TakeRearmRequest(void);
void MotorBackend_GetJogState(MotorJogState *state);
void MotorBackend_GetHealth(MotorHealth *health);
uint8_t MotorBackend_OnFrame(const CanFrame *frame);
/* 返回1表示本次受限点动指令已提交；不表示轮子已经运动或停止。 */
uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t output_allowed);
#endif
