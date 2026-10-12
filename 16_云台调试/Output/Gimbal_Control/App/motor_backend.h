#ifndef MOTOR_BACKEND_H
#define MOTOR_BACKEND_H
#include "can_frame.h"
#include "target_manager.h"
typedef struct {
    uint8_t ready, feedback_ready, flywheels_ready, actuator_fault;
} MotorHealth;
void MotorBackend_GetHealth(MotorHealth *health);
uint8_t MotorBackend_OnFrame(const CanFrame *frame);
/* 返回1才表示请求已交给正式驱动；基础版一律返回0，并且不发送任何报文。 */
uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t output_allowed);
#endif
