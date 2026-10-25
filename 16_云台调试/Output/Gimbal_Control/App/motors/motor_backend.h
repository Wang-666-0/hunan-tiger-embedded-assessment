/**
 * @file motor_backend.h
 * @brief 双摩擦轮后端
 * 职责：声明双轮初始化、反馈、遥控准备、健康判定和停机接口。
 * 主要接口：MotorBackend_Init, MotorBackend_SelectJog, MotorBackend_PrepareRemote, MotorBackend_Stop, MotorBackend_TakeRearmRequest 等。
 * 实现：motors/motor_backend.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef MOTOR_BACKEND_H
#define MOTOR_BACKEND_H
#include "app_config.h"
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
/* 新配置只在新鲜、健康的CH5 OFF安全帧下准备双轮；不启动、不改速度环。
 * barrier为已有保护状态机的旧帧屏障，返回1时上层建立新的STOP屏障。 */
uint8_t MotorBackend_PrepareRemote(const RemoteFrame *remote,
    uint32_t barrier_ms, uint8_t barrier_valid);
void MotorBackend_Stop(void);
uint8_t MotorBackend_TakeRearmRequest(void);
void MotorBackend_GetJogState(MotorJogState *state);
void MotorBackend_GetHealth(MotorHealth *health);
uint8_t MotorBackend_OnFrame(const CanFrame *frame);
/* 返回1表示本次受限点动指令已提交；不表示轮子已经运动或停止。 */
uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t output_allowed);
#endif
