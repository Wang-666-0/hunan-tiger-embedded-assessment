/**
 * @file dm3507.h
 * @brief DM3507 协议接口
 * 职责：定义 Pitch 电机反馈，声明使能、失能、读取反馈和力矩报文编码。
 * 主要接口：Dm3507_Decode, Dm3507_MakeEnable, Dm3507_MakeDisable, Dm3507_MakeReadFeedback, Dm3507_MakeTorque。
 * 实现/调用：can_bus.c；角度 rad、速度 rad/s、力矩 N*m，协议端点集中在 app_config.h。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef DM3507_H
#define DM3507_H
#include "app_config.h"
#include "can_frame.h"
#include <stdint.h>

/* 本机2026-10-11实读，不采用SDK中其他电机型号的默认量程。 */

typedef struct {
    float position_rad, velocity_rad_s, torque_nm;
    uint32_t received_ms, fault_epoch;
    uint8_t status, temp_mos, temp_rotor, motor_id;
} Dm3507Feedback;

/* 原始协议坐标；减速比是否已由驱动换算由Pitch参考/坐标流程确认。
 * status来自ERR高四位；0失能、1使能，其他值交由控制器锁故障。
 * 参数查询/写回复不能延长运动反馈寿命。 */
uint8_t Dm3507_Decode(const CanFrame *frame, Dm3507Feedback *out);
uint8_t Dm3507_MakeEnable(CanFrame *out);
uint8_t Dm3507_MakeDisable(CanFrame *out);
/* 固定只读查询：ID0x7FF，02 00 CC 00 00 00 00 00。 */
uint8_t Dm3507_MakeReadFeedback(CanFrame *out);
/* 外部双环输出协议Nm；MIT内部KP/KD均0，只裁协议可表达的±10。 */
uint8_t Dm3507_MakeTorque(float torque_nm, CanFrame *out);
#endif
