/**
 * @file gl60_ii.h
 * @brief GL60 II 协议接口
 * 职责：定义 Yaw 电机反馈及力矩、使能、失能、清错报文接口。
 * 主要接口：Gl60_Decode, Gl60_BuildTorque, Gl60_BuildEnable, Gl60_BuildDisable, Gl60_BuildClearError。
 * 实现：drivers/gl60_ii.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef GL60_II_H
#define GL60_II_H
#include "app_config.h"

#include "can_frame.h"

/* GL60 II, Firmware4407/002, MIT mode, actual saved mapping ranges.
 * Mapping ranges are neither mechanical limits nor authorized outputs. */

typedef struct {
    float position_rad, velocity_rad_s, torque_nm;
    int8_t driver_temp_c, motor_temp_c;
    uint8_t status, motor_id;
    uint32_t received_ms, fault_epoch;
} Gl60Feedback;

/* CAN1, standard Master ID0, DLC8, payload ID1 only.
 * Disabled/enabled (0/1) and errors (8..14) decode successfully: callers must
 * examine status before granting output. Invalid input leaves out unchanged. */
uint8_t Gl60_Decode(const CanFrame *frame, Gl60Feedback *out);

/* Pure MIT torque command: p=v=kp=kd=0, finite ff clamped to +/-10 Nm.
 * NaN/Infinity/null output are rejected without changing out.
 * These constructors do not transmit or authorize power. The CAN send layer
 * and controller must enforce the much smaller current-stage torque limit.
 * Symmetric odd integer ranges cannot represent exact zero: truncation maps
 * zero to p=32767, v=t=2047. Torque is approximately -0.002442 Nm; with
 * kp=kd=0, position/velocity reference quantization does not add feedback gain.
 * Stopping must use the verified disable command, not an assumed zero torque. */
uint8_t Gl60_BuildTorque(float torque_nm, CanFrame *out);
void Gl60_BuildEnable(CanFrame *out);
void Gl60_BuildDisable(CanFrame *out);
/* 构造手册FB；仅供安全OFF下人工恢复通信故障，调用方限定状态13。 */
void Gl60_BuildClearError(CanFrame *out);

/* No calibration, persistent-zero or parameter-write command. */
#endif
