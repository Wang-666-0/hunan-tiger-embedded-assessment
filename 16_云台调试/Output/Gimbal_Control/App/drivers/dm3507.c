/**
 * @file dm3507.c
 * @brief DM3507 固定协议编解码
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（5）：
 *   Dm3507_Decode / Dm3507_MakeEnable / Dm3507_MakeDisable / Dm3507_MakeReadFeedback /
 *   Dm3507_MakeTorque
 * 内部函数（4）：
 *   encode / decode / init_frame / make_special
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* DM3507 协议编解码。 */
#include "dm3507.h"
#include <float.h>
#include <string.h>

/* 达妙官方motor-sdk/Python例程/u2can/DM_CAN.py：controlMIT、
 * __process_packet、__control_cmd、refresh_motor_status。
 * 温度字节按官方STM32例程dm_motor_fbdata的Tmos/Tcoil保留uint8，
 * data[7]在本项目命名temp_rotor，不复用GL有符号温度。
 * https://github.com/dmBots/motor-control-routine/blob/master/stm32%E4%BE%8B%E7%A8%8B/dm_ctrl(f4)%20v1.1%20%E8%A3%B8%E6%9C%BA/User/dm_motor_drv.c */
static uint16_t encode(float value, float maximum, uint32_t scale)
/* 功能：把物理量限幅并按给定端点量化为协议无符号整数。 */
{
    if (value <= -maximum) { return 0U; }
    if (value >= maximum) { return (uint16_t)scale; }
    return (uint16_t)((value + maximum) * (float)scale / (2.0f * maximum));
}
static float decode(uint16_t value, float maximum, uint32_t scale)
/* 功能：按协议端点把无符号整数还原为有符号物理量。 */
{ return (float)value * (2.0f * maximum) / (float)scale - maximum; }
static uint8_t init_frame(CanFrame *out)
/* 功能：初始化 DM3507 专用 CAN 帧头与负载。 */
{
    if (!out) { return 0U; }
    memset(out, 0, sizeof(*out));
    out->bus = DM3507_CAN_BUS; out->id = DM3507_MOTOR_ID; out->dlc = 8U;
    return 1U;
}
uint8_t Dm3507_Decode(const CanFrame *frame, Dm3507Feedback *out)
/* 功能：识别 DM3507 有效反馈帧，解出位置、速度、力矩、状态和温度。 */
{
    Dm3507Feedback result;
    uint16_t position, velocity, torque;
    if (!out || !CanFrame_IsValid(frame) || frame->bus != DM3507_CAN_BUS ||
        frame->extended || frame->id != DM3507_MASTER_ID || frame->dlc != 8U ||
        (frame->data[0] & 0x0FU) != DM3507_MOTOR_ID) { return 0U; }
    /* 参数回复以完整slave uint16_le和操作字开头。仅检查data[2]
     * 会误拒正常位置低字节0x33/0x55，故连同slave高低字节匹配。 */
    if (frame->data[0] == DM3507_MOTOR_ID && frame->data[1] == 0U &&
        (frame->data[2] == 0x33U || frame->data[2] == 0x55U)) { return 0U; }
    position = (uint16_t)(((uint16_t)frame->data[1] << 8) | frame->data[2]);
    velocity = (uint16_t)(((uint16_t)frame->data[3] << 4) | (frame->data[4] >> 4));
    torque = (uint16_t)(((uint16_t)(frame->data[4] & 0x0FU) << 8) | frame->data[5]);
    memset(&result, 0, sizeof(result));
    result.position_rad = decode(position, DM3507_POSITION_MAX_RAD, 65535U);
    result.velocity_rad_s = decode(velocity, DM3507_VELOCITY_MAX_RAD_S, 4095U);
    result.torque_nm = decode(torque, DM3507_TORQUE_MAX_NM, 4095U);
    result.status = frame->data[0] >> 4; result.motor_id = DM3507_MOTOR_ID;
    result.temp_mos = frame->data[6]; result.temp_rotor = frame->data[7];
    result.received_ms = frame->received_ms; result.fault_epoch = frame->fault_epoch;
    *out = result;
    return 1U;
}
static uint8_t make_special(uint8_t command, CanFrame *out)
/* 功能：构造 DM3507 固定特殊命令帧，不提供任意设备写入入口。 */
{
    if (!init_frame(out)) { return 0U; }
    memset(out->data, 0xFF, sizeof(out->data)); out->data[7] = command;
    return 1U;
}
uint8_t Dm3507_MakeEnable(CanFrame *out)
/* 功能：构造 DM3507 固定使能帧，发送许可由上层负责。 */
{ return make_special(0xFCU, out); }
uint8_t Dm3507_MakeDisable(CanFrame *out)
/* 功能：构造 DM3507 固定失能帧。 */
{ return make_special(0xFDU, out); }
uint8_t Dm3507_MakeReadFeedback(CanFrame *out)
/* 功能：构造 DM3507 固定反馈查询帧。 */
{
    if (!init_frame(out)) { return 0U; }
    out->id = 0x7FFU; out->data[0] = DM3507_MOTOR_ID; out->data[2] = 0xCCU;
    return 1U;
}
uint8_t Dm3507_MakeTorque(float torque_nm, CanFrame *out)
/* 功能：按 DM3507 协议生成纯力矩帧，内部位置/速度增益保持为零。 */
{
    uint16_t position, velocity, torque;
    if (!out || torque_nm != torque_nm || torque_nm > FLT_MAX || torque_nm < -FLT_MAX) { return 0U; }
    position = encode(0.0f, DM3507_POSITION_MAX_RAD, 65535U);
    velocity = encode(0.0f, DM3507_VELOCITY_MAX_RAD_S, 4095U);
    torque = encode(torque_nm, DM3507_TORQUE_MAX_NM, 4095U);
    (void)init_frame(out);
    out->data[0] = (uint8_t)(position >> 8); out->data[1] = (uint8_t)position;
    out->data[2] = (uint8_t)(velocity >> 4);
    out->data[3] = (uint8_t)((velocity & 0x0FU) << 4);
    out->data[6] = (uint8_t)(torque >> 8); out->data[7] = (uint8_t)torque;
    return 1U;
}
