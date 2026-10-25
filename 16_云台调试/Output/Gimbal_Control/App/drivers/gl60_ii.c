/**
 * @file gl60_ii.c
 * @brief GL60 II 固定协议编解码
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（5）：
 *   Gl60_Decode / Gl60_BuildTorque / Gl60_BuildEnable / Gl60_BuildDisable / Gl60_BuildClearError
 * 内部函数（5）：
 *   EncodeSymmetric / DecodeSymmetric / DecodeTemperature / InitCommand / BuildSpecial
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* GL60 II 协议编解码。 */
#include "gl60_ii.h"
#include <float.h>
#include <string.h>

/* GL II drive manual MIT payload (sections 5.3/5.4): command p16/v12/kp12/kd12/t12;
 * feedback status4/id4, p16/v12/t12, signed driver/motor temperature bytes.
 * The manual's illustrative pointer/range expressions are not reused. */
static uint16_t EncodeSymmetric(float value, float maximum, uint32_t full_scale)
/* 功能：对称限幅物理量并按给定量程编码为协议整数。 */
{
    if (value <= -maximum) { return 0U; }
    if (value >= maximum) { return (uint16_t)full_scale; }
    return (uint16_t)((value + maximum) * (float)full_scale / (2.0f * maximum));
}

static float DecodeSymmetric(uint16_t raw, float maximum, uint32_t full_scale)
/* 功能：把协议整数按对称量程转换为物理量。 */
{
    return (float)raw * (2.0f * maximum) / (float)full_scale - maximum;
}

static int8_t DecodeTemperature(uint8_t raw)
/* 功能：把 GL60 温度字节转换为有符号摄氏温度。 */
{
    int16_t value = raw < 128U ? (int16_t)raw : (int16_t)raw - 256;
    return (int8_t)value;
}

static void InitCommand(CanFrame *out)
/* 功能：初始化 GL60 II 专用 CAN 命令帧头。 */
{
    memset(out, 0, sizeof(*out));
    out->bus = 1U;
    out->id = 1U;
    out->dlc = 8U;
}

uint8_t Gl60_Decode(const CanFrame *frame, Gl60Feedback *out)
/* 功能：识别 GL60 II 有效反馈，解出实际位置、速度、力矩和状态信息。 */
{
    Gl60Feedback decoded;
    uint16_t position, velocity, torque;
    uint8_t status;
    if (out == 0 || !CanFrame_IsValid(frame)) { return 0U; }
    if (frame->bus != 1U || frame->extended != 0U || frame->id != 0U ||
        frame->dlc != 8U || (frame->data[0] & 0x0FU) != 1U) { return 0U; }
    status = frame->data[0] >> 4;
    if (status != 0U && status != 1U && (status < 8U || status > 14U)) { return 0U; }
    position = (uint16_t)(((uint16_t)frame->data[1] << 8) | frame->data[2]);
    velocity = (uint16_t)(((uint16_t)frame->data[3] << 4) | (frame->data[4] >> 4));
    torque = (uint16_t)(((uint16_t)(frame->data[4] & 0x0FU) << 8) | frame->data[5]);
    memset(&decoded, 0, sizeof(decoded));
    decoded.position_rad = DecodeSymmetric(position, GL60_POSITION_MAX_RAD, 65535U);
    decoded.velocity_rad_s = DecodeSymmetric(velocity, GL60_VELOCITY_MAX_RAD_S, 4095U);
    decoded.torque_nm = DecodeSymmetric(torque, GL60_TORQUE_MAX_NM, 4095U);
    decoded.driver_temp_c = DecodeTemperature(frame->data[6]);
    decoded.motor_temp_c = DecodeTemperature(frame->data[7]);
    decoded.status = status;
    decoded.motor_id = 1U;
    decoded.received_ms = frame->received_ms;
    decoded.fault_epoch = frame->fault_epoch;
    *out = decoded;
    return 1U;
}

uint8_t Gl60_BuildTorque(float torque_nm, CanFrame *out)
/* 功能：生成 GL60 II 纯力矩控制帧，内部位置/速度增益保持为零。 */
{
    uint16_t position, velocity, torque;
    /* ISO C comparisons avoid relying on platform-specific isfinite macros. */
    if (out == 0 || torque_nm != torque_nm || torque_nm > FLT_MAX || torque_nm < -FLT_MAX) {
        return 0U;
    }
    position = EncodeSymmetric(0.0f, GL60_POSITION_MAX_RAD, 65535U);
    velocity = EncodeSymmetric(0.0f, GL60_VELOCITY_MAX_RAD_S, 4095U);
    torque = EncodeSymmetric(torque_nm, GL60_TORQUE_MAX_NM, 4095U);
    InitCommand(out);
    out->data[0] = (uint8_t)(position >> 8);
    out->data[1] = (uint8_t)position;
    out->data[2] = (uint8_t)(velocity >> 4);
    out->data[3] = (uint8_t)((velocity & 0x0FU) << 4); /* kp high nibble = 0. */
    out->data[4] = 0U; /* kp low byte. */
    out->data[5] = 0U; /* kd high byte. */
    out->data[6] = (uint8_t)(torque >> 8); /* kd low nibble = 0. */
    out->data[7] = (uint8_t)torque;
    return 1U;
}

static void BuildSpecial(uint8_t command, CanFrame *out)
/* 功能：构造 GL60 II 固定特殊命令负载。 */
{
    if (out == 0) { return; }
    InitCommand(out);
    memset(out->data, 0xFF, sizeof(out->data));
    out->data[7] = command;
}

void Gl60_BuildEnable(CanFrame *out)
/* 功能：构造 GL60 II 固定使能帧。 */
{ BuildSpecial(0xFCU, out); }
void Gl60_BuildDisable(CanFrame *out)
/* 功能：构造 GL60 II 固定失能帧。 */
{ BuildSpecial(0xFDU, out); }
void Gl60_BuildClearError(CanFrame *out)
/* 功能：构造 GL60 II 固定清错帧，实际清错门禁由后端检查。 */
{ BuildSpecial(0xFBU, out); }
