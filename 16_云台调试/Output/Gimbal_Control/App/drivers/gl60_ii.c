/**
 * @file gl60_ii.c
 * 作用：GL60 II 固定协议编解码
 * 对外函数：
 *   Gl60_BuildTorque / Gl60_Decode / Gl60_BuildEnable / Gl60_BuildDisable / Gl60_BuildClearError
 * 内部函数：
 *   EncodeSymmetric / DecodeSymmetric / DecodeTemperature / InitCommand / BuildSpecial
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "gl60_ii.h"
#include <float.h>
#include <string.h>

/* GL II drive manual MIT payload (sections 5.3/5.4): command p16/v12/kp12/kd12/t12;
 * feedback status4/id4, p16/v12/t12, signed driver/motor temperature bytes.
 * The manual's illustrative pointer/range expressions are not reused. */

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static uint16_t EncodeSymmetric(float value, float maximum, uint32_t full_scale);
static float DecodeSymmetric(uint16_t raw, float maximum, uint32_t full_scale);
static int8_t DecodeTemperature(uint8_t raw);
static void InitCommand(CanFrame *out);
static void BuildSpecial(uint8_t command, CanFrame *out);

uint8_t Gl60_BuildTorque(float torque_nm, CanFrame *out)
/*
 * 作用：生成 GL60 II 纯力矩控制帧，内部位置/速度增益保持为零。
 * 参数 torque_nm：协议力矩，N*m。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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

uint8_t Gl60_Decode(const CanFrame *frame, Gl60Feedback *out)
/*
 * 作用：识别 GL60 II 有效反馈，解出实际位置、速度、力矩和状态信息。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 返回是否识别并处理了该设备的数据；电机反馈中的故障状态仍由上层检查。
 */
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

void Gl60_BuildEnable(CanFrame *out)
/*
 * 作用：构造 GL60 II 固定使能帧。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ BuildSpecial(0xFCU, out); }

void Gl60_BuildDisable(CanFrame *out)
/*
 * 作用：构造 GL60 II 固定失能帧。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ BuildSpecial(0xFDU, out); }

void Gl60_BuildClearError(CanFrame *out)
/*
 * 作用：构造 GL60 II 固定清错帧，实际清错门禁由后端检查。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ BuildSpecial(0xFBU, out); }

static uint16_t EncodeSymmetric(float value, float maximum, uint32_t full_scale)
/*
 * 作用：对称限幅物理量并按给定量程编码为协议整数。
 * 参数 value：待处理的数值。
 * 参数 maximum：协议物理量的正向量程端点。
 * 参数 full_scale：协议整数最大值。
 * 返回转换或计算后的整数值，供调用方继续处理。
 */
{
    if (value <= -maximum) { return 0U; }
    if (value >= maximum) { return (uint16_t)full_scale; }
    return (uint16_t)((value + maximum) * (float)full_scale / (2.0f * maximum));
}

static float DecodeSymmetric(uint16_t raw, float maximum, uint32_t full_scale)
/*
 * 作用：把协议整数按对称量程转换为物理量。
 * 参数 raw：待解码的协议原始整数。
 * 参数 maximum：协议物理量的正向量程端点。
 * 参数 full_scale：协议整数最大值。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{
    return (float)raw * (2.0f * maximum) / (float)full_scale - maximum;
}

static int8_t DecodeTemperature(uint8_t raw)
/*
 * 作用：把 GL60 温度字节转换为有符号摄氏温度。
 * 参数 raw：待解码的协议原始整数。
 * 返回转换或计算后的整数值，供调用方继续处理。
 */
{
    int16_t value = raw < 128U ? (int16_t)raw : (int16_t)raw - 256;
    return (int8_t)value;
}

static void InitCommand(CanFrame *out)
/*
 * 作用：初始化 GL60 II 专用 CAN 命令帧头。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    memset(out, 0, sizeof(*out));
    out->bus = 1U;
    out->id = 1U;
    out->dlc = 8U;
}

static void BuildSpecial(uint8_t command, CanFrame *out)
/*
 * 作用：构造 GL60 II 固定特殊命令负载。
 * 参数 command：通道映射后的操作命令。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (out == 0) { return; }
    InitCommand(out);
    memset(out->data, 0xFF, sizeof(out->data));
    out->data[7] = command;
}
