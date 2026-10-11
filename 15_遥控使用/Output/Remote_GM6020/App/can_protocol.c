/*
 * 文件功能：CAN 报文中的通用字节转换工具。
 * 1. 将两个大端字节（高字节在前）还原为无符号或有符号 16 位数。
 * 2. 将有符号 16 位数拆成两个大端字节，用于填写电机控制报文。
 * 模块关系：gm6020_codec.c 调用这些工具解包反馈、打包命令。
 * 阅读重点：左移 8 位与按位或用于拼接字节；负数按 16 位补码处理。
 * 本文件只做数值转换，不负责 CAN 硬件收发，也不判断电机 ID。
 */
#include "can_protocol.h"

uint16_t CanProtocol_ReadU16BE(const uint8_t *data)//读取大端字节为无符号 16 位数
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]); // 先提升类型，再将高字节左移八位，与低字节合并。
}

int16_t CanProtocol_ReadI16BE(const uint8_t *data)//读取大端字节为有符号 16 位数
{
    uint16_t raw = CanProtocol_ReadU16BE(data); // 先按无符号方式取出完整的 16 位原始数。
    /* 用 32 位数计算补码负值，避免依赖超范围 unsigned->signed 转换。 */
    int32_t signed_value = (int32_t)raw; // 用 32 位中间量计算，能容纳减去 65536 的结果。
    if (raw > 32767U) // 16 位补码最高位为 1，说明原始数代表负数。
    {
        signed_value -= 65536L; // 例如 0xFF9C=65436，减去 65536 得到 -100。
    }
    return (int16_t)signed_value; // 结果已落在 -32768～32767，转为有符号 16 位返回。
}

void CanProtocol_WriteI16BE(uint8_t *data, int16_t value)//将有符号 16 位数拆成大端字节
{
    /* 负数转 uint16_t 按模 65536 转换，得到其 16 位补码。 */
    uint16_t raw = (uint16_t)value; // 保留原值的 16 位补码，正负数均可按字节拆分。
    data[0] = (uint8_t)(raw >> 8); // 先写高八位，符合 GM6020 的大端字节顺序。
    data[1] = (uint8_t)(raw & 0xFFU); // 再用掩码保留并写入低八位。
}
