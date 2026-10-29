/**
 * @file can_frame.c
 * 作用：CAN 数据帧基本校验
 * 对外函数：
 *   CanFrame_IsValid
 * 内部函数：
 *   无。
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "can_frame.h"

uint8_t CanFrame_IsValid(const CanFrame *frame)
/*
 * 作用：检查总线编号、标准/扩展标志、帧 ID 和数据长度是否有效。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 返回布尔判断结果，非零为满足条件，0 为不满足。
 */
{
    if (frame == 0 || (frame->bus != 1U && frame->bus != 2U) ||
        frame->extended > 1U || frame->dlc > 8U) { return 0U; }
    return frame->id <= (frame->extended ? 0x1FFFFFFFU : 0x7FFU);
}
