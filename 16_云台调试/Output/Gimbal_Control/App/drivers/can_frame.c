/**
 * @file can_frame.c
 * @brief CAN 数据帧基本校验
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（1）：
 *   CanFrame_IsValid
 * 内部函数（0）：
 *   无。
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 通用帧检查：与具体设备状态无关。 */
#include "can_frame.h"
uint8_t CanFrame_IsValid(const CanFrame *frame)
/* 功能：检查总线编号、标准/扩展标志、帧 ID 和数据长度是否有效。 */
{
    if (frame == 0 || (frame->bus != 1U && frame->bus != 2U) ||
        frame->extended > 1U || frame->dlc > 8U) { return 0U; }
    return frame->id <= (frame->extended ? 0x1FFFFFFFU : 0x7FFU);
}
