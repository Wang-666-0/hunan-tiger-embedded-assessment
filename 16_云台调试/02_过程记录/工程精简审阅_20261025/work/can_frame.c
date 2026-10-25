/* 通用帧检查：与具体设备状态无关。 */
#include "can_frame.h"
uint8_t CanFrame_IsValid(const CanFrame *frame)
{
    if (frame == 0 || (frame->bus != 1U && frame->bus != 2U) ||
        frame->extended > 1U || frame->dlc > 8U) { return 0U; }
    return frame->id <= (frame->extended ? 0x1FFFFFFFU : 0x7FFU);
}
