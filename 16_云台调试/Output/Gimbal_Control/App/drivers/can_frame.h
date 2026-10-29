/* 通用 CAN 帧 */
#ifndef CAN_FRAME_H
#define CAN_FRAME_H
#include "app_config.h"
#include <stdint.h>
typedef struct {
    uint32_t id, received_ms, fault_epoch;
    uint8_t bus, extended, dlc, data[8];
} CanFrame;
/* bus为1/2，extended为0/1。只承载经典CAN数据帧，不承载远程帧。 */
uint8_t CanFrame_IsValid(const CanFrame *frame);
#endif
