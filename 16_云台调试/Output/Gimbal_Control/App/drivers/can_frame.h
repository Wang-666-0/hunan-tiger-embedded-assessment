/**
 * @file can_frame.h
 * @brief 通用 CAN 帧
 * 职责：定义含总线、接收时间和故障代次的帧结构，并声明帧合法性检查。
 * 主要接口：CanFrame_IsValid。
 * 实现：drivers/can_frame.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
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
