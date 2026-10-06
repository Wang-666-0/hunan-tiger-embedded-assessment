#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#include "can.h"
#include "can_protocol.h"
#include "FreeRTOS.h"

void CanTransport_CreateQueue(void);
/* 在调度器运行后调用一次，配置过滤器、启动两个 CAN 并开启通知。 */
void CanStart(void);
/* 只用于本测试的 CAN1/CAN2，id 为标准帧 ID，marker 标记发送方。 */
void CanSend(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t marker, uint32_t sequence);
BaseType_t CanTransport_Receive(CanRxMessage *message, TickType_t wait_ticks);

#endif /* CAN_TRANSPORT_H */
