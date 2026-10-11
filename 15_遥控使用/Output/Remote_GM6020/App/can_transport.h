#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#include "can.h"
#include "can_protocol.h"
#include "FreeRTOS.h"

void CanTransport_CreateQueue(void);
/* 队列创建、调度器运行后调用一次，只启动 CAN1。 */
void CanStart(void);
BaseType_t CanTransport_Receive(CanRxMessage *message, TickType_t wait_ticks);
/* 任务调用。HAL_OK 仅代表提交到邮箱，不代表电机已执行。 */
HAL_StatusTypeDef CanTransport_Send(uint32_t id, const uint8_t data[8]);
/* 取消还在邮箱中等待发送的旧给定，避免断线恢复后重发过期指令。 */
void CanTransport_AbortPending(void);

#endif /* CAN_TRANSPORT_H */
