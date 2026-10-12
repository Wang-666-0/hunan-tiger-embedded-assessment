#ifndef CAN_BUS_H
#define CAN_BUS_H
#include "can_frame.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
typedef struct {
    uint32_t rx_frames[2], tx_frames[2], errors[2], last_error[2];
    uint32_t dropped, fault_epoch;
    uint8_t started;
} CanBusState;
void CanBus_CreateQueue(void);
void CanBus_Start(void);
BaseType_t CanBus_Receive(CanFrame *frame, TickType_t wait_ticks);
void CanBus_GetState(CanBusState *state);
void CanBus_SetOutputPermission(uint8_t allowed);
/* 为后续驱动保留的唯一发送入口。基础版编译门禁固定关闭。 */
HAL_StatusTypeDef CanBus_Send(const CanFrame *frame);
/* 独立读接口只构造固定设备/固定读命令，不接受任意CAN报文。 */
HAL_StatusTypeDef CanBus_ReadDm(uint8_t rid);
HAL_StatusTypeDef CanBus_ReadVesc(uint8_t node, uint8_t command);
void CanBus_CancelReads(void);
#endif
