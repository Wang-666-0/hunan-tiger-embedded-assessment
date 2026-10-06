#ifndef UART_SERVICE_H
#define UART_SERVICE_H

#include "main.h"

void UartService_Init(void);
void UartService_Task(void *argument);
HAL_StatusTypeDef UartSend(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size,
                           uint32_t timeout);
/* 接收队列满时丢失的字节数，保留原有 Watch 名称。 */
extern volatile uint32_t uart_rx_dropped;

#endif /* UART_SERVICE_H */
