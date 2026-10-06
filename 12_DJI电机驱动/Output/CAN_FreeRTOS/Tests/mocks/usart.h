#ifndef TEST_USART_H
#define TEST_USART_H
#include "can.h"
typedef struct { uint32_t unused; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data, uint16_t length, uint32_t timeout);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *h);
uint32_t HAL_GetTick(void);
#endif
