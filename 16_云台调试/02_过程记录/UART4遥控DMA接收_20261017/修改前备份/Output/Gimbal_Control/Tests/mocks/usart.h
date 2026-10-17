#ifndef TEST_USART_H
#define TEST_USART_H
#include "can.h"
typedef struct { uint32_t instance, error; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart4;
#define HAL_UART_ERROR_NONE 0U
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *handle);
HAL_StatusTypeDef ConsoleUart_StartReceive(uint8_t *data, uint16_t length);
#endif
