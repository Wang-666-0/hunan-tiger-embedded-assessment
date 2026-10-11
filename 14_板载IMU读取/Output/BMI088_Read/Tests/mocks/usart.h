#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { uint32_t placeholder; } UART_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, uint8_t *bytes, uint16_t count, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *handle);
#endif
