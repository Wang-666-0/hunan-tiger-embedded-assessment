#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
#define HAL_UART_ERROR_NONE 0x00U
#define HAL_UART_ERROR_PE 0x01U
#define HAL_UART_ERROR_NE 0x02U
#define HAL_UART_ERROR_FE 0x04U
#define HAL_UART_ERROR_ORE 0x08U
typedef struct { uint32_t ErrorCode; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart4;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data, uint16_t length, uint32_t timeout);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *h);
uint32_t HAL_GetTick(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);
#endif
