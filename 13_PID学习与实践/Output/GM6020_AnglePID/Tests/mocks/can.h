#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdint.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { uint32_t unused; } CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan1;
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan);
#endif
