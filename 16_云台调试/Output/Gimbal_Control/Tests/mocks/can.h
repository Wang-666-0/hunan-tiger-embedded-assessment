#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdint.h>
typedef struct { uint32_t instance; } CAN_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef struct {
    uint32_t FilterMode, FilterScale, FilterFIFOAssignment, FilterActivation;
    uint32_t SlaveStartFilterBank, FilterBank, FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
} CAN_FilterTypeDef;
typedef struct { uint32_t IDE, ExtId, StdId, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t IDE, ExtId, StdId, RTR, DLC; } CAN_RxHeaderTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RX_FIFO0 0U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_FILTER_FIFO0 0U
#define ENABLE 1U
#define DISABLE 0U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_RX_FIFO0_OVERRUN 2U
#define CAN_IT_TX_MAILBOX_EMPTY 4U
#define CAN_IT_ERROR 8U
#define CAN_IT_BUSOFF 16U
#define CAN_IT_ERROR_PASSIVE 32U
#define CAN_IT_ERROR_WARNING 64U
#define CAN_IT_LAST_ERROR_CODE 128U
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *handle, const CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *handle, uint32_t notifications);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *handle, uint32_t mailboxes);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle, const CAN_TxHeaderTypeDef *header,
    const uint8_t *data, uint32_t *mailbox);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *handle, uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *handle, uint32_t fifo,
    CAN_RxHeaderTypeDef *header, uint8_t *data);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *handle);
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *handle, uint32_t mailboxes);
uint32_t HAL_GetTick(void);
#endif
