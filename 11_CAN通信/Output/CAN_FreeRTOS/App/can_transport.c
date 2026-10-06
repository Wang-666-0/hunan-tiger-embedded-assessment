#include "can_transport.h"
#include "can_debug.h"
#include "queue.h"

/* FIFO 是 CAN 硬件缓冲；本队列是 ISR 与任务之间的软件缓冲，两者不同。 */
static QueueHandle_t can_rx_queue;

void CanTransport_CreateQueue(void)
{
    can_rx_queue = xQueueCreate(16U, sizeof(CanRxMessage));
    if (can_rx_queue == NULL)
        CanFail(CAN_FAULT_QUEUE);
    vQueueAddToRegistry(can_rx_queue, "CAN_RX_QUEUE");
}

BaseType_t CanTransport_Receive(CanRxMessage *message, TickType_t wait_ticks)
{
    /* wait_ticks 是 RTOS tick；调用者用 portMAX_DELAY 等待报文，不轮询忙等。 */
    return xQueueReceive(can_rx_queue, message, wait_ticks);
}

void CanStart(void)
{
    CAN_FilterTypeDef filter = {0};
    const uint32_t notifications =
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_TX_MAILBOX_EMPTY;

    /* CAN1、CAN2 共用 28 个滤波器组，以第 14 组作为分界。 */
    filter.SlaveStartFilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    /* 掩码为零，硬件接收所有 ID；中断再筛选标准数据帧。 */
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;

    filter.FilterBank = 0;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        CanFail(CAN_FAULT_FILTER1);
    }
    filter.FilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
    {
        CanFail(CAN_FAULT_FILTER2);
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        CanFail(CAN_FAULT_START1);
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        CanFail(CAN_FAULT_START2);
    }
    if (HAL_CAN_ActivateNotification(&hcan1, notifications) != HAL_OK)
    {
        CanFail(CAN_FAULT_NOTIFY1);
    }
    if (HAL_CAN_ActivateNotification(&hcan2, notifications) != HAL_OK)
    {
        CanFail(CAN_FAULT_NOTIFY2);
    }
    can_ready = 1;
}

void CanSend(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t marker, uint32_t sequence)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    /* 后四字节为小端序号，可检查报文是否持续更新。 */
    uint8_t data[8];
    CanProtocol_Encode(marker, sequence, data);

    /* 无空闲邮箱时跳过本帧，避免发送任务忙等。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        if (hcan == &hcan1)
        {
            can1_tx_busy_count++;
        }
        else
        {
            can2_tx_busy_count++;
        }
        return;
    }

    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;

    /* HAL_OK 只表示已提交发送邮箱；真正完成由 TxMailbox 回调计数。 */
    if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK)
    {
        if (hcan == &hcan1)
        {
            can1_tx_queued_count++;
        }
        else
        {
            can2_tx_queued_count++;
        }
    }
    else
    {
        if (hcan == &hcan1)
        {
            can1_tx_submit_error_count++;
        }
        else
        {
            can2_tx_submit_error_count++;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t task_woken = pdFALSE;
    uint8_t source;

    if (hcan == &hcan1)
    {
        source = 1;
    }
    else if (hcan == &hcan2)
    {
        source = 2;
    }
    else
    {
        return;
    }

    /* 取出 FIFO0 的待收报文；中断只搬运数据，不做阻塞操作。 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        CAN_RxHeaderTypeDef header;
        CanRxMessage message = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, message.data) != HAL_OK)
        {
            can_rx_read_error_count++;
            break;
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA)
        {
            can_rx_ignored_count++;
            continue;
        }
        if (header.DLC > 8U)
        {
            can_rx_read_error_count++;
            continue;
        }
        message.source = source;
        message.id = header.StdId;
        message.dlc = (uint8_t)header.DLC;
        for (uint32_t i = message.dlc; i < 8U; i++)
        {
            message.data[i] = 0;
        }
        /* FromISR 不阻塞并复制结构体，局部 message 的生命周期不会影响队列。
         * CAN 中断优先级配置为 5，符合当前 FreeRTOS 的系统调用优先级限制。 */
        if (xQueueSendFromISR(can_rx_queue, &message, &task_woken) != pdPASS)
        {
            can_rx_queue_full_count++;
        }
    }
    /* 必要时在退出中断后立即运行被唤醒的接收任务。 */
    portYIELD_FROM_ISR(task_woken);
}

static void CanTxCompleteCount(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        can1_tx_count++;
    }
    else if (hcan == &hcan2)
    {
        can2_tx_count++;
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CanTxCompleteCount(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CanTxCompleteCount(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CanTxCompleteCount(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    /* 当前使能的通知可报告发送邮箱和接收 FIFO 溢出等错误。 */
    if (hcan == &hcan1)
    {
        can1_error_callback_count++;
        can1_last_hal_error = HAL_CAN_GetError(hcan);
    }
    else if (hcan == &hcan2)
    {
        can2_error_callback_count++;
        can2_last_hal_error = HAL_CAN_GetError(hcan);
    }
}
