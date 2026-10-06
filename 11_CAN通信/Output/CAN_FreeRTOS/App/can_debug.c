#include "can_debug.h"
#include "can.h"
#include "main.h"

/* volatile 使任务/中断写入可供 Watch 观察，避免编译器省略访问。
 * 它不提供多变量原子快照；运行时各字段可能在一次更新的中间被读到。
 * 要比较同一时刻的完整帧，可以暂停后检查。 */
volatile uint32_t can_ready = 0;
volatile uint32_t can_fault_code = 0;
volatile uint32_t can_tx_task_cycle_count = 0;

/* 提交到邮箱和真正发送成功分别计数。 */
volatile uint32_t can1_tx_queued_count = 0;
volatile uint32_t can2_tx_queued_count = 0;
volatile uint32_t can1_tx_count = 0;
volatile uint32_t can2_tx_count = 0;
volatile uint32_t can1_tx_busy_count = 0;
volatile uint32_t can2_tx_busy_count = 0;
volatile uint32_t can1_tx_submit_error_count = 0;
volatile uint32_t can2_tx_submit_error_count = 0;

/* 接收总数、符合本次测试协议的数量、内容不符合的数量。 */
volatile uint32_t can1_rx_count = 0;
volatile uint32_t can2_rx_count = 0;
volatile uint32_t can1_rx_valid_count = 0;
volatile uint32_t can2_rx_valid_count = 0;
volatile uint32_t can1_rx_invalid_count = 0;
volatile uint32_t can2_rx_invalid_count = 0;
volatile uint32_t can1_last_rx_id = 0;
volatile uint32_t can2_last_rx_id = 0;
volatile uint8_t can1_last_rx_dlc = 0;
volatile uint8_t can2_last_rx_dlc = 0;
volatile uint8_t can1_last_rx_data[8] = {0};
volatile uint8_t can2_last_rx_data[8] = {0};
volatile uint32_t can1_last_rx_sequence = 0;
volatile uint32_t can2_last_rx_sequence = 0;

/* 软件异常和 HAL 错误回调统计。 */
volatile uint32_t can_rx_read_error_count = 0;
volatile uint32_t can_rx_queue_full_count = 0;
volatile uint32_t can_rx_ignored_count = 0;
volatile uint32_t can1_error_callback_count = 0;
volatile uint32_t can2_error_callback_count = 0;
volatile uint32_t can1_last_hal_error = 0;
volatile uint32_t can2_last_hal_error = 0;

/* 每轮发送任务采样硬件状态；ESR 包含收发错误计数和 bus-off 标志。 */
volatile uint32_t can1_error_status = 0;
volatile uint32_t can2_error_status = 0;
volatile uint32_t can1_bus_off = 0;
volatile uint32_t can2_bus_off = 0;
volatile uint32_t can1_free_mailboxes = 3;
volatile uint32_t can2_free_mailboxes = 3;

void CanFail(uint32_t code)
{
    can_fault_code = code;
    Error_Handler();
}

void CanDebug_SampleHardware(void)
{
    /* ESR 是当前硬件状态；HAL error 是 HAL 记录的错误，二者用途不同。
     * CAN SCE 未启用时仍能通过每 100 ms 采样的 ESR 观察 bus-off。
     */
    can1_error_status = hcan1.Instance->ESR;
    can2_error_status = hcan2.Instance->ESR;
    can1_bus_off = (can1_error_status & CAN_ESR_BOFF) != 0U;
    can2_bus_off = (can2_error_status & CAN_ESR_BOFF) != 0U;
    can1_last_hal_error = HAL_CAN_GetError(&hcan1);
    can2_last_hal_error = HAL_CAN_GetError(&hcan2);
    can1_free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    can2_free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
}

void CanDebug_RecordRx(const CanRxMessage *message)
{
    uint32_t sequence = (message->dlc == 8U) ? CanDecodeSequence(message->data) : 0U;
    uint32_t valid = CanProtocol_IsValid(message);
    /* 数组是二进制字节，Keil 在 uchar 后附带字符显示是正常现象。
     * 接收总数记录任务已处理的帧；valid 才记录符合测试 ID 和内容的帧。
     */
    if (message->source == 1U)
    {
        can1_last_rx_id = message->id;
        can1_last_rx_dlc = message->dlc;
        for (uint32_t i = 0; i < 8U; i++)
        {
            can1_last_rx_data[i] = message->data[i];
        }
        can1_last_rx_sequence = sequence;
        if (valid != 0U)
        {
            can1_rx_valid_count++;
        }
        else
        {
            can1_rx_invalid_count++;
        }
        can1_rx_count++;
    }
    else if (message->source == 2U)
    {
        can2_last_rx_id = message->id;
        can2_last_rx_dlc = message->dlc;
        for (uint32_t i = 0; i < 8U; i++)
        {
            can2_last_rx_data[i] = message->data[i];
        }
        can2_last_rx_sequence = sequence;
        if (valid != 0U)
        {
            can2_rx_valid_count++;
        }
        else
        {
            can2_rx_invalid_count++;
        }
        can2_rx_count++;
    }
}
