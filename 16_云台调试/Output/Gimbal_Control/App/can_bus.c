#include "can_bus.h"
#include "app_config.h"
#include "app_fault.h"
#include "dm_diag.h"
#include "vesc_diag.h"
#include "queue.h"
#include <string.h>

static QueueHandle_t rx_queue;
static volatile CanBusState state;
static volatile uint8_t output_permission;

static void AbortOutputs(void)
{
    output_permission = 0U;
    if (state.started) {
        (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        (void)HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
}

static int32_t BusIndex(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) { return 0; }
    if (hcan == &hcan2) { return 1; }
    return -1;
}

void CanBus_CreateQueue(void)
{
    rx_queue = xQueueCreate(APP_CAN_RX_QUEUE_LENGTH, sizeof(CanFrame));
    if (rx_queue == 0) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "GIMBAL_CAN");
}

void CanBus_Start(void)
{
    CAN_FilterTypeDef filter = {0};
    CAN_HandleTypeDef *handles[2] = { &hcan1, &hcan2 };
    uint32_t i;
    const uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
        CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE |
        CAN_IT_ERROR_WARNING | CAN_IT_LAST_ERROR_CODE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    /* 零掩码同时接收标准/扩展帧；软件校验IDE、ID、RTR和DLC。
     * 共享过滤器0..13给CAN1、14..27给CAN2。不得从旧工程复制单CAN过滤。 */
    for (i = 0U; i < 2U; ++i) {
        filter.FilterBank = i == 0U ? 0U : 14U;
        if (HAL_CAN_ConfigFilter(handles[i], &filter) != HAL_OK ||
            HAL_CAN_Start(handles[i]) != HAL_OK ||
            HAL_CAN_ActivateNotification(handles[i], notifications) != HAL_OK) {
            AppFault_Fatal(APP_FAULT_CAN_START);
        }
    }
    state.started = 1U;
}

BaseType_t CanBus_Receive(CanFrame *frame, TickType_t wait_ticks)
{
    return xQueueReceive(rx_queue, frame, wait_ticks);
}

void CanBus_GetState(CanBusState *out)
{
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    *out = state;
    taskEXIT_CRITICAL();
}

void CanBus_SetOutputPermission(uint8_t allowed)
{
    uint8_t previous;
    taskENTER_CRITICAL();
    previous = output_permission;
    output_permission = allowed && APP_MOTOR_DRIVERS_IMPLEMENTED && state.started;
    if (previous && !output_permission) {
        /* 以后接入驱动时，停止同时撤销两个控制器的挂起发送。 */
        AbortOutputs();
    }
    taskEXIT_CRITICAL();
}

HAL_StatusTypeDef CanBus_Send(const CanFrame *frame)
{
    CAN_TxHeaderTypeDef header = {0};
    CAN_HandleTypeDef *handle;
    uint32_t mailbox;
    HAL_StatusTypeDef result;
    if (!CanFrame_IsValid(frame)) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!output_permission || !APP_MOTOR_DRIVERS_IMPLEMENTED) {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
    handle = frame->bus == 1U ? &hcan1 : &hcan2;
    header.IDE = frame->extended ? CAN_ID_EXT : CAN_ID_STD;
    header.ExtId = frame->extended ? frame->id : 0U;
    header.StdId = frame->extended ? 0U : frame->id;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame->dlc;
    header.TransmitGlobalTime = DISABLE;
    result = HAL_CAN_AddTxMessage(handle, &header, frame->data, &mailbox);
    taskEXIT_CRITICAL();
    return result; /* HAL_OK仅代表已提交，tx_frames由完成中断增加。 */
}

/* 仅这两个公开接口能进入只读发送路径；其构造器拒绝写入、使能、
 * 设零、检测电机以及转速/电流命令。动力发送门禁始终保持关闭。 */
static HAL_StatusTypeDef SendRead(const CanFrame *frame)
{
    CAN_TxHeaderTypeDef header = {0};
    CAN_HandleTypeDef *handle;
    HAL_StatusTypeDef result;
    uint32_t mailbox;
    if (!CanFrame_IsValid(frame)) { return HAL_ERROR; }
    header.IDE = frame->extended ? CAN_ID_EXT : CAN_ID_STD;
    header.ExtId = frame->extended ? frame->id : 0U;
    header.StdId = frame->extended ? 0U : frame->id;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame->dlc;
    header.TransmitGlobalTime = DISABLE;
    handle = frame->bus == 1U ? &hcan1 : &hcan2;
    taskENTER_CRITICAL();
    result = state.started ? HAL_CAN_AddTxMessage(handle, &header, frame->data, &mailbox) : HAL_ERROR;
    taskEXIT_CRITICAL();
    return result;
}

HAL_StatusTypeDef CanBus_ReadDm(uint8_t rid)
{
    CanFrame frame;
    if (!DmDiag_BuildRead(rid, &frame)) { return HAL_ERROR; }
    return SendRead(&frame);
}

HAL_StatusTypeDef CanBus_ReadVesc(uint8_t node, uint8_t command)
{
    CanFrame frame;
    if (!VescDiag_BuildRead(node, command, &frame)) { return HAL_ERROR; }
    return SendRead(&frame);
}

void CanBus_CancelReads(void)
{
    taskENTER_CRITICAL();
    AbortOutputs();
    taskEXIT_CRITICAL();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t woken = pdFALSE;
    int32_t index = BusIndex(hcan);
    if (index < 0) { return; }
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
        CAN_RxHeaderTypeDef header;
        CanFrame frame = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
            state.errors[index]++;
            state.last_error[index] = HAL_CAN_GetError(hcan);
            state.fault_epoch++;
            AbortOutputs();
            break;
        }
        if (header.RTR != CAN_RTR_DATA || header.DLC > 8U ||
            (header.IDE != CAN_ID_STD && header.IDE != CAN_ID_EXT)) { continue; }
        frame.bus = (uint8_t)(index + 1);
        frame.extended = header.IDE == CAN_ID_EXT;
        frame.id = frame.extended ? header.ExtId : header.StdId;
        frame.dlc = (uint8_t)header.DLC;
        /* HAL可能复制整个邮箱的8字节；DLC之外清零，避免短帧带入残留数据。 */
        if (frame.dlc < 8U) { memset(&frame.data[frame.dlc], 0, 8U - frame.dlc); }
        frame.received_ms = HAL_GetTick();
        frame.fault_epoch = state.fault_epoch;
        if (!CanFrame_IsValid(&frame)) { continue; }
        state.rx_frames[index]++;
        if (xQueueSendFromISR(rx_queue, &frame, &woken) != pdPASS) {
            state.dropped++;
            state.fault_epoch++;
            AbortOutputs();
        }
    }
    portYIELD_FROM_ISR(woken);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    int32_t index = BusIndex(hcan);
    if (index < 0) { return; }
    state.errors[index]++;
    state.last_error[index] = HAL_CAN_GetError(hcan);
    state.fault_epoch++;
    AbortOutputs();
}

static void TxComplete(CAN_HandleTypeDef *hcan)
{
    int32_t index = BusIndex(hcan);
    if (index >= 0) { state.tx_frames[index]++; }
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { TxComplete(hcan); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { TxComplete(hcan); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { TxComplete(hcan); }
