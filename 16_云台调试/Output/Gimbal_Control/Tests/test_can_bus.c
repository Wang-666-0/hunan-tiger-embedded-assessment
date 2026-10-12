#include "can_bus.h"
#include "app_fault.h"
#include "app_config.h"
#include "dm_diag.h"
#include "queue.h"
#include <string.h>
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
CAN_HandleTypeDef hcan1 = {1U}, hcan2 = {2U};
static CanFrame queued;
static CAN_RxHeaderTypeDef incoming;
static CAN_FilterTypeDef filters[2];
static uint32_t queue_has_frame, incoming_bus, sends, starts, notices, aborts, now = 123U;
static uint8_t overflow, read_failure;
static CAN_TxHeaderTypeDef sent_header;
static uint8_t sent_data[8];
static uint32_t sent_bus;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle);
void AppFault_Fatal(AppFault fault) { (void)fault; CHECK(0); }
QueueHandle_t xQueueCreate(uint32_t length, uint32_t size)
{ CHECK(length > 0U && size == sizeof(CanFrame)); return &queued; }
void vQueueAddToRegistry(QueueHandle_t queue, const char *name) { (void)queue; (void)name; }
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t wait)
{
    (void)queue; (void)wait;
    if (!queue_has_frame) { return 0; }
    memcpy(data, &queued, sizeof(queued)); queue_has_frame = 0U; return pdPASS;
}
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken)
{
    (void)queue; (void)woken;
    if (overflow) { return 0; }
    memcpy(&queued, data, sizeof(queued)); queue_has_frame = 1U; return pdPASS;
}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *handle, const CAN_FilterTypeDef *filter)
{ filters[handle->instance - 1U] = *filter; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *handle) { (void)handle; starts++; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *handle, uint32_t flags)
{ (void)handle; CHECK(flags & CAN_IT_ERROR); notices++; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *handle, uint32_t mailboxes)
{ (void)handle; CHECK(mailboxes == 7U); aborts++; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle, const CAN_TxHeaderTypeDef *header,
    const uint8_t *data, uint32_t *mailbox)
{
    (void)mailbox; sent_bus = handle->instance; sent_header = *header;
    memcpy(sent_data, data, 8U); sends++; return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *handle, uint32_t fifo)
{ (void)fifo; return incoming_bus == handle->instance; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *handle, uint32_t fifo,
    CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)handle; (void)fifo;
    if (read_failure) { incoming_bus = 0U; return HAL_ERROR; }
    *header = incoming; memset(data, 0x42, 8U); incoming_bus = 0U; return HAL_OK;
}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *handle) { (void)handle; return 0x80U; }
uint32_t HAL_GetTick(void) { return now; }

void test_run(void)
{
    CanFrame frame = {0};
    CanBusState state;
    test_stage = 1U; CanBus_CreateQueue(); CanBus_Start();
    CHECK(starts == 2U && notices == 2U);
    CHECK(filters[0].FilterBank == 0U && filters[1].FilterBank == 14U);
    CHECK(filters[0].SlaveStartFilterBank == 14U && filters[1].FilterMaskIdLow == 0U);
    frame.bus = 1U; frame.id = 2U; frame.dlc = 8U;
    CHECK(CanBus_Send(&frame) == HAL_ERROR);
    CanBus_SetOutputPermission(1U); CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == 0U);
    frame.bus = 2U; frame.extended = 1U; frame.id = 0x346U; frame.dlc = 4U;
    CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == 0U);
    test_stage = 2U;
    incoming_bus = 1U; incoming.IDE = CAN_ID_STD; incoming.StdId = 0x205U; incoming.DLC = 8U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    CHECK(CanBus_Receive(&frame, 0U) == pdPASS);
    CHECK(frame.bus == 1U && !frame.extended && frame.id == 0x205U && frame.dlc == 8U);
    CHECK(frame.received_ms == now && frame.data[0] == 0x42U);
    incoming_bus = 2U; incoming.IDE = CAN_ID_EXT; incoming.ExtId = 0x946U; incoming.DLC = 4U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    CHECK(CanBus_Receive(&frame, 0U) == pdPASS);
    CHECK(frame.bus == 2U && frame.extended && frame.id == 0x946U && frame.dlc == 4U);
    CHECK(frame.data[3] == 0x42U && frame.data[4] == 0U && frame.data[7] == 0U);
    incoming_bus = 2U; incoming.DLC = 9U; HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    CHECK(CanBus_Receive(&frame, 0U) != pdPASS);
    incoming_bus = 2U; incoming.DLC = 8U; incoming.RTR = 2U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2); CHECK(CanBus_Receive(&frame, 0U) != pdPASS);
    test_stage = 3U; incoming.RTR = 0U; incoming_bus = 2U; overflow = 1U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2); CanBus_GetState(&state);
    CHECK(state.dropped == 1U && state.fault_epoch == 1U);
    HAL_CAN_ErrorCallback(&hcan2); CanBus_GetState(&state);
    CHECK(state.errors[1] == 1U && state.last_error[1] == 0x80U && state.fault_epoch == 2U);
    CHECK(aborts >= 2U && sends == 0U);
    overflow = 0U; read_failure = 1U; incoming_bus = 1U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); CanBus_GetState(&state);
    CHECK(state.errors[0] == 1U && state.last_error[0] == 0x80U && state.fault_epoch == 3U);
    CHECK(CanBus_Receive(&frame, 0U) != pdPASS);
    CanBus_SetOutputPermission(1U); frame.bus = 2U;
    CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == 0U);
    test_stage = 4U;
    CHECK(CanBus_ReadDm(DM_DIAG_RID_MASTER_ID) == HAL_OK);
    CHECK(sends == 1U && sent_bus == 1U && sent_header.IDE == CAN_ID_STD &&
        sent_header.StdId == 0x7FFU && sent_header.DLC == 8U && sent_header.RTR == CAN_RTR_DATA);
    CHECK(sent_data[0] == 2U && sent_data[1] == 0U && sent_data[2] == 0x33U && sent_data[3] == 7U &&
        sent_data[4] == 0U && sent_data[5] == 0U && sent_data[6] == 0U && sent_data[7] == 0U);
    CHECK(CanBus_ReadDm(0U) == HAL_ERROR && sends == 1U);
    CHECK(CanBus_ReadVesc(70U, 0U) == HAL_OK);
    CHECK(sends == 2U && sent_bus == 2U && sent_header.IDE == CAN_ID_EXT &&
        sent_header.ExtId == 0x846U && sent_header.DLC == 3U &&
        sent_data[0] == APP_DIAG_VESC_HOST_ID && sent_data[1] == 0U && sent_data[2] == 0U);
    CHECK(CanBus_ReadVesc(71U, 91U) == HAL_OK && sent_header.ExtId == 0x847U && sent_data[2] == 91U);
    CHECK(CanBus_ReadVesc(70U, 8U) == HAL_ERROR && sends == 3U); /* SET_RPM不能穿透读接口 */
    CHECK(CanBus_ReadVesc(70U, 30U) == HAL_ERROR && sends == 3U); /* ALIVE也不开放 */
    CHECK(CanBus_ReadVesc(72U, 14U) == HAL_ERROR && sends == 3U);
    CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == 3U);
    {
        uint32_t previous_aborts = aborts;
        CanBus_CancelReads(); CHECK(aborts == previous_aborts + 2U);
    }
    test_done = 1U;
}
