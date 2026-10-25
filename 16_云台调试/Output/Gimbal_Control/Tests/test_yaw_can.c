#include "can_bus.h"
#include "app_config.h"
#include "app_fault.h"
#include "gl60_ii.h"
#include "queue.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
CAN_HandleTypeDef hcan1 = {1U}, hcan2 = {2U};
#include "bus_contention.h"
typedef struct { CAN_TxHeaderTypeDef header; uint8_t data[8]; uint32_t bus; } Submission;
static Submission tx[512];
static CanFrame queued;
static uint32_t sends, starts, notices, abort_mask, aborts, fail_send_call;
static uint32_t pending[2], error_bits[2];
static uint8_t keep_pending, abort_failure_bus;

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle);
void AppFault_Fatal(AppFault fault) { (void)fault; CHECK(0); }
QueueHandle_t xQueueCreate(uint32_t length, uint32_t size)
{ CHECK(length == APP_CAN_RX_QUEUE_LENGTH && size == sizeof(CanFrame)); return &queued; }
void vQueueAddToRegistry(QueueHandle_t queue, const char *name) { (void)queue; (void)name; }
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t wait)
{ (void)queue; (void)data; (void)wait; return 0; }
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken)
{ (void)queue; (void)data; (void)woken; return pdPASS; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *handle, const CAN_FilterTypeDef *filter)
{
    CHECK(filter->FilterBank == (handle->instance == 1U ? 0U : 14U));
    CHECK(filter->SlaveStartFilterBank == 14U); return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *handle) { (void)handle; ++starts; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *handle, uint32_t flags)
{ (void)handle; CHECK(flags & CAN_IT_ERROR); ++notices; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *handle, uint32_t mailboxes)
{
    CHECK(mailboxes == 7U); ++aborts; abort_mask |= 1U << (handle->instance - 1U);
    if (abort_failure_bus == handle->instance) { return HAL_ERROR; }
    if (!keep_pending) { pending[handle->instance - 1U] = 0U; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle, const CAN_TxHeaderTypeDef *header,
                                     const uint8_t *data, uint32_t *mailbox)
{
    CHECK(sends < 512U); *mailbox = CAN_TX_MAILBOX0;
    if (sends < 512U) { tx[sends].header = *header; tx[sends].bus = handle->instance; memcpy(tx[sends].data, data, 8U); }
    ++sends;
    if (sends == fail_send_call) { return HAL_ERROR; }
    pending[handle->instance - 1U] = 1U; return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *handle, uint32_t fifo)
{ (void)handle; (void)fifo; return 0U; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *handle, uint32_t fifo,
                                    CAN_RxHeaderTypeDef *header, uint8_t *data)
{ (void)handle; (void)fifo; (void)header; (void)data; return HAL_ERROR; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *handle) { return error_bits[handle->instance - 1U]; }
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *handle, uint32_t mailboxes)
{ CHECK(mailboxes == 7U); return pending[handle->instance - 1U]; }
uint32_t HAL_GetTick(void) { return 100U; }

static float FloatBits(uint32_t raw) { float out; memcpy(&out, &raw, sizeof(out)); return out; }
static void ClearPending(void) {
    fixture_other[0] = fixture_other[1] = 0U; pending[0] = pending[1] = 0U; }
static void CheckYaw(uint32_t index, uint8_t special)
{
    uint32_t i;
    CHECK(index < sends && index < 512U);
    if (index >= sends || index >= 512U) { return; }
    CHECK(tx[index].bus == 1U && tx[index].header.IDE == CAN_ID_STD && tx[index].header.StdId == 1U);
    CHECK(tx[index].header.ExtId == 0U && tx[index].header.DLC == 8U && tx[index].header.RTR == CAN_RTR_DATA);
    if (special) {
        for (i = 0U; i < 7U; ++i) { CHECK(tx[index].data[i] == 0xFFU); }
        CHECK(tx[index].data[7] == special);
    } else {
        uint16_t raw = (uint16_t)(((tx[index].data[6] & 15U) << 8) | tx[index].data[7]);
        float actual = (float)raw * 20.0f / 4095.0f - 10.0f;
        CHECK(tx[index].data[0] == 0x7FU && tx[index].data[1] == 0xFFU && tx[index].data[2] == 0x7FU);
        CHECK(tx[index].data[3] == 0xF0U && tx[index].data[4] == 0U && tx[index].data[5] == 0U);
        CHECK((tx[index].data[6] & 0xF0U) == 0U && raw >= 2028U && raw <= 2067U);
        CHECK(actual >= -0.10f && actual <= 0.10f);
    }
}
