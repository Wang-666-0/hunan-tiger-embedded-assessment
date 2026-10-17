#include "can_bus.h"
#include "app_config.h"
#include "app_fault.h"
#include "gl60_ii.h"
#include "queue.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
CAN_HandleTypeDef hcan1 = {1U}, hcan2 = {2U};
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
static void ClearPending(void) { pending[0] = pending[1] = 0U; }
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

void test_run(void)
{
    CanBusState state;
    CanFrame arbitrary;
    uint32_t before, i, faults;
    int32_t sample;
    static const float torques[] = {-0.10f, -0.099f, -0.095f, -0.05f, -0.03f,
        -0.025f, -0.02f, -0.01f, -0.001f, 0.0f, 0.001f, 0.01f, 0.02f, 0.025f,
        0.03f, 0.05f, 0.095f, 0.099f, 0.10f};
    test_stage = 1U;
    CHECK(APP_YAW_ENABLED == 1U && APP_MOTOR_DRIVERS_IMPLEMENTED == 0U);
    CanBus_SetYawPermission(1U); CanBus_SetOutputPermission(1U);
    CHECK(!CanBus_YawAllowed() && !CanBus_OutputAllowed());
    CHECK(CanBus_YawEnable() == HAL_ERROR && CanBus_YawTorque(0.01f) == HAL_ERROR);
    CHECK(CanBus_YawDisable(1U) == HAL_ERROR && sends == 0U);
    CHECK(CanBus_YawClearComms() == HAL_ERROR && sends == 0U);
    CanBus_CreateQueue(); CanBus_Start();
    CHECK(starts == 2U && notices == 2U);
    before = sends; i = aborts;
    CHECK(CanBus_YawTxPending() == 0U && sends == before && aborts == i);
    CHECK(CanBus_YawEnable() == HAL_ERROR && CanBus_YawTorque(0.0f) == HAL_ERROR);
    CHECK(CanBus_YawDisable(0U) == HAL_OK); CheckYaw(sends - 1U, 0xFDU); ClearPending();
    CHECK(CanBus_YawClearComms() == HAL_OK); CheckYaw(sends - 1U, 0xFBU);
    before = sends;
    CHECK(CanBus_YawClearComms() == HAL_BUSY && sends == before); ClearPending();
    CanBus_SetYawPermission(1U);
    CHECK(CanBus_YawClearComms() == HAL_ERROR && sends == before);
    CHECK(CanBus_YawAllowed() && !CanBus_OutputAllowed());
    CanBus_GetState(&state); CHECK(state.rx_frames[0] == 0U && state.rx_frames[1] == 0U);
    CHECK(CanBus_YawEnable() == HAL_OK); CheckYaw(sends - 1U, 0xFCU); ClearPending();

    test_stage = 2U;
    before = sends;
    CHECK(CanBus_YawTorque(0.10001f) == HAL_ERROR);
    CHECK(CanBus_YawTorque(-0.10001f) == HAL_ERROR);
    CHECK(CanBus_YawTorque(FloatBits(0x7FC00000U)) == HAL_ERROR);
    CHECK(CanBus_YawTorque(FloatBits(0x7F800000U)) == HAL_ERROR);
    CHECK(CanBus_YawTorque(FloatBits(0xFF800000U)) == HAL_ERROR);
    CHECK(sends == before && CanBus_YawAllowed());
    for (i = 0U; i < sizeof(torques) / sizeof(torques[0]); ++i) {
        CHECK(CanBus_YawTorque(torques[i]) == HAL_OK); CheckYaw(sends - 1U, 0U); ClearPending();
    }
    CHECK(tx[before].data[6] == 7U && tx[before].data[7] == 0xECU); /* lower raw2028 */
    CHECK(tx[sends - 1U].data[6] == 8U && tx[sends - 1U].data[7] == 0x13U); /* upper raw2067 */
    /* 从负端到正端扫过量化台阶，检查线上实际字段、内部KP/KD和绝对限幅。 */
    for (sample = -200; sample <= 200; ++sample) {
        CHECK(CanBus_YawTorque((float)sample / 2000.0f) == HAL_OK);
        CheckYaw(sends - 1U, 0U); ClearPending();
    }
    Gl60_BuildEnable(&arbitrary); before = sends;
    CanBus_SetOutputPermission(1U);
    CHECK(CanBus_Send(&arbitrary) == HAL_ERROR && sends == before);

    test_stage = 3U;
    CHECK(CanBus_YawEnable() == HAL_OK); before = sends;
    CHECK(pending[0] == 1U); abort_mask = 0U;
    before = sends; i = aborts;
    CHECK(CanBus_YawTxPending() == 1U && sends == before && aborts == i);
    CanBus_SetOutputPermission(0U);
    CHECK(abort_mask == 2U && pending[0] == 1U && CanBus_YawAllowed() && !CanBus_OutputAllowed());
    CHECK(CanBus_StopFlywheels() == HAL_OK && sends == before + 2U);
    CHECK(abort_mask == 2U && pending[0] == 1U && CanBus_YawAllowed());
    CHECK(tx[before].bus == 2U && tx[before].header.ExtId == 0x146U);
    CHECK(tx[before + 1U].bus == 2U && tx[before + 1U].header.ExtId == 0x147U);
    for (i = 0U; i < 4U; ++i) { CHECK(tx[before].data[i] == 0U && tx[before + 1U].data[i] == 0U); }
    pending[1] = 0U;
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_OK && CanBus_YawAllowed() && pending[0] == 1U);
    ClearPending(); CanBus_SetOutputPermission(1U); abort_mask = 0U;
    CanBus_SetYawPermission(0U);
    CHECK(abort_mask == 1U && !CanBus_YawAllowed() && CanBus_OutputAllowed());
    before = sends;
    CHECK(CanBus_YawClearComms() == HAL_ERROR && sends == before);
    CHECK(CanBus_YawEnable() == HAL_ERROR && CanBus_YawTorque(0.01f) == HAL_ERROR && sends == before);
    CHECK(CanBus_JogVescCurrent(70U, 2000) == HAL_OK); ClearPending();

    test_stage = 4U;
    CanBus_SetYawPermission(1U); CHECK(CanBus_YawEnable() == HAL_OK);
    before = sends; keep_pending = 1U; abort_mask = 0U;
    CHECK(CanBus_YawDisable(1U) == HAL_BUSY && sends == before && abort_mask == 1U);
    CHECK(!CanBus_YawAllowed() && CanBus_OutputAllowed());
    keep_pending = 0U;
    CHECK(CanBus_YawDisable(1U) == HAL_OK); CheckYaw(sends - 1U, 0xFDU); ClearPending();
    abort_failure_bus = 1U; before = sends;
    CHECK(CanBus_YawDisable(1U) == HAL_ERROR && sends == before);
    abort_failure_bus = 0U; CHECK(CanBus_YawDisable(1U) == HAL_OK); ClearPending();
    pending[0] = 1U; before = sends;
    CHECK(CanBus_YawDisable(0U) == HAL_BUSY && sends == before);
    pending[0] = 0U; CHECK(CanBus_YawDisable(0U) == HAL_OK); ClearPending();
    before = sends; fail_send_call = sends + 1U;
    CHECK(CanBus_YawDisable(0U) == HAL_ERROR && sends == before + 1U && !CanBus_YawAllowed());
    fail_send_call = 0U; CHECK(CanBus_YawDisable(0U) == HAL_OK); CheckYaw(sends - 1U, 0xFDU); ClearPending();

    test_stage = 5U;
    CanBus_SetYawPermission(1U); CanBus_SetOutputPermission(1U);
    CHECK(CanBus_YawAllowed() && CanBus_OutputAllowed());
    fail_send_call = sends + 1U; abort_mask = 0U;
    CHECK(CanBus_YawTorque(0.02f) == HAL_ERROR);
    CHECK(abort_mask == 3U && !CanBus_YawAllowed() && !CanBus_OutputAllowed());
    CanBus_GetState(&state); CHECK(state.fault_epoch == 1U && state.errors[0] == 1U);
    faults = state.fault_epoch; fail_send_call = 0U;
    CanBus_SetYawPermission(1U); CanBus_SetOutputPermission(1U);
    CHECK(!CanBus_YawAllowed() && !CanBus_OutputAllowed());
    CHECK(CanBus_YawDisable(1U) == HAL_OK); CheckYaw(sends - 1U, 0xFDU); ClearPending();
    error_bits[1] = 0x40U; abort_mask = 0U; HAL_CAN_ErrorCallback(&hcan2);
    CanBus_GetState(&state);
    CHECK(state.fault_epoch == faults + 1U && state.errors[1] == 1U && abort_mask == 3U);
    CHECK(!CanBus_YawAllowed() && !CanBus_OutputAllowed());
    before = sends;
    CHECK(CanBus_YawEnable() == HAL_ERROR && CanBus_YawTorque(0.0f) == HAL_ERROR && sends == before);
    CHECK(CanBus_YawClearComms() == HAL_ERROR && sends == before);
    CHECK(CanBus_YawDisable(1U) == HAL_OK); CheckYaw(sends - 1U, 0xFDU);
    test_done = 1U;
}
