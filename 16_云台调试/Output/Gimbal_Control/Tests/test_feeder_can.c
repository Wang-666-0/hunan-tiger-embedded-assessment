#include "can_bus.h"
#include "app_config.h"
#include "app_fault.h"
#include "queue.h"
#include <string.h>

/* Actual transport with three independent hardware mailboxes per bus. */
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
CAN_HandleTypeDef hcan1 = {1U}, hcan2 = {2U};
#include "bus_contention.h"
typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t bus, mailbox;
} Submission;
static Submission tx[128];
static CanFrame queue_storage;
static uint32_t sends, pending[2], error_bits[2], abort_mask[2], aborts;
static uint8_t keep_pending, fail_send;

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *handle);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *handle);
void AppFault_Fatal(AppFault fault) { (void)fault; CHECK(0); }
QueueHandle_t xQueueCreate(uint32_t length, uint32_t size)
{ CHECK(length == 64U && size == sizeof(CanFrame)); return &queue_storage; }
void vQueueAddToRegistry(QueueHandle_t queue, const char *name) { (void)queue; (void)name; }
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t wait)
{ (void)queue; (void)data; (void)wait; return 0; }
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken)
{ (void)queue; (void)data; (void)woken; return pdPASS; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *handle, const CAN_FilterTypeDef *filter)
{
    CHECK(filter->FilterBank == (handle == &hcan1 ? 0U : 14U));
    CHECK(filter->SlaveStartFilterBank == 14U);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *handle, uint32_t flags)
{ (void)handle; CHECK(flags & CAN_IT_TX_MAILBOX_EMPTY); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *handle, uint32_t mailboxes)
{
    uint32_t index = handle->instance - 1U;
    CHECK(mailboxes && !(mailboxes & ~7U));
    abort_mask[index] |= mailboxes; ++aborts;
    if (!keep_pending) { pending[index] &= ~mailboxes; }
    return HAL_OK;
}
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *handle, uint32_t mailboxes)
{
    CHECK(mailboxes && !(mailboxes & ~7U));
    return (pending[handle->instance - 1U] & mailboxes) != 0U;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle,
    const CAN_TxHeaderTypeDef *header, const uint8_t *data, uint32_t *mailbox)
{
    uint32_t bit, index = handle->instance - 1U;
    if (fail_send) { return HAL_ERROR; }
    for (bit = 0U; bit < 3U; ++bit) {
        if (!(pending[index] & (1U << bit))) {
            *mailbox = 1U << bit; CHECK(sends < 128U);
            if (sends < 128U) {
                tx[sends].header = *header; tx[sends].bus = handle->instance;
                tx[sends].mailbox = *mailbox; memcpy(tx[sends].data, data, 8U);
            }
            ++sends; pending[index] |= *mailbox; return HAL_OK;
        }
    }
    return HAL_BUSY;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *handle, uint32_t fifo)
{ (void)handle; (void)fifo; return 0U; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *handle, uint32_t fifo,
    CAN_RxHeaderTypeDef *header, uint8_t *data)
{ (void)handle; (void)fifo; (void)header; (void)data; return HAL_ERROR; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *handle) { return error_bits[handle->instance - 1U]; }
uint32_t HAL_GetTick(void) { return 100U; }

static void CompleteAll(void)
{
    fixture_other[0] = fixture_other[1] = 0U;
    CAN_HandleTypeDef *handles[2] = {&hcan1, &hcan2};
    uint32_t index;
    for (index = 0U; index < 2U; ++index) {
        pending[index] = 0U;
        HAL_CAN_TxMailbox0CompleteCallback(handles[index]);
        HAL_CAN_TxMailbox1CompleteCallback(handles[index]);
        HAL_CAN_TxMailbox2CompleteCallback(handles[index]);
    }
}
static void CheckFeeder(uint32_t index, int16_t raw_current)
{
    uint32_t i;
    uint16_t raw = (uint16_t)raw_current;
    CHECK(index < sends);
    CHECK(tx[index].bus == 2U && tx[index].header.IDE == CAN_ID_STD);
    CHECK(tx[index].header.StdId == 0x1FFU && tx[index].header.ExtId == 0U);
    CHECK(tx[index].header.RTR == CAN_RTR_DATA && tx[index].header.DLC == 8U);
    CHECK(tx[index].data[0] == (uint8_t)(raw >> 8) && tx[index].data[1] == (uint8_t)raw);
    for (i = 2U; i < 8U; ++i) { CHECK(tx[index].data[i] == 0U); }
}
static void CheckHealthy(void)
{
    CanBusState state;
    CanBus_GetState(&state);
    CHECK(state.fault_epoch == 0U && state.errors[0] == 0U && state.errors[1] == 0U);
}

void test_run(void)
{
    static const int16_t valid_current[] = {3000, -3000, 1234, -1234, 1, -1, 0};
    CanFrame arbitrary = {0};
    CanBusState state;
    CanBusDiagnostics diagnostic;
    uint32_t before, feeder_box, wheel_box, other_box, yaw_box, pitch_box, i, before_aborts;
    test_stage = 1U;
    CHECK(1U && 1U && 1U);
    CHECK(0U == 0U && APP_FEEDER_CURRENT_HARD_LIMIT == 3000);
    CanBus_SetFeederPermission(1U);
    CHECK(!CanBus_FeederAllowed());
    CHECK(CanBus_FeederCurrent(0) == HAL_ERROR && CanBus_StopFeeder() == HAL_ERROR);
    CHECK(sends == 0U && aborts == 0U);
    CanBus_CreateQueue(); CanBus_Start();
    CHECK(!CanBus_FeederAllowed() && !CanBus_OutputAllowed());
    CHECK(!CanBus_YawAllowed() && !CanBus_PitchAllowed());
    CHECK(CanBus_FeederCurrent(0) == HAL_ERROR && sends == 0U);
    CanBus_SetOutputPermission(1U); CanBus_SetYawPermission(1U); CanBus_SetPitchPermission(1U);
    CHECK(!CanBus_FeederAllowed() && CanBus_FeederCurrent(1) == HAL_ERROR && sends == 0U);
    CanBus_SetOutputPermission(0U); CanBus_SetYawPermission(0U); CanBus_SetPitchPermission(0U);
    CanBus_SetFeederPermission(1U);
    CHECK(CanBus_FeederAllowed() && !CanBus_OutputAllowed());
    CHECK(!CanBus_YawAllowed() && !CanBus_PitchAllowed());
    for (i = 0U; i < sizeof(valid_current) / sizeof(valid_current[0]); ++i) {
        CHECK(CanBus_FeederCurrent(valid_current[i]) == HAL_OK);
        CheckFeeder(sends - 1U, valid_current[i]); CompleteAll();
    }
    before = sends;
    CHECK(CanBus_FeederCurrent(3001) == HAL_ERROR && CanBus_FeederCurrent(-3001) == HAL_ERROR);
    CHECK(CanBus_FeederCurrent(32767) == HAL_ERROR && CanBus_FeederCurrent(-32768) == HAL_ERROR);
    CHECK(sends == before && CanBus_FeederAllowed());
    CanBus_SetOutputPermission(1U);
    arbitrary.bus = 2U; arbitrary.id = 0x1FFU; arbitrary.dlc = 8U; arbitrary.data[1] = 1U;
    CHECK(CanBus_Send(&arbitrary) == HAL_ERROR);
    arbitrary.extended = 1U; arbitrary.id = 0x146U; arbitrary.dlc = 4U;
    CHECK(CanBus_Send(&arbitrary) == HAL_ERROR && sends == before); CheckHealthy();

    test_stage = 2U;
    CanBus_SetYawPermission(1U); CanBus_SetPitchPermission(1U);
    CHECK(CanBus_YawTorque(1.0f) == HAL_OK); yaw_box = tx[sends - 1U].mailbox;
    CHECK(CanBus_PitchTorque(1.0f) == HAL_OK); pitch_box = tx[sends - 1U].mailbox;
    CHECK(CanBus_FeederCurrent(800) == HAL_OK); feeder_box = tx[sends - 1U].mailbox;
    CHECK(CanBus_JogVescCurrent(70U, 500) == HAL_OK); wheel_box = tx[sends - 1U].mailbox;
    CHECK(Fixture_OtherTx(&hcan2) == HAL_OK); other_box = tx[sends - 1U].mailbox;
    CHECK(feeder_box != wheel_box && wheel_box != other_box && feeder_box != other_box);
    abort_mask[0] = abort_mask[1] = 0U;
    CanBus_SetOutputPermission(0U);
    CHECK(abort_mask[0] == 0U && abort_mask[1] == wheel_box);
    CHECK(pending[1] == (feeder_box | other_box) && CanBus_FeederAllowed());
    CHECK(CanBus_YawAllowed() && CanBus_PitchAllowed() && pending[0] == (yaw_box | pitch_box));
    abort_mask[1] = 0U; CanBus_SetFeederPermission(0U);
    CHECK(abort_mask[1] == feeder_box && pending[1] == other_box && !CanBus_FeederAllowed());
    CHECK(CanBus_YawAllowed() && CanBus_PitchAllowed() && pending[0] == (yaw_box | pitch_box));
    CanBus_SetFeederPermission(1U); CHECK(CanBus_FeederCurrent(900) == HAL_OK);
    feeder_box = tx[sends - 1U].mailbox; abort_mask[0] = abort_mask[1] = 0U;
    Fixture_CancelOther();
    CHECK(abort_mask[0] == 0U && abort_mask[1] == other_box && pending[1] == feeder_box);
    CHECK(CanBus_FeederAllowed() && CanBus_YawAllowed() && CanBus_PitchAllowed());
    CanBus_SetYawPermission(0U); CanBus_SetPitchPermission(0U);
    CHECK(pending[0] == 0U && pending[1] == feeder_box && CanBus_FeederAllowed());
    CompleteAll(); CheckHealthy();

    test_stage = 3U;
    CanBus_SetOutputPermission(1U);
    CHECK(CanBus_JogVescCurrent(70U, 500) == HAL_OK);
    CHECK(CanBus_JogVescCurrent(71U, 500) == HAL_OK);
    CHECK(Fixture_OtherTx(&hcan2) == HAL_OK && pending[1] == 7U);
    before = sends;
    CHECK(CanBus_FeederCurrent(500) == HAL_BUSY && sends == before && CanBus_FeederAllowed());
    CHECK(CanBus_StopFeeder() == HAL_BUSY && sends == before && !CanBus_FeederAllowed());
    CHECK(pending[1] == 7U); CheckHealthy(); CompleteAll();
    CanBus_SetFeederPermission(1U);
    CHECK(CanBus_FeederCurrent(100) == HAL_OK); feeder_box = tx[sends - 1U].mailbox;
    before = sends;
    CHECK(CanBus_FeederCurrent(200) == HAL_BUSY && sends == before && pending[1] == feeder_box);
    CHECK(CanBus_JogVescCurrent(70U, 500) == HAL_OK); wheel_box = tx[sends - 1U].mailbox;
    CHECK(Fixture_OtherTx(&hcan2) == HAL_OK); other_box = tx[sends - 1U].mailbox;
    keep_pending = 1U; before = sends; abort_mask[0] = abort_mask[1] = 0U;
    CanBus_SetFeederPermission(0U);
    CHECK(abort_mask[0] == 0U && abort_mask[1] == feeder_box && pending[1] == 7U);
    CHECK(CanBus_StopFeeder() == HAL_BUSY && CanBus_StopFeeder() == HAL_BUSY);
    CHECK(sends == before && pending[1] == 7U && !CanBus_FeederAllowed()); CheckHealthy();
    keep_pending = 0U; abort_mask[1] = 0U;
    CHECK(CanBus_StopFeeder() == HAL_OK && sends == before + 1U);
    CHECK(abort_mask[1] == feeder_box && pending[1] == (feeder_box | wheel_box | other_box));
    CheckFeeder(sends - 1U, 0); CompleteAll();
    CHECK(CanBus_StopFeeder() == HAL_OK && !CanBus_FeederAllowed());
    CheckFeeder(sends - 1U, 0); CompleteAll();
    /* Late old completion/abort IRQ must retain the owner of a reused mailbox. */
    CanBus_SetFeederPermission(1U); CHECK(CanBus_FeederCurrent(100) == HAL_OK);
    CHECK(tx[sends - 1U].mailbox == 1U); pending[1] = 0U;
    CHECK(CanBus_FeederCurrent(200) == HAL_OK && tx[sends - 1U].mailbox == 1U);
    HAL_CAN_TxMailbox0CompleteCallback(&hcan2); HAL_CAN_TxMailbox0AbortCallback(&hcan2);
    keep_pending = 1U; before = sends;
    CHECK(CanBus_StopFeeder() == HAL_BUSY && sends == before && pending[1] == 1U);
    keep_pending = 0U; CHECK(CanBus_StopFeeder() == HAL_OK); CheckFeeder(sends - 1U, 0);
    CompleteAll(); CheckHealthy();

    test_stage = 4U;
    CanBus_SetFeederPermission(1U); CanBus_SetYawPermission(1U); CanBus_SetPitchPermission(1U);
    CanBus_SetOutputPermission(1U);
    CHECK(CanBus_YawTorque(1.0f) == HAL_OK && CanBus_PitchTorque(1.0f) == HAL_OK);
    CHECK(CanBus_FeederCurrent(1000) == HAL_OK && CanBus_JogVescCurrent(70U, 500) == HAL_OK);
    error_bits[1] = 0x80U; abort_mask[0] = abort_mask[1] = 0U;
    HAL_CAN_ErrorCallback(&hcan2);
    CHECK(abort_mask[0] == 7U && abort_mask[1] == 7U && pending[0] == 0U && pending[1] == 0U);
    CHECK(!CanBus_FeederAllowed() && !CanBus_OutputAllowed());
    CHECK(!CanBus_YawAllowed() && !CanBus_PitchAllowed());
    CanBus_GetState(&state); CHECK(state.fault_epoch == 1U && state.errors[1] == 1U);
    before = sends; before_aborts = aborts;
    CanBus_GetDiagnostics(&diagnostic);
    CHECK(sends == before && aborts == before_aborts);
    CHECK(diagnostic.state.first_error_source[1] == CAN_BUS_SOURCE_IRQ);
    CHECK(diagnostic.state.first_error[1] == 0x80U);
    error_bits[1] = 0U;
    CanBus_SetFeederPermission(1U); CanBus_SetOutputPermission(1U);
    CanBus_SetYawPermission(1U); CanBus_SetPitchPermission(1U);
    CHECK(!CanBus_FeederAllowed() && !CanBus_OutputAllowed());
    CHECK(!CanBus_YawAllowed() && !CanBus_PitchAllowed());
    CHECK(CanBus_FeederCurrent(0) == HAL_ERROR && CanBus_FeederCurrent(100) == HAL_ERROR && sends == before);
    CHECK(CanBus_StopFeeder() == HAL_OK); CheckFeeder(sends - 1U, 0); CompleteAll();
    before = sends; fail_send = 1U;
    CHECK(CanBus_StopFeeder() == HAL_ERROR && sends == before);
    CanBus_GetState(&state); CHECK(state.fault_epoch == 2U && state.errors[1] == 2U);
    CHECK(state.first_error_source[1] == CAN_BUS_SOURCE_IRQ && state.first_error[1] == 0x80U);
    test_done = 1U;
}
