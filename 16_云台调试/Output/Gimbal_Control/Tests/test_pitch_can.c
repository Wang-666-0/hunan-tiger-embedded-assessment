#include "can_bus.h"
#include "app_config.h"
#include "app_fault.h"
#include "queue.h"
#include <string.h>

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
{ CHECK(length == APP_CAN_RX_QUEUE_LENGTH && size == sizeof(CanFrame)); return &queue_storage; }
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
    abort_mask[index] |= mailboxes;
    ++aborts;
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
            *mailbox = 1U << bit;
            CHECK(sends < 128U);
            if (sends < 128U) {
                tx[sends].header = *header;
                tx[sends].bus = handle->instance;
                tx[sends].mailbox = *mailbox;
                memcpy(tx[sends].data, data, 8U);
            }
            ++sends;
            pending[index] |= *mailbox;
            return HAL_OK;
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

static float FloatBits(uint32_t raw) { float out; memcpy(&out, &raw, sizeof(out)); return out; }
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
static void CheckSpecial(uint32_t index, uint32_t id, uint8_t opcode)
{
    uint32_t i;
    CHECK(index < sends);
    CHECK(tx[index].bus == 1U && tx[index].header.IDE == CAN_ID_STD && tx[index].header.StdId == id);
    CHECK(tx[index].header.ExtId == 0U && tx[index].header.DLC == 8U && tx[index].header.RTR == CAN_RTR_DATA);
    for (i = 0U; i < 7U; ++i) { CHECK(tx[index].data[i] == 0xFFU); }
    CHECK(tx[index].data[7] == opcode);
}
static uint16_t TorqueRaw(uint32_t index)
{
    CHECK(tx[index].bus == 1U && tx[index].header.IDE == CAN_ID_STD && tx[index].header.StdId == 2U);
    CHECK(tx[index].header.DLC == 8U && tx[index].header.RTR == CAN_RTR_DATA);
    CHECK(tx[index].data[3] == 0xF0U && tx[index].data[4] == 0U && tx[index].data[5] == 0U);
    CHECK((tx[index].data[6] & 0xF0U) == 0U); /* 电机内部KP/KD固定0。 */
    return (uint16_t)(((tx[index].data[6] & 15U) << 8) | tx[index].data[7]);
}

void test_run(void)
{
    CanFrame arbitrary = {0};
    CanBusState state;
    CanBusDiagnostics diagnostics;
    uint32_t before, pitch_box, yaw_box, other_box, i;
    test_stage = 1U;
    CHECK(APP_PITCH_ENABLED && APP_YAW_ENABLED && APP_MOTOR_DRIVERS_IMPLEMENTED == 0U);
    CanBus_SetPitchPermission(1U);
    CHECK(!CanBus_PitchAllowed());
    CHECK(CanBus_PitchEnable() == HAL_ERROR && CanBus_PitchTorque(1.0f) == HAL_ERROR);
    CHECK(CanBus_PitchDisable(1U) == HAL_ERROR && CanBus_PitchReadFeedback() == HAL_ERROR);
    CHECK(CanBus_CancelPitchTx() == HAL_ERROR && CanBus_PitchTxPending() == 0U && sends == 0U);
    CanBus_CreateQueue(); CanBus_Start();
    CHECK(CanBus_PitchReadFeedback() == HAL_OK); /* 未使能即可读CC。 */
    CHECK(!CanBus_PitchAllowed() && !CanBus_YawAllowed() && !CanBus_OutputAllowed());
    CHECK(tx[0].bus == 1U && tx[0].header.StdId == 0x7FFU && tx[0].header.IDE == CAN_ID_STD);
    CHECK(tx[0].header.DLC == 8U && tx[0].header.RTR == CAN_RTR_DATA);
    CHECK(tx[0].data[0] == 2U && tx[0].data[1] == 0U && tx[0].data[2] == 0xCCU);
    for (i = 3U; i < 8U; ++i) { CHECK(tx[0].data[i] == 0U); }
    CompleteAll();
    before = sends;
    CHECK(CanBus_PitchEnable() == HAL_ERROR && CanBus_PitchTorque(0.0f) == HAL_ERROR && sends == before);
    CHECK(CanBus_PitchDisable(0U) == HAL_OK);
    CheckSpecial(sends - 1U, 2U, 0xFDU);
    CompleteAll();
    CanBus_SetPitchPermission(1U); CHECK(CanBus_PitchAllowed());
    CHECK(CanBus_PitchEnable() == HAL_OK); CheckSpecial(sends - 1U, 2U, 0xFCU);
    CompleteAll();
    before = sends;
    CHECK(CanBus_PitchTorque(FloatBits(0x7FC00000U)) == HAL_ERROR);
    CHECK(CanBus_PitchTorque(FloatBits(0x7F800000U)) == HAL_ERROR);
    CHECK(CanBus_PitchTorque(FloatBits(0xFF800000U)) == HAL_ERROR && sends == before);
    CHECK(CanBus_PitchTorque(2.0f) == HAL_OK && TorqueRaw(sends - 1U) > 2067U);
    CompleteAll();
    CHECK(CanBus_PitchTorque(100.0f) == HAL_OK && TorqueRaw(sends - 1U) == 4095U);
    CompleteAll();
    CHECK(CanBus_PitchTorque(-100.0f) == HAL_OK && TorqueRaw(sends - 1U) == 0U);
    CompleteAll();
    before = sends;
    CanBus_SetOutputPermission(1U);
    arbitrary.bus = 1U; arbitrary.id = 2U; arbitrary.dlc = 8U;
    memset(arbitrary.data, 0xFF, 8U); arbitrary.data[7] = 0xFCU;
    CHECK(CanBus_Send(&arbitrary) == HAL_ERROR);
    arbitrary.id = 3U; CHECK(CanBus_Send(&arbitrary) == HAL_ERROR && sends == before);

    test_stage = 2U;
    CanBus_SetYawPermission(1U);
    CHECK(CanBus_YawEnable() == HAL_OK); yaw_box = tx[sends - 1U].mailbox;
    CHECK(CanBus_PitchTorque(1.0f) == HAL_OK); pitch_box = tx[sends - 1U].mailbox;
    CHECK(Fixture_OtherTx(&hcan1) == HAL_OK); other_box = tx[sends - 1U].mailbox;
    CHECK(yaw_box != pitch_box && pitch_box != other_box && CanBus_PitchTxPending() == pitch_box);
    CHECK(CanBus_YawTxPending() == yaw_box);
    before = sends;
    CHECK(CanBus_PitchTorque(1.0f) == HAL_BUSY && sends == before);
    CanBus_GetState(&state); CHECK(state.fault_epoch == 0U && CanBus_PitchAllowed() && CanBus_YawAllowed());
    abort_mask[0] = abort_mask[1] = 0U;
    Fixture_CancelOther();
    CHECK(abort_mask[0] == other_box && abort_mask[1] == 0U);
    CHECK(pending[0] == (pitch_box | yaw_box) && CanBus_PitchAllowed() && CanBus_YawAllowed());
    keep_pending = 1U; abort_mask[0] = 0U; before = sends;
    CHECK(CanBus_PitchDisable(1U) == HAL_BUSY && sends == before);
    CHECK(abort_mask[0] == pitch_box && CanBus_YawTxPending() == yaw_box && CanBus_YawAllowed());
    CHECK(!CanBus_PitchAllowed());
    keep_pending = 0U;
    CHECK(CanBus_PitchDisable(1U) == HAL_OK); CheckSpecial(sends - 1U, 2U, 0xFDU);
    CHECK(CanBus_YawTxPending() == yaw_box && CanBus_YawAllowed());
    CHECK(CanBus_CancelPitchTx() == HAL_OK && CanBus_PitchTxPending() == 0U);
    CHECK(CanBus_YawTxPending() == yaw_box);
    CanBus_SetPitchPermission(1U);
    CHECK(CanBus_PitchReadFeedback() == HAL_OK); pitch_box = tx[sends - 1U].mailbox;
    CHECK(Fixture_OtherTx(&hcan1) == HAL_OK); other_box = tx[sends - 1U].mailbox;
    abort_mask[0] = 0U;
    CanBus_SetYawPermission(0U);
    CHECK(abort_mask[0] == yaw_box && CanBus_PitchTxPending() == pitch_box && CanBus_PitchAllowed());
    CHECK(pending[0] == (pitch_box | other_box));
    abort_mask[0] = 0U; CanBus_SetPitchPermission(0U);
    CHECK(abort_mask[0] == pitch_box && pending[0] == other_box);
    Fixture_CancelOther(); CompleteAll();

    test_stage = 3U;
    /* 旧完成/取消IRQ迟到，邮箱已经被新CC占用：不可丢失新Pitch归属。 */
    CHECK(CanBus_PitchReadFeedback() == HAL_OK && tx[sends - 1U].mailbox == 1U);
    pending[0] = 0U; /* 仅模拟硬件已发送，暂不回调。 */
    CHECK(CanBus_PitchReadFeedback() == HAL_OK && tx[sends - 1U].mailbox == 1U);
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    HAL_CAN_TxMailbox0AbortCallback(&hcan1);
    CHECK(CanBus_PitchTxPending() == 1U);
    keep_pending = 1U;
    CHECK(CanBus_CancelPitchTx() == HAL_BUSY && CanBus_PitchTxPending() == 1U);
    keep_pending = 0U; CHECK(CanBus_CancelPitchTx() == HAL_OK);
    CHECK(CanBus_PitchTxPending() == 0U);
    CompleteAll();
    CanBus_SetPitchPermission(1U); CanBus_SetYawPermission(1U);
    CHECK(CanBus_PitchTorque(1.0f) == HAL_OK); pitch_box = tx[sends - 1U].mailbox;
    CHECK(CanBus_YawEnable() == HAL_OK); yaw_box = tx[sends - 1U].mailbox;
    CHECK(Fixture_OtherTx(&hcan2) == HAL_OK);
    other_box = tx[sends - 1U].mailbox;
    CanBus_SetOutputPermission(1U);
    CHECK(CanBus_JogVescCurrent(70U, 500) == HAL_OK);
    abort_mask[0] = abort_mask[1] = 0U;
    CanBus_SetOutputPermission(0U);
    CHECK(abort_mask[0] == 0U && pending[0] == (pitch_box | yaw_box));
    CHECK(pending[1] == other_box && CanBus_PitchAllowed() && CanBus_YawAllowed());
    CHECK(CanBus_StopFlywheels() == HAL_OK);
    CHECK(abort_mask[0] == 0U && CanBus_PitchTxPending() == pitch_box && CanBus_YawTxPending() == yaw_box);
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_BUSY);
    CompleteAll();
    CanBus_SetPitchPermission(0U); CanBus_SetYawPermission(0U);
    CHECK(CanBus_YawClearComms() == HAL_OK); CheckSpecial(sends - 1U, 1U, 0xFBU);
    CompleteAll(); CanBus_SetPitchPermission(1U); before = sends;
    CHECK(CanBus_YawClearComms() == HAL_ERROR && sends == before);

    test_stage = 4U;
    CanBus_SetYawPermission(1U); CanBus_SetOutputPermission(1U);
    CHECK(CanBus_YawEnable() == HAL_OK);
    abort_mask[0] = abort_mask[1] = 0U; fail_send = 1U;
    CHECK(CanBus_PitchTorque(1.0f) == HAL_ERROR);
    CHECK(abort_mask[0] == 7U && abort_mask[1] == 7U);
    CHECK(!CanBus_PitchAllowed() && !CanBus_YawAllowed() && !CanBus_OutputAllowed());
    CanBus_GetState(&state); CHECK(state.fault_epoch == 1U && state.errors[0] == 1U);
    fail_send = 0U; before = sends;
    CanBus_SetPitchPermission(1U); CHECK(!CanBus_PitchAllowed());
    CHECK(CanBus_PitchReadFeedback() == HAL_ERROR && CanBus_PitchEnable() == HAL_ERROR && sends == before);
    CHECK(CanBus_PitchDisable(1U) == HAL_OK); CheckSpecial(sends - 1U, 2U, 0xFDU);
    CompleteAll();
    error_bits[1] = 0x80U; abort_mask[0] = abort_mask[1] = 0U;
    HAL_CAN_ErrorCallback(&hcan2);
    CHECK(abort_mask[0] == 7U && abort_mask[1] == 7U);
    CHECK(!CanBus_PitchAllowed() && !CanBus_YawAllowed());
    before = sends; i = aborts;
    CanBus_GetDiagnostics(&diagnostics);
    CHECK(sends == before && aborts == i && diagnostics.state.fault_epoch == 2U);
    CHECK(diagnostics.state.first_error_source[0] == CAN_BUS_SOURCE_TX);
    CHECK(diagnostics.state.first_error_source[1] == CAN_BUS_SOURCE_IRQ);
    CHECK(CanBus_PitchDisable(1U) == HAL_OK); CheckSpecial(sends - 1U, 2U, 0xFDU);
    test_done = 1U;
}
