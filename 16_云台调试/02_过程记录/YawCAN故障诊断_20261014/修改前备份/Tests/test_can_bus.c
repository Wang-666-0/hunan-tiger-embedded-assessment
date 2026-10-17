#include "can_bus.h"
#include "app_fault.h"
#include "app_config.h"
#include "dm_diag.h"
#include "queue.h"
#include <string.h>
#ifndef TEST_HIGH_SPEED
#define TEST_HIGH_SPEED 0
#endif
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
CAN_HandleTypeDef hcan1 = {1U}, hcan2 = {2U};
static CanFrame queued;
static CAN_RxHeaderTypeDef incoming;
static CAN_FilterTypeDef filters[2];
static uint32_t queue_has_frame, incoming_bus, sends, starts, notices, aborts, now = 123U;
static uint8_t overflow, read_failure, abort_failure_bus, keep_pending, probe_on_abort;
static uint32_t error_bits[2], pending[2], fail_send_call, abort_bus_mask;
static CAN_TxHeaderTypeDef sent_header;
static uint8_t sent_data[8];
static uint32_t sent_bus;
typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t bus, aborts_before;
} TxRecord;
static TxRecord tx[96];
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
{
    uint32_t before = sends;
    CHECK(mailboxes == 7U); aborts++; abort_bus_mask |= 1U << (handle->instance - 1U);
    if (probe_on_abort) {
        CHECK(!CanBus_OutputAllowed());
        CHECK(CanBus_JogVescCurrent(70U, 200) == HAL_ERROR && sends == before);
    }
    if (abort_failure_bus == handle->instance) { return HAL_ERROR; }
    if (!keep_pending) { pending[handle->instance - 1U] = 0U; }
    return HAL_OK;
}
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *handle, uint32_t mailboxes)
{ CHECK(mailboxes == 7U); return pending[handle->instance - 1U] ? 1U : 0U; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *handle, const CAN_TxHeaderTypeDef *header,
    const uint8_t *data, uint32_t *mailbox)
{
    CHECK(sends < sizeof(tx) / sizeof(tx[0]));
    *mailbox = CAN_TX_MAILBOX0;
    sent_bus = handle->instance; sent_header = *header;
    memcpy(sent_data, data, 8U);
    if (sends < sizeof(tx) / sizeof(tx[0])) {
        tx[sends].header = *header; tx[sends].bus = handle->instance;
        tx[sends].aborts_before = aborts; memcpy(tx[sends].data, data, 8U);
    }
    sends++;
    if (fail_send_call == sends) { return HAL_ERROR; }
    pending[handle->instance - 1U]++;
    return HAL_OK;
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
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *handle) { return error_bits[handle->instance - 1U]; }
uint32_t HAL_GetTick(void) { return now; }

#if APP_FLYWHEEL_JOG_ENABLED
static void CheckCurrent(uint32_t index, uint8_t node, int32_t milliamps)
{
    uint32_t raw;
    CHECK(index < sends);
    CHECK(tx[index].bus == 2U && tx[index].header.IDE == CAN_ID_EXT &&
        tx[index].header.ExtId == (0x100U | node) && tx[index].header.StdId == 0U);
    CHECK(tx[index].header.RTR == CAN_RTR_DATA && tx[index].header.DLC == 4U &&
        tx[index].header.TransmitGlobalTime == DISABLE);
    raw = ((uint32_t)tx[index].data[0] << 24) | ((uint32_t)tx[index].data[1] << 16) |
          ((uint32_t)tx[index].data[2] << 8) | tx[index].data[3];
    CHECK(raw == (uint32_t)milliamps);
    CHECK(tx[index].data[4] == 0U && tx[index].data[5] == 0U &&
        tx[index].data[6] == 0U && tx[index].data[7] == 0U);
}

static void TestJog(void)
{
    uint32_t before = sends, before_aborts = aborts, fault_before, errors_before;
    CanFrame raw = {0};
    CanBusState state;
    CHECK(APP_FLYWHEEL_JOG_LEFT_MAX_MA == (APP_FLYWHEEL_SPEED_ACTIVE ? (TEST_HIGH_SPEED ? 2000 : 1000) : 500));
    CHECK(APP_FLYWHEEL_JOG_RIGHT_MAX_MA == (APP_FLYWHEEL_SPEED_ACTIVE ? (TEST_HIGH_SPEED ? 2000 : 1000) : 500));
    CHECK(!CanBus_OutputAllowed());
    CHECK(CanBus_JogVescCurrent(70U, 200) == HAL_ERROR && sends == before);
    CHECK(CanBus_JogVescCurrent(70U, 0) == HAL_ERROR && sends == before);
    CanBus_SetOutputPermission(1U); CHECK(CanBus_OutputAllowed());
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_ERROR && sends == before && aborts == before_aborts);
    CHECK(CanBus_JogVescCurrent(70U, 500) == HAL_OK); CheckCurrent(before, 70U, 500);
    CHECK(sent_data[0] == 0U && sent_data[1] == 0U && sent_data[2] == 1U && sent_data[3] == 0xF4U);
    CHECK(CanBus_JogVescCurrent(71U, 0) == HAL_OK); CheckCurrent(before + 1U, 71U, 0U);
    CHECK(CanBus_OutputAllowed() && pending[1] == 2U);
    CHECK(CanBus_JogVescCurrent(71U, 500) == HAL_OK); CheckCurrent(before + 2U, 71U, 500U);
    CHECK(sent_data[0] == 0U && sent_data[1] == 0U && sent_data[2] == 1U && sent_data[3] == 0xF4U);
    before = sends;
#if APP_FLYWHEEL_SPEED_ACTIVE
    CHECK(CanBus_JogVescCurrent(70U, 1000) == HAL_OK); CheckCurrent(before, 70U, 1000);
    CHECK(sent_data[2] == 3U && sent_data[3] == 0xE8U);
    CHECK(CanBus_JogVescCurrent(71U, 1000) == HAL_OK); CheckCurrent(before + 1U, 71U, 1000);
    CHECK(sent_data[2] == 3U && sent_data[3] == 0xE8U);
    CHECK(CanBus_JogVescCurrent(70U, 501) == HAL_OK);
    CHECK(CanBus_JogVescCurrent(71U, 501) == HAL_OK);
    before = sends;
#if TEST_HIGH_SPEED
    CHECK(CanBus_JogVescCurrent(70U, 2000) == HAL_OK); CheckCurrent(before, 70U, 2000);
    CHECK(sent_data[0] == 0U && sent_data[1] == 0U && sent_data[2] == 7U && sent_data[3] == 0xD0U);
    CHECK(CanBus_JogVescCurrent(71U, 2000) == HAL_OK); CheckCurrent(before + 1U, 71U, 2000);
    CHECK(sent_data[2] == 7U && sent_data[3] == 0xD0U);
    before = sends;
#endif
#endif
    CHECK(CanBus_JogVescCurrent(0U, 200) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(69U, 200) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(72U, 200) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(255U, 0) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(70U, -1) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(70U, -200) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(71U, -1) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(71U, -200) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(70U, APP_FLYWHEEL_JOG_LEFT_MAX_MA + 1) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(70U, -500) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(71U, -500) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(71U, APP_FLYWHEEL_JOG_RIGHT_MAX_MA + 1) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(70U, (-2147483647 - 1)) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(70U, -201) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(70U, 2147483647) == HAL_ERROR && sends == before);
    /* 完整驱动门禁仍为0：任意总线/协议/幅值无法借点动许可穿透。 */
    raw.bus = 2U; raw.extended = 1U; raw.id = 0x146U; raw.dlc = 4U; raw.data[3] = 200U;
    CHECK(CanBus_Send(&raw) == HAL_ERROR);
    raw.id = 0x246U; CHECK(CanBus_Send(&raw) == HAL_ERROR);
    raw.id = 0x346U; CHECK(CanBus_Send(&raw) == HAL_ERROR);
    raw.id = 0x146U; raw.extended = 0U; CHECK(CanBus_Send(&raw) == HAL_ERROR);
    raw.bus = 1U; raw.extended = 1U; CHECK(CanBus_Send(&raw) == HAL_ERROR);
    CHECK(sends == before);
    /* STOP先取消两控制器，探针验证撤权早于任何取消动作。 */
    before_aborts = aborts; abort_bus_mask = 0U; probe_on_abort = 1U;
    CHECK(pending[1] > 0U && CanBus_StopFlywheels() == HAL_OK);
    probe_on_abort = 0U;
    CHECK(aborts == before_aborts + 2U && abort_bus_mask == 3U && sends == before + 2U);
    CHECK(tx[before].aborts_before == before_aborts + 2U);
    CHECK(tx[before + 1U].aborts_before == before_aborts + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    CHECK(!CanBus_OutputAllowed());
    before_aborts = aborts;
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_BUSY && sends == before + 2U && aborts == before_aborts);
    CanBus_GetState(&state); CHECK(state.fault_epoch == 0U && state.errors[1] == 0U);
    pending[0] = 0U; pending[1] = 0U;
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_OK && sends == before + 4U && aborts == before_aborts);
    CheckCurrent(before + 2U, 70U, 0U); CheckCurrent(before + 3U, 71U, 0U);
    before = sends - 2U;
    CHECK(CanBus_JogVescCurrent(71U, 1) == HAL_ERROR && sends == before + 2U);
    CHECK(CanBus_JogVescCurrent(71U, 0) == HAL_ERROR && sends == before + 2U);
    before = sends; CHECK(CanBus_StopFlywheels() == HAL_OK && sends == before + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    CanBus_SetOutputPermission(1U); CHECK(CanBus_JogVescCurrent(70U, 1) == HAL_OK);
    CheckCurrent(sends - 1U, 70U, 1);
    CHECK(sent_data[0] == 0U && sent_data[1] == 0U && sent_data[2] == 0U && sent_data[3] == 1U);
    CHECK(CanBus_StopFlywheels() == HAL_OK);
    CanBus_SetOutputPermission(1U); CHECK(CanBus_JogVescCurrent(71U, 1) == HAL_OK);
    CheckCurrent(sends - 1U, 71U, 1U);
    /* 异步取消尚未完成时不能复用邮箱；下一周期可可靠重试。 */
    keep_pending = 1U; before = sends;
    CHECK(CanBus_StopFlywheels() == HAL_ERROR && sends == before && !CanBus_OutputAllowed());
    CanBus_GetState(&state); CHECK(state.fault_epoch == 0U && state.errors[1] == 0U);
    keep_pending = 0U; CHECK(CanBus_StopFlywheels() == HAL_OK && sends == before + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    /* 双CAN取消都要尝试；任意取消失败不能报告STOP成功。 */
    abort_failure_bus = 1U; before = sends; before_aborts = aborts; abort_bus_mask = 0U;
    CHECK(CanBus_StopFlywheels() == HAL_ERROR && sends == before);
    CHECK(aborts == before_aborts + 2U && abort_bus_mask == 3U && !CanBus_OutputAllowed());
    CanBus_GetState(&state); CHECK(state.fault_epoch == 0U && state.errors[1] == 0U);
    abort_failure_bus = 0U;
    /* 尚未执行中断的实时bus-off也阻止非零及重新授权；零仍尝试但报失败。 */
    CanBus_SetOutputPermission(1U); error_bits[1] = 4U; before = sends;
    CHECK(!CanBus_OutputAllowed() && CanBus_JogVescCurrent(70U, 200) == HAL_ERROR && sends == before);
    CHECK(CanBus_JogVescCurrent(70U, 0) == HAL_ERROR && sends == before);
    CanBus_SetOutputPermission(1U); CHECK(!CanBus_OutputAllowed());
    CHECK(CanBus_StopFlywheels() == HAL_ERROR && sends == before + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    error_bits[1] = 0U;
    CHECK(CanBus_StopFlywheels() == HAL_OK && !CanBus_OutputAllowed());
    /* 点动提交失败即锁存故障：Setter1也不能再次开放，回零仍可重试。 */
    CanBus_SetOutputPermission(1U); before = sends; before_aborts = aborts;
    fail_send_call = before + 1U;
    CHECK(CanBus_JogVescCurrent(70U, 200) == HAL_ERROR);
    CHECK(sends == before + 1U && aborts == before_aborts + 2U && !CanBus_OutputAllowed());
    CanBus_SetOutputPermission(1U);
    CHECK(CanBus_JogVescCurrent(71U, 200) == HAL_ERROR && sends == before + 1U);
    CanBus_GetState(&state); CHECK(state.fault_epoch == 1U && state.errors[1] == 1U);
    fail_send_call = 0U;
    before = sends; CHECK(CanBus_StopFlywheels() == HAL_OK && sends == before + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    /* 一个零提交失败也要尝试另一零，真实提交失败锁故障且回零可重试。 */
    CanBus_GetState(&state); fault_before = state.fault_epoch; errors_before = state.errors[1];
    before = sends; fail_send_call = before + 1U;
    CHECK(CanBus_StopFlywheels() == HAL_ERROR && sends == before + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    CanBus_GetState(&state);
    CHECK(state.fault_epoch == fault_before + 1U && state.errors[1] == errors_before + 1U &&
        state.last_error[1] == 0U);
    CanBus_SetOutputPermission(1U); CHECK(!CanBus_OutputAllowed());
    CHECK(CanBus_JogVescCurrent(70U, 200) == HAL_ERROR && sends == before + 2U);
    fail_send_call = 0U; before = sends;
    CHECK(CanBus_StopFlywheels() == HAL_OK && sends == before + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    /* 刷新在历史故障后仍仅发固定零，实时故障/提交失败也不会取消读请求。 */
    pending[0] = 0U; pending[1] = 0U; before = sends; before_aborts = aborts;
    error_bits[1] = 4U;
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_ERROR && sends == before + 2U && aborts == before_aborts);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    error_bits[1] = 0U; pending[0] = 0U; pending[1] = 0U; before = sends;
    CanBus_GetState(&state); fault_before = state.fault_epoch; errors_before = state.errors[1];
    fail_send_call = before + 1U;
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_ERROR && sends == before + 2U && aborts == before_aborts);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    CanBus_GetState(&state);
    CHECK(state.fault_epoch == fault_before + 1U && state.errors[1] == errors_before + 1U &&
        state.last_error[1] == 0U);
    CanBus_SetOutputPermission(1U); CHECK(!CanBus_OutputAllowed());
    CHECK(CanBus_JogVescCurrent(71U, 200) == HAL_ERROR && sends == before + 2U && aborts == before_aborts);
    fail_send_call = 0U; pending[0] = 0U; pending[1] = 0U; before = sends;
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_OK && sends == before + 2U && aborts == before_aborts);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U);
    CHECK(CanBus_JogVescCurrent(70U, 200) == HAL_ERROR && sends == before + 2U);
}
#endif

void test_run(void)
{
    CanFrame frame = {0};
    CanBusState state;
    uint32_t before, prior_faults, prior_errors;
    test_stage = 1U;
    CHECK(!CanBus_OutputAllowed());
    CHECK(CanBus_JogVescCurrent(70U, 200) == HAL_ERROR);
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_ERROR && sends == 0U && aborts == 0U);
    CHECK(CanBus_StopFlywheels() == HAL_ERROR && sends == 0U && aborts == 0U);
    CanBus_CreateQueue(); CanBus_Start();
    CHECK(starts == 2U && notices == 2U && sends == 0U && aborts == 0U);
    CHECK(filters[0].FilterBank == 0U && filters[1].FilterBank == 14U);
    CHECK(filters[0].SlaveStartFilterBank == 14U && filters[1].FilterMaskIdLow == 0U);
    frame.bus = 1U; frame.id = 2U; frame.dlc = 8U;
    CHECK(CanBus_Send(&frame) == HAL_ERROR);
    CanBus_SetOutputPermission(1U); CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == 0U);
    frame.bus = 2U; frame.extended = 1U; frame.id = 0x346U; frame.dlc = 4U;
    CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == 0U);
    CanBus_SetOutputPermission(0U);
#if APP_FLYWHEEL_JOG_ENABLED
    TestJog();
#else
    CHECK(!CanBus_OutputAllowed()); CanBus_SetOutputPermission(1U); CHECK(!CanBus_OutputAllowed());
    CHECK(CanBus_JogVescCurrent(70U, 0) == HAL_ERROR && sends == 0U);
    CHECK(CanBus_StopFlywheels() == HAL_ERROR && sends == 0U);
    CHECK(CanBus_KeepFlywheelsStopped() == HAL_ERROR && sends == 0U);
#endif
    before = sends;
    CanBus_GetState(&state); prior_faults = state.fault_epoch; prior_errors = state.errors[1];
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
    CHECK(state.dropped == 1U && state.fault_epoch == prior_faults + 1U && !CanBus_OutputAllowed());
    error_bits[1] = 0x80U; HAL_CAN_ErrorCallback(&hcan2); CanBus_GetState(&state);
    CHECK(state.errors[1] == prior_errors + 1U && state.last_error[1] == 0x80U &&
        state.fault_epoch == prior_faults + 2U);
    CHECK(aborts >= 2U && sends == before);
    overflow = 0U; read_failure = 1U; incoming_bus = 1U; error_bits[0] = 0x80U;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); CanBus_GetState(&state);
    CHECK(state.errors[0] == 1U && state.last_error[0] == 0x80U && state.fault_epoch == prior_faults + 3U);
    CHECK(CanBus_Receive(&frame, 0U) != pdPASS);
    /* 清除测试中的实时标志仍无法清除历史fault_epoch或重开非零。 */
    error_bits[0] = 0U; error_bits[1] = 0U;
    CanBus_SetOutputPermission(1U); CHECK(!CanBus_OutputAllowed()); frame.bus = 2U;
    CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == before);
    CHECK(CanBus_JogVescCurrent(70U, 200) == HAL_ERROR && sends == before);
#if APP_FLYWHEEL_JOG_ENABLED
    CHECK(CanBus_StopFlywheels() == HAL_OK && sends == before + 2U);
    CheckCurrent(before, 70U, 0U); CheckCurrent(before + 1U, 71U, 0U); before = sends;
#endif
    test_stage = 4U;
    pending[0] = 0U; pending[1] = 0U;
    CHECK(CanBus_ReadDm(DM_DIAG_RID_MASTER_ID) == HAL_OK);
    CHECK(sends == before + 1U && sent_bus == 1U && sent_header.IDE == CAN_ID_STD &&
        sent_header.StdId == 0x7FFU && sent_header.DLC == 8U && sent_header.RTR == CAN_RTR_DATA);
    CHECK(sent_data[0] == 2U && sent_data[1] == 0U && sent_data[2] == 0x33U && sent_data[3] == 7U &&
        sent_data[4] == 0U && sent_data[5] == 0U && sent_data[6] == 0U && sent_data[7] == 0U);
    CHECK(CanBus_ReadDm(0U) == HAL_ERROR && sends == before + 1U);
#if APP_FLYWHEEL_JOG_ENABLED
    {
        uint32_t previous_aborts = aborts;
        CHECK(pending[0] > 0U && pending[1] == 0U);
        CHECK(CanBus_KeepFlywheelsStopped() == HAL_BUSY && sends == before + 1U && aborts == previous_aborts);
    }
#endif
    CHECK(CanBus_ReadVesc(70U, 0U) == HAL_OK);
    CHECK(sends == before + 2U && sent_bus == 2U && sent_header.IDE == CAN_ID_EXT &&
        sent_header.ExtId == 0x846U && sent_header.DLC == 3U &&
        sent_data[0] == APP_DIAG_VESC_HOST_ID && sent_data[1] == 0U && sent_data[2] == 0U);
    CHECK(CanBus_ReadVesc(71U, 91U) == HAL_OK && sent_header.ExtId == 0x847U && sent_data[2] == 91U);
    CHECK(CanBus_ReadVesc(70U, 8U) == HAL_ERROR && sends == before + 3U);
    CHECK(CanBus_ReadVesc(70U, 30U) == HAL_ERROR && sends == before + 3U);
    CHECK(CanBus_ReadVesc(72U, 14U) == HAL_ERROR && sends == before + 3U);
    CHECK(CanBus_Send(&frame) == HAL_ERROR && sends == before + 3U);
    before = aborts; CanBus_CancelReads(); CHECK(aborts == before + 2U);
    test_done = 1U;
}
