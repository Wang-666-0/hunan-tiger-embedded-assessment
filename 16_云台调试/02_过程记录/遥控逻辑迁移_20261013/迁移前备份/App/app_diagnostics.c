#include "app_diagnostics.h"
#include "app_config.h"
#include "can_bus.h"
#include "dm_diag.h"
#include "vesc_diag.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static VescDiagReceiver receiver;
static DiagPassiveFrame passive[APP_DIAG_PASSIVE_SLOTS];
static uint8_t result[VESC_DIAG_MAX_PAYLOAD];
static uint16_t result_length, result_offset;
static DiagKind current_kind;
static uint8_t current_node, current_field;
static uint8_t pending, active, ready;
static DiagError result_error;
static uint32_t started_ms, request_epoch, cancel_sequence, observed_cancel;
static uint32_t dm_master_id;
static uint8_t dm_master_verified;

uint8_t AppDiagnostics_Request(DiagKind kind, uint8_t node, uint8_t field)
{
    CanFrame check;
    uint8_t accepted = 0U;
    if ((kind == DIAG_DM && (node != APP_PITCH_DEVICE_ID || !DmDiag_BuildRead(field, &check))) ||
        (kind == DIAG_VESC && !VescDiag_BuildRead(node, field, &check)) ||
        (kind != DIAG_DM && kind != DIAG_VESC)) { return 0U; }
    taskENTER_CRITICAL();
    if (!pending && !active && !ready && observed_cancel == cancel_sequence) {
        current_kind = kind; current_node = node; current_field = field;
        pending = 1U;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void AppDiagnostics_Cancel(void)
{
    taskENTER_CRITICAL();
    cancel_sequence++;
    /* 取消立即撤销发送，CAN任务下次Poll再清理拼接状态。 */
    CanBus_CancelReads();
    taskEXIT_CRITICAL();
}

static void Complete(DiagError error, const uint8_t *data, uint16_t length)
{
    if (error == DIAG_ERROR_NONE && (data == 0 || length == 0U || length > sizeof(result))) {
        error = DIAG_ERROR_BAD_REPLY; length = 0U;
    }
    if (error == DIAG_ERROR_NONE) { memcpy(result, data, length); }
    taskENTER_CRITICAL();
    result_error = error;
    result_length = error == DIAG_ERROR_NONE ? length : 0U;
    result_offset = 0U;
    pending = 0U; active = 0U; ready = 1U;
    taskEXIT_CRITICAL();
    VescDiag_Reset(&receiver);
    CanBus_CancelReads();
}

void AppDiagnostics_Poll(uint32_t now)
{
    CanBusState bus;
    HAL_StatusTypeDef status;
    CanBus_GetState(&bus);
    taskENTER_CRITICAL();
    if (observed_cancel != cancel_sequence) {
        observed_cancel = cancel_sequence;
        pending = 0U; active = 0U; ready = 0U;
        dm_master_verified = 0U;
        VescDiag_Reset(&receiver);
    }
    if (active) {
        if (request_epoch != bus.fault_epoch) { Complete(DIAG_ERROR_CAN_FAULT, 0, 0U); }
        else if ((uint32_t)(now - started_ms) >= APP_DIAG_TIMEOUT_MS) { Complete(DIAG_ERROR_TIMEOUT, 0, 0U); }
    }
    if (pending && !active && !ready) {
        pending = 0U; active = 1U;
        started_ms = now; request_epoch = bus.fault_epoch;
        if (current_kind == DIAG_VESC) {
            VescDiag_Begin(&receiver, current_node, current_field);
            status = CanBus_ReadVesc(current_node, current_field);
        }
        else { status = CanBus_ReadDm(current_field); }
        if (status != HAL_OK) { Complete(DIAG_ERROR_TX_FAILED, 0, 0U); }
    }
    taskEXIT_CRITICAL();
}

static void ObservePassive(const CanFrame *frame)
{
    uint32_t i, chosen = APP_DIAG_PASSIVE_SLOTS;
    /* VESC分片查询报文不挤占观察表；其结果已经单独完整保存。
     * CAN1保留任意数据帧，CAN2保留标准帧和VESC STATUS广播。 */
    if (frame->bus == 2U && frame->extended) {
        uint32_t command = frame->id >> 8;
        if (command != 9U && command != 14U && command != 15U &&
            command != 16U && command != 27U && command != 58U) { return; }
    }
    taskENTER_CRITICAL();
    for (i = 0U; i < APP_DIAG_PASSIVE_SLOTS; ++i) {
        if (passive[i].count && passive[i].frame.bus == frame->bus &&
            passive[i].frame.extended == frame->extended && passive[i].frame.id == frame->id) { chosen = i; break; }
        if (!passive[i].count && chosen == APP_DIAG_PASSIVE_SLOTS) { chosen = i; }
    }
    if (chosen < APP_DIAG_PASSIVE_SLOTS) {
        passive[chosen].frame = *frame;
        passive[chosen].count++;
        if (!passive[chosen].count) { passive[chosen].count = 1U; }
    }
    taskEXIT_CRITICAL();
}

void AppDiagnostics_OnFrame(const CanFrame *frame)
{
    int parsed;
    if (!CanFrame_IsValid(frame)) { return; }
    ObservePassive(frame);
    if (!active || observed_cancel != cancel_sequence || frame->fault_epoch != request_epoch ||
        (uint32_t)(frame->received_ms - started_ms) >= 0x80000000UL) { return; }
    if ((uint32_t)(frame->received_ms - started_ms) >= APP_DIAG_TIMEOUT_MS) {
        Complete(DIAG_ERROR_TIMEOUT, 0, 0U); return;
    }
    if (current_kind == DIAG_DM) {
        if (DmDiag_MatchesReply(current_field, frame)) {
            if (current_field == DM_DIAG_RID_MASTER_ID) {
                uint32_t master = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8) |
                    ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);
                /* 第一条先读MasterID，核对参数值和本次回复CAN ID一致。
                 * 随后的参数回复限定到此ID，减少普通反馈偶然同形的误匹配。 */
                if (master != frame->id) { Complete(DIAG_ERROR_BAD_REPLY, 0, 0U); return; }
                dm_master_id = master; dm_master_verified = 1U;
            }
            else if (dm_master_verified && frame->id != dm_master_id) { return; }
            Complete(DIAG_ERROR_NONE, frame->data, 8U);
        }
    }
    else {
        parsed = VescDiag_OnFrame(&receiver, frame);
        if (parsed == 1) { Complete(DIAG_ERROR_NONE, receiver.buffer, receiver.length); }
        else if (parsed < 0) { Complete(DIAG_ERROR_BAD_REPLY, 0, 0U); }
    }
}

uint8_t AppDiagnostics_TakeEvent(DiagEvent *out)
{
    uint16_t remaining;
    if (out == 0) { return 0U; }
    taskENTER_CRITICAL();
    if (!ready || observed_cancel != cancel_sequence) { taskEXIT_CRITICAL(); return 0U; }
    memset(out, 0, sizeof(*out));
    out->kind = current_kind; out->node = current_node; out->field = current_field;
    out->total = result_length; out->offset = result_offset;
    if (result_error != DIAG_ERROR_NONE) {
        out->event = DIAG_EVENT_ERROR; out->error = result_error; ready = 0U;
    }
    else if (result_offset < result_length) {
        out->event = DIAG_EVENT_DATA;
        remaining = result_length - result_offset;
        out->length = (uint8_t)(remaining > sizeof(out->data) ? sizeof(out->data) : remaining);
        memcpy(out->data, result + result_offset, out->length);
        result_offset += out->length;
    }
    else { out->event = DIAG_EVENT_DONE; ready = 0U; }
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t AppDiagnostics_GetPassive(uint32_t index, DiagPassiveFrame *out)
{
    if (out == 0 || index >= APP_DIAG_PASSIVE_SLOTS) { return 0U; }
    taskENTER_CRITICAL();
    *out = passive[index];
    taskEXIT_CRITICAL();
    return out->count != 0U;
}
