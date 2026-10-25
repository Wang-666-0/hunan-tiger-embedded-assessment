/**
 * @file app_diagnostics.c
 * @brief 只读设备诊断
 *
 * 职责：构造参数读取请求、拼接回复，处理超时并向控制台交付诊断结果。
 * 主要函数：
 *   - DmDiag_BuildRead / MatchesReply：白名单 RID 查询及回复核对。
 *   - VescDiag_Begin / OnFrame：VESC 分片接收、长度和 CRC 校验。
 *   - AppDiagnostics_Request / Poll：串行推进查询及超时处理。
 *   - AppDiagnostics_TakeEvent / Cancel：交付结果或撤销未完成查询。
 * 调用关系：控制台请求 → CanTask 轮询 → CAN 回复 → 控制台逐段读取。
 * 维护约束：本模块仅查询；取消代次隔离旧回复，不添加使能或写入设备参数的行为。
 * 参数入口：app_config.h；对外声明仍在原头文件中。
 */

/* APP_SECTION_BEGIN dm_diag.c */
/* 达妙 RID 白名单与只读查询编码。 */
#include "dm_diag.h"
#include "app_config.h"
#include <string.h>

/* 依据 dmBots/motor-sdk 官方 Python 示例的 __read_RID_param、
 * __process_set_param_packet 与 DM_variable：
 * https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py
 * 固定发送 0x33 读取操作；本模块没有写参数、保存、使能或运动操作。
 * 7..10、13..16、35..36 为 uint32 小端；20..23 为 float32 小端。
 */
const uint8_t DmDiag_ReadableRids[] = {
    DM_DIAG_RID_MASTER_ID, DM_DIAG_RID_SLAVE_ID, DM_DIAG_RID_TIMEOUT,
    DM_DIAG_RID_CONTROL_MODE, DM_DIAG_RID_HW_VERSION, DM_DIAG_RID_SW_VERSION,
    DM_DIAG_RID_SERIAL_NUMBER, DM_DIAG_RID_NPP, DM_DIAG_RID_GEAR_RATIO,
    DM_DIAG_RID_PMAX, DM_DIAG_RID_VMAX, DM_DIAG_RID_TMAX,
    DM_DIAG_RID_CAN_BAUD, DM_DIAG_RID_SUB_VERSION
};
const uint32_t DmDiag_ReadableRidCount = sizeof(DmDiag_ReadableRids) / sizeof(DmDiag_ReadableRids[0]);

uint8_t DmDiag_IsReadableRid(uint8_t rid)
{
    uint32_t i;
    for (i = 0U; i < DmDiag_ReadableRidCount; ++i) {
        if (DmDiag_ReadableRids[i] == rid) { return 1U; }
    }
    return 0U;
}

uint8_t DmDiag_BuildRead(uint8_t rid, CanFrame *out)
{
    if (out == 0 || !DmDiag_IsReadableRid(rid)) { return 0U; }
    memset(out, 0, sizeof(*out));
    out->bus = APP_PITCH_CAN_BUS;
    out->id = 0x7FFU;
    out->dlc = 8U;
    out->data[0] = (uint8_t)(APP_PITCH_DEVICE_ID & 0xFFU);
    out->data[1] = (uint8_t)((APP_PITCH_DEVICE_ID >> 8) & 0xFFU);
    out->data[2] = 0x33U;
    out->data[3] = rid;
    return 1U;
}

uint8_t DmDiag_MatchesReply(uint8_t rid, const CanFrame *frame)
{
    if (frame == 0 || !DmDiag_IsReadableRid(rid) || !CanFrame_IsValid(frame)) { return 0U; }
    if (frame->bus != APP_PITCH_CAN_BUS || frame->extended || frame->dlc != 8U) { return 0U; }
    /* 参数回复的 CAN ID 是电机配置的 MasterID，可以为 0，不能猜为从机 ID。
     * 0x7FF 是读取请求地址，保守拒绝，防止把请求回显当作零值回复。
     * 如果实机 MasterID 恰为 0x7FF，应使用官方工具核实，不能放宽此检查。
     */
    if (frame->id == 0x7FFU) { return 0U; }
    return (uint8_t)(frame->data[0] == (uint8_t)(APP_PITCH_DEVICE_ID & 0xFFU)
        && frame->data[1] == (uint8_t)((APP_PITCH_DEVICE_ID >> 8) & 0xFFU)
        && frame->data[2] == 0x33U && frame->data[3] == rid);
}
/* APP_SECTION_END dm_diag.c */

/* APP_SECTION_BEGIN vesc_diag.c */
/* VESC 只读查询、分片重组和 CRC。 */
#include "vesc_diag.h"
#include "app_config.h"
#include <string.h>


static uint8_t read_is_allowed(uint8_t node, uint8_t command)
{
    if (node != APP_FLYWHEEL_LEFT_ID && node != APP_FLYWHEEL_RIGHT_ID) {
        return 0U;
    }
    return (uint8_t)(command == 0U || command == 4U || command == 14U ||
                     command == 17U || command == 91U);
}

static uint16_t payload_crc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0U, i;
    uint8_t bit;
    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc & 0x8000U) != 0U ?
                ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
        }
    }
    return crc;
}

uint8_t VescDiag_BuildRead(uint8_t node, uint8_t command, CanFrame *out)
{
    if (out == 0 || !read_is_allowed(node, command)) { return 0U; }
    memset(out, 0, sizeof(*out));
    out->bus = APP_FLYWHEEL_CAN_BUS;
    out->extended = 1U;
    out->id = (VESC_PROCESS_SHORT_BUFFER << 8) | node;
    out->dlc = 3U;
    out->data[0] = APP_DIAG_VESC_HOST_ID;
    out->data[1] = 0U; /* send=0：电调处理请求，再将响应发回data[0]。 */
    out->data[2] = command;
    return 1U;
}

void VescDiag_Reset(VescDiagReceiver *receiver)
{
    if (receiver != 0) { memset(receiver, 0, sizeof(*receiver)); }
}

void VescDiag_Begin(VescDiagReceiver *receiver, uint8_t node, uint8_t command)
{
    if (receiver == 0) { return; }
    VescDiag_Reset(receiver);
    if (!read_is_allowed(node, command)) { return; }
    receiver->expected_node = node;
    receiver->expected_command = command;
    receiver->active = 1U;
}

static int damaged(VescDiagReceiver *receiver)
{
    receiver->length = 0U;
    receiver->next_offset = 0U;
    receiver->active = 0U;
    return -1;
}

int VescDiag_OnFrame(VescDiagReceiver *receiver, const CanFrame *frame)
{
    uint32_t packet_type;
    uint16_t offset, payload_length, expected_crc;
    uint8_t header_length;
    if (receiver == 0 || frame == 0 || !receiver->active) { return 0; }
    if (frame->bus != APP_FLYWHEEL_CAN_BUS || frame->extended != 1U ||
        frame->id > 0x1FFFFFFFU || (frame->id & 0xFFU) != APP_DIAG_VESC_HOST_ID) {
        return 0;
    }
    packet_type = frame->id >> 8;
    if (packet_type < VESC_FILL_RX_BUFFER || packet_type > VESC_PROCESS_SHORT_BUFFER) {
        return 0;
    }
    if (frame->dlc > 8U) { return damaged(receiver); }

    if (packet_type == VESC_PROCESS_SHORT_BUFFER || packet_type == VESC_PROCESS_RX_BUFFER) {
        /* 收到其他节点或请求方向的帧不能结束本节点正在等待的查询。 */
        if (frame->dlc == 0U) { return damaged(receiver); }
        if (frame->data[0] != receiver->expected_node) { return 0; }
        if (frame->dlc < 2U) { return damaged(receiver); }
        if (frame->data[1] != 1U) { return 0; }
        if (packet_type == VESC_PROCESS_SHORT_BUFFER) {
            if (frame->dlc < 3U) { return damaged(receiver); }
            if (frame->data[2] != receiver->expected_command) { return 0; }
            receiver->length = (uint16_t)(frame->dlc - 2U);
            memcpy(receiver->buffer, frame->data + 2U, receiver->length);
            receiver->next_offset = receiver->length;
            receiver->active = 0U;
            return 1;
        }

        /* 官方comm_can_send_buffer的长包终结帧恰好含6个字节。 */
        if (frame->dlc != 6U) { return damaged(receiver); }
        payload_length = (uint16_t)(((uint16_t)frame->data[2] << 8) | frame->data[3]);
        expected_crc = (uint16_t)(((uint16_t)frame->data[4] << 8) | frame->data[5]);
        if (payload_length == 0U || payload_length > VESC_DIAG_MAX_PAYLOAD ||
            payload_length != receiver->next_offset ||
            payload_crc(receiver->buffer, payload_length) != expected_crc) {
            return damaged(receiver);
        }
        if (receiver->buffer[0] != receiver->expected_command) {
            receiver->length = 0U;
            receiver->next_offset = 0U;
            return 0;
        }
        receiver->length = payload_length;
        receiver->active = 0U;
        return 1;
    }

    header_length = (uint8_t)(packet_type == VESC_FILL_RX_BUFFER ? 1U : 2U);
    if (frame->dlc <= header_length) { return damaged(receiver); }
    offset = frame->data[0];
    if (header_length == 2U) {
        offset = (uint16_t)(((uint16_t)frame->data[0] << 8) | frame->data[1]);
    }
    payload_length = (uint16_t)(frame->dlc - header_length);
    /* 连续偏移防止漏片、重复片和乱序片被当成完整配置。
     * 不自动重启拼接或重发查询；故障包留给上层明确报告。 */
    if (offset != receiver->next_offset ||
        (uint32_t)offset + payload_length > VESC_DIAG_MAX_PAYLOAD) {
        return damaged(receiver);
    }
    memcpy(receiver->buffer + offset, frame->data + header_length, payload_length);
    receiver->next_offset = (uint16_t)(offset + payload_length);
    return 0;
}
/* APP_SECTION_END vesc_diag.c */

/* APP_SECTION_BEGIN app_diagnostics.c */
/* 诊断会话调度、取消、超时与结果交付。 */
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
/* APP_SECTION_END app_diagnostics.c */
