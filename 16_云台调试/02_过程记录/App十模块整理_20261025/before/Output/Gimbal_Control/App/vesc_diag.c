#include "vesc_diag.h"
#include "app_config.h"
#include <string.h>

#define VESC_FILL_RX_BUFFER 5U
#define VESC_FILL_RX_BUFFER_LONG 6U
#define VESC_PROCESS_RX_BUFFER 7U
#define VESC_PROCESS_SHORT_BUFFER 8U

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
