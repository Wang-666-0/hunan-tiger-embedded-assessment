#include "vesc_diag.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

static VescDiagReceiver receiver;
static uint8_t payload[VESC_DIAG_MAX_PAYLOAD + 1U];

static uint16_t crc(const uint8_t *data, uint16_t length)
{
    uint16_t value = 0U, i;
    uint8_t b;
    for (i = 0U; i < length; ++i) {
        value ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0U; b < 8U; ++b) {
            value = (uint16_t)((value & 0x8000U) ?
                ((uint32_t)value << 1) ^ 0x1021U : (uint32_t)value << 1);
        }
    }
    return value;
}

static CanFrame reply(uint32_t type)
{
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.bus = APP_FLYWHEEL_CAN_BUS;
    frame.extended = 1U;
    frame.id = (type << 8) | APP_DIAG_VESC_HOST_ID;
    return frame;
}

static CanFrame end_frame(uint8_t node, const uint8_t *data, uint16_t length)
{
    CanFrame frame = reply(7U);
    uint16_t value = crc(data, length);
    frame.dlc = 6U;
    frame.data[0] = node;
    frame.data[1] = 1U;
    frame.data[2] = (uint8_t)(length >> 8);
    frame.data[3] = (uint8_t)length;
    frame.data[4] = (uint8_t)(value >> 8);
    frame.data[5] = (uint8_t)value;
    return frame;
}

static void fill_payload(const uint8_t *data, uint16_t length)
{
    uint16_t offset = 0U;
    while (offset < length) {
        CanFrame frame = reply(offset <= 255U ? 5U : 6U);
        uint8_t header = (uint8_t)(offset <= 255U ? 1U : 2U);
        uint16_t size = (uint16_t)(8U - header);
        if (size > length - offset) { size = (uint16_t)(length - offset); }
        if (header == 1U) { frame.data[0] = (uint8_t)offset; }
        else {
            frame.data[0] = (uint8_t)(offset >> 8);
            frame.data[1] = (uint8_t)offset;
        }
        memcpy(frame.data + header, data + offset, size);
        frame.dlc = (uint8_t)(size + header);
        CHECK(VescDiag_OnFrame(&receiver, &frame) == 0);
        CHECK(receiver.next_offset == offset + size && receiver.active);
        offset = (uint16_t)(offset + size);
    }
}

void test_run(void)
{
    CanFrame frame, original;
    uint16_t i;
    uint8_t command;
    const uint8_t valid_commands[] = {0U, 4U, 14U, 17U, 91U};
    test_stage = 1U;
    CHECK(crc((const uint8_t *)"123456789", 9U) == 0x31C3U);
    for (i = 0U; i < 256U; ++i) {
        uint8_t allowed = (uint8_t)(i == 0U || i == 4U || i == 14U || i == 17U || i == 91U);
        memset(&frame, 0xA5, sizeof(frame)); original = frame;
        CHECK(VescDiag_BuildRead(70U, (uint8_t)i, &frame) == allowed);
        if (allowed) {
            CHECK(frame.bus == 2U && frame.extended == 1U && frame.id == 0x846U);
            CHECK(frame.dlc == 3U && frame.data[0] == APP_DIAG_VESC_HOST_ID);
            CHECK(frame.data[1] == 0U && frame.data[2] == i && frame.data[3] == 0U);
        } else { CHECK(memcmp(&frame, &original, sizeof(frame)) == 0); }
    }
    for (i = 0U; i < 256U; ++i) {
        CHECK(VescDiag_BuildRead((uint8_t)i, 4U, &frame) == (uint8_t)(i == 70U || i == 71U));
    }
    CHECK(VescDiag_BuildRead(70U, 0U, 0) == 0U);
    VescDiag_Begin(0, 70U, 4U); VescDiag_Reset(0);
    VescDiag_Begin(&receiver, 69U, 4U); CHECK(!receiver.active);
    VescDiag_Begin(&receiver, 70U, 30U); CHECK(!receiver.active);

    test_stage = 2U;
    for (i = 0U; i < sizeof(valid_commands); ++i) {
        command = valid_commands[i];
        VescDiag_Begin(&receiver, 71U, command);
        frame = reply(8U); frame.dlc = 4U;
        frame.data[0] = 71U; frame.data[1] = 1U; frame.data[2] = command; frame.data[3] = 0xABU;
        CHECK(VescDiag_OnFrame(&receiver, 0) == 0 && receiver.active);
        CHECK(VescDiag_OnFrame(0, &frame) == 0);
        frame.bus = 1U; CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
        frame.bus = 2U; frame.extended = 0U;
        CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
        frame.extended = 1U; frame.id++;
        CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
        frame.id = (8U << 8) | APP_DIAG_VESC_HOST_ID; frame.data[0] = 70U;
        CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
        frame.data[0] = 71U; frame.data[1] = 0U;
        CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
        frame.data[1] = 1U; frame.data[2] = 123U;
        CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
        frame.data[2] = command; CHECK(VescDiag_OnFrame(&receiver, &frame) == 1);
        CHECK(!receiver.active && receiver.length == 2U && receiver.buffer[0] == command);
        CHECK(receiver.buffer[1] == 0xABU && VescDiag_OnFrame(&receiver, &frame) == 0);
    }
    VescDiag_Reset(&receiver); CHECK(receiver.length == 0U && receiver.next_offset == 0U);

    test_stage = 3U;
    for (i = 0U; i < sizeof(payload); ++i) { payload[i] = (uint8_t)(i * 17U + 3U); }
    payload[0] = 4U;
    VescDiag_Begin(&receiver, 70U, 4U); fill_payload(payload, 7U);
    frame = end_frame(70U, payload, 7U); frame.data[0] = 71U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
    frame.data[0] = 70U; frame.data[1] = 0U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == 0 && receiver.active);
    frame.data[1] = 1U; CHECK(VescDiag_OnFrame(&receiver, &frame) == 1);
    CHECK(receiver.length == 7U && memcmp(receiver.buffer, payload, 7U) == 0);
    VescDiag_Begin(&receiver, 70U, 4U); fill_payload(payload, 513U);
    frame = end_frame(70U, payload, 513U);
    CHECK(VescDiag_OnFrame(&receiver, &frame) == 1);
    CHECK(receiver.length == 513U && memcmp(receiver.buffer, payload, 513U) == 0);

    test_stage = 4U;
    VescDiag_Begin(&receiver, 70U, 4U); fill_payload(payload, VESC_DIAG_MAX_PAYLOAD);
    frame = end_frame(70U, payload, VESC_DIAG_MAX_PAYLOAD);
    CHECK(VescDiag_OnFrame(&receiver, &frame) == 1);
    CHECK(receiver.length == VESC_DIAG_MAX_PAYLOAD);
    CHECK(memcmp(receiver.buffer, payload, VESC_DIAG_MAX_PAYLOAD) == 0);
    VescDiag_Begin(&receiver, 70U, 4U); fill_payload(payload, VESC_DIAG_MAX_PAYLOAD);
    frame = reply(6U); frame.dlc = 3U; frame.data[0] = 4U; frame.data[1] = 0U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1 && !receiver.active);
    CHECK(receiver.length == 0U);

    test_stage = 5U;
    VescDiag_Begin(&receiver, 70U, 4U); frame = reply(5U); frame.dlc = 2U;
    frame.data[0] = 7U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1 && !receiver.active);
    VescDiag_Begin(&receiver, 70U, 4U); frame = reply(5U); frame.dlc = 1U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);
    VescDiag_Begin(&receiver, 70U, 4U); frame = reply(6U); frame.dlc = 2U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);
    VescDiag_Begin(&receiver, 70U, 4U); frame = reply(5U); frame.dlc = 2U;
    frame.data[1] = 4U; CHECK(VescDiag_OnFrame(&receiver, &frame) == 0);
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1 && !receiver.active);
    VescDiag_Begin(&receiver, 70U, 4U); fill_payload(payload, 9U);
    frame = end_frame(70U, payload, 9U); frame.data[4] ^= 1U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1 && !receiver.active);
    VescDiag_Begin(&receiver, 70U, 4U); fill_payload(payload, 9U);
    frame = end_frame(70U, payload, 8U);
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);
    VescDiag_Begin(&receiver, 70U, 4U); frame = end_frame(70U, payload, 0U);
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);
    VescDiag_Begin(&receiver, 70U, 4U); frame = reply(8U); frame.dlc = 9U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);
    VescDiag_Begin(&receiver, 70U, 4U); frame = reply(8U); frame.dlc = 1U; frame.data[0] = 70U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);
    VescDiag_Begin(&receiver, 70U, 4U); frame.dlc = 2U; frame.data[1] = 1U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);
    VescDiag_Begin(&receiver, 70U, 4U); frame = end_frame(70U, payload, 9U); frame.dlc = 7U;
    CHECK(VescDiag_OnFrame(&receiver, &frame) == -1);

    test_stage = 6U;
    VescDiag_Begin(&receiver, 70U, 4U);
    payload[0] = 17U; fill_payload(payload, 9U); frame = end_frame(70U, payload, 9U);
    CHECK(VescDiag_OnFrame(&receiver, &frame) == 0);
    CHECK(receiver.active && receiver.length == 0U && receiver.next_offset == 0U);
    payload[0] = 4U; fill_payload(payload, 9U); frame = end_frame(70U, payload, 9U);
    CHECK(VescDiag_OnFrame(&receiver, &frame) == 1 && receiver.length == 9U);
    test_done = 1U;
}
