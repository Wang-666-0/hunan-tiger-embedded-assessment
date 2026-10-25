#include "app_diagnostics.h"
#include "app_config.h"
#include "can_bus.h"
#include "dm_diag.h"
#include "vesc_diag.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

static CanBusState bus_state;
static CanFrame last_query;
static uint32_t sends, cancels, now;
static HAL_StatusTypeDef send_result;

/* 只 mock 总线投递；诊断状态机和两种协议解析全部链接正式代码。 */
void CanBus_GetState(CanBusState *out) { *out = bus_state; }
HAL_StatusTypeDef CanBus_ReadDm(uint8_t rid)
{
    CHECK(DmDiag_BuildRead(rid, &last_query));
    sends++;
    return send_result;
}
HAL_StatusTypeDef CanBus_ReadVesc(uint8_t node, uint8_t command)
{
    CHECK(VescDiag_BuildRead(node, command, &last_query));
    sends++;
    return send_result;
}
void CanBus_CancelReads(void) { cancels++; }
uint32_t HAL_GetTick(void) { return now; }

static void ResetFixture(void)
{
    AppDiagnostics_Cancel();
    AppDiagnostics_Poll(0U);
    memset(&bus_state, 0, sizeof(bus_state));
    bus_state.started = 1U;
    bus_state.fault_epoch = 9U;
    memset(&last_query, 0, sizeof(last_query));
    sends = 0U; cancels = 0U; now = 100U; send_result = HAL_OK;
}

static CanFrame DmReply(uint8_t rid, uint32_t when)
{
    CanFrame frame;
    CHECK(DmDiag_BuildRead(rid, &frame));
    frame.id = 0U; frame.received_ms = when; frame.fault_epoch = bus_state.fault_epoch;
    /* MasterID=0与其参数值相符；其他 RID 的数据仍原样交付。 */
    if (rid != DM_DIAG_RID_MASTER_ID) { frame.data[4] = 0x42U; frame.data[5] = 0x51U; }
    return frame;
}

static CanFrame VescFrame(uint8_t command, uint8_t dlc, uint32_t when)
{
    CanFrame frame = {0};
    frame.bus = APP_FLYWHEEL_CAN_BUS; frame.extended = 1U;
    frame.id = ((uint32_t)command << 8) | APP_DIAG_VESC_HOST_ID;
    frame.dlc = dlc; frame.received_ms = when; frame.fault_epoch = bus_state.fault_epoch;
    return frame;
}

static uint16_t TestCrc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0U, i;
    uint8_t bit;
    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
        }
    }
    return crc;
}

static void FeedVescPayload(const uint8_t *data, uint16_t length, uint32_t when)
{
    uint16_t offset = 0U, crc;
    uint8_t size, header;
    CanFrame frame;
    while (offset < length) {
        header = offset <= 255U ? 1U : 2U;
        size = (uint8_t)((uint32_t)(length - offset) > (8U - header) ? (8U - header) : (uint32_t)(length - offset));
        frame = VescFrame(header == 1U ? 5U : 6U, (uint8_t)(size + header), when);
        if (header == 1U) { frame.data[0] = (uint8_t)offset; }
        else { frame.data[0] = (uint8_t)(offset >> 8); frame.data[1] = (uint8_t)offset; }
        memcpy(frame.data + header, data + offset, size);
        AppDiagnostics_OnFrame(&frame);
        offset = (uint16_t)(offset + size);
    }
    crc = TestCrc(data, length);
    frame = VescFrame(7U, 6U, when);
    frame.data[0] = APP_FLYWHEEL_LEFT_ID; frame.data[1] = 1U;
    frame.data[2] = (uint8_t)(length >> 8); frame.data[3] = (uint8_t)length;
    frame.data[4] = (uint8_t)(crc >> 8); frame.data[5] = (uint8_t)crc;
    AppDiagnostics_OnFrame(&frame);
}

static void ExpectError(DiagError error)
{
    DiagEvent event;
    CHECK(AppDiagnostics_TakeEvent(&event) == 1U);
    CHECK(event.event == DIAG_EVENT_ERROR && event.error == error);
    CHECK(event.length == 0U && event.total == 0U);
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
}

void test_run(void)
{
    CanFrame frame;
    DiagEvent event;
    DiagPassiveFrame passive;
    uint8_t payload[333];
    uint16_t offset;
    uint32_t i, prior;

    test_stage = 1U;
    ResetFixture();
    CHECK(AppDiagnostics_Request((DiagKind)0, 2U, 7U) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 1U, 7U) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 0xFFU) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_VESC, 69U, 14U) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 5U) == 0U);
    CHECK(sends == 0U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 7U) == 1U);
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 14U) == 0U);
    CHECK(sends == 0U); /* 接收任务尚未 Poll，不得发送。 */
    AppDiagnostics_Poll(now);
    CHECK(sends == 1U && last_query.bus == 1U && last_query.id == 0x7FFU && last_query.data[2] == 0x33U);
    AppDiagnostics_Poll(now + 1U); CHECK(sends == 1U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 8U) == 0U);
    CHECK(AppDiagnostics_TakeEvent(0) == 0U && AppDiagnostics_TakeEvent(&event) == 0U);
    frame = DmReply(8U, now + 1U); AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    frame = DmReply(7U, now - 1U); AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    frame = DmReply(7U, now); frame.fault_epoch--;
    AppDiagnostics_OnFrame(&frame); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    frame = DmReply(7U, now + 1U); AppDiagnostics_OnFrame(&frame);
    CHECK(cancels == 1U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 8U) == 0U); /* 结果未读完仍忙。 */
    CHECK(AppDiagnostics_TakeEvent(&event) == 1U);
    CHECK(event.event == DIAG_EVENT_DATA && event.kind == DIAG_DM && event.node == 2U && event.field == 7U);
    CHECK(event.total == 8U && event.offset == 0U && event.length == 8U && memcmp(event.data, frame.data, 8U) == 0);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 8U) == 0U);
    CHECK(AppDiagnostics_TakeEvent(&event) == 1U && event.event == DIAG_EVENT_DONE && event.offset == 8U);
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 8U) == 1U);

    test_stage = 2U;
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 21U)); AppDiagnostics_Poll(now);
    AppDiagnostics_Poll(now + APP_DIAG_TIMEOUT_MS - 1U);
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U && sends == 1U);
    AppDiagnostics_Poll(now + APP_DIAG_TIMEOUT_MS);
    CHECK(cancels == 1U && sends == 1U); ExpectError(DIAG_ERROR_TIMEOUT);
    frame = DmReply(21U, now + APP_DIAG_TIMEOUT_MS); AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    ResetFixture(); now = 0xFFFFFFF0U;
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 21U)); AppDiagnostics_Poll(now);
    AppDiagnostics_Poll(now + APP_DIAG_TIMEOUT_MS - 1U); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    AppDiagnostics_Poll(now + APP_DIAG_TIMEOUT_MS); ExpectError(DIAG_ERROR_TIMEOUT);
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 21U)); AppDiagnostics_Poll(now);
    frame = DmReply(21U, now + APP_DIAG_TIMEOUT_MS - 1U); AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DATA);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DONE);
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 21U)); AppDiagnostics_Poll(now);
    /* Receive的2ms等待可能跨过截止时间；不必等待下一次Poll才能超时。 */
    frame = DmReply(21U, now + APP_DIAG_TIMEOUT_MS); AppDiagnostics_OnFrame(&frame);
    ExpectError(DIAG_ERROR_TIMEOUT);
    ResetFixture(); now = 0xFFFFFFF0U;
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 21U)); AppDiagnostics_Poll(now);
    frame = DmReply(21U, now + 32U); AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DATA);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DONE);

    test_stage = 3U;
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 14U));
    AppDiagnostics_Cancel(); CHECK(cancels == 1U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 7U) == 0U);
    AppDiagnostics_Poll(now); CHECK(sends == 0U && AppDiagnostics_TakeEvent(&event) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 7U)); AppDiagnostics_Poll(now); CHECK(sends == 1U);
    AppDiagnostics_Cancel(); CHECK(cancels == 2U);
    frame = DmReply(7U, now + 1U); AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 8U) == 0U);
    AppDiagnostics_Poll(now + 2U); CHECK(sends == 1U);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 8U)); AppDiagnostics_Poll(now + 2U);
    CHECK(sends == 2U); frame = DmReply(8U, now + 2U); AppDiagnostics_OnFrame(&frame);
    AppDiagnostics_Cancel(); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    AppDiagnostics_Poll(now + 3U); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);

    test_stage = 4U;
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 14U)); AppDiagnostics_Poll(now);
    CHECK(last_query.bus == 2U && last_query.extended == 1U && last_query.id == 0x846U);
    CHECK(last_query.data[0] == APP_DIAG_VESC_HOST_ID && last_query.data[1] == 0U && last_query.data[2] == 14U);
    payload[0] = 14U;
    for (i = 1U; i < sizeof(payload); ++i) { payload[i] = (uint8_t)(i * 17U); }
    FeedVescPayload(payload, sizeof(payload), now + 1U);
    CHECK(sends == 1U && cancels == 1U);
    offset = 0U;
    while (offset < sizeof(payload)) {
        CHECK(AppDiagnostics_Request(DIAG_VESC, 71U, 17U) == 0U);
        CHECK(AppDiagnostics_TakeEvent(&event));
        CHECK(event.event == DIAG_EVENT_DATA && event.kind == DIAG_VESC && event.node == 70U && event.field == 14U);
        CHECK(event.total == sizeof(payload) && event.offset == offset);
        CHECK(event.length == (sizeof(payload) - offset > 32U ? 32U : sizeof(payload) - offset));
        CHECK(memcmp(event.data, payload + offset, event.length) == 0);
        offset = (uint16_t)(offset + event.length);
    }
    CHECK(AppDiagnostics_Request(DIAG_VESC, 71U, 17U) == 0U);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DONE && event.offset == sizeof(payload));
    CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    CHECK(AppDiagnostics_Request(DIAG_VESC, 71U, 17U));

    test_stage = 5U;
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 0U)); AppDiagnostics_Poll(now);
    frame = VescFrame(8U, 5U, now + 1U);
    frame.data[0] = 71U; frame.data[1] = 1U; frame.data[2] = 0U; frame.data[3] = 6U; frame.data[4] = 5U;
    AppDiagnostics_OnFrame(&frame); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    frame.data[0] = 70U; frame.received_ms = now - 1U;
    AppDiagnostics_OnFrame(&frame); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    frame.received_ms = now + 1U; frame.fault_epoch--;
    AppDiagnostics_OnFrame(&frame); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    frame.fault_epoch++; AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.length == 3U && event.data[0] == 0U && event.data[1] == 6U);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DONE);

    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 14U)); AppDiagnostics_Poll(now);
    frame = VescFrame(5U, 2U, now + 1U); frame.data[0] = 0U; frame.data[1] = 14U;
    AppDiagnostics_OnFrame(&frame);
    frame = VescFrame(7U, 6U, now + 1U); frame.data[0] = 70U; frame.data[1] = 1U;
    frame.data[3] = 1U; /* 错误CRC不能交付参数。 */
    AppDiagnostics_OnFrame(&frame); ExpectError(DIAG_ERROR_BAD_REPLY);

    test_stage = 6U;
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 7U)); AppDiagnostics_Poll(now);
    bus_state.fault_epoch++; AppDiagnostics_Poll(now + 1U);
    CHECK(sends == 1U && cancels == 1U); ExpectError(DIAG_ERROR_CAN_FAULT);
    ResetFixture(); send_result = HAL_ERROR;
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, 7U)); AppDiagnostics_Poll(now);
    CHECK(sends == 1U && cancels == 1U);
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 0U) == 0U); ExpectError(DIAG_ERROR_TX_FAILED);
    send_result = HAL_OK;
    CHECK(AppDiagnostics_Request(DIAG_VESC, 70U, 0U)); AppDiagnostics_Poll(now + 1U);
    CHECK(sends == 2U);

    test_stage = 7U;
    ResetFixture();
    CHECK(AppDiagnostics_GetPassive(APP_DIAG_PASSIVE_SLOTS, &passive) == 0U);
    CHECK(AppDiagnostics_GetPassive(0U, 0) == 0U);
    frame = DmReply(7U, now); prior = 0U;
    for (i = 0U; i < APP_DIAG_PASSIVE_SLOTS; ++i) {
        if (AppDiagnostics_GetPassive(i, &passive) && passive.frame.bus == frame.bus &&
            passive.frame.extended == frame.extended && passive.frame.id == frame.id) { prior = passive.count; break; }
    }
    AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_GetPassive(i, &passive) && passive.count == prior + 1U);
    CHECK(passive.frame.received_ms == frame.received_ms && memcmp(passive.frame.data, frame.data, 8U) == 0);
    frame.dlc = 9U; AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_GetPassive(i, &passive) && passive.count == prior + 1U);
    CHECK(sends == 0U && AppDiagnostics_TakeEvent(&event) == 0U);

    test_stage = 8U;
    ResetFixture();
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, DM_DIAG_RID_MASTER_ID)); AppDiagnostics_Poll(now);
    frame = DmReply(DM_DIAG_RID_MASTER_ID, now); frame.data[4] = 0x42U;
    AppDiagnostics_OnFrame(&frame); ExpectError(DIAG_ERROR_BAD_REPLY);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, DM_DIAG_RID_MASTER_ID)); AppDiagnostics_Poll(now + 1U);
    frame = DmReply(DM_DIAG_RID_MASTER_ID, now + 1U); frame.id = 0x142U;
    frame.data[4] = 0x42U; frame.data[5] = 1U;
    AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DATA);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DONE);
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, DM_DIAG_RID_SLAVE_ID)); AppDiagnostics_Poll(now + 2U);
    frame = DmReply(DM_DIAG_RID_SLAVE_ID, now + 2U); frame.id = 0x143U;
    AppDiagnostics_OnFrame(&frame); CHECK(AppDiagnostics_TakeEvent(&event) == 0U);
    frame.id = 0x142U; AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DATA);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DONE);
    AppDiagnostics_Cancel(); AppDiagnostics_Poll(now + 3U);
    /* 新会话取消验证缓存，不能误用上次确认的MasterID。 */
    CHECK(AppDiagnostics_Request(DIAG_DM, 2U, DM_DIAG_RID_SLAVE_ID)); AppDiagnostics_Poll(now + 3U);
    frame = DmReply(DM_DIAG_RID_SLAVE_ID, now + 3U); frame.id = 0x143U;
    AppDiagnostics_OnFrame(&frame);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DATA);
    CHECK(AppDiagnostics_TakeEvent(&event) && event.event == DIAG_EVENT_DONE);
    test_done = 1U;
}
