#include "yaw_backend.h"
#include "can_bus.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static CanBusState bus;
static YawBackendState state;
static TargetRequest request;
static uint32_t now_ms, enable_calls, torque_calls, disable_calls, cancel_calls;
static uint8_t permission, deny_permission, pending, keep_pending;
static HAL_StatusTypeDef enable_result, torque_result, disable_result;
static float last_torque;

uint32_t HAL_GetTick(void) { return now_ms; }
void CanBus_GetState(CanBusState *out) { if (out) { *out = bus; } }
void CanBus_SetYawPermission(uint8_t allowed)
{ permission = (uint8_t)(allowed && !deny_permission && bus.started && !bus.fault_epoch); }
uint8_t CanBus_YawAllowed(void) { return permission; }
HAL_StatusTypeDef CanBus_YawEnable(void)
{ ++enable_calls; CHECK(permission && bus.started && !bus.fault_epoch); return enable_result; }
HAL_StatusTypeDef CanBus_YawTorque(float torque_nm)
{
    ++torque_calls; last_torque = torque_nm;
    CHECK(permission && bus.started && !bus.fault_epoch);
    CHECK(torque_nm >= -0.03f && torque_nm <= 0.03f); return torque_result;
}
HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel_old)
{
    ++disable_calls; permission = 0U;
    if (cancel_old) { ++cancel_calls; if (!keep_pending) { pending = 0U; } }
    if (!bus.started) { return HAL_ERROR; }
    if (pending) { return HAL_BUSY; }
    return disable_result;
}

static CanFrame Feedback(uint32_t timestamp, uint8_t status, uint8_t driver_temp, uint8_t motor_temp)
{
    CanFrame frame;
    memset(&frame, 0, sizeof(frame)); frame.bus = 1U; frame.dlc = 8U;
    frame.received_ms = timestamp; frame.fault_epoch = bus.fault_epoch;
    frame.data[0] = (uint8_t)((status << 4) | 1U);
    frame.data[1] = 0x7FU; frame.data[2] = 0xFFU;
    frame.data[3] = 0x7FU; frame.data[4] = 0xF7U; frame.data[5] = 0xFFU;
    frame.data[6] = driver_temp; frame.data[7] = motor_temp;
    return frame;
}
static void Feed(uint32_t timestamp, uint8_t status)
{
    CanFrame frame = Feedback(timestamp, status, 25U, 30U);
    now_ms = timestamp; CHECK(YawBackend_OnFrame(&frame) == 1U);
}
static void Read(void) { YawBackend_GetState(&state); }
static void Apply(uint32_t timestamp, uint8_t allowed)
{ now_ms = timestamp; YawBackend_Apply(&request, allowed, 0U); Read(); }
static void Reset(uint32_t timestamp)
{
    memset(&bus, 0, sizeof(bus)); memset(&request, 0, sizeof(request));
    bus.started = 1U; now_ms = timestamp;
    enable_calls = torque_calls = disable_calls = cancel_calls = 0U;
    permission = deny_permission = pending = keep_pending = 0U;
    enable_result = torque_result = disable_result = HAL_OK; last_torque = 0.0f;
    YawBackend_Init(); Read();
}
static void Prepare(uint32_t timestamp)
{
    Reset(timestamp); CHECK(YawBackend_Request(1U) == 1U);
    Apply(timestamp, 0U);
    CHECK(disable_calls == 1U && cancel_calls == 1U && enable_calls == 0U && torque_calls == 0U);
    Feed(timestamp + 10U, 0U); Feed(timestamp + 30U, 0U);
    Apply(timestamp + 30U, 0U);
    CHECK(!state.active && !state.enabling && !state.stop_pending && !permission);
}
static void Activate(uint32_t timestamp)
{
    uint32_t t;
    Prepare(timestamp); Apply(timestamp + 32U, 1U);
    CHECK(enable_calls == 1U && state.enabling && !state.active && torque_calls == 0U);
    Feed(timestamp + 34U, 1U); Apply(timestamp + 34U, 1U);
    CHECK(state.active && !state.enabling && torque_calls == 0U);
    for (t = timestamp + 36U; t <= timestamp + 42U; t += 2U) { Apply(t, 1U); }
    CHECK(torque_calls == 1U && state.active && permission && state.controller.faults == 0U);
}

void test_run(void)
{
    CanFrame frame;
    uint32_t before, timestamp, sequence;
    test_stage = 1U;
    CHECK(APP_YAW_ENABLED == 1U);
    Reset(100U); Apply(100U, 0U); Apply(200U, 1U);
    CHECK(enable_calls == 0U && torque_calls == 0U && disable_calls == 0U && !state.requested);
    Feed(210U, 1U); Apply(212U, 1U);
    CHECK(enable_calls == 0U && torque_calls == 0U && disable_calls == 0U);
    CHECK(YawBackend_SetParameter('T', 1.0f) == 0U);
    CHECK(YawBackend_SetParameter('P', 0.5f) == 1U);
    CHECK(YawBackend_SetParameter('P', -0.1f) == 0U);
    CHECK(YawBackend_SetParameter('X', 1.0f) == 0U);
    CHECK(YawBackend_Request(1U) == 1U); Apply(214U, 1U);
    CHECK(disable_calls == 1U && enable_calls == 0U && torque_calls == 0U && state.stop_pending);
    frame = Feedback(212U, 0U, 25U, 30U); CHECK(YawBackend_OnFrame(&frame) == 1U); Read();
    CHECK(state.stop_pending); /* same-ms feedback cannot cross the STOP barrier */
    Feed(224U, 0U); Feed(244U, 0U); Apply(244U, 0U);
    CHECK(!state.stop_pending && enable_calls == 0U && torque_calls == 0U);
    now_ms = 246U; YawBackend_Apply(&request, 1U, 1U);
    CHECK(enable_calls == 0U && torque_calls == 0U);
    Apply(248U, 1U); CHECK(enable_calls == 1U && state.enabling && !state.active);
    before = torque_calls; Apply(250U, 1U);
    CHECK(torque_calls == before && state.enabling);

    test_stage = 2U;
    Activate(1000U);
    CHECK(bus.rx_frames[1] == 0U); /* no wheel feedback participates in yaw */
    CHECK(YawBackend_SetParameter('T', 2.0f) == 1U);
    CHECK(YawBackend_SetParameter('T', 6.0f) == 0U);
    Read(); CHECK(state.controller.target_deg == 2.0f);
    before = torque_calls; YawBackend_Stop(); Read();
    CHECK(!permission && !state.active && state.stop_pending && state.controller.torque_nm == 0.0f);
    CHECK(state.controller.target_deg != 2.0f && state.controller.integral == 0.0f);
    CHECK(YawBackend_SetParameter('T', 1.0f) == 0U);
    pending = keep_pending = 1U; Apply(1044U, 0U);
    CHECK(state.stop_pending && torque_calls == before && cancel_calls >= 2U);
    before = disable_calls; Apply(1046U, 0U); CHECK(disable_calls == before);
    keep_pending = 0U; Apply(1054U, 0U);
    CHECK(state.stop_pending && !pending && !state.active && torque_calls == 1U);
    Feed(1056U, 0U); Read(); CHECK(!state.stop_pending && !state.active);
    before = enable_calls; Apply(1058U, 0U); CHECK(enable_calls == before);
    CHECK(YawBackend_Request(0U) == 1U); Apply(1060U, 1U);
    CHECK(enable_calls == before && !state.requested);

    test_stage = 3U;
    Prepare(2000U); timestamp = state.feedback.received_ms; sequence = state.rx_count;
    frame = Feedback(timestamp - 1U, 1U, 25U, 30U); now_ms = timestamp + 1U;
    CHECK(YawBackend_OnFrame(&frame) == 0U); Read();
    CHECK(state.feedback.received_ms == timestamp && state.feedback.status == 0U && state.rx_count == sequence);
    frame = Feedback(timestamp, 1U, 60U, 70U);
    CHECK(YawBackend_OnFrame(&frame) == 0U); Read();
    CHECK(state.feedback.status == 0U && state.rx_count == sequence && state.controller.faults == 0U);
    now_ms = timestamp + 100U; frame = Feedback(timestamp + 1U, 0U, 25U, 30U);
    frame.received_ms = timestamp; CHECK(YawBackend_OnFrame(&frame) == 0U);
    Read(); CHECK(!state.online && state.feedback_age_ms == 100U);
    CHECK(YawBackend_OnFrame(0) == 0U);

    test_stage = 4U;
    Activate(3000U); before = torque_calls;
    timestamp = state.feedback.received_ms + YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS;
    Apply(timestamp, 1U);
    CHECK(!state.active && !permission && state.stop_pending && torque_calls == before);
    CHECK(state.controller.faults & YAW_CONTROLLER_FAULT_FEEDBACK);
    CHECK(YawBackend_TakeRearmRequest() == 1U && YawBackend_TakeRearmRequest() == 0U);
    Feed(timestamp + 2U, 0U); Feed(timestamp + 24U, 0U);
    before = enable_calls; Apply(timestamp + 24U, 1U);
    CHECK(enable_calls == before && !state.active); /* stale ON cannot re-enable */
    CHECK(YawBackend_Request(0U) == 1U); Apply(timestamp + 26U, 0U);
    Feed(timestamp + 28U, 0U); Feed(timestamp + 50U, 0U);
    CHECK(YawBackend_Request(1U) == 1U); Apply(timestamp + 50U, 0U);
    Feed(timestamp + 52U, 0U); Feed(timestamp + 74U, 0U);
    Apply(timestamp + 76U, 1U);
    CHECK(enable_calls == before + 1U && state.enabling && !state.active);

    test_stage = 5U;
    Activate(4000U); Feed(4044U, 0U); Read();
    CHECK(!state.active && !permission && state.stop_pending && state.controller.faults != 0U);
    CHECK(YawBackend_TakeRearmRequest() == 1U);
    before = enable_calls; Apply(4046U, 1U);
    Feed(4048U, 0U); Feed(4070U, 0U); Apply(4070U, 1U);
    CHECK(enable_calls == before && !state.active);
    CHECK(YawBackend_Request(0U) == 1U); Apply(4072U, 0U);
    Feed(4074U, 0U); Feed(4096U, 0U);
    CHECK(YawBackend_Request(1U) == 1U); /* intentional manual restart, not clear-error CAN */
    Apply(4096U, 0U); Feed(4098U, 0U); Feed(4120U, 0U); Apply(4122U, 1U);
    CHECK(enable_calls == before + 1U && state.enabling);

    test_stage = 6U;
    Activate(5000U); now_ms = 5044U; frame = Feedback(now_ms, 1U, 60U, 30U);
    CHECK(YawBackend_OnFrame(&frame) == 1U); Read();
    CHECK(state.controller.faults & YAW_CONTROLLER_FAULT_TEMPERATURE);
    CHECK(!permission && !state.active && state.stop_pending && YawBackend_TakeRearmRequest() == 1U);
    Activate(6000U); now_ms = 6044U; frame = Feedback(now_ms, 1U, 59U, 70U);
    CHECK(YawBackend_OnFrame(&frame) == 1U); Read();
    CHECK(state.controller.faults & YAW_CONTROLLER_FAULT_TEMPERATURE);
    CHECK(!permission && !state.active && YawBackend_TakeRearmRequest() == 1U);
    Activate(7000U); now_ms = 7044U; frame = Feedback(now_ms, 8U, 25U, 30U);
    CHECK(YawBackend_OnFrame(&frame) == 1U); Read();
    CHECK(state.controller.faults & YAW_CONTROLLER_FAULT_STATUS);
    CHECK(!permission && !state.active && YawBackend_TakeRearmRequest() == 1U);

    test_stage = 7U;
    Prepare(8000U); deny_permission = 1U; Apply(8032U, 1U);
    CHECK(enable_calls == 0U && torque_calls == 0U && state.stop_pending && YawBackend_TakeRearmRequest() == 1U);
    Prepare(9000U); enable_result = HAL_BUSY; Apply(9032U, 1U);
    CHECK(enable_calls == 1U && !state.enabling && !permission && state.stop_pending && state.controller.faults != 0U);
    CHECK(YawBackend_TakeRearmRequest() == 1U);
    Activate(10000U); torque_result = HAL_ERROR; Apply(10044U, 1U); Apply(10046U, 1U);
    Apply(10048U, 1U); Apply(10050U, 1U); Apply(10052U, 1U);
    CHECK(!permission && !state.active && state.stop_pending && state.controller.faults != 0U);
    CHECK(YawBackend_TakeRearmRequest() == 1U);
    Prepare(11000U); Apply(11032U, 1U);
    for (timestamp = 11034U; timestamp <= 11132U; timestamp += 2U) {
        Feed(timestamp, 0U); Apply(timestamp, 1U);
    }
    CHECK(!permission && !state.active && !state.enabling && state.stop_pending && state.controller.faults != 0U);
    CHECK(YawBackend_TakeRearmRequest() == 1U);
    test_done = 1U;
}
