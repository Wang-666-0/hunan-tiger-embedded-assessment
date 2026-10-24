#include "pitch_backend.h"
#include "can_bus.h"
#include "app_config.h"
#include <string.h>
#include <math.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static CanBusState bus;
static PitchBackendState state;
static TargetRequest request;
static uint32_t now_ms, enable_calls, torque_calls, disable_calls, query_calls, cancel_calls;
static uint8_t permission, deny_permission, pending;
static HAL_StatusTypeDef enable_result, torque_result, disable_result, query_result;
static float last_torque;

uint32_t HAL_GetTick(void) { return now_ms; }
void CanBus_GetState(CanBusState *out) { if (out) { *out = bus; } }
void CanBus_SetPitchPermission(uint8_t allowed)
{ permission = (uint8_t)(allowed && !deny_permission && bus.started && !bus.fault_epoch); }
uint8_t CanBus_PitchAllowed(void) { return permission; }
uint32_t CanBus_PitchTxPending(void) { return pending; }
HAL_StatusTypeDef CanBus_CancelPitchTx(void) { ++cancel_calls; pending = 0U; return HAL_OK; }
HAL_StatusTypeDef CanBus_PitchEnable(void)
{ ++enable_calls; CHECK(permission && bus.started && !bus.fault_epoch); return enable_result; }
HAL_StatusTypeDef CanBus_PitchTorque(float torque_nm)
{
    ++torque_calls; last_torque = torque_nm;
    CHECK(permission && bus.started && !bus.fault_epoch);
    CHECK(torque_nm >= -DM3507_TORQUE_MAX_NM && torque_nm <= DM3507_TORQUE_MAX_NM);
    return torque_result;
}
HAL_StatusTypeDef CanBus_PitchDisable(uint8_t cancel_old)
{
    ++disable_calls; permission = 0U;
    if (cancel_old) { ++cancel_calls; pending = 0U; }
    if (!bus.started) { return HAL_ERROR; }
    return pending ? HAL_BUSY : disable_result;
}
HAL_StatusTypeDef CanBus_PitchReadFeedback(void)
{ ++query_calls; return bus.started && !bus.fault_epoch ? query_result : HAL_ERROR; }

static uint16_t Quantize(float value, float maximum, uint32_t scale)
{
    float scaled = (value + maximum) * (float)scale / (2.0f * maximum);
    if (scaled < 0.0f) { scaled = 0.0f; }
    if (scaled > (float)scale) { scaled = (float)scale; }
    return (uint16_t)scaled;
}
static CanFrame Feedback(uint32_t timestamp, uint8_t status, float angle_deg, float speed_dps)
{
    CanFrame frame;
    uint16_t p = Quantize(angle_deg * DM3507_POSITION_MAX_RAD / 180.0f, DM3507_POSITION_MAX_RAD, 65535U);
    uint16_t v = Quantize(speed_dps * DM3507_POSITION_MAX_RAD / 180.0f, DM3507_VELOCITY_MAX_RAD_S, 4095U);
    uint16_t t = Quantize(0.0f, DM3507_TORQUE_MAX_NM, 4095U);
    memset(&frame, 0, sizeof(frame)); frame.bus = 1U; frame.id = 2U; frame.dlc = 8U;
    frame.received_ms = timestamp; frame.fault_epoch = bus.fault_epoch;
    frame.data[0] = (uint8_t)((status << 4) | 2U);
    frame.data[1] = (uint8_t)(p >> 8); frame.data[2] = (uint8_t)p;
    frame.data[3] = (uint8_t)(v >> 4); frame.data[4] = (uint8_t)((v << 4) | (t >> 8));
    frame.data[5] = (uint8_t)t; frame.data[6] = 25U; frame.data[7] = 30U;
    return frame;
}
static void Read(void) { PitchBackend_GetState(&state); }
static void Feed(uint32_t timestamp, uint8_t status)
{
    CanFrame frame = Feedback(timestamp, status, 0.0f, 0.0f);
    now_ms = timestamp; CHECK(PitchBackend_OnFrame(&frame)); Read();
}
static void Apply(uint32_t timestamp, uint8_t allowed)
{ now_ms = timestamp; PitchBackend_Apply(&request, allowed, 0U); Read(); }
static void Reset(uint32_t timestamp)
{
    PitchStep snapshot;
    memset(&bus, 0, sizeof(bus)); memset(&request, 0, sizeof(request)); bus.started = 1U;
    now_ms = timestamp; enable_calls = torque_calls = disable_calls = query_calls = cancel_calls = 0U;
    permission = deny_permission = pending = 0U;
    enable_result = torque_result = disable_result = query_result = HAL_OK;
    last_torque = 0.0f; PitchBackend_Init(); Read();
    PitchBackend_GetStep(&snapshot);
    CHECK(snapshot.id == 0U && PitchBackend_GetStepId() == snapshot.id);
}
static void Prepare(uint32_t timestamp)
{
    Reset(timestamp); CHECK(PitchBackend_QueryFeedback()); Feed(timestamp, 0U); Feed(timestamp + 20U, 0U);
    CHECK(!PitchBackend_Request(1U)); CHECK(PitchBackend_SetCoordinate(1U));
    CHECK(PitchBackend_CaptureReference());
    CHECK(PitchBackend_SetGravityOffset(0.0f, 1U));
    CHECK(PitchBackend_Request(1U)); Read();
    CHECK(state.requested && state.zero_set && state.coord_confirmed && !state.active);
    CHECK(!enable_calls && !torque_calls && !disable_calls);
}
static void Activate(uint32_t timestamp)
{
    uint32_t t;
    Prepare(timestamp); Apply(timestamp + 22U, 1U);
    CHECK(state.enabling && !state.active && enable_calls == 1U && !torque_calls);
    Feed(timestamp + 24U, 1U); Apply(timestamp + 24U, 1U);
    CHECK(state.active && !state.enabling && !torque_calls);
    for (t = timestamp + 26U; t <= timestamp + 44U; t += 2U) {
        if (t == timestamp + 34U || t == timestamp + 44U) { Feed(t, 1U); }
        Apply(t, 1U);
    }
    CHECK(state.active && !state.controller.faults && torque_calls && permission);
}

void test_run(void)
{
    CanFrame frame;
    PitchStep measurement;
    PitchRemoteStepEvent event;
    uint32_t before, tick;
    test_stage = 1U;
    Reset(100U); Apply(100U, 0U); Apply(120U, 1U);
    CHECK(!query_calls && !disable_calls && !enable_calls && !torque_calls && !state.coord_confirmed);
    CHECK(PitchBackend_QueryFeedback()); Feed(122U, 0U); Apply(134U, 0U);
    CHECK(query_calls >= 2U && !disable_calls && !enable_calls && !torque_calls && state.monitoring);
    CHECK(!PitchBackend_SetCoordinate(2U)); CHECK(!PitchBackend_Request(1U));
    frame = Feedback(136U, 0U, 0.0f, 0.0f); frame.id = 0U; now_ms = 136U;
    CHECK(!PitchBackend_OnFrame(&frame)); frame.id = 2U; frame.data[0] = 1U;
    CHECK(!PitchBackend_OnFrame(&frame));
    frame = Feedback(136U, 0U, 0.0f, 0.0f); frame.data[1] = 0U; frame.data[2] = 0x33U;
    CHECK(!PitchBackend_OnFrame(&frame));

    test_stage = 2U;
    Prepare(1000U); PitchBackend_Stop(); Apply(1022U, 0U);
    CHECK(!disable_calls && !enable_calls && !torque_calls && !state.requested);
    CHECK(PitchBackend_Request(1U)); Apply(1024U, 1U);
    CHECK(state.enabling && !torque_calls);
    Feed(1024U, 1U); Apply(1024U, 1U); CHECK(!state.active && state.enabling);
    Feed(1026U, 1U); Apply(1026U, 1U); CHECK(state.active && !state.enabling);

    test_stage = 3U;
    Activate(2000U); CHECK(!PitchBackend_StartStep(1.0f)); CHECK(!PitchBackend_StartStep(50.0f));
    CHECK(!PitchBackend_StartStep(-30.0f)); Read(); CHECK(state.controller.target_deg == 0.0f);
    PitchBackend_GetStep(&measurement);
    CHECK(measurement.id == 0U && PitchBackend_GetStepId() == measurement.id);
    CHECK(PitchBackend_StartStep(30.0f)); PitchBackend_GetStep(&measurement);
    CHECK(measurement.phase == PITCH_STEP_RUNNING && measurement.target_deg == 30.0f);
    CHECK(measurement.id == 1U && PitchBackend_GetStepId() == measurement.id);
    CHECK(measurement.kp == APP_PITCH_POSITION_KP && measurement.vkp == APP_PITCH_SPEED_KP);
    before = torque_calls; Apply(2046U, 1U); CHECK(torque_calls == before + 1U);
    CHECK(PitchBackend_SetLoopPid('P', APP_PITCH_POSITION_KP, APP_PITCH_POSITION_KI, APP_PITCH_POSITION_KD));
    PitchBackend_GetStep(&measurement); CHECK(measurement.phase == PITCH_STEP_RUNNING);
    CHECK(PitchBackend_SetLoopParameter('P', 'I', 0.3f)); PitchBackend_GetStep(&measurement); Read();
    CHECK(measurement.phase == PITCH_STEP_ABORTED && measurement.reason == PITCH_STEP_PID_CHANGE);
    CHECK(measurement.id == 1U && PitchBackend_GetStepId() == measurement.id);
    CHECK(measurement.ki == APP_PITCH_POSITION_KI && state.controller.target_deg == 30.0f);
    CHECK(!PitchBackend_SetLoopParameter('V', 'P', -0.1f));
    CHECK(PitchBackend_TakeStepReport(&measurement));
    CHECK(measurement.id == 1U && PitchBackend_GetStepId() == measurement.id);
    CHECK(!PitchBackend_TakeStepReport(&measurement));
    PitchBackend_GetStep(&measurement);
    CHECK(!measurement.pending && measurement.id == 1U && PitchBackend_GetStepId() == measurement.id);
    CHECK(PitchBackend_SetTarget(0.0f)); PitchBackend_GetStep(&measurement);
    CHECK(measurement.phase != PITCH_STEP_RUNNING && measurement.pass == 0U);

    test_stage = 4U;
    Activate(3000U); request.gimbal_mode = GIMBAL_TEST;
    request.step_event = 1U; request.step_axis = GIMBAL_STEP_YAW; request.step_delta_deg = 30.0f; request.step_sequence = 1U;
    Apply(3046U, 1U); CHECK(!PitchBackend_TakeRemoteStepEvent(&event));
    CHECK(state.controller.target_deg == 0.0f);
    request.step_axis = GIMBAL_STEP_PITCH; request.step_sequence = 2U;
    Apply(3048U, 1U); CHECK(PitchBackend_TakeRemoteStepEvent(&event) && event.result == 1U);
    request.step_return_event = 1U; request.step_sequence = 3U; Apply(3050U, 1U);
    CHECK(PitchBackend_TakeRemoteStepEvent(&event) && event.result == 2U && event.target_deg == 0.0f);
    request.step_event = 0U; request.gimbal_mode = GIMBAL_MANUAL; Apply(3052U, 1U);
    CHECK(!state.active && !state.requested && !permission && state.stop_pending);
    CHECK(state.stop_reason == PITCH_STOP_MODE_EXIT && state.stop_id == 1U && state.stopped_ms == 3052U);
    PitchBackend_Stop(); Read(); CHECK(state.stop_reason == PITCH_STOP_MODE_EXIT && state.stop_id == 1U);
    CHECK(PitchBackend_TakeRearmRequest()); Apply(3054U, 0U); CHECK(disable_calls && cancel_calls);
    frame = Feedback(3056U, 0U, -0.1f, -50.0f); now_ms = 3056U; CHECK(PitchBackend_OnFrame(&frame)); Read();
    CHECK(!state.stop_pending); before = enable_calls; Apply(3058U, 1U); CHECK(enable_calls == before);

    test_stage = 5U;
    Prepare(4000U); request.gimbal_mode = GIMBAL_TEST; request.step_axis = GIMBAL_STEP_PITCH;
    request.step_event = 1U; request.step_sequence = 1U; request.step_delta_deg = 30.0f;
    Apply(4022U, 1U); CHECK(PitchBackend_TakeRemoteStepEvent(&event) && event.result == 3U);
    Feed(4024U, 1U); Apply(4024U, 1U);
    request.step_event = 0U;
    for (tick = 4026U; tick <= 4044U; tick += 2U) { if (tick == 4044U) { Feed(tick, 1U); } Apply(tick, 1U); }
    PitchBackend_GetStep(&measurement); CHECK(measurement.phase == PITCH_STEP_IDLE && state.controller.target_deg == 0.0f);
    torque_result = HAL_ERROR; Apply(4046U, 1U);
    CHECK(state.controller.faults & PITCH_BACKEND_FAULT_TX); CHECK(!state.active && !permission);
    CHECK(state.stop_reason == PITCH_STOP_TORQUE_TX);

    test_stage = 6U;
    Activate(5000U); before = torque_calls; Apply(5144U, 1U);
    CHECK(!state.active && !state.requested && !permission && torque_calls == before);
    CHECK(state.controller.faults & PITCH_CONTROLLER_FAULT_FEEDBACK);
    CHECK(state.stop_reason == PITCH_STOP_FEEDBACK);
    CHECK(PitchBackend_TakeRearmRequest());
    Reset(6000U); CHECK(PitchBackend_Request(0U)); Apply(6000U, 0U);
    CHECK(disable_calls == 1U && !enable_calls && !torque_calls);
    Feed(6000U, 0U); CHECK(state.stop_pending); Feed(6002U, 0U); CHECK(!state.stop_pending);
    Activate(7000U); Apply(7046U, 0U);
    CHECK(state.stop_reason == PITCH_STOP_PERMISSION && !state.active && !permission);
    Apply(7048U, 0U); Read(); CHECK(state.stop_reason == PITCH_STOP_PERMISSION && state.stop_id == 1U);

    test_stage = 7U;
    Activate(8000U); request.gimbal_mode = GIMBAL_TEST;
    CHECK(PitchBackend_StartStep(30.0f)); Read();
    CHECK(state.controller.brake_tracking && !state.controller.brake_arrived);
    PitchBackend_GetStep(&measurement); CHECK(measurement.pitch_brake_enabled == 1U);
    CHECK(!PitchBackend_SetBrakeEnabled(0U) && PitchBackend_SetBrakeEnabled(1U));
    for (tick = 8046U; tick <= 8104U; tick += 2U) {
        frame = Feedback(tick, 1U, (float)(tick - 8044U) * 29.9f / 60.0f, 500.0f);
        now_ms = tick; CHECK(PitchBackend_OnFrame(&frame)); Apply(tick, 1U);
    }
    CHECK(state.controller.brake_arrived && state.controller.brake_torque_nm == 0.0f);
    /* Drive real position differentiation far enough to observe a reversed velocity. */
    for (tick = 8106U; tick <= 8144U; tick += 2U) {
        frame = Feedback(tick, 1U, 29.9f - (float)(tick - 8104U) * 0.075f, -75.0f);
        now_ms = tick; CHECK(PitchBackend_OnFrame(&frame)); Apply(tick, 1U);
    }
    CHECK(state.controller.brake_torque_nm > 0.0f && state.controller.brake_torque_nm <= 0.30f);
    CHECK(PitchBackend_SetTarget(0.0f)); Read();
    CHECK(state.controller.brake_tracking && !state.controller.brake_arrived && state.controller.brake_torque_nm == 0.0f);
    CHECK(state.controller.brake_direction == -1.0f);
    PitchBackend_Stop(); Read();
    CHECK(!state.controller.brake_tracking && state.controller.brake_torque_nm == 0.0f);
    test_done = 1U;
}
