/* Production defaults + actual remote/target/backend/controller path; no hardware. */
#define test_run baseline_backend_test_run
#include "test_pitch_backend.c"
#undef test_run
#include "app_logic.h"

static AppLogic logic;
static SafetyInput input;
static uint8_t near(float a, float b) { return (uint8_t)(fabsf(a - b) < 0.001f); }
static void Defaults(void)
{
    Read();
    CHECK(state.controller.cascade.position.kp == 10.678f);
    CHECK(state.controller.cascade.position.ki == 24.153f);
    CHECK(state.controller.cascade.position.kd == 0.002f);
    CHECK(state.controller.cascade.speed.kp == 0.00406f);
    CHECK(state.controller.cascade.speed.ki == 0.0005f);
    CHECK(state.controller.cascade.speed.kd == 0.000001f);
    CHECK(state.controller.gravity_kg_nm == -0.13f);
}
static void Tick(uint32_t ms, uint16_t ch5, uint16_t ch2, uint8_t fresh)
{
    now_ms = ms; Read();
    if (fresh) {
        input.remote.has_frame = 1U; input.remote.received_ms = ms; ++input.remote.sequence;
        input.remote.channels[4] = ch5; input.remote.channels[1] = ch2;
    }
    input.now_ms = ms;
    input.backend_ready = state.requested;
    input.feedback_ready = (uint8_t)(state.online && !state.stop_pending && !state.controller.faults);
    input.actuator_fault = (uint8_t)(state.requested && state.controller.faults != 0U);
    AppLogic_Step(&logic, &input, 0U);
    request = logic.targets.request;
    PitchBackend_Apply(&request, logic.safety.output_allowed, 0U); Read();
}
static void Follow(uint32_t ms)
{
    CanFrame frame = Feedback(ms, 1U, state.controller.target_deg, 0.0f);
    now_ms = ms; CHECK(PitchBackend_OnFrame(&frame)); Read();
}
static void Start(uint32_t ms)
{
    uint32_t i;
    Reset(ms); Defaults(); AppLogic_Init(&logic); memset(&input, 0, sizeof(input));
    for (i = 0U; i < 14U; ++i) { input.remote.channels[i] = 1500U; }
    input.remote.channels[4] = input.remote.channels[5] = 1000U;
    CHECK(!state.active && !state.requested && !state.controller.gravity_calibrated);
    CHECK(PitchBackend_QueryFeedback()); Feed(ms, 0U); Feed(ms + 20U, 0U);
    CHECK(PitchBackend_SetCoordinate(1U)); CHECK(PitchBackend_CaptureReference());
    CHECK(PitchBackend_Request(1U));
    Apply(ms + 22U, 1U); /* An unlock before calibration must never send FC. */
    CHECK(!enable_calls && !torque_calls && !state.active && !state.enabling && !permission);
    CHECK(!state.controller.faults && state.requested && state.controller.gravity_kg_nm == -0.13f);
    CHECK(PitchBackend_SetGravityOffset(0.0f, 1U)); Read();
    CHECK(state.controller.gravity_calibrated && !enable_calls && !torque_calls);
    Tick(ms + 24U, 1000U, 1500U, 1U);
    Tick(ms + 26U, 1500U, 1500U, 1U);
    CHECK(state.enabling && enable_calls == 1U && logic.safety.operator_armed);
    Feed(ms + 28U, 1U); Tick(ms + 28U, 1500U, 1500U, 1U);
    Tick(ms + 30U, 1500U, 1500U, 1U);
    CHECK(state.active && !state.controller.faults && near(state.controller.gravity_torque_nm, -0.13f));
}
static void Manual(uint32_t *ms, uint32_t duration, uint16_t ch2)
{
    uint32_t elapsed;
    for (elapsed = 2U; elapsed <= duration; elapsed += 2U) {
        *ms += 2U;
        if (*ms % 10U == 0U) { Follow(*ms); }
        Tick(*ms, 1500U, ch2, 1U);
        CHECK(state.active && !state.controller.faults && permission);
        CHECK(state.controller.target_deg >= -18.0f && state.controller.target_deg <= 33.0f);
    }
}
void test_run(void)
{
    uint32_t ms, before, elapsed;
    float target;
    test_stage = 1U;
    CHECK(APP_PITCH_ENABLED && APP_REMOTE_PROFILE_ASSESSMENT);
    CHECK(APP_PITCH_RATE_LIMIT_DPS == 90.0f && APP_YAW_RATE_LIMIT_DPS == 90.0f);
    Start(1000U); ms = 1030U; target = state.controller.target_deg;
    Manual(&ms, 100U, 1765U);
    CHECK(request.pitch_rate_dps == 45.0f && near(state.controller.target_deg - target, 4.5f));
    target = state.controller.target_deg; Manual(&ms, 100U, 2000U);
    CHECK(request.pitch_rate_dps == 90.0f && near(state.controller.target_deg - target, 9.0f));
    target = state.controller.target_deg; Manual(&ms, 100U, 1000U);
    CHECK(request.pitch_rate_dps == -90.0f && near(state.controller.target_deg - target, -9.0f));

    test_stage = 2U;
    target = state.controller.target_deg;
    Manual(&ms, 100U, 1500U); Manual(&ms, 100U, 1470U); Manual(&ms, 100U, 1530U);
    CHECK(state.controller.target_deg == target && !request.pitch_rate_dps);
    input.remote.channels[0] = 2000U; Manual(&ms, 100U, 1500U);
    CHECK(request.yaw_rate_dps == 90.0f && state.controller.target_deg == target);
    CHECK(!request.friction_requested && !request.shot_event && !request.step_event);
    input.remote.channels[0] = 1500U;

    test_stage = 3U;
    Manual(&ms, 800U, 2000U); CHECK(state.controller.target_deg == 33.0f);
    Manual(&ms, 100U, 1500U); CHECK(state.controller.target_deg == 33.0f);
    Manual(&ms, 1000U, 1000U); CHECK(state.controller.target_deg == -18.0f);
    Manual(&ms, 100U, 1500U); CHECK(state.controller.target_deg == -18.0f);
    target = state.controller.target_deg;
    ms += 2U; Tick(ms, 2000U, 2000U, 1U);
    CHECK(request.gimbal_mode == GIMBAL_TEST && !request.pitch_rate_dps && state.controller.target_deg == target);
    ms += 2U; Tick(ms, 1500U, 1500U, 1U);
    CHECK(!state.active && state.stop_reason == PITCH_STOP_MODE_EXIT && !permission);

    test_stage = 4U;
    Start(10000U); ms = 10030U; before = enable_calls;
    input.remote.channels[7] = 1000U; Tick(ms + 2U, 1500U, 1500U, 1U);
    CHECK(!state.active && !logic.safety.operator_armed && !permission);
    input.remote.channels[7] = 1500U; Feed(ms + 4U, 0U); Tick(ms + 4U, 1500U, 1500U, 1U);
    CHECK(!state.active && enable_calls == before);

    test_stage = 5U;
    Start(20000U); ms = 20030U; before = enable_calls;
    for (elapsed = 2U; elapsed <= 100U; elapsed += 2U) {
        if ((ms + elapsed) % 10U == 0U) { Follow(ms + elapsed); }
        Tick(ms + elapsed, 1500U, 1500U, 0U);
    }
    CHECK(!state.active && !logic.safety.operator_armed && !permission && enable_calls == before);
    Feed(ms + 102U, 0U); Tick(ms + 102U, 1500U, 1500U, 1U);
    CHECK(!state.active && enable_calls == before);

    test_stage = 6U;
    Start(30000U);
    CHECK(PitchBackend_SetLoopParameter('P', 'P', 10.0f));
    CHECK(PitchBackend_SetGravityGain(-0.21f));
    Reset(40000U); Defaults();
    CHECK(!state.zero_set && !state.coord_confirmed && !state.controller.gravity_calibrated);
    CHECK(!enable_calls && !torque_calls && !query_calls);
    test_done = 1U;
}
