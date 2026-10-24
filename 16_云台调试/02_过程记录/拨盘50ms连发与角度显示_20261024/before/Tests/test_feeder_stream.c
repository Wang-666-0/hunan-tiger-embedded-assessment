#define test_run feeder_legacy_backend_run
#include "test_feeder_backend.c"
#undef test_run

/* These are deterministic encoder inputs, not an identified motor plant and
 * not evidence that the real mechanism reaches 20Hz or this single-shot time. */
static uint32_t start_stream;

static void auto_prepare(uint32_t start)
{
    uint32_t i, prepared = 0U;
    Reset(start);
    for (i = 1U; i <= 105U; ++i) {
        Tick(start + i);
        if (FeederBackend_PrepareRemote()) { ++prepared; }
    }
    Tick(start + 106U);
    CHECK(prepared == 1U && state.armed && state.controller.have_zero && !state.stop_pending);
    CHECK(!nonzero_calls && !state.controller.busy && !state.controller.accepted);
    CHECK(state.controller.config.gear_ratio == 35.68f && state.controller.config.direction == -1);
}

static void launch(uint16_t mode)
{
    SetRemote(now_ms + 1U, 1500U, 1000U, 1500U);
    wheels.flywheels_ready = 1U;
    SetRemote(now_ms + 1U, 1500U, mode, 1500U);
    SetRemote(now_ms + 1U, 1500U, mode, 2000U);
    start_stream = now_ms;
    CHECK(state.armed && state.controller.busy && state.controller.accepted == 1U);
}

static void stream_sample(uint32_t delta_ms)
{
    double step_counts = 40.0 * (double)state.controller.config.gear_ratio * 8192.0 / 360.0;
    if (delta_ms > 10U) {
        rotor_counts = -(int64_t)((double)(delta_ms - 10U) * step_counts / 50.0);
        rotor_rpm = -4757;
    } else { rotor_counts = 0; rotor_rpm = 0; }
    Tick(start_stream + delta_ms);
    CHECK(!state.controller.faults && state.armed && permission);
    CHECK(state.controller.outstanding <= FEEDER_STREAM_MAX_PENDING);
    CHECK(last_current >= -3000 && last_current <= 3000);
}

static void tail_to_target(void)
{
    uint32_t i;
    int64_t goal = (int64_t)(state.controller.target_deg * 8192.0 / 360.0) - 1;
    for (i = 0U; i < 110U; ++i) {
        if (rotor_counts > goal) {
            rotor_counts -= rotor_counts - goal > 500 ? 500 : rotor_counts - goal;
            rotor_rpm = -3000;
        } else { rotor_rpm = 0; }
        Tick(now_ms + 1U);
    }
    CHECK(!state.controller.busy && !state.controller.outstanding && !state.controller.faults);
    CHECK(state.armed && state.controller.have_zero && !state.controller.aborted);
}

void test_run(void)
{
    uint32_t i, accepted;
    int64_t reference;
    FeederConfig config;

    test_stage = 1U;
    Reset(10U);
    for (i = 0U; i < 150U; ++i) {
        SetRemote(now_ms + 1U, 1500U, 2000U, 2000U);
        CHECK(!FeederBackend_PrepareRemote());
    }
    CHECK(!state.armed && !state.controller.have_zero && !nonzero_calls);
    auto_prepare(1000U); launch(2000U);
    for (i = 1U; i <= 1005U; ++i) { stream_sample(i); }
    CHECK(state.controller.accepted == 21U && state.controller.completed == 19U);
    CHECK(state.controller.continuous_completed == 19U && !state.controller.single_completed);
    CHECK(!state.controller.rejected && state.controller.outstanding == 2U);
    CHECK(state.controller.measured_hz > 19.99f && state.controller.measured_hz < 20.01f);
    CHECK(state.controller.last_motion_ms >= 60U && state.controller.last_motion_ms <= 62U);
    CHECK(app.logic.targets.request.continuous_fire);

    test_stage = 2U;
    SetRemote(now_ms + 1U, 1500U, 2000U, 1500U);
    CHECK(!app.logic.targets.request.continuous_fire && state.controller.cancelled == 1U);
    CHECK(state.controller.outstanding == 1U && state.controller.motion_kind == FEEDER_MOTION_TAIL);
    CHECK(state.controller.axis_target_deg < -799.9 && state.controller.axis_target_deg > -800.1);
    tail_to_target();
    CHECK(state.controller.completed == 20U && state.controller.accepted == 21U);
    CHECK(state.controller.continuous_completed == 20U && state.continuous_accepted == 21U);
    CHECK(FeederConsole_Handle("FEEDER RESULT", response, sizeof(response)));
    CHECK(strstr(response, "|COUNT_SOURCE|ENCODER_40DEG|") && strstr(response, "|CONT_DONE|20|"));
    CHECK(strstr(response, "|CANCEL|1|") && strstr(response, "|RATE_VALID|1|"));
    accepted = state.controller.accepted;
    for (i = 0U; i < 160U; ++i) { Tick(now_ms + 1U); }
    CHECK(state.controller.accepted == accepted && !state.controller.busy);

    test_stage = 3U;
    /* An idle manual rearm must preserve the original software reference. */
    reference = state.controller.zero_counts;
    FeederBackend_Stop(); SetRemote(now_ms + 1U, 1000U, 1000U, 1500U);
    for (i = 0U; i < 110U; ++i) { Tick(now_ms + 1U); (void)FeederBackend_PrepareRemote(); }
    Tick(now_ms + 1U);
    CHECK(state.armed && state.controller.have_zero && state.controller.zero_counts == reference);
    SetRemote(now_ms + 1U, 1500U, 1000U, 1500U);
    SetRemote(now_ms + 1U, 1500U, 1500U, 1500U);
    SetRemote(now_ms + 1U, 1500U, 1500U, 2000U);
    CHECK(state.controller.busy);
    rotor_counts -= 100; rotor_rpm = -600; Tick(now_ms + 1U);
    SetRemote(now_ms + 1U, 1500U, 1500U, 1000U);
    CHECK(!state.armed && !state.controller.have_zero && !permission);
    CHECK(state.controller.aborted == 1U);
    rotor_rpm = 0; SetRemote(now_ms + 1U, 1000U, 1000U, 1500U);
    accepted = state.controller.accepted;
    for (i = 0U; i < 150U; ++i) { Tick(now_ms + 1U); CHECK(!FeederBackend_PrepareRemote()); }
    CHECK(!state.armed && !state.controller.have_zero && state.controller.accepted == accepted);

    test_stage = 4U;
    auto_prepare(4000U); launch(2000U);
    for (i = 1U; i <= 55U; ++i) { stream_sample(i); }
    wheels.flywheels_ready = 0U; stream_sample(56U);
    CHECK(!app.logic.targets.request.continuous_fire && state.controller.cancelled == 1U);
    CHECK(state.controller.motion_kind == FEEDER_MOTION_TAIL && !state.controller.aborted);
    tail_to_target(); accepted = state.controller.accepted;
    wheels.flywheels_ready = 1U;
    for (i = 0U; i < 100U; ++i) { Tick(now_ms + 1U); }
    CHECK(state.controller.accepted == accepted && !state.controller.busy);
    CHECK(!app.logic.targets.request.continuous_fire); /* Held trigger cannot replay. */

    test_stage = 5U;
    auto_prepare(5000U); launch(2000U);
    for (i = 1U; i <= 20U; ++i) { stream_sample(i); }
    accepted = nonzero_calls; wheels.feedback_ready = 0U; Tick(now_ms + 1U);
    CHECK(!state.armed && !state.controller.busy && !state.controller.have_zero && !permission);
    CHECK(nonzero_calls == accepted && !app.logic.safety.operator_armed);

    test_stage = 6U;
    auto_prepare(6000U);
    FeederBackend_Stop(); Tick(now_ms + 1U);
    config = state.controller.config; config.continuous_max_rpm = 4700.0f;
    CHECK(FeederBackend_SetConfig(&config)); CHECK(FeederBackend_Arm()); Tick(now_ms + 1U);
    SetRemote(now_ms + 1U, 1500U, 1000U, 1500U);
    wheels.flywheels_ready = 1U; SetRemote(now_ms + 1U, 1500U, 2000U, 1500U);
    SetRemote(now_ms + 1U, 1500U, 2000U, 2000U);
    CHECK(!state.controller.accepted && state.controller.rejected == 1U && state.controller.rate_blocked);
    CHECK(!state.controller.busy && !nonzero_calls);

    test_stage = 7U;
    auto_prepare(0xFFFFFFC0U); /* 100ms quiet preparation crosses tick wrap. */
    for (i = 0U; i < 100U; ++i) { Tick(now_ms + 1U); CHECK(!FeederBackend_PrepareRemote()); }
    CHECK(state.armed && state.controller.have_feedback);
    rotor_counts = 1000; Feed(now_ms + 1U); LogicApply(now_ms, 1U);
    CHECK(state.controller.faults == FEEDER_FAULT_JUMP && !state.armed && !state.controller.have_zero);
    for (i = 0U; i < 150U; ++i) { Tick(now_ms + 1U); CHECK(!FeederBackend_PrepareRemote()); }
    CHECK(state.controller.faults == FEEDER_FAULT_JUMP && !state.armed);
    Console("FEEDER READY", 1U); Tick(now_ms + 1U);
    CHECK(state.armed && state.controller.have_zero && !state.controller.faults);
    CHECK(!state.controller.accepted && !state.controller.busy && !nonzero_calls);
    Console("FEEDER OFF", 1U); Tick(now_ms + 1U);
    for (i = 0U; i < 120U; ++i) { Tick(now_ms + 1U); CHECK(!FeederBackend_PrepareRemote()); }
    CHECK(!state.armed && state.controller.have_zero);
    Console("FEEDER ARM", 1U); Tick(now_ms + 1U);
    CHECK(state.armed);
    test_done = 1U;
}
