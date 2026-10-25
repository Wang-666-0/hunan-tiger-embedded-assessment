/* Extend the real assessment receiver/logic/backend harness. Only transport is
 * mocked; MotorBackend_GetHealth and both original speed loops are linked. */
#define test_run assessment_wheels_baseline_run
#include "test_assessment_wheels.c"
#undef test_run

static void ready_read(void)
{
    MotorBackend_GetJogState(&state);
    MotorBackend_GetHealth(&health);
    CHECK(state.requested_left_mA >= 0 && state.requested_left_mA <= 2000);
    CHECK(state.requested_right_mA >= 0 && state.requested_right_mA <= 2000);
}

static void ready_cycle(uint32_t timestamp, int32_t left, int32_t right)
{
    TargetRequest request;
    now_ms = timestamp; input.now_ms = timestamp;
    feedback(left, right);
    input.remote.sequence++; input.remote.received_ms = timestamp;
    if (MotorBackend_PrepareRemote(&input.remote, logic.safety.barrier_ms,
                                   logic.safety.barrier_valid)) { stop(); }
    MotorBackend_GetHealth(&health);
    input.backend_ready = health.ready;
    input.feedback_ready = health.feedback_ready;
    input.actuator_fault = health.actuator_fault;
    input.can_epoch = bus.fault_epoch; input.stop_sequence = stop_sequence;
    AppLogic_Step(&logic, &input, health.flywheels_ready);
    request = logic.targets.request;
    request.wheel_reset_allowed = logic.command.valid && logic.command.failsafe_ok &&
        logic.command.wheel_switch == RC_LOW;
    CanBus_SetOutputPermission(logic.safety.output_allowed && request.friction_requested);
    (void)MotorBackend_Apply(&request, logic.safety.output_allowed);
    if (MotorBackend_TakeRearmRequest()) {
        stop(); logic.safety.output_allowed = logic.safety.operator_armed = 0U;
    }
    ready_read();
}

static void start_ready(void)
{
    uint32_t time, first_band = 0U;
    uint8_t band_seen = 0U, ready_seen = 0U;
    start();
    CHECK(!health.flywheels_ready);
    for (time = 222U; time <= 1902U; time += 2U) {
        ready_cycle(time, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
        if (!band_seen && state.speed.band_ms && state.speed_right.band_ms) {
            band_seen = 1U; first_band = time;
            CHECK(!health.flywheels_ready); /* The first in-band sample is insufficient. */
        }
        if (state.speed.band_ms < APP_FLYWHEEL_SPEED_BAND_HOLD_MS ||
            state.speed_right.band_ms < APP_FLYWHEEL_SPEED_BAND_HOLD_MS) {
            CHECK(!health.flywheels_ready);
        }
        if (health.flywheels_ready) {
            ready_seen = 1U;
            CHECK(band_seen);
            CHECK(time - first_band >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS -
                APP_FLYWHEEL_SPEED_PERIOD_MS);
        }
    }
    CHECK(band_seen && ready_seen && health.flywheels_ready);
    CHECK(state.active && state.selection == FLYWHEEL_JOG_BOTH);
    CHECK(state.speed.in_band && state.speed_right.in_band);
    CHECK(state.speed.band_ms >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS);
    CHECK(state.speed_right.band_ms >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS);
    CHECK(!state.speed.coasting && !state.speed_right.coasting);
}

static void continuous_band_and_individual_exit(void)
{
    uint32_t out_at, time;
    test_stage = 10U; start_ready();
    /* This dip is shorter than the 20 ms PI period. Immediate revocation and
     * another complete hold are both required; stale in_band cannot reauthorize. */
    out_at = now_ms + 2U;
    ready_cycle(out_at, APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM - 1,
                APP_FLYWHEEL_SPEED_TARGET_ERPM);
    CHECK(!health.flywheels_ready);
    for (time = out_at + 2U; time < out_at + APP_FLYWHEEL_SPEED_BAND_HOLD_MS; time += 2U) {
        ready_cycle(time, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
        CHECK(!health.flywheels_ready);
    }
    for (; time <= out_at + APP_FLYWHEEL_SPEED_BAND_HOLD_MS + 40U; time += 2U) {
        ready_cycle(time, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
    }
    CHECK(health.flywheels_ready);
    ready_cycle(now_ms + 2U, APP_FLYWHEEL_SPEED_TARGET_ERPM,
                APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM + 1);
    CHECK(!health.flywheels_ready); /* Either wheel independently removes permission. */
}

static void timeout_off_and_locks(void)
{
    uint32_t old_nonzero;
    test_stage = 11U; start_ready();
    now_ms += FLYWHEEL_JOG_STATUS1_TIMEOUT_MS + 1U;
    ready_read();
    CHECK(!health.feedback_ready && !health.flywheels_ready);

    test_stage = 12U; start_ready();
    input.remote.channels[5] = 1000U; /* CH6 off preserves operator arming. */
    ready_cycle(now_ms + 2U, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
    CHECK(!state.active && !health.flywheels_ready && !permission);
    CHECK(logic.safety.operator_armed);

    test_stage = 13U; start_ready(); old_nonzero = nonzeros;
    input.remote.channels[7] = 1000U; /* CH8 lock revokes all operator authority. */
    ready_cycle(now_ms + 2U, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
    CHECK(!state.active && !health.flywheels_ready && !permission);
    CHECK(!logic.safety.operator_armed && nonzeros == old_nonzero);
    input.remote.channels[7] = 1500U;
    ready_cycle(now_ms + 2U, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
    CHECK(!state.active && !health.flywheels_ready && !logic.safety.operator_armed);

    test_stage = 14U; start_ready();
    ready_cycle(now_ms + 2U, APP_FLYWHEEL_JOG_HARD_ERPM + 1,
                APP_FLYWHEEL_SPEED_TARGET_ERPM);
    CHECK(!state.active && !health.flywheels_ready && !permission);
    CHECK(state.health.faults & FLYWHEEL_JOG_FAULT_OVERSPEED);
    CHECK(!logic.safety.operator_armed);
    ready_cycle(now_ms + 2U, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
    CHECK(!health.flywheels_ready && !state.active);
    CHECK(state.health.faults & FLYWHEEL_JOG_FAULT_OVERSPEED);

    test_stage = 15U; start_ready(); bus.fault_epoch = 1U;
    ready_cycle(now_ms + 2U, APP_FLYWHEEL_SPEED_TARGET_ERPM, APP_FLYWHEEL_SPEED_TARGET_ERPM);
    CHECK(!state.active && !health.flywheels_ready && !permission);
    CHECK(!logic.safety.operator_armed);
}

void test_run(void)
{
    assessment_wheels_baseline_run();
    test_done = 0U;
    continuous_band_and_individual_exit();
    timeout_off_and_locks();
    test_done = 1U;
}
