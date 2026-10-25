#define YAW_CASCADE_CONSOLE_TEST 1
#include "test_yaw_console_backend.c"
#include <math.h>
#include <stdio.h>

/* Synthetic feedback verifies command semantics/guards, not motor dynamics. */
static void Drive(uint32_t start, uint32_t duration, float from, float to)
{
    uint32_t dt;
    for (dt = 2U; dt <= duration; dt += 2U) {
        if (dt % 10U == 0U) {
            float f = dt < 200U ? (float)dt / 200.0f : 1.0f;
            FeedDegrees(start + dt, from + (to - from) * f, dt % 20U ? 13.99f : -13.99f);
        }
        Control(start + dt, 2U);
    }
}
void test_run(void)
{
    uint32_t move, at, id;
    float from, target = 0.0f, delta;
    YawStep snap, prior;
    YawRemoteStepEvent event;
    char expected[96], default_pid[128];
    test_stage = 1U;
    ConsoleZeroActivate(2000U); Control(2068U, 2U); at = 2068U;
    CHECK(!APP_YAW_EXPERIMENTAL_TRACKING);
    (void)snprintf(default_pid, sizeof(default_pid), "POS|%.4f,%.4f,%.4f|VEL|%.6f,%.6f,%.6f",
        (double)APP_YAW_POSITION_KP, (double)APP_YAW_POSITION_KI, (double)APP_YAW_POSITION_KD,
        (double)APP_YAW_SPEED_KP, (double)APP_YAW_SPEED_KI, (double)APP_YAW_SPEED_KD);
    Command(at, "YAW PID", default_pid);
    /* Five positive steps, ten negative, then five positive: no count cap,
     * no HOME/READY, no five-second pause. Exercise serial and RC paths. */
    test_stage = 2U;
    for (move = 0U; move < 20U; ++move) {
        from = target; delta = move < 5U || move >= 15U ? 30.0f : -30.0f; target += delta;
        if (move & 1U) {
            receiver.channels[RC_STEP_YAW_CHANNEL] = delta > 0 ? 2000U : 1000U;
            receiver.channels[RC_STEP_CHANNEL] = 1500U; Control(at + 2U, 2U);
            receiver.channels[RC_STEP_CHANNEL] = 2000U; Control(at + 4U, 2U);
            CHECK(YawBackend_TakeRemoteStepEvent(&event) && event.result == 1U && !event.blocks);
            CHECK(event.target_deg == target);
            at += 4U;
            receiver.channels[RC_STEP_CHANNEL] = receiver.channels[RC_STEP_YAW_CHANNEL] = 1500U;
        } else {
            (void)snprintf(expected, sizeof(expected), "STEP_STARTED|TARGET|%.1f|DELTA|%.1f|MODE|RELATIVE", (double)target, (double)delta);
            Command(at, delta > 0 ? "YAW STEP 30" : "YAW STEP -30", expected);
        }
        YawBackend_GetStep(&snap); id = snap.id;
        CHECK(id == move + 1U && snap.phase == YAW_STEP_RUNNING && snap.target_deg == target);
        CHECK(fabsf(snap.start_deg - from) < 0.02f && fabsf(snap.span_deg - 30.0f) < 0.02f);
        CHECK(!snap.shape_ms && !snap.ff_enabled && state.controller.target_deg == target);
        CHECK(!YawBackend_StartStep(delta)); /* Never stack steps while still moving. */
        if (move) {
            CHECK(YawBackend_TakeStepReport(&prior));
            CHECK(prior.id == id - 1U && prior.phase == YAW_STEP_ABORTED);
            CHECK(prior.reason == YAW_STEP_TARGET_CHANGE && !prior.pass && !prior.sse_valid);
        }
        Drive(at, 400U, from, target); at += 400U;
        YawBackend_GetStep(&snap);
        CHECK(state.active && !state.controller.faults && permission && snap.id == id);
        CHECK(!state.step_blocks && snap.phase == YAW_STEP_RUNNING && snap.elapsed_ms < 5000U);
        if (move == 4U || move == 14U) {
            CHECK(YawBackend_StepBlocks(delta) & YAW_STEP_BLOCK_TARGET_LIMIT);
            Command(at, delta > 0 ? "YAW STEP" : "YAW STEP -30", "WHY|TARGET_LIMIT");
            CHECK(state.controller.target_deg == target && YawBackend_GetStepId() == id);
            CHECK(!YawBackend_StepBlocks(-delta));
        }
    }
    test_stage = 3U;
    Command(at, "YAW PID", default_pid);
    CHECK(target == 0.0f && YawBackend_GetStepId() == 20U);
    YawBackend_SetViewLock(1U); CHECK(!YawBackend_StartStep(30.0f));
    CHECK(YawBackend_StepBlocks(30.0f) & YAW_STEP_BLOCK_VIEW);
    YawBackend_SetViewLock(0U);
    FeedDegrees(at + 10U, 0.0f, 70.0f); Control(at + 10U, 2U);
    CHECK(YawBackend_StepBlocks(30.0f) & YAW_STEP_BLOCK_MOVING);
    CHECK(!YawBackend_StartStep(30.0f));
    Control(at + 12U, 0U); CHECK(!state.active && !permission && !YawBackend_StartStep(30.0f));
    /* Original actual feedback hard limit still stops output on either side. */
    test_stage = 4U;
    for (move = 0U; move < 2U; ++move) {
        ConsoleZeroActivate(30000U + move * 2000U); at = 30066U + move * 2000U;
        Control(at + 2U, 2U);
        Drive(at + 2U, 400U, 0.0f, move ? -161.0f : 161.0f);
        CHECK(!state.active && !permission && (state.controller.faults & YAW_CONTROLLER_FAULT_LIMIT));
    }
    test_done = 1U;
}
