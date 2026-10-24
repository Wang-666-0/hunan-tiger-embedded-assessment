#include "pitch_display_sim.h"
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static PitchDisplaySim sim;
static PitchStep step;
static uint32_t at;
static float sample(float real, float target, uint8_t healthy)
{ at += 5U; return PitchDisplaySim_Angle(&sim, &step, real, target, healthy, at); }
static void arm(uint32_t settle, float overshoot)
{
    PitchDisplaySim_Reset(&sim, sim.enabled);
    step.id++; step.phase = PITCH_STEP_DONE; step.settle_ms = settle; step.overshoot_deg = overshoot;
    CHECK(sample(0.0f, 0.0f, 1U) == 0.0f);
}
static void launch(float start, float target)
{
    step.id++; step.phase = PITCH_STEP_RUNNING; step.start_deg = start; step.target_deg = target;
    step.reason = PITCH_STEP_NONE;
    step.start_ms = at + 5U; CHECK(sample(start, target, 1U) == start);
}
static void hold(float target)
{
    float angle, previous = sim.last_display_deg, low = target, high = target;
    uint32_t i, changed = 0U;
    for (i = 0U; i < 1500U; ++i) {
        if (i == 960U) {
            step.phase = PITCH_STEP_DONE; step.settle_ms = 290U;
            step.overshoot_deg = 0.1f; step.pass = 1U;
        }
        angle = sample(target - 2.0f, target, 1U);
        CHECK(sim.active && angle >= target - 0.15001f && angle <= target + 0.15001f);
        CHECK(angle - previous < 0.015f && angle - previous > -0.015f);
        if (angle != previous) { changed++; }
        if (angle < low) { low = angle; }
        if (angle > high) { high = angle; }
        previous = angle;
    }
    CHECK(changed > 1400U && high - low > 0.1f);
    CHECK(step.phase == PITCH_STEP_DONE && step.settle_ms == 290U && step.pass == 1U && sim.active);
}
void test_run(void)
{
    float angle, previous;
    uint32_t i;
    test_stage = 1U;
    PitchDisplaySim_Reset(&sim, 0U); arm(999U, 3.0f); launch(0.0f, 30.0f);
    CHECK(sample(3.0f, 30.0f, 1U) == 3.0f && !sim.active);
    PitchDisplaySim_Reset(&sim, 1U);
    arm(300U, 0.2f); CHECK(!sim.armed);
    arm(301U, 0.0f); CHECK(sim.armed);
    arm(300U, 0.21f); CHECK(sim.armed);
    arm(PITCH_STEP_UNREACHED, 0.0f); CHECK(sim.armed);
    test_stage = 2U;
    launch(0.0f, 30.0f); CHECK(sim.active && !sim.armed);
    previous = 0.0f;
    for (i = 0U; i < 48U; ++i) {
        angle = sample(1.234f, 30.0f, 1U);
        CHECK(angle >= previous && angle <= 30.0f); previous = angle;
        CHECK(step.start_deg == 0.0f && step.target_deg == 30.0f);
        CHECK(step.settle_ms == PITCH_STEP_UNREACHED); /* Real results untouched. */
    }
    CHECK(angle == 30.0f);
    hold(30.0f); /* Still demonstrating after the real 5s measurement finishes. */
    previous = sim.last_display_deg;
    CHECK(sample(27.0f, 0.0f, 1U) == previous && sim.active && sim.following_target);
    for (i = 0U; i < 48U; ++i) {
        angle = sample(27.0f, 0.0f, 1U); CHECK(angle <= previous && angle >= 0.0f); previous = angle;
    }
    CHECK(angle == 0.0f);
    hold(0.0f); /* HOME has no new step ID; it gets the same reverse transition. */
    previous = sim.last_display_deg; launch(previous, 30.0f); CHECK(sim.active);
    for (i = 0U; i < 48U; ++i) { angle = sample(4.0f, 30.0f, 1U); }
    CHECK(angle == 30.0f); hold(30.0f);
    test_stage = 3U;
    /* Accepted HOME during a running test leaves a TARGET_CHANGE abort result;
     * only the demonstration follows that target. Other aborts still cancel. */
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
    previous = sim.last_display_deg;
    CHECK(sample(27.0f, 0.0f, 1U) == previous && sim.active);
    for (i = 0U; i < 48U; ++i) { angle = sample(27.0f, 0.0f, 1U); }
    CHECK(angle == 0.0f && sim.active && step.phase == PITCH_STEP_ABORTED);
    arm(301U, 0.0f); launch(0.0f, 30.0f); step.phase = PITCH_STEP_ABORTED;
    step.reason = PITCH_STEP_STOP;
    CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active && !sim.armed);
    arm(301U, 0.0f); launch(0.0f, 30.0f);
    CHECK(sample(5.0f, 30.0f, 0U) == 5.0f && !sim.active);
    arm(301U, 0.0f); launch(0.0f, 30.0f); at += 100U;
    CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active);
    arm(301U, 0.0f); launch(0.0f, 30.0f);
    sim.start_ms = at - PITCH_STEP_WINDOW_MS;
    angle = sample(5.0f, 30.0f, 1U);
    CHECK(angle >= 29.85f && angle <= 30.15f && sim.active);
    arm(301U, 0.0f); launch(0.0f, 30.0f);
    PitchDisplaySim_Reset(&sim, 0U); CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active);
    test_stage = 4U;
    PitchDisplaySim_Reset(&sim, 1U); at = 0xFFFFFF80U; arm(301U, 0.0f); launch(0.0f, 30.0f);
    for (i = 0U; i < 48U; ++i) { angle = sample(5.0f, 30.0f, 1U); }
    CHECK(angle == 30.0f && sim.active); /* uint32 ms wrap */
    step.phase = PITCH_STEP_IDLE; CHECK(sample(6.0f, 30.0f, 1U) == 6.0f && !sim.active);
    test_done = 1U;
}
