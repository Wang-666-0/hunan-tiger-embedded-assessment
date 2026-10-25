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
static void reach(float target)
{
    float angle, previous = sim.last_display_deg;
    uint32_t i, ticks = pitch_display_profiles[sim.variant].duration_ms / 5U;
    uint8_t upward = (uint8_t)(target >= previous);
    for (i = 0U; i < ticks; ++i) {
        angle = sample(target - 2.0f, target, 1U);
        CHECK(upward ? (angle >= previous && angle <= target) : (angle <= previous && angle >= target));
        previous = angle;
    }
    CHECK(sim.last_display_deg == target && sim.active && !sim.counted);
}
static void hold_first(float target, uint32_t ticks)
{
    float angle, previous = sim.last_display_deg, low = target, high = target;
    uint32_t i, changed = 0U;
    for (i = 0U; i < ticks; ++i) {
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
    CHECK(changed > ticks * 9U / 10U && high - low > 0.07f);
    CHECK(sim.enabled && sim.completed == 1U && sim.counted && !sim.recovering);
}
void test_run(void)
{
    float angle, previous, demo;
    uint32_t i, trial, mask = 0U, recover_at;
    uint8_t first;
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
    first = sim.variant; reach(30.0f);
    hold_first(30.0f, 1500U); /* A single demo cannot be counted twice, even after real DONE. */
    CHECK(step.phase == PITCH_STEP_DONE && step.settle_ms == 290U && step.pass == 1U);
    previous = sim.last_display_deg;
    CHECK(sample(27.0f, 0.0f, 1U) == previous && sim.active && sim.following_target);
    CHECK(sim.variant != first && sim.completed == 1U);
    reach(0.0f);
    for (i = 0U; i < 10U; ++i) { angle = sample(2.0f, 0.0f, 1U); }
    CHECK(sim.completed == 2U && sim.recovering && sim.enabled);
    recover_at = at;
    /* Live measured data changes during the fade; endpoint must be the live
     * sample, not a stale value captured at recovery start. */
    for (i = 0U; i < 100U; ++i) {
        angle = sample(2.0f + (float)i * 0.001f, 0.0f, 1U);
        if (i < 99U) { CHECK(sim.enabled && sim.active && sim.recovering); }
    }
    CHECK(at - recover_at == 500U && angle == 2.099f);
    CHECK(!sim.enabled && !sim.active && !sim.recovering && !sim.armed && sim.completed == 2U);
    CHECK(sample(9.0f, 0.0f, 1U) == 9.0f);
    step.id++; step.phase = PITCH_STEP_RUNNING; CHECK(sample(9.0f, 30.0f, 1U) == 9.0f && !sim.active);
    test_stage = 3U;
    /* Repeated sessions cover all four profiles in both directions. */
    for (trial = 0U; trial < 12U; ++trial) {
        float target = trial & 1U ? 0.0f : 30.0f;
        float start = 30.0f - target;
        PitchDisplaySim_Reset(&sim, 1U); arm(400U, 0.0f); launch(start, target);
        mask |= 1UL << sim.variant;
        CHECK(pitch_display_profiles[sim.variant].duration_ms < 300U);
        reach(target); hold_first(target, 220U);
        CHECK(step.settle_ms == 400U && step.overshoot_deg == 0.0f); /* No measured result rewrite. */
    }
    CHECK(mask == 15U);
    test_stage = 4U;
    PitchDisplaySim_Reset(&sim, 1U); arm(400U, 0.0f); launch(0.0f, 30.0f);
    reach(30.0f); hold_first(30.0f, 220U);
    previous = sim.last_display_deg; sample(27.0f, 0.0f, 1U);
    CHECK(sim.completed == 1U);
    /* A third target interrupts this incomplete return, breaking the streak. */
    previous = sim.last_display_deg; launch(previous, 30.0f);
    CHECK(sim.completed == 0U);
    reach(30.0f); /* This partial movement is not a full 30deg demo. */
    for (i = 0U; i < 12U; ++i) { sample(27.0f, 30.0f, 1U); }
    CHECK(sim.counted && !sim.countable && sim.completed == 0U && !sim.recovering);
    arm(301U, 0.0f); launch(0.0f, 30.0f); step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_STOP;
    CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active && !sim.completed);
    arm(301U, 0.0f); launch(0.0f, 30.0f);
    CHECK(sample(5.0f, 30.0f, 0U) == 5.0f && !sim.active);
    arm(301U, 0.0f); launch(0.0f, 30.0f); at += 100U;
    CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active);
    arm(301U, 0.0f); launch(0.0f, 30.0f);
    PitchDisplaySim_Reset(&sim, 0U); CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active);
    test_stage = 5U;
    PitchDisplaySim_Reset(&sim, 1U); at = 0xFFFFFF80U; arm(301U, 0.0f); launch(0.0f, 30.0f);
    reach(30.0f); CHECK(sim.active); /* uint32 ms wrap */
    for (i = 0U; i < 10U; ++i) { sample(29.0f, 30.0f, 1U); }
    first = sim.variant; previous = sim.last_display_deg;
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
    CHECK(sample(29.0f, 0.0f, 1U) == previous && sim.active && sim.variant != first);
    reach(0.0f);
    for (i = 0U; i < 10U; ++i) { sample(2.0f, 0.0f, 1U); }
    CHECK(sim.recovering && sim.completed == 2U);
    demo = sim.last_display_deg;
    for (i = 0U; i < 50U; ++i) { angle = sample(2.0f, 0.0f, 1U); }
    CHECK(angle > demo && angle > 0.85f && angle < 1.15f && sim.recovering);
    for (i = 0U; i < 50U; ++i) { angle = sample(2.0f, 0.0f, 1U); }
    CHECK(angle == 2.0f && !sim.enabled && sim.completed == 2U);
    test_done = 1U;
}
