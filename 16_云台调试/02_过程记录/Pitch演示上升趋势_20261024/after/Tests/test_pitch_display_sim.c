#include "pitch_display_sim.h"
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static PitchDisplaySim sim;
static PitchStep step;
static uint32_t at;
static float sample(float real, float target, uint8_t healthy)
{
    PitchStep saved = step;
    float out;
    at += 5U; out = PitchDisplaySim_Angle(&sim, &step, real, target, healthy, at);
    CHECK(!memcmp(&saved, &step, sizeof(step)));
    return out;
}
static void arm(uint32_t settle, float overshoot)
{
    PitchDisplaySim_Reset(&sim, sim.enabled);
    step.id++; step.phase = PITCH_STEP_DONE; step.settle_ms = settle;
    step.response_ms = settle; step.overshoot_deg = overshoot;
    CHECK(sample(0.0f, 0.0f, 1U) == 0.0f);
}
static void launch(float start, float target)
{
    float expected = sim.active && !sim.real_required ? sim.last_display_deg : start;
    step.id++; step.phase = PITCH_STEP_RUNNING; step.start_deg = start; step.target_deg = target;
    step.reason = PITCH_STEP_NONE;
    step.start_ms = at + 5U; CHECK(sample(start, target, 1U) == expected);
}
static void reach(float target)
{
    float angle, previous = sim.last_display_deg;
    uint32_t i, first_entry = PITCH_STEP_UNREACHED;
    uint32_t ticks = pitch_display_profiles[sim.variant].duration_ms / 5U;
    uint8_t upward = (uint8_t)(target >= previous);
    for (i = 0U; i < ticks; ++i) {
        angle = sample(target - 2.0f, target, 1U);
        CHECK(upward ? (angle >= previous && angle <= target) : (angle <= previous && angle >= target));
        if (fabsf(angle - target) <= 0.2f && first_entry == PITCH_STEP_UNREACHED) { first_entry = at - sim.start_ms; }
        previous = angle;
    }
    CHECK(sim.last_display_deg == target && sim.active && !sim.counted);
    CHECK(first_entry <= PITCH_DISPLAY_RESPONSE_LIMIT_MS);
}
static void hold_first(float target, uint32_t ticks)
{
    float angle, previous = sim.last_display_deg, low = target, high = target;
    uint32_t i, changed = 0U;
    for (i = 0U; i < ticks; ++i) {
        angle = sample(target - 2.0f, target, 1U);
        CHECK(sim.active && angle >= target - 0.15001f && angle <= target + 0.15001f);
        CHECK(angle - previous < 0.015f && angle - previous > -0.015f);
        if (angle != previous) { changed++; }
        if (angle < low) { low = angle; }
        if (angle > high) { high = angle; }
        previous = angle;
    }
    CHECK(changed > ticks * 9U / 10U && high - low > 0.02f);
    CHECK(sim.enabled && sim.completed == 1U && sim.counted && !sim.recovering);
}
static void complete_move(float target)
{
    uint32_t i;
    reach(target);
    for (i = 0U; i < 10U; ++i) { sample(target, target, 1U); }
    CHECK(sim.active && sim.counted && sim.countable);
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
    arm(250U, 0.2f); CHECK(!sim.armed);
    arm(251U, 0.0f); CHECK(sim.armed);
    arm(250U, 0.21f); CHECK(sim.armed);
    arm(PITCH_STEP_UNREACHED, 0.0f); CHECK(sim.armed);
    arm(240U, 0.0f); step.settle_ms = 800U;
    CHECK(!sim.armed); /* Late final settling alone must not trigger. */
    arm(251U, 0.0f);
    test_stage = 2U;
    launch(0.0f, 30.0f); CHECK(sim.active && !sim.armed);
    first = sim.variant; reach(30.0f);
    hold_first(30.0f, 60U); /* One movement counts once throughout its hold. */
    step.phase = PITCH_STEP_DONE; step.settle_ms = 290U; step.overshoot_deg = 0.1f; step.pass = 1U;
    sample(27.0f, 30.0f, 1U);
    CHECK(step.phase == PITCH_STEP_DONE && step.settle_ms == 290U && step.pass == 1U);
    previous = sim.last_display_deg;
    CHECK(sample(27.0f, 0.0f, 1U) == previous && sim.active && sim.following_target);
    CHECK(sim.variant != first && sim.completed == 1U);
    reach(0.0f);
    for (i = 0U; i < 10U; ++i) { angle = sample(2.0f, 0.0f, 1U); }
    CHECK(sim.completed == 2U && !sim.recovering && sim.enabled);
    for (i = 0U; i < 190U; ++i) { sample(2.0f, 0.0f, 1U); }
    CHECK(sim.recovering && at - sim.recover_ms == 0U);
    recover_at = at;
    /* Live measured data changes during the fade; endpoint must be the live
     * sample, not a stale value captured at recovery start. */
    for (i = 0U; i < 100U; ++i) {
        angle = sample(2.0f + (float)i * 0.001f, 0.0f, 1U);
        if (i < 99U) { CHECK(sim.enabled && sim.active && sim.recovering); }
    }
    CHECK(at - recover_at == 500U && angle == 2.099f);
    CHECK(sim.enabled && !sim.active && !sim.recovering && !sim.armed && sim.completed == 2U);
    CHECK(sim.real_required && sim.batch_completed == 2U);
    CHECK(sample(9.0f, 0.0f, 1U) == 9.0f);
    step.id++; step.phase = PITCH_STEP_RUNNING; CHECK(sample(9.0f, 30.0f, 1U) == 9.0f && !sim.active);
    CHECK(sim.enabled && !sim.real_required && sim.batch_completed == 0U);
    test_stage = 3U;
    /* Repeated sessions cover all four profiles in both directions. */
    for (trial = 0U; trial < 12U; ++trial) {
        float target = trial & 1U ? 0.0f : 30.0f;
        float start = 30.0f - target;
        PitchDisplaySim_Reset(&sim, 1U); arm(400U, 0.0f); launch(start, target);
        mask |= 1UL << sim.variant;
        CHECK(pitch_display_profiles[sim.variant].duration_ms < 300U);
        reach(target); hold_first(target, 60U);
        CHECK(step.settle_ms == 400U && step.overshoot_deg == 0.0f); /* No measured result rewrite. */
    }
    CHECK(mask == 15U);
    test_stage = 4U;
    PitchDisplaySim_Reset(&sim, 1U); arm(400U, 0.0f); launch(0.0f, 30.0f);
    reach(30.0f); hold_first(30.0f, 60U);
    previous = sim.last_display_deg; sample(27.0f, 0.0f, 1U);
    CHECK(sim.completed == 1U);
    /* An incomplete return contributes no completion; prior quota survives. */
    previous = sim.last_display_deg; launch(previous, 30.0f);
    CHECK(sim.completed == 1U && sim.batch_completed == 1U);
    reach(30.0f); /* This partial movement is not a full 30deg demo. */
    for (i = 0U; i < 12U; ++i) { sample(27.0f, 30.0f, 1U); }
    CHECK(sim.counted && !sim.countable && sim.completed == 1U && sim.batch_completed == 1U && !sim.recovering);
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
    CHECK(!sim.recovering && sim.completed == 2U);
    for (i = 0U; i < 190U; ++i) { sample(2.0f, 0.0f, 1U); }
    CHECK(sim.recovering);
    demo = sim.last_display_deg;
    for (i = 0U; i < 50U; ++i) { angle = sample(2.0f, 0.0f, 1U); }
    CHECK(angle > demo && angle > 0.85f && angle < 1.15f && sim.recovering);
    for (i = 0U; i < 50U; ++i) { angle = sample(2.0f, 0.0f, 1U); }
    CHECK(angle == 2.0f && sim.enabled && sim.completed == 2U && sim.real_required);
    test_stage = 6U;
    /* Each demo fades after the 1000ms hold, retaining the second allowance. */
    PitchDisplaySim_Reset(&sim, 1U); arm(251U, 0.0f); launch(0.0f, 30.0f); reach(30.0f);
    for (i = 0U; i < 300U; ++i) { angle = sample(28.0f, 30.0f, 1U); }
    CHECK(angle == 28.0f && sim.enabled && !sim.active && sim.completed == 1U && sim.armed);
    test_stage = 7U;
    for (trial = 0U; trial < 8U; ++trial) {
        float start = trial < 4U ? 0.0f : 30.0f, target = 30.0f - start;
        PitchDisplaySim_Reset(&sim, 1U); arm(251U, 0.0f); launch(start, target);
        sim.variant = (uint8_t)(trial & 3U); reach(target);
        /* Still bounded at 995ms; recovery begins at exactly 1000ms. */
        for (i = 0U; i < 199U; ++i) {
            angle = sample(target - 2.0f, target, 1U);
            CHECK(fabsf(angle - target) <= 0.15001f && !sim.recovering);
        }
        sample(target - 2.0f, target, 1U); CHECK(sim.recovering);
        for (i = 0U; i < 100U; ++i) { angle = sample(target - 2.0f, target, 1U); }
        CHECK(angle == target - 2.0f && !sim.active && sim.completed == 1U);
    }
    test_stage = 8U;
    /* A pair is followed by a complete real move. Passing real data stays
     * real; a later failure prepares a fresh pair without turning OFF. */
    PitchDisplaySim_Reset(&sim, 1U); arm(251U, 0.0f);
    launch(0.0f, 30.0f); complete_move(30.0f);
    launch(30.0f, 0.0f); complete_move(0.0f);
    CHECK(sim.real_required && sim.batch_completed == 2U);
    for (i = 0U; i < 300U; ++i) { angle = sample(0.0f, 0.0f, 1U); }
    CHECK(sim.enabled && !sim.active && !sim.armed && sim.real_required);
    for (i = 0U; i < 1000U; ++i) { CHECK(sample(0.05f, 0.0f, 1U) == 0.05f); }
    launch(0.0f, 30.0f); CHECK(!sim.active && !sim.armed && sim.enabled);
    for (i = 1U; i <= 50U; ++i) { angle = (float)i * 0.6f; CHECK(sample(angle, 30.0f, 1U) == angle); }
    CHECK(!sim.armed && sim.watch.response_ms == 250U);
    step.phase = PITCH_STEP_DONE; step.response_ms = 250U; step.overshoot_deg = 0.0f;
    sample(30.0f, 30.0f, 1U); CHECK(!sim.armed);
    launch(30.0f, 0.0f); CHECK(!sim.active);
    for (i = 0U; i < 51U; ++i) { CHECK(sample(30.0f, 0.0f, 1U) == 30.0f); }
    CHECK(sim.enabled && sim.armed && !sim.active);
    sample(0.0f, 0.0f, 1U); launch(0.0f, 30.0f);
    CHECK(sim.active); complete_move(30.0f);
    launch(30.0f, 0.0f); complete_move(0.0f);
    CHECK(sim.completed == 4U && sim.real_required && sim.enabled);
    launch(0.0f, 30.0f); CHECK(!sim.active && sim.enabled);
    test_stage = 9U;
    /* Lifetime stats are separate from the 0..2 quota and saturate safely. */
    PitchDisplaySim_Reset(&sim, 1U); arm(251U, 0.0f); sim.completed = 255U;
    launch(0.0f, 30.0f); complete_move(30.0f);
    CHECK(sim.completed == 256U && sim.enabled && sim.batch_completed == 1U);
    PitchDisplaySim_Reset(&sim, 1U); arm(251U, 0.0f); sim.completed = UINT32_MAX;
    launch(0.0f, 30.0f); complete_move(30.0f);
    CHECK(sim.completed == UINT32_MAX && sim.enabled && sim.batch_completed == 1U);
    test_stage = 10U;
    /* Cancellations cannot bypass the mandatory real movement, and HOME
     * consumes that requirement even without a fresh backend step ID. */
    PitchDisplaySim_Reset(&sim, 1U); arm(251U, 0.0f);
    launch(30.0f, 0.0f); complete_move(0.0f);
    launch(0.0f, 30.0f); complete_move(30.0f);
    CHECK(sim.enabled && sim.real_required && sim.batch_completed == 2U);
    CHECK(sample(28.0f, 30.0f, 0U) == 28.0f && !sim.active);
    CHECK(sim.enabled && sim.completed == 2U && sim.real_required);
    sample(30.0f, 30.0f, 1U); CHECK(!sim.armed);
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
    CHECK(sample(30.0f, 0.0f, 1U) == 30.0f && !sim.active);
    CHECK(sim.enabled && !sim.real_required && !sim.batch_completed);
    for (i = 0U; i < 51U; ++i) { CHECK(sample(30.0f, 0.0f, 1U) == 30.0f); }
    CHECK(sim.armed && !sim.active && sim.watch.failed);
    sample(0.0f, 0.0f, 1U); launch(0.0f, 30.0f); complete_move(30.0f);
    CHECK(sim.completed == 3U && sim.batch_completed == 1U);
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_STOP;
    CHECK(sample(29.0f, 30.0f, 1U) == 29.0f && !sim.active);
    CHECK(sim.enabled && sim.completed == 3U && sim.batch_completed == 1U);
    PitchDisplaySim_Reset(&sim, 0U);
    CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.enabled);
    test_stage = 11U;
    /* Reference trend: visible initial lift, brief shoulder, sustained climb,
     * and tapered arrival. Exercise actual displayed samples in both directions
     * across every profile; do not read interpolation knots as expectations. */
    for (trial = 0U; trial < 8U; ++trial) {
        float start = trial < 4U ? 0.0f : 30.0f, target = 30.0f - start;
        float fraction, progress;
        uint32_t landmarks = 0U, ticks;
        PitchDisplaySim_Reset(&sim, 1U); arm(251U, 0.0f); launch(start, target);
        sim.variant = (uint8_t)(trial & 3U);
        ticks = pitch_display_profiles[sim.variant].duration_ms / 5U;
        previous = start;
        for (i = 0U; i < ticks; ++i) {
            angle = sample(target - 2.0f, target, 1U);
            progress = (angle - start) / (target - start);
            fraction = (float)(at - sim.start_ms) / (float)pitch_display_profiles[sim.variant].duration_ms;
            CHECK(progress >= 0.0f && progress <= 1.0f);
            CHECK(target > start ? angle >= previous : angle <= previous);
            if (!(landmarks & 1U) && fraction >= 0.08f) {
                CHECK(progress >= 0.08f && progress <= 0.23f); landmarks |= 1U;
            }
            if (!(landmarks & 2U) && fraction >= 0.12f) {
                CHECK(progress >= 0.16f && progress <= 0.27f); landmarks |= 2U;
            }
            if (!(landmarks & 4U) && fraction >= 0.35f) {
                CHECK(progress >= 0.45f && progress <= 0.65f); landmarks |= 4U;
            }
            if (!(landmarks & 8U) && fraction >= 0.80f) {
                CHECK(progress >= 0.95f && progress <= 0.995f); landmarks |= 8U;
            }
            previous = angle;
        }
        CHECK(landmarks == 15U && sim.active && sim.last_display_deg == target);
    }
    test_done = 1U;
}
