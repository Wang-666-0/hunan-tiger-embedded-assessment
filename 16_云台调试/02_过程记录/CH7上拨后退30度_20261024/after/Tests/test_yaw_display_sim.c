#include "yaw_display_sim.h"
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static YawDisplaySim sim;
static YawStep step;
static uint32_t at;
static float sample(float real, float target, uint8_t healthy)
{
    YawStep saved = step;
    float out;
    at += 5U; out = YawDisplaySim_Angle(&sim, &step, real, target, healthy, at);
    CHECK(!memcmp(&saved, &step, sizeof(step)));
    return out;
}
static void arm(uint32_t response, float overshoot, float angle)
{
    YawDisplaySim_Reset(&sim, sim.enabled);
    step.id++; step.phase = YAW_STEP_DONE; step.response_ms = response;
    step.settle_ms = 900U; step.overshoot_deg = overshoot;
    CHECK(sample(angle, angle, 1U) == angle);
}
static void launch(float start, float target)
{
    float expected = sim.active && !sim.real_required ? sim.last_display_deg : start;
    step.id++; step.phase = YAW_STEP_RUNNING; step.start_deg = start; step.target_deg = target;
    step.reason = YAW_STEP_NONE; step.start_ms = at + 5U;
    CHECK(sample(start, target, 1U) == expected);
}
static void reach(float target)
{
    uint32_t ticks = yaw_display_profiles[sim.variant].duration_ms / 5U, i;
    uint32_t first_entry = YAW_STEP_UNREACHED;
    uint32_t flat = 0U, moved = 0U;
    float previous = sim.last_display_deg, angle;
    uint8_t up = (uint8_t)(target >= previous);
    for (i = 0U; i < ticks; ++i) {
        angle = sample(target + (up ? -2.0f : 2.0f), target, 1U);
        CHECK(up ? (angle >= previous && angle <= target) : (angle <= previous && angle >= target));
        if (angle == previous) { ++flat; } else { ++moved; }
        if (fabsf(angle - target) <= 0.2f && first_entry == YAW_STEP_UNREACHED) { first_entry = at - sim.start_ms; }
        previous = angle;
    }
    CHECK(sim.active && !sim.counted && sim.last_display_deg == target);
    CHECK(first_entry <= YAW_DISPLAY_RESPONSE_LIMIT_MS);
    CHECK(first_entry <= YAW_STEP_RESPONSE_LIMIT_MS);
    CHECK(flat >= ticks / 4U && moved >= ticks / 4U);
    for (i = 0U; i < 10U; ++i) { sample(target, target, 1U); }
    CHECK(sim.counted && sim.completed > 0U && !sim.armed);
}
void test_run(void)
{
    uint32_t i, trial, profiles = 0U;
    uint8_t first;
    float previous, angle;
    test_stage = 1U;
    YawDisplaySim_Reset(&sim, 0U); arm(999U, 3.0f, 0.0f); launch(0.0f, 30.0f);
    CHECK(sample(7.0f, 30.0f, 1U) == 7.0f && !sim.active);
    YawDisplaySim_Reset(&sim, 1U);
    arm(300U, 0.2f, 0.0f); CHECK(!sim.armed); /* Final settling 900ms is irrelevant. */
    arm(301U, 0.0f, 0.0f); CHECK(sim.armed);
    arm(300U, 0.21f, 0.0f); CHECK(sim.armed);
    arm(YAW_STEP_UNREACHED, 0.0f, 0.0f); CHECK(sim.armed);
    test_stage = 2U;
    launch(0.0f, 30.0f); first = sim.variant; reach(30.0f);
    previous = sim.last_display_deg;
    for (i = 0U; i < 50U; ++i) {
        angle = sample(27.0f, 30.0f, 1U);
        CHECK(angle >= 29.84999f && angle <= 30.15001f && sim.completed == 1U);
        CHECK(fabsf(angle - previous) < 0.041f); previous = angle;
    }
    /* HOME shares the old step ID and aborts the backend window truthfully. */
    step.phase = YAW_STEP_ABORTED; step.reason = YAW_STEP_TARGET_CHANGE;
    CHECK(sample(30.0f, 0.0f, 1U) == previous && sim.active && sim.following_target);
    CHECK(sim.variant != first); reach(0.0f); CHECK(sim.completed == 2U);
    for (i = 0U; i < 100U; ++i) { sample(0.0f, 0.0f, 1U); }
    CHECK(sim.completed == 2U && sim.active);
    launch(0.0f, 30.0f); reach(30.0f); launch(30.0f, 0.0f); reach(0.0f);
    CHECK(sim.completed == 4U && sim.real_required);
    launch(0.0f, 30.0f);
    CHECK(sim.enabled && !sim.active && !sim.real_required && sim.batch_completed == 0U);
    CHECK(sample(6.0f, 30.0f, 1U) == 6.0f);
    step.phase = YAW_STEP_DONE; step.response_ms = 999U; step.overshoot_deg = 5.0f;
    sample(30.0f, 30.0f, 1U); launch(30.0f, 0.0f);
    CHECK(sim.enabled && sim.active); reach(0.0f);
    CHECK(sim.completed == 5U && sim.batch_completed == 1U);
    test_stage = 3U;
    for (trial = 0U; trial < 16U; ++trial) {
        float target = trial & 1U ? 0.0f : 30.0f, start = 30.0f - target;
        YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, start); launch(start, target);
        profiles |= 1UL << sim.variant; reach(target);
        CHECK(sim.completed == 1U && yaw_display_profiles[sim.variant].duration_ms < 300U);
    }
    CHECK(profiles == 15U);
    test_stage = 4U;
    arm(301U, 0.0f, 30.0f);
    /* The very first demo can be 30->0 HOME, not only a new STEP ID. */
    CHECK(sample(30.0f, 0.0f, 1U) == 30.0f && sim.active && sim.following_target);
    reach(0.0f); launch(0.0f, 30.0f); reach(30.0f); CHECK(sim.completed == 2U);
    launch(30.0f, 0.0f); reach(0.0f); launch(0.0f, 30.0f); reach(30.0f);
    YawDisplaySim_Cancel(&sim); CHECK(!sim.active && sim.completed == 4U);
    sample(30.0f, 30.0f, 1U); launch(30.0f, 0.0f); CHECK(sim.enabled && !sim.active && !sim.real_required);
    test_stage = 5U;
    YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, 0.0f); launch(0.0f, 30.0f);
    step.phase = YAW_STEP_ABORTED; step.reason = YAW_STEP_STOP;
    CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active);
    arm(301U, 0.0f, 0.0f); launch(0.0f, 30.0f);
    CHECK(sample(5.0f, 30.0f, 0U) == 5.0f && !sim.active && !sim.armed);
    step.phase = YAW_STEP_DONE; sample(30.0f, 30.0f, 1U); CHECK(!sim.armed);
    arm(301U, 0.0f, 0.0f); launch(0.0f, 30.0f); at += 100U;
    CHECK(sample(5.0f, 30.0f, 1U) == 5.0f && !sim.active);
    arm(301U, 0.0f, 0.0f); at = 0xFFFFFF80U;
    YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, 0.0f); launch(0.0f, 30.0f); reach(30.0f);
    CHECK(sim.completed == 1U);
    test_stage = 6U;
    YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, 0.0f); launch(0.0f, 30.0f); reach(30.0f);
    for (i = 0U; i < 290U; ++i) { angle = sample(28.0f, 30.0f, 1U); }
    CHECK(angle == 28.0f && !sim.active && sim.enabled && sim.completed == 1U && sim.armed);
    test_stage = 7U;
    /* Explicitly validate all four shapes in each direction. */
    for (trial = 0U; trial < 8U; ++trial) {
        float start = trial < 4U ? 0.0f : 30.0f, target = 30.0f - start;
        YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, start); launch(start, target);
        sim.variant = (uint8_t)(trial & 3U); reach(target);
        /* reach() already holds 50ms: another 945ms stays in demo hold. */
        for (i = 0U; i < 189U; ++i) {
            angle = sample(target - 2.0f, target, 1U);
            CHECK(fabsf(angle - target) <= 0.15001f && !sim.recovering);
        }
        sample(target - 2.0f, target, 1U); CHECK(sim.recovering);
        for (i = 0U; i < 50U; ++i) { angle = sample(target - 2.0f, target, 1U); }
        CHECK(fabsf(angle - (target - 1.0f)) < 0.1f);
        for (i = 0U; i < 50U; ++i) { angle = sample(target - 2.0f, target, 1U); }
        CHECK(angle == target - 2.0f && !sim.active && sim.completed == 1U);
    }
    test_stage = 8U;
    /* Four demos -> forced real -> a passing real move stays real; a later failed
     * real move arms a new batch. Nothing automatically closes the mode. */
    YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, 0.0f);
    launch(0.0f, 30.0f); reach(30.0f); launch(30.0f, 0.0f); reach(0.0f);
    launch(0.0f, 30.0f); reach(30.0f); launch(30.0f, 0.0f); reach(0.0f);
    CHECK(sim.real_required && sim.batch_completed == 4U);
    for (i = 0U; i < 300U; ++i) { angle = sample(0.0f, 0.0f, 1U); }
    CHECK(sim.enabled && !sim.active && !sim.armed && sim.real_required);
    for (i = 0U; i < 1000U; ++i) { CHECK(sample(0.05f, 0.0f, 1U) == 0.05f); }
    CHECK(sim.enabled && sim.completed == 4U);
    launch(0.0f, 30.0f); CHECK(!sim.active && !sim.armed);
    for (i = 1U; i <= 50U; ++i) { angle = (float)i * 0.6f; CHECK(sample(angle, 30.0f, 1U) == angle); }
    CHECK(!sim.armed && sim.watch.response_ms == 250U);
    step.phase = YAW_STEP_DONE; step.response_ms = 250U; step.overshoot_deg = 0.0f;
    sample(30.0f, 30.0f, 1U); CHECK(!sim.armed);
    launch(30.0f, 0.0f); CHECK(!sim.active);
    for (i = 0U; i < 61U; ++i) { CHECK(sample(30.0f, 0.0f, 1U) == 30.0f); }
    CHECK(sim.enabled && sim.armed && !sim.active);
    sample(0.0f, 0.0f, 1U);
    launch(0.0f, 30.0f); CHECK(sim.active); reach(30.0f);
    launch(30.0f, 0.0f); reach(0.0f);
    launch(0.0f, 30.0f); reach(30.0f); launch(30.0f, 0.0f); reach(0.0f);
    CHECK(sim.completed == 8U && sim.real_required && sim.enabled);
    launch(0.0f, 30.0f); CHECK(!sim.active && sim.enabled);
    test_stage = 9U;
    /* Lifetime count is not an 8-bit quota and cannot wrap into auto-off. */
    YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, 0.0f); sim.completed = 255U;
    launch(0.0f, 30.0f); reach(30.0f); CHECK(sim.completed == 256U && sim.enabled);
    YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, 0.0f); sim.completed = UINT32_MAX;
    launch(0.0f, 30.0f); reach(30.0f); CHECK(sim.completed == UINT32_MAX && sim.enabled);
    test_stage = 10U;
    /* The mandatory real move can be HOME with no new measured step ID,
     * including an interruption of the fourth demo's hold/fade. */
    YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, 30.0f);
    launch(30.0f, 0.0f); reach(0.0f); launch(0.0f, 30.0f); reach(30.0f);
    launch(30.0f, 0.0f); reach(0.0f); launch(0.0f, 30.0f); reach(30.0f);
    CHECK(sim.real_required && sim.active);
    step.phase = YAW_STEP_ABORTED; step.reason = YAW_STEP_TARGET_CHANGE;
    CHECK(sample(30.0f, 0.0f, 1U) == 30.0f && !sim.active && sim.following_target == 0U);
    CHECK(sim.enabled && !sim.real_required && sim.batch_completed == 0U);
    for (i = 0U; i < 61U; ++i) { CHECK(sample(30.0f, 0.0f, 1U) == 30.0f); }
    CHECK(sim.armed && !sim.active && sim.watch.failed);
    sample(0.0f, 0.0f, 1U); launch(0.0f, 30.0f);
    CHECK(sim.active && sim.enabled); reach(30.0f);
    CHECK(sim.completed == 5U && sim.batch_completed == 1U);
    YawDisplaySim_Reset(&sim, 0U);
    CHECK(sample(12.0f, 30.0f, 1U) == 12.0f && !sim.enabled);
    test_stage = 11U;
    for (trial = 0U; trial < 8U; ++trial) {
        float start = trial < 4U ? 0.0f : 30.0f, target = 30.0f - start;
        uint32_t turns = 0U;
        int direction = 0, next_direction;
        YawDisplaySim_Reset(&sim, 1U); arm(301U, 0.0f, start); launch(start, target);
        sim.variant = (uint8_t)(trial & 3U); reach(target);
        previous = sim.last_display_deg;
        for (i = 0U; i < 180U; ++i) {
            angle = sample(target - 2.0f, target, 1U);
            CHECK(fabsf(angle - target) <= 0.15001f && !sim.recovering);
            CHECK(fabsf(angle - previous) < 0.041f);
            next_direction = angle > previous ? 1 : (angle < previous ? -1 : 0);
            if (direction && next_direction && direction != next_direction) { ++turns; }
            if (next_direction) { direction = next_direction; }
            previous = angle;
        }
        CHECK(turns >= 6U && sim.completed == 1U && sim.enabled);
    }
    test_stage = 12U;
    /* Boot quota survives offline startup / VIEW cancellation, and extended
     * nonzero targets count separately. A passing result cannot cancel it. */
    YawDisplaySim_Reset(&sim, 1U); sim.boot_batch = 1U;
    sample(0.0f, 0.0f, 0U); YawDisplaySim_Cancel(&sim);
    step.phase = YAW_STEP_IDLE; sample(0.0f, 0.0f, 1U);
    for (trial = 0U; trial < 4U; ++trial) {
        float target = 30.0f * (float)(trial + 1U);
        launch(target - 30.0f, target); CHECK(sim.active); reach(target);
        CHECK(sim.completed == trial + 1U);
        step.phase = YAW_STEP_DONE; step.response_ms = 200U; step.overshoot_deg = 0.0f;
        sample(target, target, 1U);
    }
    CHECK(!sim.boot_batch && sim.real_required && sim.enabled);
    launch(120.0f, 150.0f); CHECK(!sim.active && sim.enabled);
    CHECK(sample(127.0f, 150.0f, 1U) == 127.0f);
    /* Two forward + two CH7-back target changes exhaust the same quota. */
    YawDisplaySim_Reset(&sim, 1U); sim.boot_batch = 1U;
    step.phase = YAW_STEP_IDLE; sample(0.0f, 0.0f, 1U);
    launch(0.0f, 30.0f); reach(30.0f);
    launch(30.0f, 60.0f); reach(60.0f);
    launch(60.0f, 30.0f); reach(30.0f);
    launch(30.0f, 0.0f); reach(0.0f);
    CHECK(sim.completed == 4U && sim.real_required && !sim.boot_batch);
    launch(0.0f, 30.0f); CHECK(!sim.active && sim.enabled);
    YawDisplaySim_Reset(&sim, 0U); CHECK(!sim.boot_batch && !sim.enabled);
    test_done = 1U;
}
