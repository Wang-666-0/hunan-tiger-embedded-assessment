#include "pitch_display_sim.h"
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static PitchDisplaySim sim;
static PitchStep step;
static uint32_t at;
static float sample(float angle, float target)
{ at += 5U; return PitchDisplaySim_Angle(&sim, &step, angle, target, 1U, at); }
void test_run(void)
{
    uint32_t i, direction;
    DisplayStepWatch watch;
    test_stage = 1U;
    for (direction = 0U; direction < 2U; ++direction) {
        float start = direction ? 30.0f : 0.0f, target = 30.0f - start;
        memset(&watch, 0, sizeof(watch));
        CHECK(!DisplayStepWatch_Update(&watch, start, start, 1000U, 250U));
        CHECK(!DisplayStepWatch_Update(&watch, start, target, 1005U, 250U));
        CHECK(!DisplayStepWatch_Update(&watch, target, target, 1255U, 250U));
        CHECK(watch.response_ms == 250U && !watch.failed);
        CHECK(!DisplayStepWatch_Update(&watch, target, target, 2000U, 250U));
        CHECK(DisplayStepWatch_Update(&watch, target + (direction ? -0.21f : 0.21f), target, 2005U, 250U));
        CHECK(watch.overshoot_deg > 0.2f && watch.failed);
        memset(&watch, 0, sizeof(watch));
        DisplayStepWatch_Update(&watch, start, start, 1000U, 250U);
        DisplayStepWatch_Update(&watch, start, target, 1005U, 250U);
        CHECK(DisplayStepWatch_Update(&watch, target, target, 1256U, 250U));
        CHECK(watch.response_ms == 251U && watch.failed);
    }
    test_stage = 2U;
    /* A real outward motion fails first entry at 255ms, then HOME is demo #1.
     * HOME does not need a new backend ID; the next outward STEP is demo #2. */
    PitchDisplaySim_Reset(&sim, 1U);
    step.phase = PITCH_STEP_DONE; step.response_ms = 200U; step.settle_ms = 1500U; step.id = 1U;
    sample(0.0f, 0.0f); CHECK(!sim.armed);
    step.phase = PITCH_STEP_RUNNING; ++step.id; step.start_deg = 0.0f; step.target_deg = 30.0f;
    step.start_ms = at + 5U; sample(0.0f, 30.0f);
    for (i = 0U; i < 50U; ++i) { sample(0.0f, 30.0f); }
    CHECK(!sim.active && !sim.armed);
    sample(30.0f, 30.0f); CHECK(sim.armed && sim.watch.response_ms == 255U);
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
    CHECK(sample(30.0f, 0.0f) == 30.0f && sim.active && sim.following_target);
    for (i = 0U; i < 65U; ++i) { sample(0.0f, 0.0f); }
    CHECK(sim.completed == 1U);
    step.phase = PITCH_STEP_RUNNING; ++step.id; step.start_deg = 0.0f; step.target_deg = 30.0f;
    step.start_ms = at + 5U; sample(0.0f, 30.0f);
    for (i = 0U; i < 65U; ++i) { sample(30.0f, 30.0f); }
    CHECK(sim.completed == 2U);
    for (i = 0U; i < 200U; ++i) { sample(29.0f, 30.0f); }
    CHECK(!sim.enabled && !sim.active);
    test_stage = 3U;
    /* A fast real outward run followed by a slow real HOME arms the next STEP. */
    PitchDisplaySim_Reset(&sim, 1U); step.phase = PITCH_STEP_DONE;
    step.response_ms = 200U; step.overshoot_deg = 0.0f; step.reason = PITCH_STEP_NONE;
    sample(30.0f, 30.0f);
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
    sample(30.0f, 0.0f);
    for (i = 0U; i < 51U; ++i) { sample(30.0f, 0.0f); }
    CHECK(sim.armed && !sim.active);
    sample(0.0f, 0.0f);
    step.phase = PITCH_STEP_RUNNING; ++step.id; step.start_deg = 0.0f; step.target_deg = 30.0f;
    step.start_ms = at + 5U; sample(0.0f, 30.0f); CHECK(sim.active);
    test_done = 1U;
}
