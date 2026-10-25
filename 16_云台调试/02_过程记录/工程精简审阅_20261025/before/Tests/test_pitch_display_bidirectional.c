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
    PitchDisplaySim_Reset(&sim, 1U);
    step.phase = PITCH_STEP_IDLE; sample(0.0f, 0.0f);
    for (direction = 0U; direction < 10U; ++direction) {
        float target = direction & 1U ? 0.0f : 30.0f;
        step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
        sample(15.0f, target);
        for (i = 0U; i < 65U; ++i) { sample(15.0f, target); }
        CHECK(sim.active && sim.enabled && sim.completed == direction + 1U);
        CHECK(fabsf(sim.last_display_deg - target) <= 0.15f);
        CHECK(sim.watch.failed && sim.watch.response_ms == PITCH_STEP_UNREACHED);
    }
    test_done = 1U;
}
