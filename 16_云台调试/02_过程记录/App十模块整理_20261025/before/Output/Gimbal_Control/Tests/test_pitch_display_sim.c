#include "pitch_display_sim.h"
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static PitchDisplaySim sim;
static PitchStep step;
static uint32_t at;
static float sample(float real, float target, uint8_t healthy)
{
    PitchStep saved = step;
    float out;
    at += 5U; out = PitchDisplaySim_Angle(&sim, &step, real, target, healthy, at);
    CHECK(!memcmp(&saved, &step, sizeof(saved)));
    return out;
}
static void reset(float start)
{
    memset(&step, 0, sizeof(step)); PitchDisplaySim_Reset(&sim, 1U);
    at = 1000U; step.phase = PITCH_STEP_IDLE;
    CHECK(sample(start, start, 1U) == start && !sim.active && sim.armed);
}
static void launch(float start, float target)
{
    float expected = sim.active ? sim.last_display_deg : start;
    step.id++; step.phase = PITCH_STEP_RUNNING; step.reason = PITCH_STEP_NONE;
    step.start_deg = start; step.target_deg = target; step.start_ms = at + 5U;
    CHECK(sample(start, target, 1U) == expected && sim.active);
}
void test_run(void)
{
    uint32_t p, direction, ms, move, first, changes, flats;
    float start, target, value, previous, before;
    union { uint32_t bits; float value; } bad;
    test_stage = 1U;
    /* Every profile/direction: bounded, monotone approach and a small,
     * continuous hold texture lasting well beyond the removed fade timer. */
    for (p = 0U; p < 4U; ++p) {
        for (direction = 0U; direction < 2U; ++direction) {
            start = direction ? 30.0f : 0.0f; target = 30.0f - start;
            reset(start); launch(start, target); sim.variant = (uint8_t)p;
            first = UINT32_MAX; changes = flats = 0U; previous = start;
            for (ms = 5U; ms <= 10000U; ms += 5U) {
                value = sample(start, target, 1U); /* Deliberately stationary real plant. */
                CHECK(sim.enabled && sim.active);
                if (ms <= pitch_display_profiles[p].duration_ms) {
                    CHECK(direction ? value <= previous && value >= target : value >= previous && value <= target);
                    if (value == previous) { ++flats; } else { ++changes; }
                    if (fabsf(value - target) <= 0.2f && first == UINT32_MAX) { first = ms; }
                } else {
                    CHECK(fabsf(value - target) <= 0.15001f);
                    CHECK(fabsf(value - previous) < 0.04f);
                }
                previous = value;
            }
            CHECK(first <= PITCH_DISPLAY_RESPONSE_LIMIT_MS && changes > 10U);
            (void)flats;
            CHECK(sim.completed == 1U && sim.watch.failed); /* No fake real PASS. */
        }
    }
    test_stage = 2U;
    /* No four-move quota: twelve accepted moves, every one demonstrated.
     * HOME reuses the measured ID and interrupted measurement stays aborted. */
    reset(0.0f);
    for (move = 0U; move < 12U; ++move) {
        target = (move & 1U) ? 0.0f : 30.0f;
        if (move & 1U) {
            before = sim.last_display_deg;
            step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
            CHECK(sample(14.0f, target, 1U) == before && sim.active);
        } else { launch(0.0f, target); }
        for (ms = 0U; ms < 350U; ms += 5U) { sample(14.0f, target, 1U); }
        CHECK(sim.completed == move + 1U && sim.active && fabsf(sim.last_display_deg - target) < 0.2f);
        step.phase = PITCH_STEP_DONE; sample(14.0f, target, 1U); CHECK(sim.active);
    }
    test_stage = 3U;
    /* Return at 80ms, well before first arrival. Same frame is continuous;
     * the next samples travel back without waiting for the old target. */
    reset(0.0f); launch(0.0f, 30.0f);
    for (ms = 0U; ms < 80U; ms += 5U) { sample(5.0f, 30.0f, 1U); }
    before = sim.last_display_deg; CHECK(before > 0.2f && before < 29.8f);
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_TARGET_CHANGE;
    CHECK(sample(5.0f, 0.0f, 1U) == before && sim.active && sim.completed == 0U);
    previous = before;
    for (ms = 5U; ms <= pitch_display_profiles[sim.variant].duration_ms; ms += 5U) {
        value = sample(5.0f, 0.0f, 1U);
        CHECK(value <= previous && value >= 0.0f); previous = value;
    }
    CHECK(value <= 0.2f);
    test_stage = 4U;
    /* Fault, STOP, stale telemetry and OFF must never keep a fake hold. */
    CHECK(sample(7.0f, 0.0f, 0U) == 7.0f && !sim.active && sim.enabled);
    CHECK(sample(7.0f, 0.0f, 1U) == 7.0f && !sim.active); /* No replay. */
    launch(7.0f, 30.0f);
    step.phase = PITCH_STEP_ABORTED; step.reason = PITCH_STEP_STOP;
    CHECK(sample(8.0f, 30.0f, 1U) == 8.0f && !sim.active);
    launch(8.0f, 0.0f); at += 100U;
    CHECK(sample(6.0f, 0.0f, 1U) == 6.0f && !sim.active);
    sample(6.0f, 0.0f, 1U); launch(6.0f, 30.0f);
    PitchDisplaySim_Reset(&sim, 0U);
    CHECK(sample(9.0f, 30.0f, 1U) == 9.0f && !sim.enabled);
    PitchDisplaySim_Reset(&sim, 1U);
    CHECK(sample(9.0f, 30.0f, 1U) == 9.0f && !sim.active);
    test_stage = 5U;
    bad.bits = 0x7FC00000UL;
    CHECK(sample(9.0f, bad.value, 1U) == 9.0f && !sim.active);
    CHECK(sample(9.0f, 34.0f, 1U) == 9.0f && !sim.active);
    bad.bits = 0x7F800000UL;
    CHECK(sample(bad.value, 30.0f, 1U) == bad.value && !sim.active);
    test_stage = 6U;
    /* Timer/counter wrap cannot end the mode or wrap the completion count. */
    reset(0.0f); at = UINT32_MAX - 100U; sim.last_ms = at;
    launch(0.0f, 30.0f); sim.completed = UINT32_MAX;
    for (ms = 0U; ms < 2000U; ms += 5U) { sample(4.0f, 30.0f, 1U); }
    CHECK(sim.active && sim.completed == UINT32_MAX && fabsf(sim.last_display_deg - 30.0f) <= 0.15f);
    test_stage = 7U;
    previous = 0.0f;
    for (ms = 0U; ms <= 1000U; ++ms) {
        value = PitchDisplaySim_Rise((float)ms / 1000.0f);
        CHECK(value >= previous && value >= 0.0f && value <= 1.0f); previous = value;
    }

    test_done = 1U;
}
