#include "pitch_step.h"
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static PitchStep pitch;
static YawStep yaw;
static void ramp(uint32_t start, uint32_t duration, float target)
{
    uint32_t t;
    float angle;
    PitchStep_Init(&pitch); YawStep_Init(&yaw);
    CHECK(PitchStep_Start(&pitch, start, 0.0f, target)); CHECK(YawStep_Start(&yaw, start, 0.0f, target));
    for (t = 10U; t <= 5000U; t += 10U) {
        angle = t < duration ? target * (float)t / (float)duration : target;
        PitchStep_Sample(&pitch, start + t, angle, 120.0f, 7.2f, 0U);
        YawStep_Sample(&yaw, start + t, angle, 120.0f, 7.2f, 0U);
    }
}
void test_run(void)
{
    uint32_t t, samples;
    test_stage = 1U;
    PitchStep_Init(0); PitchStep_Sample(0, 0U, 0.0f, 0.0f, 0.0f, 0U);
    PitchStep_Abort(0, 0U, PITCH_STEP_STOP, 0U); PitchStep_Command(0, 0U, 0.0f);
    CHECK(!PitchStep_Start(0, 0U, 0.0f, 30.0f));
    PitchStep_Init(&pitch); CHECK(pitch.phase == YAW_STEP_IDLE && pitch.response_ms == PITCH_STEP_UNREACHED);
    CHECK(!PitchStep_Start(&pitch, 0U, 0.0f, 50.01f));
    test_stage = 2U;
    ramp(1000U, 270U, 30.0f);
    CHECK(pitch.phase == PITCH_STEP_DONE && pitch.pass && pitch.sse_valid && !yaw.pass);
    CHECK(pitch.response_ms == 270U && pitch.settle_ms == 270U && yaw.response_ms == 270U);
    CHECK(pitch.ss_duration_ms == 1000U && pitch.samples == 500U && pitch.max_gap_ms == 10U);
    CHECK(pitch.t10_ms == 30U && pitch.t90_ms == 250U && pitch.overshoot_deg == 0.0f);
    samples = pitch.samples; PitchStep_Abort(&pitch, 6100U, PITCH_STEP_FAULT, 1U);
    PitchStep_Sample(&pitch, 6110U, 40.0f, 0.0f, 0.0f, 0U);
    CHECK(pitch.samples == samples && pitch.pass);
    ramp(0xFFFFFFC0UL, 300U, -30.0f); CHECK(pitch.pass && !yaw.pass && pitch.response_ms == 300U);
    ramp(0U, 310U, 30.0f); CHECK(!pitch.pass && pitch.sse_valid && pitch.response_ms == 310U);
    test_stage = 3U;
    PitchStep_Init(&pitch); CHECK(PitchStep_Start(&pitch, 100U, 0.0f, 30.0f));
    PitchStep_Sample(&pitch, 100U, 30.0f, 0.0f, 0.0f, 0U);
    PitchStep_Sample(&pitch, 99U, 30.0f, 0.0f, 0.0f, 0U); CHECK(!pitch.samples);
    for (t = 10U; t <= 5000U; t += 10U) {
        float angle = t == 150U || t >= 2000U ? 30.0f : 29.5f;
        PitchStep_Sample(&pitch, 100U + t, angle, 0.0f, 0.5f, 0U);
    }
    CHECK(pitch.response_ms == 150U && pitch.settle_ms == 2000U && pitch.sse_valid && !pitch.pass);
    test_stage = 4U;
    PitchStep_Init(&pitch); CHECK(PitchStep_Start(&pitch, 0U, 0.0f, 30.0f));
    for (t = 10U; t <= 5000U; t += 10U) {
        PitchStep_Sample(&pitch, t, t == 4000U ? 29.7f : 30.0f, 0.0f, 0.5f, 0U);
    }
    CHECK(!pitch.sse_valid && !pitch.pass && pitch.settle_ms == 4010U);
    PitchStep_Init(&pitch); CHECK(PitchStep_Start(&pitch, 0U, 0.0f, 30.0f));
    for (t = 10U; t <= 5000U; t += 10U) {
        PitchStep_Sample(&pitch, t, t == 200U ? 30.3f : 30.0f, 0.0f, 0.5f, 0U);
    }
    CHECK(pitch.sse_valid && !pitch.pass && pitch.overshoot_deg > 0.29f);
    test_stage = 5U;
    PitchStep_Init(&pitch); CHECK(PitchStep_Start(&pitch, 100U, 0.0f, 30.0f));
    PitchStep_Command(&pitch, 102U, 7.2f); CHECK(pitch.command_samples == 1U && pitch.first_command_nm == 7.2f);
    PitchStep_Abort(&pitch, 104U, YAW_STEP_PID_CHANGE, 0U);
    CHECK(pitch.phase == PITCH_STEP_ABORTED && pitch.reason == YAW_STEP_PID_CHANGE && !pitch.sse_valid && !pitch.pass);
    CHECK(PitchStep_Start(&pitch, 200U, 0.0f, 30.0f));
    PitchStep_Sample(&pitch, 300U, 30.0f, 0.0f, 0.0f, 0U);
    CHECK(pitch.phase == PITCH_STEP_ABORTED && pitch.reason == YAW_STEP_GAP && !pitch.pass);
    test_stage = 6U;
    PitchStep_Init(&pitch); CHECK(PitchStep_Start(&pitch, 0U, 0.0f, 30.0f));
    for (t = 10U; t <= 5000U; t += 10U) {
        PitchStep_Sample(&pitch, t, t < 300U ? 0.0f : 29.7f, 0.0f, 0.1f, 0U);
    }
    CHECK(pitch.response_ms == PITCH_STEP_UNREACHED && pitch.settle_ms == PITCH_STEP_UNREACHED && !pitch.pass);
    ramp(0U, 300U, 30.0f); CHECK(pitch.pass && !yaw.pass && pitch.settle_ms == 300U);
    ramp(0U, 310U, 30.0f); CHECK(!pitch.pass && pitch.settle_ms == 310U);
    test_done = 1U;
}
