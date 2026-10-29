#define test_run old_step_fixture_not_run
#include "test_yaw_step.c"
#undef test_run

static void window(uint32_t start, float direction, uint32_t entry, uint32_t settled,
                   float peak, float tail_error, uint32_t later_exit)
{
    uint32_t t;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, start, 0, direction * 30));
    for (t = 10; t <= 5000; t += 10) {
        float angle = t < entry ? 0 : (t == entry ? 30 + peak :
            (t < settled ? 29.6f : 30 - tail_error));
        if (later_exit && t == later_exit) { angle = 29.7f; }
        YawStep_Sample(&step, start + t, direction * angle, 0, 0.01f, 0);
    }
    CHECK(step.phase == YAW_STEP_DONE && step.ss_duration_ms == 1000);
}

void test_run(void)
{
    uint32_t d, start;
    float direction;
    CHECK(YAW_STEP_BAND_DEG == 0.2f && 250U == 250);
    CHECK(!YawStep_Passed(0, 250));
    for (d = 0; d < 2; ++d) {
        direction = d ? -1.0f : 1.0f;
        start = d ? 0xFFFFFFC0UL : 1000U;
        test_stage = 1;
        /* Screenshot-like fast settling, but 0.410608-degree overshoot fails. */
        window(start, direction, 200, 200, 0.410608f, 0, 0);
        CHECK(step.response_ms == 210 && step.settle_ms == 210 && step.sse_valid);
        CHECK(step.overshoot_deg > 0.4105f && !step.pass);
        test_stage = 2;
        /* First entry is fast; a later rebound must determine settling time. */
        window(start, direction, 200, 500, 0, 0, 0);
        CHECK(step.response_ms == 200 && step.settle_ms == 500 && !step.pass && step.sse_valid);
        window(start, direction, 200, 200, 0, 0, 1000);
        CHECK(step.response_ms == 200 && step.settle_ms == 1010 && !step.pass);
        test_stage = 3;
        window(start, direction, 250, 250, 0, 0, 0);
        CHECK(step.settle_ms == 250 && step.pass);
        window(start, direction, 260, 260, 0, 0, 0);
        CHECK(step.settle_ms == 260 && !step.pass);
        CHECK(YawStep_Passed(&step, 300));
        test_stage = 4;
        /* Inclusive 0.2 boundaries tolerate float roundoff, not 0.2001. */
        window(start, direction, 200, 200, 0.2f, 0, 0);
        CHECK(step.overshoot_deg > 0.1999f && step.pass);
        window(start, direction, 200, 200, 0.2001f, 0, 0);
        CHECK(step.overshoot_deg > 0.20009f && !step.pass);
        window(start, direction, 200, 200, 0, 0.2f, 0);
        CHECK(absolute(step.steady_error_deg) > 0.1999f && step.sse_valid && step.pass);
        window(start, direction, 200, 200, 0, 0.2001f, 0);
        CHECK(absolute(step.steady_error_deg) > 0.20009f && !step.sse_valid && !step.pass);
        test_stage = 5;
        window(start, direction, 200, 200, 0, 0, 4000);
        CHECK(absolute(step.steady_error_deg) < 0.01f && step.ss_max_error > 0.29f);
        CHECK(!step.sse_valid && !step.pass); /* Mean cannot hide tail excursions. */
        window(start, direction, 200, 200, 0, 0, 0);
        CHECK(step.pass);
        step.faults = 16; CHECK(!YawStep_Passed(&step, 250));
    }
    test_stage = 6;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 0, 0, 30));
    YawStep_Sample(&step, 10, 30, 0, 0.01f, 0);
    CHECK(!step.pass && !YawStep_Passed(&step, 250));
    YawStep_Abort(&step, 12, YAW_STEP_FF_CHANGE, 0);
    CHECK(!step.pass && step.settle_ms == YAW_STEP_UNREACHED);
    test_done = 1;
}
