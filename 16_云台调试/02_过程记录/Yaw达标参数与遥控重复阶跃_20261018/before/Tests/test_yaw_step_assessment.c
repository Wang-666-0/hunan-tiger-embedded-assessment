#define test_run old_step_fixture_not_run
#include "test_yaw_step.c"
#undef test_run

void test_run(void)
{
    uint32_t direction_index, t, start;
    float direction;
    test_stage = 1U;
    for (direction_index = 0; direction_index < 2; ++direction_index) {
        direction = direction_index ? -1.0f : 1.0f;
        start = direction_index ? 0xFFFFFFC0UL : 1000U;
        YawStep_Init(&step); CHECK(YawStep_Start(&step, start, 0, direction * 30));
        for (t = 10; t <= 5000; t += 10) {
            float angle = t < 150 ? 0 : (t < 200 ? 30.7f : (t < 230 ? 29.4f : 29.7f));
            YawStep_Sample(&step, start + t, direction * angle, 0, 0.01f, 0);
        }
        CHECK(step.phase == YAW_STEP_DONE && step.response05_ms == 230 && step.settle05_ms == 230 && step.pass05);
        CHECK(!step.pass && !step.sse_valid && step.overshoot_deg > 0.6f);
    }
    test_stage = 2U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 0, 0, 30));
    for (t = 10; t <= 5000; t += 10) {
        float angle = t < 250 ? 0 : 29.5f;
        YawStep_Sample(&step, t, angle, 0, 0.01f, 0);
    }
    CHECK(step.settle05_ms == 250 && step.pass05 && !step.pass);
    test_stage = 3U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 0, 0, 30));
    for (t = 10; t <= 5000; t += 10) {
        float angle = t < 200 ? 0 : (t == 1000 ? 29.49f : 30.0f);
        YawStep_Sample(&step, t, angle, 0, 0.01f, 0);
    }
    CHECK(step.response05_ms == 200 && step.settle05_ms == 1010 && !step.pass05 && step.pass);
    test_stage = 4U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 0, 0, 30));
    YawStep_Sample(&step, 10, 30, 0, 0.01f, 0);
    YawStep_Abort(&step, 12, YAW_STEP_FF_CHANGE, 0);
    CHECK(!step.pass05 && step.settle05_ms == YAW_STEP_UNREACHED);
    test_done = 1U;
}
