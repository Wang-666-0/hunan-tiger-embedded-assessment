#include "yaw_step.h"
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static YawStep step;
static float absolute(float x) { return x < 0.0f ? -x : x; }
static void ramp(uint32_t start, uint32_t duration, float target, float overshoot)
{
    uint32_t t;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, start, 0.0f, target));
    for (t = 10U; t <= 5000U; t += 10U) {
        float angle = t < duration ? target * (float)t / (float)duration : target;
        if (t == duration && overshoot != 0.0f) { angle += overshoot; }
        YawStep_Sample(&step, start + t, angle, t < duration ? target * 1000.0f / (float)duration : 0.0f,
            0.1f, (uint8_t)(t < duration));
    }
}
void test_run(void)
{
    uint32_t t, id, samples;
    union { uint32_t bits; float value; } bad;
    test_stage = 1U;
    YawStep_Init(0); YawStep_Abort(0, 0U, YAW_STEP_STOP, 0U);
    YawStep_Sample(0, 1U, 0.0f, 0.0f, 0.0f, 0U);
    CHECK(!YawStep_Start(0, 0U, 0.0f, 30.0f));
    YawStep_Init(&step); CHECK(step.phase == YAW_STEP_IDLE && step.response_ms == YAW_STEP_UNREACHED);
    bad.bits = 0x7FC00000U;
    CHECK(!YawStep_Start(&step, 0U, bad.value, 30.0f));
    CHECK(!YawStep_Start(&step, 0U, 0.0f, 0.5f));
    CHECK(!YawStep_Start(&step, 0U, 0.0f, 181.0f));
    CHECK(YawStep_Start(&step, 100U, 0.0f, 30.0f));
    CHECK(!YawStep_Start(&step, 101U, 0.0f, -30.0f));
    YawStep_Sample(&step, 100U, 999.0f, 0.0f, 0.0f, 0U);
    YawStep_Sample(&step, 99U, 999.0f, 0.0f, 0.0f, 0U);
    CHECK(step.samples == 0U && step.phase == YAW_STEP_RUNNING);
    YawStep_Abort(&step, 110U, YAW_STEP_STOP, 0U); id = step.id;
    CHECK(YawStep_Start(&step, 120U, 0.0f, 30.0f) && step.id == id + 1U);

    test_stage = 2U;
    ramp(1000U, 200U, 30.0f, 0.0f);
    CHECK(step.phase == YAW_STEP_DONE && step.pending && step.elapsed_ms == 5000U);
    CHECK(step.t10_ms == 20U && step.t90_ms == 180U && step.response_ms == 200U && step.settle_ms == 200U);
    CHECK(step.sse_valid && step.pass && absolute(step.steady_error_deg) < 0.00001f);
    CHECK(step.ss_duration_ms == 1000U && step.samples == 500U && step.max_gap_ms == 10U);
    CHECK(step.overshoot_deg == 0.0f && step.limited_samples == 19U);
    CHECK(absolute(step.peak_velocity_dps - 150.0f) < 0.001f);
    samples = step.samples; YawStep_Abort(&step, 6100U, YAW_STEP_STOP, 16U);
    YawStep_Sample(&step, 6200U, 50.0f, 1000.0f, 1.0f, 1U);
    CHECK(step.phase == YAW_STEP_DONE && step.pass && step.samples == samples);

    test_stage = 3U;
    ramp(0xFFFFFFC0UL, 200U, -30.0f, -0.3f);
    CHECK(step.phase == YAW_STEP_DONE && !step.pass && step.sse_valid);
    CHECK(step.overshoot_deg > 0.299f && step.overshoot_deg < 0.301f);
    CHECK(step.overshoot_pct > 0.999f && step.overshoot_pct < 1.001f);
    CHECK(step.response_ms == 210U && step.steady_error_deg == 0.0f);
    ramp(100U, 1000U, 30.0f, 0.0f);
    CHECK(step.sse_valid && !step.pass && step.response_ms == 1000U);

    test_stage = 4U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 100U, 0.0f, 30.0f));
    YawStep_Sample(&step, 110U, 1.0f, 30.0f, 0.05f, 0U);
    YawStep_Abort(&step, 120U, YAW_STEP_PID_CHANGE, 0U);
    CHECK(step.phase == YAW_STEP_ABORTED && step.reason == YAW_STEP_PID_CHANGE && !step.sse_valid && !step.pass);
    YawStep_Abort(&step, 130U, YAW_STEP_FAULT, 16U);
    CHECK(step.reason == YAW_STEP_PID_CHANGE && step.faults == 0U && step.elapsed_ms == 20U);
    CHECK(YawStep_Start(&step, 200U, 0.0f, 30.0f));
    YawStep_Sample(&step, 300U, 3.0f, 30.0f, 0.05f, 0U);
    CHECK(step.phase == YAW_STEP_ABORTED && step.reason == YAW_STEP_GAP && !step.pass);
    CHECK(YawStep_Start(&step, 400U, 0.0f, 30.0f));
    YawStep_Sample(&step, 410U, bad.value, 0.0f, 0.0f, 0U);
    CHECK(step.phase == YAW_STEP_ABORTED && step.reason == YAW_STEP_DATA);

    test_stage = 5U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 0U, 0.0f, 30.0f));
    for (t = 10U; t <= 5000U; t += 10U) { YawStep_Sample(&step, t, 29.0f, 0.0f, 0.01f, 0U); }
    CHECK(step.phase == YAW_STEP_DONE && !step.sse_valid && !step.pass);
    CHECK(step.response_ms == YAW_STEP_UNREACHED && step.settle_ms == YAW_STEP_UNREACHED);
    CHECK(absolute(step.steady_error_deg - 1.0f) < 0.0001f);

    test_stage = 6U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 0U, 0.0f, 30.0f));
    for (t = 37U; t <= 5032U; t += 37U) {
        float angle = t < 222U ? 30.0f * (float)t / 222.0f :
            (t > 4000U ? 30.0f + (float)(t - 4000U) * 0.0001f : 30.0f);
        YawStep_Sample(&step, t, angle, 0.0f, 0.01f, 0U);
    }
    CHECK(step.phase == YAW_STEP_DONE && step.elapsed_ms == 5000U && step.ss_duration_ms == 1000U);
    CHECK(step.sse_valid && absolute(step.steady_error_deg + 0.05f) < 0.0001f);
    CHECK(step.max_gap_ms == 37U && step.overshoot_deg > 0.099f && step.overshoot_deg < 0.101f);

    test_stage = 7U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 0U, 0.0f, 30.0f));
    for (t = 10U; t <= 5000U; t += 10U) {
        float angle = t < 100U ? 0.0f : 30.0f;
        if (t >= 150U && t < 200U) { angle = 29.0f; }
        if (t >= 4800U && t < 4990U) { angle = 29.5f; }
        YawStep_Sample(&step, t, angle, 0.0f, 0.01f, 0U);
    }
    CHECK(step.response_ms == 100U && step.settle_ms == YAW_STEP_UNREACHED);
    CHECK(!step.sse_valid && !step.pass);
    test_stage = 8U;
    YawStep_Init(&step); CHECK(YawStep_Start(&step, 100U, 0.0f, 30.0f));
    YawStep_Command(0, 101U, 0.1f); YawStep_Command(&step, 99U, 0.1f);
    CHECK(step.command_samples == 0U);
    YawStep_Command(&step, 101U, 0.1f); YawStep_Command(&step, 102U, -0.05f);
    CHECK(step.command_samples == 2U && step.first_command_nm == 0.1f && step.peak_torque_nm == 0.1f);
    YawStep_Abort(&step, 110U, YAW_STEP_FAULT, 16U);
    CHECK(step.samples == 0U && step.command_samples == 2U && step.peak_torque_nm == 0.1f);
    YawStep_Command(&step, 111U, 1.0f); CHECK(step.command_samples == 2U);
    CHECK(YawStep_Start(&step, 0xFFFFFFFCUL, 0.0f, -30.0f));
    YawStep_Command(&step, 1U, -0.08f);
    CHECK(step.command_samples == 1U && step.first_command_nm == -0.08f);
    YawStep_Command(&step, 5000U, 1.0f); CHECK(step.command_samples == 1U);

    /* 首次只在带内一包也立即记时；后续离带和重新到位不改写。
     * 正反向及计时器回卷使用同一定义，忽略重复/乱序反馈。 */
    test_stage = 9U;
    for (id = 0U; id < 2U; ++id) {
        uint32_t start = id ? 0xFFFFFFC0UL : 1000U;
        float direction = id ? -1.0f : 1.0f;
        YawStep_Init(&step); CHECK(YawStep_Start(&step, start, 0.0f, direction * 30.0f));
        for (t = 10U; t <= 140U; t += 10U) {
            YawStep_Sample(&step, start + t, direction * 29.5f, 0.0f, 0.01f, 0U);
        }
        CHECK(step.response_ms == YAW_STEP_UNREACHED);
        YawStep_Sample(&step, start + 140U, direction * 30.0f, 0.0f, 0.01f, 0U);
        YawStep_Sample(&step, start + 139U, direction * 30.0f, 0.0f, 0.01f, 0U);
        CHECK(step.response_ms == YAW_STEP_UNREACHED);
        YawStep_Sample(&step, start + 150U, direction * 29.96f, 0.0f, 0.01f, 0U);
        CHECK(step.phase == YAW_STEP_RUNNING && step.response_ms == 150U);
        CHECK(step.settle_ms == YAW_STEP_UNREACHED && !step.sse_valid && !step.pass);
        for (t = 160U; t <= 5000U; t += 10U) {
            YawStep_Sample(&step, start + t, direction * (t < 500U ? 29.5f : 30.0f), 0.0f, 0.01f, 0U);
            CHECK(step.response_ms == 150U);
        }
        CHECK(step.phase == YAW_STEP_DONE && step.settle_ms == 500U);
        CHECK(step.sse_valid && step.pass && step.steady_error_deg == 0.0f);
    }
    test_done = 1U;
}
