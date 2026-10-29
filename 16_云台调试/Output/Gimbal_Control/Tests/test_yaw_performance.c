/* 默认考核档：复用实际后端的传输夹具，历史受限档另组执行。 */
#define test_run guarded_backend_fixture_not_run
#include "test_yaw_backend.c"
#undef test_run

static void PerformanceFeed(uint32_t timestamp, float degrees, float velocity)
{
    CanFrame frame = Feedback(timestamp, 1U, 25U, 30U);
    uint32_t position = (uint32_t)(32767.0f + degrees * 65535.0f / 360.0f + 0.5f);
    uint32_t speed = (uint32_t)((velocity * 3.141593f / 180.0f + 200.0f) * 4095.0f / 400.0f + 0.5f);
    frame.data[1] = (uint8_t)(position >> 8); frame.data[2] = (uint8_t)position;
    frame.data[3] = (uint8_t)(speed >> 4); frame.data[4] = (uint8_t)(((speed & 15U) << 4) | 7U);
    now_ms = timestamp; CHECK(YawBackend_OnFrame(&frame)); Read();
}

static void FastTrace(uint32_t base, float target)
{
    uint32_t t;
    YawStep result;
    Activate(base); now_ms = base + 42U;
    CHECK(YawBackend_CaptureReference() == 0U); /* 活动时不能改软件零点。 */
    /* 人工零点由真实的安全OFF流程取得，而非修改状态字段。 */
    CHECK(YawBackend_Request(0U)); Apply(base + 44U, 0U);
    Feed(base + 46U, 0U); Feed(base + 68U, 0U); Apply(base + 68U, 0U);
    CHECK(YawBackend_CaptureReference()); CHECK(YawBackend_Request(1U));
    Apply(base + 70U, 0U); Feed(base + 72U, 0U); Feed(base + 94U, 0U); Apply(base + 94U, 0U);
    Apply(base + 96U, 1U); Feed(base + 98U, 1U); Apply(base + 98U, 1U);
    Apply(base + 100U, 1U); Apply(base + 102U, 1U);
    now_ms = base + 102U; CHECK(YawBackend_StartStep(target));
    for (t = 2U; t <= 5000U; t += 2U) {
        if (t % 10U == 0U) {
            PerformanceFeed(base + 102U + t, t < 200U ? target * (float)t / 200.0f : target,
                t < 200U ? target * 5.0f : 0.0f);
        }
        Apply(base + 102U + t, 1U);
        CHECK(state.active && !state.controller.faults && !state.controller.cascade.velocity_limited);
        CHECK(last_torque <= GL60_TORQUE_MAX_NM && last_torque >= -GL60_TORQUE_MAX_NM);
    }
    YawBackend_GetStep(&result);
    CHECK(result.phase == YAW_STEP_DONE && result.sse_valid && result.pass);
    CHECK(result.peak_velocity_dps > 120.0f && result.response_ms == 200U);
    CHECK(result.command_samples > 400U && result.peak_torque_nm > 0.09f);
    CHECK(state.active && state.step_hold && state.controller.target_deg == target);
    Apply(base + 5104U, 0U); CHECK(!state.active && !permission);
}

void test_run(void)
{
    YawStep result;
    CanFrame bad;
    test_stage = 1U;
    CHECK(0U == 0U);
    FastTrace(1000U, 30.0f);
    test_stage = 2U;
    FastTrace(8000U, -30.0f);

    test_stage = 3U;
    Reset(15000U); Feed(15002U, 0U); Feed(15024U, 0U);
    CHECK(YawBackend_CaptureReference()); CHECK(YawBackend_Request(1U));
    Apply(15026U, 0U); Feed(15028U, 0U); Feed(15050U, 0U); Apply(15050U, 0U);
    Apply(15058U, 1U); Feed(15060U, 1U); Apply(15060U, 1U); Apply(15062U, 1U);
    now_ms = 15062U; CHECK(YawBackend_StartStep(30.0f));
    Apply(15064U, 1U); Apply(15066U, 1U); Apply(15068U, 1U);
    PerformanceFeed(15072U, 0.2f, 64.36f); Apply(15072U, 1U);
    YawBackend_GetStep(&result);
    CHECK(state.active && result.phase == YAW_STEP_RUNNING && !state.controller.faults);
    CHECK(result.peak_velocity_dps > 60.0f && !state.controller.cascade.velocity_limited);
    /* 后续温度故障发生前已经提交的0.10请求仍须留在报告，不能变成0。 */
    bad = Feedback(15082U, 1U, 60U, 30U); now_ms = 15082U;
    CHECK(YawBackend_OnFrame(&bad)); Read(); YawBackend_GetStep(&result);
    CHECK(!state.active && result.phase == YAW_STEP_ABORTED && result.reason == YAW_STEP_FAULT);
    CHECK(result.command_samples > 0U && result.peak_torque_nm > 0.09f && result.first_command_nm > 0.09f);
    CHECK((result.faults & YAW_CONTROLLER_FAULT_TEMPERATURE) && !(result.faults & YAW_CONTROLLER_FAULT_SPEED));

    test_stage = 4U;
    Activate(17000U);
    PerformanceFeed(17052U, 10.0f, 300.0f); Apply(17052U, 1U);
    CHECK(state.active && !state.controller.faults);
    PerformanceFeed(17062U, 20.0f, 300.0f); Apply(17062U, 1U);
    PerformanceFeed(17072U, 30.0f, 300.0f); Apply(17072U, 1U);
    PerformanceFeed(17082U, 39.0f, 300.0f); Apply(17082U, 1U);
    PerformanceFeed(17092U, 49.0f, 300.0f); Apply(17092U, 1U);
    CHECK(state.active && !state.controller.faults);
    PerformanceFeed(17102U, 59.0f, 300.0f); Apply(17102U, 1U);
    CHECK(state.active && !state.controller.faults);
    {
        uint32_t sample_ms = 17112U;
        float angle;
        for (angle = 61.0f; angle <= YAW_CONTROLLER_HARD_LIMIT_DEG + 1.0f; angle += 10.0f) {
            PerformanceFeed(sample_ms, angle, 300.0f); Apply(sample_ms, 1U);
            if (angle <= YAW_CONTROLLER_HARD_LIMIT_DEG) { CHECK(state.active && !state.controller.faults); }
            sample_ms += 10U;
        }
    }
    CHECK(!state.active && (state.controller.faults & YAW_CONTROLLER_FAULT_LIMIT));
    CHECK(!(state.controller.faults & YAW_CONTROLLER_FAULT_SPEED));
    Activate(18000U); Apply(18134U, 1U);
    CHECK(!state.active && (state.controller.faults & YAW_CONTROLLER_FAULT_FEEDBACK));
    Activate(19000U); bus.fault_epoch = 1U; PerformanceFeed(19052U, 0.0f, 0.0f);
    CHECK(!state.active && (state.controller.faults & YAW_CONTROLLER_FAULT_CAN));
    test_done = 1U;
}
