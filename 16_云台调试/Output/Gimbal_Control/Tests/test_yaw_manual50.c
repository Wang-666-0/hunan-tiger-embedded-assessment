#define YAW_CASCADE_CONSOLE_TEST 1
#include "test_yaw_console_backend.c"
#include <math.h>
#include <stdio.h>

/* 注入跟随目标的反馈只验证遥控接口/边界，不代表实机响应性能。 */
static void ManualRun(uint32_t *clock_ms, uint32_t duration_ms, uint16_t channel)
{
    uint32_t elapsed;
    receiver.channels[0U] = channel;
    for (elapsed = 2U; elapsed <= duration_ms; elapsed += 2U) {
        *clock_ms += 2U;
        if (*clock_ms % 10U == 0U) { FeedDegrees(*clock_ms, state.controller.target_deg, 0); }
        Control(*clock_ms, 1U);
        CHECK(state.active && !state.controller.faults && permission);
        CHECK(state.controller.target_deg >= -150.0f && state.controller.target_deg <= 150.0f);
    }
}
static void FeedAt(uint32_t timestamp, float degrees, uint8_t status)
{
    CanFrame frame = Feedback(timestamp, status, 25U, 30U);
    uint32_t position = (uint32_t)(32767.0f + degrees * 65535.0f / 360.0f + 0.5f);
    frame.data[1] = (uint8_t)(position >> 8); frame.data[2] = (uint8_t)position;
    now_ms = timestamp; CHECK(YawBackend_OnFrame(&frame)); Read();
}
static void WithoutRemote(uint32_t timestamp)
{
    SafetyInput input;
    now_ms = timestamp; Read(); memset(&input, 0, sizeof(input));
    memcpy(input.remote.channels, receiver.channels, sizeof(input.remote.channels));
    input.remote.has_frame = receiver.has_frame;
    input.remote.received_ms = receiver.received_ms; input.remote.sequence = receiver.valid_frames;
    input.now_ms = timestamp; input.stop_sequence = stop_sequence; input.can_epoch = bus.fault_epoch;
    input.backend_ready = state.requested;
    input.feedback_ready = state.online && !state.stop_pending && !state.controller.faults;
    input.actuator_fault = state.requested && state.controller.faults != 0U;
    AppLogic_Step(&application.logic, &input, 0U);
    request = application.logic.targets.request;
    YawBackend_Apply(&request, application.logic.safety.output_allowed, diagnostic_mode);
    Read();
}
void test_run(void)
{
    uint32_t clock_ms, t, enabled;
    float held, before;
    YawStep step;
    char default_pid[128];
    test_stage = 1U;
    CHECK(!0U);
    (void)snprintf(default_pid, sizeof(default_pid), "|POS|%.4f,%.4f,%.4f|VEL|%.6f,%.6f,%.6f",
        (double)APP_YAW_POSITION_KP, (double)APP_YAW_POSITION_KI, (double)APP_YAW_POSITION_KD,
        (double)APP_YAW_SPEED_KP, (double)APP_YAW_SPEED_KI, (double)APP_YAW_SPEED_KD);
    CHECK(YAW_CONTROLLER_TARGET_LIMIT_DEG == 150 && YAW_CONTROLLER_HARD_LIMIT_DEG == 160);
    CHECK(YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS == 90 && APP_STEP_DEGREES == 30);
    ConsoleZeroActivate(1000U); clock_ms = 1066U;
    Command(clock_ms, "YAW STATUS", "GL60II_FIXED_PID_V2Q");
    CHECK(strstr(response, "|TARGET_LIMIT_DEG|150.0|HARD_LIMIT_DEG|160.0|") != 0);
    Command(clock_ms, "YAW PID", default_pid);
    before = state.controller.target_deg;
    ManualRun(&clock_ms, 100U, 1765U); /* 半杆精确45deg/s，100ms目标前进4.5度。 */
    CHECK(request.yaw_rate_dps == 45 && state.controller.target_deg - before > 4.499f &&
        state.controller.target_deg - before < 4.501f);
    CHECK(!request.friction_requested && !request.shot_event && !request.step_event);

    test_stage = 2U;
    held = state.controller.target_deg;
    ManualRun(&clock_ms, 100U, 1500U);
    ManualRun(&clock_ms, 100U, 1470U); ManualRun(&clock_ms, 100U, 1530U);
    CHECK(state.controller.target_deg == held && request.yaw_rate_dps == 0);
    receiver.channels[1U] = 2000U;
    ManualRun(&clock_ms, 100U, 1500U);
    CHECK(state.controller.target_deg == held); /* 右杆上下不串入Yaw。 */
    receiver.channels[1U] = 1500U;

    test_stage = 3U;
    ManualRun(&clock_ms, 2000U, 2000U);
    CHECK(state.controller.target_deg == 150 && request.yaw_rate_dps == 90);
    ManualRun(&clock_ms, 100U, 1500U); CHECK(state.controller.target_deg == 150);
    ManualRun(&clock_ms, 3800U, 1000U);
    CHECK(state.controller.target_deg == -150 && request.yaw_rate_dps == -90);
    ManualRun(&clock_ms, 100U, 1500U); CHECK(state.controller.target_deg == -150);
    CHECK(!YawBackend_StartStep(50) && !YawBackend_StartStep(-50));
    /* CH5 captures the measured pose, then ignores manual stick rates. */
    receiver.channels[0U] = 2000U;
    clock_ms += 2U; Control(clock_ms, 2U);
    CHECK(request.gimbal_mode == GIMBAL_TEST && request.yaw_rate_dps == 0 && state.active && state.step_hold);
    CHECK(fabsf(state.controller.target_deg - state.controller.angle_deg) <= YAW_STEP_BAND_DEG);
    receiver.channels[0U] = 1500U;
    clock_ms += 2U; Control(clock_ms, 1U);

    test_stage = 4U;
    enabled = enable_calls; receiver.channels[7U] = 1000U;
    clock_ms += 2U; Control(clock_ms, 1U);
    CHECK(!state.active && !permission && !application.logic.safety.operator_armed);
    receiver.channels[7U] = 1500U;
    clock_ms += 2U; FeedAt(clock_ms, -150.0f, 0U); Control(clock_ms, 1U);
    CHECK(!state.active && enable_calls == enabled); /* 仅释放安全锁不会自启。 */

    test_stage = 5U;
    /* 端点微小静差仍能按OFF->ON重启，保留原零点并裁目标，而非追赶0。 */
    ConsoleZeroActivate(30000U); clock_ms = 30066U;
    ManualRun(&clock_ms, 1900U, 1000U);
    receiver.channels[0U] = 1500U;
    clock_ms += 2U; FeedAt(clock_ms, -150.1f, 1U); Control(clock_ms, 1U);
    clock_ms += 2U; Control(clock_ms, 0U);
    FeedAt(clock_ms + 2U, -150.1f, 0U); Control(clock_ms + 2U, 0U);
    FeedAt(clock_ms + 24U, -150.1f, 0U); Control(clock_ms + 24U, 0U);
    Command(clock_ms + 26U, "YAW ON", "Y|OK|PREPARE");
    Control(clock_ms + 30U, 0U);
    FeedAt(clock_ms + 32U, -150.1f, 0U); Control(clock_ms + 32U, 0U);
    FeedAt(clock_ms + 54U, -150.1f, 0U); Control(clock_ms + 54U, 0U);
    Control(clock_ms + 56U, 1U);
    FeedAt(clock_ms + 58U, -150.1f, 1U); Control(clock_ms + 58U, 1U);
    for (t = 60U; t <= 66U; t += 2U) { Control(clock_ms + t, 1U); }
    CHECK(state.active && state.zero_set && !state.controller.faults && state.controller.target_deg == -150);
    CHECK(state.controller.angle_deg < -150 && state.controller.angle_deg > -150.2f);

    test_stage = 6U;
    ConsoleZeroActivate(40000U); enabled = enable_calls;
    for (t = 40068U; t <= 40166U; t += 2U) {
        if (t % 10U == 0U) { FeedDegrees(t, state.controller.target_deg, 0); }
        WithoutRemote(t);
        if (t < 40166U) { CHECK(state.active && permission); }
    }
    CHECK(!state.active && !permission && application.logic.safety.reason == SAFETY_REMOTE);
    Control(40168U, 1U); CHECK(!state.active && enable_calls == enabled);
    YawBackend_GetStep(&step); CHECK(step.phase == YAW_STEP_IDLE);

    test_stage = 7U;
    ConsoleReset(50000U); Read();
    CHECK(!state.active && !state.requested && !state.zero_set && !enable_calls && !torque_calls);
    Command(50000U, "YAW PID", default_pid);
    test_done = 1U;
}
