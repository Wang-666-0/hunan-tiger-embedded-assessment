/* Reuse the backend transport substitute, fixtures and assertion counters;
 * this group links the real console, backend, controller and remote logic. */
#define test_run backend_standalone_test_run
#include "test_yaw_backend.c"
#undef test_run
#include "yaw_console.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"

static RemoteInput_State receiver;
static AppState application;
static uint32_t stop_sequence, stop_calls;
static uint8_t diagnostic_mode;
static char response[512];
void CanBus_GetDiagnostics(CanBusDiagnostics *out)
{
    if (out) { memset(out, 0, sizeof(*out)); out->state = bus; out->pending[0] = pending; }
}

void RemoteInput_GetState(RemoteInput_State *out) { if (out) { *out = receiver; } }
void AppTasks_GetState(AppState *out) { if (out) { *out = application; } }
uint8_t AppConsole_IsDiagnosticMode(void) { return diagnostic_mode; }
void AppTasks_RequestStop(void)
{
    ++stop_sequence; ++stop_calls;
    YawBackend_Stop(); /* actual effect which formerly prevented fault recovery */
}

static void ConsoleReset(uint32_t timestamp)
{
    uint32_t i;
    YawConsole_CancelPreparation();
    Reset(timestamp);
    memset(&receiver, 0, sizeof(receiver)); memset(&application, 0, sizeof(application));
    for (i = 0U; i < 14U; ++i) { receiver.channels[i] = RC_CENTER; }
    receiver.channels[RC_ARM_CHANNEL] = 1000U;
    receiver.channels[RC_FRICTION_CHANNEL] = 1000U;
    receiver.has_frame = 1U; receiver.received_ms = timestamp;
    receiver.valid_frames = 1U; stop_sequence = stop_calls = 0U; diagnostic_mode = 0U;
    AppLogic_Init(&application.logic);
}

/* Model the real task's remote protection and Yaw submission order. CH6 stays
 * OFF and no wheel feedback is supplied. Each call receives a new iBUS frame. */
static void Control(uint32_t timestamp, uint8_t mode_on)
{
    SafetyInput input;
    now_ms = timestamp;
    receiver.channels[RC_ARM_CHANNEL] = mode_on == 2U ? 2000U : (mode_on ? 1500U : 1000U);
    receiver.received_ms = timestamp; ++receiver.valid_frames;
    Read(); memset(&input, 0, sizeof(input));
    memcpy(input.remote.channels, receiver.channels, sizeof(input.remote.channels));
    input.remote.has_frame = receiver.has_frame;
    input.remote.received_ms = timestamp; input.remote.sequence = receiver.valid_frames;
    input.now_ms = timestamp; input.stop_sequence = stop_sequence; input.can_epoch = bus.fault_epoch;
    input.backend_ready = state.requested;
    input.feedback_ready = state.online && !state.stop_pending && !state.controller.faults;
    input.actuator_fault = state.requested && state.controller.faults != 0U;
    AppLogic_Step(&application.logic, &input, 0U);
    request = application.logic.targets.request;
    YawBackend_Apply(&request, application.logic.safety.output_allowed, diagnostic_mode);
    if (YawBackend_TakeRearmRequest()) {
        AppTasks_RequestStop();
        application.logic.safety.operator_armed = application.logic.safety.output_allowed = 0U;
    }
    Read();
}

static void Command(uint32_t timestamp, const char *line, const char *expected)
{
    now_ms = timestamp; memset(response, 0, sizeof(response));
    CHECK(YawConsole_Handle(line, response, sizeof(response)) == 1U);
    CHECK(strstr(response, expected) != 0); Read();
}

static void ConsolePrepare(uint32_t timestamp)
{
    ConsoleReset(timestamp); Control(timestamp, 0U);
    Command(timestamp + 2U, "YAW ON", "Y|OK|PREPARE");
    CHECK(state.requested && state.stop_pending && enable_calls == 0U && torque_calls == 0U);
    Control(timestamp + 4U, 0U);
    CHECK(disable_calls == 1U && enable_calls == 0U);
    Feed(timestamp + 6U, 0U); Control(timestamp + 6U, 0U);
    Feed(timestamp + 28U, 0U); Control(timestamp + 28U, 0U);
    CHECK(!state.stop_pending && state.online && !application.logic.safety.operator_armed);
    Control(timestamp + 30U, 1U);
    CHECK(application.logic.safety.operator_armed && state.enabling && !state.active && enable_calls == 1U);
}

static void ConsoleActivate(uint32_t timestamp)
{
    uint32_t t;
    ConsolePrepare(timestamp);
    Feed(timestamp + 32U, 1U); Control(timestamp + 32U, 1U);
    CHECK(state.active && !state.enabling && torque_calls == 0U);
    for (t = timestamp + 34U; t <= timestamp + 40U; t += 2U) { Control(t, 1U); }
    CHECK(state.active && torque_calls == 1U && permission);
}

static void FeedDegrees(uint32_t timestamp, float degrees, float velocity)
{
    CanFrame frame = Feedback(timestamp, 1U, 25U, 30U);
    uint32_t position = (uint32_t)(32767.0f + degrees * 65535.0f / 360.0f + 0.5f);
    uint32_t speed = (uint32_t)((velocity * 3.141593f / 180.0f + 200.0f) * 4095.0f / 400.0f + 0.5f);
    frame.data[1] = (uint8_t)(position >> 8); frame.data[2] = (uint8_t)position;
    frame.data[3] = (uint8_t)(speed >> 4); frame.data[4] = (uint8_t)(((speed & 15U) << 4) | 7U);
    now_ms = timestamp; CHECK(YawBackend_OnFrame(&frame)); Read();
}
static void ConsoleZeroActivate(uint32_t timestamp)
{
    uint32_t t;
    ConsoleReset(timestamp); Control(timestamp, 0U);
    Feed(timestamp + 2U, 0U); Control(timestamp + 2U, 0U);
    Feed(timestamp + 24U, 0U); Control(timestamp + 24U, 0U);
    Command(timestamp + 26U, "YAW ZERO", "Y|OK|SOFTWARE_ZERO"); CHECK(state.zero_set);
    Command(timestamp + 28U, "YAW ON", "Y|OK|PREPARE"); Control(timestamp + 30U, 0U);
    Feed(timestamp + 32U, 0U); Control(timestamp + 32U, 0U);
    Feed(timestamp + 54U, 0U); Control(timestamp + 54U, 0U);
    Control(timestamp + 56U, 1U); Feed(timestamp + 58U, 1U); Control(timestamp + 58U, 1U);
    for (t = timestamp + 60U; t <= timestamp + 66U; t += 2U) { Control(t, 1U); }
    CHECK(state.active && state.controller.active && state.zero_set && !state.controller.faults);
}
static void RemoteStart(uint32_t timestamp)
{
    ConsoleZeroActivate(timestamp);
    Control(timestamp + 68U, 2U); /* CH5测试，先观察CH7中位。 */
    receiver.channels[RC_STEP_YAW_CHANNEL] = 2000U; Control(timestamp + 70U, 2U);
    receiver.channels[RC_STEP_CHANNEL] = 2000U; Control(timestamp + 72U, 2U);
    CHECK(state.step_hold && state.controller.target_deg == 30.0f && request.step_sequence == 1U);
    receiver.channels[RC_STEP_YAW_CHANNEL] = receiver.channels[RC_STEP_CHANNEL] = 1500U;
    Control(timestamp + 74U, 2U);
}

static void RecoverThroughConsole(uint32_t timestamp)
{
    uint32_t before_enables = enable_calls, before_stops;
    uint32_t fault = state.controller.faults;
    CHECK(fault != 0U && !state.active && !permission);
    Control(timestamp, 0U);
    Command(timestamp + 2U, "YAW OFF", "Y|OK|OFF");
    CHECK(!state.requested && state.stop_pending && state.controller.faults != 0U);
    Feed(timestamp + 4U, 0U); Control(timestamp + 4U, 0U);
    Feed(timestamp + 26U, 0U); Control(timestamp + 26U, 0U);
    CHECK(!state.stop_pending && state.online && !application.logic.safety.operator_armed);
    before_stops = stop_calls;
    Command(timestamp + 28U, "YAW ON", "Y|OK|PREPARE");
    CHECK(stop_calls == before_stops + 1U); /* actual STOP precedes Request */
    CHECK(state.requested && state.controller.faults == 0U && state.stop_pending);
    CHECK(!state.active && !state.enabling && enable_calls == before_enables);
    /* Switching CH5 ON immediately cannot outrun the STOP/new-feedback barrier. */
    Control(timestamp + 30U, 1U); Control(timestamp + 32U, 1U);
    CHECK(enable_calls == before_enables && !state.active && state.stop_pending);
    Feed(timestamp + 34U, 0U); Control(timestamp + 34U, 1U);
    CHECK(!state.stop_pending && enable_calls == before_enables && !application.logic.safety.operator_armed);
    /* New healthy status0 is insufficient: operator must issue fresh OFF->ON. */
    Control(timestamp + 36U, 0U); Control(timestamp + 38U, 1U);
    CHECK(application.logic.safety.operator_armed && state.enabling && enable_calls == before_enables + 1U);
    Feed(timestamp + 40U, 1U); Control(timestamp + 40U, 1U);
    CHECK(state.active && !state.enabling);
    Control(timestamp + 42U, 1U); Control(timestamp + 44U, 1U);
    Control(timestamp + 46U, 1U); Control(timestamp + 48U, 1U);
    CHECK(state.active && permission && state.controller.faults == 0U);
}

void test_run(void)
{
    uint32_t t, before;
    CanFrame frame;
    YawStep measurement;
    YawRemoteStepEvent event;
    char step_response[1024];
    test_stage = 1U;
    ConsoleActivate(100U);
    Command(142U, "YAW TARGET 2", "Y|OK|PARAMETER");
    CHECK(state.controller.target_deg == 2.0f);
    Command(144U, "YAW ON", "Y|ERROR|SAFE_OFF_REQUIRED");
    CHECK(state.active && permission); /* rejected ON does not interrupt active axis */
    Control(232U, 1U); /* age100: actual feedback timeout and unified lock */
    CHECK(state.controller.faults & YAW_CONTROLLER_FAULT_FEEDBACK);
    CHECK(!state.active && !permission && state.stop_pending);
    CHECK(state.controller.target_deg != 2.0f && !application.logic.safety.operator_armed);
    RecoverThroughConsole(234U);

    test_stage = 2U;
    ConsoleActivate(1000U); Feed(1042U, 0U); Read();
    CHECK(state.controller.faults & (1UL << 18)); /* unexpected driver disable */
    RecoverThroughConsole(1044U);

    test_stage = 3U;
    ConsolePrepare(2000U);
    for (t = 2032U; t <= 2130U; t += 2U) { Feed(t, 0U); Control(t, 1U); }
    CHECK(state.controller.faults & (1UL << 16)); /* FC never acknowledged */
    CHECK(!state.active && !state.enabling && !permission);
    RecoverThroughConsole(2132U);

    test_stage = 4U;
    ConsoleActivate(3000U); now_ms = 3042U;
    frame = Feedback(now_ms, 1U, 60U, 30U); CHECK(YawBackend_OnFrame(&frame) == 1U); Read();
    CHECK(state.controller.faults & YAW_CONTROLLER_FAULT_TEMPERATURE);
    RecoverThroughConsole(3044U);

    test_stage = 5U;
    ConsoleActivate(4000U); torque_result = HAL_ERROR;
    for (t = 4042U; t <= 4050U; t += 2U) { Control(t, 1U); }
    CHECK(state.controller.faults & (1UL << 17)); /* TX fault cannot be cleared by console */
    Control(4052U, 0U); Command(4054U, "YAW OFF", "Y|OK|OFF");
    Feed(4056U, 0U); Control(4056U, 0U); Feed(4078U, 0U); Control(4078U, 0U);
    before = enable_calls; Command(4080U, "YAW ON", "Y|ERROR|SAFE_OFF_REQUIRED");
    CHECK(state.controller.faults & (1UL << 17));
    CHECK(!state.requested && !permission && enable_calls == before);
    test_stage = 6U;
    ConsoleReset(5000U); Control(5000U, 0U);
    Command(5002U, "YAW ON", "Y|OK|PREPARE"); Control(5004U, 0U);
    Feed(5006U, 13U); Control(5006U, 0U);
    Feed(5028U, 13U); Control(5028U, 0U);
    CHECK(state.online && state.feedback.status == 13U && state.controller.faults == 4U);
    CHECK(state.requested && state.stop_pending && !state.active && enable_calls == 0U);
    Command(5030U, "YAW CLEAR", "Y|ERROR|CLEAR|REMOTE|");
    CHECK(strstr(response, "|LAST_BLOCKS|2|") != 0);
    CHECK(clear_calls == 0U);
    Command(5032U, "YAW OFF", "Y|OK|OFF"); Control(5034U, 0U);
    Feed(5036U, 13U); Control(5036U, 0U); Feed(5058U, 13U); Control(5058U, 0U);
    Command(5060U, "YAW CLEAR", "Y|OK|CLEAR_SENT");
    CHECK(clear_calls == 1U && pending && !state.requested && !state.active && !permission);
    Control(5062U, 0U); Feed(5064U, 0U); Control(5064U, 0U);
    CHECK(state.stop_pending && enable_calls == 0U && torque_calls == 0U);
    pending = 0U; Control(5072U, 0U); Feed(5074U, 0U); Control(5074U, 0U);
    Feed(5096U, 0U); Control(5096U, 0U);
    CHECK(!state.stop_pending && state.feedback.status == 0U && state.controller.faults == 4U);
    CHECK(!state.requested && !state.active && enable_calls == 0U && torque_calls == 0U);
    Command(5098U, "YAW ON", "Y|OK|PREPARE");
    CHECK(state.controller.faults == 0U && state.stop_pending && enable_calls == 0U);
    Control(5100U, 0U); Feed(5102U, 0U); Control(5102U, 0U);
    Feed(5124U, 0U); Control(5124U, 0U);
    CHECK(!state.active && enable_calls == 0U);
    Control(5126U, 1U); CHECK(state.enabling && enable_calls == 1U && torque_calls == 0U);
    Feed(5128U, 1U); Control(5128U, 1U); CHECK(state.active && !state.enabling);
    test_stage = 7U;
    ConsoleActivate(6000U);
    CHECK(state.controller.pid.kp == 1.0f && state.controller.pid.ki == 0.0f);
    Command(6042U, "YAW TARGET 15", "Y|OK|PARAMETER");
    CHECK(state.controller.target_deg == 15.0f && state.active);
    Command(6042U, "YAW TARGET 30.01", "Y|ERROR|PARAMETER_OR_STATE");
    CHECK(state.controller.target_deg == 15.0f);
    Command(6042U, "YAW TARGET -15", "Y|OK|PARAMETER");
    Command(6042U, "YAW TARGET -30.01", "Y|ERROR|PARAMETER_OR_STATE");
    CHECK(state.controller.target_deg == -15.0f);
    Command(6042U, "YAW TARGET 15", "Y|OK|PARAMETER");
    for (t = 6044U; t <= 6050U; t += 2U) { Control(t, 1U); }
    CHECK(state.active && permission && last_torque == YAW_CONTROLLER_TORQUE_LIMIT_NM);
    before = torque_calls;
    Control(6052U, 0U);
    Command(6054U, "YAW OFF", "Y|OK|OFF"); Control(6054U, 0U);
    CHECK(!state.active && !state.requested && !permission && torque_calls == before);
    Command(6054U, "YAW TARGET 15", "Y|ERROR|PARAMETER_OR_STATE");
    Feed(6056U, 0U); Control(6056U, 0U);
    CHECK(!state.stop_pending && state.controller.faults == 0U && torque_calls == before);
    test_stage = 8U;
    ConsoleActivate(7000U);
    Command(7042U, "YAW STEP 30", "Y|ERROR|STEP_REQUIRES_ARMED_ZERO_STATIONARY_HEALTHY");
    CHECK(!state.step_hold && state.controller.target_deg == 0.0f); /* 必须先人工ZERO。 */
    ConsoleZeroActivate(8000U);
    Command(8068U, "YAW STEP 30", "Y|OK|STEP_STARTED");
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_RUNNING && state.step_hold);
    Command(8070U, "YAW KP 0.8", "Y|OK|PARAMETER");
    YawBackend_GetStep(&measurement);
    CHECK(measurement.phase == YAW_STEP_ABORTED && measurement.reason == YAW_STEP_PID_CHANGE);
    CHECK(state.active && state.step_hold && state.controller.target_deg == 30.0f && state.controller.pid.kp == 0.8f);
    CHECK(YawConsole_Poll(step_response, sizeof(step_response)));
    CHECK(strstr(step_response, "|STATE|ABORTED|REASON|PID_CHANGE|VALID|0|") != 0);
    CHECK(!YawConsole_Poll(step_response, sizeof(step_response)));
    Command(8072U, "YAW TARGET 0", "Y|OK|PARAMETER"); CHECK(!state.step_hold && state.controller.target_deg == 0.0f);

    test_stage = 9U;
    RemoteStart(9000U);
    CHECK(YawConsole_Poll(step_response, sizeof(step_response)));
    CHECK(strstr(step_response, "|RESULT|STARTED|TARGET|30.0") != 0);
    for (t = 9076U; t <= 14072U; t += 2U) {
        uint32_t elapsed = t - 9072U;
        if (elapsed % 10U == 0U) {
            FeedDegrees(t, elapsed < 1500U ? (float)elapsed * 0.02f : 30.0f,
                        elapsed < 1500U ? 20.0f : 0.0f);
        }
        Control(t, 2U);
    }
    YawBackend_GetStep(&measurement);
    CHECK(measurement.phase == YAW_STEP_DONE && measurement.sse_valid && !measurement.pass);
    CHECK(measurement.response_ms >= 1480U && measurement.response_ms <= 1510U && measurement.max_gap_ms == 10U);
    CHECK(state.active && state.step_hold && state.controller.target_deg == 30.0f && state.controller.faults == 0U);
    CHECK(YawConsole_Poll(step_response, sizeof(step_response)));
    CHECK(strstr(step_response, "|STATE|DONE|REASON|NONE|VALID|1|") != 0);
    CHECK(strstr(step_response, "|SSE_VALID|1|PASS|0|FAULT|0|") != 0);
    CHECK(strlen(step_response) < 600U && !YawConsole_Poll(step_response, sizeof(step_response)));
    Command(14072U, "YAW RESULT", "|STATE|DONE|");
    receiver.channels[RC_STEP_CHANNEL] = 1000U; Control(14074U, 2U);
    CHECK(state.controller.target_deg == 0.0f && !state.step_hold && state.active);
    CHECK(YawBackend_TakeRemoteStepEvent(&event) && event.result == 2U && event.sequence == 2U);
    receiver.channels[RC_STEP_CHANNEL] = 1500U; Control(14076U, 2U);
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_DONE);
    Control(14078U, 0U); CHECK(!state.active && !permission);

    test_stage = 10U;
    RemoteStart(15000U); CHECK(YawBackend_TakeRemoteStepEvent(&event) && event.result == 1U);
    /* CH7回中不取消30度；退出测试模式必须停机，不能悄悄混入手动。 */
    Control(15076U, 1U); YawBackend_GetStep(&measurement);
    CHECK(measurement.phase == YAW_STEP_ABORTED && measurement.reason == YAW_STEP_MANUAL);
    CHECK(!state.active && !permission && !state.step_hold);
    RemoteStart(16000U); CHECK(YawBackend_TakeRemoteStepEvent(&event));
    FeedDegrees(16082U, 0.2f, 70.0f); Control(16082U, 2U); YawBackend_GetStep(&measurement);
#if YAW_CONTROLLER_SPEED_GUARD_ENABLED
    CHECK(measurement.phase == YAW_STEP_ABORTED && measurement.reason == YAW_STEP_FAULT);
    CHECK((measurement.faults & YAW_CONTROLLER_FAULT_SPEED) && measurement.peak_velocity_dps > 60.0f);
    CHECK(!measurement.pass && !measurement.sse_valid && !state.active && !permission);
    CHECK(YawConsole_Poll(step_response, sizeof(step_response)));
    CHECK(strstr(step_response, "|STATE|ABORTED|REASON|FAULT|VALID|0|") != 0);
#else
    CHECK(measurement.phase == YAW_STEP_RUNNING && !measurement.faults && measurement.peak_velocity_dps > 60.0f);
    CHECK(state.active && permission && state.controller.target_deg == 30.0f && !state.controller.speed_limited);
    CHECK(measurement.command_samples > 0U && measurement.peak_torque_nm > 0.0f);
#endif

    test_stage = 11U;
    ConsoleZeroActivate(17000U); Control(17068U, 2U);
    receiver.channels[RC_STEP_PITCH_CHANNEL] = 2000U; Control(17070U, 2U);
    receiver.channels[RC_STEP_CHANNEL] = 2000U; Control(17072U, 2U);
    CHECK(YawBackend_TakeRemoteStepEvent(&event) && event.result == 4U);
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_IDLE && !state.step_hold);
    CHECK(state.controller.target_deg == 0.0f);
    RemoteStart(18000U); CHECK(YawBackend_TakeRemoteStepEvent(&event));
    receiver.channels[RC_TRIGGER_CHANNEL] = 1000U; Control(18076U, 2U);
    YawBackend_GetStep(&measurement);
    CHECK(!state.active && !permission && measurement.phase == YAW_STEP_ABORTED && !measurement.pass);

    test_stage = 12U;
    Reset(19000U); request.gimbal_mode = GIMBAL_TEST; request.step_event = 1U;
    request.step_sequence = 1U; request.step_axis = GIMBAL_STEP_YAW; request.step_delta_deg = 30.0f;
    Apply(19000U, 0U);
    CHECK(YawBackend_TakeRemoteStepEvent(&event) && event.result == 3U && !state.active && enable_calls == 0U);
    Apply(19002U, 1U);
    CHECK(!YawBackend_TakeRemoteStepEvent(&event)); /* 同一事件不在放行后重放。 */
    test_stage = 13U;
    ConsoleReset(20000U); Control(20000U, 0U);
    Feed(20002U, 0U); Control(20002U, 0U); Feed(20024U, 0U); Control(20024U, 0U);
    Command(20026U, "YAW ZERO", "Y|OK|SOFTWARE_ZERO");
    for (t = 20036U; t <= 20066U; t += 10U) {
        uint32_t position = (uint32_t)(32767.0f + 3.0f * 65535.0f / 360.0f + 0.5f);
        frame = Feedback(t, 0U, 25U, 30U);
        frame.data[1] = (uint8_t)(position >> 8); frame.data[2] = (uint8_t)position;
        now_ms = t; CHECK(YawBackend_OnFrame(&frame)); Control(t, 0U);
    }
    CHECK(state.zero_set && state.controller.angle_deg > 2.99f && state.controller.angle_deg < 3.01f);
    Command(20068U, "YAW ON", "Y|OK|PREPARE");
    CHECK(state.zero_set && state.controller.angle_deg > 2.99f && state.controller.target_deg > 2.99f);
    Control(20070U, 0U);
    for (t = 20072U; t <= 20092U; t += 10U) {
        frame.received_ms = t; now_ms = t; CHECK(YawBackend_OnFrame(&frame)); Control(t, 0U);
    }
    Control(20094U, 1U); FeedDegrees(20096U, 3.0f, 0.0f); Control(20096U, 1U);
    for (t = 20098U; t <= 20104U; t += 2U) { Control(t, 1U); }
    CHECK(state.active && state.zero_set && state.controller.angle_deg > 2.99f && state.controller.target_deg > 2.99f);
    Command(20106U, "YAW STEP 30", "Y|ERROR|STEP_REQUIRES_ARMED_ZERO_STATIONARY_HEALTHY");
    CHECK(state.controller.target_deg > 2.99f); /* ON/FC不覆盖人工零点，也不自行回零。 */
    test_stage = 14U;
    /* 单条READY从无反馈起步：原FD->单次FB->真实状态0->静止ZERO->准备。
     * 在成功报告前即使上层错误给allowed，也不得发送FC。 */
    ConsoleReset(21000U); Control(21000U, 0U);
    Command(21002U, "YAW READY", "Y|BUSY|READY");
    CHECK(!state.requested && !state.zero_set && state.stop_pending);
    for (t = 21010U; t <= 21200U; t += 10U) {
        pending = 0U;
        Feed(t, clear_calls ? 0U : 13U); Control(t, 0U);
        if (YawConsole_Poll(response, sizeof(response))) {
            CHECK(strstr(response, "Y|OK|READY|ZERO_SET|1|REQUESTED|1|ACTIVE|0|") == response);
            break;
        }
        CHECK(!enable_calls && !torque_calls);
    }
    CHECK(t <= 21200U && clear_calls == 1U && !enable_calls && !torque_calls);
    Read(); CHECK(state.zero_set && state.requested && !state.stop_pending && !state.controller.faults);
    CHECK(!YawConsole_Poll(response, sizeof(response))); /* 成功只报一次 */
    Control(t + 2U, 0U); Feed(t + 4U, 0U); Control(t + 4U, 2U);
    CHECK(enable_calls == 1U && state.enabling); /* 上->下可直接解锁，无中档硬要求。 */
    Feed(t + 6U, 1U); Control(t + 6U, 2U);
    CHECK(state.active && !state.controller.faults);
    before = enable_calls;
    Command(t + 8U, "YAW PID 0.8 0.1 0.02", "Y|OK|PID");
    CHECK(state.active && enable_calls == before && state.controller.pid.kp == 0.8f && state.controller.pid.ki == 0.1f);
    Command(t + 10U, "YAW PID 0.9 2 0.03", "Y|ERROR|");
    CHECK(state.controller.pid.kp == 0.8f && state.controller.pid.ki == 0.1f && state.controller.pid.kd == 0.02f);

    test_stage = 15U;
    ConsoleReset(22000U); Control(22000U, 0U);
    Command(22002U, "YAW READY", "Y|BUSY|READY");
    Command(22004U, "YAW ON", "Y|ERROR|READY_BUSY");
    Command(22006U, "YAW READY", "Y|BUSY|READY");
    CHECK(!enable_calls && !state.requested);
    Command(22008U, "YAW OFF", "Y|OK|OFF");
    CHECK(!YawConsole_Poll(response, sizeof(response)) && !state.requested);
    Command(22010U, "YAW READY", "Y|BUSY|READY");
    receiver.channels[RC_FRICTION_CHANNEL] = 1500U; Control(22012U, 0U);
    CHECK(YawConsole_Poll(response, sizeof(response)));
    CHECK(strstr(response, "Y|ERROR|READY|NOT_NEUTRAL") == response && !state.requested && !enable_calls);

    test_stage = 16U;
    ConsoleReset(23000U); Control(23000U, 0U);
    Command(23002U, "YAW READY", "Y|BUSY|READY");
    Control(28002U, 0U);
    CHECK(YawConsole_Poll(response, sizeof(response)) && strstr(response, "Y|ERROR|READY|TIMEOUT") == response);
    CHECK(!state.requested && !enable_calls);
    ConsoleReset(29000U); Control(29000U, 0U); bus.fault_epoch = 1U;
    Command(29002U, "YAW READY", "Y|ERROR|READY|CAN_FAULT_USE_YAW_BUS");
    CHECK(!state.requested && !enable_calls);
    bus.fault_epoch = 0U;
    Command(29004U, "YAW READY", "Y|BUSY|READY");
    bus.fault_epoch = 1U;
    CHECK(YawConsole_Poll(response, sizeof(response)) && strstr(response, "Y|ERROR|READY|CAN_FAULT") == response);
    CHECK(!state.requested && !enable_calls);
    ConsoleReset(30000U); Control(30000U, 0U);
    Command(30002U, "YAW READY", "Y|BUSY|READY");
    YawConsole_CancelPreparation();
    CHECK(!YawConsole_Poll(response, sizeof(response)) && !state.requested && !enable_calls);
    test_stage = 17U;
    /* 驱动保持13时只发一次FB，不循环清错；到期仍退出，不会FC。 */
    ConsoleReset(31000U); Control(31000U, 0U);
    Command(31002U, "YAW READY", "Y|BUSY|READY");
    for (t = 31010U; t < 36002U; t += 10U) {
        pending = 0U; Feed(t, 13U); Control(t, 0U);
        CHECK(!YawConsole_Poll(response, sizeof(response)));
    }
    CHECK(clear_calls == 1U && !enable_calls && !torque_calls);
    Control(36002U, 0U);
    CHECK(YawConsole_Poll(response, sizeof(response)) && strstr(response, "|TIMEOUT_") != 0);
    CHECK(!state.requested && clear_calls == 1U);

    test_stage = 18U;
    /* 原状态0时无需FB；成功报告前仍有准备门禁，提前拨CH5会取消。 */
    ConsoleReset(37000U); Control(37000U, 0U);
    Command(37002U, "YAW READY", "Y|BUSY|READY");
    for (t = 37010U; t <= 37100U; t += 10U) {
        pending = 0U; Feed(t, 0U); Control(t, 0U);
        CHECK(!YawConsole_Poll(response, sizeof(response)));
        Read(); if (state.requested) { break; }
    }
    CHECK(state.requested && !enable_calls && !clear_calls);
    Feed(t + 2U, 0U); Control(t + 2U, 2U);
    CHECK(!enable_calls && !state.active);
    CHECK(YawConsole_Poll(response, sizeof(response)) && strstr(response, "|CH5_NOT_OFF") != 0);
    Read(); CHECK(!state.requested && !enable_calls);
    ConsoleReset(38000U); Control(38000U, 0U);
    Command(38002U, "YAW READY", "Y|BUSY|READY");
    receiver.has_frame = 0U;
    CHECK(YawConsole_Poll(response, sizeof(response)) && strstr(response, "|STALE") != 0);
    Read(); CHECK(!state.requested && !enable_calls);

    test_stage = 19U;
    RemoteStart(39000U);
    Command(39076U, "YAW PID 1.2 0.1 0.03", "Y|OK|PID");
    YawBackend_GetStep(&measurement);
    CHECK(measurement.phase == YAW_STEP_ABORTED && measurement.reason == YAW_STEP_PID_CHANGE);
    CHECK(state.active && state.controller.target_deg == 30.0f && !measurement.pass);
    test_done = 1U;
}
