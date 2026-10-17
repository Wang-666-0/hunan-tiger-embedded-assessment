#include "yaw_console.h"
#include "yaw_backend.h"
#include "app_tasks.h"
#include "remote_input.h"
#include "app_console.h"
#include "app_config.h"
#include "can_bus.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

static RemoteInput_State remote;
static YawBackendState yaw;
static uint32_t now, request_count, stop_count, capture_count, parameter_count, state_reads;
static uint32_t operation_order, stop_order, request_order;
static uint8_t app_armed, diagnostic, request_result, capture_result;
static char last_parameter, response[512];
static float last_value;
static uint32_t clear_count;
static YawClearCheck clear_check;
static YawStep step_mock;
static CanBusDiagnostics bus_mock;
void CanBus_GetDiagnostics(CanBusDiagnostics *out) { if (out) { *out = bus_mock; } }
uint8_t YawBackend_PrepareStop(void)
{
    yaw.requested = yaw.zero_set = 0U; YawBackend_Stop(); return 1U;
}
void YawBackend_ReleasePreparation(void) { }
uint8_t YawBackend_SetPid(float kp, float ki, float kd)
{
    ++parameter_count;
    if (!(kp >= 0.0f && kp <= 2.0f && ki >= 0.0f && ki <= 1.0f && kd >= 0.0f && kd <= 0.2f)) { return 0U; }
    yaw.controller.pid.kp = kp; yaw.controller.pid.ki = ki; yaw.controller.pid.kd = kd; return 1U;
}
uint8_t YawBackend_TakeRemoteStepEvent(YawRemoteStepEvent *out) { (void)out; return 0U; }
void YawBackend_GetStep(YawStep *out) { *out = step_mock; }
uint8_t YawBackend_TakeStepReport(YawStep *out)
{ if (!step_mock.pending) { return 0U; } *out = step_mock; step_mock.pending = 0U; return 1U; }
uint8_t YawBackend_StartStep(float value)
{
    if (!yaw.active || !yaw.zero_set || (value != 30.0f && value != -30.0f)) { return 0U; }
    return YawStep_Start(&step_mock, now, 0.0f, value);
}
void YawBackend_GetClearCheck(YawClearCheck *out) { *out = clear_check; }
uint8_t YawBackend_ClearCommsFault(void)
{
    ++clear_count;
    return !yaw.requested && !yaw.active && yaw.online && yaw.feedback.status == 13U;
}

uint32_t HAL_GetTick(void) { return now; }
void RemoteInput_GetState(RemoteInput_State *out) { *out = remote; }
void AppTasks_GetState(AppState *out)
{
    memset(out, 0, sizeof(*out));
    out->logic.safety.operator_armed = app_armed;
}
uint8_t AppConsole_IsDiagnosticMode(void) { return diagnostic; }
void AppTasks_RequestStop(void)
{
    ++stop_count; stop_order = ++operation_order;
    yaw.active = yaw.controller.active = 0U;
    yaw.controller.torque_nm = 0.0f;
}
void YawBackend_GetState(YawBackendState *out) { ++state_reads; *out = yaw; }
void YawBackend_Stop(void)
{
    yaw.active = yaw.controller.active = 0U;
    yaw.controller.torque_nm = 0.0f;
}
uint8_t YawBackend_Request(uint8_t enabled)
{
    ++request_count; request_order = ++operation_order;
    if (!enabled) { yaw.requested = 0U; return 1U; }
    /* ON为准备请求，未在线可先FD取反馈；真实使能由后端另外验收。 */
    if (request_result) { yaw.requested = 1U; }
    return request_result;
}
uint8_t YawBackend_CaptureReference(void)
{
    ++capture_count;
    if (!capture_result || yaw.active || yaw.requested) { return 0U; }
    yaw.controller.angle_deg = yaw.controller.target_deg = 0.0f;
    return 1U;
}
uint8_t YawBackend_SetParameter(char parameter, float value)
{
    ++parameter_count; last_parameter = parameter; last_value = value;
    /* 模拟后端范围和实际活动许可，观察解析器是否正确委派。 */
    if (parameter == 'P' && value >= 0.0f && value <= 2.0f) {
        yaw.controller.pid.kp = value; return 1U;
    }
    if (parameter == 'I' && value >= 0.0f && value <= 1.0f) {
        yaw.controller.pid.ki = value; return 1U;
    }
    if (parameter == 'D' && value >= 0.0f && value <= 0.2f) {
        yaw.controller.pid.kd = value; return 1U;
    }
    if (parameter == 'T' && yaw.active && value >= -YAW_CONTROLLER_TARGET_LIMIT_DEG &&
        value <= YAW_CONTROLLER_TARGET_LIMIT_DEG) {
        yaw.controller.target_deg = value; return 1U;
    }
    return 0U;
}

static void baseline(void)
{
    uint32_t index;
    YawConsole_CancelPreparation();
    memset(&remote, 0, sizeof(remote)); memset(&yaw, 0, sizeof(yaw));
    YawStep_Init(&step_mock);
    for (index = 0U; index < 14U; ++index) { remote.channels[index] = 1500U; }
    remote.channels[RC_ARM_CHANNEL] = 1000U;
    remote.channels[RC_FRICTION_CHANNEL] = 1000U;
    now = 200U; remote.has_frame = 1U; remote.received_ms = now - 1U;
    app_armed = diagnostic = 0U; request_result = capture_result = 1U;
    request_count = stop_count = capture_count = parameter_count = state_reads = 0U;
    operation_order = stop_order = request_order = 0U;
    clear_count = 0U;
    memset(&clear_check, 0, sizeof(clear_check)); clear_check.clear_result = 4U;
    last_parameter = 0; last_value = 0.0f;
    yaw.controller.pid.kp = 0.35f; yaw.controller.pid.kd = 0.015f;
    response[0] = '\0';
}

static void command(const char *line, const char *expected)
{
    memset(response, 0x5A, sizeof(response));
    CHECK(YawConsole_Handle(line, response, sizeof(response)));
    if (strcmp(expected, "Y|ERROR|CLEAR|REMOTE|") == 0) {
        CHECK(strstr(response, expected) == response);
    } else { CHECK(strcmp(response, expected) == 0); }
}

static void refused_on(void)
{
    command("YAW ON", "Y|ERROR|SAFE_OFF_REQUIRED\r\n");
    CHECK(request_count == 0U && stop_count == 0U);
    CHECK(!yaw.requested && !yaw.active);
}

void test_run(void)
{
    static const char *invalid[] = {
        "YAW KP nan", "YAW KI NaN", "YAW KD inf", "YAW TARGET -inf",
        "YAW KP 1e100", "YAW KP 0.3junk", "YAW KP 0.3 0.4",
        "YAW KP 0.3 ", "YAW KP ", "YAW KI ", "YAW KD ",
        "YAW TARGET ", "YAW KP", "YAW TARGET", "YAW BAD 1"
    };
    YawBackendState snapshot;
    uint32_t index, old_parameters, old_requests, old_stops;
    char small[8];
    test_stage = 1U;
    CHECK(APP_REMOTE_PROFILE_ASSESSMENT && !APP_REMOTE_PROFILE_LEGACY && APP_YAW_ENABLED);
    baseline();
    CHECK(!YawConsole_Handle(0, response, sizeof(response)));
    CHECK(!YawConsole_Handle("WHEEL OFF", response, sizeof(response)));
    CHECK(YawConsole_Handle("YAW", response, sizeof(response)));
    CHECK(strstr(response, "Y|STATE|") == response && !request_count && !stop_count && !parameter_count);
    command("YAW ON", "Y|OK|PREPARE\r\n");
    CHECK(request_count == 1U && stop_count == 1U && yaw.requested);
    CHECK(stop_order == 1U && request_order == 2U);
    CHECK(!yaw.active && !yaw.controller.active && yaw.controller.torque_nm == 0.0f);
    CHECK(!yaw.online); /* 仅准备，可先发失能取MIT响应，不构造使能证据。 */
    baseline(); remote.channels[RC_TRIGGER_CHANNEL] = 1000U; refused_on();
    baseline(); remote.channels[RC_TRIGGER_CHANNEL] = 2000U; refused_on();
    baseline(); remote.channels[RC_FRICTION_CHANNEL] = 1500U; refused_on();
    baseline(); remote.channels[RC_FRICTION_CHANNEL] = 2000U; refused_on();
    for (index = 0U; index < 4U; ++index) {
        baseline(); remote.channels[index] = 1531U; refused_on();
        baseline(); remote.channels[index] = 1469U; refused_on();
    }
    baseline(); remote.channels[RC_ARM_CHANNEL] = 1500U; refused_on();
    baseline(); remote.channels[RC_ARM_CHANNEL] = 2000U; refused_on();
    baseline(); remote.channels[RC_STEP_CHANNEL] = 1000U; refused_on();
    baseline(); remote.channels[RC_STEP_CHANNEL] = 2000U; refused_on();
    baseline(); diagnostic = 1U; refused_on();
    baseline(); app_armed = 1U; refused_on();
    baseline(); remote.has_frame = 0U; refused_on();
    baseline(); remote.received_ms = now - REMOTE_TIMEOUT_MS; refused_on();
    baseline(); remote.received_ms = now + 1U; refused_on();
    baseline(); remote.channels[2] = 799U; refused_on();
    baseline(); remote.channels[7] = 2201U; refused_on();
    baseline(); remote.received_ms = now - REMOTE_TIMEOUT_MS + 1U;
    for (index = 0U; index < 4U; ++index) { remote.channels[index] = 1470U; }
    command("YAW ON", "Y|OK|PREPARE\r\n");
    CHECK(stop_count == 1U && request_count == 1U);
    baseline(); request_result = 0U;
    command("YAW ON", "Y|ERROR|SAFE_OFF_REQUIRED\r\n");
    CHECK(stop_count == 1U && request_count == 1U && !yaw.requested);
    CHECK(stop_order < request_order); /* 准备被后端拒绝也已经建立STOP屏障。 */

    test_stage = 2U;
    baseline(); diagnostic = app_armed = 1U; remote.has_frame = 0U;
    yaw.active = yaw.controller.active = yaw.requested = 1U;
    yaw.controller.torque_nm = 0.02f;
    command("YAW OFF", "Y|OK|OFF\r\n");
    CHECK(request_count == 1U && stop_count == 1U);
    CHECK(request_order == 1U && stop_order == 2U);
    CHECK(!yaw.requested && !yaw.active && yaw.controller.torque_nm == 0.0f);
    baseline(); yaw.controller.angle_deg = 2.0f;
    command("YAW ZERO", "Y|OK|SOFTWARE_ZERO\r\n");
    CHECK(capture_count == 1U && request_count == 0U && stop_count == 0U);
    CHECK(yaw.controller.angle_deg == 0.0f && !yaw.active);
    baseline(); remote.channels[RC_ARM_CHANNEL] = 1500U;
    command("YAW ZERO", "Y|ERROR|STOP_AND_STATIONARY_REQUIRED\r\n");
    CHECK(capture_count == 0U);
    baseline(); diagnostic = 1U;
    command("YAW ZERO", "Y|ERROR|STOP_AND_STATIONARY_REQUIRED\r\n");
    CHECK(capture_count == 0U);
    baseline(); capture_result = 0U;
    command("YAW ZERO", "Y|ERROR|STOP_AND_STATIONARY_REQUIRED\r\n");
    CHECK(capture_count == 1U && !yaw.active);
    baseline(); yaw.requested = 1U;
    command("YAW ZERO", "Y|ERROR|STOP_AND_STATIONARY_REQUIRED\r\n");
    CHECK(capture_count == 1U && yaw.requested);

    test_stage = 3U;
    baseline(); yaw.active = yaw.controller.active = 1U;
    remote.channels[RC_ARM_CHANNEL] = 1500U; app_armed = 1U;
    command("YAW KP 0.5", "Y|OK|PARAMETER\r\n");
    CHECK(last_parameter == 'P' && last_value == 0.5f && yaw.controller.pid.kp == 0.5f);
    command("YAW KI 0.01", "Y|OK|PARAMETER\r\n");
    CHECK(last_parameter == 'I' && last_value == 0.01f);
    command("YAW KD 0.02", "Y|OK|PARAMETER\r\n");
    CHECK(last_parameter == 'D' && last_value == 0.02f);
    CHECK(parameter_count == 3U && request_count == 0U && stop_count == 0U && yaw.active);
    command("YAW KP 2.1", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    CHECK(parameter_count == 4U && yaw.controller.pid.kp == 0.5f);
    old_parameters = parameter_count;
    for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); ++index) {
        command(invalid[index], strcmp(invalid[index], "YAW BAD 1") == 0 ?
            "Y|ERROR|UNKNOWN_COMMAND|USE|YAW HELP\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n");
        CHECK(parameter_count == old_parameters);
    }
    diagnostic = 1U;
    command("YAW KP 0.6", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    CHECK(parameter_count == old_parameters && request_count == 0U && stop_count == 0U);
    diagnostic = 0U; yaw.active = 0U;
    command("YAW TARGET 1", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    CHECK(last_parameter == 'T' && last_value == 1.0f && yaw.controller.target_deg == 0.0f);
    yaw.active = 1U;
    command("YAW TARGET -2", "Y|OK|PARAMETER\r\n");
    CHECK(last_parameter == 'T' && last_value == -2.0f && yaw.controller.target_deg == -2.0f);
    command("YAW TARGET 30.1", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    CHECK(yaw.controller.target_deg == -2.0f && request_count == 0U && stop_count == 0U);

    test_stage = 4U;
    yaw.online = 1U; yaw.requested = 1U; yaw.stop_pending = 1U;
    yaw.feedback.status = 1U; yaw.controller.faults = 17U;
    yaw.feedback_age_ms = 12U; yaw.rx_count = 123U;
    snapshot = yaw; old_parameters = parameter_count;
    old_requests = request_count; old_stops = stop_count;
    CHECK(YawConsole_Handle("YAW STATUS", response, sizeof(response)));
    CHECK(strncmp(response, "Y|STATUS|" APP_YAW_VERSION "|", sizeof("Y|STATUS|" APP_YAW_VERSION "|") - 1U) == 0);
    CHECK(strstr(response, "|ONLINE|1|ACTIVE|1|REQUESTED|1|STATUS|1|FAULT|17|AGE_MS|12|STOP_PENDING|1|") != 0);
    CHECK(strstr(response, "|RX|123\r\n") != 0);
    CHECK(strstr(response, "|LIMIT_NM|0.100|") != 0);
    CHECK(strstr(response, "|TARGET_LIMIT_DEG|30.0|HARD_LIMIT_DEG|40.0|") != 0);
    CHECK(strstr(response, YAW_CONTROLLER_SPEED_GUARD_ENABLED ? "|SOFT_DPS|40.0|HARD_DPS|60.0|SPEED_GUARD|1|" :
        "|SOFT_DPS|0.0|HARD_DPS|0.0|SPEED_GUARD|0|") != 0);
    CHECK(memcmp(&snapshot, &yaw, sizeof(yaw)) == 0);
    CHECK(parameter_count == old_parameters && request_count == old_requests && stop_count == old_stops);
    CHECK(YawConsole_Handle("YAW INFO", response, sizeof(response)));
    CHECK(memcmp(&snapshot, &yaw, sizeof(yaw)) == 0 && state_reads == 2U);
    memset(small, 0x5A, sizeof(small));
    CHECK(YawConsole_Handle("YAW STATUS", small, 6U));
    CHECK(small[5] == '\0' && small[6] == 0x5A && small[7] == 0x5A);
    CHECK(memcmp(&snapshot, &yaw, sizeof(yaw)) == 0);
    test_stage = 5U;
    baseline(); yaw.online = 1U; yaw.feedback.status = 13U;
    command("YAW CLEAR", "Y|OK|CLEAR_SENT\r\n");
    CHECK(clear_count == 1U && request_count == 0U && stop_count == 0U && !yaw.active);
    baseline(); remote.channels[RC_ARM_CHANNEL] = 1500U;
    command("YAW CLEAR", "Y|ERROR|CLEAR|REMOTE|");
    CHECK(strstr(response, "|REMOTE|CH5_NOT_OFF|") != 0);
    CHECK(clear_count == 0U);
    baseline(); remote.channels[RC_TRIGGER_CHANNEL] = 1000U;
    command("YAW CLEAR", "Y|ERROR|CLEAR|REMOTE|");
    CHECK(strstr(response, "|REMOTE|CH8_LOCK|") != 0);
    CHECK(clear_count == 0U);
    baseline(); diagnostic = 1U;
    command("YAW CLEAR", "Y|ERROR|CLEAR|REMOTE|");
    CHECK(strstr(response, "|REMOTE|DIAGNOSTIC|") != 0);
    CHECK(clear_count == 0U);
    baseline(); yaw.online = 1U; yaw.feedback.status = 8U;
    command("YAW CLEAR", "Y|ERROR|CLEAR|REMOTE|");
    CHECK(clear_count == 1U && request_count == 0U);
    test_stage = 6U;
    baseline(); remote.channels[2] = 1548U;
    yaw.online = 1U; yaw.feedback.status = 13U;
    clear_check.blocks = YAW_CLEAR_NOT_STATIONARY;
    clear_check.samples = 1U; clear_check.hold_ms = 10U;
    clear_check.fd_result = 0U; clear_check.mailbox_pending = 1U;
    clear_check.off_motion_faults = 48U; clear_check.required_hold_ms = 500U;
    snapshot = yaw;
    CHECK(YawConsole_Handle("YAW CHECK", response, sizeof(response)));
    CHECK(strstr(response, "Y|CHECK|REMOTE|NOT_NEUTRAL|BLOCKS|256|") == response);
    CHECK(strstr(response, "|SAMPLES|1|HOLD_MS|10|") != 0);
    CHECK(strstr(response, "|OFF_MOTION|48|ACTUATION_STARTED|0|REQUIRED_HOLD_MS|500|") != 0);
    CHECK(strstr(response, "|CH|1500,1500,1548,1500,1000,1000,1500,1500\r\n") != 0);
    CHECK(clear_count == 0U && request_count == 0U && stop_count == 0U);
    CHECK(memcmp(&snapshot, &yaw, sizeof(yaw)) == 0);
    command("YAW CLEAR", "Y|ERROR|CLEAR|REMOTE|");
    CHECK(strstr(response, "|REMOTE|NOT_NEUTRAL|") != 0 && clear_count == 0U);
    memset(small, 0x5A, sizeof(small));
    CHECK(YawConsole_Handle("YAW CHECK", small, 6U));
    CHECK(small[5] == '\0' && small[6] == 0x5A && small[7] == 0x5A);
    test_stage = 7U;
    baseline(); command("YAW STEP 30", "Y|ERROR|STEP_REQUIRES_ARMED_ZERO_STATIONARY_HEALTHY\r\n");
    yaw.active = yaw.zero_set = 1U; app_armed = 1U; remote.channels[RC_ARM_CHANNEL] = 1500U;
    command("YAW STEP 30.1", "Y|ERROR|STEP_REQUIRES_ARMED_ZERO_STATIONARY_HEALTHY\r\n");
    command("YAW STEP nan", "Y|ERROR|STEP_REQUIRES_ARMED_ZERO_STATIONARY_HEALTHY\r\n");
    command("YAW STEP 30", "Y|OK|STEP_STARTED|TARGET|30.0|WINDOW_MS|5000|HOLD|1\r\n");
    CHECK(step_mock.phase == YAW_STEP_RUNNING);
    YawStep_Abort(&step_mock, now + 10U, YAW_STEP_FAULT, 16U);
    CHECK(!YawConsole_Poll(small, sizeof(small)) && step_mock.pending);
    CHECK(YawConsole_Poll(response, sizeof(response)) && !step_mock.pending);
    CHECK(strstr(response, "|STATE|ABORTED|REASON|FAULT|VALID|0|") != 0);
    CHECK(strstr(response, "|RESP_MS|-1|") != 0 && strstr(response, "|FAULT|16|") != 0);
    CHECK(!YawConsole_Poll(response, sizeof(response)));
    CHECK(YawConsole_Handle("YAW RESULT", response, sizeof(response)));
    CHECK(strstr(response, "|STATE|ABORTED|") != 0);
    test_stage = 8U;
    baseline(); memset(&bus_mock, 0, sizeof(bus_mock));
    bus_mock.state.fault_epoch = 141422U; bus_mock.state.errors[0] = 70332U;
    bus_mock.state.first_error[0] = 0x800U; bus_mock.state.last_error[0] = 0x20800U;
    bus_mock.hal_error[0] = 0x20800U; bus_mock.state.first_error_source[0] = CAN_BUS_SOURCE_IRQ;
    bus_mock.state.first_error_ms[0] = 1234U; bus_mock.pending[1] = 1U;
    yaw.active = yaw.requested = yaw.zero_set = 1U; diagnostic = app_armed = 1U;
    snapshot = yaw; old_parameters = parameter_count;
    CHECK(YawConsole_Handle("YAW BUS", response, sizeof(response)));
    CHECK(strstr(response, "Y|BUS|" APP_YAW_VERSION "|EPOCH|141422|DROP|0|C1_RX|") == response);
    CHECK(strstr(response, "|ERR|70332|LAST|0x00020800|HAL|0x00020800|FIRST|0x00000800|SOURCE|3|FIRST_MS|1234|") != 0);
    CHECK(strstr(response, "|C2_RX|") != 0 && strstr(response, "|PENDING|1\r\n") != 0);
    CHECK(!memcmp(&snapshot, &yaw, sizeof(yaw)) && parameter_count == old_parameters);
    CHECK(request_count == 0U && stop_count == 0U && capture_count == 0U && clear_count == 0U);
    memset(small, 0x5A, sizeof(small));
    CHECK(YawConsole_Handle("YAW BUS", small, 6U));
    CHECK(small[5] == '\0' && small[6] == 0x5A && small[7] == 0x5A);
    CHECK(!memcmp(&snapshot, &yaw, sizeof(yaw)));
    test_stage = 9U;
    baseline(); command("YAW PID 1 0 0.015", "Y|OK|PID\r\n");
    CHECK(yaw.controller.pid.kp == 1.0f && yaw.controller.pid.ki == 0.0f && yaw.controller.pid.kd == 0.015f);
    snapshot = yaw;
    command("YAW PID 3 0 0.015", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    CHECK(!memcmp(&snapshot, &yaw, sizeof(yaw)));
    command("YAW PID 1 nan 0.015", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    command("YAW PID 1 0", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    command("YAW PID 1 0 0.015 extra", "Y|ERROR|PARAMETER_OR_STATE\r\n");
    CHECK(!memcmp(&snapshot, &yaw, sizeof(yaw)));
    baseline(); diagnostic = 1U;
    command("YAW READY", "Y|ERROR|READY|DIAGNOSTIC\r\n");
    CHECK(!request_count && !stop_count);
    baseline(); memset(&bus_mock, 0, sizeof(bus_mock)); bus_mock.state.started = 1U;
    bus_mock.state.fault_epoch = 1U;
    command("YAW READY", "Y|ERROR|READY|CAN_FAULT_USE_YAW_BUS\r\n");
    CHECK(!request_count && !stop_count);
    test_done = 1U;
}
