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
static char response[1024];
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
    receiver.channels[4U] = 1000U;
    receiver.channels[5U] = 1000U;
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
    receiver.channels[4U] = mode_on == 2U ? 2000U : (mode_on ? 1500U : 1000U);
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

static void __attribute__((unused)) ConsoleActivate(uint32_t timestamp)
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
static void __attribute__((unused)) RemoteStart(uint32_t timestamp)
{
    ConsoleZeroActivate(timestamp);
    Control(timestamp + 68U, 2U); /* CH5测试，先观察CH7中位。 */
    receiver.channels[RC_STEP_YAW_CHANNEL] = 2000U; Control(timestamp + 70U, 2U);
    receiver.channels[6U] = 2000U; Control(timestamp + 72U, 2U);
    CHECK(state.step_hold && state.controller.target_deg == 30.0f && request.step_sequence == 1U);
    receiver.channels[RC_STEP_YAW_CHANNEL] = receiver.channels[6U] = 1500U;
    Control(timestamp + 74U, 2U);
}

static void __attribute__((unused)) RecoverThroughConsole(uint32_t timestamp)
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
