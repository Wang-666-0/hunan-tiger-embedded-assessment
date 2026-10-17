/* This links the real console/backend/core. Only HAL transport and app/receiver snapshots are mocked. */
#define test_run backend_standalone_test_run
#include "test_pitch_backend.c"
#undef test_run
#include "pitch_console.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"

static RemoteInput_State receiver;
static AppState application;
static uint32_t stop_calls, rearm_calls;
static uint8_t diagnostic_mode;
static char response[1024];
void RemoteInput_GetState(RemoteInput_State *out) { if (out) { *out = receiver; } }
void AppTasks_GetState(AppState *out) { if (out) { *out = application; } }
uint8_t AppConsole_IsDiagnosticMode(void) { return diagnostic_mode; }
void AppTasks_RequestStop(void) { ++stop_calls; PitchBackend_Stop(); }
void AppTasks_RequireRearm(void) { ++rearm_calls; ++application.stop_sequence; }
static void Receiver(uint32_t timestamp, uint16_t ch5)
{ receiver.received_ms = timestamp; receiver.channels[4] = ch5; }
static void ConsoleReset(uint32_t timestamp)
{
    uint32_t i;
    Reset(timestamp); PitchConsole_Init();
    memset(&receiver, 0, sizeof(receiver)); memset(&application, 0, sizeof(application));
    for (i = 0U; i < 14U; ++i) { receiver.channels[i] = 1500U; }
    receiver.channels[4] = receiver.channels[5] = 1000U;
    receiver.has_frame = 1U; receiver.received_ms = timestamp;
    stop_calls = rearm_calls = 0U; diagnostic_mode = 0U;
}
static void Command(const char *line, const char *expected)
{
    CHECK(PitchConsole_Handle(line, response, sizeof(response)));
    CHECK(strstr(response, expected) != 0);
}
void test_run(void)
{
    uint32_t before, tick;
    float reference;
    PitchStep measurement;
    test_stage = 1U;
    ConsoleReset(100U);
    CHECK(!PitchConsole_Handle("YAW STATUS", response, sizeof(response)));
    Command("PITCH STATUS", "COORD_CONFIRMED|0"); CHECK(!query_calls && !disable_calls && !enable_calls);
    CHECK(strstr(response, "DM3507_CASCADE_V1A") && strstr(response, "MIN_DEG|-18|MAX_DEG|33"));
    CHECK(strstr(response, "HARD_MIN_DEG|-19|HARD_MAX_DEG|34") && strstr(response, "SUPPORT_REQUIRED|1\r\n"));
    Command("PITCH READY", "COORD_UNCONFIRMED"); CHECK(!query_calls);
    Command("PITCH CHECK", "P|BUSY|CHECK"); CHECK(query_calls == 1U);
    Feed(102U, 0U); CHECK(PitchConsole_Poll(response, sizeof(response)));
    CHECK(strstr(response, "P|CHECK|") && strstr(response, "RAW_RAD|") && strstr(response, "READ_ONLY_MONITOR|1"));
    CHECK(!PitchConsole_Poll(response, sizeof(response))); CHECK(!disable_calls && !enable_calls && !torque_calls);
    Command("PITCH COORD 2", "P|ERROR|"); Command("PITCH COORD 1", "P|OK|COORD|DIVISOR|1");
    Command("PITCH READY", "P|BUSY|READY");
    Feed(112U, 0U); CHECK(!PitchConsole_Poll(response, sizeof(response)));
    Feed(124U, 0U); Receiver(124U, 1000U); CHECK(PitchConsole_Poll(response, sizeof(response)));
    CHECK(strstr(response, "P|OK|READY|") && rearm_calls == 1U && stop_calls == 0U);
    Read(); CHECK(state.requested && state.zero_set && !state.active && !enable_calls && !disable_calls);
    before = query_calls; Apply(136U, 0U); CHECK(query_calls > before && !enable_calls && !disable_calls);
    Read(); reference = state.controller.reference_rad; before = query_calls;
    Command("PITCH READY", "ALREADY_READY"); Read();
    CHECK(query_calls == before && !enable_calls && !disable_calls && rearm_calls == 1U);
    CHECK(state.controller.reference_rad == reference && state.requested && state.zero_set);
    Command("PITCH STEP 30", "P|ERROR|");

    test_stage = 2U;
    Receiver(138U, 2000U); application.logic.safety.operator_armed = 1U; request.gimbal_mode = GIMBAL_TEST;
    Apply(138U, 1U); Feed(140U, 1U); Apply(140U, 1U);
    for (tick = 142U; tick <= 160U; tick += 2U) {
        if (tick == 150U || tick == 160U) { Feed(tick, 1U); }
        Receiver(tick, 2000U); Apply(tick, 1U);
    }
    Command("PITCH STEP 1", "P|ERROR|"); Command("PITCH STEP 30", "P|OK|STEP_STARTED|");
    Command("PITCH STEP -30", "STEP_OUTSIDE_MECHANICAL_RANGE");
    Command("PITCH RESULT", "STATE|RUNNING"); CHECK(strstr(response, "RESP_LIMIT_MS|300"));
    Command("PITCH PKP 12", "P|OK|PARAMETER"); PitchBackend_GetStep(&measurement);
    CHECK(measurement.phase == PITCH_STEP_RUNNING);
    Command("PITCH PKP 13", "P|OK|PARAMETER"); PitchBackend_GetStep(&measurement); Read();
    CHECK(measurement.phase == PITCH_STEP_ABORTED && state.controller.target_deg == 30.0f);
    CHECK(PitchConsole_Poll(response, sizeof(response)) && strstr(response, "REASON|PID_CHANGE"));
    CHECK(!PitchConsole_Poll(response, sizeof(response)));
    Command("PITCH TARGET 0", "P|OK|TARGET|0.000");
    Command("PITCH PID", "POS|13,"); Command("PITCH VMAX 90", "OUTPUT_LIMITS_DISABLED");
    before = state.controller.cascade.revision;
    Command("PITCH POS 12 0.2", "P|ERROR|"); Command("PITCH POS 12 0.2 0.02 9", "P|ERROR|");
    Command("PITCH VKP nan", "P|ERROR|"); Command("PITCH VKD inf", "P|ERROR|");
    Command("PITCH VKP -1", "P|ERROR|"); Read(); CHECK(state.controller.cascade.revision == before);
    Command("PITCH VEL 0.1 0.2 0.0001", "P|OK|VEL");
    Command("PITCH OFF", "DISABLE_IS_NOT_BRAKE|1"); CHECK(stop_calls == 1U);
    Apply(162U, 0U); CHECK(disable_calls && !permission);

    test_stage = 3U;
    ConsoleReset(1000U); Command("PITCH CHECK", "P|BUSY|CHECK");
    now_ms = 2000U; CHECK(PitchConsole_Poll(response, sizeof(response)));
    CHECK(strstr(response, "QUERY|TIMEOUT") && !enable_calls && !disable_calls);
    ConsoleReset(3000U); Feed(3000U, 0U); Feed(3020U, 0U); Receiver(3020U, 1000U);
    Command("PITCH COORD 7", "P|OK|COORD|DIVISOR|7");
    Command("PITCH READY", "P|BUSY|READY"); PitchConsole_CancelPreparation();
    Feed(3022U, 0U); Feed(3044U, 0U); Receiver(3044U, 1000U);
    CHECK(!PitchConsole_Poll(response, sizeof(response))); Read();
    CHECK(!state.requested && !state.active && !enable_calls && !disable_calls && !rearm_calls);
    Command("PITCH READY", "P|BUSY|READY"); Receiver(3046U, 2000U); now_ms = 3046U;
    CHECK(PitchConsole_Poll(response, sizeof(response)) && strstr(response, "CH5_NOT_OFF"));
    CHECK(!state.requested && !enable_calls && !disable_calls);
    test_stage = 4U;
    ConsoleReset(4000U);
    CHECK(!PitchConsole_Poll(response, sizeof(response)));
    application.logic.stop_trace.id = 1U;
    application.logic.stop_trace.source = APP_STOP_SOURCE_SAFETY;
    application.logic.stop_trace.reason = SAFETY_UART;
    application.logic.stop_trace.triggers = APP_STOP_TRIGGER_UART | APP_STOP_TRIGGER_STOP;
    application.logic.stop_trace.at_ms = 4001U;
    application.logic.stop_trace.channels[4] = 2000U;
    application.guard_control_gap_ms = 2U;
    application.guard_console_uart_epoch = 1U;
    before = query_calls + enable_calls + disable_calls + torque_calls;
    Command("PITCH GUARD", "SOURCE|SAFETY|REASON|UART|FLAGS|0x00000003");
    CHECK(strstr(response, "GAP_MS|2") && strstr(response, "CONSOLE_EPOCH|1"));
    CHECK(query_calls + enable_calls + disable_calls + torque_calls == before && !stop_calls && !rearm_calls);
    CHECK(PitchConsole_Poll(response, sizeof(response)) && strstr(response, "P|GUARD|"));
    CHECK(!PitchConsole_Poll(response, sizeof(response)));
    application.logic.safety.reason = SAFETY_NONE;
    Command("PITCH GUARD", "REASON|UART"); CHECK(strstr(response, "NOW_REASON|0"));
    CHECK(application.logic.stop_trace.id == 1U);
    test_done = 1U;
}
