/* This links the real console/backend/core. Only HAL transport and app/receiver snapshots are mocked. */
#define test_run backend_standalone_test_run
#include "test_pitch_backend.c"
#undef test_run
#include "pitch_console.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"
#include <stdio.h>

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
static void CommandAt(const char *line, const char *expected, uint32_t caller_line)
{
    CHECK(PitchConsole_Handle(line, response, sizeof(response)));
    if (!strstr(response, expected) && !test_failed_line) { test_failed_line = caller_line; }
    CHECK(strstr(response, expected) != 0);
}
#define Command(line, expected) CommandAt(line, expected, __LINE__)
static void GravityConsoleTests(void)
{
    uint32_t tick, calls;
    PitchStep measured;
    float reference;
    ConsoleReset(6000U);
    Command("PITCH KG -0.2", "P|ERROR|");
    Command("PITCH CHECK", "P|BUSY|CHECK");
    calls = query_calls; Command("PITCH FF", "P|FF|KG|-0.13"); CHECK(query_calls == calls);
    Command("PITCH KG -0.2", "P|BUSY|CHECK"); PitchConsole_CancelPreparation();
    Feed(6000U, 0U); Feed(6020U, 0U); Receiver(6020U, 1000U);
    Command("PITCH COORD 1", "P|OK|COORD"); Command("PITCH ZERO", "P|OK|SOFTWARE_ZERO");
    Read(); reference = state.controller.reference_rad;
    Command("PITCH GHORIZ", "P|OK|HORIZONTAL_SET"); Read();
    CHECK(state.controller.gravity_calibrated && state.controller.reference_rad == reference);
    Command("PITCH GOFF 12", "GOFF_DEG|12");
    Command("PITCH GOFF 181", "P|ERROR|"); Command("PITCH GOFF nan", "P|ERROR|");
    Command("PITCH GOFF 0 extra", "P|ERROR|");
    Command("PITCH KG nan", "P|ERROR|"); Command("PITCH KG inf", "P|ERROR|");
    Command("PITCH KG 11", "P|ERROR|"); Command("PITCH KG -0.2 extra", "P|ERROR|");
    Command("PITCH KG -0.2", "P|OK|KG_SET|KG|-0.2");
    CHECK(strstr(response, "G_CAL|1") && strstr(response, "GFF_NM|0"));
    CHECK(PitchBackend_Request(1U)); request.gimbal_mode = GIMBAL_TEST;
    Receiver(6022U, 2000U); application.logic.safety.operator_armed = 1U;
    Apply(6022U, 1U); Feed(6024U, 1U); Apply(6024U, 1U);
    for (tick = 6026U; tick <= 6044U; tick += 2U) {
        if (tick == 6034U || tick == 6044U) { Feed(tick, 1U); }
        Receiver(tick, 2000U); Apply(tick, 1U);
    }
    Command("PITCH GHORIZ", "P|ERROR|"); Command("PITCH GOFF 0", "P|ERROR|");
    Command("PITCH STEP 30", "P|OK|STEP_STARTED");
    Command("PITCH KG -0.2", "P|OK|KG_SET"); PitchBackend_GetStep(&measured);
    CHECK(measured.phase == PITCH_STEP_RUNNING && measured.gravity_kg_nm == -0.2f);
    CHECK(measured.gravity_offset_deg == 12.0f && measured.gravity_calibrated);
    Command("PITCH KG -0.21", "P|OK|KG_SET"); PitchBackend_GetStep(&measured);
    CHECK(measured.phase == PITCH_STEP_ABORTED && measured.reason == PITCH_STEP_FF_CHANGE);
    CHECK(measured.gravity_kg_nm == -0.2f && measured.gravity_offset_deg == 12.0f);
    Command("PITCH RESULT", "REASON|FF_CHANGE"); CHECK(strstr(response, "KG|-0.2|GOFF_DEG|12|G_CAL|1"));
    calls = query_calls + enable_calls + disable_calls + torque_calls;
    Command("PITCH FF", "P|FF|KG|-0.21"); CHECK(strstr(response, "VI_NM|") && strstr(response, "SUM_NM|"));
    CHECK(query_calls + enable_calls + disable_calls + torque_calls == calls);
    Command("PITCH FF OFF", "P|OK|KG_SET|KG|0");
    Prepare(7000U); Receiver(7020U, 1000U); application.logic.safety.operator_armed = 0U;
    Command("PITCH GHORIZ", "G_CAL|1"); Command("PITCH KG -0.2", "P|OK|KG_SET");
    CHECK(PitchBackend_Request(0U)); Apply(7022U, 0U); Feed(7024U, 0U);
    Receiver(7024U, 1000U); Command("PITCH COORD 1", "P|OK|COORD"); Read();
    CHECK(state.controller.gravity_kg_nm == -0.2f && !state.controller.gravity_calibrated && !state.zero_set);
}

void test_run(void)
{
    char expected_pid[256], same_pid_command[64];
    uint32_t before, tick;
    float reference;
    PitchStep measurement;
    test_stage = 1U;
    ConsoleReset(100U);
    CHECK(!PitchConsole_Handle("YAW STATUS", response, sizeof(response)));
    Command("PITCH STATUS", "COORD_CONFIRMED|0"); CHECK(!query_calls && !disable_calls && !enable_calls);
    CHECK(strstr(response, "DM3507_FAST_OBSERVER_V2F") && strstr(response, "MIN_DEG|-18|MAX_DEG|33"));
    CHECK(strstr(response, "HARD_MIN_DEG|-19|HARD_MAX_DEG|34") && strstr(response, "SUPPORT_REQUIRED|1\r\n"));
    Command("PITCH", "P|STATE|DM3507_FAST_OBSERVER_V2F"); CHECK(strlen(response) < 300U);
    Command("PITCH HELP", "P|HELP|READ|PITCH;PID;RESULT;STATUS");
    Command("PITCH ?", "PITCH_ONLY|COORD 1;GHORIZ");
    (void)snprintf(expected_pid, sizeof(expected_pid), "POS|%.6g,%.6g,%.6g|VEL|%.6g,%.6g,%.6g|KG|%.6g",
        (double)APP_PITCH_POSITION_KP, (double)APP_PITCH_POSITION_KI, (double)APP_PITCH_POSITION_KD,
        (double)APP_PITCH_SPEED_KP, (double)APP_PITCH_SPEED_KI, (double)APP_PITCH_SPEED_KD,
        (double)APP_PITCH_GRAVITY_KG_NM);
    Command("PITCH PID", expected_pid);
    Command("PITCH BAD", "UNKNOWN_COMMAND|USE|PITCH HELP");
    CHECK(!query_calls && !enable_calls && !disable_calls && !torque_calls && !stop_calls && !rearm_calls);
    Command("PITCH READY", "COORD_UNCONFIRMED"); CHECK(!query_calls);
    Command("PITCH CHECK", "P|BUSY|CHECK"); CHECK(query_calls == 1U);
    Command("PITCH", "P|STATE|"); Command("PITCH HELP", "P|HELP|"); Command("PITCH PID", "P|PID|");
    CHECK(query_calls == 1U && !enable_calls && !disable_calls);
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

    Command("PITCH GHORIZ", "G_CAL|1");
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
    CHECK(strstr(response, "|BAND02|0.2|") && strstr(response, "|TIME_LIMIT_MS|300|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2|"));
    CHECK(strstr(response, "|ASSESSMENT|FINAL_SETTLE|") && !strstr(response, "PASS05"));
    (void)snprintf(same_pid_command, sizeof(same_pid_command), "PITCH PKP %.9g", (double)APP_PITCH_POSITION_KP);
    Command(same_pid_command, "P|OK|PARAMETER"); PitchBackend_GetStep(&measurement);
    CHECK(measurement.phase == PITCH_STEP_RUNNING);
    Command("PITCH PKP 13", "P|OK|PARAMETER"); PitchBackend_GetStep(&measurement); Read();
    CHECK(measurement.phase == PITCH_STEP_ABORTED && state.controller.target_deg == 30.0f);
    CHECK(PitchConsole_Poll(response, sizeof(response)) && strstr(response, "REASON|PID_CHANGE"));
    CHECK(!PitchConsole_Poll(response, sizeof(response)));
    Command("PITCH TARGET 0", "P|OK|TARGET|0.000");
    Read(); CHECK(state.controller.target_deg == 0.0f);
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
    GravityConsoleTests();
    ConsoleReset(8000U);
    Command("PITCH SHAPE", "P|ERROR|UNKNOWN_COMMAND");
    Command("PITCH GUARD", "P|ERROR|UNKNOWN_COMMAND");
    Command("PITCH BRAKE", "P|BRAKE|ENABLED|1");
    Command("PITCH BRAKE OFF", "P|OK|BRAKE|OFF");
    Command("PITCH BRAKE ON", "P|OK|BRAKE|ON");
    Command("PITCH BRAKE bad", "P|ERROR|");
    CHECK(!query_calls && !enable_calls && !disable_calls && !torque_calls);
    test_stage = 5U;
    Activate(9000U); Receiver(9044U, 2000U); application.logic.safety.operator_armed = 1U;
    Command("PITCH STEP", "P|OK|STEP_STARTED");
    Command("PITCH RESULT", "SHAPE_MS|0|SHAPEV|0|TERMINAL_BRAKE|1");
    Command("PITCH BRAKE OFF", "P|ERROR|BRAKE|USE_ON_OFF|FINISH_STEP_FIRST");
    before = state.controller.cascade.revision;
    Command("PITCH HOME", "P|OK|HOME"); Read();
    CHECK(state.controller.target_deg == 0.0f && state.controller.cascade.revision == before);
    PitchBackend_GetStep(&measurement);
    CHECK(measurement.phase == PITCH_STEP_ABORTED && measurement.reason == PITCH_STEP_TARGET_CHANGE && !measurement.pass);
    PitchBackend_Stop();
    Command("PITCH BRAKE OFF", "P|OK|BRAKE|OFF"); Read();
    CHECK(!state.controller.brake.enabled && !state.controller.brake.tracking);
    test_done = 1U;
}
