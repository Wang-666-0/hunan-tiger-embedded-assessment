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
    Command("PITCH PID", "POS|10.678,24.153,0.002|VEL|0.00406,0.0005,1e-06|KG|-0.13");
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
    Command("PITCH PKP 10.678", "P|OK|PARAMETER"); PitchBackend_GetStep(&measurement);
    CHECK(measurement.phase == PITCH_STEP_RUNNING);
    Command("PITCH PKP 13", "P|OK|PARAMETER"); PitchBackend_GetStep(&measurement); Read();
    CHECK(measurement.phase == PITCH_STEP_ABORTED && state.controller.target_deg == 30.0f);
    CHECK(PitchConsole_Poll(response, sizeof(response)) && strstr(response, "REASON|PID_CHANGE"));
    CHECK(!PitchConsole_Poll(response, sizeof(response)));
    Command("PITCH TARGET 0", "P|OK|TARGET|0.000");
    Read(); CHECK(state.controller.target_deg == 0.0f && !state.controller.shape.active);
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
    test_stage = 5U; GravityConsoleTests();
    test_stage = 6U;
    ConsoleReset(5000U); application.logic.stop_trace.id = 1U;
    application.logic.stop_trace.source = APP_STOP_SOURCE_FEEDER;
    Command("PITCH GUARD", "SOURCE|FEEDER");
    CHECK(!query_calls && !enable_calls && !disable_calls && !torque_calls && !stop_calls && !rearm_calls);
    application.logic.stop_trace.source = 255U;
    Command("PITCH GUARD", "SOURCE|UNKNOWN");
    CHECK(!query_calls && !enable_calls && !disable_calls && !torque_calls && !stop_calls && !rearm_calls);
    test_stage = 7U;
    ConsoleReset(8000U);
    Command("PITCH SHAPE", "P|SHAPE|MS|0|VEL_FF|0|ACTIVE|0");
    CHECK(strstr(response, "T0|STEP_ACCEPTED|ASSESSMENT|FINAL_GOAL"));
    CHECK(!query_calls && !enable_calls && !disable_calls && !torque_calls);
    Command("PITCH SHAPE 180", "P|OK|SHAPE|MS|180");
    Command("PITCH SHAPEV 0.3", "P|OK|SHAPEV|0.3");
    Command("PITCH SHAPEV", "P|SHAPEV|0.3");
    Command("PITCH SHAPE 99", "P|ERROR|"); Command("PITCH SHAPE 251", "P|ERROR|");
    Command("PITCH SHAPE 200.5", "P|ERROR|"); Command("PITCH SHAPE 200 extra", "P|ERROR|");
    Command("PITCH SHAPE nan", "P|ERROR|"); Command("PITCH SHAPE inf", "P|ERROR|");
    Command("PITCH SHAPEV -0.1", "P|ERROR|"); Command("PITCH SHAPEV 1.1", "P|ERROR|");
    Command("PITCH SHAPEV nan", "P|ERROR|"); Command("PITCH SHAPEV 0.2 extra", "P|ERROR|");
    diagnostic_mode = 1U;
    Command("PITCH SHAPE 200", "P|ERROR|"); Command("PITCH SHAPEV 0.2", "P|ERROR|");
    diagnostic_mode = 0U;
    Command("PITCH BRAKE", "P|BRAKE|MODE|TERMINAL_AND_RETURN|ENABLED|1");
    Command("PITCH BRAKE OFF", "P|OK|BRAKE|OFF");
    Command("PITCH BRAKE ON", "P|OK|BRAKE|ON");
    Activate(9000U); Receiver(9044U, 2000U); application.logic.safety.operator_armed = 1U;
    CHECK(PitchBackend_StartStep(30.0f));
    Command("PITCH SHAPE 180", "P|ERROR|"); Command("PITCH SHAPEV 0.3", "P|ERROR|");
    Command("PITCH SHAPE 200", "P|OK|SHAPE|MS|200"); Command("PITCH SHAPEV 0.2", "P|OK|SHAPEV|0.2");
    Command("PITCH RESULT", "SHAPE_MS|200|SHAPEV|0.2");
    Command("PITCH SHAPE", "ACTIVE|1|REF_DEG|");
    Command("PITCH FF", "SHAPE_ACTIVE|1|REF_DEG|");
    Command("PITCH TARGET 30", "P|OK|TARGET|30.000");
    PitchBackend_GetStep(&measurement); CHECK(measurement.phase == PITCH_STEP_RUNNING && measurement.start_ms == 9044U);
    Command("PITCH HOME", "P|ERROR|HOME_REQUIRES_ARMED_ACTIVE_IDLE_PROFILE_AND_STATIONARY");
    for (tick = 9046U; tick <= 9284U; tick += 2U) {
        CanFrame frame;
        float u = (float)(tick - 9044U) / 200.0f;
        float angle = u >= 1.0f ? 30.0f : 30.0f * u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
        frame = Feedback(tick, 1U, angle, 0.0f);
        now_ms = tick; CHECK(PitchBackend_OnFrame(&frame)); Receiver(tick, 2000U); Apply(tick, 1U);
    }
    Command("PITCH SHAPE", "ACTIVE|0|REF_DEG|30|REF_DPS|0");
    Command("PITCH BRAKE", "LAND_SEEN|1|LAND_MS|");
    CHECK(strstr(response, "VI_START_NM|") && strstr(response, "LAND_OUT_NM|") && strstr(response, "DIP_DEG|"));
    CHECK(strstr(response, "V_FILTER_MS|2|A_FILTER_MS|20|"));
    CHECK(strstr(response, "DIRECT_STEP_DEFAULT|1\r\n") && strlen(response) < sizeof(response) - 1U);
    Command("PITCH SHAPE 180", "P|ERROR|"); /* Full five-second measurement still owns the settings. */
    Command("PITCH HOME", "P|OK|HOME");
    before = state.controller.cascade.revision;
    Command("PITCH BRAKE ON", "P|OK|BRAKE|ON"); Read();
    CHECK(state.controller.cascade.revision == before && state.controller.shape.active);
    PitchBackend_GetStep(&measurement);
    CHECK(measurement.reason == PITCH_STEP_TARGET_CHANGE && measurement.shape_ms == 200U);
    CHECK(measurement.shape_velocity_gain == 0.2f && measurement.target_deg == 30.0f && measurement.start_ms == 9044U);
    test_stage = 8U;
    Activate(10000U); Receiver(10044U, 2000U); application.logic.safety.operator_armed = 1U;
    CHECK(PitchBackend_SetShape(0U, 0.0f));
    Command("PITCH BRAKE", "MODE|TERMINAL_AND_RETURN|ENABLED|1");
    CHECK(strstr(response, "WINDOW_DEG|4|GAIN|0.002|MAX_NM|0.12"));
    CHECK(strstr(response, "RETURN_MS|250|RETURN_GAIN|0.01|RETURN_MAX_NM|0.3"));
    CHECK(strstr(response, "PEAK_NM|0|RETURN_PEAK_NM|0|N|0|RETURN_N|0"));
    Command("PITCH BRAKE bad", "P|ERROR|"); Command("PITCH BRAKE ON extra", "P|ERROR|");
    diagnostic_mode = 1U; Command("PITCH BRAKE OFF", "P|ERROR|"); diagnostic_mode = 0U;
    Command("PITCH STEP", "P|OK|STEP_STARTED");
    Apply(10046U, 1U); Receiver(10046U, 2000U);
    CHECK(!state.controller.shape.active && state.controller.shape.position_deg == 30.0f);
    CHECK(state.controller.cascade.target_velocity_dps > 300.0f && state.controller.brake.tracking);
    Command("PITCH BRAKE OFF", "P|ERROR|BRAKE|USE_ON_OFF|FINISH_STEP_FIRST");
    Command("PITCH BRAKE ON", "P|OK|BRAKE|ON");
    Command("PITCH RESULT", "SHAPE_MS|0|SHAPEV|0|TERMINAL_BRAKE|1");
    PitchBackend_GetStep(&measurement);
    CHECK(measurement.start_ms == 10044U && measurement.target_deg == 30.0f && measurement.pitch_brake_enabled);
    PitchBackend_Stop();
    Command("PITCH BRAKE OFF", "P|OK|BRAKE|OFF"); Read();
    CHECK(!state.controller.brake.enabled && !state.controller.brake.tracking && state.controller.brake.torque_nm == 0.0f);
    test_done = 1U;
}
