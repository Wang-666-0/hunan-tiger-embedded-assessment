#include "app_config.h"
#if APP_PITCH_ENABLED
#define PITCH_TELEMETRY_TEST_ENTRY pitch_telemetry_baseline_test_run
#include "test_pitch_telemetry.c"
#define step_sample yaw_step_sample
#else
#define YAW_TELEMETRY_TEST_ENTRY yaw_telemetry_baseline_test_run
#include "test_yaw_telemetry.c"
#endif
static float demo_columns[75];
static void frame(uint8_t demo)
{
    const char *group = demo ? "gimbal_demo:" : "gimbal:", *cursor;
    unsigned i, count = demo ? 75U : 72U;
    char *end;
    now += 5U; clear_output(); AppConsole_Poll();
    CHECK(strncmp(output, group, strlen(group)) == 0);
    cursor = output + strlen(group);
    for (i = 0U; i < count; ++i) {
        demo_columns[i] = strtof(cursor, &end); CHECK(end != cursor);
        if (i + 1U < count) { CHECK(*end == ','); cursor = end + 1; }
        else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    CHECK(output_length * 10000000UL / APP_CONSOLE_BAUDRATE < 4500U);
}
void test_run(void)
{
    unsigned i;
    YawStep result;
    YawBackendState state;
    test_stage = 1U;
    AppConsole_CreateQueue(); AppConsole_Start(); now = 100U;
    yaw_sample.active = yaw_sample.online = 1U;
    yaw_sample.controller.angle_deg = yaw_sample.controller.target_deg = 0.0f;
    yaw_sample.controller.cascade.position.kp = APP_YAW_POSITION_KP;
    yaw_sample.controller.cascade.position.ki = APP_YAW_POSITION_KI;
    yaw_sample.controller.cascade.speed.kp = APP_YAW_SPEED_KP;
    yaw_sample.controller.cascade.speed.ki = APP_YAW_SPEED_KI;
    step_sample.id = 1U; step_sample.phase = YAW_STEP_DONE; step_sample.response_ms = 301U;
    step_sample.settle_ms = 800U; step_sample.overshoot_deg = 0.3f;
    frame(0U); CHECK(demo_columns[0] == 0.0f);
    clear_output(); command_line("YAW SIM"); CHECK(strstr(output, "ENABLED|0|ACTIVE|0|ARMED|0|"));
    clear_output(); command_line("YAW SIM ON");
    CHECK(strstr(output, "NOT_A_MEASUREMENT|1") && strstr(output, "TRIGGER_MS|300"));
    CHECK(strstr(output, "MODE|GUARDED_CONTINUOUS|AUTO_OFF|0"));
    CHECK(strstr(output, "RISE_STYLE|STAIRCASE|RISE_SAMPLE_MS|"));
    frame(1U); CHECK(demo_columns[72] == 0 && demo_columns[73] == 0 && demo_columns[74] == 1);
    clear_output(); command_line("YAW VIEW"); CHECK(strstr(output, "GROUP|gimbal_demo|COLS|75|"));
#if APP_PITCH_ENABLED
    CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_PITCH);
#endif
    step_sample.phase = YAW_STEP_RUNNING; step_sample.id++;
    step_sample.start_ms = now + 5U; step_sample.start_deg = 0; step_sample.target_deg = 30;
    yaw_sample.controller.target_deg = 30;
    frame(1U); CHECK(demo_columns[0] == 0 && demo_columns[1] == 30 && demo_columns[73] == 1);
    yaw_sample.controller.angle_deg = 27.0f; state = yaw_sample; result = step_sample;
    for (i = 0U; i < 65U; ++i) { frame(1U); }
    CHECK(demo_columns[0] >= 29.85f && demo_columns[0] <= 30.15f && demo_columns[72] == 27.0f);
    CHECK(demo_columns[7] == APP_YAW_POSITION_KP && demo_columns[8] == APP_YAW_POSITION_KI);
    CHECK(demo_columns[10] == APP_YAW_SPEED_KP && demo_columns[11] == APP_YAW_SPEED_KI);
    CHECK(demo_columns[26] == 301 && demo_columns[27] == 800 && demo_columns[28] == 0.3f);
    CHECK(!memcmp(&state, &yaw_sample, sizeof(state)) && !memcmp(&result, &step_sample, sizeof(result)));
    clear_output(); command_line("YAW SIM"); CHECK(strstr(output, "DEMO_DONE|1|DEMO_LIMIT|2"));
    clear_output(); command_line("YAW SIM ON");
    CHECK(strstr(output, "ACTIVE|1|") && strstr(output, "DEMO_DONE|1|DEMO_LIMIT|2|BATCH_DONE|1|"));
    /* A second demo is a HOME return on the old backend step ID. */
    yaw_sample.controller.target_deg = 0.0f;
    step_sample.phase = YAW_STEP_ABORTED; step_sample.reason = YAW_STEP_TARGET_CHANGE;
    frame(1U); CHECK(demo_columns[73] == 1 && demo_columns[1] == 0);
    for (i = 0U; i < 65U; ++i) { frame(1U); }
    CHECK(demo_columns[0] >= -0.15f && demo_columns[0] <= 0.15f);
    clear_output(); command_line("YAW SIM"); CHECK(strstr(output, "DEMO_DONE|2|"));
    step_sample.phase = YAW_STEP_RUNNING; step_sample.id++; step_sample.start_ms = now + 5U;
    step_sample.start_deg = 0.0f; yaw_sample.controller.target_deg = 30.0f;
    frame(1U); CHECK(demo_columns[0] == 27.0f && demo_columns[72] == 27.0f && demo_columns[73] == 0);
    frame(1U); CHECK(demo_columns[0] == 27.0f);
    clear_output(); command_line("YAW SIM"); CHECK(strstr(output, "ENABLED|1|ACTIVE|0|ARMED|0|"));
    CHECK(strstr(output, "DEMO_DONE|2|DEMO_LIMIT|2|BATCH_DONE|0|NEXT_REAL|0|"));
    test_stage = 2U;
    clear_output(); command_line("YAW SIM ON"); frame(1U);
    step_sample.phase = YAW_STEP_DONE; frame(1U); CHECK(demo_columns[74] == 1);
    step_sample.phase = YAW_STEP_RUNNING; step_sample.id++; step_sample.start_ms = now + 5U;
    frame(1U); CHECK(demo_columns[73] == 1);
    clear_output(); command_line("TELEM OFF"); clear_output(); now += 5U; AppConsole_Poll(); CHECK(!output_length);
    clear_output(); command_line("TELEM ON"); frame(1U); CHECK(demo_columns[73] == 0 && demo_columns[74] == 0);
    clear_output(); command_line("YAW SIM"); CHECK(strstr(output, "ENABLED|1|ACTIVE|0|ARMED|0|"));
    clear_output(); command_line("YAW SIM OFF"); frame(0U); CHECK(demo_columns[0] == 27.0f);
    CHECK(!yaw_commands && !stops && !selections); /* No control side effect from demo commands. */
    test_done = 1U;
}
