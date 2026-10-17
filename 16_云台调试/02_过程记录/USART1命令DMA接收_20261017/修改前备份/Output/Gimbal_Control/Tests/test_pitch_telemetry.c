/* Exercise the real USART router and both data groups, using deterministic snapshots. */
#define test_run legacy_console_test_run
#include "test_app_console.c"
#undef test_run
#include "pitch_backend.h"
#include "pitch_console.h"
#include "yaw_backend.h"
#include "yaw_console.h"
#include <stdlib.h>
static PitchBackendState pitch_sample;
static PitchStep pitch_step_sample;
static YawBackendState yaw_sample;
static YawStep yaw_step_sample;
static uint32_t pitch_commands, yaw_commands, pitch_cancel_count;
volatile uint32_t test_max_wire_bytes, test_max_wire_us;
void PitchConsole_Init(void) { }
void PitchConsole_CancelPreparation(void) { ++pitch_cancel_count; }
uint8_t PitchConsole_Poll(char *response, size_t capacity) { (void)response; (void)capacity; return 0U; }
uint8_t PitchConsole_Handle(const char *text, char *response, size_t capacity)
{
    if (strncmp(text, "PITCH ", 6U)) { return 0U; }
    ++pitch_commands; (void)snprintf(response, capacity, "P|MOCK\r\n"); return 1U;
}
void PitchBackend_GetState(PitchBackendState *out) { *out = pitch_sample; }
void PitchBackend_GetStep(PitchStep *out) { *out = pitch_step_sample; }
void YawConsole_CancelPreparation(void) { }
uint8_t YawConsole_Poll(char *response, size_t capacity) { (void)response; (void)capacity; return 0U; }
uint8_t YawConsole_Handle(const char *text, char *response, size_t capacity)
{
    if (strncmp(text, "YAW ", 4U)) { return 0U; }
    ++yaw_commands; (void)snprintf(response, capacity, "Y|MOCK\r\n"); return 1U;
}
void YawBackend_GetState(YawBackendState *out) { *out = yaw_sample; }
void YawBackend_GetStep(YawStep *out) { *out = yaw_step_sample; }
static float columns[48];
static void parse(const char *group)
{
    uint32_t i;
    const char *cursor = output + strlen(group);
    char *end;
    CHECK(strncmp(output, group, strlen(group)) == 0);
    for (i = 0U; i < 48U; ++i) {
        columns[i] = strtof(cursor, &end); CHECK(end != cursor);
        if (i < 47U) { CHECK(*end == ','); cursor = end + 1; }
        else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    if (output_length > test_max_wire_bytes) { test_max_wire_bytes = output_length; }
    test_max_wire_us = (test_max_wire_bytes * 10000000UL + APP_CONSOLE_BAUDRATE - 1U) / APP_CONSOLE_BAUDRATE;
    CHECK(test_max_wire_us < 4500U);
}
void test_run(void)
{
    uint32_t i, frames = 0U;
    test_stage = 1U;
    AppConsole_CreateQueue(); AppConsole_Start();
    pitch_sample.controller.feedback_divisor = 1U;
    yaw_sample.controller.angle_deg = 11.25f; yaw_sample.controller.cascade.position.kp = 10.0f;
    now = 100U; clear_output(); AppConsole_Poll(); parse("gimbal:");
    CHECK(columns[0] == 11.25f && columns[7] == 10.0f);
    clear_output(); command_line("TELEM PITCH"); CHECK(strstr(output, "GROUP|pitch|COLS|48") != 0);
    pitch_sample.controller.angle_deg = -1.234f; pitch_sample.controller.target_deg = 30.0f;
    pitch_sample.controller.cascade.target_velocity_dps = 360.0f;
    pitch_sample.controller.cascade.velocity_dps = 90.0f;
    pitch_sample.controller.velocity_dps = 92.0f;
    pitch_sample.controller.torque_nm = 7.2f; pitch_sample.feedback.torque_nm = 6.7f;
    pitch_sample.controller.cascade.position.kp = 12.0f;
    pitch_sample.controller.cascade.position.ki = 0.2f;
    pitch_sample.controller.cascade.position.kd = 0.02f;
    pitch_sample.controller.cascade.speed.kp = 0.02f;
    pitch_sample.controller.cascade.speed.ki = 0.03f;
    pitch_sample.controller.cascade.speed.kd = 0.00005f;
    pitch_sample.online = pitch_sample.active = pitch_sample.requested = pitch_sample.zero_set = 1U;
    pitch_sample.coord_confirmed = 1U; pitch_sample.feedback.status = 1U;
    pitch_sample.feedback.temp_mos = 38U; pitch_sample.feedback.temp_rotor = 41U;
    pitch_sample.rx_count = 210U; pitch_sample.feedback_age_ms = 2U;
    pitch_sample.feedback.position_rad = 0.2f; pitch_sample.feedback.velocity_rad_s = 1.6f;
    pitch_step_sample.id = 3U; pitch_step_sample.phase = PITCH_STEP_DONE;
    pitch_step_sample.response_ms = 290U; pitch_step_sample.settle_ms = 800U;
    pitch_step_sample.overshoot_deg = 0.1f; pitch_step_sample.steady_error_deg = -0.02f;
    pitch_step_sample.sse_valid = pitch_step_sample.pass = 1U;
    pitch_step_sample.t10_ms = 30U; pitch_step_sample.t90_ms = 180U;
    now = 105U; clear_output(); AppConsole_Poll(); parse("pitch:");
    CHECK(columns[0] < -1.233f && columns[1] == 30 && columns[2] == 360);
    CHECK(columns[3] == 90 && columns[4] == 92 && columns[5] > 7.19f && columns[6] > 6.69f);
    CHECK(columns[7] == 12 && columns[8] == 0.2f && columns[9] == 0.02f);
    CHECK(columns[10] == 0.02f && columns[11] == 0.03f && columns[12] == 0.00005f);
    CHECK(columns[14] == 1 && columns[15] == 1 && columns[17] == 1 && columns[19] == 2);
    CHECK(columns[21] == 1 && columns[23] == 1 && columns[24] == 3);
    CHECK(columns[26] == 290 && columns[27] == 800 && columns[31] == 1 && columns[32] == 1);
    CHECK(columns[33] == 180 && columns[34] == 150 && columns[37] == 1 && columns[38] == 210);
    CHECK(columns[46] == 38 && columns[47] == 41);
    test_stage = 2U;
    clear_output(); command_line("PITCH VKP 0.02"); CHECK(pitch_commands == 1 && yaw_commands == 0);
    clear_output(); command_line("YAW VKP 0.0037"); CHECK(pitch_commands == 1 && yaw_commands == 1);
    pitch_step_sample.response_ms = pitch_step_sample.t90_ms = PITCH_STEP_UNREACHED;
    now += 5U; clear_output(); AppConsole_Poll(); parse("pitch:");
    CHECK(columns[26] == -1 && columns[33] == -1 && columns[34] == -1);
    for (i = 0U; i < 200U; ++i) {
        now += 5U; clear_output(); AppConsole_Poll(); parse("pitch:");
        ++frames; CHECK(columns[0] < -1.233f); /* no made-up fresh samples */
    }
    CHECK(frames == 200U);
    now += 1000U; clear_output(); AppConsole_Poll(); parse("pitch:");
    clear_output(); AppConsole_Poll(); CHECK(!output_length);
    test_stage = 3U;
    pitch_sample.controller.angle_deg = -49.9999f;
    pitch_sample.controller.cascade.target_velocity_dps = -1234567890.0f;
    pitch_sample.controller.cascade.velocity_dps = -1300.123f; pitch_sample.controller.velocity_dps = -11459.2f;
    pitch_sample.controller.torque_nm = pitch_sample.feedback.torque_nm = -10.0f;
    pitch_sample.controller.cascade.position.kp = 100; pitch_sample.controller.cascade.position.ki = 30;
    pitch_sample.controller.cascade.position.kd = 10; pitch_sample.controller.cascade.speed.kp = 1;
    pitch_sample.controller.cascade.speed.ki = 10; pitch_sample.controller.cascade.speed.kd = 0.02f;
    pitch_sample.controller.faults = pitch_sample.feedback_age_ms = pitch_sample.rx_count = 0xFFFFFFFFUL;
    pitch_sample.feedback.position_rad = -3.141593f; pitch_sample.feedback.velocity_rad_s = -200;
    pitch_sample.feedback.temp_mos = pitch_sample.feedback.temp_rotor = 255U;
    pitch_step_sample.id = 0xFFFFFFFFUL; pitch_step_sample.response_ms = pitch_step_sample.settle_ms = 5000U;
    pitch_step_sample.t90_ms = 5000U; pitch_step_sample.t10_ms = 10U;
    pitch_step_sample.overshoot_deg = 49.9999f; pitch_step_sample.overshoot_pct = 166.666f;
    pitch_step_sample.steady_error_deg = -79.9999f;
    now += 5U; clear_output(); AppConsole_Poll(); parse("pitch:");
    CHECK(columns[0] < -49.999f && columns[2] < -1.234e9f && columns[5] == -10);
    CHECK(columns[35] < -3.14f && columns[36] == -200 && columns[46] == 255);
    test_stage = 4U;
    clear_output(); command_line("TELEM YAW"); CHECK(strstr(output, "GROUP|gimbal") != 0);
    now += 5U; clear_output(); AppConsole_Poll(); parse("gimbal:"); CHECK(columns[7] == 10);
    clear_output(); command_line("STOP"); CHECK(stops == 1 && pitch_cancel_count == 1);
    test_done = 1U;
}
