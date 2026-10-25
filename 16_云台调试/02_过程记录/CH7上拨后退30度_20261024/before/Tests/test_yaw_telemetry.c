/* USART替身验证正式48列和原命令分发。 */
#define test_run legacy_console_test_run
#include "test_app_console.c"
#undef test_run
#include "yaw_backend.h"
#include "yaw_console.h"
#include <stdlib.h>
static YawBackendState yaw_sample;
static YawStep step_sample;
static uint32_t yaw_commands;
volatile uint32_t test_max_wire_bytes, test_max_wire_us;
void YawConsole_CancelPreparation(void) { }
uint8_t YawConsole_Poll(char *response, size_t capacity) { (void)response; (void)capacity; return 0U; }
void YawBackend_GetState(YawBackendState *out) { *out = yaw_sample; }
void YawBackend_GetStep(YawStep *out) { *out = step_sample; }
uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity)
{
    if (strncmp(line, "YAW ", 4U)) { return 0U; }
    ++yaw_commands; (void)snprintf(response, capacity, "Y|MOCK\r\n"); return 1U;
}
static float columns[72];
static void ParseColumns(void)
{
    const char *cursor = output + 7U;
    char *end;
    uint32_t index;
    CHECK(strncmp(output, "gimbal:", 7U) == 0);
    for (index = 0U; index < 72U; ++index) {
        columns[index] = strtof(cursor, &end); CHECK(end != cursor);
        if (index < 71U) { CHECK(*end == ','); cursor = end + 1U; }
        else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    if (output_length > test_max_wire_bytes) { test_max_wire_bytes = output_length; }
    test_max_wire_us = (test_max_wire_bytes * 10000000UL + APP_CONSOLE_BAUDRATE - 1U) / APP_CONSOLE_BAUDRATE;
    CHECK(test_max_wire_us < 4500U); /* 64列仍小于5ms线路预算。 */
}
#ifndef YAW_TELEMETRY_TEST_ENTRY
#define YAW_TELEMETRY_TEST_ENTRY test_run
#endif
void YAW_TELEMETRY_TEST_ENTRY(void)
{
    uint32_t index, frames;
    test_stage = 1U;
    CHECK(APP_TELEMETRY_PERIOD_MS == 5U && APP_CONSOLE_PERIOD_MS == 5U && APP_CONSOLE_BAUDRATE == 921600U);
    AppConsole_CreateQueue(); AppConsole_Start(); clear_output(); command_line("YAW SIM OFF");
    memset(&yaw_sample, 0, sizeof(yaw_sample)); memset(&step_sample, 0, sizeof(step_sample));
    yaw_sample.controller.angle_deg = 1.234f; yaw_sample.controller.target_deg = 30.0f;
    yaw_sample.controller.cascade.target_velocity_dps = 180.0f;
    yaw_sample.controller.cascade.velocity_dps = 50.0f; yaw_sample.controller.velocity_dps = 53.16f;
    yaw_sample.controller.torque_nm = 0.1f; yaw_sample.feedback.torque_nm = 0.09524f;
    yaw_sample.controller.cascade.position.kp = 8.0f;
    yaw_sample.controller.cascade.position.ki = 0.1f; yaw_sample.controller.cascade.position.kd = 0.02f;
    yaw_sample.controller.cascade.speed.kp = 0.001f;
    yaw_sample.controller.cascade.speed.ki = 0.004f; yaw_sample.controller.cascade.speed.kd = 0.000001f;
    yaw_sample.controller.cascade.velocity_limit_dps = 180.0f;
    yaw_sample.controller.cascade.torque_limited = yaw_sample.controller.cascade.velocity_limited = 1U;
    yaw_sample.online = yaw_sample.active = yaw_sample.requested = yaw_sample.zero_set = 1U;
    yaw_sample.feedback.status = 1U; yaw_sample.controller.faults = 256U;
    yaw_sample.feedback_age_ms = 12U; yaw_sample.stop_pending = 1U;
    yaw_sample.feedback.driver_temp_c = -3; yaw_sample.feedback.motor_temp_c = 42;
    step_sample.id = 14U; step_sample.phase = YAW_STEP_DONE;
    step_sample.response_ms = 240U; step_sample.settle_ms = YAW_STEP_UNREACHED;
    step_sample.t10_ms = 50U; step_sample.t90_ms = 180U;
    step_sample.overshoot_deg = 1.23f; step_sample.overshoot_pct = 4.1f; step_sample.steady_error_deg = -0.2f;
    yaw_sample.controller.ff.enabled = 1U; yaw_sample.controller.ff.acceleration_gain = 0.000002f;
    yaw_sample.controller.ff.limit_nm = 0.1f; yaw_sample.controller.cascade.feedforward_nm = -0.015f;
    yaw_sample.controller.pid_torque_nm = 0.115f; yaw_sample.controller.cascade.speed_i = 0.04f;
    yaw_sample.controller.cascade.position_i = 0.25f; yaw_sample.controller.cascade.feedforward_limited = 1U;
    yaw_sample.controller.last_feedback_ms = 4294967200UL;
    yaw_sample.controller.shape.duration_ms = 160;
    yaw_sample.controller.shape.active = 1;
    yaw_sample.controller.shape.position_deg = 15;
    yaw_sample.controller.shape.velocity_dps = 351.5625f;
    yaw_sample.controller.shape.acceleration_dps2 = -3000;
    yaw_sample.controller.shape.velocity_gain = 0.25f;
    jog_state.active = 1U; jog_state.speed.filtered_erpm = 42000;
    jog_state.speed_right.filtered_erpm = 41300; jog_state.health.faults = 16U;
    app_armed = output_allowed = 1U;
    now = 100U; clear_output(); AppConsole_Poll(); ParseColumns();
    CHECK(columns[0] > 1.233f && columns[0] < 1.235f && columns[1] == 30.0f);
    CHECK(columns[2] == 180 && columns[3] == 50 && columns[4] > 53.15f);
    CHECK(columns[5] > 0.0999f && columns[6] > 0.0952f);
    CHECK(columns[7] == 8 && columns[8] > 0.0999f && columns[9] > 0.0199f);
    CHECK(columns[10] > 0.00099f && columns[11] > 0.00399f && columns[12] == 0.000001f && columns[13] == 180);
    CHECK(columns[14] == 1 && columns[15] == 1 && columns[16] == 1 && columns[17] == 1);
    CHECK(columns[18] == 256 && columns[19] == 12 && columns[20] == 1 && columns[21] == 1);
    CHECK(columns[22] == 1 && columns[23] == 1 && columns[24] == 14 && columns[25] == YAW_STEP_DONE);
    CHECK(columns[26] == 240 && columns[27] == -1 && columns[28] > 1.229f && columns[29] > 4.099f);
    CHECK(columns[30] < -0.199f && columns[31] == 0 && columns[32] == 0 && columns[33] == 180 && columns[34] == 130);
    CHECK(columns[35] == 1 && columns[36] == 6000 && columns[37] == 5900 && columns[38] == 16);
    CHECK(columns[39] == 1 && columns[40] == 1 && columns[41] == 1000 && columns[42] == 1500);
    CHECK(columns[43] == 1500 && columns[44] == 2000 && columns[45] == 6 && columns[46] == -3 && columns[47] == 42);
    CHECK(columns[48] == 1 && columns[49] == 0 && columns[50] == 0 && columns[51] == 0);
    CHECK(columns[52] == 0 && columns[53] == 0 && columns[54] == 0.1f && columns[55] == -0.015f);
    CHECK(columns[56] == 0.115f && columns[57] == 0.04f && columns[58] == 0.25f && columns[59] == 1);
    CHECK(columns[60] == 240 && columns[61] == -1 && columns[62] == 0);
    CHECK(columns[60] == columns[26] && columns[61] == columns[27] && columns[62] == columns[32]);
    CHECK(columns[64] == 160 && columns[65] == 1 && columns[66] == 15);
    CHECK(columns[67] > 351.55f && columns[67] < 351.58f && columns[68] == -3000);
    CHECK(columns[69] > 13.765f && columns[69] < 13.767f);
    CHECK(columns[70] == 0.25f && columns[71] == 0.000002f);
    clear_output(); command_line("YAW VIEW");
    CHECK(strstr(output, "RESP02_I|60|SETTLE02_I|61|") && strstr(output, "PASS02_I|62|BAND_DEG|0.2|"));
    CHECK(!strstr(output, "BAND05"));
    step_sample.response_ms = 190; step_sample.settle_ms = 240; step_sample.pass = step_sample.sse_valid = 1;
    step_sample.overshoot_deg = 0.1f; step_sample.steady_error_deg = 0.02f;
    clear_output(); now += 5; AppConsole_Poll(); ParseColumns();
    CHECK(columns[60] == 190 && columns[61] == 240 && columns[62] == 1);
    CHECK(columns[26] == columns[60] && columns[27] == columns[61] && columns[32] == columns[62]);
    test_stage = 2U;
    clear_output(); command_line("YAW VKP 0.001000");
    CHECK(yaw_commands == 1 && strcmp(output, "Y|MOCK\r\n") == 0);
    clear_output(); command_line("STOP"); CHECK(stops == 1 && !output_allowed);
    step_sample.response_ms = step_sample.t90_ms = YAW_STEP_UNREACHED;
    clear_output(); now += 50U; AppConsole_Poll(); ParseColumns();
    CHECK(columns[26] == -1 && columns[33] == -1 && columns[34] == -1);
    test_stage = 3U;
    clear_output(); now = 200U; AppConsole_Poll(); ParseColumns();
    for (now = 201U; now < 205U; ++now) { clear_output(); AppConsole_Poll(); CHECK(output_length == 0U); }
    clear_output(); now = 205U; AppConsole_Poll(); ParseColumns();
    clear_output(); AppConsole_Poll(); CHECK(output_length == 0U);
    clear_output(); now = 211U; AppConsole_Poll(); ParseColumns(); /* 一次迟1ms */
    clear_output(); now = 215U; AppConsole_Poll(); ParseColumns(); /* 下一固定相位仍发送 */
    clear_output(); now = 300U; AppConsole_Poll(); ParseColumns();
    frames = 0U;
    for (index = 1U; index <= 200U; ++index) {
        clear_output(); now = 300U + index * 5U; AppConsole_Poll(); ParseColumns();
        if (output_length) { ++frames; }
        CHECK(columns[0] > 1.233f && columns[0] < 1.235f); /* 没有新CAN不伪造角度插值。 */
    }
    CHECK(frames == 200U);
    test_stage = 4U;
    /* 高幅值/长计数/细增益，验证48列和串行带宽余量。 */
    yaw_sample.controller.angle_deg = -39.9999f;
    yaw_sample.controller.target_deg = -30;
    yaw_sample.controller.cascade.target_velocity_dps = -1234567890.0f;
    yaw_sample.controller.cascade.velocity_dps = -1300.123f;
    yaw_sample.controller.velocity_dps = -11459.2f;
    yaw_sample.controller.torque_nm = yaw_sample.feedback.torque_nm = -10;
    yaw_sample.controller.cascade.position.kp = 30; yaw_sample.controller.cascade.position.ki = 10;
    yaw_sample.controller.cascade.position.kd = 2;
    yaw_sample.controller.cascade.speed.kp = 0.02f; yaw_sample.controller.cascade.speed.ki = 0.2f;
    yaw_sample.controller.cascade.speed.kd = 0.000001f;
    yaw_sample.controller.cascade.velocity_limit_dps = 0;
    yaw_sample.controller.faults = yaw_sample.feedback_age_ms = 0xFFFFFFFFUL;
    yaw_sample.feedback.driver_temp_c = -128; yaw_sample.feedback.motor_temp_c = 127;
    step_sample.id = 0xFFFFFFFFUL; step_sample.response_ms = step_sample.settle_ms = step_sample.t90_ms = 5000;
    step_sample.t10_ms = 10; step_sample.overshoot_deg = 39.9999f;
    step_sample.overshoot_pct = 133.333f; step_sample.steady_error_deg = -69.9999f;
    jog_state.speed.filtered_erpm = jog_state.speed_right.filtered_erpm = -46200;
    jog_state.health.faults = 0xFFFFFFFFUL;
    yaw_sample.controller.ff.acceleration_gain = 0.001f;
    yaw_sample.controller.ff.limit_nm = 1;
    yaw_sample.controller.cascade.feedforward_nm = -1;
    yaw_sample.controller.pid_torque_nm = -11;
    yaw_sample.controller.cascade.speed_i = -10;
    yaw_sample.controller.cascade.position_i = -1234567890.0f;
    yaw_sample.controller.shape.duration_ms = 500;
    yaw_sample.controller.shape.position_deg = -50;
    yaw_sample.controller.shape.velocity_dps = -708;
    yaw_sample.controller.shape.acceleration_dps2 = -28000;
    clear_output(); now += 5U; AppConsole_Poll(); ParseColumns();
    CHECK(columns[0] < -39.999f && columns[2] < -1.234e9f && columns[5] == -10);
    CHECK(columns[12] == 0.000001f && columns[13] == 0 && columns[36] == -6600);
    CHECK(columns[46] == -128 && columns[47] == 127);
    test_stage = 5U;
    /* 长时间未轮询只发当前一帧，不能在同一tick追发过期样本。 */
    clear_output(); now += 1000U; AppConsole_Poll(); ParseColumns();
    clear_output(); AppConsole_Poll(); CHECK(output_length == 0U);
    test_done = 1U;
}
