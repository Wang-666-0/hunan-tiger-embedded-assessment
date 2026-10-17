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
void YawConsole_CancelPreparation(void) { }
uint8_t YawConsole_Poll(char *response, size_t capacity) { (void)response; (void)capacity; return 0U; }
void YawBackend_GetState(YawBackendState *out) { *out = yaw_sample; }
void YawBackend_GetStep(YawStep *out) { *out = step_sample; }
uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity)
{
    if (strncmp(line, "YAW ", 4U)) { return 0U; }
    ++yaw_commands; (void)snprintf(response, capacity, "Y|MOCK\r\n"); return 1U;
}
static float columns[48];
static void ParseColumns(void)
{
    const char *cursor = output + 7U;
    char *end;
    uint32_t index;
    CHECK(strncmp(output, "gimbal:", 7U) == 0);
    for (index = 0U; index < 48U; ++index) {
        columns[index] = strtof(cursor, &end); CHECK(end != cursor);
        if (index < 47U) { CHECK(*end == ','); cursor = end + 1U; }
        else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    CHECK(output_length < 500U);
}
void test_run(void)
{
    test_stage = 1U;
    AppConsole_CreateQueue(); AppConsole_Start();
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
    test_stage = 2U;
    clear_output(); command_line("YAW VKP 0.001000");
    CHECK(yaw_commands == 1 && strcmp(output, "Y|MOCK\r\n") == 0);
    clear_output(); command_line("STOP"); CHECK(stops == 1 && !output_allowed);
    step_sample.response_ms = step_sample.t90_ms = YAW_STEP_UNREACHED;
    clear_output(); now += 50U; AppConsole_Poll(); ParseColumns();
    CHECK(columns[26] == -1 && columns[33] == -1 && columns[34] == -1);
    test_done = 1U;
}
