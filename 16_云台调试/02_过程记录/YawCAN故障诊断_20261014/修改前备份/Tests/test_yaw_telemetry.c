/* 复用原USART/RTOS替身，实际执行生产app_console的70列格式和行缓冲。 */
#define test_run legacy_console_test_run
#include "test_app_console.c"
#undef test_run
#include "yaw_backend.h"
#include "yaw_console.h"
#include <stdlib.h>
static YawBackendState yaw_sample;
static uint32_t yaw_commands;
uint8_t YawConsole_Poll(char *response, size_t capacity) { (void)response; (void)capacity; return 0U; }
void YawBackend_GetState(YawBackendState *out) { *out = yaw_sample; }
uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity)
{
    if (strncmp(line, "YAW ", 4U)) { return 0U; }
    ++yaw_commands;
    (void)snprintf(response, capacity, "Y|MOCK\r\n"); return 1U;
}
static float columns[70];
static uint32_t ParseColumns(void)
{
    const char *cursor = output + 7U;
    char *end;
    uint32_t index;
    CHECK(strncmp(output, "gimbal:", 7U) == 0);
    for (index = 0U; index < 70U; ++index) {
        columns[index] = strtof(cursor, &end);
        CHECK(end != cursor);
        if (index < 69U) { CHECK(*end == ','); cursor = end + 1U; }
        else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    return index;
}
void test_run(void)
{
    test_stage = 1U;
    AppConsole_CreateQueue(); AppConsole_Start();
    memset(&yaw_sample, 0, sizeof(yaw_sample));
    yaw_sample.controller.angle_deg = 1.234f;
    yaw_sample.controller.target_deg = 2.5f;
    yaw_sample.controller.velocity_dps = -5.5f;
    yaw_sample.controller.torque_nm = 0.01234f;
    yaw_sample.feedback.torque_nm = -0.02345f;
    yaw_sample.controller.pid.kp = 0.35f; yaw_sample.controller.pid.ki = 0.125f;
    yaw_sample.controller.pid.kd = 0.015f;
    yaw_sample.online = yaw_sample.active = yaw_sample.requested = 1U;
    yaw_sample.feedback.status = 1U; yaw_sample.controller.faults = 256U;
    yaw_sample.feedback_age_ms = 12U; yaw_sample.stop_pending = 1U;
    yaw_sample.feedback.driver_temp_c = -3; yaw_sample.feedback.motor_temp_c = 42;
    yaw_sample.feedback.position_rad = -0.25f; yaw_sample.rx_count = 1234U;
    jog_state.active = 1U; jog_state.selection = FLYWHEEL_JOG_BOTH;
    jog_state.health.feedback[0].erpm = 42000;
    jog_state.health.feedback[1].erpm = 41900;
    output_allowed = 1U; app_armed = 1U;
    now = 100U; clear_output(); AppConsole_Poll();
    CHECK(ParseColumns() == 70U);
    CHECK(columns[11] == 1.0f && columns[12] == 1.0f);
    CHECK(columns[29] == 42000.0f && columns[30] == 41900.0f);
    CHECK(columns[33] == 3.0f && columns[35] == 1.0f);
    CHECK(columns[51] > 1.233f && columns[51] < 1.235f);
    CHECK(columns[52] == 2.5f && columns[53] == -5.5f);
    CHECK(columns[54] > 0.01233f && columns[54] < 0.01235f);
    CHECK(columns[55] < -0.02344f && columns[55] > -0.02346f);
    CHECK(columns[56] > 0.3499f && columns[56] < 0.3501f);
    CHECK(columns[57] == 0.125f && columns[58] > 0.0149f && columns[58] < 0.0151f);
    CHECK(columns[59] == 1 && columns[60] == 1 && columns[61] == 1);
    CHECK(columns[62] == 256 && columns[63] == 12 && columns[64] == 1 && columns[65] == 1);
    CHECK(columns[66] == -3 && columns[67] == 42 && columns[68] == -0.25f && columns[69] == 1234);
    test_stage = 2U;
    clear_output(); command_line("YAW KP 0.350000");
    CHECK(yaw_commands == 1U && strcmp(output, "Y|MOCK\r\n") == 0);
    /* UART 50ms周期未到，不插入额外遥测；原STOP仍从同一分发入口撤权。 */
    clear_output(); command_line("STOP"); CHECK(stops == 1U && !output_allowed);
    clear_output(); now += 50U; AppConsole_Poll(); CHECK(ParseColumns() == 70U);
    CHECK(columns[12] == 0.0f && columns[59] == 1.0f);
    test_done = 1U;
}
