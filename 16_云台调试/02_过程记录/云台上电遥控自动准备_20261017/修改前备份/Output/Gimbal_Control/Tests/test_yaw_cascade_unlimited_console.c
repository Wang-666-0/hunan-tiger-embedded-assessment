#define YAW_CASCADE_CONSOLE_TEST 1
#include "test_yaw_console_backend.c"

void test_run(void)
{
    uint32_t t;
    YawStep saved, result;
    char report[1024];
    test_stage = 1;
    CHECK(!YAW_CASCADE_OUTPUT_LIMITS_ENABLED);
    ConsoleReset(1000); Control(1000, 0);
    Command(1000, "YAW", "Y|STATE|GL60II_CASCADE_MANUAL50_V2H"); CHECK(strlen(response) < 300U);
    Command(1000, "YAW HELP", "Y|HELP|READ|YAW;PID;RESULT;STATUS");
    Command(1000, "YAW ?", "VIEW|YAW VIEW");
    Command(1000, "YAW BAD", "UNKNOWN_COMMAND|USE|YAW HELP");
    CHECK(!enable_calls && !disable_calls && !torque_calls && !clear_calls && !stop_calls);
    Command(1000, "YAW PID", "Y|PID|POS|10.0000,0.1500,0.0100|VEL|0.003700,0.003000,0.000001");
    Command(1002, "YAW READY", "Y|BUSY|READY");
    Command(1002, "YAW", "Y|STATE|"); Command(1002, "YAW HELP", "Y|HELP|");
    Command(1002, "YAW PID", "Y|PID|POS|10.0000");
    for (t = 1004; t < 1200; t += 2) {
        if (t % 10 == 6) { pending = 0; Feed(t, clear_calls ? 0 : 13); }
        Control(t, 0);
        if (YawConsole_Poll(report, sizeof(report)) && strstr(report, "Y|OK|READY|")) { break; }
    }
    CHECK(t < 1200 && clear_calls == 1 && state.zero_set && !enable_calls && !torque_calls);
    Command(t, "YAW STATUS", "|PROTOCOL_NM|10.000|OUTPUT_LIMITS|0|");
    CHECK(strstr(response, "|VREF_MAX_DPS|0.0|") && strstr(response, "|SPEED_GUARD|0|"));
    Command(t, "YAW PID", "|VMAX_DPS|0.0|OUTPUT_LIMITS|0");
    Command(t, "YAW VMAX 180", "Y|ERROR|OUTPUT_LIMITS_DISABLED");
    CHECK(!state.controller.cascade.velocity_limit_dps && !state.active);

    test_stage = 2;
    RemoteStart(3000);
    YawBackend_GetStep(&result);
    CHECK(result.cascade && !result.output_limits && result.vmax_dps == 0);
    Control(3076, 2); Control(3078, 2); Control(3080, 2); Control(3082, 2);
    CHECK(last_torque > 1.10f && last_torque < 1.13f);
    CHECK(state.controller.cascade.target_velocity_dps > 299 && state.active && !state.controller.faults);
    Command(3082, "YAW POS 17 1.25 0", "Y|OK|POS");
    Command(3082, "YAW VEL 0.0048 0.013 0", "Y|OK|VEL");
    YawBackend_GetStep(&result);
    CHECK(result.phase == YAW_STEP_ABORTED && result.reason == YAW_STEP_PID_CHANGE && state.step_hold);
    for (t = 3084; t <= 3092; t += 2) { FeedDegrees(t, 0, 0); Control(t, 2); }
    /* 2.448基础输出加新反馈期间的积分，不再受原.10发送校验阻碍。 */
    CHECK(last_torque > 2.44f && last_torque < 2.6f && state.active && !state.controller.faults);
    CHECK(state.controller.cascade.target_velocity_dps > 509 && !state.controller.cascade.velocity_limited);
    receiver.channels[RC_TRIGGER_CHANNEL] = 1000;
    Control(3094, 2); CHECK(!state.active && !permission);

    test_stage = 3;
    RemoteStart(5000);
    CHECK(YawConsole_Poll(report, sizeof(report)) && strstr(report, "|RESULT|STARTED|"));
    for (t = 5076; t <= 10072; t += 2) {
        uint32_t elapsed = t - 5072;
        if (elapsed % 10 == 0) { FeedDegrees(t, elapsed < 200 ? 30 * (float)elapsed / 200 : 30, elapsed < 200 ? 150 : 0); }
        Control(t, 2);
    }
    YawBackend_GetStep(&saved);
    CHECK(saved.phase == YAW_STEP_DONE && saved.pass && saved.sse_valid && saved.response_ms == 200);
    CHECK(saved.peak_torque_nm > 0.2f && saved.command_samples > 400 && !saved.output_limits);
    CHECK(state.active && state.step_hold && state.controller.target_deg == 30);
    CHECK(YawConsole_Poll(report, sizeof(report)));
    CHECK(strstr(report, "|RESP_MS|200|") && strstr(report, "|OS_DEG|0.000|") && strstr(report, "|SSE_VALID|1|PASS|1|"));
    CHECK(strstr(report, "|PKP|10.0000|PKI|0.1500|PKD|0.0100|") != 0);
    CHECK(strstr(report, "|VKP|0.003700|VKI|0.003000|VKD|0.000001|") != 0);
    CHECK(strstr(report, "|VREF_MAX_DPS|0.0|VFILTER_MS|10|OUTPUT_LIMITS|0\r\n"));
    Command(10072, "YAW VKI 0.006", "Y|OK|PARAMETER");
    Command(10072, "YAW RESULT", "|VKI|0.003000|");
    YawBackend_GetStep(&result); CHECK(result.phase == YAW_STEP_DONE && result.response_ms == saved.response_ms);
    Command(10072, "YAW OFF", "Y|OK|OFF");
    CHECK(!state.active && !state.requested && !permission && !state.controller.cascade.speed_i);

    test_stage = 4;
    /* 正式后端重初始化模拟断电重启；在线改参跨OFF保留，重启回固化值。
     * 查询及初始化均不启动电机，也不保留软件零点/阶跃请求。 */
    ConsoleReset(12000); Read();
    Command(12000, "YAW POS 9.5 0.2 0.02", "Y|OK|POS");
    Command(12000, "YAW VEL 0.0029 0.006 0", "Y|OK|VEL");
    Command(12000, "YAW OFF", "Y|OK|OFF");
    Command(12000, "YAW PID", "Y|PID|POS|9.5000,0.2000,0.0200|VEL|0.002900,0.006000,0.000000");
    CHECK(!state.active && !state.requested && !state.zero_set && !enable_calls && !torque_calls);
    ConsoleReset(13000); Read();
    Command(13000, "YAW PID", "Y|PID|POS|10.0000,0.1500,0.0100|VEL|0.003700,0.003000,0.000001");
    CHECK(!state.active && !state.requested && !state.zero_set && !enable_calls && !torque_calls);
    CHECK(!state.controller.cascade.speed_i && !state.controller.cascade.position_i);
    test_done = 1;
}
