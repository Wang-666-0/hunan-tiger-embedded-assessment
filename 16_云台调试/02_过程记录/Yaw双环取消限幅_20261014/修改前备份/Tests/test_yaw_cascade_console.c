#define YAW_CASCADE_CONSOLE_TEST 1
#include "test_yaw_console_backend.c"

void test_run(void)
{
    uint32_t t, revision;
    YawStep measurement, saved;
    char report[1024], tiny[8];
    test_stage = 1U;
    CHECK(YAW_CONTROLLER_CASCADE_ENABLED == 1U && !YAW_CONTROLLER_SPEED_GUARD_ENABLED);
    ConsoleReset(1000U); Control(1000U, 0U);
    Command(1002U, "YAW READY", "Y|BUSY|READY");
    for (t = 1004U; t < 1200U; t += 2U) {
        if (t % 10U == 6U) { Feed(t, 0U); }
        Control(t, 0U);
        if (YawConsole_Poll(report, sizeof(report)) && strstr(report, "Y|OK|READY|")) { break; }
        CHECK(!enable_calls && !torque_calls);
    }
    Read(); CHECK(t < 1200U && state.zero_set && state.requested && !state.active);
    Command(t, "YAW PID", "Y|PID|POS|8.0000,0.0000,0.0000|VEL|0.001000,0.004000,0.000000");
    Command(t, "YAW POS 10 0.1 0.02", "Y|OK|POS");
    Command(t, "YAW VEL 0.002 0.006 0.000002", "Y|OK|VEL");
    CHECK(state.controller.cascade.position.kp == 10 && state.controller.cascade.speed.kp == 0.002f);
    revision = state.controller.cascade.revision;
    Command(t, "YAW VEL 0.003 0.3 0", "Y|ERROR|");
    Command(t, "YAW POS 11 nan 0", "Y|ERROR|");
    Command(t, "YAW POS 11 0", "Y|ERROR|");
    Command(t, "YAW VKD 0.01", "Y|ERROR|");
    Command(t, "YAW PKP 9junk", "Y|ERROR|");
    Command(t, "YAW VMAX 0", "Y|ERROR|");
    Command(t, "YAW PID 1 0 0.015", "Y|ERROR|USE_YAW_POS_OR_VEL");
    Command(t, "YAW KP 1", "Y|ERROR|USE_YAW_POS_OR_VEL");
    CHECK(state.controller.cascade.revision == revision && state.controller.cascade.speed.ki == 0.006f);
    Command(t, "YAW PKP 9", "Y|OK|PARAMETER");
    Command(t, "YAW PKI 0.2", "Y|OK|PARAMETER");
    Command(t, "YAW PKD 0.03", "Y|OK|PARAMETER");
    Command(t, "YAW VKP 0.0015", "Y|OK|PARAMETER");
    Command(t, "YAW VKI 0.005", "Y|OK|PARAMETER");
    Command(t, "YAW VKD 0.000001", "Y|OK|PARAMETER");
    Command(t, "YAW VMAX 240", "Y|OK|VMAX");
    CHECK(state.controller.cascade.position.kp == 9 && state.controller.cascade.position.ki == 0.2f);
    CHECK(state.controller.cascade.speed.kp == 0.0015f && state.controller.cascade.velocity_limit_dps == 240);
    Command(t, "YAW STATUS", "|CTRL|CASCADE|ONLINE|1|");
    CHECK(strstr(response, "|PKP|9.0000|PKI|0.2000|PKD|0.0300|VKP|0.001500|VKI|0.005000|VKD|0.000001|") != 0);
    CHECK(strstr(response, "|SPEED_GUARD|0|") != 0);
    memset(tiny, 0x5A, sizeof(tiny)); CHECK(YawConsole_Handle("YAW STATUS", tiny, 6U));
    CHECK(tiny[5] == '\0' && tiny[6] == 0x5A);
    diagnostic_mode = 1U; Command(t, "YAW PKP 4", "Y|ERROR|");
    CHECK(state.controller.cascade.position.kp == 9); diagnostic_mode = 0U;

    test_stage = 2U;
    RemoteStart(3000U); YawBackend_GetStep(&measurement);
    CHECK(measurement.phase == YAW_STEP_RUNNING && measurement.cascade && measurement.kp == 8);
    CHECK(measurement.vkp == 0.001f && measurement.vki == 0.004f && measurement.vmax_dps == 180);
    Command(3074U, "YAW VKP 0.001", "Y|OK|PARAMETER");
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_RUNNING);
    Command(3074U, "YAW VKP 0.0015", "Y|OK|PARAMETER");
    YawBackend_GetStep(&measurement);
    CHECK(measurement.phase == YAW_STEP_ABORTED && measurement.reason == YAW_STEP_PID_CHANGE);
    CHECK(state.active && state.step_hold && state.controller.target_deg == 30);

    test_stage = 3U;
    RemoteStart(5000U);
    CHECK(YawConsole_Poll(report, sizeof(report)) && strstr(report, "|RESULT|STARTED|"));
    for (t = 5076U; t <= 10072U; t += 2U) {
        uint32_t elapsed = t - 5072U;
        if (elapsed % 10U == 0U) { FeedDegrees(t, elapsed < 200U ? 30 * (float)elapsed / 200 : 30, elapsed < 200U ? 150 : 0); }
        Control(t, 2U);
    }
    YawBackend_GetStep(&saved);
    CHECK(saved.phase == YAW_STEP_DONE && saved.pass && saved.sse_valid && saved.response_ms == 200);
    CHECK(state.active && state.step_hold && state.controller.target_deg == 30);
    CHECK(YawConsole_Poll(report, sizeof(report)));
    CHECK(strstr(report, "|RESP_MS|200|") && strstr(report, "|OS_DEG|0.000|") && strstr(report, "|SSE_VALID|1|PASS|1|"));
    CHECK(strstr(report, "|SAT_N|") && strstr(report, "|PKP|8.0000|PKI|0.0000|PKD|0.0000|"));
    CHECK(strstr(report, "|CTRL|CASCADE|VKP|0.001000|VKI|0.004000|VKD|0.000000|VREF_MAX_DPS|180.0|VFILTER_MS|10\r\n"));
    CHECK(strlen(report) < 900U && !YawConsole_Poll(report, sizeof(report)));
    Command(10072U, "YAW VKI 0.006", "Y|OK|PARAMETER");
    Command(10072U, "YAW RESULT", "|VKI|0.004000|");
    YawBackend_GetStep(&measurement);
    CHECK(measurement.phase == YAW_STEP_DONE && measurement.response_ms == saved.response_ms);
    Command(10072U, "YAW OFF", "Y|OK|OFF"); Read();
    CHECK(!state.active && !state.requested && !permission);
    CHECK(state.controller.cascade.speed_i == 0 && state.controller.cascade.position_i == 0);
    test_stage = 4U;
    ConsoleReset(12000U); Control(12000U, 0U);
    Command(12002U, "YAW READY", "Y|BUSY|READY");
    for (t = 12004U; t < 12200U; t += 2U) {
        if (t % 10U == 6U) {
            CanFrame f = Feedback(t, clear_calls ? 0U : 13U, 25U, 30U);
            pending = 0U; /* 模拟FB已由硬件邮箱发完，非软件取消该命令。 */
            now_ms = t; CHECK(YawBackend_OnFrame(&f));
        }
        Control(t, 0U);
        if (YawConsole_Poll(report, sizeof(report)) && strstr(report, "Y|OK|READY|")) { break; }
    }
    CHECK(t < 12200U && clear_calls == 1U && state.zero_set && !enable_calls && !torque_calls);
    /* 用户指定的遥控锁定停机在生产双环中仍即时撤权。 */
    RemoteStart(13000U); receiver.channels[RC_TRIGGER_CHANNEL] = 1000U;
    Control(13076U, 2U); YawBackend_GetStep(&measurement);
    CHECK(!state.active && !permission && measurement.phase == YAW_STEP_ABORTED);
    /* 两路CAN错误锁存不因换环或READY被清掉。 */
    ConsoleReset(14000U); Control(14000U, 0U); bus.fault_epoch = 1U;
    Command(14002U, "YAW READY", "Y|ERROR|READY|CAN_FAULT_USE_YAW_BUS");
    CHECK(bus.fault_epoch == 1U && !enable_calls && !torque_calls);
    test_done = 1U;
}
