#define YAW_CASCADE_CONSOLE_TEST 1
#include "test_yaw_console_backend.c"

static void FFCommand(uint32_t timestamp, const char *line, const char *expected, uint32_t source_line)
{
    now_ms = timestamp; memset(response, 0, sizeof(response));
    CHECK(YawConsole_Handle(line, response, sizeof(response)) == 1U);
    ++test_checks;
    if (!strstr(response, expected)) {
        ++test_failures; if (!test_failed_line) { test_failed_line = source_line; }
    }
    Read();
}
#define Command(t, line, expected) FFCommand(t, line, expected, __LINE__)

void test_run(void)
{
    uint32_t t, revision;
    YawStep snapshot;
    char report[1024];
    test_stage = 1U;
    ConsoleReset(1000); Read();
    Command(1000, "YAW FF", "Y|FF|ON|0|FBIAS|0|FANG|0|FKV|0|FC|0|FVS|5|FMAX|0.1");
    Command(1000, "YAW FF ON", "Y|ERROR|FF_REQUIRES");
    Command(1000, "YAW FBIAS 0.02", "Y|OK|FBIAS");
    Command(1000, "YAW FANG -0.001", "Y|OK|FANG");
    Command(1000, "YAW FKV 0.0001", "Y|OK|FKV");
    Command(1000, "YAW FC 0.01", "Y|OK|FC");
    Command(1000, "YAW FVS 4", "Y|OK|FVS");
    Command(1000, "YAW FMAX 0.08", "Y|OK|FMAX");
    Command(1000, "YAW FVS 0", "Y|ERROR|FF_PARAMETER");
    Command(1000, "YAW FBIAS nan", "Y|ERROR|FF_PARAMETER");
    Command(1000, "YAW FKV -0.01", "Y|ERROR|FF_PARAMETER");
    Command(1000, "YAW FC 0.01 junk", "Y|ERROR|FF_PARAMETER");
    CHECK(!enable_calls && !torque_calls && !state.controller.ff.enabled);

    test_stage = 2U;
    ConsoleZeroActivate(2000);
    Command(2066, "YAW FBIAS 0.02", "Y|OK|FBIAS");
    Command(2066, "YAW FF 1", "Y|OK|FF|ON|1");
    Read(); CHECK(state.controller.ff.enabled && state.controller.ff.i_transferred);
    CHECK(YawBackend_StartStep(30));
    YawBackend_GetStep(&snapshot);
    CHECK(snapshot.ff_enabled && snapshot.ff_bias_nm == 0.02f && snapshot.ff_limit_nm == 0.1f);
    revision = state.controller.ff.revision;
    Command(2066, "YAW FBIAS 0.02", "Y|OK|FBIAS");
    YawBackend_GetStep(&snapshot); CHECK(snapshot.phase == YAW_STEP_RUNNING);
    Read(); CHECK(state.controller.ff.revision == revision);
    Command(2066, "YAW FC 0.01", "Y|OK|FC");
    YawBackend_GetStep(&snapshot);
    CHECK(snapshot.phase == YAW_STEP_ABORTED && snapshot.reason == YAW_STEP_FF_CHANGE);
    CHECK(snapshot.ff_friction_nm == 0 && state.active && state.controller.target_deg == 30);
    Command(2066, "YAW RESULT", "REASON|FF_CHANGE");
    CHECK(strstr(response, "|FF_ON|1|FBIAS|0.02|") && strstr(response, "|SETTLE05_MS|-1|PASS05|0|"));
    diagnostic_mode = 1;
    Command(2066, "YAW FC 0.02", "Y|ERROR|FF_PARAMETER");
    Command(2066, "YAW FF OFF", "Y|OK|FF|ON|0");
    diagnostic_mode = 0;

    test_stage = 3U;
    ConsoleZeroActivate(3000);
    Command(3066, "YAW FBIAS 0.02", "Y|OK|FBIAS");
    Command(3066, "YAW FF ON", "Y|OK|FF|ON|1");
    Control(3068, 0); Feed(3070, 0); Control(3070, 0);
    for (t = 3072; t <= 3100; t += 2) { if (t % 10U == 0) { Feed(t, 0); } Control(t, 0); }
    Control(3102, 1); Feed(3104, 1); Control(3104, 1);
    CHECK(state.controller.ff.enabled && state.controller.ff.bias_nm == 0.02f && state.active);
    Command(3106, "YAW OFF", "Y|OK|OFF");
    Control(3106, 0); Feed(3108, 0); Control(3108, 0);
    Feed(3130, 0); Control(3130, 0);
    Command(3130, "YAW ZERO", "Y|OK|SOFTWARE_ZERO");
    CHECK(!state.controller.ff.enabled && state.controller.ff.bias_nm == 0.02f);

    test_stage = 4U;
    RemoteStart(5000);
    CHECK(YawConsole_Poll(report, sizeof(report)) && strstr(report, "|RESULT|STARTED|"));
    for (t = 5076; t <= 10072; t += 2) {
        uint32_t elapsed = t - 5072;
        if (elapsed % 10 == 0) { FeedDegrees(t, elapsed < 200 ? 30 * (float)elapsed / 200 : 29.7f, 0); }
        Control(t, 2);
    }
    YawBackend_GetStep(&snapshot);
    CHECK(snapshot.phase == YAW_STEP_DONE && !snapshot.pass && snapshot.pass05 && snapshot.settle05_ms == 200);
    CHECK(YawConsole_Poll(report, sizeof(report)));
    CHECK(strstr(report, "|BAND05|0.5|RESP05_MS|200|SETTLE05_MS|200|PASS05|1|") != 0);
    CHECK(strlen(report) < sizeof(report) - 1);
    ConsoleReset(12000); Read();
    CHECK(!state.controller.ff.enabled && state.controller.ff.bias_nm == 0);
    test_done = 1U;
}
