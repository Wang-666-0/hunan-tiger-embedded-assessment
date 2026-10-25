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
    uint32_t t, revision, start;
    YawStep snapshot;
    char report[1024];
    test_stage = 1;
    ConsoleReset(1000); Read();
    Command(1000, "YAW FF", "Y|FF|ON|0|TYPE|ACCEL|FKA|0|FMAX|0.1");
    Command(1000, "YAW FF ON", "Y|ERROR|FF_REQUIRES");
    Command(1000, "YAW FBIAS 0.02", "Y|ERROR|REMOVED|FBIAS");
    Command(1000, "YAW FANG 0.001", "Y|ERROR|REMOVED|FANG");
    Command(1000, "YAW FKV 0.0001", "Y|ERROR|REMOVED|FKV");
    Command(1000, "YAW FC 0.01", "Y|ERROR|REMOVED|FC");
    Command(1000, "YAW FVS 5", "Y|ERROR|REMOVED|FVS");
    Command(1000, "YAW FKA 0.000002", "Y|OK|FKA");
    Command(1000, "YAW FKA", "Y|FKA|2e-06");
    Command(1000, "YAW FMAX 0.08", "Y|OK|FMAX");
    Command(1000, "YAW FMAX", "Y|FMAX|0.08");
    Command(1000, "YAW FKA -0.000001", "Y|ERROR|FF_PARAMETER");
    Command(1000, "YAW FKA 0.0011", "Y|ERROR|FF_PARAMETER");
    Command(1000, "YAW FKA nan", "Y|ERROR|FF_PARAMETER");
    Command(1000, "YAW FKA inf", "Y|ERROR|FF_PARAMETER");
    Command(1000, "YAW FKA 0.000002 junk", "Y|ERROR|FF_PARAMETER");
    CHECK(!enable_calls && !torque_calls && !state.controller.ff.enabled);

    test_stage = 2;
    ConsoleZeroActivate(2000);
    Command(2066, "YAW SHAPE 160", "Y|OK|SHAPE");
    Command(2066, "YAW FKA 0.000002", "Y|OK|FKA");
    Command(2066, "YAW FF 1", "Y|OK|FF|ON|1");
    CHECK(state.controller.ff.enabled);
    CHECK(YawBackend_StartStep(30)); YawBackend_GetStep(&snapshot); start = snapshot.start_ms;
    CHECK(snapshot.ff_enabled && snapshot.ff_acceleration_gain == 0.000002f && snapshot.ff_limit_nm == 0.1f);
    revision = state.controller.ff.revision;
    Command(start, "YAW FKA 0.000004", "Y|ERROR|FF_PARAMETER");
    Command(start, "YAW FMAX 0.2", "Y|ERROR|FF_PARAMETER");
    YawBackend_GetStep(&snapshot); CHECK(snapshot.phase == YAW_STEP_RUNNING);
    CHECK(state.controller.ff.revision == revision);
    for (t = 2; t <= 220; t += 2) {
        if (t % 10 == 0) { FeedDegrees(start + t, 0, 0); }
        Control(start + t, 1);
        if (t == 40) { CHECK(state.controller.cascade.feedforward_nm > 0); }
        if (t == 120) { CHECK(state.controller.cascade.feedforward_nm < 0); }
    }
    CHECK(!state.controller.shape.active && state.controller.cascade.feedforward_nm == 0);
    Command(start + 220, "YAW FKA 0.000004", "Y|ERROR|FF_PARAMETER");
    Command(start + 220, "YAW FF OFF", "Y|OK|FF|ON|0");
    YawBackend_GetStep(&snapshot);
    CHECK(snapshot.phase == YAW_STEP_ABORTED && snapshot.reason == YAW_STEP_FF_CHANGE);
    CHECK(snapshot.ff_enabled && snapshot.ff_acceleration_gain == 0.000002f && state.active);
    Command(start + 220, "YAW RESULT", "REASON|FF_CHANGE");
    CHECK(strstr(response, "|FF_ON|1|FF_TYPE|ACCEL|FKA|2e-06|") && !strstr(response, "FBIAS"));
    diagnostic_mode = 1;
    Command(start + 220, "YAW FKA 0.000004", "Y|ERROR|FF_PARAMETER");
    Command(start + 220, "YAW FF OFF", "Y|OK|FF|ON|0");
    diagnostic_mode = 0;

    test_stage = 3;
    ConsoleZeroActivate(3000);
    Command(3066, "YAW FKA 0.000002", "Y|OK|FKA");
    Command(3066, "YAW FF ON", "Y|OK|FF|ON|1");
    Control(3068, 0); Feed(3070, 0); Control(3070, 0);
    for (t = 3072; t <= 3100; t += 2) { if (t % 10U == 0) { Feed(t, 0); } Control(t, 0); }
    Control(3102, 1); Feed(3104, 1); Control(3104, 1);
    CHECK(state.controller.ff.enabled && state.controller.ff.acceleration_gain == 0.000002f && state.active);
    Command(3106, "YAW OFF", "Y|OK|OFF");
    Control(3106, 0); Feed(3108, 0); Control(3108, 0);
    Feed(3130, 0); Control(3130, 0);
    Command(3130, "YAW ZERO", "Y|OK|SOFTWARE_ZERO");
    CHECK(!state.controller.ff.enabled && state.controller.ff.acceleration_gain == 0.000002f);

    test_stage = 4;
    RemoteStart(5000);
    CHECK(YawConsole_Poll(report, sizeof(report)) && strstr(report, "|RESULT|STARTED|"));
    for (t = 5076; t <= 10072; t += 2) {
        uint32_t elapsed = t - 5072;
        if (elapsed % 10 == 0) { FeedDegrees(t, elapsed < 200 ? 30 * (float)elapsed / 200 : 29.7f, 0); }
        Control(t, 2);
    }
    YawBackend_GetStep(&snapshot);
    CHECK(snapshot.phase == YAW_STEP_DONE && !snapshot.pass && snapshot.settle_ms == YAW_STEP_UNREACHED);
    CHECK(YawConsole_Poll(report, sizeof(report)));
    CHECK(strstr(report, "|BAND02|0.2|RESP02_MS|-1|SETTLE02_MS|-1|PASS02|0|") != 0);
    CHECK(strlen(report) < sizeof(report) - 1);
    ConsoleReset(12000); Read();
    CHECK(!state.controller.ff.enabled && state.controller.ff.acceleration_gain == 0);
    test_done = 1;
}
