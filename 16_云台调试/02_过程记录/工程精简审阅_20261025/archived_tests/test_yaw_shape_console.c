#define YAW_CASCADE_CONSOLE_TEST 1
#include "test_yaw_console_backend.c"
#include <math.h>

void test_run(void)
{
    uint32_t t, start;
    YawStep snapshot;
    char report[1024];
    test_stage = 1;
    ConsoleReset(1000);
    Command(1000, "YAW SHAPE", "Y|SHAPE|ENABLED|0|MS|0|ACTIVE|0");
    Command(1000, "YAW SHAPEV", "Y|SHAPEV|0");
    Command(1000, "YAW SHAPEV -0.1", "Y|ERROR|SHAPEV");
    Command(1000, "YAW SHAPEV 1.1", "Y|ERROR|SHAPEV");
    Command(1000, "YAW SHAPEV nan", "Y|ERROR|SHAPEV");
    Command(1000, "YAW SHAPEV 0.25 junk", "Y|ERROR|SHAPEV");
    Command(1000, "YAW SHAPEV 0.250000", "Y|OK|SHAPEV|0.25");
    Command(1000, "YAW SHAPE 79", "Y|ERROR|SHAPE");
    Command(1000, "YAW SHAPE 501", "Y|ERROR|SHAPE");
    Command(1000, "YAW SHAPE -1", "Y|ERROR|SHAPE");
    Command(1000, "YAW SHAPE nan", "Y|ERROR|SHAPE");
    Command(1000, "YAW SHAPE 160.5", "Y|ERROR|SHAPE");
    Command(1000, "YAW SHAPE 160 junk", "Y|ERROR|SHAPE");
    Command(1000, "YAW SHAPE 160.000000", "Y|OK|SHAPE|MS|160");
    CHECK(!enable_calls && !torque_calls && !state.controller.shape.active);

    test_stage = 2;
    ConsoleZeroActivate(2000);
    Command(2066, "YAW SHAPE 160", "Y|OK|SHAPE|MS|160");
    Command(2066, "YAW SHAPEV 1", "Y|OK|SHAPEV|1");
    CHECK(YawBackend_StartStep(30)); Read(); YawBackend_GetStep(&snapshot); start = snapshot.start_ms;
    CHECK(snapshot.shape_ms == 160 && snapshot.target_deg == 30 && state.controller.target_deg == 30);
    CHECK(state.controller.shape.active && state.controller.shape.start_ms == start);
    CHECK(snapshot.shape_velocity_gain == 1);
    Command(start, "YAW SHAPEV 1", "Y|OK|SHAPEV|1");
    Command(start, "YAW SHAPEV 0.25", "Y|ERROR|SHAPEV");
    Command(start, "YAW SHAPE 160", "Y|OK|SHAPE|MS|160");
    Command(start, "YAW SHAPE 180", "Y|ERROR|SHAPE");
    Command(start, "YAW SHAPE 0", "Y|ERROR|SHAPE");
    Command(start, "YAW POS 9 0.2 0.05", "Y|ERROR|PARAMETER_OR_STATE");
    Command(start, "YAW FKA 0.000002", "Y|ERROR|FF_PARAMETER");
    CHECK(!state.controller.ff.enabled);
    for (t = 2; t <= 5010; t += 2) {
        if (t % 10 == 0) { FeedDegrees(start + t, t < 200 ? 30.0f * t / 200.0f : 30, 0); }
        Control(start + t, 1);
        if (t == 80) {
            CHECK(state.controller.shape.active);
            CHECK(fabsf(state.controller.shape.position_deg - 15) < 0.1f);
            CHECK(state.controller.cascade.target_velocity_dps > 300);
        }
        if (t == 160) { CHECK(!state.controller.shape.active && state.controller.shape.position_deg == 30); }
    }
    YawBackend_GetStep(&snapshot);
    CHECK(snapshot.phase == YAW_STEP_DONE && snapshot.shape_ms == 160 && snapshot.start_ms == start);
    CHECK(snapshot.response_ms == 200 && snapshot.settle_ms == 200 && snapshot.pass);
    CHECK(YawConsole_Poll(report, sizeof(report)));
    CHECK(strstr(report, "|SHAPE_MS|160|SHAPE_V|1\r\n") && strstr(report, "|RESP02_MS|200|SETTLE02_MS|200|PASS02|1|"));
    Command(start + 5010, "YAW SHAPEV 0.25", "Y|OK|SHAPEV|0.25");
    Command(start + 5010, "YAW SHAPE 0", "Y|OK|SHAPE|MS|0");
    Command(start + 5010, "YAW RESULT", "SHAPE_MS|160"); /* Frozen trial condition. */
    CHECK(strstr(response, "SHAPE_V|1"));

    test_stage = 3;
    ConsoleZeroActivate(9000);
    Command(9066, "YAW SHAPE 160", "Y|OK|SHAPE|MS|160");
    CHECK(YawBackend_StartStep(-30)); Read(); CHECK(state.controller.shape.active);
    Command(9066, "YAW HOME", "Y|OK|HOME");
    CHECK(!state.controller.shape.active && state.controller.target_deg == 0);
    YawBackend_GetStep(&snapshot); CHECK(snapshot.phase == YAW_STEP_ABORTED && snapshot.reason == YAW_STEP_TARGET_CHANGE);
    diagnostic_mode = 1;
    Command(9066, "YAW SHAPE 180", "Y|ERROR|SHAPE");
    Command(9066, "YAW SHAPEV 0.25", "Y|ERROR|SHAPEV");
    diagnostic_mode = 0;
    ConsoleZeroActivate(10000);
    Command(10066, "YAW SHAPE 160", "Y|OK|SHAPE|MS|160");
    CHECK(YawBackend_StartStep(30));
    Control(10068, 0);
    CHECK(!state.controller.shape.active && !state.controller.torque_nm && !permission);
    ConsoleReset(12000); Read(); CHECK(!state.controller.shape.duration_ms);
    test_stage = 4;
    ConsoleZeroActivate(13000);
    Command(13066, "YAW SHAPE 160", "Y|OK|SHAPE|MS|160");
    Control(13068, 2);
    receiver.channels[RC_STEP_YAW_CHANNEL] = 2000; Control(13070, 2);
    receiver.channels[RC_STEP_CHANNEL] = 2000; Control(13072, 2);
    YawBackend_GetStep(&snapshot);
    CHECK(state.controller.shape.active && snapshot.shape_ms == 160 && snapshot.start_ms == 13072);
    CHECK(state.controller.target_deg == 30 && snapshot.phase == YAW_STEP_RUNNING);
    Control(13074, 0);
    CHECK(!state.controller.shape.active && !permission);
    test_done = 1;
}
