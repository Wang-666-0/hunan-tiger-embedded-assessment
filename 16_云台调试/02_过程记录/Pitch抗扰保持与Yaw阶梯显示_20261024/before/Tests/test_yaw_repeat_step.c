#define YAW_CASCADE_CONSOLE_TEST 1
#include "test_yaw_console_backend.c"

static void Drive(uint32_t start, uint32_t duration, float from, float to)
{
    uint32_t dt;
    for (dt = 2; dt <= duration; dt += 2) {
        if (dt % 10 == 0) {
            float f = dt < 600 ? (float)dt / 600 : 1.0f;
            FeedDegrees(start + dt, from + (to - from) * f, dt % 20 ? 13.99f : -13.99f);
        }
        Control(start + dt, 2);
    }
}

void test_run(void)
{
    YawStep snap;
    YawRemoteStepEvent event;
    uint32_t cycle, t, start, id;
    test_stage = 1;
    CHECK(!APP_YAW_EXPERIMENTAL_TRACKING);
    ConsoleReset(1000);
    Command(1000, "YAW PID", "POS|11.9330,3.2120,0.1390|VEL|0.005040,0.000800,0.000074");
    Command(1000, "YAW SHAPE 80", "Y|ERROR|REMOVED|SHAPE");
    Command(1000, "YAW SHAPEV 0.15", "Y|ERROR|REMOVED|SHAPEV");
    Command(1000, "YAW FF ON", "Y|ERROR|REMOVED|FF");
    Command(1000, "YAW FKA 0.000002", "Y|ERROR|REMOVED|FKA");
    CHECK(!enable_calls && !torque_calls);

    test_stage = 2;
    ConsoleZeroActivate(2000); Control(2068, 2);
    /* CH7 may be confirmed before the direction is chosen; no event is lost. */
    receiver.channels[RC_STEP_CHANNEL] = 2000; Control(2070, 2);
    CHECK(!request.step_event);
    receiver.channels[RC_STEP_YAW_CHANNEL] = 2000; Control(2072, 2);
    CHECK(request.step_event && state.controller.target_deg == 30);
    YawBackend_GetStep(&snap); CHECK(snap.phase == YAW_STEP_RUNNING && snap.id == 1);
    CHECK(!snap.shape_ms && !snap.shape_velocity_gain && !snap.ff_enabled);
    CHECK(YawBackend_TakeRemoteStepEvent(&event) && event.result == 1 && !event.blocks);
    start = snap.start_ms;

    /* Three full cycles, real command/backend path, no re-zero or READY. */
    for (cycle = 0; cycle < 3; ++cycle) {
        id = cycle + 1;
        for (t = 2; t <= 5000; t += 2) {
            if (t % 10 == 0) { FeedDegrees(start + t, t < 200 ? 30.0f * t / 200 : 30, 13.99f); }
            Control(start + t, 2);
            if (t == 200) {
                receiver.channels[RC_STEP_CHANNEL] = 1500; Control(start + t, 2);
                receiver.channels[RC_STEP_CHANNEL] = 2000; Control(start + t, 2);
                CHECK(YawBackend_TakeRemoteStepEvent(&event) && event.result == 3);
                CHECK(event.blocks & YAW_STEP_BLOCK_WINDOW);
                Command(start + t, "YAW STEP CHECK", "WHY|WAIT_5S_WINDOW");
            }
        }
        YawBackend_GetStep(&snap); CHECK(snap.phase == YAW_STEP_DONE && snap.id == id && snap.pass);
        CHECK(YawBackend_GetStepId() == snap.id);
        /* Holding CH7 high after DONE must not replay the rejected request. */
        Drive(start + 5000, 100, 30, 30);
        YawBackend_GetStep(&snap); CHECK(snap.id == id && state.controller.target_deg == 30);
        Command(start + 5100, "YAW STEP CHECK", "WHY|SEND_HOME_FIRST");
        receiver.channels[RC_STEP_CHANNEL] = 1000; Control(start + 5102, 2);
        CHECK(state.controller.target_deg == 0);
        Drive(start + 5102, 800, 30, 0);
        Read(); CHECK(!state.controller.stationary_samples); /* Raw speed alone is noisy. */
        CHECK(!state.step_blocks && state.step_still_ms >= 20);
        Command(start + 5902, "YAW STEP CHECK", "READY|1|WHY|READY");
        if (cycle < 2) {
            receiver.channels[RC_STEP_CHANNEL] = 1500; Control(start + 5904, 2);
            receiver.channels[RC_STEP_CHANNEL] = 2000; Control(start + 5906, 2);
            YawBackend_GetStep(&snap);
            CHECK(snap.phase == YAW_STEP_RUNNING && snap.id == id + 1);
            CHECK(YawBackend_GetStepId() == snap.id);
            start = snap.start_ms;
        }
    }

    test_stage = 3;
    now_ms = start + 5904;
    FeedDegrees(now_ms, 0, 70); Control(now_ms, 2);
    CHECK(state.step_blocks & YAW_STEP_BLOCK_MOVING);
    CHECK(!YawBackend_StartStep(30));
    FeedDegrees(now_ms + 10, 0.15f, 0); Control(now_ms + 10, 2);
    CHECK(state.step_blocks & (YAW_STEP_BLOCK_MOVING | YAW_STEP_BLOCK_STILL));
    CHECK(!YawBackend_StartStep(30));
    YawBackend_GetStep(&snap);
    CHECK(snap.id == id && YawBackend_GetStepId() == snap.id);
    Control(now_ms + 12, 0);
    CHECK(!state.active && !permission && (state.step_blocks & YAW_STEP_BLOCK_INACTIVE));
    test_done = 1;
}
