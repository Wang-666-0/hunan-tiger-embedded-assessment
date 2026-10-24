#define test_run controller_fixture_not_run
#include "test_yaw_controller.c"
#undef test_run

/* Rest-to-rest planned acceleration must accelerate, brake in the opposite
 * direction, have zero net impulse, and leave no holding compensation. */
static void accel_case(uint32_t start, uint32_t duration, float goal, float gain, float limit)
{
    uint32_t dt;
    float impulse = 0, direction = goal > 0 ? 1.0f : -1.0f;
    uint8_t clipped = 0;
    ready(start - 20U, 0);
    yaw.cascade.position.kp = yaw.cascade.position.ki = yaw.cascade.position.kd = 0;
    yaw.cascade.speed.kp = yaw.cascade.speed.ki = yaw.cascade.speed.kd = 0;
    CHECK(YawController_SetShape(&yaw, duration));
    CHECK(YawController_SetFeedforward(&yaw, 'A', gain));
    CHECK(YawController_SetFeedforward(&yaw, 'L', limit));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
    (void)YawController_Update(&yaw, 0, start, 1);
    CHECK(YawController_SetTarget(&yaw, goal));
    YawController_StartShape(&yaw, start);
    for (dt = 0; dt <= duration; dt += 2) {
        float u;
        if (dt && dt % 10 == 0) { frame(start + dt, 0, 0); CHECK(YawController_OnFeedback(&yaw, &feedback)); }
        u = YawController_Update(&yaw, 0, start + dt, 1);
        CHECK(!yaw.faults && absolute(u) <= limit && yaw.pid_torque_nm == 0);
        CHECK(near(u, yaw.cascade.feedforward_nm, 0.000001f));
        if (dt && dt < duration / 2) { CHECK(direction * u > 0); }
        if (dt > duration / 2 && dt < duration) { CHECK(direction * u < 0); }
        if (!dt || dt == duration / 2 || dt == duration) { CHECK(absolute(u) < 0.000001f); }
        impulse += u * 0.002f;
        clipped |= yaw.cascade.feedforward_limited;
        if (dt == duration / 4) {
            float before = u;
            CHECK(!YawController_SetFeedforward(&yaw, 'A', gain + 0.000001f));
            CHECK(!YawController_SetFeedforward(&yaw, 'L', 0.5f));
            /* Measured acceleration is not the feedforward source. */
            yaw.cascade.acceleration_dps2 = -123456;
            CHECK(YawController_Update(&yaw, 0, start + dt, 1) == before);
        }
    }
    CHECK(absolute(impulse) < 0.00001f);
    CHECK(!yaw.shape.active && yaw.cascade.feedforward_nm == 0 && yaw.shape.position_deg == goal);
    CHECK(gain < 0.0001f ? !clipped : clipped);
    frame(start + duration + 2, 0, 0); CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(YawController_Update(&yaw, 0, start + duration + 2, 1) == 0);
}

void test_run(void)
{
    uint32_t dt, revision;
    YawController comparison;
    YawCascade loop;
    YawCascadeFeedforward ff;
    union { uint32_t bits; float value; } bad;
    test_stage = 1;
    YawController_Init(&yaw);
    CHECK(!yaw.ff.enabled && yaw.ff.acceleration_gain == 0 && yaw.ff.limit_nm == 0.1f);
    CHECK(!YawController_SetFeedforward(&yaw, 'E', 1));
    ready(100, 0);
    CHECK(!YawController_SetFeedforward(&yaw, 'B', 0));
    CHECK(!YawController_SetFeedforward(&yaw, 'V', 0));
    CHECK(!YawController_SetFeedforward(&yaw, 'C', 0));
    CHECK(!YawController_SetFeedforward(&yaw, 'S', 5));
    CHECK(!YawController_SetFeedforward(&yaw, 'A', -0.000001f));
    CHECK(!YawController_SetFeedforward(&yaw, 'A', 0.001001f));
    CHECK(!YawController_SetFeedforward(&yaw, 'L', -1));
    CHECK(!YawController_SetFeedforward(&yaw, 'L', 1.01f));
    CHECK(!YawController_SetFeedforward(&yaw, 'E', 0.5f));
    bad.bits = 0x7FC00000UL; CHECK(!YawController_SetFeedforward(&yaw, 'A', bad.value));
    bad.bits = 0x7F800000UL; CHECK(!YawController_SetFeedforward(&yaw, 'L', bad.value));
    revision = yaw.ff.revision;
    CHECK(YawController_SetFeedforward(&yaw, 'A', 0) && yaw.ff.revision == revision);

    test_stage = 2;
    accel_case(1000, 80, 30, 0.000002f, 0.1f);
    accel_case(2000, 160, 30, 0.000002f, 0.1f);
    accel_case(3000, 500, -30, 0.000002f, 0.1f);
    accel_case(4000, 160, -30, 0.001f, 0.04f);
    accel_case(0xFFFFFFD0UL, 160, 30, 0.000002f, 0.1f);

    test_stage = 3;
    /* Gain=0 with FF ON exactly preserves disabled profile output. */
    ready(6000, 0);
    CHECK(YawController_SetShape(&yaw, 160));
    CHECK(YawController_SetTarget(&yaw, 30));
    (void)YawController_Update(&yaw, 0, 6020, 1);
    comparison = yaw;
    CHECK(YawController_SetFeedforward(&comparison, 'E', 1));
    YawController_StartShape(&yaw, 6020); YawController_StartShape(&comparison, 6020);
    for (dt = 0; dt <= 200; dt += 2) {
        if (dt && dt % 10 == 0) {
            frame(6020 + dt, 0, 0);
            CHECK(YawController_OnFeedback(&yaw, &feedback));
            CHECK(YawController_OnFeedback(&comparison, &feedback));
        }
        CHECK(YawController_Update(&yaw, 0, 6020 + dt, 1) ==
              YawController_Update(&comparison, 0, 6020 + dt, 1));
        CHECK(yaw.cascade.speed_i == comparison.cascade.speed_i &&
              yaw.cascade.position_i == comparison.cascade.position_i);
    }
    /* No profile: nonzero gain and measured acceleration cannot apply FF. */
    ready(7000, 0);
    yaw.cascade.speed_i = 0.08f;
    CHECK(YawController_SetFeedforward(&yaw, 'A', 0.001f));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
    CHECK(yaw.cascade.speed_i == 0.08f);
    yaw.cascade.acceleration_dps2 = 123456;
    CHECK(YawController_SetTarget(&yaw, 30));
    (void)YawController_Update(&yaw, 0, 7020, 1);
    CHECK(yaw.cascade.feedforward_nm == 0);

    test_stage = 4;
    /* Total-torque anti-windup includes FF; opposite change can unwind. */
    YawCascade_Init(&loop);
    loop.position.kp = 1; loop.position.ki = loop.position.kd = 0;
    loop.speed.kp = 0.02f; loop.speed.ki = 0.1f; loop.speed.kd = 0;
    ff.torque_nm = 9.9f; ff.limited = 1;
    CHECK(YawCascade_UpdateTracking(&loop, 10, 0, 1, 10, &ff, 0) == 10);
    CHECK(YawCascade_UpdateTracking(&loop, 10, 10, 2, 10, &ff, 0) == 10);
    CHECK(loop.speed_i == 0 && loop.torque_limited && loop.feedforward_limited);
    ff.torque_nm = -9.9f; ff.limited = 0;
    CHECK(YawCascade_UpdateTracking(&loop, 10, 20, 3, 10, &ff, 0) < 0);
    CHECK(loop.speed_i > 0 && !loop.torque_limited && !loop.feedforward_limited);

    test_stage = 5;
    ready(8000, 0);
    CHECK(YawController_SetFeedforward(&yaw, 'A', 0.000002f));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
    CHECK(YawController_SetShape(&yaw, 160)); CHECK(YawController_SetTarget(&yaw, 30));
    (void)YawController_Update(&yaw, 0, 8020, 1);
    YawController_StartShape(&yaw, 8020);
    for (dt = 0; dt <= 40; dt += 2) {
        if (dt && dt % 10 == 0) { frame(8020 + dt, 0, 0); CHECK(YawController_OnFeedback(&yaw, &feedback)); }
        (void)YawController_Update(&yaw, 0, 8020 + dt, 1);
    }
    CHECK(yaw.cascade.feedforward_nm > 0);
    CHECK(YawController_SetFeedforward(&yaw, 'E', 0));
    (void)YawController_Update(&yaw, 0, 8062, 1); CHECK(yaw.cascade.feedforward_nm == 0);
    CHECK(!YawController_SetFeedforward(&yaw, 'E', 1));
    YawController_Stop(&yaw); CHECK(!yaw.torque_nm && !yaw.shape.active && !yaw.cascade.feedforward_nm);
    CHECK(YawController_CaptureReference(&yaw) && !yaw.ff.enabled && yaw.ff.acceleration_gain == 0.000002f);
    ready(9000, 0); yaw.ff.acceleration_gain = -1;
    CHECK(YawController_Update(&yaw, 0, 9020, 1) == 0 && (yaw.faults & YAW_CONTROLLER_FAULT_DATA));

    test_stage = 6;
    /* Cancellation paths must remove a genuinely nonzero acceleration pulse. */
    for (revision = 0; revision < 5; ++revision) {
        uint32_t start = 10000 + revision * 1000;
        ready(start - 20, 0);
        yaw.cascade.position.kp = yaw.cascade.position.ki = yaw.cascade.position.kd = 0;
        yaw.cascade.speed.kp = yaw.cascade.speed.ki = yaw.cascade.speed.kd = 0;
        CHECK(YawController_SetFeedforward(&yaw, 'A', 0.000002f));
        CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
        CHECK(YawController_SetShape(&yaw, 160));
        (void)YawController_Update(&yaw, 0, start, 1);
        CHECK(YawController_SetTarget(&yaw, 30)); YawController_StartShape(&yaw, start);
        for (dt = 2; dt <= 40; dt += 2) {
            if (dt % 10 == 0) { frame(start + dt, 0, 0); CHECK(YawController_OnFeedback(&yaw, &feedback)); }
            (void)YawController_Update(&yaw, 0, start + dt, 1);
        }
        CHECK(yaw.cascade.feedforward_nm > 0);
        if (revision == 0) { YawController_Stop(&yaw); }
        else if (revision == 1) {
            CHECK(YawController_SetTarget(&yaw, 0));
            (void)YawController_Update(&yaw, 0, start + 42, 1);
        } else if (revision == 2) { (void)YawController_Update(&yaw, 1, start + 42, 1); }
        else if (revision == 3) { (void)YawController_Update(&yaw, 0, start + 42, 0); }
        else {
            (void)YawController_Update(&yaw, 0, start + 140, 1);
            CHECK(yaw.faults & YAW_CONTROLLER_FAULT_FEEDBACK);
        }
        CHECK(!yaw.shape.active && yaw.cascade.feedforward_nm == 0 && yaw.torque_nm == 0);
    }
    test_done = 1;
}
