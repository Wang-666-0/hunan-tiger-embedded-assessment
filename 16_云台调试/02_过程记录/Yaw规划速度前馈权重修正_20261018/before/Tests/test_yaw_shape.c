#define test_run controller_fixture_not_run
#include "test_yaw_controller.c"
#undef test_run

static void shape_case(uint32_t start, uint32_t duration, float goal)
{
    uint32_t dt;
    float previous = 0, direction = goal > 0 ? 1.0f : -1.0f;
    ready(start - 20U, 0);
    CHECK(YawController_SetShape(&yaw, duration));
    (void)YawController_Update(&yaw, 0, start, 1);
    yaw.cascade.position_i = 0.25f;
    CHECK(YawController_SetTarget(&yaw, goal));
    YawController_StartShape(&yaw, start);
    CHECK(yaw.shape.active && yaw.shape.position_deg == 0);
    for (dt = 0; dt <= duration; dt += 2) {
        if (dt && dt % 10 == 0) {
            frame(start + dt, 0, 0); CHECK(YawController_OnFeedback(&yaw, &feedback));
        }
        (void)YawController_Update(&yaw, 0, start + dt, 1);
        CHECK(!yaw.faults && yaw.target_deg == goal);
        CHECK(direction * yaw.shape.position_deg >= previous - 0.0001f);
        CHECK(direction * yaw.shape.position_deg <= absolute(goal) + 0.0001f);
        CHECK(direction * yaw.shape.velocity_dps >= 0);
        previous = direction * yaw.shape.position_deg;
        if (!dt) {
            CHECK(yaw.shape.velocity_dps == 0 && yaw.shape.acceleration_dps2 == 0);
        }
        if (dt < duration) { CHECK(yaw.cascade.position_i == 0.25f); }
        if (dt == duration / 2U) {
            CHECK(near(yaw.shape.position_deg, goal / 2, 0.0001f));
            CHECK(near(yaw.shape.velocity_dps, goal * 1.875f / (duration * 0.001f), 0.002f));
            CHECK(absolute(yaw.shape.acceleration_dps2) < 0.001f);
        }
    }
    CHECK(!yaw.shape.active && yaw.shape.position_deg == goal);
    CHECK(yaw.shape.velocity_dps == 0 && yaw.shape.acceleration_dps2 == 0);
    CHECK(direction * (yaw.cascade.position_i - 0.25f) > 0);
}

void test_run(void)
{
    uint32_t dt;
    YawCascade plain, tracking;
    YawCascadeReference reference;
    test_stage = 1;
    YawController_Init(&yaw);
    CHECK(!yaw.shape.duration_ms && !yaw.shape.active);
    CHECK(!YawController_SetShape(0, 160));
    CHECK(!YawController_SetShape(&yaw, 79) && !YawController_SetShape(&yaw, 501));
    CHECK(!YawController_SetShape(&yaw, 0xFFFFFFFFUL));
    CHECK(YawController_SetShape(&yaw, 80) && YawController_SetShape(&yaw, 500));
    CHECK(YawController_SetShape(&yaw, 0));
    YawController_StartShape(0, 0);
    YawController_StartShape(&yaw, 0); CHECK(!yaw.shape.active);

    test_stage = 2;
    shape_case(1000, 80, 30);
    shape_case(2000, 160, 30);
    shape_case(3000, 500, -30);
    shape_case(0xFFFFFFD0UL, 160, -30); /* Exact MCU tick rollover. */

    test_stage = 3;
    YawCascade_Init(&plain); tracking = plain;
    for (dt = 0; dt <= 100; dt += 2) {
        uint32_t stamp = dt / 10 * 10;
        CHECK(YawCascade_Update(&plain, 3, stamp, stamp + 1, 10) ==
            YawCascade_UpdateTracking(&tracking, 3, stamp, stamp + 1, 10, 0, 0));
        CHECK(plain.position_i == tracking.position_i && plain.speed_i == tracking.speed_i);
    }
    YawCascade_Init(&tracking);
    tracking.position.kp = 9; tracking.position.ki = 0.2f; tracking.position.kd = 0.1f;
    tracking.speed.kp = 0.003f; tracking.speed.ki = tracking.speed.kd = 0;
    tracking.position_i = 0.5f; tracking.velocity_dps = 100;
    reference.velocity_dps = 100; reference.freeze_position_i = 1;
    CHECK(near(YawCascade_UpdateTracking(&tracking, 0, 0, 1, 10, 0, &reference), 0.0015f, 0.00001f));
    CHECK(near(tracking.target_velocity_dps, 100.5f, 0.0001f));
    CHECK(near(YawCascade_UpdateTracking(&tracking, 0, 10, 2, 10, 0, &reference), 0.0015f, 0.00001f));

    test_stage = 4;
    ready(4000, 0); (void)YawController_Update(&yaw, 0, 4020, 1);
    CHECK(YawController_SetShape(&yaw, 160)); CHECK(YawController_SetTarget(&yaw, 30));
    YawController_StartShape(&yaw, 4020);
    CHECK(!YawController_SetShape(&yaw, 180));
    CHECK(YawController_SetTarget(&yaw, 0) && !yaw.shape.active);
    CHECK(yaw.shape.position_deg == 0 && yaw.shape.velocity_dps == 0);
    CHECK(YawController_SetTarget(&yaw, 30)); YawController_StartShape(&yaw, 4020);
    (void)YawController_Update(&yaw, 1, 4022, 1); CHECK(!yaw.shape.active);
    YawController_StartShape(&yaw, 4022);
    CHECK(YawController_Update(&yaw, 0, 4024, 0) == 0 && !yaw.shape.active);
    CHECK(yaw.shape.duration_ms == 160 && yaw.shape.velocity_dps == 0);
    (void)YawController_Update(&yaw, 0, 4026, 1);
    CHECK(YawController_SetTarget(&yaw, 30)); YawController_StartShape(&yaw, 4026);
    CHECK(YawController_Update(&yaw, 0, 4120, 1) == 0 && !yaw.shape.active && yaw.faults);
    ready(5000, 0); (void)YawController_Update(&yaw, 0, 5020, 1);
    CHECK(YawController_SetShape(&yaw, 160)); CHECK(YawController_SetTarget(&yaw, 30));
    YawController_StartShape(&yaw, 5020); yaw.shape.duration_ms = 0;
    CHECK(YawController_Update(&yaw, 0, 5022, 1) == 0 && (yaw.faults & YAW_CONTROLLER_FAULT_DATA));
    test_done = 1;
}
