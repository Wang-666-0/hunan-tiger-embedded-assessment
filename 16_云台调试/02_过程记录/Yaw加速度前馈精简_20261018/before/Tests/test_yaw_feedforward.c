#define test_run controller_fixture_not_run
#include "test_yaw_controller.c"
#undef test_run

void test_run(void)
{
    uint32_t t, revision;
    float previous;
    YawCascade plain, modeled;
    YawCascadeFeedforward model;
    union { uint32_t bits; float value; } bad;
    test_stage = 1U;
    YawController_Init(&yaw);
    CHECK(!yaw.ff.enabled && yaw.ff.bias_nm == 0 && yaw.ff.friction_velocity_dps == 5 && yaw.ff.limit_nm == 0.1f);
    CHECK(!YawController_SetFeedforward(&yaw, 'E', 1));
    ready(100U, 0);
    CHECK(!YawController_SetFeedforward(&yaw, 'B', 1.01f));
    CHECK(!YawController_SetFeedforward(&yaw, 'A', -0.0201f));
    CHECK(!YawController_SetFeedforward(&yaw, 'V', -0.0001f));
    CHECK(!YawController_SetFeedforward(&yaw, 'C', 0.501f));
    CHECK(!YawController_SetFeedforward(&yaw, 'S', 0));
    CHECK(!YawController_SetFeedforward(&yaw, 'L', -1));
    CHECK(!YawController_SetFeedforward(&yaw, 'E', 0.5f));
    bad.bits = 0x7FC00000UL; CHECK(!YawController_SetFeedforward(&yaw, 'B', bad.value));
    bad.bits = 0x7F800000UL; CHECK(!YawController_SetFeedforward(&yaw, 'V', bad.value));
    revision = yaw.ff.revision;
    CHECK(YawController_SetFeedforward(&yaw, 'B', 0) && yaw.ff.revision == revision);

    test_stage = 2U;
    YawCascade_Init(&plain); modeled = plain;
    memset(&model, 0, sizeof(model)); model.friction_velocity_dps = 5; model.limit_nm = 0.1f;
    for (t = 0; t <= 100; t += 2) {
        uint32_t stamp = t / 10U * 10U;
        CHECK(YawCascade_Update(&plain, 5, stamp, stamp + 1, 10) ==
            YawCascade_UpdateWithCompensation(&modeled, 5, stamp, stamp + 1, 10, &model));
        CHECK(plain.position_i == modeled.position_i && plain.speed_i == modeled.speed_i);
    }
    /* Only feedforward remains: verify actual angle, reference velocity, signs and cap. */
    ready(1000U, 0);
    yaw.cascade.position.kp = 1; yaw.cascade.position.ki = yaw.cascade.position.kd = 0;
    yaw.cascade.speed.kp = yaw.cascade.speed.ki = yaw.cascade.speed.kd = 0;
    frame(1040U, 10 * DEG_TO_RAD, 0); CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(YawController_SetTarget(&yaw, 20));
    CHECK(YawController_SetFeedforward(&yaw, 'B', 0.02f));
    CHECK(YawController_SetFeedforward(&yaw, 'A', 0.001f));
    CHECK(YawController_SetFeedforward(&yaw, 'V', 0.001f));
    CHECK(YawController_SetFeedforward(&yaw, 'C', 0.03f));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
    CHECK(near(YawController_Update(&yaw, 0, 1040U, 1), 0.06f, 0.00001f));
    CHECK(near(yaw.pid_torque_nm, 0, 0.000001f));
    CHECK(YawController_SetFeedforward(&yaw, 'L', 0.04f));
    CHECK(near(YawController_Update(&yaw, 0, 1042U, 1), 0.04f, 0.00001f) && yaw.cascade.feedforward_limited);
    CHECK(YawController_SetTarget(&yaw, 0));
    CHECK(near(YawController_Update(&yaw, 0, 1044U, 1), 0, 0.00001f));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 0));
    CHECK(YawController_Update(&yaw, 0, 1046U, 1) == 0);

    test_stage = 3U;
    ready(2000U, 0);
    (void)YawController_Update(&yaw, 0, 2020U, 1);
    yaw.cascade.speed_i = 0.08f;
    previous = YawController_Update(&yaw, 0, 2022U, 1);
    CHECK(YawController_SetFeedforward(&yaw, 'B', 0.05f));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
    CHECK(yaw.ff.i_transferred && near(yaw.cascade.speed_i, 0.03f, 0.00001f));
    CHECK(near(YawController_Update(&yaw, 0, 2024U, 1), previous, 0.00001f));
    revision = yaw.ff.revision;
    CHECK(YawController_SetFeedforward(&yaw, 'B', 0.05f) && yaw.ff.revision == revision);
    CHECK(YawController_SetFeedforward(&yaw, 'E', 0));
    CHECK(near(YawController_Update(&yaw, 0, 2026U, 1), previous, 0.00001f));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
    CHECK(YawController_SetFeedforward(&yaw, 'L', 1));
    yaw.cascade.speed_i = 9.99f;
    CHECK(YawController_SetFeedforward(&yaw, 'B', -1));
    CHECK(yaw.cascade.speed_i == 10 && !yaw.ff.i_transferred);
    YawController_Stop(&yaw);
    CHECK(yaw.ff.enabled && yaw.ff.bias_nm == -1 && !yaw.torque_nm && !yaw.cascade.feedforward_nm);
    CHECK(YawController_CaptureReference(&yaw) && !yaw.ff.enabled && yaw.ff.bias_nm == -1);

    test_stage = 4U;
    ready(3000U, 0);
    CHECK(YawController_SetFeedforward(&yaw, 'B', 1));
    CHECK(YawController_SetFeedforward(&yaw, 'L', 1));
    CHECK(YawController_SetFeedforward(&yaw, 'E', 1));
    yaw.cascade.position.kp = 30; yaw.cascade.speed.kp = 0.02f; yaw.cascade.speed.ki = 0.2f;
    CHECK(YawController_SetTarget(&yaw, 30));
    CHECK(YawController_Update(&yaw, 0, 3020U, 1) == 10);
    for (t = 3022U; t <= 3080U; t += 2U) {
        if (t % 10U == 0) { frame(t, 0, 0); CHECK(YawController_OnFeedback(&yaw, &feedback)); }
        CHECK(YawController_Update(&yaw, 0, t, 1) == 10 && yaw.cascade.speed_i == 0);
    }
    CHECK(YawController_Update(&yaw, 0, 3082U, 0) == 0 && !yaw.cascade.feedforward_nm);
    frame(3180U, 0, 0); CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(!yaw.ff.enabled && !yaw.have_reference && (yaw.faults & YAW_CONTROLLER_FAULT_FEEDBACK));
    ready(4000U, 0);
    yaw.ff.friction_velocity_dps = 0;
    CHECK(YawController_Update(&yaw, 0, 4020U, 1) == 0 && (yaw.faults & YAW_CONTROLLER_FAULT_DATA));
    test_done = 1U;
}
