#define test_run startup_baseline_test_run
#include "test_gimbal_startup.c"
#undef test_run

void test_run(void)
{
    float held_y, held_p, yzero, pzero, speed_i;
    YawStep ystep;
    PitchStep pstep;
    char reply[1024];
    uint32_t before_y, before_p;
    test_stage = 1U;
    Reset(90000U); Run(400U, 1000U); Run(100U, 1500U);
    CHECK(yaw.active && pitch.active);
    follow_target = 1U; input.remote.channels[0] = input.remote.channels[1] = 1765U;
    Run(80U, 1500U);
    input.remote.channels[0] = input.remote.channels[1] = 1500U;
    follow_target = 0U;
    CHECK(YawBackend_SetParameter('T', yaw.controller.angle_deg + 5.0f));
    CHECK(PitchBackend_SetTarget(pitch.controller.angle_deg + 5.0f));
    Run(20U, 1500U);
    CHECK(yaw.controller.target_deg > yaw.controller.angle_deg + 4.0f);
    CHECK(pitch.controller.target_deg > pitch.controller.angle_deg + 4.0f);
    yzero = yaw.controller.reference_rad; pzero = pitch.controller.reference_rad;
    before_y = yc; before_p = pc;
    Run(2U, 2000U);
    held_y = yaw.controller.target_deg; held_p = pitch.controller.target_deg;
    CHECK(held_y > 1.0f && held_p > 1.0f);
    CHECK(fabsf(held_y - yaw.controller.angle_deg) < 0.001f);
    CHECK(fabsf(held_p - pitch.controller.angle_deg) < 0.001f);
    CHECK(yaw.active && pitch.active && yaw.step_hold && pitch.step_hold);
    CHECK(yaw.controller.reference_rad == yzero && pitch.controller.reference_rad == pzero);
    CHECK(pitch.controller.gravity_calibrated && pitch.controller.gravity_kg_nm == APP_PITCH_GRAVITY_KG_NM);
    CHECK(yc == before_y && pc == before_p); /* No disable/enable cycle. */
    speed_i = pitch.controller.cascade.speed_i;
    CHECK(PitchBackend_HoldCurrent()); Read();
    CHECK(pitch.controller.cascade.speed_i == speed_i && pitch.controller.cascade.position_i == 0.0f);
    speed_i = yaw.controller.cascade.speed_i;
    CHECK(YawBackend_HoldCurrent()); Read();
    CHECK(yaw.controller.cascade.speed_i == speed_i && yaw.controller.cascade.position_i == 0.0f);
    input.remote.channels[0] = 2000U; input.remote.channels[1] = 1000U;
    input.remote.channels[3] = 2000U;
    Run(80U, 2000U);
    CHECK(yaw.controller.target_deg == held_y && pitch.controller.target_deg == held_p);

    test_stage = 2U;
    Reset(100000U); Run(400U, 1000U); Run(100U, 1500U); Run(30U, 2000U);
    CHECK(PitchBackend_SetTarget(4.0f)); /* Pending old move on the companion. */
    input.remote.channels[3] = 2000U; input.remote.channels[6] = 2000U;
    Run(2U, 2000U);
    CHECK(yaw.controller.target_deg == 30.0f);
    CHECK(fabsf(pitch.controller.target_deg - pitch.controller.angle_deg) < 0.001f);
    CHECK(fabsf(pitch.controller.target_deg) < 0.01f && !pitch.controller.shape.active);
    held_p = pitch.controller.target_deg;
    follow_target = 1U; Run(200U, 2000U);
    CHECK(pitch.controller.target_deg == held_p && yaw.active && pitch.active);
    YawBackend_GetStep(&ystep); CHECK(ystep.phase == YAW_STEP_RUNNING);
    /* Request the other axis: the first axis is held where it really is,
     * and its interrupted measurement is reported as aborted, never a pass. */
    input.remote.channels[6] = 1500U; Run(10U, 2000U);
    input.remote.channels[3] = 1500U; input.remote.channels[2] = 2000U;
    input.remote.channels[6] = 2000U; Run(2U, 2000U);
    held_y = yaw.controller.target_deg;
    CHECK(pitch.controller.target_deg == 30.0f);
    CHECK(fabsf(held_y - yaw.controller.angle_deg) < 0.001f);
    YawBackend_GetStep(&ystep); CHECK(ystep.phase == YAW_STEP_ABORTED && !ystep.pass);
    Run(220U, 2000U); CHECK(yaw.controller.target_deg == held_y);
    input.remote.channels[6] = 1000U; Run(2U, 2000U);
    CHECK(pitch.controller.target_deg == 0.0f && yaw.controller.target_deg == held_y);
    Run(220U, 2000U); CHECK(yaw.controller.target_deg == held_y);

    test_stage = 3U;
    Reset(110000U); Run(400U, 1000U); Run(100U, 1500U); Run(30U, 2000U);
    CHECK(PitchBackend_SetTarget(4.0f));
    CHECK(YawConsole_Handle("YAW STEP", reply, sizeof(reply)) && strstr(reply, "Y|OK|STEP_STARTED"));
    Run(2U, 2000U);
    CHECK(yaw.controller.target_deg == 30.0f && fabsf(pitch.controller.target_deg) < 0.01f);
    follow_target = 1U; Run(220U, 2000U);
    CHECK(PitchConsole_Handle("PITCH STEP", reply, sizeof(reply)) && strstr(reply, "P|OK|STEP_STARTED"));
    Run(2U, 2000U); held_y = yaw.controller.target_deg;
    CHECK(pitch.controller.target_deg == 30.0f && fabsf(held_y - yaw.controller.angle_deg) < 0.001f);
    Run(220U, 2000U); CHECK(yaw.controller.target_deg == held_y);
    /* Healthy hold APIs cannot re-enable an axis after STOP, fault or loss. */
    AppTasks_RequestStop(); Read(); before_y = yc; before_p = pc;
    CHECK(!YawBackend_HoldCurrent() && !PitchBackend_HoldCurrent());
    Run(20U, 2000U); CHECK(!yaw.active && !pitch.active && yc == before_y && pc == before_p);
    CHECK(!YawBackend_HoldCurrent() && !PitchBackend_HoldCurrent());
    PitchBackend_GetStep(&pstep); CHECK(pstep.phase == PITCH_STEP_ABORTED && !pstep.pass);
    test_done = 1U;
}
