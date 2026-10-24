/* Real dual-axis backends/controllers; only Yaw STEP acquires a Pitch hold. */
#define test_run startup_fixture_not_run
#include "test_gimbal_startup.c"
#undef test_run

static void Active(uint32_t base)
{
    Reset(base); Run(400U, 1000U); Run(100U, 1500U);
    CHECK(yaw.active && pitch.active && yp && pp);
    follow_target = 1U;
}

void test_run(void)
{
    YawStep measurement;
    uint32_t end, t, y_enables, p_enables;
    float held, reference, kg;
    test_stage = 1U;
    Active(0U); CHECK(PitchBackend_SetTarget(5.0f)); Run(300U, 1500U);
    held = pitch.controller.angle_deg; reference = pitch.controller.reference_rad;
    kg = pitch.controller.gravity_kg_nm;
    CHECK(PitchBackend_SetTarget(6.0f)); /* Capture the actual pose, not an outstanding target. */
    CHECK(YawBackend_StartStep(30.0f)); Read();
    CHECK(pitch.yaw_step_hold && pitch.active && pp && pitch.controller.target_deg == held);
    CHECK(pitch.controller.gravity_calibrated && pitch.controller.gravity_kg_nm == kg);
    CHECK(pitch.controller.reference_rad == reference);
    CHECK(!PitchBackend_SetTarget(0.0f) && !PitchBackend_StartStep(30.0f));
    CHECK(!PitchBackend_SetLoopParameter('P', 'P', 12.0f) && !PitchBackend_SetGravityGain(0.0f));
    input.remote.channels[1] = 1765U; Run(100U, 1500U);
    CHECK(pitch.controller.target_deg == held && pitch.yaw_step_hold && pitch.active);
    follow_target = 0U; physical_pitch += 0.15f; Run(30U, 1500U);
    CHECK(PitchBackend_HoldForYawStep(1U)); Read();
    CHECK(pitch.controller.target_deg == held && pitch.controller.reference_rad == reference);
    CHECK(YawBackend_SetLoopPid('P', APP_YAW_POSITION_KP, APP_YAW_POSITION_KI, APP_YAW_POSITION_KD));
    Read(); CHECK(pitch.yaw_step_hold); /* Same-value writes must not end a measurement. */
    input.remote.channels[1] = 1500U;
    YawBackend_GetStep(&measurement); end = measurement.start_ms + YAW_STEP_WINDOW_MS;
    /* Lifecycle only: the original maximum permitted 10ms control gap bounds emulation work. */
    while ((int32_t)(end - now) > 0) { Tick(now + 10U, 1500U, 1U, 1U, 1U); }
    YawBackend_GetStep(&measurement); Read();
    CHECK(measurement.phase == YAW_STEP_DONE && !pitch.yaw_step_hold && pitch.active);
    CHECK(pitch.controller.target_deg == held && pitch.controller.reference_rad == reference);
    input.remote.channels[1] = 1765U; Run(20U, 1500U);
    CHECK(pitch.controller.target_deg > held + 0.8f);

    test_stage = 2U;
    Active(10000U); CHECK(YawBackend_StartStep(-30.0f)); Read(); CHECK(pitch.yaw_step_hold);
    CHECK(YawBackend_SetParameter('T', 0.0f)); Read(); CHECK(!pitch.yaw_step_hold && pitch.active);
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_ABORTED);
    CHECK(YawBackend_StartStep(30.0f));
    CHECK(YawBackend_SetLoopParameter('P', 'P', 14.0f)); Read();
    CHECK(!pitch.yaw_step_hold && pitch.active);

    test_stage = 3U;
    Active(20000U); CHECK(PitchBackend_StartStep(30.0f)); Read();
    CHECK(!pitch.yaw_step_hold && !YawBackend_StartStep(30.0f));
    input.remote.channels[0] = 1765U; Run(100U, 1500U);
    CHECK(yaw.controller.target_deg > 4.4f && yaw.active && pitch.active);
    PitchBackend_GetStep(&measurement); CHECK(measurement.phase == PITCH_STEP_RUNNING);

    test_stage = 4U;
    Active(30000U); PitchBackend_Stop();
    CHECK(!YawBackend_StartStep(30.0f)); Read(); CHECK(yaw.controller.target_deg == 0.0f);
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_IDLE);
    Active(40000U); CHECK(YawBackend_StartStep(30.0f)); PitchBackend_Stop(); Run(20U, 1500U);
    CHECK(!yaw.active && !pitch.active && !pitch.yaw_step_hold);
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_ABORTED && !measurement.pass);
    y_enables = yc; p_enables = pc; Run(100U, 1500U); CHECK(yc == y_enables && pc == p_enables);
    Active(50000U); CHECK(YawBackend_StartStep(30.0f));
    for (t = 0U; t < 130U; t += 2U) { Tick(now + 2U, 1500U, 1U, 1U, 0U); }
    CHECK(!yaw.active && !pitch.active && !pitch.yaw_step_hold);
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_ABORTED && !measurement.pass);

    test_stage = 5U;
    Active(60000U); Run(30U, 2000U);
    input.remote.channels[3] = 2000U; input.remote.channels[6] = 1500U; Run(20U, 2000U);
    input.remote.channels[6] = 2000U; Run(20U, 2000U); Read();
    YawBackend_GetStep(&measurement); CHECK(measurement.phase == YAW_STEP_RUNNING && pitch.yaw_step_hold);
    input.remote.channels[6] = 1000U; Run(20U, 2000U); CHECK(!pitch.yaw_step_hold);
    Active(70000U); CHECK(YawBackend_StartStep(30.0f)); AppTasks_RequestStop(); Read();
    CHECK(!pitch.yaw_step_hold && !yaw.active && !pitch.active);
    test_done = 1U;
}
