/* Exercise both real backends/controllers with the existing CAN/RC integration fixture. */
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
    YawStep measured;
    uint32_t end, y_enables, p_enables, t;
    float held, reference, kg;
    char reply[1200];
    test_stage = 1U;
    Active(0U);
    CHECK(PitchBackend_SetTarget(5.0f)); Run(300U, 1500U);
    held = pitch.controller.angle_deg; reference = pitch.controller.reference_rad;
    kg = pitch.controller.gravity_kg_nm;
    CHECK(PitchBackend_SetTarget(6.0f)); /* Pending target differs from the measured hold pose. */
    CHECK(YawBackend_StartStep(30.0f)); Read();
    CHECK(pitch.peer_step_locked && !yaw.peer_step_locked && yaw.controller.target_deg == 30.0f);
    CHECK(pitch.controller.target_deg == held && pitch.active && pp);
    CHECK(pitch.controller.gravity_calibrated && pitch.controller.gravity_kg_nm == kg);
    CHECK(pitch.controller.reference_rad == reference);
    CHECK(PitchConsole_Handle("PITCH STATUS", reply, sizeof(reply)) && strstr(reply, "|PEER_STEP_LOCK|1|"));
    CHECK(!PitchBackend_SetTarget(0.0f) && !PitchBackend_StartStep(30.0f));
    CHECK(!PitchBackend_SetLoopParameter('P', 'P', 12.0f));
    CHECK(!PitchBackend_SetGravityGain(0.0f));
    input.remote.channels[1] = 1765U; Run(100U, 1500U);
    CHECK(pitch.peer_step_locked && pitch.controller.target_deg == held && pitch.active);
    CHECK(pitch.controller.reference_rad == reference && pitch.controller.gravity_calibrated);
    /* A disturbance must be corrected against the original pose, never recaptured. */
    follow_target = 0U; physical_pitch += 0.15f; Run(30U, 1500U);
    CHECK(pitch.controller.target_deg == held && PitchBackend_SetStepLock(1U)); Read();
    CHECK(pitch.controller.target_deg == held && pitch.peer_step_locked && yaw.active);

    test_stage = 2U;
    input.remote.channels[1] = 1500U;
    YawBackend_GetStep(&measured); end = measured.start_ms + YAW_STEP_WINDOW_MS;
    /* Ten-ms worst permitted control gap keeps the full-window lifecycle test bounded. */
    while ((int32_t)(end - now) > 0) { Tick(now + 10U, 1500U, 1U, 1U, 1U); }
    YawBackend_GetStep(&measured); Read();
    CHECK(measured.phase == YAW_STEP_DONE && !pitch.peer_step_locked);
    CHECK(pitch.active && pitch.controller.target_deg == held && pitch.controller.reference_rad == reference);
    input.remote.channels[1] = 1765U; Run(20U, 1500U);
    CHECK(pitch.controller.target_deg > held + 0.8f && pitch.active);

    test_stage = 3U;
    Active(10000U);
    CHECK(YawBackend_SetParameter('T', 7.0f)); Run(300U, 1500U);
    held = yaw.controller.angle_deg; reference = yaw.controller.reference_rad;
    CHECK(YawBackend_SetParameter('T', 8.0f));
    CHECK(PitchBackend_StartStep(30.0f)); Read();
    CHECK(yaw.peer_step_locked && yaw.controller.target_deg == held && yaw.active && yp);
    CHECK(YawConsole_Handle("YAW STATUS", reply, sizeof(reply)) && strstr(reply, "|PEER_STEP_LOCK|1"));
    CHECK(!YawBackend_SetParameter('T', 0.0f) && !YawBackend_StartStep(30.0f));
    CHECK(!YawBackend_SetLoopParameter('P', 'P', 12.0f));
    CHECK(!YawBackend_SetShape(0U) && !YawBackend_SetFeedforward('E', 0.0f));
    input.remote.channels[0] = 1765U; Run(100U, 1500U);
    CHECK(yaw.controller.target_deg == held && yaw.active && yaw.controller.reference_rad == reference);
    CHECK(PitchBackend_SetTarget(0.0f)); Read();
    CHECK(!yaw.peer_step_locked && yaw.controller.target_deg == held);
    PitchBackend_GetStep(&measured); CHECK(measured.phase == PITCH_STEP_ABORTED && !measured.pass);
    Run(20U, 1500U); CHECK(yaw.controller.target_deg > held + 0.8f);

    test_stage = 4U;
    Active(20000U);
    PitchBackend_Stop(); Read();
    CHECK(!YawBackend_StartStep(30.0f)); Read();
    CHECK(yaw.controller.target_deg == 0.0f && !pitch.peer_step_locked);
    YawBackend_GetStep(&measured); CHECK(measured.phase == YAW_STEP_IDLE);
    Active(30000U); CHECK(YawBackend_StartStep(-30.0f));
    PitchBackend_Stop(); Run(20U, 1500U); Read();
    CHECK(!yaw.active && !pitch.active && !yaw.peer_step_locked && !pitch.peer_step_locked);
    YawBackend_GetStep(&measured); CHECK(measured.phase == YAW_STEP_ABORTED && !measured.pass);
    y_enables = yc; p_enables = pc; Run(100U, 1500U);
    CHECK(yc == y_enables && pc == p_enables);

    test_stage = 5U;
    Active(40000U); CHECK(PitchBackend_StartStep(30.0f));
    for (t = 0U; t < 130U; t += 2U) { Tick(now + 2U, 1500U, 1U, 0U, 1U); }
    Read(); CHECK(!yaw.active && !pitch.active && !yaw.peer_step_locked && !pitch.peer_step_locked);
    PitchBackend_GetStep(&measured); CHECK(measured.phase == PITCH_STEP_ABORTED && !measured.pass);
    Active(50000U); CHECK(YawBackend_StartStep(30.0f)); AppTasks_RequestStop(); Read();
    CHECK(!yaw.active && !pitch.active && !yaw.peer_step_locked && !pitch.peer_step_locked);
    YawBackend_GetStep(&measured); CHECK(measured.phase == YAW_STEP_ABORTED && !measured.pass);

    test_stage = 6U;
    Active(60000U); Run(30U, 2000U);
    input.remote.channels[3] = 2000U; input.remote.channels[6] = 1500U; Run(20U, 2000U);
    input.remote.channels[6] = 2000U; Run(20U, 2000U); Read();
    YawBackend_GetStep(&measured);
    CHECK(measured.phase == YAW_STEP_RUNNING && pitch.peer_step_locked && pitch.active);
    input.remote.channels[6] = 1000U; Run(20U, 2000U); Read();
    CHECK(!pitch.peer_step_locked && yaw.controller.target_deg == 0.0f);

    test_stage = 7U;
    Active(70000U); CHECK(PitchBackend_StartStep(30.0f));
    PitchBackend_GetStep(&measured); end = measured.start_ms + PITCH_STEP_WINDOW_MS;
    while ((int32_t)(end - now) > 0) { Tick(now + 10U, 1500U, 1U, 1U, 1U); }
    PitchBackend_GetStep(&measured); Read();
    CHECK(measured.phase == PITCH_STEP_DONE && !yaw.peer_step_locked && yaw.active && pitch.active);
    Active(80000U); Run(30U, 2000U);
    input.remote.channels[2] = 2000U; input.remote.channels[6] = 1500U; Run(20U, 2000U);
    input.remote.channels[6] = 2000U; Run(20U, 2000U); Read();
    PitchBackend_GetStep(&measured);
    CHECK(measured.phase == PITCH_STEP_RUNNING && yaw.peer_step_locked && yaw.active);
    YawBackend_Stop(); Run(20U, 2000U); Read();
    CHECK(!yaw.active && !pitch.active && !yaw.peer_step_locked && !pitch.peer_step_locked);
    PitchBackend_GetStep(&measured); CHECK(measured.phase == PITCH_STEP_ABORTED && !measured.pass);
    test_done = 1U;
}
