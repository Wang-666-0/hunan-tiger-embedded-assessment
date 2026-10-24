/* Real remote/target/backends plus the production view filter. The USART
 * command-to-mask mapping is independently checked in feeder_speed_view. */
#define test_run startup_baseline_test_run
#include "test_gimbal_startup.c"
#undef test_run

void test_run(void)
{
    float yheld, pheld, previous, speed_i;
    uint32_t enabled_y, enabled_p;
    PitchStep pstep;
    YawStep ystep;
    char reply[1024];
    test_stage = 1U;
    Reset(140000U); Run(400U, 1000U); Run(100U, 1500U);
    follow_target = 1U; input.remote.channels[0] = input.remote.channels[1] = 1765U;
    Run(80U, 1500U);
    input.remote.channels[0] = input.remote.channels[1] = 1500U; Run(40U, 1500U);
    follow_target = 0U;
    CHECK(YawBackend_SetParameter('T', yaw.controller.angle_deg + 5.0f)); Run(10U, 1500U);
    view_hold_request = APP_VIEW_HOLD_YAW; Run(2U, 1500U);
    yheld = yaw.controller.target_deg; previous = pitch.controller.target_deg;
    CHECK(yaw.active && pitch.active && yaw.view_locked && !yaw.view_hold_pending && !pitch.view_locked);
    CHECK(fabsf(yheld - yaw.controller.angle_deg) < 0.001f);
    input.remote.channels[0] = input.remote.channels[1] = 1765U;
    follow_target = 1U; Run(80U, 1500U);
    CHECK(yaw.controller.target_deg == yheld && pitch.controller.target_deg > previous + 3.5f);
    CHECK(!YawBackend_SetParameter('T', 0.0f) && !YawBackend_StartStep(30.0f));
    CHECK(yaw.step_blocks & YAW_STEP_BLOCK_VIEW);
    /* Repeated view requests and companion holds must not track a disturbance. */
    physical_yaw += 0.3f; follow_target = 0U; Run(20U, 1500U);
    CHECK(fabsf(yaw.controller.angle_deg - yheld) > 0.2f && yaw.controller.target_deg == yheld);
    CHECK(YawBackend_HoldCurrent()); Read(); CHECK(yaw.controller.target_deg == yheld);
    Run(20U, 1500U); CHECK(yaw.controller.target_deg == yheld && yaw.active && pitch.active);

    view_hold_request = APP_VIEW_HOLD_PITCH; Run(2U, 1500U);
    pheld = pitch.controller.target_deg; previous = yaw.controller.target_deg;
    CHECK(!yaw.view_locked && pitch.view_locked && !pitch.view_hold_pending);
    CHECK(fabsf(pheld - pitch.controller.angle_deg) < 0.001f);
    follow_target = 1U; Run(80U, 1500U);
    CHECK(pitch.controller.target_deg == pheld && yaw.controller.target_deg > previous + 3.5f);
    CHECK(!PitchBackend_SetTarget(0.0f) && !PitchBackend_StartStep(30.0f));
    physical_pitch -= 0.3f; follow_target = 0U; Run(20U, 1500U);
    speed_i = pitch.controller.cascade.speed_i;
    CHECK(PitchBackend_HoldCurrent()); Read();
    CHECK(pitch.controller.target_deg == pheld && pitch.controller.cascade.speed_i == speed_i);
    CHECK(pitch.controller.gravity_calibrated && pitch.controller.gravity_kg_nm == APP_PITCH_GRAVITY_KG_NM);
    view_hold_request = 0U; previous = pitch.controller.target_deg; Run(80U, 1500U);
    CHECK(!yaw.view_locked && !pitch.view_locked && pitch.controller.target_deg > previous + 3.5f);

    test_stage = 2U;
    Reset(150000U); view_hold_request = APP_VIEW_HOLD_YAW;
    Run(400U, 1000U); CHECK(!yc && !pc && !yaw.active && !pitch.active);
    CHECK(yaw.view_locked && yaw.view_hold_pending && !YawBackend_HoldCurrent());
    Run(100U, 1500U); Run(30U, 2000U);
    CHECK(yaw.active && pitch.active && !yaw.view_hold_pending);
    yheld = yaw.controller.target_deg;
    CHECK(PitchConsole_Handle("PITCH STEP", reply, sizeof(reply)) && strstr(reply, "P|OK|STEP_STARTED"));
    follow_target = 1U; Run(40U, 2000U);
    CHECK(pitch.controller.target_deg == 30.0f && yaw.controller.target_deg == yheld);
    input.remote.channels[3] = 2000U; input.remote.channels[6] = 2000U; Run(2U, 2000U);
    YawBackend_GetStep(&ystep); PitchBackend_GetStep(&pstep);
    CHECK(ystep.id == 0U && pstep.phase == PITCH_STEP_RUNNING && pitch.controller.target_deg == 30.0f);
    input.remote.channels[6] = 1000U; Run(2U, 2000U);
    CHECK(pitch.controller.target_deg == 30.0f && yaw.controller.target_deg == yheld);
    Run(120U, 2000U);
    input.remote.channels[3] = 1500U; input.remote.channels[6] = 1500U;
    view_hold_request = APP_VIEW_HOLD_PITCH; Run(2U, 2000U);
    pheld = pitch.controller.target_deg;
    PitchBackend_GetStep(&pstep); CHECK(pstep.phase == PITCH_STEP_ABORTED && !pstep.pass);
    CHECK(YawConsole_Handle("YAW STEP", reply, sizeof(reply)) && strstr(reply, "Y|OK|STEP_STARTED"));
    Run(40U, 2000U); CHECK(yaw.controller.target_deg == 30.0f && pitch.controller.target_deg == pheld);
    CHECK(!PitchBackend_SetTarget(0.0f) && !PitchBackend_StartStep(30.0f));

    test_stage = 3U;
    enabled_y = yc; enabled_p = pc; AppTasks_RequestStop(); Read();
    CHECK(pitch.view_locked && pitch.view_hold_pending && !PitchBackend_HoldCurrent());
    Run(40U, 2000U);
    CHECK(!yaw.active && !pitch.active && yc == enabled_y && pc == enabled_p);
    CHECK(pitch.view_locked && !YawBackend_HoldCurrent() && !PitchBackend_HoldCurrent());
    /* A view selected while stopped is a request only, never a rearm source. */
    view_hold_request = APP_VIEW_HOLD_YAW; Run(40U, 2000U);
    CHECK(yaw.view_locked && !pitch.view_locked && !yaw.active && !pitch.active);
    CHECK(yc == enabled_y && pc == enabled_p);
    test_done = 1U;
}
