/* Real remote/target/backends plus the production view filter. The USART
 * command-to-mask mapping is independently checked in feeder_speed_view. */
#define test_run startup_baseline_test_run
#include "test_gimbal_startup.c"
#undef test_run

void test_run(void)
{
    float yheld, pheld, previous, speed_i, position_i, equilibrium;
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
    test_stage = 4U;
    /* A load-supporting outer I must survive entry into the stiffer hold. */
    Reset(170000U); Run(400U, 1000U); Run(100U, 1500U);
    CHECK(PitchBackend_SetTarget(0.1f)); Run(400U, 1500U);
    position_i = pitch.controller.cascade.position_i;
    speed_i = pitch.controller.cascade.speed_i;
    CHECK(position_i > 0.5f);
    view_hold_request = APP_VIEW_HOLD_PITCH; Run(2U, 1500U);
    pheld = pitch.view_hold_deg;
    CHECK(pitch.view_locked && pitch.controller.target_deg == pheld && !pitch.view_hold_pending);
    CHECK(pitch.controller.cascade.position_i == 0.0f);
    CHECK(fabsf(pitch.controller.cascade.speed_i -
        (speed_i + pitch.controller.cascade.speed.kp * position_i)) < 0.00001f);
    equilibrium = pitch.controller.torque_nm;
    /* Inject a physical displacement, not a joystick/target change. Real C
     * feedback must demand a restoring torque while retaining the old pose. */
    physical_pitch += 0.5f; Run(40U, 1500U);
    CHECK(pitch.controller.angle_deg > pheld + 0.45f);
    CHECK(pitch.controller.target_deg == pheld && pitch.view_hold_deg == pheld);
    CHECK(pitch.controller.torque_nm < equilibrium - 0.015f);
    CHECK(pitch.controller.cascade.position.kp == APP_PITCH_POSITION_KP &&
        pitch.controller.cascade.speed.kp == APP_PITCH_SPEED_KP);
    CHECK(PitchBackend_HoldCurrent()); Read(); CHECK(pitch.view_hold_deg == pheld);
    view_hold_request = APP_VIEW_HOLD_YAW; Run(2U, 1500U);
    CHECK(!pitch.view_locked && !pitch.controller.faults && pitch.active);
    CHECK(pitch.controller.cascade.position.kp == APP_PITCH_POSITION_KP &&
        pitch.controller.cascade.speed.kp == APP_PITCH_SPEED_KP);
    test_stage = 5U;
    /* Reproduce the user's V2 pending snapshot: Yaw remains armed/running,
     * Pitch is healthy but its requested session was stopped; target is old. */
    Reset(180000U); Run(400U, 1000U); Run(100U, 1500U);
    view_hold_request = APP_VIEW_HOLD_YAW; Run(2U, 1500U);
    CHECK(PitchBackend_SetTarget(-18.0f)); follow_target = 1U; Run(100U, 1500U); follow_target = 0U;
    GimbalStartup_ManualOverride(); /* Existing single-axis READY/ON workflow. */
    CHECK(PitchBackend_Request(0U)); Run(50U, 1500U);
    CHECK(yaw.active && logic.safety.operator_armed && logic.safety.output_allowed);
    CHECK(!pitch.active && !pitch.requested && !pitch.stop_pending && !pitch.controller.faults);
    CHECK(pitch.controller.target_deg == -18.0f && pitch.controller.torque_nm == 0.0f);
    physical_pitch = 99.375f; Run(20U, 1500U);
    enabled_p = pc;
    view_hold_request = APP_VIEW_HOLD_PITCH; Run(50U, 1500U);
    CHECK(pitch.active && pitch.requested && pitch.view_locked && !pitch.view_hold_pending);
    CHECK(pc == enabled_p + 1U && yaw.active && !pitch.controller.faults);
    CHECK(fabsf(pitch.controller.target_deg - pitch.controller.angle_deg) < 0.01f);
    CHECK(pitch.controller.target_deg > -1.0f && pitch.controller.target_deg < -0.25f);
    enabled_p = pc;
    CHECK(PitchBackend_Request(0U)); Run(50U, 1500U);
    CHECK(!pitch.active && !pitch.requested && !pitch.view_prepare_pending && pc == enabled_p);
    CHECK(pitch.view_hold_suspended);
    /* A repeated explicit YAW VIEW can request a new session without changing
     * an already applied lockpoint or undoing the global rearm barrier. */
    PitchBackend_PrepareViewHold(); Run(50U, 1500U);
    CHECK(pitch.active && pitch.requested && pc == enabled_p + 1U && !pitch.view_hold_pending);
    pheld = pitch.view_hold_deg; PitchBackend_PrepareViewHold(); Run(20U, 1500U);
    CHECK(pitch.view_hold_deg == pheld && !pitch.view_prepare_pending);
    enabled_p = pc; AppTasks_RequestStop(); PitchBackend_PrepareViewHold(); Run(80U, 1500U);
    CHECK(!pitch.active && !pitch.requested && pc == enabled_p);
    test_stage = 6U;
    Reset(190000U); view_hold_request = APP_VIEW_HOLD_PITCH; PitchBackend_PrepareViewHold();
    Run(400U, 1000U); CHECK(!pc && !yc && !pitch.active);
    Run(100U, 1500U); CHECK(pc == 1U && pitch.active && !pitch.view_hold_pending);
    test_stage = 7U;
    /* Real workflow missed by V3: choose YAW VIEW, then READY/global STOP
     * clears the one-shot ticket, prepare Yaw alone and unlock again. The
     * persistent selected view must restore the companion after that unlock. */
    Reset(200000U); Run(400U, 1000U); Run(100U, 1500U);
    CHECK(PitchBackend_SetTarget(0.64f)); follow_target = 1U; Run(30U, 1500U); follow_target = 0U;
    view_hold_request = APP_VIEW_HOLD_PITCH; PitchBackend_PrepareViewHold(); Run(2U, 1500U);
    CHECK(pitch.active && !pitch.view_hold_pending);
    GimbalStartup_ManualOverride(); AppTasks_RequestStop(); Run(40U, 1000U);
    physical_pitch = 102.12f; Run(20U, 1000U);
    CHECK(!pitch.active && !pitch.requested && pitch.view_hold_pending);
    CHECK(pitch.view_prepare_pending && !pitch.view_hold_suspended);
    CHECK(YawBackend_Request(1U)); Run(40U, 1000U); Run(100U, 1500U);
    CHECK(yaw.active && logic.safety.operator_armed && logic.safety.output_allowed);
    CHECK(pitch.active && pitch.requested && !pitch.view_hold_pending);
    CHECK(fabsf(pitch.controller.target_deg - 2.12f) < 0.02f && pitch.controller.torque_nm != 0.0f);
    test_stage = 8U;
    /* A large error accumulated during motion is not static load support. */
    Reset(210000U); Run(400U, 1000U); Run(100U, 1500U);
    CHECK(PitchBackend_SetTarget(10.0f)); Run(40U, 1500U);
    CHECK(pitch.controller.cascade.position_i > 1.0f); speed_i = pitch.controller.cascade.speed_i;
    view_hold_request = APP_VIEW_HOLD_PITCH; Run(2U, 1500U);
    CHECK(pitch.active && !pitch.view_hold_pending && pitch.controller.cascade.position_i == 0.0f);
    CHECK(fabsf(pitch.controller.cascade.speed_i - speed_i) < 0.000001f);
    /* First arrival can have small raw velocity but a still-fast observer.
     * Reaching the angle alone must not transfer the old motion integral. */
    Reset(220000U); Run(400U, 1000U); Run(100U, 1500U);
    CHECK(PitchBackend_SetTarget(10.0f)); Run(40U, 1500U);
    physical_pitch += 0.5f; Run(10U, 1500U);
    CHECK(fabsf(pitch.controller.cascade.velocity_dps) > PITCH_CONTROLLER_STATIONARY_SPEED_DPS);
    CHECK(PitchBackend_SetTarget(pitch.controller.angle_deg)); speed_i = pitch.controller.cascade.speed_i;
    view_hold_request = APP_VIEW_HOLD_PITCH; Run(2U, 1500U);
    CHECK(pitch.active && !pitch.view_hold_pending && pitch.controller.cascade.position_i == 0.0f);
    CHECK(fabsf(pitch.controller.cascade.speed_i - speed_i) < 0.000001f);
    test_done = 1U;
}
