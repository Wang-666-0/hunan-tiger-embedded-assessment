/* Replay positions through the actual controller. This is not a plant/performance model. */
#define test_run original_pitch_controller_test_run
#include "test_pitch_controller.c"
#undef test_run

static void brake_ready(uint32_t ms, float start, float end)
{
    ready(ms, 0.0f);
    CHECK(PitchController_SetPid(&pitch, 'P', 0.0f, 0.0f, 0.0f));
    CHECK(PitchController_SetPid(&pitch, 'V', 0.0f, 0.0f, 0.0f));
    sample(ms + 32U, start, 0.0f, 1U);
    CHECK(PitchController_SetTarget(&pitch, start));
    CHECK(near(PitchController_Update(&pitch, 0.0f, ms + 32U, 1U), 0.0f));
    CHECK(PitchController_SetTarget(&pitch, end));
    CHECK(pitch.brake_tracking && !pitch.brake_arrived && pitch.brake_enabled);
}
static float move(uint32_t ms, float angle, float observed_velocity)
{
    sample(ms, angle, observed_velocity, 1U);
    /* Isolate the damping rule from observer transients; real position increments still gate it. */
    pitch.cascade.velocity_dps = observed_velocity;
    return PitchController_Update(&pitch, 0.0f, ms, 1U);
}
void test_run(void)
{
    uint32_t ms;
    float torque, previous_angle, pi, vi;
    union { uint32_t bits; float value; } invalid;
    test_stage = 1U;
    CHECK(!PitchController_SetBrakeEnabled(0, 1U));
    PitchController_Init(&pitch);
    CHECK(!PitchController_SetBrakeEnabled(&pitch, 2U));
    CHECK(pitch.brake_enabled && APP_PITCH_POSITION_KP == 13.55f);
    CHECK(pitch.cascade.position.ki == 0.001f && pitch.cascade.position.kd == 0.026f);
    CHECK(pitch.cascade.speed.kp == 0.0062f && pitch.cascade.speed.ki == 0.1019f);
    CHECK(pitch.cascade.speed.kd == 0.000050f && pitch.gravity_kg_nm == -0.48f);
    brake_ready(100U, 0.0f, 30.0f);
    CHECK(near(move(134U, 25.0f, 100.0f), 0.0f) && !pitch.brake_arrived);
    CHECK(near(move(136U, 24.9f, -20.0f), 0.0f)); /* retreat before first arrival: no brake */
    CHECK(near(move(138U, 29.9f, 20.0f), 0.0f) && pitch.brake_arrived);
    CHECK(PitchController_SetTarget(&pitch, 30.0f) && pitch.brake_arrived); /* identical target */
    torque = move(140U, 29.7f, -20.0f);
    CHECK(near(torque, 0.04f) && pitch.brake_torque_nm > 0.0f);
    CHECK(near(pitch.pid_torque_nm, 0.0f) && near(pitch.cascade.feedforward_nm, torque));
    CHECK(near(PitchController_Update(&pitch, 0.0f, 141U, 1U), torque)); /* no duplicate ramp */
    previous_angle = 29.7f;
    for (ms = 142U; ms <= 170U; ms += 2U) {
        torque = move(ms, previous_angle - 0.05f, -100.0f);
        CHECK(torque > 0.0f && torque <= APP_PITCH_RETURN_BRAKE_LIMIT_NM);
        CHECK(torque * (pitch.angle_deg - previous_angle) <= 0.0f); /* removes energy from retreat */
        previous_angle = pitch.angle_deg;
    }
    CHECK(near(torque, 0.30f));
    CHECK(near(move(172U, previous_angle + 0.01f, -20.0f), 0.0f)); /* stale filtered sign */
    CHECK(near(move(174U, previous_angle + 0.01f, -20.0f), 0.0f)); /* stationary */
    CHECK(near(move(176U, 30.2f, 20.0f), 0.0f));
    CHECK(near(move(178U, 30.1f, -20.0f), 0.0f)); /* returning from overshoot towards target */

    test_stage = 2U;
    brake_ready(1000U, 30.0f, 0.0f);
    CHECK(near(move(1034U, 0.1f, -20.0f), 0.0f) && pitch.brake_arrived);
    torque = move(1036U, 0.3f, 20.0f);
    CHECK(near(torque, -0.04f) && torque * 0.2f < 0.0f);
    CHECK(near(move(1038U, 0.29f, 20.0f), 0.0f)); /* returns towards zero */
    CHECK(PitchController_SetTarget(&pitch, -10.0f));
    CHECK(!pitch.brake_arrived && pitch.brake_torque_nm == 0.0f);
    (void)PitchController_Update(&pitch, 1.0f, 1040U, 1U);
    CHECK(!pitch.brake_tracking && !pitch.brake_arrived && pitch.brake_torque_nm == 0.0f);

    test_stage = 3U;
    brake_ready(2000U, 0.0f, 30.0f);
    (void)move(2034U, 29.9f, 20.0f); (void)move(2036U, 29.7f, -20.0f);
    CHECK(pitch.brake_torque_nm > 0.0f);
    for (ms = 2038U; ms <= 2048U; ms += 2U) { (void)PitchController_Update(&pitch, 0.0f, ms, 1U); }
    CHECK(pitch.brake_torque_nm == 0.0f && !pitch.faults); /* stale auxiliary output removed */
    (void)move(2050U, 29.6f, -20.0f);
    pi = pitch.cascade.position_i = 1.0f; vi = pitch.cascade.speed_i = 0.1f;
    CHECK(PitchController_SetBrakeEnabled(&pitch, 0U));
    CHECK(!pitch.brake_tracking && !pitch.brake_arrived && pitch.brake_torque_nm == 0.0f);
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    CHECK(PitchController_SetBrakeEnabled(&pitch, 1U));
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    CHECK(!pitch.brake_tracking); /* new move required after changing mode */

    test_stage = 4U;
    brake_ready(3000U, 0.0f, 30.0f);
    (void)move(3034U, 29.9f, 20.0f);
    pitch.cascade.position.kp = 100.0f; pitch.cascade.speed.kp = 1.0f;
    pitch.cascade.position.ki = 30.0f; pitch.cascade.speed.ki = 10.0f;
    CHECK(move(3036U, 29.7f, -20.0f) == 10.0f && pitch.cascade.torque_limited);
    CHECK(pitch.brake_torque_nm > 0.0f && pitch.cascade.speed_i == 0.0f);
    CHECK(PitchController_Update(&pitch, 0.0f, 3038U, 0U) == 0.0f);
    CHECK(!pitch.brake_tracking && !pitch.brake_arrived && pitch.brake_torque_nm == 0.0f);
    brake_ready(4000U, 0.0f, 30.0f);
    (void)move(4034U, 29.9f, 20.0f); (void)move(4036U, 29.7f, -20.0f);
    invalid.bits = 0x7FC00000U;
    CHECK(PitchController_Update(&pitch, invalid.value, 4038U, 1U) == 0.0f);
    CHECK(pitch.faults && !pitch.brake_arrived && pitch.brake_torque_nm == 0.0f);
    brake_ready(5000U, 0.0f, 30.0f);
    (void)move(5034U, 29.9f, 20.0f); (void)move(5036U, 29.7f, -20.0f);
    CHECK(PitchController_SetPid(&pitch, 'V', 0.01f, 0.01f, 0.0f));
    CHECK(!pitch.brake_tracking && pitch.brake_torque_nm == 0.0f);

    test_stage = 5U;
    /* Millisecond wrap must not change the damping direction or ramp interval. */
    brake_ready(0xFFFFFFD0UL, 0.0f, 30.0f);
    (void)move(0xFFFFFFF2UL, 29.9f, 20.0f);
    (void)move(0xFFFFFFF4UL, 29.7f, -20.0f);
    for (ms = 0xFFFFFFF6UL; ms != 4U; ms += 2U) { (void)PitchController_Update(&pitch, 0.0f, ms, 1U); }
    CHECK(!pitch.faults && pitch.brake_torque_nm == 0.0f);
    CHECK(move(4U, 29.5f, -20.0f) > 0.0f);
    CHECK(!pitch.faults);
    test_done = 1U;
}
