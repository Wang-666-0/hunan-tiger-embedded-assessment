/* Measurement/control replay only; no mechanical settling time is predicted. */
#define test_run original_pitch_controller_test_run
#include "test_pitch_controller.c"
#undef test_run

static void feed_pair(PitchController *legacy, uint32_t ms, float angle, float speed)
{
    GimbalPid old = legacy->cascade;
    float previous_angle = legacy->angle_deg;
    uint32_t gap = ms - legacy->last_feedback_ms;
    sample(ms, angle, speed, 1U);
    CHECK(PitchController_OnFeedback(legacy, &feedback));
    /* Replay the former observer with the same accepted position and timestamp. */
    GimbalPid_Observe(&old, legacy->angle_deg - previous_angle, gap);
    legacy->cascade.velocity_dps = old.velocity_dps;
    legacy->cascade.acceleration_dps2 = old.acceleration_dps2;
    (void)PitchController_Update(&pitch, 0.0f, ms, 1U);
    (void)PitchController_Update(legacy, 0.0f, ms, 1U);
    CHECK(!pitch.faults && !legacy->faults);
}

static void stopping_replay(uint32_t base, uint8_t reverse)
{
    PitchController legacy;
    uint32_t now = base + 32U, i;
    float start = reverse ? 30.0f : 0.0f, sign = reverse ? -1.0f : 1.0f;
    float travelled = 0.0f, fast_i, slow_i;
    ready(base, 0.0f);
    CHECK(PitchController_SetPid(&pitch, 'P', APP_PITCH_POSITION_KP, APP_PITCH_POSITION_KI, APP_PITCH_POSITION_KD));
    CHECK(PitchController_SetPid(&pitch, 'V', APP_PITCH_SPEED_KP, APP_PITCH_SPEED_KI, APP_PITCH_SPEED_KD));
    CHECK(PitchController_SetBrakeEnabled(&pitch, 0U)); /* Isolate measurement/PID changes. */
    sample(now, start, 0.0f, 1U);
    GimbalPid_ResetObserver(&pitch.cascade);
    CHECK(PitchController_SetTarget(&pitch, start));
    (void)PitchController_Update(&pitch, 0.0f, now, 1U);
    legacy = pitch;
    CHECK(PitchController_StartMove(&pitch, reverse ? 0.0f : 30.0f, now));
    CHECK(PitchController_StartMove(&legacy, reverse ? 0.0f : 30.0f, now));
    CHECK(near(PitchController_Update(&pitch, 0.0f, now, 1U),
               PitchController_Update(&legacy, 0.0f, now, 1U)));
    for (i = 1U; i <= 71U; ++i) {
        travelled = 0.4f * (float)i;
        feed_pair(&legacy, now + 2U * i, start + sign * travelled, sign * 200.0f);
    }
    now += 142U;
    /* Identical 16ms deceleration, from 200 to 0 deg/s, ending at 30 degrees. */
    for (i = 1U; i <= 8U; ++i) {
        travelled += (425.0f - 50.0f * (float)i) * 0.001f;
        feed_pair(&legacy, now + 2U * i, start + sign * travelled,
                  sign * (200.0f - 25.0f * (float)i));
    }
    now += 16U;
    CHECK(absolute(pitch.velocity_dps) < 0.0001f);
    /* Position differences measure interval-average speed, not endpoint speed. */
    CHECK(pitch.cascade.velocity_dps * sign < 40.0f);
    CHECK(legacy.cascade.velocity_dps * sign > 80.0f);
    fast_i = pitch.cascade.speed_i; slow_i = legacy.cascade.speed_i;
    for (i = 1U; i <= 5U; ++i) {
        feed_pair(&legacy, now + 2U * i, reverse ? 0.0f : 30.0f, 0.0f);
    }
    /* Old motion decays promptly; stale negative speed error does less I unwinding. */
    CHECK(absolute(pitch.cascade.velocity_dps) < 1.3f);
    CHECK(legacy.cascade.velocity_dps * sign > 25.0f);
    CHECK((pitch.torque_nm - legacy.torque_nm) * sign > 0.1f);
    /* Compare the saved speed-error area (>0.1 degree), independent of Ki scale. */
    CHECK(((pitch.cascade.speed_i - fast_i) - (legacy.cascade.speed_i - slow_i)) * sign >
          0.1f * APP_PITCH_SPEED_KI);
    CHECK(pitch.brake.torque_nm == 0.0f && legacy.brake.torque_nm == 0.0f);
}

void test_run(void)
{
    uint32_t i, sequence;
    float velocity, acceleration, pi, vi;
    GimbalPid yaw;
    test_stage = 1U;
    CHECK(2U == 2U && 20U == 20U);
    CHECK(YAW_CASCADE_VELOCITY_FILTER_MS == 10U && 20U == 20U);
    ready(1000U, 0.0f);
    sample(1032U, 0.04f, 20.0f, 1U);
    CHECK(absolute(pitch.cascade.velocity_dps - 10.0f) < 0.001f);
    CHECK(absolute(pitch.cascade.acceleration_dps2 - 454.54545f) < 0.1f);
    GimbalPid_Init(&yaw); GimbalPid_Observe(&yaw, 0.04f, 2U);
    CHECK(absolute(yaw.velocity_dps - 3.333333f) < 0.001f);
    CHECK(absolute(yaw.acceleration_dps2 - 151.51515f) < 0.1f);
    CHECK(PitchController_SetTarget(&pitch, 0.0f));
    (void)PitchController_Update(&pitch, 0.0f, 1032U, 1U);
    velocity = pitch.cascade.velocity_dps; acceleration = pitch.cascade.acceleration_dps2;
    pi = pitch.cascade.position_i; vi = pitch.cascade.speed_i; sequence = pitch.last_sequence;
    CHECK(!PitchController_OnFeedback(&pitch, &feedback));
    feedback.received_ms = 1031U;
    CHECK(!PitchController_OnFeedback(&pitch, &feedback));
    for (i = 1U; i <= 4U; ++i) {
        (void)PitchController_Update(&pitch, 0.0f, 1032U + 2U * i, 1U);
        CHECK(pitch.cascade.velocity_dps == velocity && pitch.cascade.acceleration_dps2 == acceleration);
        CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi && pitch.last_sequence == sequence);
    }
    test_stage = 2U; stopping_replay(2000U, 0U);
    test_stage = 3U; stopping_replay(3000U, 1U);
    test_stage = 4U; stopping_replay(0xFFFFFF80UL, 0U);

    test_stage = 5U;
    ready(4000U, 0.0f);
    CHECK(PitchController_SetTarget(&pitch, 0.0f));
    (void)PitchController_Update(&pitch, 0.0f, 4030U, 1U);
    /* A quantized raw CAN speed must not bias a stationary position observer. */
    for (i = 1U; i <= 40U; ++i) {
        sample(4030U + 2U * i, 0.0f, i & 1U ? 2.79852f : -2.79852f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, 4030U + 2U * i, 1U);
        CHECK(pitch.cascade.velocity_dps == 0.0f && pitch.cascade.acceleration_dps2 == 0.0f);
        CHECK(pitch.cascade.speed_i == 0.0f && pitch.torque_nm == 0.0f);
    }

    test_stage = 6U;
    ready(5000U, 0.0f);
    CHECK(PitchController_SetPid(&pitch, 'P', APP_PITCH_POSITION_KP, APP_PITCH_POSITION_KI, APP_PITCH_POSITION_KD));
    CHECK(PitchController_SetPid(&pitch, 'V', APP_PITCH_SPEED_KP, APP_PITCH_SPEED_KI, APP_PITCH_SPEED_KD));
    CHECK(PitchController_SetTarget(&pitch, 0.0f));
    (void)PitchController_Update(&pitch, 0.0f, 5030U, 1U);
    /* One 16-bit encoder count alternates every millisecond: bound added ripple. */
    for (i = 1U; i <= 200U; ++i) {
        sample(5030U + i, i & 1U ? 360.0f / 65535.0f : 0.0f, i & 1U ? 2.79852f : -2.79852f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, 5030U + i, 1U);
        CHECK(!pitch.faults && absolute(pitch.cascade.velocity_dps) < 2.0f);
        CHECK(absolute(pitch.cascade.acceleration_dps2) < 100.0f && absolute(pitch.torque_nm) < 0.03f);
    }

    test_stage = 7U;
    PitchController_Init(&pitch); CHECK(PitchController_SetFeedbackDivisor(&pitch, 7U));
    sample(6000U, 0.0f, 0.0f, 0U); sample(6010U, 0.0f, 0.0f, 0U); sample(6020U, 0.0f, 0.0f, 0U);
    CHECK(PitchController_CaptureReference(&pitch));
    sample(6022U, 0.28f, 140.0f, 1U);
    CHECK(absolute(pitch.velocity_dps - 20.0f) < 0.001f);
    CHECK(absolute(pitch.cascade.velocity_dps - 10.0f) < 0.001f);
    CHECK(absolute(pitch.cascade.acceleration_dps2 - 454.54545f) < 0.1f);
    sample(6122U, 0.28f, 0.0f, 0U);
    CHECK(pitch.faults & PITCH_CONTROLLER_FAULT_FEEDBACK);
    CHECK(pitch.cascade.velocity_dps == 0.0f && pitch.cascade.acceleration_dps2 == 0.0f);
    CHECK(!pitch.have_reference && !pitch.active && pitch.torque_nm == 0.0f);
    test_done = 1U;
}
