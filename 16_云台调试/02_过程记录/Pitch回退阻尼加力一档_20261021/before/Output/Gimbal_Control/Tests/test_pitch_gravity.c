/* Real controller/cascade with synthetic CAN feedback; no motor transport. */
#define test_run baseline_controller_test_run
#include "test_pitch_controller.c"
#undef test_run
#include <math.h>
static YawCascade plain, with_ff;
static void gravity_ready(uint32_t ms, float offset, float gain)
{
    PitchController_Init(&pitch);
    sample(ms, 0.0f, 0.0f, 0U); sample(ms + 10U, 0.0f, 0.0f, 0U); sample(ms + 20U, 0.0f, 0.0f, 0U);
    CHECK(PitchController_CaptureReference(&pitch));
    CHECK(PitchController_SetGravityOffset(&pitch, offset));
    CHECK(PitchController_SetGravityGain(&pitch, gain));
    CHECK(PitchController_SetPid(&pitch, 'P', 0.0f, 0.0f, 0.0f));
    CHECK(PitchController_SetPid(&pitch, 'V', 0.0f, 0.0f, 0.0f));
    sample(ms + 30U, 0.0f, 0.0f, 1U);
}
void test_run(void)
{
    uint32_t ms;
    float previous, integral;
    union { uint32_t bits; float value; } bad;
    test_stage = 1U;
    PitchController_Init(&pitch);
    CHECK(pitch.gravity_kg_nm == -0.47f && !pitch.gravity_calibrated);
    CHECK(!PitchController_SetGravityGain(&pitch, -0.2f));
    CHECK(!PitchController_SetGravityOffset(&pitch, 0.0f));
    CHECK(PitchController_SetGravityGain(&pitch, 0.0f));
    CHECK(!PitchController_SetGravityGain(0, 0.1f) && !PitchController_CaptureHorizontal(0));
    PitchController_InvalidateGravity(0);
    gravity_ready(100U, 0.0f, -0.2f);
    CHECK(near(PitchController_Update(&pitch, 0.0f, 130U, 1U), -0.2f));
    CHECK(near(pitch.gravity_torque_nm, -0.2f) && near(pitch.pid_torque_nm, 0.0f));
    CHECK(!PitchController_SetGravityOffset(&pitch, 0.0f));
    sample(140U, 30.0f, 0.0f, 1U);
    CHECK(near(PitchController_Update(&pitch, 0.0f, 140U, 1U), -0.17320508f));
    CHECK(PitchController_Update(&pitch, 0.0f, 142U, 0U) == 0.0f);
    CHECK(!pitch.gravity_torque_nm && !pitch.pid_torque_nm && pitch.gravity_kg_nm == -0.2f);
    gravity_ready(200U, 90.0f, 0.4f);
    CHECK(near(PitchController_Update(&pitch, 0.0f, 230U, 1U), 0.0f));
    gravity_ready(300U, 180.0f, 0.4f);
    CHECK(near(PitchController_Update(&pitch, 0.0f, 330U, 1U), -0.4f));

    test_stage = 2U;
    gravity_ready(1000U, 10.0f, -0.3f); PitchController_Stop(&pitch);
    sample(1040U, 5.0f, 0.0f, 0U); sample(1050U, 5.0f, 0.0f, 0U); sample(1070U, 5.0f, 0.0f, 0U);
    CHECK(PitchController_CaptureReference(&pitch));
    CHECK(near(pitch.gravity_offset_deg, 15.0f) && pitch.gravity_calibrated && pitch.gravity_kg_nm == -0.3f);
    CHECK(PitchController_CaptureHorizontal(&pitch)); CHECK(near(pitch.gravity_offset_deg, 0.0f));
    CHECK(!PitchController_SetGravityOffset(&pitch, 180.01f));
    CHECK(!PitchController_SetGravityGain(&pitch, -10.01f));
    bad.bits = 0x7FC00000U;
    CHECK(!PitchController_SetGravityGain(&pitch, bad.value) && !PitchController_SetGravityOffset(&pitch, bad.value));
    bad.bits = 0x7F800000U; CHECK(!PitchController_SetGravityGain(&pitch, bad.value));
    sample(1170U, 5.0f, 0.0f, 0U);
    CHECK(!pitch.have_reference && !pitch.gravity_calibrated && pitch.gravity_kg_nm == -0.3f);
    gravity_ready(2000U, -12.0f, -0.2f); sample(2040U, 0.0f, 0.0f, 0U);
    CHECK(PitchController_SetFeedbackDivisor(&pitch, 7U));
    CHECK(!pitch.gravity_calibrated && pitch.gravity_kg_nm == -0.2f && !pitch.have_reference);

    test_stage = 3U;
    gravity_ready(3000U, 0.0f, 0.0f);
    CHECK(PitchController_SetPid(&pitch, 'V', 0.005f, 0.005f, 0.0f));
    (void)PitchController_Update(&pitch, 0.0f, 3030U, 1U);
    pitch.cascade.speed_i = -0.2f;
    previous = PitchController_Update(&pitch, 0.0f, 3032U, 1U);
    CHECK(PitchController_SetGravityGain(&pitch, -0.2f));
    CHECK(pitch.gravity_i_transferred && near(pitch.cascade.speed_i, 0.0f));
    CHECK(near(PitchController_Update(&pitch, 0.0f, 3034U, 1U), previous));
    integral = pitch.cascade.speed_i;
    CHECK(PitchController_SetGravityGain(&pitch, -0.2f) && pitch.cascade.speed_i == integral);
    for (ms = 3036U; ms <= 3042U; ms += 2U) {
        CHECK(near(PitchController_Update(&pitch, 0.0f, ms, 1U), previous));
        CHECK(pitch.cascade.speed_i == integral);
    }
    CHECK(PitchController_SetPid(&pitch, 'V', 0.005f, 0.0f, 0.0f));
    CHECK(PitchController_SetGravityGain(&pitch, -0.1f));
    CHECK(!pitch.gravity_i_transferred && pitch.cascade.speed_i == 0.0f);
    CHECK(near(PitchController_Update(&pitch, 0.0f, 3044U, 1U), -0.1f));
    CHECK(PitchController_SetPid(&pitch, 'V', 0.005f, 0.005f, 0.0f));
    pitch.cascade.speed_i = 9.9f;
    CHECK(PitchController_SetGravityGain(&pitch, -10.0f));
    CHECK(!pitch.gravity_i_transferred && pitch.cascade.speed_i == 10.0f);
    bad.bits = 0x7FC00000U; pitch.gravity_offset_deg = bad.value;
    CHECK(PitchController_Update(&pitch, 0.0f, 3046U, 1U) == 0.0f);
    CHECK((pitch.faults & PITCH_CONTROLLER_FAULT_DATA) && !pitch.gravity_torque_nm && !pitch.torque_nm);

    test_stage = 4U;
    YawCascade_Init(&plain); plain.position.kp = 1.0f; plain.position.ki = 0.0f;
    plain.position.kd = 0.0f; plain.speed.kp = 0.2f; plain.speed.ki = 1.0f; plain.speed.kd = 0.0f;
    with_ff = plain;
    for (ms = 100U; ms < 150U; ms += 2U) {
        CHECK(YawCascade_Update(&plain, 2.0f, ms, ms, 10.0f) ==
              YawCascade_UpdateWithFeedforward(&with_ff, 2.0f, ms, ms, 10.0f, 0.0f));
        CHECK(plain.speed_i == with_ff.speed_i && plain.unsaturated_torque_nm == with_ff.unsaturated_torque_nm);
    }
    YawCascade_Stop(&with_ff);
    CHECK(YawCascade_UpdateWithFeedforward(&with_ff, 1.0f, 200U, 1U, 10.0f, 9.9f) == 10.0f);
    CHECK(YawCascade_UpdateWithFeedforward(&with_ff, 1.0f, 210U, 2U, 10.0f, 9.9f) == 10.0f);
    CHECK(with_ff.speed_i == 0.0f && with_ff.torque_limited);
    CHECK(YawCascade_UpdateWithFeedforward(&with_ff, -1.0f, 220U, 3U, 10.0f, -9.9f) == -10.0f);
    CHECK(with_ff.speed_i == 0.0f && with_ff.torque_limited);
    (void)YawCascade_UpdateWithFeedforward(&with_ff, 1.0f, 230U, 4U, 10.0f, -9.9f);
    CHECK(with_ff.speed_i > 0.0f && !with_ff.torque_limited);
    test_done = 1U;
}
