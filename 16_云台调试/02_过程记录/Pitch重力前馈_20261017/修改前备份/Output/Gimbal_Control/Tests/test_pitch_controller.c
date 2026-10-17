#include "pitch_controller.h"
#include <float.h>
#include <string.h>
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
#define DEG_TO_RAD (DM3507_POSITION_MAX_RAD / 180.0f)
static PitchController pitch;
static Dm3507Feedback feedback;
static float absolute(float x) { return x < 0.0f ? -x : x; }
static uint8_t near(float a, float b) { return (uint8_t)(absolute(a - b) < 0.0001f); }
static void sample(uint32_t ms, float degrees, float velocity, uint8_t status)
{
    memset(&feedback, 0, sizeof(feedback)); feedback.motor_id = 2U; feedback.received_ms = ms;
    feedback.position_rad = degrees * DEG_TO_RAD; feedback.velocity_rad_s = velocity * DEG_TO_RAD;
    feedback.status = status; feedback.temp_mos = 25U; feedback.temp_rotor = 26U;
    CHECK(PitchController_OnFeedback(&pitch, &feedback));
}
static void ready(uint32_t ms, float degrees)
{
    PitchController_Init(&pitch); sample(ms, degrees, 0.0f, 0U);
    sample(ms + 10U, degrees, 0.0f, 0U); sample(ms + 20U, degrees, 0.0f, 0U);
    CHECK(PitchController_CaptureReference(&pitch)); sample(ms + 30U, degrees, 0.0f, 1U);
}
void test_run(void)
{
    uint32_t ms, revision, sequence;
    float pi, vi, target;
    union { uint32_t bits; float value; } bad;
    test_stage = 1U;
    PitchController_Init(0); PitchController_Stop(0);
    CHECK(!PitchController_CaptureReference(0) && !PitchController_SetFeedbackDivisor(0, 1U));
    CHECK(!PitchController_SetTarget(0, 30.0f) && PitchController_Update(0, 0.0f, 0U, 1U) == 0.0f);
    PitchController_Init(&pitch); CHECK(!pitch.have_reference && pitch.feedback_divisor == 1U);
    CHECK(pitch.cascade.position.kp == 12.0f && pitch.cascade.speed.kp == 0.02f);
    sample(100U, 170.0f, 0.0f, 0U); CHECK(!pitch.have_reference && !pitch.faults);
    CHECK(!PitchController_CaptureReference(&pitch) && !PitchController_SetTarget(&pitch, 30.0f));
    sample(110U, 170.0f, 0.0f, 0U); CHECK(!PitchController_CaptureReference(&pitch));
    sample(120U, 170.0f, 0.0f, 0U); CHECK(PitchController_CaptureReference(&pitch));
    CHECK(pitch.have_reference && pitch.angle_deg == 0.0f && !pitch.active);
    test_stage = 2U;
    ready(200U, 0.0f); CHECK(PitchController_SetTarget(&pitch, 30.0f));
    CHECK(near(PitchController_Update(&pitch, 0.0f, 230U, 1U), 7.2f));
    CHECK(pitch.cascade.target_velocity_dps == 360.0f && pitch.active && !pitch.cascade.velocity_limited);
    pi = pitch.cascade.position_i; vi = pitch.cascade.speed_i;
    for (ms = 232U; ms <= 240U; ms += 2U) {
        CHECK(PitchController_Update(&pitch, 0.0f, ms, 1U) > 7.1f);
        CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    }
    sample(240U, 1.0f, 300.0f, 1U);
    CHECK(!pitch.faults && pitch.velocity_dps > 299.9f);
    (void)PitchController_Update(&pitch, 0.0f, 242U, 1U);
    CHECK(pitch.cascade.position_i > pi && pitch.cascade.speed_i > vi);
    sequence = pitch.last_sequence; feedback.received_ms = 240U;
    CHECK(!PitchController_OnFeedback(&pitch, &feedback) && pitch.last_sequence == sequence);
    feedback.received_ms = 239U; CHECK(!PitchController_OnFeedback(&pitch, &feedback));
    test_stage = 3U;
    ready(1000U, 0.0f); CHECK(PitchController_SetPid(&pitch, 'P', 0.0f, 10.0f, 0.0f));
    CHECK(PitchController_SetPid(&pitch, 'V', 0.001f, 0.0f, 0.0f));
    CHECK(PitchController_SetTarget(&pitch, 30.0f));
    for (ms = 1030U; ms <= 2030U; ms += 2U) {
        if (ms > 1030U && ms % 10U == 0U) { sample(ms, 0.0f, 0.0f, 1U); }
        (void)PitchController_Update(&pitch, 0.0f, ms, 1U);
    }
    CHECK(pitch.cascade.position_i > 299.0f && pitch.cascade.target_velocity_dps > 299.0f);
    CHECK(!pitch.cascade.velocity_limited && !pitch.faults && pitch.torque_nm > 0.299f);
    CHECK(PitchController_SetPid(&pitch, 'P', 100.0f, 30.0f, 0.0f));
    CHECK(PitchController_SetPid(&pitch, 'V', 1.0f, 10.0f, 0.0f));
    CHECK(PitchController_Update(&pitch, 0.0f, 2032U, 1U) == 10.0f);
    pi = pitch.cascade.position_i;
    for (ms = 2034U; ms <= 2200U; ms += 2U) {
        if (ms % 10U == 0U) { sample(ms, 0.0f, 0.0f, 1U); }
        CHECK(PitchController_Update(&pitch, 0.0f, ms, 1U) == 10.0f);
    }
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == 0.0f);
    CHECK(!PitchController_SetTarget(&pitch, -30.0f));
    CHECK(PitchController_SetTarget(&pitch, -18.0f));
    CHECK(PitchController_Update(&pitch, 0.0f, 2202U, 1U) == -10.0f);
    test_stage = 4U;
    revision = pitch.cascade.revision;
    CHECK(PitchController_SetPid(&pitch, 'V', 0.1f, 0.1f, 0.001f));
    CHECK(pitch.cascade.revision == revision + 1U);
    revision = pitch.cascade.revision; pitch.cascade.speed_i = 0.5f;
    CHECK(PitchController_SetPid(&pitch, 'V', 0.1f, 0.1f, 0.001f));
    CHECK(pitch.cascade.revision == revision && pitch.cascade.speed_i == 0.5f);
    CHECK(!PitchController_SetPid(&pitch, 'V', 1.01f, 0.0f, 0.0f));
    CHECK(!PitchController_SetPid(&pitch, 'P', 101.0f, 0.0f, 0.0f));
    CHECK(!PitchController_SetPid(&pitch, 'X', 1.0f, 0.0f, 0.0f));
    bad.bits = 0x7FC00000U; CHECK(!PitchController_SetPid(&pitch, 'P', bad.value, 0.0f, 0.0f));
    CHECK(!PitchController_SetTarget(&pitch, bad.value) && !PitchController_SetTarget(&pitch, 50.01f));
    CHECK(!PitchController_SetFeedbackDivisor(&pitch, 7U));
    CHECK(PitchController_Update(&pitch, 0.0f, 2204U, 0U) == 0.0f && !pitch.active);
    CHECK(!pitch.cascade.speed_i && !pitch.cascade.position_i && !pitch.faults);
    test_stage = 5U;
    PitchController_Init(&pitch); CHECK(PitchController_SetFeedbackDivisor(&pitch, 7U));
    CHECK(!PitchController_SetFeedbackDivisor(&pitch, 2U));
    sample(3000U, 170.0f, 0.0f, 0U); sample(3010U, 170.0f, 0.0f, 0U); sample(3020U, 170.0f, 0.0f, 0U);
    CHECK(PitchController_CaptureReference(&pitch));
    sample(3030U, -179.0f, 700.0f, 1U);
    CHECK(near(pitch.angle_deg, 11.0f / 7.0f) && near(pitch.velocity_dps, 100.0f));
    CHECK(!pitch.faults && !PitchController_SetFeedbackDivisor(&pitch, 1U));
    sample(3040U, -179.0f, 0.0f, 0U); CHECK(PitchController_SetFeedbackDivisor(&pitch, 1U));
    CHECK(!pitch.have_reference && pitch.angle_deg == 0.0f);
    test_stage = 6U;
    ready(4000U, 0.0f); CHECK(PitchController_SetTarget(&pitch, 33.0f));
    (void)PitchController_Update(&pitch, 0.0f, 4030U, 1U);
    sample(4040U, 34.01f, 0.0f, 1U);
    CHECK((pitch.faults & PITCH_CONTROLLER_FAULT_LIMIT) && !pitch.active && !pitch.torque_nm);
    ready(5000U, 0.0f); (void)PitchController_Update(&pitch, 0.0f, 5030U, 1U);
    CHECK(PitchController_Update(&pitch, 0.0f, 5041U, 1U) == 0.0f);
    CHECK(pitch.faults & PITCH_CONTROLLER_FAULT_TIMING);
    ready(6000U, 0.0f); CHECK(PitchController_Update(&pitch, 0.0f, 6130U, 1U) == 0.0f);
    CHECK(pitch.faults & PITCH_CONTROLLER_FAULT_FEEDBACK);
    sample(6131U, 10.0f, 0.0f, 0U); CHECK(!pitch.have_reference);
    sample(6141U, 10.0f, 0.0f, 0U); sample(6151U, 10.0f, 0.0f, 0U);
    CHECK(PitchController_CaptureReference(&pitch) && !pitch.faults);
    test_stage = 7U;
    ready(7000U, 0.0f); sample(7040U, 0.0f, 0.0f, 13U);
    CHECK(pitch.faults & PITCH_CONTROLLER_FAULT_STATUS);
    ready(8000U, 0.0f); feedback.received_ms = 8040U; feedback.temp_mos = 60U;
    CHECK(PitchController_OnFeedback(&pitch, &feedback)); CHECK(pitch.faults & PITCH_CONTROLLER_FAULT_TEMPERATURE);
    ready(9000U, 0.0f); feedback.received_ms = 9040U; feedback.fault_epoch = 1U;
    CHECK(PitchController_OnFeedback(&pitch, &feedback)); CHECK(pitch.faults & PITCH_CONTROLLER_FAULT_CAN);
    sample(9050U, 0.0f, 0.0f, 0U); sample(9060U, 0.0f, 0.0f, 0U); sample(9070U, 0.0f, 0.0f, 0U);
    CHECK(!PitchController_CaptureReference(&pitch));
    ready(10000U, 0.0f); feedback.received_ms = 10040U; feedback.position_rad = bad.value;
    CHECK(!PitchController_OnFeedback(&pitch, &feedback)); CHECK(pitch.faults & PITCH_CONTROLLER_FAULT_DATA);
    ready(11000U, 0.0f); pitch.cascade.position.kp = FLT_MAX;
    CHECK(PitchController_SetTarget(&pitch, 30.0f)); CHECK(PitchController_Update(&pitch, 0.0f, 11030U, 1U) == 0.0f);
    CHECK((pitch.faults & PITCH_CONTROLLER_FAULT_DATA) && !pitch.active);
    test_stage = 8U;
    ready(0xFFFFFFE0UL, 0.0f); CHECK(PitchController_SetTarget(&pitch, -18.0f));
    CHECK(PitchController_Update(&pitch, 0.0f, 0xFFFFFFFEUL, 1U) < -4.3f);
    sample(8U, -1.0f, -100.0f, 1U);
    CHECK(PitchController_Update(&pitch, 0.0f, 8U, 1U) < 0.0f && !pitch.faults);
    target = pitch.target_deg;
    CHECK(PitchController_Update(&pitch, 45.0f, 10U, 1U) < 0.0f && pitch.target_deg > target);
    test_stage = 9U;
    ready(12000U, 0.0f);
    CHECK(PitchController_SetTarget(&pitch, -18.0f));
    CHECK(!PitchController_SetTarget(&pitch, -18.001f) && pitch.target_deg == -18.0f);
    CHECK(PitchController_SetTarget(&pitch, 33.0f));
    CHECK(!PitchController_SetTarget(&pitch, 33.001f) && pitch.target_deg == 33.0f);
    (void)PitchController_Update(&pitch, 45.0f, 12030U, 1U);
    (void)PitchController_Update(&pitch, 45.0f, 12032U, 1U);
    CHECK(pitch.target_deg == 33.0f && !pitch.faults);
    CHECK(PitchController_SetTarget(&pitch, -18.0f));
    (void)PitchController_Update(&pitch, -45.0f, 12034U, 1U);
    CHECK(pitch.target_deg == -18.0f && !pitch.faults);
    sample(12040U, -18.5f, -10.0f, 1U); CHECK(!pitch.faults);
    sample(12050U, -19.01f, -10.0f, 1U);
    CHECK((pitch.faults & PITCH_CONTROLLER_FAULT_LIMIT) && !pitch.active);
    ready(13000U, 0.0f);
    sample(13040U, 33.5f, 10.0f, 1U); CHECK(!pitch.faults);
    sample(13050U, 34.01f, 10.0f, 1U);
    CHECK((pitch.faults & PITCH_CONTROLLER_FAULT_LIMIT) && pitch.target_deg == 33.0f);
    test_done = 1U;
}
