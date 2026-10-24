/* Actual controller/reference replay; no motor/load performance model is claimed. */
#define test_run original_pitch_controller_test_run
#include "test_pitch_controller.c"
#undef test_run

static uint8_t close_value(float a, float b, float tolerance)
{ return (uint8_t)(absolute(a - b) <= tolerance); }

static uint32_t shape_ready(uint32_t base, float start)
{
    uint32_t offset;
    ready(base, 0.0f);
    CHECK(PitchController_SetShape(&pitch, 200U, 0.2f)); /* Explicit legacy-profile fixture. */
    CHECK(PitchController_SetBrakeEnabled(&pitch, 0U));
    sample(base + 32U, start, 0.0f, 1U);
    CHECK(PitchController_SetTarget(&pitch, start));
    (void)PitchController_Update(&pitch, 0.0f, base + 32U, 1U);
    /* A reverse move starts at a genuinely settled observed 30-degree pose. */
    for (offset = 34U; offset <= 132U; offset += 2U) {
        sample(base + offset, start, 0.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, base + offset, 1U);
    }
    CHECK(pitch.active && pitch.have_reference && !pitch.faults);
    CHECK(absolute(pitch.velocity_dps) <= 6.0f && absolute(pitch.cascade.velocity_dps) <= 6.0f);
    return base + 132U;
}

static void curve_test(uint32_t base, float start, float target)
{
    uint32_t began = shape_ready(base, start), elapsed;
    float previous = start, direction = target > start ? 1.0f : -1.0f;
    float span = absolute(target - start), speed_i_at_start;
    CHECK(PitchController_SetShape(&pitch, 200U, 0.2f));
    pitch.cascade.position_i = 1.25f;
    pitch.cascade.speed_i = 0.4f;
    CHECK(PitchController_StartMove(&pitch, target, began));
    CHECK(pitch.target_deg == target && pitch.shape.active);
    CHECK(near(pitch.shape.position_deg, start) && pitch.shape.velocity_dps == 0.0f);
    CHECK(pitch.shape.acceleration_dps2 == 0.0f && pitch.shape.start_ms == began);
    CHECK(pitch.cascade.position_i == 1.25f && pitch.cascade.speed_i == 0.4f);
    speed_i_at_start = pitch.cascade.speed_i;
    for (elapsed = 2U; elapsed <= 200U; elapsed += 2U) {
        sample(began + elapsed, start, 0.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, began + elapsed, 1U);
        CHECK(!pitch.faults && pitch.target_deg == target);
        CHECK((pitch.shape.position_deg - previous) * direction >= -0.0001f);
        CHECK((pitch.shape.position_deg - start) * direction >= -0.0001f);
        CHECK((target - pitch.shape.position_deg) * direction >= -0.0001f);
        CHECK(pitch.shape.velocity_dps * direction >= -0.0001f);
        CHECK(absolute(pitch.shape.velocity_dps) <= span * 9.375f + 0.001f);
        if (elapsed < 200U) {
            CHECK(pitch.shape.active && pitch.cascade.position_i == 1.25f);
        }
        if (elapsed < 100U) { CHECK(pitch.shape.acceleration_dps2 * direction >= -0.001f); }
        if (elapsed > 100U && elapsed < 200U) { CHECK(pitch.shape.acceleration_dps2 * direction <= 0.001f); }
        if (elapsed == 100U) {
            CHECK(close_value(pitch.shape.position_deg, (start + target) * 0.5f, 0.0001f));
            CHECK(close_value(pitch.shape.velocity_dps, direction * span * 9.375f, 0.001f));
            CHECK(close_value(pitch.shape.acceleration_dps2, 0.0f, 0.01f));
            if (start == 0.0f) {
                CHECK(close_value(pitch.shape.velocity_dps, 281.25f, 0.001f));
                CHECK(pitch.cascade.speed_i > speed_i_at_start);
                CHECK(close_value(pitch.cascade.target_velocity_dps,
                    12.0f * pitch.shape.position_deg + 1.25f + 0.2f * pitch.shape.velocity_dps,
                    0.001f));
            }
        }
        previous = pitch.shape.position_deg;
    }
    CHECK(!pitch.shape.active && near(pitch.shape.position_deg, target));
    CHECK(pitch.shape.velocity_dps == 0.0f && pitch.shape.acceleration_dps2 == 0.0f);
    CHECK(pitch.cascade.position_i != 0.0f && pitch.cascade.speed_i != 0.0f);
    if (start == 0.0f) { CHECK(pitch.cascade.position_i >= 1.25f); }
    sample(began + 202U, start, 0.0f, 1U);
    (void)PitchController_Update(&pitch, 0.0f, began + 202U, 1U);
    CHECK(!pitch.shape.active && near(pitch.shape.position_deg, target));
    CHECK(pitch.shape.velocity_dps == 0.0f && pitch.shape.acceleration_dps2 == 0.0f);
}

static void cascade_reference_test(void)
{
    YawCascade direct, tracking, zero_ff, old_entry;
    YawCascadeFeedforward ff;
    YawCascadeReference reference;
    float direct_torque, tracking_torque, old_torque;
    uint32_t tick;
    YawCascade_Init(&direct);
    direct.position.kp = 2.0f; direct.position.ki = 0.0f; direct.position.kd = 0.5f;
    direct.speed.kp = 0.1f; direct.speed.ki = 0.0f; direct.speed.kd = 0.0f;
    direct.velocity_dps = 3.0f; direct.acceleration_dps2 = 0.0f;
    tracking = direct;
    ff.torque_nm = 0.2f; ff.limited = 0U;
    reference.velocity_dps = 10.0f; reference.freeze_position_i = 1U;
    direct_torque = YawCascade_UpdateReference(&direct, 4.0f, 100U, 1U, 10.0f, 0.2f, 10.0f, 1U);
    tracking_torque = YawCascade_UpdateTracking(&tracking, 4.0f, 100U, 1U, 10.0f, &ff, &reference);
    CHECK(near(direct.target_velocity_dps, 16.5f));
    CHECK(near(tracking.target_velocity_dps, 21.5f));
    CHECK(near(direct_torque, 1.55f) && near(tracking_torque, 2.05f));
    CHECK(near(tracking.target_velocity_dps - direct.target_velocity_dps, 5.0f));

    YawCascade_Init(&zero_ff);
    zero_ff.position.kp = 2.0f; zero_ff.position.ki = 0.3f; zero_ff.position.kd = 0.5f;
    zero_ff.speed.kp = 0.01f; zero_ff.speed.ki = 0.04f; zero_ff.speed.kd = 0.0f;
    zero_ff.velocity_dps = 1.0f;
    old_entry = zero_ff;
    for (tick = 100U; tick <= 120U; tick += 2U) {
        direct_torque = YawCascade_UpdateReference(&zero_ff, 4.0f, tick, tick, 10.0f, 0.2f, 0.0f, 0U);
        old_torque = YawCascade_UpdateWithFeedforward(&old_entry, 4.0f, tick, tick, 10.0f, 0.2f);
        CHECK(near(direct_torque, old_torque));
        CHECK(near(zero_ff.position_i, old_entry.position_i) && near(zero_ff.speed_i, old_entry.speed_i));
        CHECK(near(zero_ff.target_velocity_dps, old_entry.target_velocity_dps));
    }
}

void test_run(void)
{
    uint32_t now, began, elapsed;
    float pi, vi, torque, old_torque;
    PitchController immediate;
    union { uint32_t bits; float value; } invalid;
    test_stage = 1U;
    PitchController_Init(&pitch);
    CHECK(pitch.shape.duration_ms == 0U && pitch.shape.velocity_gain == 0.0f && !pitch.shape.active);
    CHECK(pitch.cascade.position.kp == 14.4f && pitch.cascade.position.ki == 0.687f);
    CHECK(pitch.cascade.position.kd == 0.009f && pitch.cascade.speed.kp == 0.0063f);
    CHECK(pitch.cascade.speed.ki == 0.0874f && pitch.cascade.speed.kd == 0.000008f);
    CHECK(pitch.gravity_kg_nm == -0.47f);
    CHECK(!PitchController_SetShape(0, 200U, 0.2f));
    CHECK(!PitchController_StartMove(0, 30.0f, 100U));
    CHECK(!PitchController_StartMove(&pitch, 30.0f, 100U));
    pitch.cascade.position_i = 0.25f; pitch.cascade.speed_i = -0.1f;
    CHECK(PitchController_SetShape(&pitch, 100U, 0.0f));
    CHECK(PitchController_SetShape(&pitch, 250U, 1.0f));
    CHECK(PitchController_SetShape(&pitch, 0U, 0.2f));
    CHECK(pitch.cascade.position_i == 0.25f && pitch.cascade.speed_i == -0.1f);
    CHECK(!PitchController_SetShape(&pitch, 99U, 0.2f));
    CHECK(!PitchController_SetShape(&pitch, 251U, 0.2f));
    CHECK(!PitchController_SetShape(&pitch, 0xFFFFFFFFUL, 0.2f));
    CHECK(!PitchController_SetShape(&pitch, 200U, -0.001f));
    CHECK(!PitchController_SetShape(&pitch, 200U, 1.001f));
    invalid.bits = 0x7FC00000U;
    CHECK(!PitchController_SetShape(&pitch, 200U, invalid.value));
    invalid.bits = 0x7F800000U;
    CHECK(!PitchController_SetShape(&pitch, 200U, invalid.value));

    test_stage = 2U; curve_test(1000U, 0.0f, 30.0f);
    test_stage = 3U; curve_test(2000U, 30.0f, 0.0f);
    test_stage = 4U; curve_test(0xFFFFFF00UL, 0.0f, 30.0f);

    test_stage = 5U;
    now = shape_ready(3000U, 0.0f);
    CHECK(PitchController_SetShape(&pitch, 200U, 0.2f));
    pitch.cascade.position_i = 0.25f; pitch.cascade.speed_i = 0.1f;
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    began = pitch.shape.start_ms; pi = pitch.cascade.position_i; vi = pitch.cascade.speed_i;
    CHECK(PitchController_StartMove(&pitch, 30.0f, now + 2U));
    CHECK(pitch.shape.start_ms == began && pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    CHECK(PitchController_SetShape(&pitch, 200U, 0.2f));
    CHECK(!PitchController_SetShape(&pitch, 250U, 0.2f));
    CHECK(!PitchController_SetShape(&pitch, 200U, 0.0f));
    CHECK(!PitchController_StartMove(&pitch, 0.0f, now + 2U));
    CHECK(pitch.target_deg == 30.0f && pitch.shape.start_ms == began);
    CHECK(pitch.shape.duration_ms == 200U && pitch.shape.velocity_gain == 0.2f);
    CHECK(PitchController_SetTarget(&pitch, 30.0f));
    CHECK(!pitch.shape.active && pitch.shape.position_deg == 30.0f);
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    CHECK(PitchController_SetTarget(&pitch, 10.0f));
    CHECK(PitchController_StartMove(&pitch, 30.0f, now + 2U));
    CHECK(pitch.shape.active);
    CHECK(PitchController_SetTarget(&pitch, 10.0f));
    CHECK(!pitch.shape.active && pitch.target_deg == 10.0f);
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);

    test_stage = 6U;
    now = shape_ready(4000U, 0.0f);
    pitch.active = 0U; CHECK(!PitchController_StartMove(&pitch, 30.0f, now)); pitch.active = 1U;
    pitch.have_reference = 0U; CHECK(!PitchController_StartMove(&pitch, 30.0f, now)); pitch.have_reference = 1U;
    pitch.faults = PITCH_CONTROLLER_FAULT_DATA;
    CHECK(!PitchController_StartMove(&pitch, 30.0f, now)); pitch.faults = 0U;
    CHECK(!PitchController_StartMove(&pitch, 33.01f, now));
    CHECK(!PitchController_StartMove(&pitch, -18.01f, now));
    invalid.bits = 0x7FC00000U; CHECK(!PitchController_StartMove(&pitch, invalid.value, now));
    sample(now + 2U, 0.0f, 7.0f, 1U);
    CHECK(!PitchController_StartMove(&pitch, 30.0f, now + 2U));
    sample(now + 4U, 0.0f, 0.0f, 1U); pitch.cascade.velocity_dps = 7.0f;
    CHECK(!PitchController_StartMove(&pitch, 30.0f, now + 4U));
    pitch.cascade.velocity_dps = 0.0f;
    CHECK(PitchController_StartMove(&pitch, 30.0f, now + 4U));
    now = shape_ready(4500U, 0.0f);
    CHECK(!PitchController_StartMove(&pitch, 30.0f, now + 100U));

    test_stage = 7U;
    now = shape_ready(5000U, 0.0f);
    CHECK(PitchController_SetShape(&pitch, 200U, 0.0f));
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    for (elapsed = 2U; elapsed <= 50U; elapsed += 2U) {
        sample(now + elapsed, 0.0f, 0.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, now + elapsed, 1U);
    }
    CHECK(pitch.shape.active && close_value(pitch.shape.position_deg, 3.10546875f, 0.0001f));
    CHECK(close_value(pitch.cascade.target_velocity_dps,
        12.0f * pitch.shape.position_deg + pitch.cascade.position_i, 0.001f));
    now = shape_ready(5500U, 0.0f);
    CHECK(PitchController_SetShape(&pitch, 0U, 0.2f));
    immediate = pitch;
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    CHECK(PitchController_SetTarget(&immediate, 30.0f));
    CHECK(!pitch.shape.active && pitch.target_deg == 30.0f);
    for (elapsed = 2U; elapsed <= 40U; elapsed += 2U) {
        sample(now + elapsed, 0.0f, 0.0f, 1U);
        CHECK(PitchController_OnFeedback(&immediate, &feedback));
        torque = PitchController_Update(&pitch, 0.0f, now + elapsed, 1U);
        old_torque = PitchController_Update(&immediate, 0.0f, now + elapsed, 1U);
        CHECK(near(torque, old_torque));
        CHECK(near(pitch.cascade.position_i, immediate.cascade.position_i));
        CHECK(near(pitch.cascade.speed_i, immediate.cascade.speed_i));
    }

    test_stage = 8U;
    now = shape_ready(6000U, 0.0f);
    CHECK(PitchController_StartMove(&pitch, 30.0f, now)); PitchController_Stop(&pitch);
    CHECK(!pitch.shape.active && pitch.shape.velocity_dps == 0.0f && pitch.shape.acceleration_dps2 == 0.0f);
    now = shape_ready(6500U, 0.0f);
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    CHECK(PitchController_Update(&pitch, 0.0f, now + 2U, 0U) == 0.0f && !pitch.shape.active);
    now = shape_ready(7000U, 0.0f);
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    (void)PitchController_Update(&pitch, 45.0f, now + 2U, 1U);
    CHECK(!pitch.shape.active && pitch.shape.velocity_dps == 0.0f && pitch.shape.acceleration_dps2 == 0.0f);
    now = shape_ready(7500U, 0.0f);
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    CHECK(PitchController_Update(&pitch, 0.0f, now + 100U, 1U) == 0.0f);
    CHECK(pitch.faults && !pitch.shape.active);
    now = shape_ready(8000U, 0.0f);
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    feedback.received_ms = now + 2U; invalid.bits = 0x7FC00000U; feedback.position_rad = invalid.value;
    CHECK(!PitchController_OnFeedback(&pitch, &feedback));
    CHECK(pitch.faults && !pitch.shape.active);

    test_stage = 9U; cascade_reference_test();
    test_done = 1U;
}
