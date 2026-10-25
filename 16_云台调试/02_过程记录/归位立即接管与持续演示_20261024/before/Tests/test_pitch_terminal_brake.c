/* Replay real controller outputs; this is not an identified mechanical plant. */
#define test_run original_pitch_controller_test_run
#include "test_pitch_controller.c"
#undef test_run

static uint32_t prepare_move(uint32_t base, float start)
{
    ready(base, 0.0f);
    CHECK(PitchController_SetPid(&pitch, 'P', APP_PITCH_POSITION_KP, APP_PITCH_POSITION_KI, APP_PITCH_POSITION_KD));
    CHECK(PitchController_SetPid(&pitch, 'V', APP_PITCH_SPEED_KP, APP_PITCH_SPEED_KI, APP_PITCH_SPEED_KD));
    CHECK(PitchController_SetShape(&pitch, 0U, 0.0f));
    sample(base + 32U, start, 0.0f, 1U);
    YawCascade_ResetObserver(&pitch.cascade);
    CHECK(PitchController_SetTarget(&pitch, start));
    (void)PitchController_Update(&pitch, 0.0f, base + 32U, 1U);
    CHECK(pitch.active && !pitch.faults && pitch.brake.enabled);
    return base + 32U;
}

static uint32_t reach_braking(uint32_t base)
{
    uint32_t now = prepare_move(base, 0.0f), i;
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    for (i = 1U; i <= 72U; ++i) {
        sample(now + 2U * i, 0.4f * (float)i, 200.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, now + 2U * i, 1U);
    }
    CHECK(pitch.brake.active && pitch.brake.torque_nm < 0.0f && !pitch.faults);
    return now + 144U;
}

static void compare_direct_step(uint32_t base, uint8_t reverse)
{
    PitchController baseline;
    uint32_t now = prepare_move(base, reverse ? 30.0f : 0.0f), i;
    float target = reverse ? 0.0f : 30.0f, sign = reverse ? -1.0f : 1.0f;
    float torque, original, previous = 0.0f, before_pi, before_vi;
    uint8_t saw_brake = 0U;
    pitch.cascade.position_i = 0.25f; pitch.cascade.speed_i = 0.2f;
    baseline = pitch; CHECK(PitchController_SetBrakeEnabled(&baseline, 0U));
    CHECK(PitchController_StartMove(&pitch, target, now));
    CHECK(PitchController_StartMove(&baseline, target, now));
    CHECK(!pitch.shape.active && pitch.target_deg == target && pitch.brake.tracking);
    CHECK(pitch.cascade.position_i == 0.25f && pitch.cascade.speed_i == 0.2f);
    torque = PitchController_Update(&pitch, 0.0f, now, 1U);
    original = PitchController_Update(&baseline, 0.0f, now, 1U);
    CHECK(near(torque, original)); /* First command is exactly the direct PID step. */
    CHECK(near(pitch.cascade.target_velocity_dps, sign * 30.0f * APP_PITCH_POSITION_KP + 0.25f));
    for (i = 1U; i <= 76U; ++i) {
        float travelled = 0.4f * (float)i;
        float angle = reverse ? 30.0f - travelled : travelled;
        sample(now + 2U * i, angle, sign * 200.0f, 1U);
        CHECK(PitchController_OnFeedback(&baseline, &feedback));
        torque = PitchController_Update(&pitch, 0.0f, now + 2U * i, 1U);
        original = PitchController_Update(&baseline, 0.0f, now + 2U * i, 1U);
        CHECK(!pitch.faults && pitch.target_deg == target && !pitch.shape.active);
        CHECK(near(pitch.shape.position_deg, target));
        CHECK(near(pitch.cascade.target_velocity_dps, baseline.cascade.target_velocity_dps));
        CHECK(near(pitch.cascade.position_i, baseline.cascade.position_i));
        CHECK(near(pitch.cascade.speed_i, baseline.cascade.speed_i));
        CHECK(absolute(pitch.brake.torque_nm) <= 0.120001f);
        CHECK(pitch.brake.torque_nm * pitch.velocity_dps <= 0.0f);
        CHECK(pitch.brake.torque_nm * pitch.cascade.velocity_dps <= 0.0f);
        CHECK(absolute(pitch.brake.torque_nm) <= previous + 0.012001f);
        CHECK(near(torque - original, pitch.brake.torque_nm));
        if (travelled <= 25.9f) {
            CHECK(pitch.brake.torque_nm == 0.0f && near(torque, original));
        }
        if (pitch.brake.active) { saw_brake = 1U; CHECK(pitch.brake.torque_nm * sign < 0.0f); }
        previous = absolute(pitch.brake.torque_nm);
    }
    CHECK(saw_brake && pitch.brake.arrived);
    now += 154U;
    sample(now, reverse ? -0.4f : 30.4f, 0.0f, 1U);
    CHECK(PitchController_OnFeedback(&baseline, &feedback));
    torque = PitchController_Update(&pitch, 0.0f, now, 1U);
    original = PitchController_Update(&baseline, 0.0f, now, 1U);
    CHECK(!pitch.brake.active && pitch.brake.tracking && pitch.brake.approach_done && pitch.brake.torque_nm == 0.0f);
    CHECK(near(torque, original)); /* No residual brake at standstill despite filtered speed. */
    sample(now + 2U, reverse ? 0.0f : 30.0f, -sign * 200.0f, 1U);
    CHECK(PitchController_OnFeedback(&baseline, &feedback));
    torque = PitchController_Update(&pitch, 0.0f, now + 2U, 1U);
    original = PitchController_Update(&baseline, 0.0f, now + 2U, 1U);
    CHECK(pitch.brake.torque_nm == 0.0f && near(torque, original));
    before_pi = pitch.cascade.position_i; before_vi = pitch.cascade.speed_i;
    CHECK(PitchController_StartMove(&pitch, target, now + 2U));
    CHECK(pitch.brake.tracking && pitch.cascade.position_i == before_pi && pitch.cascade.speed_i == before_vi);
    for (i = 4U; i <= 260U; i += 2U) {
        sample(now + i, target, 0.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, now + i, 1U);
    }
    CHECK(!pitch.brake.tracking && pitch.brake.peak_abs_nm > 0.0f);
    CHECK(pitch.brake.active_frames > 0U && pitch.brake.return_frames == 0U);
}

static void capture_rebound(uint32_t base, uint8_t reverse)
{
    uint32_t now = prepare_move(base, reverse ? 30.0f : 0.0f), i, arrival;
    float sign = reverse ? -1.0f : 1.0f, target = reverse ? 0.0f : 30.0f;
    float previous, peak, pi, vi;
    uint32_t frames;
    CHECK(PitchController_StartMove(&pitch, target, now));
    CHECK(pitch.landing.active && pitch.landing.start_ms == now && !pitch.landing.landed);
    CHECK(pitch.landing.start_i_nm == pitch.cascade.speed_i);
    for (i = 1U; i <= 75U; ++i) {
        float travelled = 0.4f * (float)i;
        sample(now + i * 2U, reverse ? 30.0f - travelled : travelled, sign * 200.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, now + i * 2U, 1U);
    }
    arrival = pitch.brake.arrival_ms;
    CHECK(pitch.brake.arrived && pitch.brake.tracking);
    now += 152U; sample(now, target, 0.0f, 1U);
    (void)PitchController_Update(&pitch, 0.0f, now, 1U);
    CHECK(pitch.brake.approach_done && pitch.brake.torque_nm == 0.0f);
    CHECK(pitch.landing.landed && pitch.landing.land_ms == 152U);
    CHECK(pitch.landing.land_raw_dps == 0.0f && pitch.landing.land_filtered_dps * sign > 90.0f &&
          pitch.landing.land_filtered_dps * sign < 110.0f);
    CHECK(pitch.landing.land_i_nm == pitch.cascade.speed_i && pitch.landing.land_torque_nm == pitch.torque_nm);
    CHECK(pitch.landing.land_target_dps == pitch.cascade.target_velocity_dps);
    CHECK(pitch.landing.land_gravity_nm == pitch.gravity_torque_nm && pitch.landing.land_brake_nm == pitch.brake.torque_nm);
    previous = 0.0f;
    for (i = 1U; i <= 25U; ++i) {
        sample(now + i * 2U, target - sign * 0.12f * (float)i, -sign * 60.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, now + i * 2U, 1U);
        CHECK(!pitch.faults && pitch.brake.tracking && pitch.brake.arrival_ms == arrival);
        CHECK(pitch.brake.torque_nm * sign >= 0.0f);
        CHECK(pitch.brake.torque_nm * pitch.velocity_dps <= 0.0f);
        CHECK(pitch.brake.torque_nm * pitch.cascade.velocity_dps <= 0.0f);
        CHECK(absolute(pitch.brake.torque_nm) <= 0.300001f);
        CHECK(absolute(pitch.brake.torque_nm) <= previous + 0.024001f);
        previous = absolute(pitch.brake.torque_nm);
    }
    CHECK(pitch.brake.return_frames > 0U && pitch.brake.return_peak_nm > 0.12f);
    CHECK(pitch.brake.return_peak_nm > 0.20f);
    CHECK(pitch.landing.dip_deg >= 2.999f && pitch.landing.dip_ms >= 200U);
    CHECK(pitch.landing.min_i_nm <= pitch.landing.land_i_nm && pitch.landing.max_i_nm >= pitch.landing.land_i_nm);
    peak = pitch.brake.return_peak_nm; frames = pitch.brake.return_frames;
    pi = pitch.cascade.position_i; vi = pitch.cascade.speed_i;
    (void)PitchController_Update(&pitch, 0.0f, now + 50U, 1U);
    CHECK(pitch.brake.return_frames == frames && pitch.brake.return_peak_nm == peak);
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    now += 52U;
    sample(now, target - sign * 2.9f, -sign * 60.0f, 1U);
    (void)PitchController_Update(&pitch, 0.0f, now, 1U);
    CHECK(pitch.brake.torque_nm == 0.0f); /* Position increment vetoes stale reverse velocity. */
    /* Do not damp the corrective motion back towards the goal, even with observer lag. */
    for (i = 1U; i <= 25U; ++i) {
        sample(now + i * 2U, target - sign * (3.0f - 0.12f * (float)i), sign * 60.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, now + i * 2U, 1U);
        CHECK(pitch.brake.torque_nm == 0.0f && !pitch.brake.active);
        CHECK(pitch.brake.return_frames == frames && pitch.brake.return_peak_nm == peak);
    }
    now += 50U;
    for (i = 2U; i <= 400U; i += 2U) {
        sample(now + i, target, 0.0f, 1U);
        (void)PitchController_Update(&pitch, 0.0f, now + i, 1U);
    }
    CHECK(!pitch.brake.tracking && pitch.brake.torque_nm == 0.0f);
    CHECK(pitch.brake.return_peak_nm == peak && pitch.brake.return_frames == frames);
    CHECK(!pitch.landing.active && pitch.landing.landed && pitch.landing.land_ms == 152U);
    PitchController_Stop(&pitch);
    CHECK(pitch.brake.return_peak_nm == peak && pitch.brake.return_frames == frames);
    CHECK(pitch.landing.landed && pitch.landing.land_ms == 152U);
    sample(now + 402U, target, 0.0f, 1U);
    (void)PitchController_Update(&pitch, 0.0f, now + 402U, 1U);
    CHECK(PitchController_StartMove(&pitch, reverse ? 30.0f : 0.0f, now + 402U));
    CHECK(!pitch.landing.landed && pitch.landing.dip_deg == 0.0f && pitch.landing.active);
    CHECK(pitch.brake.peak_abs_nm == 0.0f && pitch.brake.return_peak_nm == 0.0f);
    CHECK(pitch.brake.active_frames == 0U && pitch.brake.return_frames == 0U);
}

void test_run(void)
{
    uint32_t now, i;
    float pi, vi, held;
    union { uint32_t bits; float value; } invalid;
    test_stage = 1U;
    PitchController_Init(&pitch);
    CHECK(pitch.shape.duration_ms == 0U && pitch.shape.velocity_gain == 0.0f);
    CHECK(pitch.brake.enabled && !pitch.brake.active && !pitch.brake.tracking);
    CHECK(pitch.cascade.position.kp == APP_PITCH_POSITION_KP && pitch.cascade.position.ki == APP_PITCH_POSITION_KI);
    CHECK(pitch.cascade.speed.kp == APP_PITCH_SPEED_KP && pitch.gravity_kg_nm == APP_PITCH_GRAVITY_KG_NM);
    CHECK(!PitchController_SetBrakeEnabled(0, 1U));
    CHECK(!PitchController_SetBrakeEnabled(&pitch, 2U));
    test_stage = 2U; compare_direct_step(1000U, 0U);
    test_stage = 3U; compare_direct_step(2000U, 1U);
    test_stage = 4U; compare_direct_step(0xFFFFFF80UL, 0U);

    test_stage = 5U;
    now = reach_braking(3000U); held = pitch.brake.torque_nm;
    pi = pitch.cascade.position_i; vi = pitch.cascade.speed_i;
    (void)PitchController_Update(&pitch, 0.0f, now, 1U);
    CHECK(pitch.brake.torque_nm == held && pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    CHECK(PitchController_SetBrakeEnabled(&pitch, 1U) && pitch.brake.torque_nm == held);
    CHECK(PitchController_StartMove(&pitch, 30.0f, now) && pitch.brake.torque_nm == held);
    CHECK(PitchController_SetBrakeEnabled(&pitch, 0U));
    CHECK(pitch.brake.torque_nm == 0.0f && !pitch.brake.tracking);
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);
    CHECK(PitchController_SetBrakeEnabled(&pitch, 1U) && !pitch.brake.tracking);

    test_stage = 6U;
    now = reach_braking(4000U);
    pi = pitch.cascade.position_i; vi = pitch.cascade.speed_i;
    for (i = 2U; i <= 12U; i += 2U) { (void)PitchController_Update(&pitch, 0.0f, now + i, 1U); }
    CHECK(!pitch.faults && pitch.brake.torque_nm == 0.0f && !pitch.brake.active);
    CHECK(pitch.cascade.position_i == pi && pitch.cascade.speed_i == vi);

    test_stage = 7U;
    now = reach_braking(5000U);
    sample(now + 2U, 28.7f, -50.0f, 1U);
    (void)PitchController_Update(&pitch, 0.0f, now + 2U, 1U);
    CHECK(pitch.brake.torque_nm == 0.0f); /* Retreat is never opposed by this approach brake. */
    now = reach_braking(6000U);
    sample(now + 2U, 28.9f, 0.0f, 1U);
    (void)PitchController_Update(&pitch, 0.0f, now + 2U, 1U);
    CHECK(pitch.brake.torque_nm == 0.0f); /* Raw velocity vetoes filtered lag. */
    now = prepare_move(6500U, 0.0f);
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    sample(now + 2U, 27.0f, 50.0f, 1U);
    (void)PitchController_Update(&pitch, 0.0f, now + 2U, 1U);
    CHECK(pitch.brake.torque_nm == 0.0f && pitch.brake.safe_velocity_dps > 50.0f);

    test_stage = 8U;
    now = reach_braking(7000U); PitchController_Stop(&pitch);
    CHECK(!pitch.brake.tracking && !pitch.brake.active && pitch.brake.torque_nm == 0.0f);
    now = reach_braking(8000U);
    CHECK(PitchController_Update(&pitch, 0.0f, now + 2U, 0U) == 0.0f);
    CHECK(!pitch.brake.tracking && pitch.brake.torque_nm == 0.0f);
    now = reach_braking(9000U);
    (void)PitchController_Update(&pitch, 45.0f, now + 2U, 1U);
    CHECK(!pitch.brake.tracking && pitch.brake.torque_nm == 0.0f);
    now = reach_braking(10000U);
    CHECK(PitchController_SetPid(&pitch, 'P', 14.5f, 0.672f, 0.009f));
    CHECK(!pitch.brake.tracking && pitch.brake.torque_nm == 0.0f);
    now = reach_braking(11000U);
    feedback.received_ms = now + 2U; invalid.bits = 0x7FC00000U; feedback.position_rad = invalid.value;
    CHECK(!PitchController_OnFeedback(&pitch, &feedback));
    CHECK(pitch.faults && !pitch.brake.tracking && pitch.brake.torque_nm == 0.0f);
    now = prepare_move(12000U, 0.0f);
    CHECK(PitchController_SetShape(&pitch, 200U, 0.2f));
    CHECK(PitchController_StartMove(&pitch, 30.0f, now));
    CHECK(pitch.shape.active && !pitch.brake.tracking);
    test_stage = 9U; capture_rebound(13000U, 0U);
    test_stage = 10U; capture_rebound(14000U, 1U);
    test_stage = 11U; capture_rebound(0xFFFFFF50UL, 0U);
    test_done = 1U;
}
