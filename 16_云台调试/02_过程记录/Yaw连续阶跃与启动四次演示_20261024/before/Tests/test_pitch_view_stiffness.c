/* Real controller math; no motor/plant-performance claim. */
#define test_run controller_baseline_test_run
#include "test_pitch_controller.c"
#undef test_run
static PitchController normal, held;
void test_run(void)
{
    YawCascadePid position, speed;
    float usual, strong;
    uint32_t revision, ms;
    test_stage = 1U;
    CHECK(APP_PITCH_VIEW_HOLD_POSITION_GAIN == 1.0f && APP_PITCH_VIEW_HOLD_SPEED_GAIN == 1.0f);
    CHECK(PitchController_UpdateViewHold(0, 0U, 1U) == 0.0f);
    ready(1000U, 0.0f);
    CHECK(PitchController_SetPid(&pitch, 'P', 10.0f, 0.0f, 0.0f));
    CHECK(PitchController_SetPid(&pitch, 'V', 0.01f, 0.0f, 0.0f));
    CHECK(PitchController_SetTarget(&pitch, 2.0f));
    normal = held = pitch; position = pitch.cascade.position; speed = pitch.cascade.speed;
    revision = pitch.cascade.revision;
    usual = PitchController_Update(&normal, 0.0f, 1030U, 1U);
    strong = PitchController_UpdateViewHold(&held, 1030U, 1U);
    CHECK(near(usual, 0.2f) && near(strong, 0.2f));
    CHECK(strong == usual);
    CHECK(!memcmp(&position, &held.cascade.position, sizeof(position)));
    CHECK(!memcmp(&speed, &held.cascade.speed, sizeof(speed)) && held.cascade.revision == revision);
    for (ms = 1032U; ms < 1040U; ms += 2U) {
        CHECK(near(PitchController_UpdateViewHold(&held, ms, 1U), 0.2f));
        CHECK(!memcmp(&position, &held.cascade.position, sizeof(position)));
        CHECK(!memcmp(&speed, &held.cascade.speed, sizeof(speed)));
    }
    /* Switching back uses the original stored gains on the very next update. */
    CHECK(near(PitchController_Update(&held, 0.0f, 1040U, 1U), 0.2f));
    test_stage = 2U;
    CHECK(PitchController_SetTarget(&pitch, 0.0f));
    pitch.cascade.velocity_dps = 5.0f;
    normal = held = pitch;
    usual = PitchController_Update(&normal, 0.0f, 1030U, 1U);
    strong = PitchController_UpdateViewHold(&held, 1030U, 1U);
    CHECK(near(usual, -0.05f) && strong == usual); /* No hidden damping/P gain. */
    pitch.cascade.velocity_dps = 0.0f;
    CHECK(PitchController_SetTarget(&pitch, -2.0f)); normal = held = pitch;
    usual = PitchController_Update(&normal, 0.0f, 1030U, 1U);
    strong = PitchController_UpdateViewHold(&held, 1030U, 1U);
    CHECK(near(usual, -0.2f) && strong == usual);
    /* Nonzero P/I/D and a fresh sample must evolve exactly as the original
     * loop, including integral and derivative state, not only the gains readout. */
    CHECK(PitchController_SetPid(&pitch, 'P', 10.0f, 2.0f, 0.02f));
    CHECK(PitchController_SetPid(&pitch, 'V', 0.01f, 0.004f, 0.000074f));
    normal = held = pitch;
    usual = PitchController_Update(&normal, 0.0f, 1030U, 1U);
    strong = PitchController_UpdateViewHold(&held, 1030U, 1U); CHECK(strong == usual);
    feedback.received_ms = 1040U; feedback.position_rad = 0.1f * DEG_TO_RAD;
    CHECK(PitchController_OnFeedback(&normal, &feedback)); CHECK(PitchController_OnFeedback(&held, &feedback));
    usual = PitchController_Update(&normal, 0.0f, 1040U, 1U);
    strong = PitchController_UpdateViewHold(&held, 1040U, 1U);
    CHECK(strong == usual && !memcmp(&normal.cascade, &held.cascade, sizeof(normal.cascade)));
    test_stage = 3U;
    CHECK(PitchController_SetPid(&pitch, 'P', 100.0f, 30.0f, 0.1f));
    CHECK(PitchController_SetPid(&pitch, 'V', 1.0f, 10.0f, 0.001f));
    CHECK(PitchController_SetTarget(&pitch, 33.0f)); held = pitch;
    position = held.cascade.position; speed = held.cascade.speed;
    CHECK(PitchController_UpdateViewHold(&held, 1030U, 1U) == PITCH_CONTROLLER_TORQUE_LIMIT_NM);
    CHECK(held.cascade.torque_limited);
    CHECK(PitchController_UpdateViewHold(&held, 1032U, 0U) == 0.0f && !held.active);
    CHECK(!held.cascade.position_i && !held.cascade.speed_i);
    CHECK(!memcmp(&position, &held.cascade.position, sizeof(position)));
    CHECK(!memcmp(&speed, &held.cascade.speed, sizeof(speed)));
    held = pitch;
    CHECK(PitchController_UpdateViewHold(&held, 1130U, 1U) == 0.0f);
    CHECK(held.faults & PITCH_CONTROLLER_FAULT_FEEDBACK);
    CHECK(!memcmp(&position, &held.cascade.position, sizeof(position)));
    CHECK(!memcmp(&speed, &held.cascade.speed, sizeof(speed)));
    test_done = 1U;
}
