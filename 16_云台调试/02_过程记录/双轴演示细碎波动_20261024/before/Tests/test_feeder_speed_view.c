#define PITCH_TELEMETRY_TEST_ENTRY pitch_view_baseline_test_run
#include "test_pitch_telemetry.c"
#include "feeder_console.h"
static uint32_t feeder_speed_frames, feeder_detail_frames;
uint8_t FeederConsole_Handle(const char *text, char *reply, size_t capacity)
{ (void)text; (void)reply; (void)capacity; return 0U; }
void FeederConsole_Telemetry(char *reply, size_t capacity)
{ ++feeder_detail_frames; (void)snprintf(reply, capacity, "feeder:1,2,3\r\n"); }
void FeederConsole_SpeedTelemetry(char *reply, size_t capacity)
{ ++feeder_speed_frames; (void)snprintf(reply, capacity, "feeder_speed:-3568,-1784,-100,-50,35.68,1,1,0,2\r\n"); }
static void frame(const char *group, unsigned count)
{
    const char *cursor;
    char *end;
    unsigned i;
    now += 5U; clear_output(); AppConsole_Poll();
    if (!group) {
        group = strncmp(output, "pitch_demo:", 11U) == 0 ? "pitch_demo:" : "pitch:";
        count = strcmp(group, "pitch_demo:") == 0 ? 68U : 65U;
    }
    CHECK(strncmp(output, group, strlen(group)) == 0);
    cursor = output + strlen(group);
    for (i = 0U; i < count; ++i) {
        columns[i] = strtof(cursor, &end); CHECK(end != cursor);
        if (i + 1U < count) { CHECK(*end == ','); cursor = end + 1; }
        else { CHECK(strcmp(end, "\r\n") == 0); }
    }
    CHECK(output_length * 10000000UL / APP_CONSOLE_BAUDRATE < APP_TELEMETRY_PERIOD_MS * 1000U);
}
void test_run(void)
{
    unsigned i;
    uint8_t saw_real_flag = 0U;
    PitchBackendState before;
    PitchStep result_before;
    test_stage = 1U;
    AppConsole_CreateQueue(); AppConsole_Start(); now = 100U;
    CHECK(AppConsole_GetViewHoldMask() == 0U);
    clear_output(); AppConsole_Poll();
    clear_output(); command_line("FEEDER SPEED VIEW"); CHECK(strstr(output, "GROUP|feeder_speed|COLS|9"));
    frame("feeder_speed:", 9U); CHECK(columns[0] == -3568 && columns[1] == -1784 && feeder_speed_frames == 1U);
    clear_output(); command_line("FEEDER VIEW"); frame("feeder:", 3U); CHECK(feeder_detail_frames == 1U);
    jog_state.active = 1U; jog_state.last_speed_selection = FLYWHEEL_JOG_BOTH;
    jog_state.health.feedback[0].erpm = 21000; jog_state.health.feedback[1].erpm = 35000;
    jog_state.health.status1_fresh_mask = 3U; jog_state.health.faults = 4U;
    jog_state.speed.target_erpm = 28000; jog_state.speed_right.target_erpm = 42000;
    clear_output(); command_line("WHEEL VIEW"); CHECK(strstr(output, "GROUP|wheel_speed|COLS|12"));
    frame("wheel_speed:", 12U);
    CHECK(columns[0] == 3000 && columns[1] == 5000 && columns[2] == 4000 && columns[3] == 6000);
    CHECK(columns[6] == 3 && columns[7] == 4 && columns[11] == 7);
    CHECK(feeder_detail_frames == 1U && feeder_speed_frames == 1U);
    clear_output(); command_line("FEEDER ANGLE VIEW");
    CHECK(strstr(output, "GROUP|feeder|COLS|39|ACTUAL_SHAFT_DEG_I|0|TARGET_SHAFT_DEG_I|1"));
    CHECK(strstr(output, "REQUEST_MS|50|CONTROL_MS|2"));
    frame("feeder:", 3U); CHECK(columns[0] == 1 && columns[1] == 2 && feeder_detail_frames == 2U);
    clear_output(); command_line("FEEDER SPEED VIEW"); frame("feeder_speed:", 9U);
    clear_output(); command_line("TELEM FEEDER ANGLE"); frame("feeder:", 3U);
    CHECK(feeder_detail_frames == 3U);
    clear_output(); command_line("WHEEL VIEW"); frame("wheel_speed:", 12U);
    clear_output(); command_line("YAW VIEW"); frame("gimbal:", 72U);
    CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_PITCH);
    clear_output(); command_line("PITCH STEP"); CHECK(strstr(output, "P|ERROR|VIEW_LOCKED_USE_PITCH_VIEW"));
    clear_output(); command_line("PITCH HOME"); CHECK(strstr(output, "P|ERROR|VIEW_LOCKED_USE_PITCH_VIEW"));
    clear_output(); command_line("PITCH TARGET 5"); CHECK(strstr(output, "P|ERROR|VIEW_LOCKED_USE_PITCH_VIEW"));
    clear_output(); command_line("FEEDER SPEED VIEW"); frame("feeder_speed:", 9U);
    CHECK(AppConsole_GetViewHoldMask() == 0U);
    clear_output(); command_line("PITCH VIEW"); frame("pitch:", 65U);
    CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_YAW);
    clear_output(); command_line("YAW STEP"); CHECK(strstr(output, "Y|ERROR|VIEW_LOCKED_USE_YAW_VIEW"));
    clear_output(); command_line("YAW HOME"); CHECK(strstr(output, "Y|ERROR|VIEW_LOCKED_USE_YAW_VIEW"));
    clear_output(); command_line("YAW TARGET 5"); CHECK(strstr(output, "Y|ERROR|VIEW_LOCKED_USE_YAW_VIEW"));
    clear_output(); command_line("GIMBAL VIEW LOCK"); CHECK(strstr(output, APP_GIMBAL_VIEW_HOLD_VERSION "|YAW|1|PITCH|0|"));
    clear_output(); command_line("TELEM OFF"); CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_YAW);
    clear_output(); command_line("TELEM ON");
    clear_output(); command_line("GIMBAL VIEW FREE"); CHECK(AppConsole_GetViewHoldMask() == 0U);
    clear_output(); command_line("TELEM PITCH"); CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_YAW);
    clear_output(); command_line("TELEM YAW"); CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_PITCH);
    clear_output(); command_line("WHEEL VIEW"); CHECK(AppConsole_GetViewHoldMask() == 0U);
    clear_output(); command_line("PITCH VIEW"); CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_YAW);
    CHECK(!stops && !selections && !pitch_cancel_count && !pitch_commands && !yaw_commands);
    test_stage = 2U;
    pitch_sample.online = pitch_sample.active = 1U; pitch_sample.controller.feedback_divisor = 1U;
    pitch_sample.controller.angle_deg = 2.25f; pitch_sample.controller.target_deg = 30.0f;
    pitch_sample.controller.cascade.position.kp = 10.678f;
    pitch_step_sample.phase = PITCH_STEP_DONE; pitch_step_sample.id = 8U;
    pitch_step_sample.settle_ms = 350U; pitch_step_sample.overshoot_deg = 0.3f;
    clear_output(); command_line("PITCH SIM ON"); CHECK(strstr(output, "NOT_A_MEASUREMENT|1"));
    CHECK(strstr(output, "MODE|GUARDED_CONTINUOUS|AUTO_OFF|0|"));
    CHECK(strstr(output, "HOLD_MS|1000|") && strstr(output, "FADE_MS|500|"));
    frame("pitch_demo:", 68U); CHECK(columns[0] == 2.25f && columns[65] == 2.25f && columns[66] == 0 && columns[67] == 1);
    pitch_step_sample.phase = PITCH_STEP_RUNNING; pitch_step_sample.id = 9U;
    pitch_step_sample.start_ms = now + 5U; pitch_step_sample.start_deg = 0.0f; pitch_step_sample.target_deg = 30.0f;
    before = pitch_sample; result_before = pitch_step_sample;
    frame("pitch_demo:", 68U); CHECK(columns[0] == 0 && columns[1] == 30 && columns[66] == 1);
    for (i = 0U; i < 64U; ++i) { frame("pitch_demo:", 68U); }
    CHECK(columns[0] >= 29.85f && columns[0] <= 30.15f && columns[65] == 2.25f && columns[7] == 10.678f);
    CHECK(!memcmp(&before, &pitch_sample, sizeof(before)) && !memcmp(&result_before, &pitch_step_sample, sizeof(result_before)));
    clear_output(); command_line("PITCH SIM"); CHECK(strstr(output, "ACTIVE|1|ARMED|0"));
    CHECK(strstr(output, "DISPLAY_VERSION|6|RISE_STYLE|KICK_RAMP_TAPER|") && strstr(output, "FADE_MS|500"));
    CHECK(strstr(output, "DEMO_DONE|1|"));
    clear_output(); command_line("PITCH SIM ON");
    CHECK(strstr(output, "ACTIVE|1|") && strstr(output, "DEMO_DONE|1|DEMO_LIMIT|2|BATCH_DONE|1|"));
    /* Measured completion no longer ends the display hold. Both outgoing and
     * HOME/return directions update I0 only, retaining real I65 and scores. */
    pitch_step_sample.phase = PITCH_STEP_DONE;
    frame("pitch_demo:", 68U); CHECK(columns[66] == 1 && columns[0] >= 29.85f && columns[0] <= 30.15f);
    pitch_sample.controller.target_deg = 0.0f;
    frame("pitch_demo:", 68U); CHECK(columns[66] == 1 && columns[1] == 0 && columns[65] == 2.25f);
    for (i = 0U; i < 360U; ++i) {
        frame(0, 0U);
        if (strncmp(output, "pitch_demo:", 11U) == 0 && columns[66] == 0) { saw_real_flag = 1U; }
    }
    CHECK(strncmp(output, "pitch_demo:", 11U) == 0 && columns[0] == 2.25f && columns[27] == 350 && saw_real_flag);
    clear_output(); command_line("PITCH SIM");
    CHECK(strstr(output, "ENABLED|1|ACTIVE|0|ARMED|0|") && strstr(output, "DEMO_DONE|2|"));
    CHECK(strstr(output, "BATCH_DONE|2|NEXT_REAL|1|"));
    clear_output(); command_line("TELEM OFF");
    clear_output(); command_line("TELEM ON"); frame("pitch_demo:", 68U);
    clear_output(); command_line("PITCH SIM");
    CHECK(strstr(output, "ENABLED|1|ACTIVE|0|ARMED|0|") && strstr(output, "BATCH_DONE|2|NEXT_REAL|1|"));
    pitch_step_sample.id++; pitch_step_sample.phase = PITCH_STEP_RUNNING;
    pitch_step_sample.start_ms = now + 5U; pitch_step_sample.target_deg = 30.0f;
    pitch_sample.controller.target_deg = 30.0f;
    frame("pitch_demo:", 68U); CHECK(columns[0] == 2.25f && columns[66] == 0);
    clear_output(); command_line("PITCH SIM");
    CHECK(strstr(output, "ENABLED|1|ACTIVE|0|ARMED|0|") && strstr(output, "BATCH_DONE|0|NEXT_REAL|0|"));
    for (i = 0U; i < 55U; ++i) { frame("pitch_demo:", 68U); CHECK(columns[0] == 2.25f && columns[66] == 0); }
    clear_output(); command_line("PITCH SIM"); CHECK(strstr(output, "ENABLED|1|ACTIVE|0|ARMED|1|"));
    clear_output(); command_line("PITCH SIM OFF"); frame("pitch:", 65U); CHECK(columns[0] == 2.25f);
    clear_output(); command_line("PITCH SIM ON"); frame("pitch_demo:", 68U); CHECK(columns[66] == 0);
    CHECK(AppConsole_GetViewHoldMask() == APP_VIEW_HOLD_YAW);
    pitch_step_sample.phase = PITCH_STEP_DONE; frame("pitch_demo:", 68U); CHECK(columns[67] == 1);
    pitch_step_sample.phase = PITCH_STEP_RUNNING; pitch_step_sample.id++;
    pitch_step_sample.start_ms = now + 5U; frame("pitch_demo:", 68U); CHECK(columns[66] == 1);
    clear_output(); command_line("TELEM OFF"); now += 10U; clear_output(); AppConsole_Poll(); CHECK(!output_length);
    clear_output(); command_line("TELEM ON"); frame("pitch_demo:", 68U); CHECK(columns[66] == 0);
    clear_output(); command_line("FEEDER ANGLE VIEW"); frame("feeder:", 3U);
    clear_output(); command_line("PITCH VIEW"); frame("pitch_demo:", 68U); CHECK(columns[66] == 0);
    CHECK(!stops && !selections && !pitch_cancel_count && !pitch_commands && !yaw_commands);
    /* Query distinguishes a requested mask from actual feedback-based hold. */
    pitch_sample.view_locked = 1U; pitch_sample.view_hold_pending = 0U;
    pitch_sample.view_hold_deg = pitch_sample.controller.target_deg = 2.0f;
    pitch_sample.controller.angle_deg = 2.5f;
    clear_output(); command_line("GIMBAL VIEW LOCK");
    CHECK(strstr(output, "PITCH_APPLIED|1|PITCH_PENDING|0|PITCH_HOLD_DEG|2|PITCH_ACTUAL_DEG|2.5|PITCH_TARGET_DEG|2|PITCH_ERROR_DEG|-0.5|"));
    CHECK(strstr(output, "PITCH_POSITION_GAIN|1|PITCH_SPEED_GAIN|1"));
    clear_output(); command_line("YAW VIEW");
    pitch_sample.requested = pitch_sample.coord_confirmed = pitch_sample.zero_set = 1U;
    pitch_sample.controller.have_reference = pitch_sample.controller.gravity_calibrated = 1U;
    pitch_sample.feedback.status = 1U; app_armed = output_allowed = 1U;
    clear_output(); command_line("GIMBAL VIEW LOCK"); CHECK(strstr(output, "PITCH_BLOCK|APPLIED|PITCH_HOLD_VALID|1|"));
    pitch_sample.active = pitch_sample.requested = 0U; pitch_sample.feedback.status = 0U;
    pitch_sample.view_hold_pending = 1U; pitch_sample.view_prepare_pending = 0U;
    pitch_sample.controller.target_deg = -18.0f; pitch_sample.controller.angle_deg = -0.625f;
    clear_output(); command_line("GIMBAL VIEW LOCK");
    CHECK(strstr(output, "PITCH_BLOCK|SESSION_NOT_REQUESTED|PITCH_HOLD_VALID|0|"));
    pitch_sample.view_hold_suspended = 1U;
    clear_output(); command_line("GIMBAL VIEW LOCK"); CHECK(strstr(output, "PITCH_BLOCK|HOLD_SUSPENDED|"));
    clear_output(); command_line("YAW VIEW"); CHECK(pitch_sample.view_prepare_pending);
    CHECK(!pitch_sample.view_hold_suspended);
    clear_output(); command_line("GIMBAL VIEW LOCK"); CHECK(strstr(output, "PITCH_BLOCK|WAIT_VIEW_PREPARE|"));
    app_armed = output_allowed = 0U;
    clear_output(); command_line("GIMBAL VIEW LOCK"); CHECK(strstr(output, "PITCH_BLOCK|REMOTE_NOT_ARMED|"));
    pitch_sample.controller.faults = 16U;
    clear_output(); command_line("GIMBAL VIEW LOCK"); CHECK(strstr(output, "PITCH_BLOCK|PITCH_FAULT|"));
    test_done = 1U;
}
