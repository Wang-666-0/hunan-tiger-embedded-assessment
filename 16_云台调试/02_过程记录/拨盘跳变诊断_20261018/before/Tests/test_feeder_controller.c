#include "feeder_controller.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static FeederController c;
static CanFrame frame;
static double absolute(double value) { return value < 0.0 ? -value : value; }
static uint8_t near(double a, double b) { return (uint8_t)(absolute(a - b) < 0.00001); }
static void make_frame(uint32_t ms, uint16_t angle, int16_t rpm, int16_t current, uint32_t epoch)
{
    memset(&frame, 0, sizeof(frame)); frame.bus = 2U; frame.id = 0x205U; frame.dlc = 8U;
    frame.received_ms = ms; frame.fault_epoch = epoch;
    frame.data[0] = (uint8_t)(angle >> 8); frame.data[1] = (uint8_t)angle;
    frame.data[2] = (uint8_t)((uint16_t)rpm >> 8); frame.data[3] = (uint8_t)rpm;
    frame.data[4] = (uint8_t)((uint16_t)current >> 8); frame.data[5] = (uint8_t)current;
    /* C610 reserves these bytes; deliberately hot-looking bytes are harmless. */
    frame.data[6] = 255U; frame.data[7] = 255U;
}
static void sample(uint32_t ms, uint16_t angle, int16_t rpm, int16_t current)
{
    make_frame(ms, angle, rpm, current, 0U);
    CHECK(Feeder_OnFrame(&c, &frame, ms, 0U));
}
static void ready(uint32_t ms)
{
    FeederConfig config;
    Feeder_Init(&c); config = c.config; config.direction = 1; config.calibrated = 1U;
    CHECK(Feeder_SetConfig(&c, &config));
    sample(ms, 0U, 0, 0); sample(ms + 1U, 0U, 0, 0); sample(ms + 2U, 0U, 0, 0);
    CHECK(Feeder_Zero(&c, ms + 2U));
}
void test_run(void)
{
    FeederConfig config;
    uint32_t ms, accepted, sequence;
    int16_t output;
    union { uint32_t bits; float value; } invalid;
    test_stage = 1U;
    Feeder_Init(0); Feeder_Stop(0);
    CHECK(!Feeder_ConfigValid(0) && !Feeder_SetConfig(0, 0));
    CHECK(!Feeder_OnFrame(0, 0, 0U, 0U));
    CHECK(!Feeder_Clear(0, 0U) && !Feeder_Zero(0, 0U));
    CHECK(Feeder_Step(0, 0U, 1U, 1U, 1U, 1U) == 0);
    Feeder_Init(&c); CHECK(Feeder_ConfigValid(&c.config));
    CHECK(c.config.step_deg == 40.0f && c.config.gear_ratio == 1.0f && !c.config.direction && !c.config.calibrated);
    CHECK(c.config.current_limit == 1000.0f && c.config.max_rpm == 300.0f);
    CHECK(!c.have_zero && !c.have_feedback && !c.faults);
    CHECK(Feeder_Step(&c, 0U, 1U, 1U, 1U, 1U) == 0 && c.rejected == 1U);
    CHECK(!Feeder_Zero(&c, 0U));
    sample(100U, 8190U, 100, -1234); CHECK(!c.faults && c.raw_current == -1234 && c.position_deg == 0.0);
    sample(102U, 10U, 100, 1234); CHECK(c.accumulated_counts == 12 && !c.faults);
    sample(104U, 8190U, -100, -100); CHECK(c.accumulated_counts == 0 && !c.faults);
    make_frame(105U, 0U, 0, 0, 0U); frame.bus = 1U;
    CHECK(!Feeder_OnFrame(&c, &frame, 105U, 0U) && c.raw_angle == 8190U);
    frame.bus = 2U; frame.id = 0x204U; CHECK(!Feeder_OnFrame(&c, &frame, 105U, 0U));
    frame.id = 0x205U; frame.extended = 1U; CHECK(!Feeder_OnFrame(&c, &frame, 105U, 0U));
    frame.extended = 0U; frame.dlc = 7U; CHECK(!Feeder_OnFrame(&c, &frame, 105U, 0U));
    CHECK(c.faults & FEEDER_FAULT_DATA);

    test_stage = 2U;
    ready(200U); CHECK(c.have_zero && !c.faults);
    output = Feeder_Step(&c, 202U, 1U, 1U, 1U, 1U);
    CHECK(output == 400 && c.busy && c.accepted == 1U && near(c.target_deg, 40.0));
    CHECK(!Feeder_SetConfig(&c, &c.config) && !Feeder_Zero(&c, 202U));
    CHECK(Feeder_Step(&c, 203U, 1U, 1U, 0U, 0U) == output && c.integral == 0.0f);
    CHECK(Feeder_Step(&c, 203U, 1U, 1U, 1U, 2U) == output && c.rejected == 1U);
    CHECK(Feeder_Step(&c, 204U, 1U, 1U, 1U, 2U) == output && c.rejected == 1U);
    sample(204U, 30U, 150, 100);
    CHECK(Feeder_Step(&c, 204U, 1U, 1U, 0U, 0U) > 0 && c.integral > 0.0f);
    CHECK(Feeder_Step(&c, 205U, 1U, 0U, 1U, 3U) == 0 && !c.busy && c.aborted == 1U && c.rejected == 2U);
    CHECK(c.command_raw == 0 && c.integral == 0.0f && near(c.target_deg, c.position_deg));
    CHECK(Feeder_Step(&c, 206U, 1U, 1U, 1U, 3U) == 0 && c.accepted == 1U);
    CHECK(Feeder_Step(&c, 207U, 1U, 1U, 1U, 4U) > 0 && c.accepted == 2U);
    CHECK(Feeder_Step(&c, 208U, 0U, 1U, 0U, 0U) == 0 && c.aborted == 2U && c.have_zero);
    CHECK(Feeder_Step(&c, 209U, 1U, 1U, 1U, 1U) == 0 && c.accepted == 2U);

    test_stage = 3U;
    ready(300U); CHECK(Feeder_Step(&c, 302U, 1U, 1U, 1U, 1U) > 0);
    for (ms = 303U; ms <= 311U; ++ms) {
        sample(ms, (uint16_t)((ms - 302U) * 100U), 600, 100);
        (void)Feeder_Step(&c, ms, 1U, 1U, 0U, 0U);
    }
    sample(312U, 910U, 0, 0); (void)Feeder_Step(&c, 312U, 1U, 1U, 0U, 0U);
    CHECK(c.settling && c.busy && !c.faults);
    for (ms = 313U; ms <= 332U; ++ms) {
        sample(ms, 910U, 0, 0); (void)Feeder_Step(&c, ms, 1U, 1U, 0U, 0U);
    }
    CHECK(!c.busy && c.completed == 1U && !c.aborted && c.command_raw == 0);
    CHECK(near(c.target_deg, 40.0) && c.position_deg < 40.0);
    CHECK(Feeder_Step(&c, 332U, 1U, 1U, 1U, 2U) > 0 && near(c.target_deg, 80.0));
    Feeder_Stop(&c); CHECK(c.aborted == 1U && near(c.target_deg, c.position_deg));
    config = c.config; config.gear_ratio = 36.0f; config.direction = -1;
    CHECK(Feeder_SetConfig(&c, &config) && !c.have_zero);
    CHECK(Feeder_Zero(&c, 332U));
    CHECK(Feeder_Step(&c, 332U, 1U, 1U, 1U, 3U) < 0);
    CHECK(near(c.target_deg, -1440.0) && near(c.axis_target_deg, -40.0));
    Feeder_Stop(&c); config = c.config; config.calibrated = 0U;
    CHECK(Feeder_SetConfig(&c, &config) && !Feeder_Zero(&c, 332U));

    test_stage = 4U;
    ready(400U); config = c.config;
    config.current_limit = 3001.0f; CHECK(!Feeder_ConfigValid(&config) && !Feeder_SetConfig(&c, &config));
    config = c.config; config.max_rpm = 6001.0f; CHECK(!Feeder_ConfigValid(&config));
    config = c.config; config.direction = 2; CHECK(!Feeder_ConfigValid(&config));
    config = c.config; config.direction = 0; CHECK(!Feeder_ConfigValid(&config));
    config = c.config; config.gear_ratio = 0.0f; CHECK(!Feeder_ConfigValid(&config));
    config = c.config; config.gear_ratio = 100.1f; CHECK(!Feeder_ConfigValid(&config));
    config = c.config; config.motion_timeout_ms = config.jam_ms; CHECK(!Feeder_ConfigValid(&config));
    config = c.config; invalid.bits = 0x7FC00000U; config.speed_kp = invalid.value; CHECK(!Feeder_ConfigValid(&config));
    config = c.config; invalid.bits = 0x7F800000U; config.speed_ki = invalid.value; CHECK(!Feeder_ConfigValid(&config));
    make_frame(403U, 8192U, 0, 0, 0U); CHECK(!Feeder_OnFrame(&c, &frame, 403U, 0U) && (c.faults & FEEDER_FAULT_DATA));
    CHECK(!c.have_zero);
    ready(410U); sample(413U, 0U, 6001, 0); CHECK(c.faults & FEEDER_FAULT_OVERSPEED);
    ready(420U); sample(423U, 0U, 0, -10001); CHECK(c.faults & FEEDER_FAULT_CURRENT);
    ready(430U); sample(433U, 200U, 0, 0); CHECK(c.faults & FEEDER_FAULT_JUMP);
    ready(440U); sample(452U, 0U, 3000, 0); CHECK(c.faults & FEEDER_FAULT_JUMP);

    test_stage = 5U;
    ready(500U); CHECK(Feeder_Step(&c, 502U, 1U, 1U, 1U, 1U) > 0);
    sample(523U, 0U, 0, 0);
    CHECK((c.faults & FEEDER_FAULT_FEEDBACK) && !c.busy && !c.have_zero && c.aborted == 1U);
    CHECK(!Feeder_Clear(&c, 523U));
    sample(524U, 0U, 0, 0); sample(525U, 0U, 0, 0); sample(526U, 0U, 0, 0);
    CHECK(Feeder_Clear(&c, 526U) && !c.faults && !c.have_zero);
    CHECK(Feeder_Step(&c, 526U, 1U, 1U, 1U, 2U) == 0 && c.rejected == 1U);
    CHECK(Feeder_Zero(&c, 526U));
    CHECK(Feeder_Step(&c, 526U, 1U, 1U, 1U, 2U) == 0 && c.accepted == 1U);
    CHECK(Feeder_Step(&c, 526U, 1U, 1U, 1U, 3U) > 0 && c.accepted == 2U);
    CHECK(Feeder_Step(&c, 537U, 1U, 1U, 0U, 0U) == 0 && (c.faults & FEEDER_FAULT_TIMING));
    ready(550U); CHECK(Feeder_Step(&c, 573U, 0U, 0U, 0U, 0U) == 0);
    CHECK(c.faults & FEEDER_FAULT_FEEDBACK);
    ready(580U); make_frame(583U, 0U, 0, 0, 1U);
    CHECK(Feeder_OnFrame(&c, &frame, 583U, 1U) && (c.faults & FEEDER_FAULT_CAN) && !c.have_zero);
    make_frame(584U, 0U, 0, 0, 0U); CHECK(!Feeder_OnFrame(&c, &frame, 584U, 1U));
    ready(590U); make_frame(593U, 0U, 0, 0, 0U);
    CHECK(!Feeder_OnFrame(&c, &frame, 614U, 0U) && (c.faults & FEEDER_FAULT_FEEDBACK));
    ready(620U); sequence = c.feedback_sequence;
    make_frame(622U, 100U, 0, 0, 0U); CHECK(!Feeder_OnFrame(&c, &frame, 622U, 0U) && c.feedback_sequence == sequence);
    frame.received_ms = 621U; CHECK(!Feeder_OnFrame(&c, &frame, 622U, 0U));
    CHECK(!c.faults && !c.accumulated_counts);

    test_stage = 6U;
    ready(1000U); config = c.config; config.pos_kp = 100.0f; config.speed_kp = 10.0f;
    CHECK(Feeder_SetConfig(&c, &config) && c.have_zero);
    CHECK(Feeder_Step(&c, 1002U, 1U, 1U, 1U, 1U) == 1000);
    for (ms = 1003U; ms <= 1152U; ++ms) {
        sample(ms, 0U, 0, 900); (void)Feeder_Step(&c, ms, 1U, 1U, 0U, 0U);
    }
    CHECK((c.faults & FEEDER_FAULT_JAM) && !c.busy && c.aborted == 1U && !c.have_zero && !c.integral);
    ready(2000U); config = c.config; config.jam_current_raw = 3000.0f;
    CHECK(Feeder_SetConfig(&c, &config));
    CHECK(Feeder_Step(&c, 2002U, 1U, 1U, 1U, 1U) > 0);
    for (ms = 2003U; ms <= 3002U; ++ms) {
        sample(ms, 0U, 0, 0); (void)Feeder_Step(&c, ms, 1U, 1U, 0U, 0U);
    }
    CHECK((c.faults & FEEDER_FAULT_TIMEOUT) && !c.busy && c.aborted == 1U && !c.completed);
    ready(4000U); accepted = c.accepted;
    CHECK(Feeder_Step(&c, 4002U, 1U, 1U, 1U, 0xFFFFFFFFU) > 0);
    Feeder_Stop(&c);
    CHECK(Feeder_Step(&c, 4002U, 1U, 1U, 1U, 0U) > 0 && c.accepted == accepted + 2U);
    CHECK(c.command_raw <= 1000 && c.command_raw >= -1000);
    test_done = 1U;
}
