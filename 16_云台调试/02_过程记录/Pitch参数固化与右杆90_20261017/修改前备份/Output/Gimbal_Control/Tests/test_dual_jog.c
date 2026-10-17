#include "flywheel_jog.h"
#include <string.h>
#ifndef TEST_HIGH_SPEED
#define TEST_HIGH_SPEED 0
#endif
#define TEST_MAX_MA (TEST_HIGH_SPEED ? 2000 : 1000)
#define TEST_TARGET (TEST_HIGH_SPEED ? 42000 : 7000)
#define TEST_SOFT (TEST_HIGH_SPEED ? 44100 : 8000)
#define TEST_HARD (TEST_HIGH_SPEED ? 46200 : 9000)
#define TEST_FB_HUNDRED (TEST_HIGH_SPEED ? 30 : 20)

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static FlywheelJog jog;

static CanFrame make_frame(uint8_t node, uint32_t packet, uint32_t time, int32_t value)
{
    CanFrame frame; uint32_t raw = (uint32_t)value;
    memset(&frame, 0, sizeof(frame));
    frame.bus = 2U; frame.extended = 1U; frame.dlc = 8U;
    frame.id = (packet << 8) | node; frame.received_ms = time;
    if (packet == 9U) {
        frame.data[0] = (uint8_t)(raw >> 24); frame.data[1] = (uint8_t)(raw >> 16);
        frame.data[2] = (uint8_t)(raw >> 8); frame.data[3] = (uint8_t)raw;
    } else { frame.data[0] = (uint8_t)(raw >> 8); frame.data[1] = (uint8_t)raw; }
    return frame;
}

static void feedback(uint32_t time, int32_t left, int32_t right)
{
    uint8_t node;
    for (node = 70U; node <= 71U; ++node) {
        CanFrame frame = make_frame(node, 9U, time, node == 70U ? left : right);
        CHECK(FlywheelJog_OnFrame(&jog, &frame, time, 0U));
        frame = make_frame(node, 16U, time, 250);
        CHECK(FlywheelJog_OnFrame(&jog, &frame, time, 0U));
    }
}

static void drain(void)
{
    uint8_t node, count = 0U; int32_t current;
    while (FlywheelJog_TakeCommand(&jog, &node, &current)) {
        CHECK(current >= 0 && current <= TEST_MAX_MA);
        CHECK(node == (count == 0U ? 70U : 71U));
        CHECK(current == (node == 70U ? jog.current_mA : jog.right_current_mA));
        ++count;
    }
    CHECK(count == 2U);
}

static void start(void)
{
    FlywheelJog_Init(&jog); feedback(100U, 0, 0);
    FlywheelJog_Step(&jog, 100U, 0U, 0U, 1U, 0U);
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_BOTH, 100U, 0U));
    FlywheelJog_Step(&jog, 101U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 102U, 1U, 0U, 1U, 0U);
    CHECK(jog.ready && !jog.active);
    FlywheelJog_Step(&jog, 103U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.speed.running && jog.speed_right.running);
    CHECK(jog.last_speed_selection == FLYWHEEL_JOG_BOTH && jog.stop_reason == FLYWHEEL_STOP_NONE);
    drain();
}

static void stopped(uint32_t fault)
{
    CHECK(!jog.active && !jog.speed.running && !jog.speed_right.running);
    CHECK(jog.current_mA == 0 && jog.right_current_mA == 0 && jog.faults == fault);
    CHECK(jog.stop_reason == FLYWHEEL_STOP_FAULT); drain();
}

void test_run(void)
{
    uint32_t time; uint8_t node; CanFrame frame;
    test_stage = 1U;
    CHECK(APP_FLYWHEEL_SPEED_TARGET_ERPM == TEST_TARGET);
    CHECK(APP_FLYWHEEL_JOG_MAX_MA == TEST_MAX_MA);
    start();
    for (time = 123U; time <= 6123U; time += 20U) {
        feedback(time, 0, 0); FlywheelJog_Step(&jog, time, 1U, 1U, 0U, 0U);
        CHECK(jog.active && !jog.faults && jog.stop_reason == FLYWHEEL_STOP_NONE); drain();
    }
    CHECK(jog.speed.target_erpm == (float)TEST_TARGET && jog.speed_right.target_erpm == (float)TEST_TARGET);
    CHECK(jog.current_mA == TEST_MAX_MA && jog.right_current_mA == TEST_MAX_MA);
    CHECK(jog.speed.low_speed_ms > 0U && jog.speed_right.low_speed_ms > 0U);
    /* 左轮已有速度、右轮速度不足，给定必须独立，不能将左电流复制到右轮。 */
    for (time = 6143U; time <= 7143U; time += 20U) {
        feedback(time, TEST_TARGET + APP_FLYWHEEL_SPEED_BAND_ERPM - 50, 0);
        FlywheelJog_Step(&jog, time, 1U, 1U, 0U, 0U); drain();
    }
    CHECK(jog.active && jog.current_mA < jog.right_current_mA && jog.right_current_mA == TEST_MAX_MA);
    CHECK(jog.speed.peak_erpm == (uint32_t)(TEST_TARGET + APP_FLYWHEEL_SPEED_BAND_ERPM - 50) && jog.speed_right.peak_erpm == 0U);
    CHECK(jog.speed.rise90_seen && jog.speed.rise90_ms == 6040U && !jog.speed_right.rise90_seen);
    feedback(7145U, TEST_SOFT, 0); FlywheelJog_Step(&jog, 7145U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.speed.coasting && !jog.speed_right.coasting && !jog.faults);
    CHECK(jog.current_mA == 0 && jog.right_current_mA == TEST_MAX_MA);
    feedback(7165U, TEST_TARGET + 1, 0); FlywheelJog_Step(&jog, 7165U, 1U, 1U, 0U, 0U); drain();
    CHECK(jog.current_mA == 0 && jog.right_current_mA == TEST_MAX_MA);
    feedback(7185U, 0, 0); FlywheelJog_Step(&jog, 7185U, 1U, 1U, 0U, 0U); drain();
    CHECK(!jog.speed.coasting && jog.speed.coast_ms > 0U && jog.active);
    FlywheelJog_Step(&jog, 7186U, 1U, 0U, 1U, 0U);
    CHECK(!jog.active && jog.stop_reason == FLYWHEEL_STOP_INPUT); drain();
    CHECK(jog.speed.integral_mA == 0.0f && jog.speed_right.integral_mA == 0.0f);
    FlywheelJog_Step(&jog, 7187U, 1U, 1U, 0U, 0U); CHECK(!jog.active);
    feedback(7188U, 0, 0); FlywheelJog_Step(&jog, 7188U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 7189U, 1U, 0U, 1U, 0U); CHECK(jog.ready);
    FlywheelJog_Step(&jog, 7190U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.stop_reason == FLYWHEEL_STOP_NONE && jog.speed_right.peak_erpm == 0U);

    test_stage = 2U;
    for (node = 70U; node <= 71U; ++node) {
        start(); frame = make_frame(node, 9U, 104U, node == 70U ? TEST_HARD + 1 : -TEST_HARD - 1);
        CHECK(FlywheelJog_OnFrame(&jog, &frame, 104U, 0U)); stopped(FLYWHEEL_JOG_FAULT_OVERSPEED);
        CHECK((node == 70U ? jog.speed.peak_erpm : jog.speed_right.peak_erpm) == (uint32_t)TEST_HARD + 1U);
        feedback(105U, 0, 0); FlywheelJog_Step(&jog, 105U, 0U, 0U, 1U, 0U);
        FlywheelJog_Step(&jog, 106U, 1U, 0U, 1U, 0U);
        FlywheelJog_Step(&jog, 107U, 1U, 1U, 0U, 0U); CHECK(!jog.active && !jog.ready);
        FlywheelJog_Step(&jog, 108U, 0U, 0U, 1U, 0U);
        CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_OFF, 108U, 0U));
        CHECK(FlywheelJog_ClearFault(&jog, 108U, 0U));
        CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_BOTH, 108U, 0U));
        FlywheelJog_Step(&jog, 109U, 0U, 0U, 1U, 0U);
        FlywheelJog_Step(&jog, 110U, 1U, 0U, 1U, 0U);
        FlywheelJog_Step(&jog, 111U, 1U, 1U, 0U, 0U); CHECK(jog.active && !jog.faults);
    }

    test_stage = 3U;
    for (node = 70U; node <= 71U; ++node) {
        start(); frame = make_frame(node, 9U, 104U, 0); frame.data[5] = TEST_FB_HUNDRED;
        CHECK(FlywheelJog_OnFrame(&jog, &frame, 104U, 0U)); CHECK(jog.active && !jog.faults);
        frame.received_ms = 105U; frame.data[5] = TEST_FB_HUNDRED + 1U;
        CHECK(FlywheelJog_OnFrame(&jog, &frame, 105U, 0U)); stopped(FLYWHEEL_JOG_FAULT_CURRENT);
        start(); frame = make_frame(node, 16U, 104U, 250);
        frame.data[4] = 0xFFU; frame.data[5] = (uint8_t)(-TEST_FB_HUNDRED - 1); /* 超出本档负输入电流门限。 */
        CHECK(FlywheelJog_OnFrame(&jog, &frame, 104U, 0U)); stopped(FLYWHEEL_JOG_FAULT_CURRENT);
        start(); frame = make_frame(node, 16U, 104U, 600);
        CHECK(FlywheelJog_OnFrame(&jog, &frame, 104U, 0U)); stopped(FLYWHEEL_JOG_FAULT_TEMPERATURE);
        start(); frame = make_frame(node, 9U, 104U, 0); frame.dlc = 7U;
        CHECK(!FlywheelJog_OnFrame(&jog, &frame, 104U, 0U)); stopped(FLYWHEEL_JOG_FAULT_FRAME);
        start(); frame = make_frame(node, 9U, 201U, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &frame, 201U, 0U));
        FlywheelJog_Step(&jog, 201U, 1U, 1U, 0U, 0U); stopped(FLYWHEEL_JOG_FAULT_FEEDBACK);
    }
    start(); FlywheelJog_Step(&jog, 104U, 1U, 1U, 0U, 1U);
    /* 总线故障代次改变会同时清空旧反馈，两个故障位都应保留。 */
    stopped(FLYWHEEL_JOG_FAULT_CAN_EPOCH | FLYWHEEL_JOG_FAULT_FEEDBACK);
    start(); FlywheelJog_Step(&jog, 104U, 0U, 1U, 0U, 0U);
    CHECK(!jog.active && jog.stop_reason == FLYWHEEL_STOP_PERMISSION); drain();
    FlywheelJog_Stop(&jog); CHECK(jog.stop_reason == FLYWHEEL_STOP_PERMISSION);
    test_done = 1U;
}
