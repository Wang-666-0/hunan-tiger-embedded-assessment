#include "flywheel_jog.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static FlywheelJog jog;

static CanFrame make_frame(uint8_t node, uint32_t packet, uint32_t time, int32_t value)
{
    CanFrame result; uint32_t raw = (uint32_t)value;
    memset(&result, 0, sizeof(result));
    result.bus = 2U; result.extended = 1U; result.dlc = 8U;
    result.id = (packet << 8) | node; result.received_ms = time;
    if (packet == 9U) {
        result.data[0] = (uint8_t)(raw >> 24); result.data[1] = (uint8_t)(raw >> 16);
        result.data[2] = (uint8_t)(raw >> 8); result.data[3] = (uint8_t)raw;
    } else { result.data[0] = (uint8_t)(raw >> 8); result.data[1] = (uint8_t)raw; }
    return result;
}

static void feedback(uint32_t time, uint8_t selected, int32_t erpm)
{
    uint8_t node;
    for (node = 70U; node <= 71U; ++node) {
        CanFrame frame = make_frame(node, 9U, time, node == selected ? erpm : 0);
        CHECK(FlywheelJog_OnFrame(&jog, &frame, time, 0U));
        frame = make_frame(node, 16U, time, 250);
        CHECK(FlywheelJog_OnFrame(&jog, &frame, time, 0U));
    }
}

static void drain(uint8_t selected)
{
    uint8_t node; int32_t current;
    while (FlywheelJog_TakeCommand(&jog, &node, &current)) {
        CHECK(current >= 0 && current <= 500);
        CHECK(node == selected || current == 0);
    }
}

static void start(uint8_t node)
{
    FlywheelJog_Init(&jog); feedback(100U, node, 0);
    FlywheelJog_Step(&jog, 100U, 0U, 0U, 1U, 0U);
    CHECK(FlywheelJog_Select(&jog, node == 70U ? FLYWHEEL_JOG_LEFT : FLYWHEEL_JOG_RIGHT, 100U, 0U));
    FlywheelJog_Step(&jog, 101U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 102U, 1U, 0U, 1U, 0U);
    CHECK(jog.ready && !jog.active);
    FlywheelJog_Step(&jog, 103U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.speed.running && jog.current_mA == 0);
    drain(node);
}

void test_run(void)
{
    uint8_t node; uint32_t time; CanFrame frame;
    test_stage = 1U;
    for (node = 70U; node <= 71U; ++node) {
        start(node);
        for (time = 123U; time <= 3103U; time += 20U) {
            feedback(time, node, 0);
            FlywheelJog_Step(&jog, time, 1U, 1U, 0U, 0U);
            CHECK(jog.active && jog.faults == 0U && jog.current_mA <= 500);
            drain(node);
        }
        CHECK(jog.speed.target_erpm == 1400.0f && jog.current_mA == 500);
        CHECK(jog.speed.at_limit && !jog.speed.in_band);
        feedback(3105U, node, 1600);
        FlywheelJog_Step(&jog, 3105U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.coasting && jog.current_mA == 0 && !jog.faults);
        CHECK(jog.speed.peak_erpm == 1600U);
        feedback(3125U, node, 1401);
        FlywheelJog_Step(&jog, 3125U, 1U, 1U, 0U, 0U);
        CHECK(jog.coasting && jog.current_mA == 0);
        feedback(3145U, node, 1400);
        FlywheelJog_Step(&jog, 3145U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && !jog.coasting && !jog.faults);
        drain(node);
        FlywheelJog_Step(&jog, 3146U, 1U, 0U, 1U, 0U);
        CHECK(!jog.active && !jog.speed.running && jog.current_mA == 0);
        CHECK(jog.speed.peak_erpm == 1600U && jog.speed.integral_mA == 0.0f);
        drain(node);
        FlywheelJog_Step(&jog, 3147U, 1U, 1U, 0U, 0U);
        CHECK(!jog.active);
        FlywheelJog_Step(&jog, 3148U, 0U, 0U, 1U, 0U);
        FlywheelJog_Step(&jog, 3149U, 1U, 0U, 1U, 0U);
        CHECK(!jog.ready); /* 两轮未停稳，不能重起动。 */
    }

    test_stage = 2U;
    start(70U); feedback(123U, 70U, 0);
    FlywheelJog_Step(&jog, 123U, 1U, 1U, 0U, 0U);
    frame = make_frame(70U, 9U, 124U, 2001);
    CHECK(FlywheelJog_OnFrame(&jog, &frame, 124U, 0U));
    CHECK(!jog.active && !jog.speed.running && jog.current_mA == 0);
    CHECK(jog.faults == FLYWHEEL_JOG_FAULT_OVERSPEED && jog.speed.peak_erpm == 2001U);
    drain(70U);
    start(71U);
    frame = make_frame(70U, 9U, 104U, -2001);
    CHECK(FlywheelJog_OnFrame(&jog, &frame, 104U, 0U));
    CHECK(!jog.active && jog.faults == FLYWHEEL_JOG_FAULT_OVERSPEED);
    CHECK(jog.speed.peak_erpm == 0U); /* 所选轮统计不混入另一轮，但两轮都受保护。 */

    test_stage = 3U;
    start(70U);
    frame = make_frame(70U, 9U, 201U, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &frame, 201U, 0U));
    FlywheelJog_Step(&jog, 201U, 1U, 1U, 0U, 0U);
    CHECK(!jog.active && !jog.speed.running && (jog.faults & FLYWHEEL_JOG_FAULT_FEEDBACK));
    start(71U);
    frame = make_frame(71U, 9U, 104U, 0); frame.dlc = 7U;
    CHECK(!FlywheelJog_OnFrame(&jog, &frame, 104U, 0U));
    CHECK(!jog.active && (jog.faults & FLYWHEEL_JOG_FAULT_FRAME));
    start(70U);
    frame = make_frame(70U, 9U, 104U, 0); frame.data[5] = 11U;
    CHECK(FlywheelJog_OnFrame(&jog, &frame, 104U, 0U));
    CHECK(!jog.active && (jog.faults & FLYWHEEL_JOG_FAULT_CURRENT));
    start(71U);
    frame = make_frame(70U, 16U, 104U, 600);
    CHECK(FlywheelJog_OnFrame(&jog, &frame, 104U, 0U));
    CHECK(!jog.active && (jog.faults & FLYWHEEL_JOG_FAULT_TEMPERATURE));
    start(70U);
    FlywheelJog_Step(&jog, 104U, 1U, 1U, 0U, 1U);
    CHECK(!jog.active && !jog.speed.running && (jog.faults & FLYWHEEL_JOG_FAULT_CAN_EPOCH));
    start(71U);
    FlywheelJog_Step(&jog, 104U, 0U, 1U, 0U, 0U);
    CHECK(!jog.active && !jog.speed.running && jog.current_mA == 0);
    drain(71U);
    test_done = 1U;
}
