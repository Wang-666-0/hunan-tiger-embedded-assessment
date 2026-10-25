#include "flywheel_jog.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

static FlywheelJog jog;
static FlywheelJogHealth health;

static void put16(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw >> 8);
    data[1] = (uint8_t)raw;
}

static void put32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw >> 24);
    data[1] = (uint8_t)(raw >> 16);
    data[2] = (uint8_t)(raw >> 8);
    data[3] = (uint8_t)raw;
}

static CanFrame frame(uint8_t node, uint32_t packet, uint32_t timestamp,
                      int32_t first, int32_t second, int32_t third)
{
    CanFrame result;
    memset(&result, 0, sizeof(result));
    result.bus = 2U;
    result.extended = 1U;
    result.dlc = 8U;
    result.id = (packet << 8) | node;
    result.received_ms = timestamp;
    if (packet == 9U) {
        put32(result.data, first);
        put16(result.data + 4U, second);
        put16(result.data + 6U, third);
    } else {
        put16(result.data, first);
        put16(result.data + 2U, second);
        put16(result.data + 4U, third);
    }
    return result;
}

static void feedback(uint32_t timestamp)
{
    uint8_t node;
    for (node = 70U; node <= 71U; ++node) {
        CanFrame f = frame(node, 9U, timestamp, 0, 0, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, timestamp, 0U));
        f = frame(node, 16U, timestamp, 250, -100, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, timestamp, 0U));
    }
}

static void commands(int32_t left, int32_t right)
{
    uint8_t node = 0U;
    int32_t current = -1;
    CHECK(FlywheelJog_GetRequestedCurrent(&jog) == left + right);
    CHECK(FlywheelJog_TakeCommand(&jog, &node, &current));
    CHECK(node == 70U && current == left);
    CHECK(FlywheelJog_TakeCommand(&jog, &node, &current));
    CHECK(node == 71U && current == right);
    CHECK(!FlywheelJog_TakeCommand(&jog, &node, &current));
}

#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U
static void observation_feedback(uint32_t timestamp, uint8_t selected, int32_t erpm)
{
    uint8_t node;
    for (node = 70U; node <= 71U; ++node) {
        CanFrame f = frame(node, 9U, timestamp, node == selected ? erpm : 0, 0, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, timestamp, 0U));
        f = frame(node, 16U, timestamp, 250, -100, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, timestamp, 0U));
    }
}
#endif

static void prepare(FlywheelJogSelection selection, uint32_t timestamp)
{
    FlywheelJog_Init(&jog);
    feedback(timestamp);
    FlywheelJog_Step(&jog, timestamp, 0U, 0U, 1U, 0U);
    CHECK(FlywheelJog_Select(&jog, selection, timestamp, 0U));
    CHECK(jog.reset_seen == 0U && !jog.ready);
    FlywheelJog_Step(&jog, timestamp + 1U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, timestamp + 2U, 1U, 0U, 1U, 0U);
    CHECK(jog.ready && !jog.active);
    FlywheelJog_Step(&jog, timestamp + 3U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.current_mA == 0);
    commands(0, 0);
}

void test_run(void)
{
    CanFrame f;
    uint8_t node;
    int32_t current;
    uint32_t time;

    test_stage = 1U;
    FlywheelJog_Init(&jog);
    CHECK(jog.selection == FLYWHEEL_JOG_OFF && jog.faults == 0U);
    CHECK(FlywheelJog_GetRequestedCurrent(&jog) == 0 && FlywheelJog_GetRequestedCurrent(0) == 0);
    CHECK(!FlywheelJog_TakeCommand(&jog, &node, &current));
    FlywheelJog_GetHealth(&jog, 100U, 0U, &health);
    CHECK(!health.valid && !health.fresh && !health.stopped);
    CHECK(jog.faults == 0U && jog.pending_mask == 0U);
    feedback(100U);
    FlywheelJog_GetHealth(&jog, 100U, 0U, &health);
    CHECK(health.valid && health.fresh && health.stopped);
    CHECK(health.status1_fresh_mask == 3U && health.status4_fresh_mask == 3U);
    CHECK(health.feedback[0].fet_deci_c == 250);
    CHECK(health.feedback[0].motor_deci_c == -100);
    CHECK(!health.feedback[0].motor_temperature_valid);
    FlywheelJog_GetHealth(&jog, 200U, 0U, &health);
    CHECK(health.valid);
    FlywheelJog_GetHealth(&jog, 201U, 0U, &health);
    CHECK(!health.valid && health.status1_fresh_mask == 0U && !health.stopped);
    for (node = 70U; node <= 71U; ++node) {
        f = frame(node, 9U, 700U, 0, 0, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, 700U, 0U));
    }
    FlywheelJog_GetHealth(&jog, 700U, 0U, &health);
    CHECK(health.valid && health.status4_fresh_mask == 3U);
    FlywheelJog_GetHealth(&jog, 701U, 0U, &health);
    CHECK(!health.valid && health.status4_fresh_mask == 0U);
    CHECK(jog.pending_mask == 0U && jog.faults == 0U);

    test_stage = 2U;
    FlywheelJog_Init(&jog);
    feedback(100U);
    f = frame(70U, 9U, 110U, 0, 0, 0);
    f.bus = 1U; CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    f.bus = 2U; f.extended = 0U; CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    f.extended = 1U; f.id = (9U << 8) | 69U;
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    f.id = (10U << 8) | 70U; CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    f.id = (9U << 8) | 70U; f.fault_epoch = 1U;
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    f.fault_epoch = 0U; f.received_ms = 99U;
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    f.received_ms = 100U; CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    CHECK(jog.feedback[0].status1_ms == 100U);
    f.received_ms = 120U; CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    CHECK(jog.feedback[0].status1_ms == 100U);
    f.received_ms = 110U; f.dlc = 7U;
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    CHECK(!jog.feedback[0].has_status1 && jog.faults == 0U);
    CHECK(jog.feedback[0].status1_ms == 110U);
    f.dlc = 8U; f.received_ms = 105U;
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    CHECK(!jog.feedback[0].has_status1);
    f.received_ms = 111U; CHECK(FlywheelJog_OnFrame(&jog, &f, 111U, 0U));
    f.received_ms = 112U; put16(f.data + 6U, 1001);
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 112U, 0U));
    CHECK(!jog.feedback[0].has_status1 && jog.feedback[0].status1_ms == 112U);
    f = frame(70U, 16U, 120U, -1, -100, 0);
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 120U, 0U));
    CHECK(!jog.feedback[0].has_status4);
    CHECK(jog.pending_mask == 0U && jog.faults == 0U);
    f = frame(70U, 9U, 121U, (-2147483647 - 1), -32768, -1000);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 121U, 0U));
    CHECK(jog.feedback[0].erpm == (-2147483647 - 1));
    CHECK(jog.feedback[0].motor_current_mA == -3276800);
    CHECK(jog.feedback[0].duty_milli == -1000);
    CHECK(jog.faults == 0U && jog.pending_mask == 0U);
    f = frame(70U, 9U, 122U, 0, 0, 0);
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 223U, 0U));
    CHECK(jog.feedback[0].status1_ms == 121U);
    CHECK(!FlywheelJog_OnFrame(&jog, 0, 223U, 0U));
    CHECK(!FlywheelJog_OnFrame(0, &f, 223U, 0U));
    CHECK(!FlywheelJog_Select(0, FLYWHEEL_JOG_LEFT, 223U, 0U));
    CHECK(!FlywheelJog_Select(&jog, (FlywheelJogSelection)99, 223U, 0U));
    CHECK(!FlywheelJog_TakeCommand(&jog, 0, &current));
    CHECK(!FlywheelJog_TakeCommand(&jog, &node, 0));
    FlywheelJog_Init(0); FlywheelJog_Stop(0); FlywheelJog_Fault(0, 1U);
    FlywheelJog_GetHealth(0, 0U, 0U, &health);
    CHECK(!health.valid);
    FlywheelJog_GetHealth(&jog, 0U, 0U, 0);
    FlywheelJog_Step(0, 0U, 0U, 0U, 0U, 0U);

    test_stage = 3U;
    FlywheelJog_Init(&jog); feedback(100U);
    CHECK(!FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 100U, 0U));
    FlywheelJog_Step(&jog, 100U, 0U, 0U, 1U, 0U);
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 100U, 0U));
    CHECK(!jog.reset_seen);
    commands(0, 0);
    FlywheelJog_Step(&jog, 100U, 0U, 0U, 1U, 0U);
    CHECK(!jog.reset_seen);
    FlywheelJog_Step(&jog, 101U, 1U, 0U, 1U, 0U);
    CHECK(!jog.ready);
    FlywheelJog_Step(&jog, 102U, 1U, 1U, 0U, 0U);
    CHECK(!jog.active && jog.current_mA == 0);
    FlywheelJog_Step(&jog, 103U, 0U, 0U, 0U, 0U); /* 负向不是回中。 */
    FlywheelJog_Step(&jog, 104U, 1U, 0U, 1U, 0U);
    CHECK(!jog.ready);
    FlywheelJog_Step(&jog, 105U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 106U, 1U, 0U, 1U, 0U);
    CHECK(jog.ready);
    FlywheelJog_Step(&jog, 107U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.current_mA == 0);
    commands(0, 0);
    FlywheelJog_Step(&jog, 127U, 1U, 1U, 0U, 0U);
    CHECK(jog.current_mA == 100); commands(100, 0);
    FlywheelJog_Step(&jog, 147U, 1U, 1U, 0U, 0U);
    CHECK(jog.current_mA == 200); commands(200, 0);
    CHECK(!FlywheelJog_Select(&jog, FLYWHEEL_JOG_RIGHT, 147U, 0U));
    f = frame(70U, 9U, 148U, 1000, 2, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 148U, 0U));
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_OFF, 148U, 0U));
    CHECK(jog.selection == FLYWHEEL_JOG_OFF && !jog.active && !jog.ready);
    commands(0, 0);
    FlywheelJog_Step(&jog, 149U, 0U, 0U, 1U, 0U);
    CHECK(!FlywheelJog_Select(&jog, FLYWHEEL_JOG_RIGHT, 149U, 0U));

    test_stage = 4U;
    prepare(FLYWHEEL_JOG_RIGHT, 100U);
    FlywheelJog_Step(&jog, 123U, 1U, 1U, 0U, 0U); commands(0, 100);
    FlywheelJog_Step(&jog, 125U, 1U, 0U, 1U, 0U);
    CHECK(!jog.active && !jog.ready && jog.current_mA == 0); commands(0, 0);
    FlywheelJog_Step(&jog, 130U, 1U, 1U, 0U, 0U);
    CHECK(!jog.active && !jog.ready);
    FlywheelJog_Step(&jog, 135U, 0U, 0U, 0U, 0U);
    FlywheelJog_Step(&jog, 136U, 1U, 0U, 1U, 0U);
    CHECK(!jog.ready);
    FlywheelJog_Step(&jog, 140U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 141U, 1U, 1U, 0U, 0U); /* ON时未回中不能起动。 */
    CHECK(!jog.ready && !jog.active);
    FlywheelJog_Step(&jog, 145U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 146U, 1U, 0U, 1U, 0U);
    CHECK(jog.ready);
    FlywheelJog_Step(&jog, 147U, 1U, 1U, 0U, 0U);
    CHECK(jog.active);
    FlywheelJog_Step(&jog, 167U, 1U, 1U, 0U, 0U);
    CHECK(jog.current_mA == 100);
    /* 无需先取走正命令；停机必须覆盖它。 */
    FlywheelJog_Step(&jog, 168U, 1U, 0U, 0U, 0U);
    CHECK(!jog.active && jog.current_mA == 0); commands(0, 0);
    FlywheelJog_Step(&jog, 170U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 171U, 1U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 172U, 1U, 1U, 0U, 0U);
    CHECK(jog.active);
    FlywheelJog_Step(&jog, 173U, 0U, 1U, 0U, 0U);
    CHECK(!jog.active && jog.current_mA == 0); commands(0, 0);

    test_stage = 5U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    f = frame(71U, 9U, 104U, -2000, -10, -1000);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 104U, 0U));
    CHECK(jog.faults == 0U);
    f = frame(71U, 16U, 104U, 599, -1000, 10);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 104U, 0U));
    CHECK(jog.faults == 0U && !jog.feedback[1].motor_temperature_valid);
    f = frame(71U, 9U, 105U, -2001, 0, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 105U, 0U));
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_OVERSPEED) != 0U);
    CHECK(!jog.active && jog.current_mA == 0); commands(0, 0);
    f = frame(71U, 9U, 106U, 0, 0, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 106U, 0U));
    FlywheelJog_Step(&jog, 107U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 108U, 1U, 0U, 1U, 0U);
    CHECK(!jog.ready && !jog.active && jog.faults);
    CHECK(!FlywheelJog_ClearFault(&jog, 108U, 0U));
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_OFF, 108U, 0U));
    FlywheelJog_Step(&jog, 109U, 0U, 0U, 1U, 0U);
    CHECK(FlywheelJog_ClearFault(&jog, 109U, 0U));
    CHECK(jog.faults == 0U && !jog.ready && !jog.reset_seen);
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 109U, 0U));
    f = frame(70U, 16U, 110U, 600, 250, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 110U, 0U));
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_TEMPERATURE) != 0U); commands(0, 0);
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    f = frame(70U, 9U, 104U, 0, 11, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 104U, 0U));
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_CURRENT) != 0U); commands(0, 0);
    prepare(FLYWHEEL_JOG_RIGHT, 100U);
    f = frame(71U, 9U, 104U, 0, -11, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 104U, 0U));
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_CURRENT) != 0U); commands(0, 0);
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    f = frame(71U, 16U, 104U, 250, 250, -11);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 104U, 0U));
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_CURRENT) != 0U); commands(0, 0);

    test_stage = 6U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    /* 只补左轮，右轮过期也必须停机。 */
    f = frame(70U, 9U, 201U, 0, 0, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 201U, 0U));
    FlywheelJog_Step(&jog, 201U, 1U, 1U, 0U, 0U);
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_FEEDBACK) != 0U);
    CHECK(!jog.active && jog.current_mA == 0); commands(0, 0);
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_OFF, 201U, 0U));
    FlywheelJog_Step(&jog, 202U, 0U, 0U, 1U, 0U);
    CHECK(!FlywheelJog_ClearFault(&jog, 202U, 0U));
    feedback(203U);
    CHECK(FlywheelJog_ClearFault(&jog, 203U, 0U));
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    f = frame(70U, 9U, 104U, 0, 0, 0); f.dlc = 9U;
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 104U, 0U));
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_FRAME) != 0U); commands(0, 0);

    test_stage = 7U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    for (time = 123U; time < 603U; time += 20U) {
        feedback(time);
        FlywheelJog_Step(&jog, time, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.current_mA <= 500);
        commands(time == 123U ? 100 : (time == 143U ? 200 :
                 (time == 163U ? 300 : (time == 183U ? 400 : 500))), 0);
    }
    feedback(602U);
    FlywheelJog_Step(&jog, 602U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.faults == 0U);
    feedback(603U);
    FlywheelJog_Step(&jog, 603U, 1U, 1U, 0U, 0U);
#if FLYWHEEL_JOG_MAX_DURATION_MS > 0U
    CHECK(!jog.active && !jog.ready && jog.current_mA == 0 && jog.faults == 0U);
    commands(0, 0);
    feedback(623U);
    FlywheelJog_Step(&jog, 623U, 1U, 1U, 0U, 0U);
    CHECK(!jog.active && jog.current_mA == 0); commands(0, 0);
#else
    CHECK(jog.active && jog.current_mA == 500 && !jog.faults);
    commands(500, 0);
    for (time = 623U; time <= 2103U; time += 20U) {
        feedback(time);
        FlywheelJog_Step(&jog, time, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.current_mA == 500 && !jog.faults);
        commands(500, 0);
    }
    /* 持续两秒后回中仍停止；再推正向不能自动重新起动。 */
    FlywheelJog_Step(&jog, 2104U, 1U, 0U, 1U, 0U);
    CHECK(!jog.active && !jog.ready && jog.current_mA == 0); commands(0, 0);
    FlywheelJog_Step(&jog, 2105U, 1U, 1U, 0U, 0U);
    CHECK(!jog.active && jog.current_mA == 0);
    FlywheelJog_Step(&jog, 2106U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 2107U, 1U, 0U, 1U, 0U);
    CHECK(jog.ready && !jog.active);
    FlywheelJog_Step(&jog, 2108U, 1U, 1U, 0U, 0U);
    feedback(2128U);
    FlywheelJog_Step(&jog, 2128U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.current_mA == 100); commands(100, 0);
#endif

    test_stage = 8U;
    prepare(FLYWHEEL_JOG_RIGHT, 0xFFFFFFE0UL);
    feedback(0xFFFFFFF7UL);
    FlywheelJog_Step(&jog, 0xFFFFFFF7UL, 1U, 1U, 0U, 0U);
    CHECK(jog.current_mA == 100); commands(0, 100);
    feedback(11U);
    FlywheelJog_Step(&jog, 11U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.current_mA == 200 && jog.faults == 0U);
    commands(0, 200);
    f = frame(71U, 9U, 0xFFFFFFF8UL, 0, 0, 0);
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 11U, 0U));
    CHECK(jog.feedback[1].status1_ms == 11U);
    feedback(479U);
    FlywheelJog_Step(&jog, 479U, 1U, 1U, 0U, 0U);
#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U
    CHECK(jog.active && jog.current_mA == 500); commands(0, 500);
    FlywheelJog_Step(&jog, 480U, 0U, 0U, 1U, 0U);
#endif
    CHECK(!jog.active && jog.current_mA == 0); commands(0, 0);

    test_stage = 9U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    FlywheelJog_Step(&jog, 104U, 1U, 1U, 0U, 1U);
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_CAN_EPOCH) != 0U);
    CHECK(!jog.active && !jog.feedback[0].has_status1); commands(0, 0);
    f = frame(70U, 9U, 105U, 0, 0, 0);
    CHECK(!FlywheelJog_OnFrame(&jog, &f, 105U, 1U));
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_OFF, 105U, 1U));
    feedback(106U); /* epoch即便错误地归0也不允许现场清锁。 */
    FlywheelJog_Step(&jog, 107U, 0U, 0U, 1U, 0U);
    CHECK(!FlywheelJog_ClearFault(&jog, 107U, 0U));
    CHECK(!FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 107U, 0U));
    FlywheelJog_Init(&jog);
    FlywheelJog_GetHealth(&jog, 100U, 1U, &health);
    CHECK(!health.valid && (jog.faults & FLYWHEEL_JOG_FAULT_CAN_EPOCH));
    CHECK(jog.pending_mask == 0U);
    FlywheelJog_Step(&jog, 101U, 0U, 0U, 1U, 1U);
    CHECK(jog.pending_mask == 0U);

    test_stage = 10U;
    prepare(FLYWHEEL_JOG_RIGHT, 100U);
    FlywheelJog_Step(&jog, 123U, 1U, 1U, 0U, 0U);
    FlywheelJog_Fault(&jog, FLYWHEEL_JOG_FAULT_EXTERNAL);
    CHECK(jog.current_mA == 0 && !jog.active && jog.faults); commands(0, 0);
    FlywheelJog_Step(&jog, 143U, 1U, 1U, 0U, 0U);
    commands(0, 0);
    FlywheelJog_Fault(&jog, 0U);
    CHECK(jog.faults == FLYWHEEL_JOG_FAULT_EXTERNAL);

    test_stage = 11U;
    FlywheelJog_Init(&jog); feedback(100U);
    for (node = 70U; node <= 71U; ++node) {
        f = frame(node, 9U, 600U, 0, 0, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, 600U, 0U));
    }
    FlywheelJog_Step(&jog, 600U, 0U, 0U, 1U, 0U);
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 600U, 0U));
    FlywheelJog_Step(&jog, 601U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 602U, 1U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 603U, 1U, 1U, 0U, 0U);
    CHECK(jog.active); commands(0, 0);
    for (node = 70U; node <= 71U; ++node) {
        f = frame(node, 9U, 700U, 0, 0, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, 700U, 0U));
    }
    FlywheelJog_Step(&jog, 700U, 1U, 1U, 0U, 0U);
    CHECK(jog.active && jog.current_mA == 485 && jog.faults == 0U); commands(485, 0);
    FlywheelJog_Step(&jog, 701U, 1U, 1U, 0U, 0U);
    CHECK(!jog.active && jog.current_mA == 0);
    CHECK((jog.faults & FLYWHEEL_JOG_FAULT_FEEDBACK) != 0U); commands(0, 0);

    test_stage = 12U;
    FlywheelJog_Init(&jog); feedback(100U);
    FlywheelJog_Step(&jog, 100U, 0U, 0U, 1U, 0U);
    f = frame(71U, 9U, 101U, 100, 0, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 101U, 0U));
    CHECK(!FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 101U, 0U));
    f = frame(71U, 9U, 102U, -100, 0, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 102U, 0U));
    CHECK(!FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 102U, 0U));
    f = frame(71U, 9U, 103U, -99, 0, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 103U, 0U));
    CHECK(FlywheelJog_Select(&jog, FLYWHEEL_JOG_LEFT, 103U, 0U));
    FlywheelJog_Step(&jog, 104U, 0U, 0U, 1U, 0U);
    FlywheelJog_Step(&jog, 105U, 1U, 0U, 1U, 0U);
    CHECK(jog.ready);
    f = frame(71U, 9U, 106U, 100, 0, 0);
    CHECK(FlywheelJog_OnFrame(&jog, &f, 106U, 0U));
    FlywheelJog_Step(&jog, 107U, 1U, 1U, 0U, 0U);
    CHECK(!jog.active && !jog.ready && jog.current_mA == 0);
    commands(0, 0);
#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U
    test_stage = 13U;
    for (node = 70U; node <= 71U; ++node) {
        prepare(node == 70U ? FLYWHEEL_JOG_LEFT : FLYWHEEL_JOG_RIGHT, 100U);
        observation_feedback(203U, node, 0);
        FlywheelJog_Step(&jog, 203U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && !jog.coasting && jog.current_mA == 500);
        commands(node == 70U ? 500 : 0, node == 71U ? 500 : 0);
        observation_feedback(223U, node, 999);
        FlywheelJog_Step(&jog, 223U, 1U, 1U, 0U, 0U);
        CHECK(!jog.coasting && jog.current_mA == 500);
        commands(node == 70U ? 500 : 0, node == 71U ? 500 : 0);
        observation_feedback(243U, node, 1000);
        FlywheelJog_Step(&jog, 243U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.coasting && !jog.ready && !jog.faults && jog.current_mA == 0);
        commands(0, 0);
        observation_feedback(263U, node, 1001);
        FlywheelJog_Step(&jog, 263U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.coasting && !jog.faults); commands(0, 0);
        observation_feedback(283U, node, 601);
        FlywheelJog_Step(&jog, 283U, 1U, 1U, 0U, 0U);
        CHECK(jog.coasting && jog.current_mA == 0); commands(0, 0);
        observation_feedback(303U, node, 600);
        FlywheelJog_Step(&jog, 303U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && !jog.coasting && jog.current_mA == 100);
        commands(node == 70U ? 100 : 0, node == 71U ? 100 : 0);
        observation_feedback(323U, node, -1000);
        FlywheelJog_Step(&jog, 323U, 1U, 1U, 0U, 0U);
        CHECK(jog.coasting && jog.current_mA == 0); commands(0, 0);
        observation_feedback(343U, node, -601);
        FlywheelJog_Step(&jog, 343U, 1U, 1U, 0U, 0U);
        CHECK(jog.coasting && jog.current_mA == 0); commands(0, 0);
        observation_feedback(363U, node, -600);
        FlywheelJog_Step(&jog, 363U, 1U, 1U, 0U, 0U);
        CHECK(!jog.coasting && jog.current_mA == 100);
        commands(node == 70U ? 100 : 0, node == 71U ? 100 : 0);
        observation_feedback(383U, node, -599);
        FlywheelJog_Step(&jog, 383U, 1U, 1U, 0U, 0U);
        CHECK(jog.current_mA == 200);
        commands(node == 70U ? 200 : 0, node == 71U ? 200 : 0);
        observation_feedback(403U, node, 0);
        f = frame(node == 70U ? 71U : 70U, 9U, 404U, 1500, 0, 0);
        CHECK(FlywheelJog_OnFrame(&jog, &f, 404U, 0U));
        FlywheelJog_Step(&jog, 404U, 1U, 1U, 0U, 0U);
        CHECK(!jog.coasting && jog.current_mA == 305); /* 只调节所选轮。 */
        commands(node == 70U ? 305 : 0, node == 71U ? 305 : 0);
        observation_feedback(423U, node, 1000);
        FlywheelJog_Step(&jog, 424U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.coasting && !jog.faults); commands(0, 0);
        FlywheelJog_Step(&jog, 524U, 1U, 1U, 0U, 0U);
        CHECK(!jog.active && !jog.coasting && jog.current_mA == 0);
        CHECK((jog.faults & FLYWHEEL_JOG_FAULT_FEEDBACK) != 0U); commands(0, 0);
    }

    test_stage = 14U;
    for (node = 70U; node <= 71U; ++node) {
        prepare(node == 70U ? FLYWHEEL_JOG_LEFT : FLYWHEEL_JOG_RIGHT, 100U);
        observation_feedback(203U, node, 1000);
        FlywheelJog_Step(&jog, 203U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.coasting && !jog.faults); commands(0, 0);
        observation_feedback(223U, node, 2000);
        FlywheelJog_Step(&jog, 223U, 1U, 1U, 0U, 0U);
        CHECK(jog.active && jog.coasting && !jog.faults); commands(0, 0);
        observation_feedback(243U, node, 2001);
        CHECK(!jog.active && !jog.coasting && jog.current_mA == 0);
        CHECK((jog.faults & FLYWHEEL_JOG_FAULT_OVERSPEED) != 0U); commands(0, 0);
    }
#endif
    test_done = 1U;
}
