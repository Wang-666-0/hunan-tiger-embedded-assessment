#include "motor_backend.h"
#include "can_bus.h"
#include "app_config.h"
#include <string.h>
#ifndef TEST_DUAL
#define TEST_DUAL 0
#endif
#ifndef TEST_HIGH_SPEED
#define TEST_HIGH_SPEED 0
#endif
#define TEST_DUAL_MAX (TEST_HIGH_SPEED ? 2000 : 1000)
#define TEST_DUAL_TARGET (TEST_HIGH_SPEED ? 42000 : 7000)
#define TEST_DUAL_SOFT (TEST_HIGH_SPEED ? 44100 : 8000)
#define TEST_DUAL_HARD (TEST_HIGH_SPEED ? 46200 : 9000)

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

typedef struct {
    uint32_t time;
    uint8_t node, accepted;
    int32_t current;
} JogSubmission;

static CanBusState bus;
static MotorJogState state;
static MotorHealth health;
static TargetRequest request;
static JogSubmission submissions[2048];
static uint32_t now_ms, normal_calls, nonzero_submissions, stop_calls;
static uint32_t last_stop_time, fail_normal_call, cancel_calls, delivered_stop_frames;
static uint32_t keep_calls, last_keep_time;
static uint8_t permission, fail_stop, reads_in_flight;
static HAL_StatusTypeDef keep_result;

uint32_t HAL_GetTick(void) { return now_ms; }
void CanBus_GetState(CanBusState *out) { if (out != 0) { *out = bus; } }
uint8_t CanBus_OutputAllowed(void) { return permission; }
void CanBus_SetOutputPermission(uint8_t allowed)
{
    permission = (uint8_t)(allowed && bus.started && bus.fault_epoch == 0U);
}

HAL_StatusTypeDef CanBus_JogVescCurrent(uint8_t node, int32_t current)
{
    uint32_t index = normal_calls++;
    uint8_t accepted = (uint8_t)(permission && bus.started && bus.fault_epoch == 0U &&
        ((node == 70U && current >= 0 && current <= APP_FLYWHEEL_JOG_LEFT_MAX_MA) ||
         (node == 71U && current >= 0 && current <= APP_FLYWHEEL_JOG_RIGHT_MAX_MA)) &&
        normal_calls != fail_normal_call);
    CHECK(index < 2048U);
    if (index < 2048U) {
        submissions[index].time = now_ms;
        submissions[index].node = node;
        submissions[index].current = current;
        submissions[index].accepted = accepted;
    }
    if (accepted && current != 0) { ++nonzero_submissions; }
    /* 单节点0mA仍是单节点命令；不得撤销另一轮许可。 */
    return accepted ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef CanBus_StopFlywheels(void)
{
    ++stop_calls; last_stop_time = now_ms;
    permission = 0U;
    if (fail_stop || !bus.started || bus.fault_epoch != 0U) { return HAL_BUSY; }
    reads_in_flight = 0U; /* 首次严格停止可取消只读请求。 */
    delivered_stop_frames += 2U;
    return HAL_OK;
}

HAL_StatusTypeDef CanBus_KeepFlywheelsStopped(void)
{
    ++keep_calls; last_keep_time = now_ms;
    permission = 0U;
    if (keep_result != HAL_OK) { return keep_result; }
    if (reads_in_flight) { return HAL_BUSY; }
    if (!bus.started || bus.fault_epoch != 0U) { return HAL_ERROR; }
    delivered_stop_frames += 2U;
    return HAL_OK;
}

void CanBus_CancelReads(void)
{
    ++cancel_calls; permission = 0U;
    reads_in_flight = 0U;
    /* 模拟DIAG结束取消了刚提交到邮箱、尚未发完的两个0mA。 */
    delivered_stop_frames = 0U;
}

static void put16(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw >> 8); data[1] = (uint8_t)raw;
}

static void put32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw >> 24); data[1] = (uint8_t)(raw >> 16);
    data[2] = (uint8_t)(raw >> 8); data[3] = (uint8_t)raw;
}

static CanFrame frame(uint8_t node, uint32_t packet, uint32_t timestamp, int32_t value)
{
    CanFrame result;
    memset(&result, 0, sizeof(result));
    result.bus = 2U; result.extended = 1U; result.dlc = 8U;
    result.id = (packet << 8) | node; result.received_ms = timestamp;
    result.fault_epoch = bus.fault_epoch;
    if (packet == 9U) { put32(result.data, value); }
    else { put16(result.data, value); put16(result.data + 2U, -100); }
    return result;
}

static void feedback(uint32_t timestamp)
{
    uint8_t node;
    now_ms = timestamp;
    for (node = 70U; node <= 71U; ++node) {
        CanFrame f = frame(node, 9U, timestamp, 0);
        CHECK(MotorBackend_OnFrame(&f));
        f = frame(node, 16U, timestamp, 250);
        CHECK(MotorBackend_OnFrame(&f));
    }
}

static void read_state(void)
{
    MotorBackend_GetJogState(&state);
    MotorBackend_GetHealth(&health);
    CHECK(health.flywheels_ready == 0U); /* 从不授权拨盘。 */
}

static uint8_t apply(uint32_t timestamp, float yaw, uint8_t allowed)
{
    now_ms = timestamp; request.yaw_rate_dps = yaw;
    CanBus_SetOutputPermission(allowed); /* 模拟控制任务最终许可。 */
    return MotorBackend_Apply(&request, allowed);
}

static void reset(uint32_t timestamp)
{
    memset(&bus, 0, sizeof(bus)); memset(&request, 0, sizeof(request));
    memset(submissions, 0, sizeof(submissions));
    bus.started = 1U; now_ms = timestamp;
    normal_calls = nonzero_submissions = stop_calls = last_stop_time = 0U;
    fail_normal_call = cancel_calls = delivered_stop_frames = 0U;
    keep_calls = last_keep_time = 0U;
    permission = fail_stop = reads_in_flight = 0U; keep_result = HAL_OK;
    MotorBackend_Init();
}

static void pair(uint32_t index, int32_t left, int32_t right)
{
    CHECK(normal_calls >= index + 2U);
    CHECK(submissions[index].node == 70U && submissions[index].current == left);
    CHECK(submissions[index + 1U].node == 71U && submissions[index + 1U].current == right);
    CHECK(submissions[index].accepted && submissions[index + 1U].accepted);
}

static void prepare(FlywheelJogSelection selection, uint32_t timestamp)
{
    reset(timestamp); feedback(timestamp);
    CHECK(!apply(timestamp, 0.0f, 0U));
    CHECK(MotorBackend_SelectJog(selection));
    read_state();
    CHECK(state.selection == selection && state.stop_pending);
    CHECK(!health.ready && health.feedback_ready && !state.active);
    CHECK(!apply(timestamp + 1U, 0.0f, 0U));
    read_state();
    CHECK(!state.stop_pending && stop_calls == 1U && delivered_stop_frames == 2U);
    CHECK(!permission && normal_calls == 0U);
    CHECK(!apply(timestamp + 2U, 0.0f, 0U));
    CHECK(!apply(timestamp + 3U, 0.0f, 1U));
    read_state(); CHECK(state.rearm_ready && !state.active);
    CHECK(apply(timestamp + 4U, 1.0f, 1U));
    pair(0U, 0, 0);
#if APP_FLYWHEEL_SPEED_ACTIVE
    feedback(timestamp + 24U);
    CHECK(apply(timestamp + 24U, 1.0f, 1U));
    pair(2U, selection != FLYWHEEL_JOG_RIGHT ? (TEST_HIGH_SPEED ? 100 : (TEST_DUAL ? 8 : 3)) : 0,
              selection != FLYWHEEL_JOG_LEFT ? (TEST_HIGH_SPEED ? 100 : (TEST_DUAL ? 8 : 3)) : 0);
    feedback(timestamp + 44U);
    CHECK(apply(timestamp + 44U, 1.0f, 1U));
    pair(4U, selection != FLYWHEEL_JOG_RIGHT ? (TEST_HIGH_SPEED ? 200 : (TEST_DUAL ? 17 : 7)) : 0,
              selection != FLYWHEEL_JOG_LEFT ? (TEST_HIGH_SPEED ? 200 : (TEST_DUAL ? 17 : 7)) : 0);
    read_state();
    CHECK(state.active && state.requested_mA == (TEST_HIGH_SPEED ? 200 : (TEST_DUAL ? 17 : 7)) && !state.stop_pending);
#else
    CHECK(apply(timestamp + 24U, 1.0f, 1U));
    pair(2U, selection == FLYWHEEL_JOG_LEFT ? 100 : 0,
              selection == FLYWHEEL_JOG_RIGHT ? 100 : 0);
    CHECK(apply(timestamp + 44U, 1.0f, 1U));
    pair(4U, selection == FLYWHEEL_JOG_LEFT ? 200 : 0,
              selection == FLYWHEEL_JOG_RIGHT ? 200 : 0);
    read_state();
    CHECK(state.active && state.requested_mA == 200 && !state.stop_pending);
#endif
    CHECK(permission && stop_calls == 1U && !MotorBackend_TakeRearmRequest());
}

#if TEST_DUAL
void test_run(void)
{
    uint32_t time, calls, nonzero, stops;
    CanFrame f;
    test_stage = 1U;
    prepare(FLYWHEEL_JOG_BOTH, 100U);
    CHECK(state.requested_left_mA == (TEST_HIGH_SPEED ? 200 : 17) && state.requested_right_mA == (TEST_HIGH_SPEED ? 200 : 17));
    CHECK(state.speed.running && state.speed_right.running);
    /* 右轮软滑行只改变右轮，左轮仍继续闭环。 */
    feedback(164U); now_ms = 165U;
    f = frame(71U, 9U, now_ms, TEST_DUAL_SOFT); CHECK(MotorBackend_OnFrame(&f));
    CHECK(apply(165U, 1.0f, 1U)); read_state();
    CHECK(state.active && !state.stop_pending && permission);
    CHECK(state.requested_left_mA > 0 && state.requested_right_mA == 0);
    pair(6U, state.requested_left_mA, 0);
    CHECK(state.speed.peak_erpm == 0U && state.speed_right.peak_erpm == (uint32_t)TEST_DUAL_SOFT);
    for (time = 185U; time <= 9185U; time += 20U) {
        feedback(time); CHECK(apply(time, 1.0f, 1U)); read_state();
        CHECK(state.active && !state.health.faults && state.stop_reason == FLYWHEEL_STOP_NONE);
        CHECK(state.requested_left_mA >= 0 && state.requested_left_mA <= TEST_DUAL_MAX);
        CHECK(state.requested_right_mA >= 0 && state.requested_right_mA <= TEST_DUAL_MAX);
        pair(normal_calls - 2U, state.requested_left_mA, state.requested_right_mA);
    }
    CHECK(state.speed.target_erpm == (float)TEST_DUAL_TARGET && state.speed_right.target_erpm == (float)TEST_DUAL_TARGET);
    CHECK(state.requested_left_mA == TEST_DUAL_MAX && state.requested_right_mA == TEST_DUAL_MAX);
    CHECK(state.speed.low_speed_ms > 0U && state.speed_right.low_speed_ms > 0U);
    calls = normal_calls; nonzero = nonzero_submissions; stops = stop_calls;
    fail_stop = 1U; CHECK(!apply(9186U, 0.0f, 1U)); read_state();
    CHECK(!permission && !state.active && !state.speed.running && !state.speed_right.running);
    CHECK(state.stop_reason == FLYWHEEL_STOP_INPUT && state.stop_pending);
    CHECK(normal_calls == calls && stop_calls == stops + 1U);
    CHECK(state.requested_left_mA == 0 && state.requested_right_mA == 0);
    CHECK(!apply(9205U, 1.0f, 1U)); CHECK(stop_calls == stops + 1U);
    fail_stop = 0U; feedback(9206U); CHECK(!apply(9206U, 1.0f, 1U)); read_state();
    CHECK(!state.active && !state.stop_pending && stop_calls == stops + 2U);
    feedback(9226U); (void)apply(9226U, 1.0f, 1U); read_state();
    CHECK(!state.active && nonzero_submissions == nonzero);
    CHECK(state.stop_reason == FLYWHEEL_STOP_INPUT);

    test_stage = 2U;
    prepare(FLYWHEEL_JOG_BOTH, 100U);
    now_ms = 145U; f = frame(71U, 9U, now_ms, -TEST_DUAL_HARD - 1);
    CHECK(MotorBackend_OnFrame(&f)); read_state();
    CHECK(!permission && !state.active && state.stop_pending && state.stop_reason == FLYWHEEL_STOP_FAULT);
    CHECK(state.speed_right.peak_erpm == (uint32_t)TEST_DUAL_HARD + 1U && state.health.faults == FLYWHEEL_JOG_FAULT_OVERSPEED);
    CHECK(state.requested_left_mA == 0 && state.requested_right_mA == 0);
    CHECK(MotorBackend_TakeRearmRequest()); CHECK(!apply(146U, 1.0f, 1U));
    CHECK(stop_calls == 2U);

    test_stage = 3U;
    prepare(FLYWHEEL_JOG_BOTH, 100U); feedback(164U);
    fail_normal_call = normal_calls + 2U; stops = stop_calls;
    CHECK(!apply(164U, 1.0f, 1U)); read_state();
    CHECK(!permission && !state.active && !state.speed.running && !state.speed_right.running);
    CHECK(state.stop_reason == FLYWHEEL_STOP_FAULT &&
          (state.health.faults & FLYWHEEL_JOG_FAULT_EXTERNAL));
    CHECK(state.requested_left_mA == 0 && state.requested_right_mA == 0);
    CHECK(stop_calls == stops + 1U && MotorBackend_TakeRearmRequest());

    test_stage = 4U;
    prepare(FLYWHEEL_JOG_BOTH, 100U); MotorBackend_Stop();
    read_state(); CHECK(state.stop_reason == FLYWHEEL_STOP_MANUAL && !state.active);
    CHECK(state.speed.integral_mA == 0.0f && state.speed_right.integral_mA == 0.0f);
    CHECK(!apply(145U, 1.0f, 1U)); read_state();
    CHECK(state.stop_reason == FLYWHEEL_STOP_MANUAL);
    test_done = 1U;
}
#elif !APP_FLYWHEEL_SPEED_ACTIVE
void test_run(void)
{
    uint32_t calls, nonzero, stops, time;
    CanFrame f;
    test_stage = 1U;
    reset(100U);
    read_state(); CHECK(state.selection == FLYWHEEL_JOG_OFF);
    CHECK(!health.ready && !health.feedback_ready && !health.actuator_fault);
    CHECK(!apply(100U, 0.0f, 0U));
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    MotorBackend_Stop();
    CHECK(!MotorBackend_Apply(0, 0U));
    feedback(101U);
    read_state(); CHECK(health.feedback_ready && !health.ready && !health.actuator_fault);
    request.flywheel_rpm = 6000.0f; request.shot_event = 1U; request.friction_requested = 1U;
    CHECK(!apply(102U, 1.0f, 1U));
    CHECK(!apply(103U, 0.0f, 0U));
    CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_OFF));
    CHECK(!apply(104U, 0.0f, 0U));
    CHECK(normal_calls == 0U && stop_calls == 0U && !MotorBackend_TakeRearmRequest());
    CHECK(!MotorBackend_OnFrame(0));
    MotorBackend_GetJogState(0); MotorBackend_GetHealth(0);

    test_stage = 2U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_RIGHT));
    CHECK(normal_calls == 6U && nonzero_submissions == 2U);
    CHECK(submissions[4].node == 70U && submissions[4].current == 200);
    CHECK(submissions[5].node == 71U && submissions[5].current == 0);
    CHECK(permission && state.active); /* 未选轮0不会取消被选轮。 */
    feedback(164U); CHECK(apply(164U, 1.0f, 1U)); pair(6U, 300, 0);
    read_state(); CHECK(state.active && state.requested_mA == 300);
    feedback(184U); CHECK(apply(184U, 1.0f, 1U)); pair(8U, 400, 0);
    read_state(); CHECK(state.active && state.requested_mA == 400);
    feedback(204U); CHECK(apply(204U, 1.0f, 1U)); pair(10U, 500, 0);
    read_state(); CHECK(state.active && state.requested_mA == 500);
    feedback(224U); CHECK(apply(224U, 1.0f, 1U)); pair(12U, 500, 0);
    prepare(FLYWHEEL_JOG_RIGHT, 200U);
    CHECK(submissions[4].node == 70U && submissions[4].current == 0);
    CHECK(submissions[5].node == 71U && submissions[5].current == 200);
    CHECK(permission && state.active && nonzero_submissions == 2U);
    feedback(264U); CHECK(apply(264U, 1.0f, 1U)); pair(6U, 0, 300);
    read_state(); CHECK(state.active && state.requested_mA == 300);
    feedback(284U); CHECK(apply(284U, 1.0f, 1U)); pair(8U, 0, 400);
    feedback(304U); CHECK(apply(304U, 1.0f, 1U)); pair(10U, 0, 500);
    read_state(); CHECK(state.active && state.requested_mA == 500);
    feedback(324U); CHECK(apply(324U, 1.0f, 1U)); pair(12U, 0, 500);

    test_stage = 3U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    calls = normal_calls; nonzero = nonzero_submissions; stops = stop_calls;
    CHECK(!apply(145U, 0.0f, 1U));
    read_state(); CHECK(!state.active && state.requested_mA == 0 && !state.stop_pending);
    CHECK(stop_calls == stops + 1U && normal_calls == calls && !permission);
    CHECK(MotorBackend_TakeRearmRequest());
    CHECK(!MotorBackend_TakeRearmRequest());
    (void)apply(146U, 1.0f, 1U);
    read_state(); CHECK(!state.active && !state.rearm_ready);
    CHECK(nonzero_submissions == nonzero);
    (void)apply(147U, -1.0f, 0U);
    (void)apply(148U, 0.0f, 1U);
    read_state(); CHECK(!state.rearm_ready && !state.active);
    (void)apply(149U, 0.0f, 0U);
    (void)apply(150U, 0.0f, 1U);
    read_state(); CHECK(state.rearm_ready);
    CHECK(apply(151U, 1.0f, 1U));
    CHECK(apply(171U, 1.0f, 1U));
    read_state(); CHECK(state.active && state.requested_mA == 100);
    CHECK(nonzero_submissions > nonzero);
    nonzero = nonzero_submissions; calls = normal_calls;
    CHECK(!apply(172U, -1.0f, 1U));
    CHECK(MotorBackend_TakeRearmRequest());
    read_state(); CHECK(!state.active && state.requested_mA == 0);
    (void)apply(173U, 1.0f, 1U);
    CHECK(nonzero_submissions == nonzero && normal_calls == calls);

    test_stage = 4U;
    prepare(FLYWHEEL_JOG_RIGHT, 100U);
    for (time = 164U; time <= 584U; time += 20U) {
        feedback(time); CHECK(apply(time, 1.0f, 1U));
        read_state(); CHECK(state.active && state.requested_mA ==
            (time == 164U ? 300 : (time == 184U ? 400 : 500)));
    }
    feedback(603U); (void)apply(603U, 1.0f, 1U);
    read_state(); CHECK(state.active && !MotorBackend_TakeRearmRequest());
#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U
    /* 正式观察档需在原500ms时限之后维持给定，且回中仍可靠撤权。 */
    for (time = 604U; time <= 2104U; time += 20U) {
        feedback(time); CHECK(apply(time, 1.0f, 1U));
        read_state(); CHECK(state.active && state.requested_mA == 500);
        CHECK(!MotorBackend_TakeRearmRequest());
    }
    nonzero = nonzero_submissions; calls = normal_calls; stops = stop_calls;
    CHECK(!apply(2105U, 0.0f, 1U));
    read_state(); CHECK(!state.active && state.requested_mA == 0);
    CHECK(normal_calls == calls && nonzero_submissions == nonzero && stop_calls == stops + 1U);
    CHECK(MotorBackend_TakeRearmRequest() && !MotorBackend_TakeRearmRequest());
    (void)apply(2106U, 1.0f, 1U);
    read_state(); CHECK(!state.active && nonzero_submissions == nonzero);
#else
    nonzero = nonzero_submissions; calls = normal_calls; stops = stop_calls;
    feedback(604U); CHECK(!apply(604U, 1.0f, 1U));
    read_state(); CHECK(!state.active && state.requested_mA == 0);
    CHECK(normal_calls == calls && nonzero_submissions == nonzero && stop_calls == stops + 1U);
    CHECK(MotorBackend_TakeRearmRequest() && !MotorBackend_TakeRearmRequest());
    feedback(624U); (void)apply(624U, 1.0f, 1U);
    read_state(); CHECK(!state.active && nonzero_submissions == nonzero);
#endif

    test_stage = 5U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    calls = normal_calls; nonzero = nonzero_submissions; stops = stop_calls;
    now_ms = 145U; fail_stop = 1U; MotorBackend_Stop();
    CHECK(!apply(145U, 1.0f, 1U));
    read_state(); CHECK(state.stop_pending && !state.active && !health.ready);
    CHECK(stop_calls == stops + 1U && last_stop_time == 145U && !permission);
    CHECK(!apply(146U, 1.0f, 1U));
    CHECK(!apply(164U, 1.0f, 1U));
    CHECK(stop_calls == stops + 1U && normal_calls == calls);
    CHECK(!apply(165U, 1.0f, 1U));
    CHECK(stop_calls == stops + 2U && last_stop_time == 165U);
    fail_stop = 0U;
    CHECK(!apply(185U, 1.0f, 1U));
    read_state(); CHECK(!state.stop_pending && !state.active && !permission);
    (void)apply(186U, 1.0f, 1U);
    read_state(); CHECK(!state.active && !state.rearm_ready && nonzero_submissions == nonzero);
    (void)apply(187U, 0.0f, 0U); (void)apply(188U, 0.0f, 1U);
    read_state(); CHECK(state.rearm_ready);
    CHECK(apply(189U, 1.0f, 1U));
    feedback(209U); CHECK(apply(209U, 1.0f, 1U));
    CHECK(nonzero_submissions > nonzero);

    test_stage = 6U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    now_ms = 145U;
    CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_OFF));
    CHECK(!apply(146U, 0.0f, 0U));
    read_state(); CHECK(state.selection == FLYWHEEL_JOG_OFF && !state.stop_pending);
    stops = stop_calls; calls = normal_calls;
    now_ms = 147U; CanBus_CancelReads();
    CHECK(cancel_calls == 1U && delivered_stop_frames == 0U);
    CHECK(!apply(147U, 0.0f, 0U));
    CHECK(!apply(165U, 0.0f, 0U));
    CHECK(stop_calls == stops && normal_calls == calls);
    CHECK(!apply(166U, 0.0f, 0U));
    CHECK(stop_calls == stops && keep_calls == 1U && delivered_stop_frames == 2U);
    CHECK(!apply(186U, 0.0f, 0U));
    CHECK(stop_calls == stops && keep_calls == 2U && normal_calls == calls);
    CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_RIGHT));
    read_state(); CHECK(state.stop_pending && state.selection == FLYWHEEL_JOG_RIGHT);
    CHECK(!apply(187U, 0.0f, 0U));
    (void)apply(188U, 0.0f, 1U);
    read_state(); CHECK(state.rearm_ready && !state.active);

    test_stage = 7U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    fail_normal_call = normal_calls + 1U; fail_stop = 1U;
    calls = normal_calls; stops = stop_calls; nonzero = nonzero_submissions;
    CHECK(!apply(164U, 1.0f, 1U));
    read_state();
    CHECK(!state.active && state.stop_pending && state.requested_mA == 0 && !permission);
    CHECK((state.health.faults & FLYWHEEL_JOG_FAULT_EXTERNAL) != 0U);
    CHECK(normal_calls == calls + 1U && !submissions[calls].accepted);
    CHECK(stop_calls == stops + 1U && last_stop_time == 164U);
    CHECK(MotorBackend_TakeRearmRequest() && !MotorBackend_TakeRearmRequest());
    CHECK(!apply(165U, 1.0f, 1U));
    CHECK(!apply(183U, 1.0f, 1U));
    CHECK(stop_calls == stops + 1U);
    CHECK(!apply(184U, 1.0f, 1U));
    CHECK(stop_calls == stops + 2U && normal_calls == calls + 1U);
    fail_stop = 0U;
    feedback(204U); CHECK(!apply(204U, 1.0f, 1U));
    read_state(); CHECK(!state.stop_pending && state.health.faults);
    (void)apply(205U, 1.0f, 1U);
    CHECK(nonzero_submissions == nonzero);
    now_ms = 205U; CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_OFF));
    CHECK(!apply(206U, 0.0f, 0U)); feedback(206U);
    now_ms = 207U; CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    read_state(); CHECK(state.health.faults == 0U && !state.active);

    test_stage = 8U;
    prepare(FLYWHEEL_JOG_RIGHT, 100U);
    nonzero = nonzero_submissions; calls = normal_calls;
    now_ms = 145U; f = frame(70U, 9U, now_ms, 2001);
    CHECK(MotorBackend_OnFrame(&f));
    CHECK(!permission); /* 反馈故障回调立即撤权，不等待Apply。 */
    read_state(); CHECK(state.stop_pending && !state.active && health.actuator_fault);
    CHECK((state.health.faults & FLYWHEEL_JOG_FAULT_OVERSPEED) != 0U);
    CHECK(MotorBackend_TakeRearmRequest());
    CHECK(!apply(146U, 1.0f, 1U));
    CHECK(nonzero_submissions == nonzero && normal_calls == calls);
    read_state(); CHECK(!state.stop_pending && !state.active);

    test_stage = 9U;
    reset(100U); feedback(100U);
    CHECK(!apply(100U, 0.0f, 0U));
    bus.fault_epoch = 1U;
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    read_state(); CHECK(!state.health.valid && !health.ready && !health.actuator_fault);
    CHECK(!apply(101U, 0.0f, 0U));
    CHECK(normal_calls == 0U && stop_calls == 0U);
    bus.fault_epoch = 0U; feedback(102U);
    CHECK(!apply(103U, 0.0f, 0U));
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_RIGHT));
    CHECK(normal_calls == 0U && stop_calls == 0U);

    test_stage = 10U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    calls = normal_calls; nonzero = nonzero_submissions; stops = stop_calls;
    bus.fault_epoch = 1U;
    CHECK(!apply(145U, 1.0f, 1U));
    read_state(); CHECK(state.stop_pending && !state.active && !state.health.valid);
    CHECK((state.health.faults & FLYWHEEL_JOG_FAULT_CAN_EPOCH) != 0U);
    CHECK(normal_calls == calls && nonzero_submissions == nonzero && stop_calls == stops + 1U);
    CHECK(MotorBackend_TakeRearmRequest());
    CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_OFF));
    bus.fault_epoch = 0U; feedback(146U);
    CHECK(!apply(147U, 0.0f, 0U));
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    read_state(); CHECK((state.health.faults & FLYWHEEL_JOG_FAULT_CAN_EPOCH) != 0U);
    CHECK(!health.flywheels_ready && !health.actuator_fault);

    test_stage = 11U;
    reset(100U); feedback(100U);
    CHECK(!apply(100U, 0.0f, 0U));
    CanBus_SetOutputPermission(1U);
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    CanBus_SetOutputPermission(0U); bus.started = 0U;
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    bus.started = 1U; now_ms = 201U;
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    CHECK(normal_calls == 0U && stop_calls == 0U);

    test_stage = 12U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    now_ms = 145U; CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_OFF));
    CHECK(!apply(146U, 0.0f, 0U));
    calls = normal_calls; stops = stop_calls;
    reads_in_flight = 1U;
    CHECK(!apply(166U, 0.0f, 0U));
    read_state(); CHECK(keep_calls == 1U && !state.stop_pending);
    CHECK(reads_in_flight && stop_calls == stops && last_keep_time == 166U);
    reads_in_flight = 0U; feedback(186U);
    CHECK(!apply(186U, 0.0f, 0U));
    CHECK(keep_calls == 2U && stop_calls == stops);
    keep_result = HAL_ERROR;
    CHECK(!apply(206U, 0.0f, 0U));
    read_state(); CHECK(state.stop_pending && keep_calls == 3U);
    CHECK(!apply(207U, 0.0f, 0U));
    CHECK(!apply(225U, 0.0f, 0U));
    CHECK(stop_calls == stops);
    CHECK(!apply(226U, 0.0f, 0U));
    read_state(); CHECK(stop_calls == stops + 1U && !state.stop_pending);
    CHECK(normal_calls == calls && !permission);

    test_stage = 13U;
    prepare(FLYWHEEL_JOG_RIGHT, 0xFFFFFFC0UL);
    now_ms = 0xFFFFFFF0UL; fail_stop = 1U; MotorBackend_Stop();
    stops = stop_calls; nonzero = nonzero_submissions;
    CHECK(!apply(0xFFFFFFF0UL, 1.0f, 1U));
    CHECK(stop_calls == stops + 1U && last_stop_time == 0xFFFFFFF0UL);
    CHECK(!apply(3U, 1.0f, 1U));
    CHECK(stop_calls == stops + 1U);
    CHECK(!apply(4U, 1.0f, 1U));
    CHECK(stop_calls == stops + 2U && last_stop_time == 4U);
    fail_stop = 0U;
    CHECK(!apply(24U, 1.0f, 1U));
    read_state(); CHECK(!state.stop_pending && !state.active && nonzero_submissions == nonzero);
#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U
    test_stage = 14U;
    for (time = 70U; time <= 71U; ++time) {
        prepare(time == 70U ? FLYWHEEL_JOG_LEFT : FLYWHEEL_JOG_RIGHT, 100U);
        feedback(164U); now_ms = 165U;
        f = frame((uint8_t)time, 9U, now_ms, 1000); CHECK(MotorBackend_OnFrame(&f));
        CHECK(apply(165U, 1.0f, 1U)); pair(6U, 0, 0);
        read_state(); CHECK(state.active && !state.stop_pending && state.requested_mA == 0);
        CHECK(permission && !state.health.faults && !MotorBackend_TakeRearmRequest());
        feedback(185U); now_ms = 186U;
        f = frame((uint8_t)time, 9U, now_ms, 800); CHECK(MotorBackend_OnFrame(&f));
        CHECK(apply(186U, 1.0f, 1U)); pair(8U, 0, 0);
        read_state(); CHECK(state.active && state.requested_mA == 0 && permission);
        feedback(206U); now_ms = 207U;
        f = frame((uint8_t)time, 9U, now_ms, 599); CHECK(MotorBackend_OnFrame(&f));
        CHECK(apply(207U, 1.0f, 1U)); pair(10U, time == 70U ? 105 : 0, time == 71U ? 105 : 0);
        read_state(); CHECK(state.active && state.requested_mA == 105 && !state.stop_pending);
        CHECK(!MotorBackend_TakeRearmRequest());
        feedback(227U); now_ms = 228U;
        f = frame((uint8_t)time, 9U, now_ms, 1000); CHECK(MotorBackend_OnFrame(&f));
        CHECK(apply(228U, 1.0f, 1U)); pair(12U, 0, 0);
        stops = stop_calls; calls = normal_calls;
        CHECK(!apply(229U, 0.0f, 1U));
        read_state(); CHECK(!state.active && state.requested_mA == 0 && !permission);
        CHECK(stop_calls == stops + 1U && normal_calls == calls && MotorBackend_TakeRearmRequest());
        (void)apply(230U, 1.0f, 1U);
        read_state(); CHECK(!state.active && normal_calls == calls);
    }

    test_stage = 15U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    feedback(164U); now_ms = 165U;
    f = frame(70U, 9U, now_ms, 1000); CHECK(MotorBackend_OnFrame(&f));
    fail_normal_call = normal_calls + 1U; stops = stop_calls;
    CHECK(!apply(165U, 1.0f, 1U));
    read_state(); CHECK(!state.active && !permission && state.requested_mA == 0);
    CHECK((state.health.faults & FLYWHEEL_JOG_FAULT_EXTERNAL) != 0U);
    CHECK(stop_calls == stops + 1U && MotorBackend_TakeRearmRequest());
#endif
    test_done = 1U;
}
#else
void test_run(void)
{
    uint32_t node, time, nonzero, calls, stops;
    CanFrame f;
    test_stage = 1U;
    for (node = 70U; node <= 71U; ++node) {
        prepare(node == 70U ? FLYWHEEL_JOG_LEFT : FLYWHEEL_JOG_RIGHT, 100U);
        feedback(164U); now_ms = 165U;
        f = frame((uint8_t)node, 9U, now_ms, 1600);
        CHECK(MotorBackend_OnFrame(&f));
        CHECK(apply(165U, 1.0f, 1U)); pair(6U, 0, 0);
        read_state();
        CHECK(state.active && state.speed.coasting && !state.stop_pending);
        CHECK(state.requested_mA == 0 && permission && !state.health.faults);
        CHECK(!MotorBackend_TakeRearmRequest());
        for (time = 185U; time <= 3185U; time += 20U) {
            feedback(time); CHECK(apply(time, 1.0f, 1U)); read_state();
            CHECK(state.active && state.speed.running && !state.health.faults);
            CHECK(state.requested_mA >= 0 && state.requested_mA <= 500);
            pair(normal_calls - 2U, node == 70U ? state.requested_mA : 0,
                node == 71U ? state.requested_mA : 0);
        }
        CHECK(state.speed.target_erpm == 1400.0f && state.requested_mA == 500);
        CHECK(state.speed.at_limit && state.speed.limited_ms > 0U);
        CHECK(state.speed.peak_erpm == 1600U);
        calls = normal_calls; nonzero = nonzero_submissions; stops = stop_calls;
        fail_stop = 1U; MotorBackend_Stop();
        CHECK(!apply(3186U, 1.0f, 1U)); read_state();
        CHECK(!state.active && !state.speed.running && state.stop_pending && !permission);
        CHECK(state.speed.integral_mA == 0.0f && state.speed.peak_erpm == 1600U);
        CHECK(stop_calls == stops + 1U && normal_calls == calls);
        CHECK(!apply(3205U, 1.0f, 1U)); CHECK(stop_calls == stops + 1U);
        fail_stop = 0U; feedback(3206U);
        CHECK(!apply(3206U, 1.0f, 1U)); read_state();
        CHECK(stop_calls == stops + 2U && !state.stop_pending && !state.active);
        feedback(3226U); (void)apply(3226U, 1.0f, 1U); read_state();
        CHECK(!state.active && nonzero_submissions == nonzero);
    }

    test_stage = 2U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    now_ms = 145U; f = frame(70U, 9U, now_ms, 2001);
    CHECK(MotorBackend_OnFrame(&f)); read_state();
    CHECK(!permission && !state.active && state.stop_pending);
    CHECK(state.speed.peak_erpm == 2001U && !state.speed.running);
    CHECK(state.health.faults == FLYWHEEL_JOG_FAULT_OVERSPEED);
    CHECK(MotorBackend_TakeRearmRequest());
    CHECK(!apply(146U, 1.0f, 1U));
    CHECK(stop_calls == 2U && nonzero_submissions == 2U);

    test_stage = 3U;
    prepare(FLYWHEEL_JOG_RIGHT, 100U);
    feedback(164U); now_ms = 165U;
    f = frame(71U, 9U, now_ms, 1600); CHECK(MotorBackend_OnFrame(&f));
    fail_normal_call = normal_calls + 1U; stops = stop_calls;
    CHECK(!apply(165U, 1.0f, 1U)); read_state();
    CHECK(!state.active && !state.speed.running && !permission && state.requested_mA == 0);
    CHECK((state.health.faults & FLYWHEEL_JOG_FAULT_EXTERNAL) != 0U);
    CHECK(stop_calls == stops + 1U && MotorBackend_TakeRearmRequest());
    CHECK(state.speed.peak_erpm == 1600U);

    test_stage = 4U;
    prepare(FLYWHEEL_JOG_LEFT, 100U);
    calls = normal_calls; nonzero = nonzero_submissions;
    CHECK(!apply(145U, 0.0f, 1U)); read_state();
    CHECK(!state.active && !state.speed.running && state.requested_mA == 0);
    CHECK(normal_calls == calls && !permission && MotorBackend_TakeRearmRequest());
    feedback(165U); (void)apply(165U, 1.0f, 1U); read_state();
    CHECK(!state.active && nonzero_submissions == nonzero);
    CHECK(MotorBackend_SelectJog(FLYWHEEL_JOG_OFF));
    read_state(); CHECK(state.speed.peak_erpm == 0U && !state.speed.running);
    test_done = 1U;
}
#endif
