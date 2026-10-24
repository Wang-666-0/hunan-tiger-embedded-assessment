#include "app_logic.h"
#include "motor_backend.h"
#include "can_bus.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static AppLogic logic;
static SafetyInput input;
static CanBusState bus;
static MotorJogState state;
static MotorHealth health;
static uint32_t now_ms, stop_sequence, zeros, nonzeros, commands, fail_at;
static uint8_t permission, fail_stop;

uint32_t HAL_GetTick(void) { return now_ms; }
void CanBus_GetState(CanBusState *out) { *out = bus; }
uint8_t CanBus_OutputAllowed(void) { return permission; }
void CanBus_SetOutputPermission(uint8_t value) { permission = value && bus.started && !bus.fault_epoch; }
HAL_StatusTypeDef CanBus_JogVescCurrent(uint8_t node, int32_t current)
{
    commands++;
    CHECK(node == 70U || node == 71U);
    CHECK(current >= 0 && current <= 2000);
    if (!permission || !bus.started || bus.fault_epoch || commands == fail_at) { return HAL_ERROR; }
    if (current) { nonzeros++; } else { zeros++; }
    return HAL_OK;
}
HAL_StatusTypeDef CanBus_StopFlywheels(void)
{
    permission = 0U;
    if (fail_stop || !bus.started || bus.fault_epoch) { return HAL_BUSY; }
    zeros += 2U; return HAL_OK;
}
HAL_StatusTypeDef CanBus_KeepFlywheelsStopped(void)
{
    permission = 0U;
    if (!bus.started || bus.fault_epoch) { return HAL_ERROR; }
    zeros += 2U; return HAL_OK;
}
static void stop(void)
{
    stop_sequence++; CanBus_SetOutputPermission(0U); MotorBackend_Stop();
}
static void put16(uint8_t *p, int32_t value) { p[0] = (uint8_t)((uint32_t)value >> 8); p[1] = (uint8_t)value; }
static void put32(uint8_t *p, int32_t value)
{
    p[0] = (uint8_t)((uint32_t)value >> 24); p[1] = (uint8_t)((uint32_t)value >> 16);
    p[2] = (uint8_t)((uint32_t)value >> 8); p[3] = (uint8_t)value;
}
static void feedback(int32_t left, int32_t right)
{
    uint8_t node;
    for (node = 70U; node <= 71U; ++node) {
        CanFrame f;
        memset(&f, 0, sizeof(f)); f.bus = 2U; f.extended = 1U; f.dlc = 8U;
        f.received_ms = now_ms; f.fault_epoch = bus.fault_epoch;
        f.id = (9U << 8) | node; put32(f.data, node == 70U ? left : right);
        CHECK(MotorBackend_OnFrame(&f));
        f.id = (16U << 8) | node; memset(f.data, 0, sizeof(f.data)); put16(f.data, 250);
        CHECK(MotorBackend_OnFrame(&f));
    }
}
/* 按实际公开接口走接收->保护->请求->适配->发送，不替换速度环或会话逻辑。 */
static void cycle(uint32_t timestamp, uint8_t fresh, int32_t left, int32_t right)
{
    TargetRequest request;
    now_ms = timestamp; input.now_ms = timestamp;
    feedback(left, right);
    if (fresh) { input.remote.sequence++; input.remote.received_ms = timestamp; }
    if (MotorBackend_PrepareRemote(&input.remote, logic.safety.barrier_ms, logic.safety.barrier_valid)) { stop(); }
    MotorBackend_GetHealth(&health);
    CHECK(!health.flywheels_ready); /* 拨盘/云台驱动本次仍关闭。 */
    input.backend_ready = health.ready; input.feedback_ready = health.feedback_ready;
    input.actuator_fault = health.actuator_fault; input.can_epoch = bus.fault_epoch;
    input.stop_sequence = stop_sequence;
    AppLogic_Step(&logic, &input, health.flywheels_ready);
    request = logic.targets.request;
    request.wheel_reset_allowed = logic.command.valid && logic.command.failsafe_ok &&
        logic.command.wheel_switch == RC_LOW;
    CanBus_SetOutputPermission(logic.safety.output_allowed && request.friction_requested);
    (void)MotorBackend_Apply(&request, logic.safety.output_allowed);
    if (MotorBackend_TakeRearmRequest()) {
        stop(); logic.safety.output_allowed = logic.safety.operator_armed = 0U;
    }
    MotorBackend_GetJogState(&state);
    CHECK(state.requested_left_mA >= 0 && state.requested_left_mA <= 2000);
    CHECK(state.requested_right_mA >= 0 && state.requested_right_mA <= 2000);
    CHECK(!logic.targets.request.shot_event && !logic.targets.request.shot_sequence);
}
static void reset(void)
{
    uint32_t i;
    memset(&bus, 0, sizeof(bus)); memset(&input, 0, sizeof(input));
    bus.started = 1U; now_ms = 100U; permission = fail_stop = 0U;
    stop_sequence = zeros = nonzeros = commands = fail_at = 0U;
    for (i = 0U; i < 14U; ++i) { input.remote.channels[i] = 1500U; }
    input.remote.channels[4] = input.remote.channels[5] = 1000U;
    input.remote.has_frame = 1U;
    MotorBackend_Init(); AppLogic_Init(&logic);
}
static void start(void)
{
    reset(); cycle(100U, 1U, 0, 0); cycle(120U, 1U, 0, 0); cycle(140U, 1U, 0, 0);
    CHECK(state.selection == FLYWHEEL_JOG_BOTH && !state.active);
    input.remote.channels[4] = 1500U; cycle(160U, 1U, 0, 0);
    CHECK(logic.safety.operator_armed && !state.active);
    input.remote.channels[5] = 1500U; cycle(180U, 1U, 0, 0); cycle(182U, 0U, 0, 0);
    CHECK(state.active && state.selection == FLYWHEEL_JOG_BOTH);
    cycle(202U, 1U, 0, 0);
    CHECK(nonzeros >= 2U && state.speed.target_erpm > 0 && state.speed_right.target_erpm > 0);
}
static void ordinary_controls(void)
{
    uint32_t starts, old_nonzero;
    test_stage = 1U; start(); starts = stop_sequence;
    input.remote.channels[0] = 1000U; cycle(222U, 1U, 0, 0);
    CHECK(state.active && logic.targets.request.yaw_rate_dps < 0.0f);
    input.remote.channels[5] = 2000U; input.remote.channels[7] = 2000U;
    cycle(242U, 1U, 0, 0); CHECK(state.active && stop_sequence == starts);
    input.remote.channels[7] = 1500U; input.remote.channels[5] = 1000U;
    old_nonzero = nonzeros; cycle(262U, 1U, 0, 0);
    CHECK(!state.active && !permission && logic.safety.operator_armed && stop_sequence == starts);
    CHECK(state.requested_left_mA == 0 && state.requested_right_mA == 0 && nonzeros == old_nonzero);
    cycle(282U, 1U, 0, 0);
    input.remote.channels[5] = 1500U; cycle(302U, 1U, 0, 0); cycle(304U, 0U, 0, 0);
    CHECK(state.active && logic.safety.operator_armed);
    input.remote.channels[4] = 2000U; cycle(324U, 1U, 0, 0);
    CHECK(!state.active && !logic.targets.request.friction_requested);
    input.remote.channels[4] = 1500U; cycle(344U, 1U, 0, 0); cycle(346U, 0U, 0, 0);
    CHECK(!state.active); /* TEST退出、旧CH6 ON不能恢复。 */
    input.remote.channels[5] = 1000U; cycle(366U, 1U, 0, 0);
    input.remote.channels[5] = 1500U; cycle(386U, 1U, 0, 0); cycle(388U, 0U, 0, 0);
    CHECK(state.active);
    input.remote.channels[5] = 1000U; cycle(408U, 1U, 200, 200);
    input.remote.channels[5] = 1500U; cycle(428U, 1U, 200, 200);
    cycle(448U, 1U, 0, 0); cycle(450U, 0U, 0, 0);
    CHECK(!state.active); /* 尚未停稳时ON不允许延迟自动启动。 */
}
static void failsafe_and_faults(void)
{
    uint32_t old_nonzero;
    test_stage = 2U; start(); old_nonzero = nonzeros;
    input.remote.channels[4] = input.remote.channels[5] = input.remote.channels[7] = 1000U;
    cycle(222U, 1U, 0, 0);
    CHECK(!state.active && !permission && !logic.safety.operator_armed && nonzeros == old_nonzero);
    CHECK(!logic.safety.off_seen);
    input.remote.channels[4] = input.remote.channels[5] = input.remote.channels[7] = 1500U;
    cycle(242U, 1U, 0, 0); CHECK(!state.active && !logic.safety.operator_armed);
    input.remote.channels[4] = input.remote.channels[5] = 1000U; cycle(262U, 1U, 0, 0);
    input.remote.channels[4] = 1500U; cycle(282U, 1U, 0, 0);
    input.remote.channels[5] = 1500U; cycle(302U, 1U, 0, 0); cycle(304U, 0U, 0, 0);
    CHECK(state.active);
    input.remote.has_frame = 0U; cycle(324U, 0U, 0, 0);
    CHECK(!state.active && !logic.safety.operator_armed);
    input.remote.has_frame = 1U; cycle(344U, 1U, 0, 0);
    CHECK(!state.active && !logic.safety.operator_armed);
    start(); input.uart_epoch++; cycle(222U, 1U, 0, 0);
    CHECK(!state.active && !permission && !logic.safety.operator_armed);
    start(); cycle(222U, 1U, 46201, 0);
    CHECK(!state.active && (state.health.faults & FLYWHEEL_JOG_FAULT_OVERSPEED));
    input.remote.channels[4] = input.remote.channels[5] = 1000U;
    cycle(242U, 1U, 0, 0); cycle(262U, 1U, 0, 0); cycle(282U, 1U, 0, 0);
    CHECK(!state.health.faults && state.selection == FLYWHEEL_JOG_BOTH && !state.active);
    cycle(292U, 1U, 0, 0); /* 重新准备双轮后的新OFF帧，不能复用选轮前帧。 */
    input.remote.channels[4] = 1500U; cycle(302U, 1U, 0, 0);
    input.remote.channels[5] = 1500U; cycle(322U, 1U, 0, 0); cycle(324U, 0U, 0, 0);
    CHECK(state.active);
    start(); fail_at = commands + 1U; cycle(222U, 1U, 0, 0);
    CHECK(!state.active && !permission && !logic.safety.operator_armed);
    CHECK(state.health.faults & FLYWHEEL_JOG_FAULT_EXTERNAL);
    start(); fail_stop = 1U; input.remote.channels[5] = 1000U; cycle(222U, 1U, 0, 0);
    CHECK(!state.active && state.stop_pending && !logic.safety.operator_armed);
    fail_stop = 0U; input.remote.channels[5] = 1500U; cycle(242U, 1U, 0, 0);
    cycle(262U, 1U, 0, 0); CHECK(!state.active && !logic.safety.operator_armed);
    start(); bus.fault_epoch = 1U; cycle(222U, 1U, 0, 0);
    CHECK(!state.active && !permission && !logic.safety.operator_armed);
}
void test_run(void)
{
    ordinary_controls(); failsafe_and_faults(); test_done = 1U;
}
