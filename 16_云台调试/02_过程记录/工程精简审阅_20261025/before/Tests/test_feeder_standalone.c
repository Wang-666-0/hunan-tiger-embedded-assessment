/**
 * @file test_feeder_standalone.c
 * @brief 缺摩擦轮电调时的拨盘单发/连发集成回归。
 * 链接真实遥控映射、安全/目标逻辑、拨盘及双轮后端；仅模拟传输和云台健康。
 * Cycle 重现控制调用顺序，Complete 送入已知编码器轨迹，非实机性能模型。
 */
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"
#include "motor_backend.h"
#include "feeder_backend.h"
#include "feeder_console.h"
#include "can_bus.h"
#include <math.h>
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) test_failed_line = __LINE__; } } while (0)
static AppState app;
static SafetyInput input;
static RemoteInput_State remote;
static CanBusState bus;
static FeederBackendState feeder;
static MotorHealth wheels;
static AppAxisHealth yaw, pitch;
static uint32_t now_ms, nonzero, wheel_commands, rearm, stop_calls;
static uint8_t permission, wheel_permission, diagnostic;
static int64_t rotor_counts;
static int16_t rotor_rpm, feedback_current;
static char response[2048];

uint32_t HAL_GetTick(void) { return now_ms; }
void AppTasks_GetState(AppState *out) { *out = app; }
void RemoteInput_GetState(RemoteInput_State *out) { *out = remote; }
uint8_t AppConsole_IsDiagnosticMode(void) { return diagnostic; }
uint32_t AppConsole_GetFaultEpoch(void) { return 0U; }
void CanBus_GetState(CanBusState *out) { *out = bus; }
void CanBus_SetFeederPermission(uint8_t allowed)
{ permission = allowed && bus.started && !bus.fault_epoch; }
uint8_t CanBus_FeederAllowed(void) { return permission; }
void CanBus_SetOutputPermission(uint8_t allowed)
{ wheel_permission = allowed && !APP_FEEDER_STANDALONE_TEST_ACTIVE && bus.started && !bus.fault_epoch; }
uint8_t CanBus_OutputAllowed(void) { return wheel_permission; }
HAL_StatusTypeDef CanBus_FeederCurrent(int16_t value)
{
    CHECK(permission && value >= -3000 && value <= 3000);
    if (value) ++nonzero;
    return HAL_OK;
}
HAL_StatusTypeDef CanBus_StopFeeder(void)
{ permission = 0U; ++stop_calls; return bus.started && !bus.fault_epoch ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef CanBus_JogVescCurrent(uint8_t node, int32_t value)
{ (void)node; (void)value; ++wheel_commands; return HAL_ERROR; }
HAL_StatusTypeDef CanBus_StopFlywheels(void) { wheel_permission = 0U; return HAL_OK; }
HAL_StatusTypeDef CanBus_KeepFlywheelsStopped(void) { wheel_permission = 0U; return HAL_OK; }

static void Read(void) { FeederBackend_GetState(&feeder); MotorBackend_GetHealth(&wheels); }
static void Feed(uint32_t ms)
{
    CanFrame frame;
    uint16_t angle = (uint16_t)((rotor_counts % 8192 + 8192) % 8192);
    memset(&frame, 0, sizeof(frame)); frame.bus = 2U; frame.id = 0x205U;
    frame.dlc = 8U; frame.received_ms = now_ms = ms; frame.fault_epoch = bus.fault_epoch;
    frame.data[0] = (uint8_t)(angle >> 8); frame.data[1] = (uint8_t)angle;
    frame.data[2] = (uint8_t)((uint16_t)rotor_rpm >> 8); frame.data[3] = (uint8_t)rotor_rpm;
    frame.data[4] = (uint8_t)((uint16_t)feedback_current >> 8); frame.data[5] = (uint8_t)feedback_current;
    CHECK(FeederBackend_OnFrame(&frame));
}
static void Cycle(uint32_t ms, uint8_t new_remote)
{
    now_ms = input.now_ms = ms;
    if (new_remote) {
        input.remote.received_ms = ms; ++input.remote.sequence;
        memcpy(remote.channels, input.remote.channels, sizeof(remote.channels));
        remote.received_ms = ms; remote.has_frame = 1U;
    }
    CHECK(!MotorBackend_PrepareRemote(&input.remote, app.logic.safety.barrier_ms,
                                     app.logic.safety.barrier_valid));
    MotorBackend_GetHealth(&wheels);
    CHECK(!wheels.ready && !wheels.feedback_ready && !wheels.flywheels_ready);
    input.backend_ready = wheels.ready; input.feedback_ready = wheels.feedback_ready;
    input.actuator_fault = wheels.actuator_fault; input.can_epoch = bus.fault_epoch;
    /* Production combines requested gimbal axes before the safety step. */
    AppLogic_CombineActuatorHealth(&input, &yaw, &pitch, input.remote.channels[5] != 1000U);
    AppLogic_Step(&app.logic, &input, wheels.flywheels_ready);
    CHECK(!app.logic.targets.request.friction_requested);
    CanBus_SetOutputPermission(app.logic.safety.output_allowed && app.logic.targets.request.friction_requested);
    (void)MotorBackend_Apply(&app.logic.targets.request, app.logic.safety.output_allowed);
    FeederBackend_Apply(&app.logic.targets.request, app.logic.safety.output_allowed,
                        wheels.flywheels_ready, diagnostic);
    if (FeederBackend_TakeRearmRequest()) {
        ++rearm; ++input.stop_sequence; FeederBackend_Stop();
        app.logic.safety.output_allowed = app.logic.safety.operator_armed = 0U;
    }
    CHECK(!MotorBackend_TakeRearmRequest() && !wheel_commands && !wheel_permission);
    Read(); app.remote = input.remote;
}
static void Tick(uint32_t ms) { Feed(ms); Cycle(ms, 1U); }
static void Rc(uint32_t ms, uint16_t ch5, uint16_t ch6, uint16_t ch8)
{
    input.remote.channels[4] = ch5; input.remote.channels[5] = ch6;
    input.remote.channels[7] = ch8; Tick(ms);
}
static void Prepare(uint32_t ms)
{
    uint32_t i;
    memset(&app, 0, sizeof(app)); memset(&input, 0, sizeof(input));
    memset(&remote, 0, sizeof(remote)); memset(&bus, 0, sizeof(bus)); bus.started = 1U;
    memset(&yaw, 0, sizeof(yaw)); memset(&pitch, 0, sizeof(pitch));
    yaw.requested = yaw.online = pitch.requested = pitch.online = 1U;
    nonzero = wheel_commands = rearm = stop_calls = 0U;
    permission = wheel_permission = diagnostic = 0U; rotor_counts = 0; rotor_rpm = feedback_current = 0;
    for (i = 0U; i < 14U; ++i) input.remote.channels[i] = 1500U;
    input.remote.channels[4] = input.remote.channels[5] = 1000U; input.remote.has_frame = 1U;
    AppLogic_Init(&app.logic); MotorBackend_Init(); FeederBackend_Init();
    Tick(ms); Tick(ms + 2U); Tick(ms + 4U);
    CHECK(FeederBackend_Ready()); Tick(ms + 6U);
    CHECK(feeder.armed && feeder.controller.have_zero && !feeder.stop_pending && !nonzero);
    CHECK(feeder.controller.config.pos_kp == APP_FEEDER_POSITION_KP);
    CHECK(feeder.controller.config.speed_kp == APP_FEEDER_SPEED_KP);
    CHECK(feeder.controller.config.speed_ki == APP_FEEDER_SPEED_KI);
    Rc(ms + 8U, 1500U, 1000U, 1500U);
    CHECK(app.logic.safety.output_allowed && app.logic.safety.operator_armed);
    Rc(ms + 10U, 1500U, 1500U, 1500U);
    CHECK(permission && !feeder.controller.busy && !wheels.flywheels_ready);
}
static void Start(uint32_t ms)
{
    Prepare(ms); Rc(ms + 12U, 1500U, 1500U, 2000U);
    CHECK(feeder.controller.accepted == 1U && feeder.controller.busy && nonzero);
    CHECK(feeder.controller.motion_kind == FEEDER_MOTION_SINGLE);
}
static void Complete(void)
{
    uint32_t i;
    int64_t target = feeder.controller.zero_counts +
        (int64_t)(feeder.controller.target_deg * 8192.0 / 360.0);
    for (i = 0; i < 600U && feeder.controller.busy; ++i) {
        int64_t delta = target - rotor_counts;
        rotor_counts += delta > 150 ? 150 : delta < -150 ? -150 : delta;
        rotor_rpm = delta > 0 ? 550 : delta < 0 ? -550 : 0;
        Tick(now_ms + 2U);
    }
    CHECK(!feeder.controller.busy && !feeder.controller.faults);
    CHECK(fabs(feeder.controller.axis_position_deg - feeder.controller.axis_target_deg) < 0.01);
}

void test_run(void)
{
    uint32_t i, before;
    CHECK(APP_FEEDER_STANDALONE_TEST_ACTIVE && APP_FEEDER_STANDALONE_TEST_ENABLED == 1U);
    test_stage = 1U; Start(100U);
    CHECK(fabs(feeder.controller.axis_target_deg + 40.0) < 0.001);
    for (i = 0; i < 30U; ++i) Tick(now_ms + 2U);
    CHECK(feeder.controller.accepted == 1U); /* Held trigger cannot repeat. */
    Complete(); CHECK(feeder.controller.completed == 1U && feeder.single_accepted == 1U);
    CHECK(!FeederBackend_Single()); /* Physical trigger is still held. */
    Rc(now_ms + 2U, 1500U, 1500U, 1500U); Rc(now_ms + 2U, 1500U, 1500U, 2000U);
    CHECK(feeder.controller.accepted == 2U && fabs(feeder.controller.axis_target_deg + 80.0) < 0.001);
    Complete(); CHECK(feeder.controller.completed == 2U && !feeder.continuous_accepted);
    CHECK(FeederConsole_Handle("FEEDER MODE", response, sizeof(response)));
    CHECK(strstr(response, "FEEDER_ONLY|STANDALONE_TEST|1|WHEEL_OUTPUT|DISABLED|REQUIRE_WHEELS_READY|0|CONTINUOUS|1"));
    CHECK(FeederConsole_Handle("FEEDER STATUS", response, sizeof(response)));
    CHECK(strstr(response, "|WHEELS_READY|0|") && strstr(response, "|FAULT|0|"));
    FeederConsole_Telemetry(response, sizeof(response)); CHECK(strncmp(response, "feeder:79.", 10U) == 0);

    test_stage = 2U; Prepare(2000U);
    /* Entering continuous mode with trigger held cannot start without release. */
    Rc(now_ms + 2U, 1500U, 2000U, 2000U);
    for (i = 0; i < 70U; ++i) Tick(now_ms + 2U);
    CHECK(!feeder.controller.accepted && !FeederBackend_Single() && !feeder.controller.busy);
    Rc(now_ms + 2U, 1500U, 1500U, 2000U); CHECK(!feeder.controller.accepted);
    Rc(now_ms + 2U, 1500U, 1500U, 1500U);
    CHECK(FeederBackend_Single()); CHECK(!FeederBackend_Single()); Tick(now_ms + 2U);
    CHECK(feeder.controller.accepted == 1U && feeder.single_accepted == 1U);
    Complete();
    /* Mode change observes a fresh release, then accepts exact 50ms requests. */
    Rc(now_ms + 2U, 1500U, 2000U, 1500U);
    Rc(now_ms + 2U, 1500U, 2000U, 2000U);
    {
        uint32_t started = now_ms, accepted = feeder.controller.accepted;
        CHECK(app.logic.targets.request.continuous_fire && feeder.controller.motion_kind == FEEDER_MOTION_STREAM);
        for (i = 1U; i <= 125U; ++i) {
            int64_t target = feeder.controller.zero_counts +
                (int64_t)(feeder.controller.target_deg * 8192.0 / 360.0);
            int64_t delta = target - rotor_counts;
            rotor_counts += delta < -1299 ? -1299 : delta > 1299 ? 1299 : delta;
            rotor_rpm = delta < 0 ? -4757 : delta > 0 ? 4757 : 0;
            Tick(started + i * 2U);
            CHECK(feeder.controller.accepted == accepted + i * 2U / APP_FIRE_INTERVAL_MS);
            CHECK(!feeder.controller.faults && !wheel_commands);
        }
        CHECK(feeder.continuous_accepted == 6U && !feeder.controller.rejected);
    }
    Rc(now_ms + 2U, 1500U, 2000U, 1500U); /* Release finishes only accepted tail. */
    before = feeder.controller.accepted; Complete();
    for (i = 0; i < 30U; ++i) Tick(now_ms + 2U);
    CHECK(feeder.controller.accepted == before && !app.logic.targets.request.continuous_fire);
    /* Test mode does not grant wheel ownership, even through direct console APIs. */
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_LEFT));
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_RIGHT));
    CHECK(!MotorBackend_SelectJog(FLYWHEEL_JOG_BOTH));
    for (i = 0; i < 100U; ++i) {
        TargetRequest malicious = app.logic.targets.request;
        malicious.friction_requested = 1U; malicious.gimbal_mode = GIMBAL_MANUAL;
        now_ms += 2U; CHECK(!MotorBackend_Apply(&malicious, 1U));
    }
    CHECK(!wheel_commands && !MotorBackend_TakeRearmRequest());

    test_stage = 3U; Start(4000U); before = nonzero;
    for (i = 0; i < 12U; ++i) Cycle(now_ms + 2U, 1U); /* Feeder feedback absent. */
    CHECK(feeder.controller.faults & FEEDER_FAULT_FEEDBACK);
    CHECK(!permission && !feeder.armed && !feeder.controller.have_zero && rearm == 1U);
    CHECK(!feeder.controller.busy && nonzero >= before && !FeederBackend_Single());
    before = nonzero; Tick(now_ms + 2U); CHECK(nonzero == before);

    test_stage = 4U;
    for (i = 0; i < 5U; ++i) {
        Start(5000U + i * 1000U); before = nonzero;
        if (i == 0U) input.remote.channels[7] = 1000U; /* CH8 lock. */
        if (i == 1U) input.remote.channels[4] = 2000U; /* Gimbal step mode. */
        if (i == 2U) bus.fault_epoch = 1U;
        if (i == 3U) diagnostic = 1U;
        if (i == 4U) pitch.online = 0U;
        Tick(now_ms + 2U);
        CHECK(!permission && !feeder.armed && !feeder.controller.busy && nonzero == before);
    }

    test_stage = 5U; Start(11000U); before = feeder.controller.accepted;
    for (i = 0; i < REMOTE_TIMEOUT_MS / 2U + 2U; ++i) {
        Feed(now_ms + 2U); Cycle(now_ms, 0U);
    }
    CHECK(!permission && !feeder.armed && !app.logic.safety.operator_armed);
    CHECK(feeder.controller.accepted == before && !feeder.controller.busy);
    test_stage = 6U; Start(13000U); before = nonzero;
    feedback_current = FEEDER_CURRENT_PROTOCOL_LIMIT + 1; Tick(now_ms + 2U);
    CHECK(feeder.controller.faults & FEEDER_FAULT_CURRENT);
    CHECK(!permission && !feeder.armed && nonzero == before);
    test_done = 1U;
}
