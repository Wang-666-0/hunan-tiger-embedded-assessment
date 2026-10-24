#include "feeder_backend.h"
#include "feeder_console.h"
#include "app_tasks.h"
#include "app_console.h"
#include "can_bus.h"
#include "remote_input.h"
#include "motor_backend.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
volatile uint32_t test_max_wire_bytes, test_max_wire_us;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

static AppState app;
static SafetyInput input;
static RemoteInput_State remote;
static MotorHealth wheels;
static CanBusState bus;
static FeederBackendState state;
static uint32_t now_ms, current_calls, nonzero_calls, stop_calls, rearm_calls, console_epoch;
static int16_t last_current, rotor_rpm, feedback_current;
static int64_t rotor_counts;
static uint8_t permission, diagnostic;
static HAL_StatusTypeDef current_result, stop_result;
static char response[2048];

uint32_t HAL_GetTick(void) { return now_ms; }
void AppTasks_GetState(AppState *out) { if (out) { *out = app; } }
void RemoteInput_GetState(RemoteInput_State *out) { if (out) { *out = remote; } }
uint8_t AppConsole_IsDiagnosticMode(void) { return diagnostic; }
uint32_t AppConsole_GetFaultEpoch(void) { return console_epoch; }
void MotorBackend_GetHealth(MotorHealth *out) { if (out) { *out = wheels; } }
void CanBus_GetState(CanBusState *out) { if (out) { *out = bus; } }
void CanBus_SetFeederPermission(uint8_t allowed)
{ permission = (uint8_t)(allowed && bus.started && !bus.fault_epoch); }
uint8_t CanBus_FeederAllowed(void) { return permission; }
HAL_StatusTypeDef CanBus_FeederCurrent(int16_t value)
{
    ++current_calls; if (value) { ++nonzero_calls; }
    last_current = value;
    CHECK(permission && bus.started && !bus.fault_epoch);
    CHECK(value >= -APP_FEEDER_CURRENT_HARD_LIMIT && value <= APP_FEEDER_CURRENT_HARD_LIMIT);
    return current_result;
}
HAL_StatusTypeDef CanBus_StopFeeder(void)
{
    ++stop_calls; permission = 0U; last_current = 0;
    return bus.started && !bus.fault_epoch ? stop_result : HAL_ERROR;
}
static void Read(void) { FeederBackend_GetState(&state); }
static void Feed(uint32_t ms)
{
    CanFrame frame;
    uint16_t angle = (uint16_t)((rotor_counts % 8192 + 8192) % 8192);
    memset(&frame, 0, sizeof(frame)); frame.bus = 2U; frame.id = 0x205U;
    frame.dlc = 8U; frame.received_ms = now_ms = ms; frame.fault_epoch = bus.fault_epoch;
    frame.data[0] = (uint8_t)(angle >> 8); frame.data[1] = (uint8_t)angle;
    frame.data[2] = (uint8_t)((uint16_t)rotor_rpm >> 8); frame.data[3] = (uint8_t)rotor_rpm;
    frame.data[4] = (uint8_t)((uint16_t)feedback_current >> 8); frame.data[5] = (uint8_t)feedback_current;
    CHECK(FeederBackend_OnFrame(&frame)); Read();
}
/* Mirrors the production AppTasks order: real mapper/safety/target logic,
 * then backend apply, then a backend rearm request clears the shared latch. */
static void LogicApply(uint32_t ms, uint8_t new_frame)
{
    now_ms = input.now_ms = ms;
    if (new_frame) {
        input.remote.received_ms = ms; ++input.remote.sequence;
        memcpy(remote.channels, input.remote.channels, sizeof(remote.channels));
        remote.received_ms = ms; remote.has_frame = 1U;
    }
    input.backend_ready = wheels.ready; input.feedback_ready = wheels.feedback_ready;
    input.actuator_fault = wheels.actuator_fault; input.can_epoch = bus.fault_epoch;
    input.uart_epoch = remote.fault_epoch + console_epoch;
    AppLogic_Step(&app.logic, &input, wheels.flywheels_ready);
    app.remote = input.remote;
    FeederBackend_Apply(&app.logic.targets.request, app.logic.safety.output_allowed,
        wheels.flywheels_ready, diagnostic);
    if (FeederBackend_TakeRearmRequest()) {
        ++rearm_calls; ++input.stop_sequence;
        FeederBackend_Stop();
        app.logic.safety.output_allowed = app.logic.safety.operator_armed = 0U;
    }
    Read();
}
static void Tick(uint32_t ms)
{ Feed(ms); LogicApply(ms, 1U); }
static void SetRemote(uint32_t ms, uint16_t ch5, uint16_t ch6, uint16_t ch8)
{
    input.remote.channels[4] = ch5; input.remote.channels[5] = ch6;
    input.remote.channels[7] = ch8; Tick(ms);
}
static void Reset(uint32_t ms)
{
    uint32_t index;
    memset(&app, 0, sizeof(app)); memset(&input, 0, sizeof(input));
    memset(&remote, 0, sizeof(remote)); memset(&wheels, 0, sizeof(wheels));
    memset(&bus, 0, sizeof(bus)); bus.started = 1U;
    wheels.ready = wheels.feedback_ready = 1U;
    now_ms = ms; current_calls = nonzero_calls = stop_calls = rearm_calls = console_epoch = 0U;
    permission = diagnostic = 0U; current_result = stop_result = HAL_OK;
    rotor_counts = 0; rotor_rpm = feedback_current = last_current = 0;
    for (index = 0U; index < 14U; ++index) { input.remote.channels[index] = 1500U; }
    input.remote.channels[4] = input.remote.channels[5] = 1000U;
    input.remote.has_frame = 1U;
    AppLogic_Init(&app.logic); FeederBackend_Init(); LogicApply(ms, 1U);
}
static void Prepare(uint32_t ms)
{
    FeederConfig config;
    Reset(ms); Feed(ms + 1U); Feed(ms + 2U); Feed(ms + 3U);
    config = state.controller.config; config.direction = 1; config.calibrated = 1U;
    config.gear_ratio = 1.0f; config.pos_kp = 5.0f;
    config.max_rpm = 300.0f; config.current_limit = 1000.0f;
    config.tolerance_deg = 1.0f; config.settle_rpm = 10.0f; config.settle_ms = 20U;
    config.motion_timeout_ms = 1000U; config.jam_current_raw = 800.0f; config.jam_ms = 150U;
    CHECK(FeederBackend_SetConfig(&config)); CHECK(FeederBackend_Zero());
    CHECK(FeederBackend_Arm()); Read();
    CHECK(state.armed && state.stop_pending && !current_calls && !nonzero_calls);
    LogicApply(ms + 4U, 1U);
    CHECK(state.armed && !state.stop_pending && !state.controller.busy && !current_calls);
    CHECK(stop_calls == 1U && !permission);
}
static void StartNormal(uint32_t ms, uint16_t ch6, uint8_t trigger)
{
    Prepare(ms); wheels.flywheels_ready = 1U;
    SetRemote(ms + 5U, 1500U, 1000U, 1500U);
    CHECK(app.logic.safety.operator_armed && !app.logic.targets.request.friction_requested);
    SetRemote(ms + 6U, 1500U, ch6, 1500U);
    CHECK(app.logic.targets.request.friction_requested && state.armed && permission);
    CHECK(!state.controller.busy && !nonzero_calls);
    if (trigger) {
        SetRemote(ms + 7U, 1500U, ch6, 2000U);
        CHECK(state.controller.busy && state.controller.accepted == 1U);
    }
}
static void RunTo(uint32_t end_ms)
{
    uint32_t ms;
    for (ms = now_ms + 1U; ms <= end_ms; ++ms) { Tick(ms); }
}
static void CompleteShot(uint32_t start_ms)
{
    uint32_t ms;
    int64_t target;
    Read(); target = state.controller.zero_counts + (int64_t)(state.controller.target_deg * 8192.0 / 360.0) + 1;
    for (ms = start_ms + 1U; ms <= start_ms + 31U; ++ms) {
        if (rotor_counts < target) {
            rotor_counts += target - rotor_counts > 100 ? 100 : target - rotor_counts;
            rotor_rpm = 600;
        } else { rotor_rpm = 0; }
        Tick(ms);
    }
    CHECK((!state.controller.busy || state.controller.motion_kind == FEEDER_MOTION_STREAM) && !state.controller.faults);
}
static void Console(const char *command, uint8_t ok)
{
    memset(response, 0, sizeof(response));
    CHECK(FeederConsole_Handle(command, response, sizeof(response)));
    CHECK(strstr(response, ok ? "|OK|" : "|REJECTED|") != 0);
    Read();
}
void test_run(void)
{
    FeederConfig config;
    uint32_t before, ms, accepted;
    test_stage = 1U;
    Reset(100U); config = state.controller.config;
    CHECK(!FeederBackend_Arm() && !FeederBackend_Zero() && !FeederBackend_Single());
    Console("FEEDER STATUS", 1U); CHECK(strstr(response, "|CAL|1|") && strstr(response, "|DIR|-1|"));
    Console("FEEDER CAL 0", 1U); Console("FEEDER DIR 0", 1U);
    Console("FEEDER CAL 1", 0U); Console("FEEDER DIR 1", 1U); Console("FEEDER GEAR 1", 1U);
    Console("FEEDER CAL 1", 1U); Console("FEEDER LIMIT 3001", 0U);
    Console("FEEDER RPM nan", 0U); Console("FEEDER RPM 301x", 0U);
    Feed(101U); Feed(102U); Feed(103U); Console("FEEDER ZERO", 1U);
    Console("FEEDER ARM", 1U); CHECK(state.armed && state.stop_pending && !current_calls);
    stop_result = HAL_BUSY; LogicApply(104U, 1U);
    CHECK(state.stop_pending && !current_calls && !permission);
    CHECK(!FeederBackend_Arm() && !FeederBackend_Single());
    LogicApply(105U, 1U); CHECK(state.stop_pending && stop_calls == 2U);
    stop_result = HAL_OK; LogicApply(106U, 1U);
    CHECK(!state.stop_pending && state.armed && stop_calls == 3U && !current_calls);
    CHECK(!FeederBackend_SetConfig(&config));
    Console("FEEDER OFF", 1U); LogicApply(107U, 1U); CHECK(!state.armed);
    config = state.controller.config; config.pos_kp = 6.0f;
    diagnostic = 1U; CHECK(!FeederBackend_SetConfig(&config)); diagnostic = 0U;
    remote.fault_epoch = 1U; CHECK(!FeederBackend_SetConfig(&config)); remote.fault_epoch = 0U;
    console_epoch = 1U; CHECK(!FeederBackend_SetConfig(&config)); console_epoch = 0U;
    now_ms = remote.received_ms + REMOTE_TIMEOUT_MS; CHECK(!FeederBackend_SetConfig(&config));
    now_ms = 108U; input.remote.channels[0] = 1600U; LogicApply(108U, 1U);
    CHECK(!FeederBackend_SetConfig(&config)); input.remote.channels[0] = 1500U;
    SetRemote(109U, 2000U, 1000U, 1500U); CHECK(!FeederBackend_SetConfig(&config));

    test_stage = 2U;
    StartNormal(200U, 1500U, 1U);
    CHECK(state.single_accepted == 1U && !state.continuous_accepted && state.controller.target_deg == 40.0);
    CompleteShot(207U); CHECK(state.controller.completed == 1U && state.controller.accepted == 1U);
    RunTo(307U); CHECK(state.controller.accepted == 1U && state.single_accepted == 1U);
    SetRemote(308U, 1500U, 1500U, 1500U); SetRemote(309U, 1500U, 1500U, 2000U);
    CHECK(state.controller.accepted == 2U && state.controller.target_deg == 80.0);
    CompleteShot(309U); CHECK(state.controller.completed == 2U && state.completed_interval_ms > 0U);
    CHECK(!FeederBackend_Single()); /* Physical trigger still held. */

    test_stage = 3U;
    StartNormal(400U, 2000U, 1U); CompleteShot(407U);
    RunTo(456U); CHECK(state.controller.accepted == 1U);
    Tick(457U); CHECK(state.controller.accepted == 2U && state.continuous_accepted == 2U);
    CompleteShot(457U); RunTo(506U); CHECK(state.controller.accepted == 2U);
    Tick(507U); CHECK(state.controller.accepted == 3U && state.controller.target_deg == 120.0);
    CompleteShot(507U);
    CHECK(state.controller.continuous_completed == 3U && state.controller.single_completed == 0U);
    SetRemote(539U, 1500U, 2000U, 1500U); RunTo(561U);
    CHECK(!state.controller.busy && state.armed && !state.controller.aborted);
    StartNormal(600U, 2000U, 1U); RunTo(707U);
    CHECK(state.controller.accepted == 2U && state.controller.rejected == 1U && state.controller.busy);
    CHECK(state.controller.outstanding == FEEDER_STREAM_MAX_PENDING);
    SetRemote(708U, 1500U, 2000U, 1500U); CompleteShot(708U);
    RunTo(808U); CHECK(state.controller.accepted == 2U && state.controller.completed == 1U);
    CHECK(state.controller.cancelled == 1U && !state.controller.aborted && !state.controller.busy && state.armed);

    test_stage = 4U;
    StartNormal(900U, 1500U, 0U); wheels.flywheels_ready = 0U;
    CHECK(!FeederBackend_Single()); Console("FEEDER SINGLE", 0U);
    SetRemote(907U, 1500U, 1500U, 2000U); CHECK(!state.controller.busy && !state.controller.accepted);
    wheels.flywheels_ready = 1U; RunTo(912U); CHECK(!state.controller.accepted);
    CHECK(!FeederBackend_Single());
    SetRemote(913U, 1500U, 1500U, 1500U); Console("FEEDER SINGLE", 1U);
    CHECK(!state.controller.busy); Tick(914U);
    CHECK(state.controller.busy && state.single_accepted == 1U);
    CHECK(!FeederBackend_Single()); CompleteShot(914U);
    CHECK(FeederBackend_Single()); before = state.controller.accepted;
    diagnostic = 1U; Tick(946U);
    CHECK(!state.controller.busy && state.controller.accepted == before && !state.armed && !permission);
    diagnostic = 0U; RunTo(949U); CHECK(state.controller.accepted == before);
    StartNormal(1000U, 1500U, 0U); CHECK(FeederBackend_Single());
    Feed(1017U); LogicApply(1017U, 1U); /* Pending serial request expires after 10ms. */
    CHECK(!state.controller.busy && !state.controller.accepted && state.controller.rejected == 1U);
    RunTo(1020U); CHECK(!state.controller.accepted);
    Console("FEEDER PROBE", 0U);

    test_stage = 5U;
    StartNormal(1100U, 2000U, 1U); before = nonzero_calls;
    SetRemote(1108U, 1500U, 2000U, 1000U); /* CH8 lock. */
    CHECK(!app.logic.safety.operator_armed && !state.armed && !permission && !state.controller.busy);
    CHECK(state.controller.aborted == 1U && nonzero_calls == before);
    SetRemote(1109U, 1500U, 2000U, 1500U); RunTo(1112U);
    CHECK(!app.logic.safety.operator_armed && !state.controller.busy && state.controller.accepted == 1U);
    StartNormal(1200U, 1500U, 1U); before = nonzero_calls;
    SetRemote(1208U, 2000U, 1500U, 2000U); /* CH5 test forbids both shooter requests. */
    CHECK(app.logic.targets.request.gimbal_mode == GIMBAL_TEST && !app.logic.targets.request.friction_requested);
    CHECK(!state.armed && !state.controller.busy && !permission && nonzero_calls == before);
    RunTo(1212U); CHECK(state.controller.accepted == 1U);
    StartNormal(1300U, 1500U, 1U); before = nonzero_calls;
    wheels.flywheels_ready = 0U; Tick(1308U);
    CHECK(state.armed && state.controller.busy && permission && !state.controller.aborted);
    CHECK(!rearm_calls && nonzero_calls > before && !FeederBackend_Single());
    CompleteShot(1308U); Tick(1340U);
    CHECK(state.armed && !state.controller.busy && !permission && state.controller.have_zero);
    CHECK(state.controller.completed == 1U && state.controller.accepted == 1U && !rearm_calls);
    wheels.flywheels_ready = 1U; RunTo(1342U); CHECK(state.controller.accepted == 1U);
    SetRemote(1343U, 1500U, 1500U, 1500U);
    SetRemote(1344U, 1500U, 1500U, 2000U);
    CHECK(state.controller.accepted == 2U && state.controller.busy);
    before = nonzero_calls; wheels.feedback_ready = 0U; Tick(1345U);
    CHECK(!state.armed && !state.controller.busy && !permission && !state.controller.have_zero);
    CHECK(state.controller.aborted == 1U && rearm_calls == 1U && nonzero_calls == before);

    test_stage = 6U;
    StartNormal(1400U, 2000U, 1U); before = nonzero_calls;
    for (ms = 1408U; ms <= 1428U; ++ms) { LogicApply(ms, 1U); }
    CHECK((state.controller.faults & FEEDER_FAULT_FEEDBACK) && !state.armed && !permission);
    CHECK(!state.controller.have_zero && !state.controller.busy && state.controller.aborted == 1U);
    CHECK(nonzero_calls >= before && rearm_calls == 1U);
    accepted = state.controller.accepted; RunTo(1432U);
    CHECK(state.controller.accepted == accepted && !app.logic.safety.operator_armed);
    CHECK(!FeederBackend_Arm() && !FeederBackend_Single());
    StartNormal(1500U, 1500U, 1U); current_result = HAL_ERROR;
    Tick(1508U); CHECK((state.controller.faults & FEEDER_FAULT_CAN) && !state.armed && !permission);
    CHECK(!state.controller.have_zero && !state.controller.busy && rearm_calls == 1U);
    current_result = HAL_OK; RunTo(1512U); CHECK(state.controller.accepted == 1U);
    StartNormal(1600U, 1500U, 0U); current_result = HAL_BUSY;
    for (ms = 1607U; ms <= 1620U; ++ms) { Tick(ms); }
    CHECK((state.controller.faults & FEEDER_FAULT_CAN) && !state.armed && !permission);
    CHECK(rearm_calls == 1U && !state.controller.have_zero);
    Prepare(1650U); wheels.flywheels_ready = 1U;
    SetRemote(1655U, 1500U, 1000U, 1500U);
    current_result = HAL_BUSY; SetRemote(1656U, 1500U, 1500U, 1500U);
    CHECK(state.armed && !state.controller.faults && !state.controller.busy);
    current_result = HAL_OK; Tick(1657U);
    CHECK(state.armed && !state.controller.faults && !state.controller.accepted);

    test_stage = 7U;
    Prepare(1700U); Console("FEEDER STATUS", 1U);
    CHECK(strstr(response, "FB_TORQUE_RAW") && strstr(response, "STOP_PENDING") && strstr(response, "DONE_DT_MS"));
    CHECK(strstr(response, "|STEP|40|") && strstr(response, "|GEAR|1|") && strstr(response, "|CAL|1|"));
    FeederConsole_Telemetry(response, sizeof(response)); CHECK(strncmp(response, "feeder:", 7U) == 0);
    CHECK(strlen(response) > 70U && strlen(response) < 500U);
    CHECK(!FeederConsole_Handle("YAW INFO", response, sizeof(response)));

    /* Exercise valid finite parameter endpoints through the production setter,
     * and signed physical feedback endpoints through the real C610 parser. */
    Console("FEEDER OFF", 1U); LogicApply(1705U, 1U);
    config = state.controller.config; config.step_deg = 360.0f; config.gear_ratio = 100.0f;
    config.direction = -1; config.pos_kp = config.speed_kp = 100.0f; config.speed_ki = 1000.0f;
    config.max_rpm = config.max_feedback_rpm = 6000.0f; config.current_limit = 3000.0f;
    config.tolerance_deg = 5.0f; config.settle_rpm = config.jam_rpm = 100.0f;
    config.settle_ms = 500U; config.motion_timeout_ms = 5000U;
    config.jam_current_raw = 3000.0f; config.jam_ms = 2000U;
    CHECK(FeederBackend_SetConfig(&config));
    rotor_rpm = -6000; feedback_current = -10000;
    for (ms = 1706U; ms <= 1725U; ++ms) { rotor_counts -= 800; Feed(ms); }
    CHECK(!state.controller.faults && state.controller.axis_position_deg < 0.0);
    memset(response, '~', sizeof(response));
    CHECK(FeederConsole_Handle("FEEDER STATUS", response, 1024U));
    CHECK(strlen(response) < 1023U && response[1024] == '~');
    CHECK(strstr(response, "|RPM|-6000|") && strstr(response, "|FB_TORQUE_RAW|-10000|"));
    CHECK(strstr(response, "|GEAR|100|") && strstr(response, "|TIMEOUT_MS|5000|"));
    CHECK(strstr(response, "|DONE_DT_MS|0\r\n"));
    FeederConsole_Telemetry(response, 1024U);
    test_max_wire_bytes = (uint32_t)strlen(response);
    test_max_wire_us = (test_max_wire_bytes * 10000000UL + APP_CONSOLE_BAUDRATE - 1U) / APP_CONSOLE_BAUDRATE;
    CHECK(test_max_wire_bytes < 1023U && test_max_wire_us <= APP_TELEMETRY_PERIOD_MS * 1000U);

    test_stage = 8U;
    Reset(1800U); config = state.controller.config; config.pos_kp = 6.0f;
    remote.fault_epoch = 2U; console_epoch = 3U;
    CHECK(!FeederBackend_SetConfig(&config)); /* Safety has not consumed epoch 5. */
    LogicApply(1801U, 1U);
    CHECK(app.logic.safety.uart_epoch == 5U && app.logic.safety.reason == SAFETY_UART);
    CHECK(!FeederBackend_SetConfig(&config)); /* Same-frame OFF is behind the barrier. */
    Tick(1802U); CHECK(app.logic.safety.state == SAFETY_READY && !app.logic.safety.barrier_valid);
    CHECK(FeederBackend_SetConfig(&config)); /* Cumulative nonzero epochs recover normally. */
    remote.needs_rearm = 1U; CHECK(!FeederBackend_SetConfig(&config)); remote.needs_rearm = 0U;
    ++remote.fault_epoch; CHECK(!FeederBackend_SetConfig(&config));
    LogicApply(1803U, 0U); CHECK(!FeederBackend_SetConfig(&config));
    Tick(1804U); CHECK(app.logic.safety.uart_epoch == 6U && FeederBackend_SetConfig(&config));
    ++console_epoch; CHECK(!FeederBackend_SetConfig(&config));
    Tick(1805U); CHECK(app.logic.safety.reason == SAFETY_UART && !FeederBackend_SetConfig(&config));
    Tick(1806U); CHECK(app.logic.safety.uart_epoch == 7U && FeederBackend_SetConfig(&config));

    test_stage = 9U;
    StartNormal(1900U, 1500U, 0U); CHECK(FeederBackend_Single());
    before = nonzero_calls; accepted = current_calls; stop_result = HAL_BUSY;
    SetRemote(1907U, 1500U, 1000U, 1500U);
    CHECK(state.armed && state.stop_pending && !state.controller.busy && !permission);
    CHECK(state.controller.have_zero && !state.controller.command_raw && !state.controller.integral);
    CHECK(!state.controller.accepted && state.controller.rejected == 1U && !rearm_calls);
    CHECK(nonzero_calls == before && !FeederBackend_Single());
    Tick(1908U); CHECK(state.armed && state.stop_pending && current_calls == accepted);
    stop_result = HAL_OK; Tick(1909U);
    CHECK(state.armed && !state.stop_pending && state.controller.have_zero && !rearm_calls);
    wheels.flywheels_ready = 0U;
    SetRemote(1910U, 1500U, 1500U, 2000U); Tick(1911U);
    CHECK(state.armed && !state.controller.accepted && !state.controller.busy);
    wheels.flywheels_ready = 1U; Tick(1912U);
    CHECK(state.armed && !state.controller.accepted && !state.controller.busy);
    SetRemote(1913U, 1500U, 1500U, 1500U);
    SetRemote(1914U, 1500U, 1500U, 2000U);
    CHECK(state.controller.accepted == 1U && state.controller.busy && state.armed);
    rotor_counts = 100; rotor_rpm = 600;
    SetRemote(1915U, 1500U, 1000U, 1500U);
    CHECK(!state.armed && !state.controller.busy && !state.controller.have_zero && !permission);
    CHECK(state.controller.aborted == 1U && rearm_calls == 1U && !app.logic.safety.operator_armed);
    CHECK(state.controller.position_deg > 4.0 && state.controller.target_deg == state.controller.position_deg);
    rotor_rpm = 0; SetRemote(1916U, 1500U, 1500U, 2000U); RunTo(1920U);
    CHECK(state.controller.accepted == 1U && !state.controller.have_zero && !state.controller.busy);
    test_stage = 10U;
    Reset(2100U); Feed(2101U); rotor_counts = 400; Feed(2102U);
    CHECK(state.controller.faults == FEEDER_FAULT_JUMP && state.controller.jump.valid);
    before = current_calls; accepted = stop_calls;
    memset(response, '~', sizeof(response));
    CHECK(FeederConsole_Handle("FEEDER CHECK", response, 1024U));
    CHECK(strstr(response, "F|CHECK|C610_ASSESSMENT_20HZ_V1E|FAULT|4|"));
    CHECK(strstr(response, "|JUMP_VALID|1|JUMP_COUNT|1|CAUSE|DELTA|FLAGS|1|"));
    CHECK(strstr(response, "|PREV_RAW|0|RAW|400|DELTA_COUNTS|400|GAP_MS|1|"));
    CHECK(strstr(response, "|FRAME|0190000000000000|"));
    CHECK(strstr(response, "|CH|1500,1500,1500,1500,1000,1000,1500,1500|"));
    CHECK(strstr(response, "|CAN_EPOCH|0\r\n") && strlen(response) < 1023U && response[1024] == '~');
    CHECK(current_calls == before && stop_calls == accepted && state.controller.faults == FEEDER_FAULT_JUMP);
    Feed(2103U); Feed(2104U); Feed(2105U);
    CHECK(FeederConsole_Handle("FEEDER CHECK", response, sizeof(response)));
    CHECK(strstr(response, "|AT_MS|2102|") && strstr(response, "|DELTA_COUNTS|400|"));
    Console("FEEDER CLEAR", 1U);
    CHECK(FeederConsole_Handle("FEEDER CHECK", response, sizeof(response)));
    CHECK(strstr(response, "|JUMP_VALID|0|JUMP_COUNT|1|CAUSE|NONE|"));
    CHECK(!state.controller.have_zero && !state.armed && !current_calls);
    StartNormal(2200U, 1500U, 1U); before = current_calls; accepted = stop_calls;
    CHECK(FeederConsole_Handle("FEEDER CHECK", response, sizeof(response)));
    Read(); CHECK(state.armed && state.controller.busy && current_calls == before && stop_calls == accepted);
    CHECK(strstr(response, "|ARMED|1|BUSY|1|CAL|1|DIR|1|STEP|40|"));
    test_done = 1U;
}
