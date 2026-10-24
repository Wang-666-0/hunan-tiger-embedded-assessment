#include "gimbal_startup.h"
#include "app_config.h"
#include "app_logic.h"
#include "app_tasks.h"
#include "yaw_backend.h"
#include "pitch_backend.h"
#include "can_bus.h"
#include "yaw_console.h"
#include "pitch_console.h"
#include "remote_input.h"
#include <math.h>
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static CanBusState bus;
static AppLogic logic;
static SafetyInput input;
static YawBackendState yaw;
static PitchBackendState pitch;
static GimbalStartupState startup;
static uint32_t now, stops, rearm_barriers, yc, pc, yfd, pfd, cc, fb;
static uint8_t yp, pp, wheel_permission, device_yaw, device_pitch;
static uint8_t diagnostic, follow_target;
static float physical_yaw, physical_pitch;
static float FollowBounded(float actual, float wanted)
{
    float delta = wanted - actual;
    /* Synthetic 300deg/s follower, not an identified plant or a performance claim. */
    if (delta > 0.6f) { delta = 0.6f; } else if (delta < -0.6f) { delta = -0.6f; }
    return actual + delta;
}

uint32_t HAL_GetTick(void) { return now; }
void CanBus_GetState(CanBusState *out) { *out = bus; }
void CanBus_GetDiagnostics(CanBusDiagnostics *out) { memset(out, 0, sizeof(*out)); out->state = bus; }
void AppTasks_GetState(AppState *out) { memset(out, 0, sizeof(*out)); out->logic = logic; }
uint8_t AppConsole_IsDiagnosticMode(void) { return diagnostic; }
void RemoteInput_GetState(RemoteInput_State *out)
{
    memset(out, 0, sizeof(*out)); memcpy(out->channels, input.remote.channels, sizeof(out->channels));
    out->has_frame = input.remote.has_frame; out->received_ms = input.remote.received_ms;
    out->valid_frames = input.remote.sequence;
}
void CanBus_SetYawPermission(uint8_t value) { yp = (uint8_t)(value && bus.started && !bus.fault_epoch); }
void CanBus_SetPitchPermission(uint8_t value) { pp = (uint8_t)(value && bus.started && !bus.fault_epoch); }
uint8_t CanBus_YawAllowed(void) { return yp; }
uint8_t CanBus_PitchAllowed(void) { return pp; }
uint8_t CanBus_OutputAllowed(void) { return wheel_permission; }
uint32_t CanBus_YawTxPending(void) { return 0U; }
uint32_t CanBus_PitchTxPending(void) { return 0U; }
HAL_StatusTypeDef CanBus_CancelPitchTx(void) { return HAL_OK; }
HAL_StatusTypeDef CanBus_YawEnable(void)
{ CHECK(yp && !bus.fault_epoch); ++yc; device_yaw = 1U; return HAL_OK; }
HAL_StatusTypeDef CanBus_PitchEnable(void)
{ CHECK(pp && !bus.fault_epoch); ++pc; device_pitch = 1U; return HAL_OK; }
HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel)
{ (void)cancel; ++yfd; if (device_yaw != 13U) { device_yaw = 0U; } return HAL_OK; }
HAL_StatusTypeDef CanBus_PitchDisable(uint8_t cancel)
{ (void)cancel; ++pfd; device_pitch = 0U; return HAL_OK; }
HAL_StatusTypeDef CanBus_PitchReadFeedback(void) { ++cc; return HAL_OK; }
HAL_StatusTypeDef CanBus_YawClearComms(void)
{ CHECK(!yp && !pp && !wheel_permission); ++fb; device_yaw = 0U; return HAL_OK; }
HAL_StatusTypeDef CanBus_YawTorque(float value)
{ CHECK(yp && value >= -10.0f && value <= 10.0f); return HAL_OK; }
HAL_StatusTypeDef CanBus_PitchTorque(float value)
{ CHECK(pp && value >= -10.0f && value <= 10.0f); return HAL_OK; }
void AppTasks_RequireRearm(void) { ++stops; ++rearm_barriers; }
void AppTasks_RequestStop(void)
{ ++stops; yp = pp = 0U; YawBackend_Stop(); PitchBackend_Stop(); }
static void Read(void)
{ YawBackend_GetState(&yaw); PitchBackend_GetState(&pitch); GimbalStartup_GetState(&startup); }
static uint16_t Quantize(float value, float maximum, uint32_t scale)
{ return (uint16_t)((value + maximum) * (float)scale / (2.0f * maximum)); }
static CanFrame Frame(uint8_t motor, uint8_t status, float angle)
{
    CanFrame out;
    uint16_t p = Quantize(angle * 3.141593f / 180.0f, 3.141593f, 65535U);
    uint16_t v = Quantize(0.0f, 200.0f, 4095U), t = Quantize(0.0f, 10.0f, 4095U);
    memset(&out, 0, sizeof(out)); out.bus = 1U; out.id = motor == 2U ? 2U : 0U;
    out.dlc = 8U; out.received_ms = now; out.fault_epoch = bus.fault_epoch;
    out.data[0] = (uint8_t)((status << 4) | motor);
    out.data[1] = (uint8_t)(p >> 8); out.data[2] = (uint8_t)p;
    out.data[3] = (uint8_t)(v >> 4); out.data[4] = (uint8_t)((v << 4) | (t >> 8));
    out.data[5] = (uint8_t)t; out.data[6] = 25U; out.data[7] = 30U; return out;
}
static void Tick(uint32_t ms, uint16_t ch5, uint8_t remote_fresh, uint8_t yaw_feedback, uint8_t pitch_feedback)
{
    CanFrame frame;
    AppAxisHealth yh, ph;
    TargetRequest request;
    now = ms; Read();
    if (follow_target && device_yaw == 1U) { physical_yaw = FollowBounded(physical_yaw, 10.0f + yaw.controller.target_deg); }
    if (follow_target && device_pitch == 1U) { physical_pitch = FollowBounded(physical_pitch, 100.0f + pitch.controller.target_deg); }
    if (ms % 10U == 0U) {
        if (yaw_feedback) { frame = Frame(1U, device_yaw, physical_yaw); CHECK(YawBackend_OnFrame(&frame)); }
        if (pitch_feedback) { frame = Frame(2U, device_pitch, physical_pitch); CHECK(PitchBackend_OnFrame(&frame)); }
    }
    if (remote_fresh) {
        input.remote.has_frame = 1U; input.remote.received_ms = now; ++input.remote.sequence;
        input.remote.channels[4] = ch5;
    }
    input.now_ms = now; input.can_epoch = bus.fault_epoch; input.stop_sequence = stops;
    GimbalStartup_Update(&input, diagnostic); Read();
    memset(&yh, 0, sizeof(yh)); memset(&ph, 0, sizeof(ph));
    yh.requested = yaw.requested; yh.online = yaw.online; yh.stop_pending = yaw.stop_pending;
    yh.faulted = (uint8_t)(yaw.controller.faults != 0U);
    ph.requested = pitch.requested; ph.online = pitch.online; ph.stop_pending = pitch.stop_pending;
    ph.faulted = (uint8_t)(pitch.controller.faults != 0U);
    input.backend_ready = input.feedback_ready = input.actuator_fault = 0U;
    AppLogic_CombineActuatorHealth(&input, &yh, &ph, 0U);
    if (!GimbalStartup_OutputReady()) { input.feedback_ready = 0U; }
    input.stop_sequence = stops;
    AppLogic_Step(&logic, &input, 0U); request = logic.targets.request;
    YawBackend_Apply(&request, logic.safety.output_allowed, diagnostic);
    if (YawBackend_TakeRearmRequest()) {
        AppTasks_RequestStop(); logic.safety.output_allowed = logic.safety.operator_armed = 0U;
    }
    PitchBackend_Apply(&request, logic.safety.output_allowed, diagnostic);
    if (PitchBackend_TakeRearmRequest()) {
        AppTasks_RequestStop(); logic.safety.output_allowed = logic.safety.operator_armed = 0U;
    }
    Read();
}
static void Run(uint32_t duration, uint16_t ch5)
{
    uint32_t until = now + duration;
    while ((int32_t)(until - now) > 0) { Tick(now + 2U, ch5, 1U, 1U, 1U); }
}
static void Reset(uint32_t ms)
{
    uint32_t i;
    now = ms; memset(&bus, 0, sizeof(bus)); bus.started = 1U;
    memset(&input, 0, sizeof(input)); AppLogic_Init(&logic);
    for (i = 0U; i < 14U; ++i) { input.remote.channels[i] = 1500U; }
    input.remote.channels[4] = input.remote.channels[5] = 1000U;
    yp = pp = wheel_permission = device_yaw = device_pitch = diagnostic = follow_target = 0U;
    yc = pc = yfd = pfd = cc = fb = stops = rearm_barriers = 0U;
    physical_yaw = 10.0f; physical_pitch = 100.0f;
    YawConsole_CancelPreparation(); YawBackend_Init(); PitchBackend_Init(); PitchConsole_Init(); GimbalStartup_Init(); Read();
}
void test_run(void)
{
    uint32_t before_y, before_p, t;
    float yref, pref, held_y, held_p;
    GimbalStartupState event;
    char reply[1024];
    test_stage = 1U;
    CHECK(APP_GIMBAL_REMOTE_STARTUP_ENABLED && APP_PITCH_RATE_LIMIT_DPS == 90.0f);
    Reset(0U); Run(200U, 1500U);
    CHECK(!yc && !pc && !cc && !yfd && !pfd && !startup.ready);
    Run(400U, 1000U);
    CHECK(startup.ready && startup.reference_latched && yaw.zero_set && pitch.zero_set);
    CHECK(pitch.controller.gravity_calibrated && pitch.controller.gravity_kg_nm == -0.13f);
    CHECK(fabsf(yaw.controller.angle_deg) < 0.01f && fabsf(pitch.controller.angle_deg) < 0.01f);
    CHECK(!yc && !pc && yfd && pfd && cc && rearm_barriers);
    CHECK(GimbalStartup_TakeEvent(&event) && event.ready && !GimbalStartup_TakeEvent(&event));
    Run(100U, 1500U); CHECK(yaw.active && pitch.active && yc == 1U && pc == 1U);
    follow_target = 1U; input.remote.channels[0] = input.remote.channels[1] = 1765U;
    held_y = yaw.controller.target_deg; held_p = pitch.controller.target_deg;
    Run(100U, 1500U);
    CHECK(fabsf(yaw.controller.target_deg - held_y - 4.5f) < 0.01f);
    CHECK(fabsf(pitch.controller.target_deg - held_p - 4.5f) < 0.01f);
    input.remote.channels[0] = input.remote.channels[1] = 1500U;
    held_y = yaw.controller.target_deg; held_p = pitch.controller.target_deg;
    Run(100U, 1500U);
    CHECK(yaw.controller.target_deg == held_y && pitch.controller.target_deg == held_p);

    test_stage = 2U;
    yref = yaw.controller.reference_rad; pref = pitch.controller.reference_rad;
    before_y = yc; before_p = pc; follow_target = 0U;
    Run(200U, 1000U);
    CHECK(!yaw.active && !pitch.active && startup.ready && yaw.requested && pitch.requested);
    CHECK(yaw.controller.reference_rad == yref && pitch.controller.reference_rad == pref);
    CHECK(pitch.controller.gravity_calibrated);
    Run(100U, 1500U); CHECK(yaw.active && pitch.active && yc == before_y + 1U && pc == before_p + 1U);
    CHECK(yaw.controller.reference_rad == yref && pitch.controller.reference_rad == pref);

    test_stage = 3U;
    before_y = yc; before_p = pc;
    for (t = 0U; t < 130U; t += 2U) { Tick(now + 2U, 1500U, 0U, 1U, 1U); }
    CHECK(!yaw.active && !pitch.active && !logic.safety.operator_armed);
    Run(100U, 1500U); CHECK(yc == before_y && pc == before_p);
    Run(200U, 1000U); Run(100U, 1500U);
    CHECK(yaw.active && pitch.active && yaw.controller.reference_rad == yref && pitch.controller.reference_rad == pref);

    test_stage = 4U;
    before_y = yc; before_p = pc;
    for (t = 0U; t < 130U; t += 2U) { Tick(now + 2U, 1500U, 1U, 1U, 0U); }
    Run(100U, 1500U); Run(200U, 1000U);
    CHECK(!yaw.active && !pitch.active && yc == before_y && pc == before_p);
    CHECK(startup.phase == GIMBAL_STARTUP_BLOCKED && startup.reason == GIMBAL_STARTUP_REFERENCE_LOST);
    CHECK(pitch.controller.gravity_kg_nm == -0.13f && !pitch.controller.gravity_calibrated);
    physical_yaw = 10.0f; physical_pitch = 100.0f; Run(50U, 1000U);
    CHECK(GimbalStartup_Request(&input.remote, now, 0U)); AppTasks_RequestStop(); Run(400U, 1000U);
    CHECK(startup.ready && pitch.controller.gravity_calibrated && !pitch.controller.faults);
    Run(100U, 1500U); CHECK(yaw.active && pitch.active);

    test_stage = 5U;
    Reset(10000U); Run(108U, 1000U); CHECK(startup.phase == GIMBAL_STARTUP_PREPARING);
    Run(100U, 1500U); CHECK(!yc && !pc && !startup.ready);
    Run(400U, 1000U); CHECK(startup.ready && !yc && !pc);
    Run(100U, 1500U); CHECK(yaw.active && pitch.active);

    test_stage = 6U;
    Reset(20000U); diagnostic = 1U; Run(300U, 1000U);
    CHECK(!cc && !yfd && !pfd && !yc && !pc);
    diagnostic = 0U; Run(400U, 1000U); CHECK(startup.ready);
    GimbalStartup_ManualOverride(); AppTasks_RequestStop(); before_y = yc; before_p = pc;
    Run(300U, 1000U); CHECK(startup.phase == GIMBAL_STARTUP_MANUAL && !pitch.requested);
    CHECK(yc == before_y && pc == before_p);
    CHECK(!GimbalStartup_Request(&input.remote, now, 1U));
    input.remote.channels[4] = 1500U; CHECK(!GimbalStartup_Request(&input.remote, now, 0U));
    input.remote.channels[4] = 1000U; input.remote.received_ms = now - 100U;
    CHECK(!GimbalStartup_Request(&input.remote, now, 0U));

    test_stage = 7U;
    Reset(30000U); device_yaw = 13U; Run(800U, 1000U);
    CHECK(fb == 1U && startup.ready && !yc && !pc && !yaw.controller.faults);
    Run(100U, 1500U); CHECK(yaw.active && pitch.active);
    Reset(40000U); bus.started = 0U; Run(100U, 1000U); CHECK(!cc && !yfd && !pfd);
    bus.started = 1U; Run(400U, 1000U); CHECK(startup.ready);
    Reset(50000U); bus.fault_epoch = 1U; Run(400U, 1000U);
    CHECK(startup.phase == GIMBAL_STARTUP_BLOCKED && !yc && !pc && !cc);
    test_stage = 8U;
    Reset(60000U); Run(400U, 1000U); Run(100U, 1500U); CHECK(yaw.active && pitch.active);
    CHECK(PitchConsole_Handle("PITCH STEP", reply, sizeof(reply)) && strstr(reply, "P|OK|STEP_STARTED|TARGET|30.0"));
    Read(); CHECK(pitch.controller.target_deg == 30.0f && yaw.controller.target_deg == 0.0f);
    follow_target = 1U; Run(50U, 1500U);
    CHECK(PitchConsole_Handle("PITCH HOME", reply, sizeof(reply)) && strstr(reply, "P|OK|HOME|TARGET|0"));
    Run(100U, 1500U); Read(); CHECK(pitch.controller.target_deg == 0.0f && pitch.active);
    Run(30U, 2000U);
    CHECK(YawConsole_Handle("YAW STEP -30", reply, sizeof(reply)) && strstr(reply, "Y|OK|STEP_STARTED|TARGET|-30.0"));
    Run(50U, 2000U);
    CHECK(YawConsole_Handle("YAW HOME", reply, sizeof(reply)) && strstr(reply, "Y|OK|HOME|TARGET|0"));
    Run(100U, 2000U); Read(); CHECK(yaw.controller.target_deg == 0.0f && yaw.active && pitch.active);
    CHECK(PitchConsole_Handle("PITCH STEP -30", reply, sizeof(reply)) && strstr(reply, "STEP_OUTSIDE_MECHANICAL_RANGE"));
    CHECK(PitchConsole_Handle("PITCH STEP 30 extra", reply, sizeof(reply)) && strstr(reply, "P|ERROR|"));
    Run(30U, 1000U);
    CHECK(!PitchConsole_Handle("YAW HOME", reply, sizeof(reply)));
    CHECK(YawConsole_Handle("YAW HOME", reply, sizeof(reply)) && strstr(reply, "Y|ERROR|"));
    CHECK(PitchConsole_Handle("PITCH HOME", reply, sizeof(reply)) && strstr(reply, "P|ERROR|"));
    test_stage = 9U;
    Reset(70000U); input.remote.channels[1] = 1765U; Run(400U, 1000U);
    CHECK(!cc && !yfd && !pfd && !yc && !pc);
    input.remote.channels[1] = 1500U; input.remote.channels[7] = 1000U; Run(400U, 1000U);
    CHECK(!cc && !yfd && !pfd);
    input.remote.channels[7] = 1500U; input.remote.channels[5] = 2000U; Run(400U, 1000U);
    CHECK(!cc && !yfd && !pfd);
    input.remote.channels[5] = 1000U; Run(400U, 1000U); CHECK(startup.ready);
    Run(100U, 1500U); input.remote.channels[1] = 1765U;
    CHECK(PitchConsole_Handle("PITCH STEP", reply, sizeof(reply)) && strstr(reply, "P|ERROR|"));
    CHECK(YawConsole_Handle("YAW STEP", reply, sizeof(reply)) && strstr(reply, "Y|ERROR|"));

    test_stage = 10U;
    Reset(80000U);
    for (t = 0U; t < 5300U; t += 10U) { Tick(now + 10U, 1000U, 1U, 1U, 0U); }
    CHECK(startup.phase == GIMBAL_STARTUP_BLOCKED && startup.reason == GIMBAL_STARTUP_TIMEOUT);
    CHECK(!yc && !pc && !pitch.zero_set);
    Run(300U, 1500U); CHECK(!yc && !pc); /* Late feedback must not complete an old unlock. */
    test_done = 1U;
}
