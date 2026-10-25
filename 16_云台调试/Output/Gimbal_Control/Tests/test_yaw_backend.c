#include "yaw_backend.h"
#include "can_bus.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static CanBusState bus;
static YawBackendState state;
static TargetRequest request;
static uint32_t now_ms, enable_calls, torque_calls, disable_calls, cancel_calls;
static uint8_t permission, deny_permission, pending, keep_pending;
static HAL_StatusTypeDef enable_result, torque_result, disable_result;
static float last_torque;
static uint32_t clear_calls;
static uint8_t wheel_permission;
static HAL_StatusTypeDef clear_result;

uint32_t HAL_GetTick(void) { return now_ms; }
void CanBus_GetState(CanBusState *out) { if (out) { *out = bus; } }
void CanBus_SetYawPermission(uint8_t allowed)
{ permission = (uint8_t)(allowed && !deny_permission && bus.started && !bus.fault_epoch); }
uint8_t CanBus_YawAllowed(void) { return permission; }
uint8_t CanBus_OutputAllowed(void) { return wheel_permission; }
uint32_t CanBus_YawTxPending(void) { return pending; }
HAL_StatusTypeDef CanBus_YawClearComms(void)
{
    ++clear_calls; CHECK(!permission && !wheel_permission && bus.started && !bus.fault_epoch);
    if (clear_result == HAL_OK) { pending = 1U; }
    return clear_result;
}
HAL_StatusTypeDef CanBus_YawEnable(void)
{ ++enable_calls; CHECK(permission && bus.started && !bus.fault_epoch); return enable_result; }
HAL_StatusTypeDef CanBus_YawTorque(float torque_nm)
{
    ++torque_calls; last_torque = torque_nm;
    CHECK(permission && bus.started && !bus.fault_epoch);
    CHECK(torque_nm >= -YAW_CONTROLLER_TORQUE_LIMIT_NM &&
          torque_nm <= YAW_CONTROLLER_TORQUE_LIMIT_NM); return torque_result;
}
HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel_old)
{
    ++disable_calls; permission = 0U;
    if (cancel_old) { ++cancel_calls; if (!keep_pending) { pending = 0U; } }
    if (!bus.started) { return HAL_ERROR; }
    if (pending) { return HAL_BUSY; }
    return disable_result;
}

static CanFrame Feedback(uint32_t timestamp, uint8_t status, uint8_t driver_temp, uint8_t motor_temp)
{
    CanFrame frame;
    memset(&frame, 0, sizeof(frame)); frame.bus = 1U; frame.dlc = 8U;
    frame.received_ms = timestamp; frame.fault_epoch = bus.fault_epoch;
    frame.data[0] = (uint8_t)((status << 4) | 1U);
    frame.data[1] = 0x7FU; frame.data[2] = 0xFFU;
    frame.data[3] = 0x7FU; frame.data[4] = 0xF7U; frame.data[5] = 0xFFU;
    frame.data[6] = driver_temp; frame.data[7] = motor_temp;
    return frame;
}
static void Feed(uint32_t timestamp, uint8_t status)
{
    CanFrame frame = Feedback(timestamp, status, 25U, 30U);
    now_ms = timestamp; CHECK(YawBackend_OnFrame(&frame) == 1U);
}
static void Read(void) { YawBackend_GetState(&state); }
static void Apply(uint32_t timestamp, uint8_t allowed)
{ now_ms = timestamp; YawBackend_Apply(&request, allowed, 0U); Read(); }
static void Reset(uint32_t timestamp)
{
    YawStep snapshot;
    memset(&bus, 0, sizeof(bus)); memset(&request, 0, sizeof(request));
    bus.started = 1U; now_ms = timestamp;
    enable_calls = torque_calls = disable_calls = cancel_calls = 0U;
    clear_calls = 0U; wheel_permission = 0U; clear_result = HAL_OK;
    permission = deny_permission = pending = keep_pending = 0U;
    enable_result = torque_result = disable_result = HAL_OK; last_torque = 0.0f;
    YawBackend_Init(); Read();
    YawBackend_GetStep(&snapshot);
    CHECK(snapshot.id == 0U && YawBackend_GetStepId() == snapshot.id);
}
static void Prepare(uint32_t timestamp)
{
    Reset(timestamp); CHECK(YawBackend_Request(1U) == 1U);
    Apply(timestamp, 0U);
    CHECK(disable_calls == 1U && cancel_calls == 1U && enable_calls == 0U && torque_calls == 0U);
    Feed(timestamp + 10U, 0U); Feed(timestamp + 30U, 0U);
    Apply(timestamp + 30U, 0U);
    CHECK(!state.active && !state.enabling && !state.stop_pending && !permission);
}
static void __attribute__((unused)) Activate(uint32_t timestamp)
{
    uint32_t t;
    Prepare(timestamp); Apply(timestamp + 32U, 1U);
    CHECK(enable_calls == 1U && state.enabling && !state.active && torque_calls == 0U);
    Feed(timestamp + 34U, 1U); Apply(timestamp + 34U, 1U);
    CHECK(state.active && !state.enabling && torque_calls == 0U);
    for (t = timestamp + 36U; t <= timestamp + 42U; t += 2U) { Apply(t, 1U); }
    CHECK(torque_calls == 1U && state.active && permission && state.controller.faults == 0U);
}
