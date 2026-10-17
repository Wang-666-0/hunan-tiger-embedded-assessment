#include "feeder_backend.h"
#include "app_config.h"
#include "app_tasks.h"
#include "app_console.h"
#include "can_bus.h"
#include "remote_input.h"
#include "motor_backend.h"
#include "task.h"
#include <string.h>

static FeederBackendState state;
static uint8_t owned, rearm, was_allowed, single_pending, have_completed;
static uint32_t last_tx_ms, last_success_ms, event_sequence, single_ms, completed_ms;

static uint8_t Fresh(void)
{
    return state.controller.have_feedback &&
        HAL_GetTick() - state.controller.last_feedback_ms < FEEDER_FEEDBACK_TIMEOUT_MS;
}

/* Caller holds the same critical section as mutation: a stale UI snapshot never authorizes motion. */
static uint8_t RemoteGuard(uint8_t off)
{
    RemoteInput_State remote;
    RemoteFrame frame;
    RemoteCommand command;
    AppState app;
    CanBusState bus;
    RemoteInput_GetState(&remote); AppTasks_GetState(&app); CanBus_GetState(&bus);
    memset(&frame, 0, sizeof(frame)); memcpy(frame.channels, remote.channels, sizeof(frame.channels));
    frame.has_frame = remote.has_frame;
    RemoteMapper_Map(&frame, &command);
    if (!remote.has_frame || HAL_GetTick() - remote.received_ms >= REMOTE_TIMEOUT_MS ||
        !command.valid || !command.failsafe_ok || command.gimbal_mode != GIMBAL_MANUAL ||
        AppConsole_IsDiagnosticMode() || remote.needs_rearm ||
        app.logic.safety.uart_epoch != remote.fault_epoch + AppConsole_GetFaultEpoch() ||
        !bus.started || bus.fault_epoch) { return 0U; }
    if (off) {
        return command.arm == RC_LOW && command.neutral &&
            app.logic.safety.state == SAFETY_READY && !app.logic.safety.barrier_valid &&
            !app.logic.safety.operator_armed &&
            !app.logic.safety.output_allowed && !CanBus_FeederAllowed();
    }
    return command.arm == RC_HIGH && command.trigger == RC_LOW && command.friction_on &&
        app.logic.safety.operator_armed && app.logic.safety.output_allowed;
}

void FeederBackend_Init(void)
{
    memset(&state, 0, sizeof(state)); Feeder_Init(&state.controller);
    owned = rearm = was_allowed = single_pending = have_completed = 0U;
    last_tx_ms = last_success_ms = event_sequence = single_ms = completed_ms = 0U;
}

void FeederBackend_Stop(void)
{
    taskENTER_CRITICAL();
    state.armed = was_allowed = single_pending = 0U;
    if (state.controller.busy) { state.controller.have_zero = 0U; }
    Feeder_Stop(&state.controller);
    CanBus_SetFeederPermission(0U);
    if (owned) { state.stop_pending = 1U; }
    taskEXIT_CRITICAL();
}

uint8_t FeederBackend_OnFrame(const CanFrame *frame)
{
    CanBusState bus;
    uint8_t accepted;
    uint32_t faults;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus); faults = state.controller.faults;
    accepted = Feeder_OnFrame(&state.controller, frame, HAL_GetTick(), bus.fault_epoch);
    if (state.armed && faults != state.controller.faults) { FeederBackend_Stop(); rearm = 1U; }
    taskEXIT_CRITICAL(); return accepted;
}

void FeederBackend_GetState(FeederBackendState *out)
{
    if (!out) { return; }
    taskENTER_CRITICAL(); *out = state; out->online = Fresh(); taskEXIT_CRITICAL();
}

uint8_t FeederBackend_TakeRearmRequest(void)
{
    uint8_t value;
    taskENTER_CRITICAL(); value = rearm; rearm = 0U; taskEXIT_CRITICAL(); return value;
}

uint8_t FeederBackend_SetConfig(const FeederConfig *config)
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.armed && !state.stop_pending && RemoteGuard(1U)) {
        result = Feeder_SetConfig(&state.controller, config);
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Clear(void)
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.armed && !state.stop_pending && RemoteGuard(1U)) {
        result = Feeder_Clear(&state.controller, HAL_GetTick());
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Zero(void)
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.armed && !state.stop_pending && RemoteGuard(1U)) {
        result = Feeder_Zero(&state.controller, HAL_GetTick());
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Arm(void)
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.stop_pending && RemoteGuard(1U) && Fresh() && !state.controller.faults &&
        state.controller.have_zero && state.controller.config.calibrated &&
        (state.controller.config.direction == 1 || state.controller.config.direction == -1)) {
        state.armed = owned = state.stop_pending = 1U;
        rearm = was_allowed = single_pending = 0U; result = 1U;
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Single(void)
{
    uint8_t result = 0U;
    MotorHealth wheels;
    taskENTER_CRITICAL(); MotorBackend_GetHealth(&wheels);
    if (state.armed && !state.stop_pending && !single_pending && !state.controller.busy &&
        Fresh() && !state.controller.faults && wheels.flywheels_ready && RemoteGuard(0U)) {
        single_pending = 1U; single_ms = HAL_GetTick(); result = 1U;
    }
    taskEXIT_CRITICAL(); return result;
}

void FeederBackend_Apply(const TargetRequest *request, uint8_t output_allowed,
                         uint8_t wheels_ready, uint8_t diagnostic)
{
    uint32_t now, accepted, completed, faults;
    uint8_t eligible, allowed, event, was_busy, console_single;
    int16_t current;
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL(); now = HAL_GetTick();
    was_busy = state.controller.busy; faults = state.controller.faults;
    eligible = request && output_allowed && !diagnostic && request->gimbal_mode == GIMBAL_MANUAL &&
        request->friction_requested;
    allowed = state.armed && eligible && wheels_ready && !state.stop_pending;
    console_single = single_pending;
    event = request && request->shot_event;
    if (single_pending) {
        if (now - single_ms <= APP_CONTROL_MAX_GAP_MS && allowed) { event = 1U; }
        else { state.controller.rejected++; }
        single_pending = 0U;
    }
    if (event) { event_sequence++; }
    accepted = state.controller.accepted; completed = state.controller.completed;
    current = Feeder_Step(&state.controller, now, allowed, wheels_ready, event, event_sequence);
    if (state.controller.accepted != accepted) {
        if (!console_single && request && request->fire_mode == FIRE_CONTINUOUS) { state.continuous_accepted++; }
        else { state.single_accepted++; }
    }
    if (state.controller.completed != completed) {
        state.completed_interval_ms = have_completed ? now - completed_ms : 0U;
        completed_ms = now; have_completed = 1U;
    }
    if ((was_busy && !state.controller.busy && state.controller.completed == completed) ||
        (state.armed && state.controller.faults != faults)) {
        if (was_busy && state.controller.completed == completed) { state.controller.have_zero = 0U; }
        FeederBackend_Stop(); rearm = 1U; allowed = 0U;
    } else if (was_allowed && !eligible) {
        if (request && output_allowed && !diagnostic && request->gimbal_mode == GIMBAL_MANUAL) {
            /* 空闲CH6普通关闭保留拨盘会话，沿用原遥控开关习惯；
             * 目标和串口事件已清，重新开轮仍需稳速与新鲜释放/触发。 */
            single_pending = 0U; CanBus_SetFeederPermission(0U); state.stop_pending = owned;
        } else { FeederBackend_Stop(); }
        allowed = 0U;
    }
    if (!allowed) {
        if (was_allowed) { CanBus_SetFeederPermission(0U); state.stop_pending = owned; }
        if (owned && (state.stop_pending || now - last_tx_ms >= 20U)) {
            result = CanBus_StopFeeder(); last_tx_ms = now;
            state.stop_pending = result != HAL_OK;
        }
    } else {
        if (!was_allowed) { last_success_ms = now; }
        CanBus_SetFeederPermission(1U);
        if (now - last_tx_ms >= APP_FEEDER_COMMAND_PERIOD_MS) {
            result = CanBus_FeederCurrent(current); last_tx_ms = now;
            if (result == HAL_OK) { last_success_ms = now; }
            if (result == HAL_ERROR || (result == HAL_BUSY && now - last_success_ms > 10U)) {
                state.controller.faults |= FEEDER_FAULT_CAN;
                state.controller.have_zero = 0U; FeederBackend_Stop(); rearm = 1U; allowed = 0U;
            }
        }
    }
    was_allowed = allowed;
    taskEXIT_CRITICAL();
}
