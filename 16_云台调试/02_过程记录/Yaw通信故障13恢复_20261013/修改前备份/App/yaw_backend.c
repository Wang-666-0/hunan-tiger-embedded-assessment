#include "yaw_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>
#include <math.h>

static YawBackendState state;
static uint8_t monitoring, cancel_pending, sent, rearm_request;
static uint32_t last_tx_ms, enable_ms, disable_ms;
#define YAW_BACKEND_FAULT_ENABLE (1UL << 16)
#define YAW_BACKEND_FAULT_TX (1UL << 17)
#define YAW_BACKEND_FAULT_DISABLED (1UL << 18)

static uint8_t Online(uint32_t now)
{
    CanBusState bus;
    CanBus_GetState(&bus);
    return state.controller.have_feedback &&
        (uint32_t)(now - state.feedback.received_ms) < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        state.feedback.fault_epoch == bus.fault_epoch;
}

void YawBackend_Init(void)
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    YawController_Init(&state.controller);
    monitoring = cancel_pending = sent = rearm_request = 0U;
    last_tx_ms = enable_ms = disable_ms = 0U;
    taskEXIT_CRITICAL();
}

void YawBackend_GetState(YawBackendState *out)
{
    if (!out) { return; }
    taskENTER_CRITICAL();
    state.online = Online(HAL_GetTick());
    state.feedback_age_ms = state.controller.have_feedback ?
        (uint32_t)(HAL_GetTick() - state.feedback.received_ms) : 0U;
    *out = state;
    taskEXIT_CRITICAL();
}

void YawBackend_Stop(void)
{
    taskENTER_CRITICAL();
    CanBus_SetYawPermission(0U);
    YawController_Stop(&state.controller);
    state.active = state.enabling = 0U;
    if (monitoring) {
        state.stop_pending = 1U; cancel_pending = 1U; sent = 0U;
        disable_ms = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
}

uint8_t YawBackend_Request(uint8_t enabled)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (!enabled) {
        state.requested = 0U;
        YawBackend_Stop(); accepted = 1U;
    } else if (APP_YAW_ENABLED && bus.started && !bus.fault_epoch && !state.active && !state.enabling) {
        monitoring = 1U;
        /* 不通过ON清除未知故障；新鲜静止参考捕获才可清控制器故障。 */
        if (!state.requested && Online(HAL_GetTick()) && state.feedback.status == 0U) {
            uint32_t previous_faults = state.controller.faults;
            /* 人工OFF后，失能且静止可恢复使能超时/意外失能；TX/CAN仍须复位检查。 */
            state.controller.faults &= ~(YAW_BACKEND_FAULT_ENABLE | YAW_BACKEND_FAULT_DISABLED);
            if (!YawController_CaptureReference(&state.controller)) { state.controller.faults = previous_faults; }
        }
        if (!(state.controller.faults & (YAW_BACKEND_FAULT_ENABLE |
              YAW_BACKEND_FAULT_TX | YAW_BACKEND_FAULT_DISABLED))) {
            state.requested = 1U;
            YawBackend_Stop(); accepted = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_OnFrame(const CanFrame *frame)
{
    Gl60Feedback feedback;
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    if (!Gl60_Decode(frame, &feedback)) { return 0U; }
    taskENTER_CRITICAL();
    /* 接收时刻由ISR提供，旧队列帧不允许延长反馈寿命。 */
    if ((uint32_t)(now - feedback.received_ms) < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        (!state.controller.have_feedback ||
         (int32_t)(feedback.received_ms - state.feedback.received_ms) > 0)) {
        state.feedback = feedback; state.rx_count++;
        (void)YawController_OnFeedback(&state.controller, &feedback);
        accepted = 1U;
        if (state.stop_pending && feedback.status == 0U &&
            (int32_t)(feedback.received_ms - disable_ms) > 0 &&
            fabsf(feedback.velocity_rad_s * 57.2957795f) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS) {
            state.stop_pending = 0U;
        }
        if (state.active && feedback.status == 0U) {
            state.controller.faults |= YAW_BACKEND_FAULT_DISABLED;
        }
        if (state.controller.faults && (state.active || state.enabling)) {
            YawBackend_Stop(); rearm_request = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetParameter(char parameter, float value)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (parameter == 'P') {
        accepted = YawController_SetPid(&state.controller, value, state.controller.pid.ki, state.controller.pid.kd);
    } else if (parameter == 'I') {
        accepted = YawController_SetPid(&state.controller, state.controller.pid.kp, value, state.controller.pid.kd);
    } else if (parameter == 'D') {
        accepted = YawController_SetPid(&state.controller, state.controller.pid.kp, state.controller.pid.ki, value);
    } else if (parameter == 'T' && state.active && Online(HAL_GetTick()) && CanBus_YawAllowed()) {
        accepted = YawController_SetTarget(&state.controller, value);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_CaptureReference(void)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (!state.requested && !state.active && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 0U) {
        accepted = YawController_CaptureReference(&state.controller);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_TakeRearmRequest(void)
{
    uint8_t out;
    taskENTER_CRITICAL(); out = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL();
    return out;
}

void YawBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic)
{
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    uint8_t wanted;
    taskENTER_CRITICAL();
    if ((state.active || state.enabling) && !Online(now)) {
        state.controller.faults |= YAW_CONTROLLER_FAULT_FEEDBACK;
        YawBackend_Stop(); rearm_request = 1U;
    }
    wanted = APP_YAW_ENABLED && state.requested && allowed && !diagnostic && request &&
        Online(now) && !state.controller.faults && !state.stop_pending;
    if (!wanted) {
        if (state.active || state.enabling) { YawBackend_Stop(); }
        CanBus_SetYawPermission(0U);
        if (monitoring && (!sent || (uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS)) {
            result = CanBus_YawDisable(cancel_pending);
            last_tx_ms = now; sent = 1U;
            if (result == HAL_OK) { cancel_pending = 0U; }
            /* 失能提交失败保留pending/撤权，后续继续尝试。 */
            else { state.stop_pending = 1U; }
        }
        taskEXIT_CRITICAL(); return;
    }
    CanBus_SetYawPermission(1U);
    if (!CanBus_YawAllowed()) { YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return; }
    if (!state.active && !state.enabling) {
        if (!YawController_CaptureReference(&state.controller)) {
            YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        result = CanBus_YawEnable(); last_tx_ms = now; sent = 1U;
        if (result != HAL_OK) {
            state.controller.faults |= YAW_BACKEND_FAULT_TX;
            YawBackend_Stop(); rearm_request = 1U;
        } else { state.enabling = 1U; enable_ms = now; }
        taskEXIT_CRITICAL(); return;
    }
    if (state.enabling) {
        if (state.feedback.status == 1U && (int32_t)(state.feedback.received_ms - enable_ms) > 0) {
            state.enabling = 0U; state.active = 1U;
        } else if ((uint32_t)(now - enable_ms) >= APP_YAW_ENABLE_TIMEOUT_MS) {
            state.controller.faults |= YAW_BACKEND_FAULT_ENABLE;
            YawBackend_Stop(); rearm_request = 1U;
        } else if ((uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS) {
            if (CanBus_YawEnable() != HAL_OK) {
                state.controller.faults |= YAW_BACKEND_FAULT_TX;
                YawBackend_Stop(); rearm_request = 1U;
            }
            last_tx_ms = now;
        }
        taskEXIT_CRITICAL(); return;
    }
    /* 首版只开放正常模式与串口小阶跃；CH7的30度请求留待限位验收。 */
    (void)YawController_Update(&state.controller,
        request->gimbal_mode == GIMBAL_MANUAL ? request->yaw_rate_dps : 0.0f, now, 1U);
    if (state.controller.faults) {
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if ((uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS) {
        result = CanBus_YawTorque(state.controller.torque_nm); last_tx_ms = now;
        if (result != HAL_OK) {
            state.controller.faults |= YAW_BACKEND_FAULT_TX;
            YawBackend_Stop(); rearm_request = 1U;
        }
    }
    taskEXIT_CRITICAL();
}
