#include "yaw_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>
#include <math.h>

static YawBackendState state;
static YawStep step;
static YawRemoteStepEvent remote_step_event;
static uint32_t last_step_sequence;
static uint8_t remote_event_pending, step_from_remote;
static uint8_t monitoring, cancel_pending, sent, rearm_request;
static uint8_t preparation_hold;
static float last_submitted_torque_nm;
static uint32_t last_tx_ms, enable_ms, disable_ms;
static uint8_t clear_waiting_disable, clear_stationary_samples;
static uint32_t clear_stationary_since;
static uint32_t last_clear_blocks;
static uint8_t last_clear_result, last_disable_result;
static uint32_t off_motion_faults;
/* FC尝试或实际状态1即置位，STOP/OFF不清；健康失能静止参考捕获后开新会话。 */
static uint8_t actuation_started;
#define OFF_MOTION_FAULTS (YAW_CONTROLLER_FAULT_SPEED | YAW_CONTROLLER_FAULT_JUMP)
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

static uint8_t CaptureForStart(void)
{
    float reference = state.controller.reference_rad;
    uint8_t keep = state.zero_set && state.controller.have_reference;
    if (keep && fabsf(state.controller.angle_deg) > YAW_CONTROLLER_TARGET_LIMIT_DEG) {
        state.controller.faults |= YAW_CONTROLLER_FAULT_LIMIT; return 0U;
    }
    if (!YawController_CaptureReference(&state.controller)) { return 0U; }
    if (keep) {
        state.controller.reference_rad = reference;
        state.controller.angle_deg = (state.controller.continuous_rad - reference) * 57.2957795f;
        /* 保留人工零点，但使能时先保持当前位置，不自行追赶旧零点。 */
        state.controller.target_deg = state.controller.angle_deg;
    }
    return 1U;
}

void YawBackend_Init(void)
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    YawController_Init(&state.controller);
    YawStep_Init(&step);
    memset(&remote_step_event, 0, sizeof(remote_step_event));
    last_step_sequence = 0U; remote_event_pending = step_from_remote = 0U;
    state.controller.motion_guard_active = 0U;
    monitoring = cancel_pending = sent = rearm_request = 0U;
    preparation_hold = 0U;
    last_submitted_torque_nm = 0.0f;
    last_tx_ms = enable_ms = disable_ms = 0U;
    clear_waiting_disable = clear_stationary_samples = 0U;
    clear_stationary_since = 0U;
    last_clear_blocks = 0U; last_clear_result = 4U; /* 4=尚未尝试发送FB */
    last_disable_result = (uint8_t)HAL_ERROR;
    off_motion_faults = 0U; actuation_started = 0U;
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
    YawStep_Abort(&step, HAL_GetTick(), state.controller.faults ? YAW_STEP_FAULT : YAW_STEP_STOP,
                  state.controller.faults);
    state.step_hold = 0U;
    step_from_remote = 0U;
    CanBus_SetYawPermission(0U);
    YawController_Stop(&state.controller);
    state.active = state.enabling = 0U;
    clear_waiting_disable = 0U; /* 新STOP取消待完成的人工恢复，不续发清错。 */
    if (monitoring) {
        state.stop_pending = 1U; cancel_pending = 1U; sent = 0U;
        disable_ms = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
}

uint8_t YawBackend_PrepareStop(void)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (APP_YAW_ENABLED && bus.started && !bus.fault_epoch) {
        monitoring = 1U; state.requested = 0U; state.zero_set = 0U;
        YawBackend_Stop(); preparation_hold = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}

void YawBackend_ReleasePreparation(void)
{
    taskENTER_CRITICAL(); preparation_hold = 0U; taskEXIT_CRITICAL();
}

uint8_t YawBackend_Request(uint8_t enabled)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (!enabled) {
        preparation_hold = 0U;
        state.requested = 0U;
        YawBackend_Stop(); accepted = 1U;
    } else if (APP_YAW_ENABLED && bus.started && !bus.fault_epoch && !state.active && !state.enabling &&
               !clear_waiting_disable) {
        monitoring = 1U;
        /* 不通过ON清除未知故障；新鲜静止参考捕获才可清控制器故障。 */
        if (!state.requested && Online(HAL_GetTick()) && state.feedback.status == 0U) {
            uint32_t previous_faults = state.controller.faults;
            /* 人工OFF后，失能且静止可恢复使能超时/意外失能；TX/CAN仍须复位检查。 */
            state.controller.faults &= ~(YAW_BACKEND_FAULT_ENABLE | YAW_BACKEND_FAULT_DISABLED);
            if (!CaptureForStart()) { state.controller.faults |= previous_faults; }
            else {
                off_motion_faults = 0U; actuation_started = 0U;
                state.controller.motion_guard_active = 0U;
            }
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
        float delta = feedback.position_rad - state.feedback.position_rad;
        uint32_t gap = feedback.received_ms - state.feedback.received_ms;
        uint32_t previous_faults = state.controller.faults;
        float previous_torque = last_submitted_torque_nm;
        uint8_t previous_limited = state.controller.speed_limited;
        if (feedback.status == 1U) { actuation_started = 1U; }
        uint8_t off_motion = (uint8_t)(monitoring && !state.requested && !state.active &&
            !state.enabling && !state.controller.active && !actuation_started &&
            !CanBus_YawAllowed() && state.feedback.status == 13U && feedback.status == 13U);
        if (delta > GL60_POSITION_MAX_RAD) { delta -= 2.0f * GL60_POSITION_MAX_RAD; }
        else if (delta < -GL60_POSITION_MAX_RAD) { delta += 2.0f * GL60_POSITION_MAX_RAD; }
        /* 错误码13覆盖正常状态位，恢复用独立的连续静止观察；仍不视为失能确认。 */
        if (feedback.status == 13U && feedback.driver_temp_c < YAW_CONTROLLER_DRIVER_LIMIT_C &&
            feedback.motor_temp_c < YAW_CONTROLLER_MOTOR_LIMIT_C &&
            fabsf(feedback.velocity_rad_s * 57.2957795f) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS) {
            if (!clear_stationary_samples || state.feedback.status != 13U ||
                gap >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS ||
                fabsf(delta * 57.2957795f) > 0.05f + 0.5f * (float)gap / 1000.0f) {
                clear_stationary_since = feedback.received_ms; clear_stationary_samples = 1U;
            } else if (clear_stationary_samples < 255U) { clear_stationary_samples++; }
        } else { clear_stationary_samples = 0U; }
        state.feedback = feedback; state.rx_count++;
        (void)YawController_OnFeedback(&state.controller, &feedback);
        if (!state.controller.have_reference) { state.zero_set = 0U; }
        /* 仅标记新发生且来源明确的关闭状态运动记录。
         * 先去掉已被健康参考捕获清除的旧标签，不能把后来的运行故障误标。 */
        off_motion_faults &= previous_faults;
        if (off_motion) {
            off_motion_faults |= (state.controller.faults & ~previous_faults & OFF_MOTION_FAULTS);
        }
        accepted = 1U;
        if (state.stop_pending && !clear_waiting_disable && feedback.status == 0U &&
            (int32_t)(feedback.received_ms - disable_ms) > 0 &&
            fabsf(feedback.velocity_rad_s * 57.2957795f) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS) {
            state.stop_pending = 0U;
        }
        if (state.active && feedback.status == 0U) {
            state.controller.faults |= YAW_BACKEND_FAULT_DISABLED;
        }
        if (step.phase == YAW_STEP_RUNNING) {
            if (state.controller.faults) {
                float velocity = fabsf(feedback.velocity_rad_s * 57.2957795f);
                if (velocity > step.peak_velocity_dps) { step.peak_velocity_dps = velocity; }
                YawStep_Abort(&step, now, YAW_STEP_FAULT, state.controller.faults);
            } else if (state.active && feedback.status == 1U && CanBus_YawAllowed()) {
                YawStep_Sample(&step, feedback.received_ms, state.controller.angle_deg,
                    state.controller.velocity_dps, previous_torque, previous_limited);
            } else { YawStep_Abort(&step, now, YAW_STEP_STOP, 0U); }
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
    if (accepted) {
        YawStep_Abort(&step, HAL_GetTick(), parameter == 'T' ? YAW_STEP_TARGET_CHANGE : YAW_STEP_PID_CHANGE, 0U);
        if (parameter == 'T') { state.step_hold = 0U; step_from_remote = 0U; }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetPid(float kp, float ki, float kd)
{
    uint8_t accepted;
    taskENTER_CRITICAL();
    accepted = YawController_SetPid(&state.controller, kp, ki, kd);
    if (accepted) { YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_PID_CHANGE, 0U); }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_CaptureReference(void)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (!state.requested && !state.active && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 0U) {
        accepted = YawController_CaptureReference(&state.controller);
        if (accepted) {
            state.zero_set = 1U;
            off_motion_faults = 0U; actuation_started = 0U;
            state.controller.motion_guard_active = 0U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_StartStep(float degrees)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((degrees == 30.0f || degrees == -30.0f) && state.zero_set && state.requested &&
        state.active && state.controller.active && !state.enabling && !state.stop_pending &&
        !clear_waiting_disable && Online(HAL_GetTick()) && state.feedback.status == 1U &&
        !state.controller.faults && CanBus_YawAllowed() &&
        fabsf(state.controller.angle_deg) <= YAW_STEP_BAND_DEG &&
        fabsf(state.controller.target_deg) <= YAW_STEP_BAND_DEG &&
        fabsf(state.controller.velocity_dps) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS &&
        state.controller.stationary_samples >= 2U &&
        state.controller.last_feedback_ms - state.controller.stationary_started_ms >=
            YAW_CONTROLLER_STATIONARY_HOLD_MS && step.phase != YAW_STEP_RUNNING &&
        YawController_SetTarget(&state.controller, degrees)) {
        accepted = YawStep_Start(&step, HAL_GetTick(), state.controller.angle_deg, degrees);
        if (accepted) {
            step.kp = state.controller.pid.kp; step.ki = state.controller.pid.ki; step.kd = state.controller.pid.kd;
            step.last_velocity_dps = state.controller.velocity_dps;
            step.last_torque_nm = last_submitted_torque_nm;
            state.step_hold = 1U;
            step_from_remote = 0U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
void YawBackend_GetStep(YawStep *out)
{
    if (!out) { return; }
    taskENTER_CRITICAL(); *out = step; taskEXIT_CRITICAL();
}
uint8_t YawBackend_TakeStepReport(YawStep *out)
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (step.pending) { *out = step; step.pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}
uint8_t YawBackend_TakeRemoteStepEvent(YawRemoteStepEvent *out)
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (remote_event_pending) { *out = remote_step_event; remote_event_pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}
static void RemoteNotice(const TargetRequest *request, uint8_t result)
{
    remote_step_event.sequence = request->step_sequence;
    remote_step_event.target_deg = request->step_delta_deg;
    remote_step_event.result = result; remote_event_pending = 1U;
}

uint8_t YawBackend_TakeRearmRequest(void)
{
    uint8_t out;
    taskENTER_CRITICAL(); out = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL();
    return out;
}

void YawBackend_GetClearCheck(YawClearCheck *out)
{
    uint32_t now = HAL_GetTick();
    CanBusState bus;
    if (!out) { return; }
    taskENTER_CRITICAL();
    memset(out, 0, sizeof(*out)); CanBus_GetState(&bus);
    out->hold_ms = clear_stationary_samples ?
        (uint32_t)(state.feedback.received_ms - clear_stationary_since) : 0U;
    out->fd_age_ms = sent ? (uint32_t)(now - last_tx_ms) : 0U;
    out->mailbox_pending = CanBus_YawTxPending(); out->can_epoch = bus.fault_epoch;
    out->samples = clear_stationary_samples; out->monitoring = monitoring;
    out->fd_sent = sent; out->fd_cancel = cancel_pending;
    out->fd_result = last_disable_result; out->clear_result = last_clear_result;
    out->last_blocks = last_clear_blocks;
    out->off_motion_faults = off_motion_faults & state.controller.faults;
    out->actuation_started = actuation_started;
    out->required_hold_ms = (state.controller.faults & OFF_MOTION_FAULTS) ?
        YAW_CLEAR_OFF_MOTION_HOLD_MS : YAW_CONTROLLER_STATIONARY_HOLD_MS;
    if (!monitoring) { out->blocks |= YAW_CLEAR_NOT_MONITORING; }
    if (state.requested) { out->blocks |= YAW_CLEAR_REQUESTED; }
    if (state.active || state.enabling) { out->blocks |= YAW_CLEAR_ACTIVE; }
    if (clear_waiting_disable) { out->blocks |= YAW_CLEAR_WAITING_DISABLE; }
    if (CanBus_YawAllowed() || CanBus_OutputAllowed()) { out->blocks |= YAW_CLEAR_OUTPUT_ALLOWED; }
    if (!Online(now)) { out->blocks |= YAW_CLEAR_FEEDBACK_STALE; }
    if (state.feedback.status != 13U) { out->blocks |= YAW_CLEAR_STATUS_NOT_13; }
    if (!(state.controller.faults & YAW_CONTROLLER_FAULT_STATUS) ||
        (state.controller.faults & ~(YAW_CONTROLLER_FAULT_STATUS | out->off_motion_faults)) ||
        (actuation_started && (state.controller.faults & OFF_MOTION_FAULTS))) {
        out->blocks |= YAW_CLEAR_OTHER_FAULT;
    }
    if (clear_stationary_samples < 2U || out->hold_ms < out->required_hold_ms) {
        out->blocks |= YAW_CLEAR_NOT_STATIONARY;
    }
    if (!sent || cancel_pending || out->fd_age_ms >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
        out->blocks |= YAW_CLEAR_DISABLE_UNCONFIRMED;
    }
    if (!bus.started || bus.fault_epoch) { out->blocks |= YAW_CLEAR_CAN_FAULT; }
    if (out->mailbox_pending) { out->blocks |= YAW_CLEAR_MAILBOX_BUSY; }
    taskEXIT_CRITICAL();
}

uint8_t YawBackend_ClearCommsFault(void)
{
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    YawClearCheck check;
    taskENTER_CRITICAL();
    /* 单次人工FB；关闭监测的运动记录按来源和500ms静止门禁单独审核。
     * 软件锁存仍不在此清除，需后续新的FD/状态0和人工参考捕获。 */
    YawBackend_GetClearCheck(&check);
    last_clear_blocks = check.blocks; last_clear_result = 4U;
    if (!check.blocks) {
        result = CanBus_YawClearComms(); last_clear_result = (uint8_t)result;
        if (result == HAL_OK) {
            clear_waiting_disable = 1U;
            state.stop_pending = 1U; disable_ms = now;
            /* 不abort刚提交的FB；下一控制周期等待邮箱空后补FD，不重发FB。 */
            cancel_pending = 0U; sent = 0U; accepted = 1U;
        } else {
            last_clear_blocks = result == HAL_BUSY ? YAW_CLEAR_MAILBOX_BUSY : YAW_CLEAR_TX_FAILED;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

void YawBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic)
{
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    uint8_t wanted, new_step = 0U;
    taskENTER_CRITICAL();
    /* 事件不排队：未就绪/握手时消费并拒绝，不能稍后自动补发。 */
    if (request && request->step_event && request->step_sequence != last_step_sequence) {
        last_step_sequence = request->step_sequence; new_step = 1U;
    }
    if ((state.active || state.enabling) && !Online(now)) {
        state.controller.faults |= YAW_CONTROLLER_FAULT_FEEDBACK;
        YawBackend_Stop(); rearm_request = 1U;
    }
    wanted = APP_YAW_ENABLED && !preparation_hold && state.requested && allowed && !diagnostic && request &&
        Online(now) && !state.controller.faults && !state.stop_pending && !clear_waiting_disable;
    if (!wanted) {
        if (new_step) { RemoteNotice(request, 3U); }
        if (state.active || state.enabling) { YawBackend_Stop(); }
        CanBus_SetYawPermission(0U);
        if (monitoring && (!sent || (uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS)) {
            result = CanBus_YawDisable(cancel_pending);
            last_disable_result = (uint8_t)result;
            last_tx_ms = now; sent = 1U;
            if (result == HAL_OK) {
                cancel_pending = 0U; clear_waiting_disable = 0U; last_submitted_torque_nm = 0.0f;
            }
            /* 失能提交失败保留pending/撤权，后续继续尝试。 */
            else { state.stop_pending = 1U; }
        }
        taskEXIT_CRITICAL(); return;
    }
    CanBus_SetYawPermission(1U);
    if (!CanBus_YawAllowed()) { YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return; }
    if (!state.active && !state.enabling) {
        if (new_step) { RemoteNotice(request, 3U); }
        if (!CaptureForStart()) {
            YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        off_motion_faults = 0U; actuation_started = 1U;
        state.controller.motion_guard_active = 1U;
        result = CanBus_YawEnable(); last_tx_ms = now; sent = 1U;
        if (result != HAL_OK) {
            state.controller.faults |= YAW_BACKEND_FAULT_TX;
            YawBackend_Stop(); rearm_request = 1U;
        } else { state.enabling = 1U; enable_ms = now; }
        taskEXIT_CRITICAL(); return;
    }
    if (state.enabling) {
        if (new_step) { RemoteNotice(request, 3U); }
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
    if (state.step_hold && step_from_remote && request->gimbal_mode != GIMBAL_TEST) {
        YawStep_Abort(&step, now, YAW_STEP_MANUAL, 0U);
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (new_step) {
        if (request->gimbal_mode != GIMBAL_TEST) { RemoteNotice(request, 3U); }
        else if (request->step_return_event) {
            if (YawBackend_SetParameter('T', 0.0f)) { RemoteNotice(request, 2U); }
            else { RemoteNotice(request, 3U); }
        } else if (request->step_axis != GIMBAL_STEP_YAW) { RemoteNotice(request, 4U); }
        else if (YawBackend_StartStep(request->step_delta_deg)) {
            step_from_remote = 1U; RemoteNotice(request, 1U);
        } else { RemoteNotice(request, 3U); }
    }
    /* 测试模式保留TargetManager的CH4选方向、CH7确认/回参考；回中不清目标。 */
    if (state.step_hold && request->yaw_rate_dps != 0.0f) {
        YawStep_Abort(&step, now, YAW_STEP_MANUAL, 0U);
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
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
        } else {
            last_submitted_torque_nm = state.controller.torque_nm;
            YawStep_Command(&step, now, last_submitted_torque_nm);
        }
    }
    taskEXIT_CRITICAL();
}
