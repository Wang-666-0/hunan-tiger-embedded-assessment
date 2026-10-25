/**
 * @file pitch_backend.c
 * @brief Pitch 会话、阶跃、归位与锁轴
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（26）：
 *   PitchBackend_Init / PitchBackend_GetState / PitchBackend_QueryFeedback / PitchBackend_Stop /
 *   PitchBackend_Request / PitchBackend_SetCoordinate / PitchBackend_CaptureReference /
 *   PitchBackend_OnFrame / PitchBackend_SetLoopPid / PitchBackend_SetLoopParameter /
 *   PitchBackend_SetTarget / PitchBackend_ReturnTo / PitchBackend_SetGravityGain /
 *   PitchBackend_SetGravityOffset / PitchBackend_StartStep / PitchBackend_SetShape /
 *   PitchBackend_SetBrakeEnabled / PitchBackend_GetStepId / PitchBackend_GetStep /
 *   PitchBackend_TakeStepReport / PitchBackend_TakeRemoteStepEvent / PitchBackend_TakeRearmRequest /
 *   PitchBackend_HoldCurrent / PitchBackend_SetViewLock / PitchBackend_PrepareViewHold /
 *   PitchBackend_Apply
 * 内部函数（6）：
 *   Online / StopWithReason / SetTarget / Notice / StationarySupport / TransferPositionSupport
 * 文件级变量/常量（16；局部变量见对应函数）：
 *   state：本模块维护并向外发布的设备或流程状态快照。
 *   step：当前动作基于真实反馈的阶跃成绩记录。
 *   remote_event：待向控制台交付的遥控动作事件快照。
 *   last_step_sequence：最近已经处理的遥控阶跃事件序号。
 *   last_tx_ms：最近一次控制命令的提交时间。
 *   enable_ms：最近使能/失能握手的起始时间。
 *   disable_ms：最近使能/失能握手的起始时间。
 *   last_query_ms：最近一次 Pitch 固定反馈查询时间。
 *   rearm_request：需要操作员重新解锁的待消费标记。
 *   remote_pending：是否存在尚未读取的遥控动作事件。
 *   cancel_pending：是否需要取消本轴尚未发送的旧命令。
 *   disable_submitted：Pitch 固定失能帧是否已经提交。
 *   actuation_started：本会话是否尝试过使能或观察到实际使能状态。
 *   last_mode：上一控制周期的遥控云台模式。
 *   submitted_torque：最近成功提交的实际控制力矩，单位 N·m。
 *   submitted_limited：最近提交力矩对应的饱和状态。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* Pitch 会话：使能、水平标定、锁轴和可中断归位。 */
#include "pitch_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "task.h"
#include <math.h>
#include <string.h>

static PitchBackendState state;
static PitchStep step;
static PitchRemoteStepEvent remote_event;
static uint32_t last_step_sequence, last_tx_ms, enable_ms, disable_ms, last_query_ms;
static uint8_t rearm_request, remote_pending, cancel_pending;
static uint8_t disable_submitted, actuation_started, last_mode;
static float submitted_torque;
static uint8_t submitted_limited;

static uint8_t Online(uint32_t now)
/* 功能：核对本轴反馈时间与当前 CAN 故障代次，判断设备是否在线。 */
{
    CanBusState bus;
    CanBus_GetState(&bus);
    return (uint8_t)(state.controller.have_feedback && bus.started && !bus.fault_epoch &&
        (uint32_t)(now - state.feedback.received_ms) < PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        state.feedback.fault_epoch == bus.fault_epoch);
}

void PitchBackend_Init(void)
/* 功能：初始化Pitch控制、会话、动作和故障状态。 */
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    PitchController_Init(&state.controller); PitchStep_Init(&step);
    memset(&remote_event, 0, sizeof(remote_event));
    last_step_sequence = last_tx_ms = enable_ms = disable_ms = last_query_ms = 0U;
    rearm_request = remote_pending = cancel_pending = 0U;
    disable_submitted = actuation_started = 0U; last_mode = GIMBAL_MANUAL;
    submitted_torque = 0.0f; submitted_limited = 0U;
    /* Do not transmit FC or FD merely because the MCU was reset. */
    taskEXIT_CRITICAL();
}

void PitchBackend_GetState(PitchBackendState *out)
/* 功能：在同步保护下复制Pitch实测反馈、会话和控制状态。 */
{
    uint32_t now;
    if (!out) { return; }
    taskENTER_CRITICAL(); now = HAL_GetTick();
    state.online = Online(now);
    state.feedback_age_ms = state.controller.have_feedback ? now - state.feedback.received_ms : 0U;
    *out = state; taskEXIT_CRITICAL();
}

uint8_t PitchBackend_QueryFeedback(void)
/* 功能：请求一帧 Pitch 固定反馈，供关闭状态监测和准备使用。 */
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = (uint8_t)(CanBus_PitchReadFeedback() == HAL_OK);
    if (result) { state.monitoring = 1U; last_query_ms = HAL_GetTick(); }
    taskEXIT_CRITICAL(); return result;
}

static void StopWithReason(PitchStopReason reason)
/* 功能：记录 Pitch 停止原因并撤销该轴会话、许可和旧输出。 */
{
    uint8_t had_session;
    taskENTER_CRITICAL();
    had_session = (uint8_t)(state.active || state.enabling || actuation_started);
    PitchStep_Abort(&step, HAL_GetTick(), state.controller.faults ? PITCH_STEP_FAULT : PITCH_STEP_STOP,
        state.controller.faults);
    CanBus_SetPitchPermission(0U);
    PitchController_Stop(&state.controller);
    state.requested = state.active = state.enabling = state.step_hold = 0U;
    state.view_hold_pending = state.view_locked;
    state.view_prepare_pending = 0U;
    state.view_hold_suspended = state.view_locked;
    last_mode = GIMBAL_MANUAL;
    if (had_session && !state.stop_pending) {
        ++state.stop_id; state.stopped_ms = HAL_GetTick(); state.stop_reason = reason;
        state.stop_pending = 1U; cancel_pending = 1U; disable_submitted = 0U;
        disable_ms = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_Stop(void)
/* 功能：撤销Pitch当前动作与发送许可，并管理原停止确认流程。 */
{ StopWithReason(PITCH_STOP_EXTERNAL); }

uint8_t PitchBackend_Request(uint8_t enabled)
/* 功能：更新Pitch会话请求；实际使能仍由实时许可、反馈及原握手条件决定。 */
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (!enabled) {
        /* Explicit OFF requests FD even if the user has only queried the motor. */
        actuation_started = 1U; PitchBackend_Stop(); accepted = 1U;
    } else if (APP_PITCH_ENABLED && state.coord_confirmed && state.zero_set &&
        state.controller.have_reference && Online(HAL_GetTick()) && state.feedback.status == 0U &&
        !state.controller.faults && !state.active && !state.enabling && !state.stop_pending) {
        state.requested = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetCoordinate(uint8_t divisor)
/* 功能：在关闭状态确认 Pitch 反馈坐标侧，并使旧参考在必要时失效。 */
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((divisor == 1U || divisor == 7U) && !state.requested && !state.active &&
        !state.enabling && !state.stop_pending && Online(HAL_GetTick()) && state.feedback.status == 0U) {
        accepted = PitchController_SetFeedbackDivisor(&state.controller, divisor);
        if (accepted) {
            state.coord_confirmed = 1U; state.zero_set = 0U;
            state.controller.have_reference = 0U;
            PitchController_InvalidateGravity(&state.controller);
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_TARGET_CHANGE, 0U);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_CaptureReference(void)
/* 功能：在关闭且满足原静止门禁时建立Pitch软件位置参考。 */
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (state.coord_confirmed && !state.requested && !state.active && !state.enabling &&
        !state.stop_pending && Online(HAL_GetTick()) && state.feedback.status == 0U) {
        uint32_t faults = state.controller.faults;
        /* Healthy disabled feedback can recover a prior enable timeout/disabled report.
         * TX/CAN faults remain latched; changing coordinates is not a bus recovery. */
        state.controller.faults &= ~(PITCH_BACKEND_FAULT_ENABLE | PITCH_BACKEND_FAULT_DISABLED);
        accepted = PitchController_CaptureReference(&state.controller);
        if (accepted) { state.zero_set = 1U; actuation_started = 0U; }
        else { state.controller.faults |= faults; }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_OnFrame(const CanFrame *frame)
/* 功能：接收Pitch有效新反馈，更新控制器、真实成绩和停止/故障状态。 */
{
    Dm3507Feedback feedback;
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    if (!Dm3507_Decode(frame, &feedback)) { return 0U; }
    taskENTER_CRITICAL();
    if ((uint32_t)(now - feedback.received_ms) < PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        (!state.controller.have_feedback || (int32_t)(feedback.received_ms - state.feedback.received_ms) > 0)) {
        CanBusState bus;
        CanBus_GetState(&bus);
        if (feedback.fault_epoch != bus.fault_epoch) { taskEXIT_CRITICAL(); return 0U; }
        state.feedback = feedback; ++state.rx_count;
        accepted = PitchController_OnFeedback(&state.controller, &feedback);
        if (feedback.status == 1U) { actuation_started = 1U; }
        if (!state.controller.have_reference) { state.zero_set = 0U; }
        /* Status0 confirms FD, not a brake or stationary mechanical support. */
        if (state.stop_pending && disable_submitted && !cancel_pending && feedback.status == 0U &&
            (int32_t)(feedback.received_ms - disable_ms) > 0) {
            state.stop_pending = 0U; actuation_started = 0U;
        }
        if (state.active && feedback.status == 0U) { state.controller.faults |= PITCH_BACKEND_FAULT_DISABLED; }
        if (step.phase == PITCH_STEP_RUNNING) {
            if (state.controller.faults) {
                PitchStep_Abort(&step, feedback.received_ms, PITCH_STEP_FAULT, state.controller.faults);
            } else if (state.active && feedback.status == 1U && CanBus_PitchAllowed()) {
                PitchStep_Sample(&step, feedback.received_ms, state.controller.angle_deg,
                    state.controller.velocity_dps, submitted_torque, submitted_limited);
            } else { PitchStep_Abort(&step, feedback.received_ms, PITCH_STEP_STOP, 0U); }
        }
        if (state.controller.faults && (state.active || state.enabling)) {
            StopWithReason(PITCH_STOP_FEEDBACK_FAULT); rearm_request = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetLoopPid(char loop, float kp, float ki, float kd)
/* 功能：校验并更新Pitch指定环 PID，参数变更时同步控制历史和真实阶跃状态。 */
{
    uint8_t accepted;
    uint32_t revision;
    taskENTER_CRITICAL(); revision = state.controller.cascade.revision;
    accepted = PitchController_SetPid(&state.controller, loop, kp, ki, kd);
    if (accepted && revision != state.controller.cascade.revision) {
        PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_PID_CHANGE, 0U);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetLoopParameter(char loop, char parameter, float value)
/* 功能：读取Pitch指定环参数，仅修改选中的 Kp、Ki 或 Kd 后走统一校验入口。 */
{
    YawCascadePid pid;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (loop == 'P' || loop == 'V') {
        pid = loop == 'P' ? state.controller.cascade.position : state.controller.cascade.speed;
        if (parameter == 'P') { pid.kp = value; }
        else if (parameter == 'I') { pid.ki = value; }
        else if (parameter == 'D') { pid.kd = value; }
        if (parameter == 'P' || parameter == 'I' || parameter == 'D') {
            accepted = PitchBackend_SetLoopPid(loop, pid.kp, pid.ki, pid.kd);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

static uint8_t SetTarget(float degrees, uint8_t returning)
/* 功能：按目标接管类型检查 Pitch 门禁，接受目标并同步当前阶跃统计。 */
{
    uint8_t accepted = 0U;
    uint32_t now;
    float previous;
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    if (!state.view_locked && state.active && state.requested && state.zero_set &&
        !state.enabling && !state.stop_pending && Online(now) &&
        state.feedback.status == 1U && !state.controller.faults && CanBus_PitchAllowed()) {
        previous = state.controller.target_deg;
        accepted = returning ? PitchController_ReturnTo(&state.controller, degrees, now) :
            PitchController_StartMove(&state.controller, degrees, now);
        if (accepted) {
            if (previous != degrees) { PitchStep_Abort(&step, now, PITCH_STEP_TARGET_CHANGE, 0U); }
            state.step_hold = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetTarget(float degrees)
/* 功能：在原门禁允许时接受 Pitch 位置目标，并同步真实动作统计。 */
{ return SetTarget(degrees, 0U); }
uint8_t PitchBackend_ReturnTo(float degrees)
/* 功能：立即接管 Pitch 归位目标，允许中断当前规划而不额外改写 PID 参数。 */
{ return SetTarget(degrees, 1U); }

uint8_t PitchBackend_SetGravityGain(float kg_nm)
/* 功能：在原状态门禁允许时修改 Pitch 重力补偿系数，并同步相关动作状态。 */
{
    uint8_t accepted = 0U;
    float previous;
    taskENTER_CRITICAL(); previous = state.controller.gravity_kg_nm;
    if (kg_nm == 0.0f || (state.coord_confirmed && state.zero_set && Online(HAL_GetTick()) &&
        !state.enabling && !state.stop_pending && !state.controller.faults &&
        ((!state.active && state.feedback.status == 0U) ||
         (state.active && state.feedback.status == 1U && CanBus_PitchAllowed())))) {
        accepted = PitchController_SetGravityGain(&state.controller, kg_nm);
        if (accepted && previous != state.controller.gravity_kg_nm) {
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_FF_CHANGE, 0U);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetGravityOffset(float offset_deg, uint8_t capture_horizontal)
/* 功能：在原状态门禁允许时更新 Pitch 水平偏置或捕获水平基准。 */
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (state.coord_confirmed && state.zero_set && Online(HAL_GetTick()) &&
        !state.active && !state.enabling && !state.stop_pending && state.feedback.status == 0U &&
        !state.controller.faults) {
        accepted = capture_horizontal ? PitchController_CaptureHorizontal(&state.controller) :
            PitchController_SetGravityOffset(&state.controller, offset_deg);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_StartStep(float degrees)
/* 功能：检查Pitch阶跃门禁，接受目标并建立基于真实反馈的成绩记录。 */
{
    uint8_t accepted = 0U;
    uint32_t now;
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    if (!state.view_locked && (degrees == 30.0f || degrees == -30.0f) && state.coord_confirmed && state.zero_set &&
        state.requested && state.active && state.controller.active && !state.enabling &&
        !state.stop_pending && Online(now) && state.feedback.status == 1U &&
        !state.controller.faults && CanBus_PitchAllowed() &&
        fabsf(state.controller.angle_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(state.controller.target_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(state.controller.velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        state.controller.stationary_samples >= 2U &&
        state.controller.last_feedback_ms - state.controller.stationary_started_ms >=
            PITCH_CONTROLLER_STATIONARY_HOLD_MS && step.phase != PITCH_STEP_RUNNING) {
        if (PitchController_StartMove(&state.controller, degrees, now)) {
            accepted = PitchStep_Start(&step, now, state.controller.angle_deg, degrees);
            if (accepted) {
                step.kp = state.controller.cascade.position.kp;
                step.ki = state.controller.cascade.position.ki;
                step.kd = state.controller.cascade.position.kd;
                step.vkp = state.controller.cascade.speed.kp;
                step.vki = state.controller.cascade.speed.ki;
                step.vkd = state.controller.cascade.speed.kd;
                step.gravity_kg_nm = state.controller.gravity_kg_nm;
                step.gravity_offset_deg = state.controller.gravity_offset_deg;
                step.gravity_calibrated = state.controller.gravity_calibrated;
                step.shape_ms = state.controller.shape.duration_ms;
                step.shape_velocity_gain = state.controller.shape.velocity_gain;
                step.pitch_brake_enabled = state.controller.brake.enabled;
                step.cascade = 1U; step.output_limits = 0U; step.vmax_dps = 0.0f;
                step.last_velocity_dps = state.controller.velocity_dps;
                step.last_torque_nm = submitted_torque;
                state.step_hold = 1U;
            }
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetShape(uint32_t duration_ms, float velocity_gain)
/* 功能：按原门禁配置 Pitch 可选全程轨迹时间和参考速度权重。 */
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (step.phase != PITCH_STEP_RUNNING ||
        (duration_ms == state.controller.shape.duration_ms &&
         velocity_gain == state.controller.shape.velocity_gain)) {
        accepted = PitchController_SetShape(&state.controller, duration_ms, velocity_gain);
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetBrakeEnabled(uint8_t enabled)
/* 功能：按原动作门禁更新 Pitch 制动开关，不修改 PID 参数。 */
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (step.phase != PITCH_STEP_RUNNING || enabled == state.controller.brake.enabled) {
        accepted = PitchController_SetBrakeEnabled(&state.controller, enabled);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint32_t PitchBackend_GetStepId(void)
/* 功能：读取Pitch最近接受的真实阶跃编号，用于双轴同步。 */
{
    uint32_t id;
    taskENTER_CRITICAL(); id = step.id; taskEXIT_CRITICAL();
    return id;
}
void PitchBackend_GetStep(PitchStep *out)
/* 功能：复制Pitch真实阶跃统计快照，不改变运动状态。 */
{ if (out) { taskENTER_CRITICAL(); *out = step; taskEXIT_CRITICAL(); } }
uint8_t PitchBackend_TakeStepReport(PitchStep *out)
/* 功能：取出Pitch待报告的真实阶跃记录，避免重复发布同一结果。 */
{
    uint8_t result = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (step.pending) { *out = step; step.pending = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t PitchBackend_TakeRemoteStepEvent(PitchRemoteStepEvent *out)
/* 功能：取出并清除一条Pitch遥控动作接收或拒绝事件。 */
{
    uint8_t result = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (remote_pending) { *out = remote_event; remote_pending = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t PitchBackend_TakeRearmRequest(void)
/* 功能：取出并清除Pitch后端要求重新解锁的单次标记。 */
{
    uint8_t result;
    taskENTER_CRITICAL(); result = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL(); return result;
}
static void Notice(const TargetRequest *request, uint8_t result)
/* 功能：记录一条 Pitch 遥控动作接收或拒绝事件，供控制台一次性读取。 */
{
    remote_event.sequence = request->step_sequence;
    remote_event.target_deg = result == 1U || result == 2U ? state.controller.target_deg :
        (request->step_return_event ? state.controller.target_deg + request->step_delta_deg : request->step_delta_deg);
    remote_event.result = result; remote_pending = 1U;
}

static uint8_t StationarySupport(void)
/* 功能：判断原目标附近是否已形成可用于保持的稳定支撑状态。 */
{
    const PitchController *pitch = &state.controller;
    return (uint8_t)(fabsf(pitch->target_deg - pitch->angle_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(pitch->velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        fabsf(pitch->cascade.velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        pitch->stationary_samples >= 2U &&
        pitch->last_feedback_ms - pitch->stationary_started_ms >= PITCH_CONTROLLER_STATIONARY_HOLD_MS);
}

static void TransferPositionSupport(void)
/* 功能：在满足静态条件时转移原位置积分支撑，减少进入保持时的输出突变。 */
{
    float next = state.controller.cascade.speed_i +
        state.controller.cascade.speed.kp * state.controller.cascade.position_i;
    if (next > PITCH_CONTROLLER_TORQUE_LIMIT_NM) { next = PITCH_CONTROLLER_TORQUE_LIMIT_NM; }
    else if (next < -PITCH_CONTROLLER_TORQUE_LIMIT_NM) { next = -PITCH_CONTROLLER_TORQUE_LIMIT_NM; }
    state.controller.cascade.speed_i = next;
    state.controller.cascade.position_i = 0.0f;
}

uint8_t PitchBackend_HoldCurrent(void)
/* 功能：满足原保持条件后锁定Pitch当前实测位置，保留原 PID 和观测器。 */
{
    uint8_t accepted = 0U;
    uint8_t carry_support;
    float target;
    taskENTER_CRITICAL();
    carry_support = StationarySupport();
    target = state.controller.angle_deg;
    if (target > PITCH_CONTROLLER_TARGET_MAX_DEG &&
        target <= PITCH_CONTROLLER_TARGET_MAX_DEG + PITCH_STEP_BAND_DEG) {
        target = PITCH_CONTROLLER_TARGET_MAX_DEG;
    } else if (target < PITCH_CONTROLLER_TARGET_MIN_DEG &&
        target >= PITCH_CONTROLLER_TARGET_MIN_DEG - PITCH_STEP_BAND_DEG) {
        target = PITCH_CONTROLLER_TARGET_MIN_DEG;
    }
    if (state.active && state.requested && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 1U && CanBus_PitchAllowed() &&
        !state.controller.faults) {
        if (state.view_locked && !state.view_hold_pending) { accepted = 1U; }
        else if (PitchController_SetTarget(&state.controller, target)) {
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_MANUAL, 0U);
            if (state.view_locked) {
                /* Only a settled near-target outer I is treated as load
                 * support. Motion I must not become a persistent torque bias
                 * after cancelling that movement and latching a new pose. */
                if (carry_support) { TransferPositionSupport(); }
                else { state.controller.cascade.position_i = 0.0f; }
                state.view_hold_deg = target;
            } else { state.controller.cascade.position_i = 0.0f; }
            /* Gravity and existing inner load compensation remain active. */
            state.step_hold = 1U; state.view_hold_pending = state.view_prepare_pending = 0U; accepted = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

void PitchBackend_SetViewLock(uint8_t locked)
/* 功能：更新Pitch持续视图保持请求，实际锁点在健康且已解锁后应用。 */
{
    taskENTER_CRITICAL(); locked = (uint8_t)(locked != 0U);
    if (state.view_locked != locked) {
        if (!locked && state.active && !state.view_hold_pending && !state.controller.faults) {
            if (StationarySupport()) { TransferPositionSupport(); }
            else { state.controller.cascade.position_i = 0.0f; }
        }
        state.view_locked = state.view_hold_pending = locked;
        state.view_prepare_pending = locked;
        state.view_hold_suspended = 0U;
        if (!locked) { state.step_hold = 0U; }
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_PrepareViewHold(void)
/* 功能：为 Pitch 视图保持恢复准备需求，固定锁点仍在满足原门禁后捕获。 */
{
    taskENTER_CRITICAL();
    if (!state.active && !state.enabling) {
        state.view_prepare_pending = 1U;
        state.view_hold_suspended = 0U;
        /* Refresh stopped-motor feedback through the existing read-only CC
         * path, rather than waiting forever on an expired last frame. */
        state.monitoring = 1U;
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t inhibited)
/* 功能：将Pitch目标请求与实时许可、反馈、使能及停止状态结合，计算并提交专用 CAN 输出。 */
{
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    uint8_t wanted, event = 0U, new_step = 0U;
    taskENTER_CRITICAL();
    if (request && request->step_event && request->step_sequence != last_step_sequence) {
        last_step_sequence = request->step_sequence;
        new_step = 1U;
        event = (uint8_t)(request->step_axis == GIMBAL_STEP_PITCH);
    }
    if ((state.active || state.enabling) && !Online(now)) {
        state.controller.faults |= PITCH_CONTROLLER_FAULT_FEEDBACK;
        StopWithReason(PITCH_STOP_FEEDBACK); rearm_request = 1U;
    }
    /* Selected view is durable intent, unlike V3's one-shot ticket which READY
     * could clear permanently. A stop made while permission is still live
     * suspends reconciliation. Observe blocked output (normal global STOP/OFF
     * and rearm path), or receive a fresh VIEW, before joining again. */
    if (!allowed) { state.view_hold_suspended = 0U; }
    if (state.view_locked && state.view_hold_pending && !state.requested &&
        !state.active && !state.enabling && !state.view_hold_suspended) {
        state.view_prepare_pending = 1U; state.monitoring = 1U;
    }
    /* Reuse Request + original FC/new status1 handshake. Existing reference,
     * calibration, faults, fresh feedback and global permission remain gates. */
    if (state.view_locked && state.view_hold_pending && !state.view_hold_suspended &&
        !state.requested && allowed && !inhibited && request &&
        state.coord_confirmed && state.zero_set && state.controller.have_reference &&
        !state.active && !state.enabling && !state.stop_pending && !state.controller.faults &&
        Online(now) && state.feedback.status == 0U &&
        (state.controller.gravity_kg_nm == 0.0f || state.controller.gravity_calibrated)) {
        if (PitchBackend_Request(1U)) { state.view_prepare_pending = 0U; }
    }
    wanted = (uint8_t)(APP_PITCH_ENABLED && state.requested && state.coord_confirmed &&
        state.zero_set && allowed && !inhibited && request && Online(now) &&
        !state.controller.faults && !state.stop_pending &&
        (state.controller.gravity_kg_nm == 0.0f || state.controller.gravity_calibrated));
    if (!wanted) {
        if (event) { Notice(request, 3U); }
        if (state.active || state.enabling) { StopWithReason(PITCH_STOP_PERMISSION); rearm_request = 1U; }
        CanBus_SetPitchPermission(0U);
        if (state.stop_pending && (!disable_submitted || now - last_tx_ms >= 10U)) {
            result = CanBus_PitchDisable(cancel_pending); last_tx_ms = now;
            if (result == HAL_OK) {
                if (!disable_submitted) { disable_ms = now; }
                disable_submitted = 1U; cancel_pending = 0U;
                submitted_torque = 0.0f; submitted_limited = 0U;
            }
        }
        /* The user's explicit CHECK/READY starts read-only monitoring. This keeps
         * disabled/ready feedback current while the user reads a summary or moves CH5.
         * No monitoring is started automatically on MCU reset. */
        if (state.monitoring && now - last_query_ms >= 10U) {
            if (CanBus_PitchReadFeedback() == HAL_OK) { last_query_ms = now; }
        }
        taskEXIT_CRITICAL(); return;
    }
    CanBus_SetPitchPermission(1U);
    if (!CanBus_PitchAllowed()) {
        StopWithReason(PITCH_STOP_CAN_PERMISSION); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (!state.active && !state.enabling) {
        if (event) { Notice(request, 3U); }
        /* Preserve the measured reference, but enable holding the current supported pose. */
        if (state.controller.angle_deg < PITCH_CONTROLLER_TARGET_MIN_DEG ||
            state.controller.angle_deg > PITCH_CONTROLLER_TARGET_MAX_DEG ||
            !PitchController_SetTarget(&state.controller, state.controller.angle_deg)) {
            state.controller.faults |= PITCH_CONTROLLER_FAULT_LIMIT;
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        result = CanBus_PitchEnable(); last_tx_ms = now;
        if (result == HAL_OK) {
            state.enabling = 1U; actuation_started = 1U; enable_ms = now;
        } else {
            state.controller.faults |= PITCH_BACKEND_FAULT_TX;
            StopWithReason(PITCH_STOP_ENABLE_TX); rearm_request = 1U;
        }
        taskEXIT_CRITICAL(); return;
    }
    if (state.enabling) {
        if (event) { Notice(request, 3U); }
        if (state.feedback.status == 1U && (int32_t)(state.feedback.received_ms - enable_ms) > 0) {
            state.enabling = 0U; state.active = state.controller.active = 1U;
        } else if (now - enable_ms >= APP_PITCH_ENABLE_TIMEOUT_MS) {
            state.controller.faults |= PITCH_BACKEND_FAULT_ENABLE;
            StopWithReason(PITCH_STOP_ENABLE_TIMEOUT); rearm_request = 1U;
        } else if (now - last_tx_ms >= 10U) {
            result = CanBus_PitchEnable(); last_tx_ms = now;
            if (result != HAL_OK) {
                state.controller.faults |= PITCH_BACKEND_FAULT_TX;
                StopWithReason(PITCH_STOP_ENABLE_TX); rearm_request = 1U;
            }
        }
        taskEXIT_CRITICAL(); return;
    }
    if (last_mode == GIMBAL_TEST && request->gimbal_mode != GIMBAL_TEST) {
        PitchStep_Abort(&step, now, PITCH_STEP_MANUAL, 0U);
        StopWithReason(PITCH_STOP_MODE_EXIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked && state.view_hold_pending && !PitchBackend_HoldCurrent()) {
        StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (event && state.view_locked) { Notice(request, 3U); event = 0U; }
    if (request->gimbal_mode == GIMBAL_TEST &&
        (last_mode != GIMBAL_TEST || (new_step && request->step_axis == GIMBAL_STEP_YAW))) {
        if (!PitchBackend_HoldCurrent()) {
            if (event) { Notice(request, 3U); }
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
    }
    last_mode = (uint8_t)request->gimbal_mode;
    if (event) {
        if (request->gimbal_mode != GIMBAL_TEST) { Notice(request, 3U); }
        else if (request->step_return_event) {
            Notice(request, PitchBackend_ReturnTo(state.controller.target_deg + request->step_delta_deg) ? 2U : 3U);
        } else if (PitchBackend_StartStep(request->step_delta_deg)) {
            Notice(request, 1U);
        } else { Notice(request, 3U); }
    }
    if (!state.view_locked && state.step_hold && request->gimbal_mode == GIMBAL_MANUAL && request->pitch_rate_dps != 0.0f) {
        PitchStep_Abort(&step, now, PITCH_STEP_MANUAL, 0U);
        StopWithReason(PITCH_STOP_MANUAL_OVERRIDE); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked) {
        /* The latched target is independent of disturbed feedback. Never
         * capture again until unlock/rearm; leave all original safety gates. */
        if ((state.controller.target_deg != state.view_hold_deg || state.controller.shape.active ||
             state.controller.brake.tracking) && !PitchController_SetTarget(&state.controller, state.view_hold_deg)) {
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        state.step_hold = 1U;
        (void)PitchController_UpdateViewHold(&state.controller, now, 1U);
    } else {
        (void)PitchController_Update(&state.controller,
            request->gimbal_mode == GIMBAL_MANUAL && !state.step_hold ? request->pitch_rate_dps : 0.0f, now, 1U);
    }
    if (state.controller.faults) {
        StopWithReason(PITCH_STOP_CONTROL_FAULT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (now - last_tx_ms >= APP_PITCH_COMMAND_PERIOD_MS) {
        result = CanBus_PitchTorque(state.controller.torque_nm); last_tx_ms = now;
        if (result != HAL_OK) {
            state.controller.faults |= PITCH_BACKEND_FAULT_TX;
            StopWithReason(PITCH_STOP_TORQUE_TX); rearm_request = 1U;
        } else {
            submitted_torque = state.controller.torque_nm;
            submitted_limited = state.controller.cascade.torque_limited;
            PitchStep_Command(&step, now, submitted_torque);
        }
    }
    taskEXIT_CRITICAL();
}
