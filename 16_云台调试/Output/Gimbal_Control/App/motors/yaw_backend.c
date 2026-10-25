/**
 * @file yaw_backend.c
 * @brief Yaw 会话、阶跃、归位与锁轴
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（23）：
 *   YawBackend_Init / YawBackend_GetState / YawBackend_Stop / YawBackend_PrepareStop /
 *   YawBackend_ReleasePreparation / YawBackend_Request / YawBackend_OnFrame /
 *   YawBackend_SetParameter / YawBackend_SetLoopPid / YawBackend_SetLoopParameter /
 *   YawBackend_CaptureReference / YawBackend_StepBlocks / YawBackend_StartStep /
 *   YawBackend_GetStepId / YawBackend_GetStep / YawBackend_TakeStepReport /
 *   YawBackend_TakeRemoteStepEvent / YawBackend_TakeRearmRequest / YawBackend_GetClearCheck /
 *   YawBackend_ClearCommsFault / YawBackend_HoldCurrent / YawBackend_SetViewLock / YawBackend_Apply
 * 内部函数（4）：
 *   Online / StepBlocks / CaptureForStart / RemoteNotice
 * 文件级变量/常量（26；局部变量见对应函数）：
 *   state：本模块维护并向外发布的设备或流程状态快照。
 *   step：当前动作基于真实反馈的阶跃成绩记录。
 *   previous_step：新动作开始时保留的上一条未读或中断成绩。
 *   remote_step_event：待向控制台交付的遥控动作事件快照。
 *   last_step_sequence：最近已经处理的遥控阶跃事件序号。
 *   remote_event_pending：是否存在尚未读取的遥控动作事件。
 *   step_from_remote：当前阶跃保持是否来源于遥控动作。
 *   last_mode：上一控制周期的遥控云台模式。
 *   monitoring：Yaw 当前是否允许进行关闭状态反馈监测。
 *   cancel_pending：是否需要取消本轴尚未发送的旧命令。
 *   sent：本会话是否已经提交过原使能命令。
 *   rearm_request：需要操作员重新解锁的待消费标记。
 *   preparation_hold：人工准备流程对正常运行的保持门禁。
 *   last_submitted_torque_nm：最近成功提交的实际控制力矩，单位 N·m。
 *   last_submitted_limited：最近提交力矩对应的饱和状态。
 *   last_tx_ms：最近一次控制命令的提交时间。
 *   enable_ms：最近使能/失能握手的起始时间。
 *   disable_ms：最近使能/失能握手的起始时间。
 *   clear_waiting_disable：清错前是否仍在等待可靠失能确认。
 *   clear_stationary_samples：清错前连续静止反馈样本数。
 *   clear_stationary_since：清错静止观察开始时间。
 *   last_clear_blocks：最近一次清错尝试的阻塞条件。
 *   last_clear_result：最近清错/失能提交的 HAL 返回结果。
 *   last_disable_result：最近清错/失能提交的 HAL 返回结果。
 *   off_motion_faults：已经辨明发生在关闭状态下的运动故障位。
 *   actuation_started：本会话是否尝试过使能或观察到实际使能状态。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* Yaw 会话：许可、握手、相对阶跃、归位与锁轴。 */
#include "yaw_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>
#include <math.h>

static YawBackendState state;
static YawStep step;
/* Preserve the latest interrupted/unread report when starting a new step. */
static YawStep previous_step;
static YawRemoteStepEvent remote_step_event;
static uint32_t last_step_sequence;
static uint8_t remote_event_pending, step_from_remote;
static GimbalMode last_mode;
static uint8_t monitoring, cancel_pending, sent, rearm_request;
static uint8_t preparation_hold;
static float last_submitted_torque_nm;
static uint8_t last_submitted_limited;
static uint32_t last_tx_ms, enable_ms, disable_ms;
static uint8_t clear_waiting_disable, clear_stationary_samples;
static uint32_t clear_stationary_since;
static uint32_t last_clear_blocks;
static uint8_t last_clear_result, last_disable_result;
static uint32_t off_motion_faults;
/* FC尝试或实际状态1即置位，STOP/OFF不清；健康失能静止参考捕获后开新会话。 */
static uint8_t actuation_started;

static uint8_t Online(uint32_t now)
/* 功能：核对本轴反馈时间与当前 CAN 故障代次，判断设备是否在线。 */
{
    CanBusState bus;
    CanBus_GetState(&bus);
    return state.controller.have_feedback &&
        (uint32_t)(now - state.feedback.received_ms) < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        state.feedback.fault_epoch == bus.fault_epoch;
}

static uint32_t StepBlocks(uint32_t now)
/* 功能：汇总 Yaw 正式阶跃所需的会话、位置、静止、限位和视图门禁。 */
{
    uint32_t blocks = 0U;
    if (!state.zero_set || !state.controller.have_reference) { blocks |= YAW_STEP_BLOCK_REFERENCE; }
    if (!state.requested || !state.active || !state.controller.active) { blocks |= YAW_STEP_BLOCK_INACTIVE; }
    if (state.enabling || state.stop_pending || clear_waiting_disable) { blocks |= YAW_STEP_BLOCK_TRANSITION; }
    if (!Online(now) || state.feedback.status != 1U) { blocks |= YAW_STEP_BLOCK_FEEDBACK; }
    if (state.controller.faults) { blocks |= YAW_STEP_BLOCK_FAULT; }
    if (!CanBus_YawAllowed()) { blocks |= YAW_STEP_BLOCK_PERMISSION; }
    if (fabsf(state.controller.angle_deg - state.controller.target_deg) >
        YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG) { blocks |= YAW_STEP_BLOCK_POSITION; }
    if (fabsf(state.controller.cascade.velocity_dps) > YAW_CONTROLLER_STATIONARY_SPEED_DPS ||
        fabsf(state.controller.velocity_dps) > YAW_STEP_RAW_SPEED_SANITY_DPS) { blocks |= YAW_STEP_BLOCK_MOVING; }
    if (state.step_still_samples < 2U || state.controller.last_feedback_ms - state.step_still_since_ms <
        YAW_CONTROLLER_STATIONARY_HOLD_MS) { blocks |= YAW_STEP_BLOCK_STILL; }
    if (state.view_locked) { blocks |= YAW_STEP_BLOCK_VIEW; }
    return blocks;
}

static uint8_t CaptureForStart(void)
/* 功能：在安全静止时捕获启动参考，并尽可能保留人工零点以保持当前位置。 */
{
    float reference = state.controller.reference_rad;
    float edge = YAW_CONTROLLER_TARGET_LIMIT_DEG;
    uint8_t keep = state.zero_set && state.controller.have_reference;
    /* 端点反馈允许原到位误差带，避免目标限位附近的量化/微小静差阻断重启。
     * 目标仍裁到配置的限位；更远的位置继续要求人工处理，不沿旧目标使能。 */
    edge += YAW_STEP_BAND_DEG;
    if (keep && fabsf(state.controller.angle_deg) > edge) {
        state.controller.faults |= YAW_CONTROLLER_FAULT_LIMIT; return 0U;
    }
    if (!YawController_CaptureReference(&state.controller)) { return 0U; }
    if (keep) {
        state.controller.reference_rad = reference;
        state.controller.angle_deg = (state.controller.continuous_rad - reference) * 57.2957795f;
        /* 保留人工零点，但使能时先保持当前位置，不自行追赶旧零点。 */
        state.controller.target_deg = state.controller.angle_deg;
        if (state.controller.target_deg > YAW_CONTROLLER_TARGET_LIMIT_DEG) {
            state.controller.target_deg = YAW_CONTROLLER_TARGET_LIMIT_DEG;
        } else if (state.controller.target_deg < -YAW_CONTROLLER_TARGET_LIMIT_DEG) {
            state.controller.target_deg = -YAW_CONTROLLER_TARGET_LIMIT_DEG;
        }
    }
    return 1U;
}

void YawBackend_Init(void)
/* 功能：初始化Yaw控制、会话、动作和故障状态。 */
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    YawController_Init(&state.controller);
    YawStep_Init(&step);
    YawStep_Init(&previous_step);
    memset(&remote_step_event, 0, sizeof(remote_step_event));
    last_step_sequence = 0U; remote_event_pending = step_from_remote = 0U;
    last_mode = GIMBAL_MANUAL;
    state.controller.motion_guard_active = 0U;
    monitoring = cancel_pending = sent = rearm_request = 0U;
    preparation_hold = 0U;
    last_submitted_torque_nm = 0.0f;
    last_submitted_limited = 0U;
    last_tx_ms = enable_ms = disable_ms = 0U;
    clear_waiting_disable = clear_stationary_samples = 0U;
    clear_stationary_since = 0U;
    last_clear_blocks = 0U; last_clear_result = 4U; /* 4=尚未尝试发送FB */
    last_disable_result = (uint8_t)HAL_ERROR;
    off_motion_faults = 0U; actuation_started = 0U;
    taskEXIT_CRITICAL();
}

void YawBackend_GetState(YawBackendState *out)
/* 功能：在同步保护下复制Yaw实测反馈、会话和控制状态。 */
{
    if (!out) { return; }
    taskENTER_CRITICAL();
    state.online = Online(HAL_GetTick());
    state.feedback_age_ms = state.controller.have_feedback ?
        (uint32_t)(HAL_GetTick() - state.feedback.received_ms) : 0U;
    state.step_blocks = StepBlocks(HAL_GetTick());
    state.step_still_ms = state.step_still_samples ? state.controller.last_feedback_ms - state.step_still_since_ms : 0U;
    state.step_window_left_ms = step.phase == YAW_STEP_RUNNING &&
        (uint32_t)(HAL_GetTick() - step.start_ms) < YAW_STEP_WINDOW_MS ?
        YAW_STEP_WINDOW_MS - (uint32_t)(HAL_GetTick() - step.start_ms) : 0U;
    *out = state;
    taskEXIT_CRITICAL();
}

void YawBackend_Stop(void)
/* 功能：撤销Yaw当前动作与发送许可，并管理原停止确认流程。 */
{
    taskENTER_CRITICAL();
    YawStep_Abort(&step, HAL_GetTick(), state.controller.faults ? YAW_STEP_FAULT : YAW_STEP_STOP,
                  state.controller.faults);
    state.step_hold = 0U;
    state.view_hold_pending = state.view_locked;
    last_mode = GIMBAL_MANUAL;
    state.step_still_samples = 0U;
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
/* 功能：建立 Yaw 人工准备所需的停止监测和请求屏障，不直接使能电机。 */
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
/* 功能：释放 Yaw 准备流程门禁，仍需后续的新鲜遥控解锁。 */
{
    taskENTER_CRITICAL(); preparation_hold = 0U; taskEXIT_CRITICAL();
}

uint8_t YawBackend_Request(uint8_t enabled)
/* 功能：更新Yaw会话请求；实际使能仍由实时许可、反馈及原握手条件决定。 */
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
/* 功能：接收Yaw有效新反馈，更新控制器、真实成绩和停止/故障状态。 */
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
        uint8_t previous_limited = last_submitted_limited;
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
        /* 重复STEP使用连续编码器静止观察；不改失能设零/清错的原始速度门禁。
         * 原速度近零有量化和抖动，仅作为较宽的一致性检查，不能单独触发。 */
        if (state.rx_count > 1U && gap && gap < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
            !state.controller.faults && feedback.status <= 1U &&
            fabsf(state.controller.cascade.velocity_dps) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS &&
            fabsf(state.controller.velocity_dps) <= YAW_STEP_RAW_SPEED_SANITY_DPS &&
            fabsf(delta * 57.2957795f) <= 0.05f + 0.5f * (float)gap / 1000.0f) {
            if (!state.step_still_samples) { state.step_still_since_ms = feedback.received_ms; }
            if (state.step_still_samples < 255U) { ++state.step_still_samples; }
        } else { state.step_still_samples = 0U; }
        if (!state.controller.have_reference) { state.zero_set = 0U; }
        /* 仅标记新发生且来源明确的关闭状态运动记录。
         * 先去掉已被健康参考捕获清除的旧标签，不能把后来的运行故障误标。 */
        off_motion_faults &= previous_faults;
        if (off_motion) {
            off_motion_faults |= (state.controller.faults & ~previous_faults & YAW_OFF_MOTION_FAULTS);
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
/* 功能：处理 Yaw 直接目标参数；双环 PID 使用专用位置环/速度环接口。 */
{
    uint8_t accepted = 0U;
    if (parameter != 'T') { return 0U; }
    taskENTER_CRITICAL();
    if (parameter == 'T' && !state.view_locked && state.active && Online(HAL_GetTick()) && CanBus_YawAllowed()) {
        accepted = YawController_SetTarget(&state.controller, value);
    }
    if (accepted) {
        YawStep_Abort(&step, HAL_GetTick(), parameter == 'T' ? YAW_STEP_TARGET_CHANGE : YAW_STEP_PID_CHANGE, 0U);
        if (parameter == 'T') { state.step_hold = 0U; step_from_remote = 0U; }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetLoopPid(char loop, float kp, float ki, float kd)
/* 功能：校验并更新Yaw指定环 PID，参数变更时同步控制历史和真实阶跃状态。 */
{
    uint8_t accepted;
    uint32_t revision;
    taskENTER_CRITICAL();
    revision = state.controller.cascade.revision;
    accepted = YawCascade_SetPid(&state.controller.cascade, loop, kp, ki, kd);
    if (accepted && revision != state.controller.cascade.revision) {
        state.controller.torque_nm = 0.0f;
        YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_PID_CHANGE, 0U);
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t YawBackend_SetLoopParameter(char loop, char parameter, float value)
/* 功能：读取Yaw指定环参数，仅修改选中的 Kp、Ki 或 Kd 后走统一校验入口。 */
{
    uint8_t accepted = 0U;
    YawCascadePid pid;
    taskENTER_CRITICAL();
    if (loop == 'P' || loop == 'V') {
        pid = loop == 'P' ? state.controller.cascade.position : state.controller.cascade.speed;
        if (parameter == 'P') { pid.kp = value; }
        else if (parameter == 'I') { pid.ki = value; }
        else if (parameter == 'D') { pid.kd = value; }
        if (parameter == 'P' || parameter == 'I' || parameter == 'D') {
            accepted = YawBackend_SetLoopPid(loop, pid.kp, pid.ki, pid.kd);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t YawBackend_CaptureReference(void)
/* 功能：在关闭且满足原静止门禁时建立Yaw软件位置参考。 */
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

uint32_t YawBackend_StepBlocks(float delta_deg)
/* 功能：检查相对角度后返回 Yaw 阶跃所有阻塞条件的组合。 */
{
    uint32_t blocks;
    float target;
    taskENTER_CRITICAL();
    blocks = StepBlocks(HAL_GetTick());
    target = state.controller.target_deg + delta_deg;
    if ((delta_deg != 30.0f && delta_deg != -30.0f) || target != target ||
        target < -YAW_CONTROLLER_TARGET_LIMIT_DEG || target > YAW_CONTROLLER_TARGET_LIMIT_DEG) {
        blocks |= YAW_STEP_BLOCK_TARGET_LIMIT;
    }
    taskEXIT_CRITICAL(); return blocks;
}

uint8_t YawBackend_StartStep(float degrees)
/* 功能：检查Yaw阶跃门禁，接受目标并建立基于真实反馈的成绩记录。 */
{
    uint8_t accepted = 0U;
    float target;
    taskENTER_CRITICAL();
    target = state.controller.target_deg + degrees;
    if (!YawBackend_StepBlocks(degrees) && YawController_SetTarget(&state.controller, target)) {
        YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_TARGET_CHANGE, 0U);
        if (step.pending) { previous_step = step; }
        accepted = YawStep_Start(&step, HAL_GetTick(), state.controller.angle_deg, target);
        if (accepted) {
            step.shape_ms = 0U ? 0U : 0U;
            step.shape_velocity_gain = 0U ? 0.0f : 0.0f;
            step.kp = state.controller.cascade.position.kp;
            step.ki = state.controller.cascade.position.ki;
            step.kd = state.controller.cascade.position.kd;
            step.vkp = state.controller.cascade.speed.kp;
            step.vki = state.controller.cascade.speed.ki;
            step.vkd = state.controller.cascade.speed.kd;
            step.vmax_dps = state.controller.cascade.velocity_limit_dps;
            step.cascade = 1U;
            step.output_limits = 0U;
            step.ff_acceleration_gain = 0.0f;
            step.ff_limit_nm = 0.10f;
            step.last_velocity_dps = state.controller.velocity_dps;
            step.last_torque_nm = last_submitted_torque_nm;
            state.step_hold = 1U;
            step_from_remote = 0U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint32_t YawBackend_GetStepId(void)
/* 功能：读取Yaw最近接受的真实阶跃编号，用于双轴同步。 */
{
    uint32_t id;
    taskENTER_CRITICAL(); id = step.id; taskEXIT_CRITICAL();
    return id;
}
void YawBackend_GetStep(YawStep *out)
/* 功能：复制Yaw真实阶跃统计快照，不改变运动状态。 */
{
    if (!out) { return; }
    taskENTER_CRITICAL(); *out = step; taskEXIT_CRITICAL();
}
uint8_t YawBackend_TakeStepReport(YawStep *out)
/* 功能：取出Yaw待报告的真实阶跃记录，避免重复发布同一结果。 */
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (previous_step.pending) { *out = previous_step; previous_step.pending = 0U; ready = 1U; }
    else if (step.pending) { *out = step; step.pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}
uint8_t YawBackend_TakeRemoteStepEvent(YawRemoteStepEvent *out)
/* 功能：取出并清除一条Yaw遥控动作接收或拒绝事件。 */
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (remote_event_pending) { *out = remote_step_event; remote_event_pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}
static void RemoteNotice(const TargetRequest *request, uint8_t result)
/* 功能：记录 Yaw 遥控动作的接收结果和门禁信息，供控制台报告。 */
{
    remote_step_event.sequence = request->step_sequence;
    remote_step_event.target_deg = result == 1U || result == 2U ? state.controller.target_deg :
        state.controller.target_deg + request->step_delta_deg;
    remote_step_event.result = result; remote_event_pending = 1U;
    remote_step_event.blocks = result == 3U ? YawBackend_StepBlocks(request->step_delta_deg) : 0U;
    if (result == 3U && !remote_step_event.blocks) { remote_step_event.blocks = YAW_STEP_BLOCK_PERMISSION; }
}

uint8_t YawBackend_TakeRearmRequest(void)
/* 功能：取出并清除Yaw后端要求重新解锁的单次标记。 */
{
    uint8_t out;
    taskENTER_CRITICAL(); out = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL();
    return out;
}

void YawBackend_GetClearCheck(YawClearCheck *out)
/* 功能：复制 Yaw 清错门禁与最近一次清错尝试的只读证据。 */
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
    out->required_hold_ms = (state.controller.faults & YAW_OFF_MOTION_FAULTS) ?
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
        (actuation_started && (state.controller.faults & YAW_OFF_MOTION_FAULTS))) {
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
/* 功能：核对 Yaw 原清错门禁，允许时提交固定清错帧并跟踪恢复状态。 */
{
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    YawClearCheck check;
    taskENTER_CRITICAL();
    /* 单次FB供人工READY或受控上电准备；关闭监测的运动记录按来源和500ms静止门禁审核。
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

uint8_t YawBackend_HoldCurrent(void)
/* 功能：满足原保持条件后锁定Yaw当前实测位置，保留原 PID 和观测器。 */
{
    uint8_t accepted = 0U;
    float target;
    taskENTER_CRITICAL();
    target = state.controller.angle_deg;
    /* Match the existing start-capture allowance for endpoint quantization. */
    if (target > YAW_CONTROLLER_TARGET_LIMIT_DEG &&
        target <= YAW_CONTROLLER_TARGET_LIMIT_DEG + YAW_STEP_BAND_DEG) {
        target = YAW_CONTROLLER_TARGET_LIMIT_DEG;
    } else if (target < -YAW_CONTROLLER_TARGET_LIMIT_DEG &&
        target >= -YAW_CONTROLLER_TARGET_LIMIT_DEG - YAW_STEP_BAND_DEG) {
        target = -YAW_CONTROLLER_TARGET_LIMIT_DEG;
    }
    if (state.active && state.requested && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 1U && CanBus_YawAllowed() &&
        !state.controller.faults) {
        if (state.view_locked && !state.view_hold_pending) { accepted = 1U; }
        else if (YawController_SetTarget(&state.controller, target)) {
            YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_MANUAL, 0U);
            /* Preserve observer and inner load compensation. */
            state.controller.cascade.position_i = 0.0f;
            state.step_hold = 1U; step_from_remote = (uint8_t)!state.view_locked;
            state.view_hold_pending = 0U; accepted = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

void YawBackend_SetViewLock(uint8_t locked)
/* 功能：更新Yaw持续视图保持请求，实际锁点在健康且已解锁后应用。 */
{
    taskENTER_CRITICAL(); locked = (uint8_t)(locked != 0U);
    if (state.view_locked != locked) {
        state.view_locked = state.view_hold_pending = locked;
        if (!locked) { state.step_hold = step_from_remote = 0U; }
    }
    taskEXIT_CRITICAL();
}

void YawBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t inhibited)
/* 功能：将Yaw目标请求与实时许可、反馈、使能及停止状态结合，计算并提交专用 CAN 输出。 */
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
    wanted = APP_YAW_ENABLED && !preparation_hold && state.requested && allowed && !inhibited && request &&
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
                last_submitted_limited = 0U;
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
    if ((last_mode == GIMBAL_TEST || (state.step_hold && step_from_remote)) &&
        request->gimbal_mode != GIMBAL_TEST) {
        YawStep_Abort(&step, now, YAW_STEP_MANUAL, 0U);
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked && state.view_hold_pending && !YawBackend_HoldCurrent()) {
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (new_step && state.view_locked && request->step_axis == GIMBAL_STEP_YAW) {
        RemoteNotice(request, 3U); new_step = 0U;
    }
    if (request->gimbal_mode == GIMBAL_TEST &&
        (last_mode != GIMBAL_TEST || (new_step && request->step_axis == GIMBAL_STEP_PITCH))) {
        if (!YawBackend_HoldCurrent()) {
            if (new_step) { RemoteNotice(request, 3U); }
            YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
    }
    last_mode = request->gimbal_mode;
    if (new_step) {
        if (request->gimbal_mode != GIMBAL_TEST) { RemoteNotice(request, 3U); }
        else if (request->step_return_event
#if APP_PITCH_ENABLED
                 && request->step_axis == GIMBAL_STEP_YAW
#endif
                 ) {
            /* A return is a retarget request, not a new settled-start test.
             * Do not drop the edge while the old motion is still settling. */
            if (YawBackend_SetParameter('T', state.controller.target_deg + request->step_delta_deg)) {
                state.step_hold = step_from_remote = 1U; RemoteNotice(request, 2U);
            }
            else { RemoteNotice(request, 3U); }
        } else if (request->step_axis != GIMBAL_STEP_YAW) { RemoteNotice(request, 4U); }
        else if (YawBackend_StartStep(request->step_delta_deg)) {
            step_from_remote = 1U; RemoteNotice(request, 1U);
        } else { RemoteNotice(request, 3U); }
    }
    /* 测试模式保留TargetManager的CH4选方向、CH7确认/回参考；回中不清目标。 */
    if (!state.view_locked && state.step_hold && request->gimbal_mode == GIMBAL_MANUAL && request->yaw_rate_dps != 0.0f) {
        YawStep_Abort(&step, now, YAW_STEP_MANUAL, 0U);
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    (void)YawController_Update(&state.controller,
        request->gimbal_mode == GIMBAL_MANUAL && !state.view_locked ? request->yaw_rate_dps : 0.0f, now, 1U);
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
            last_submitted_limited = (uint8_t)(state.controller.cascade.torque_limited ||
                state.controller.cascade.velocity_limited);
            YawStep_Command(&step, now, last_submitted_torque_nm);
        }
    }
    taskEXIT_CRITICAL();
}
