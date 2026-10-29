/**
 * @file yaw_backend.c
 * 作用：Yaw 会话、阶跃、归位与锁轴
 * 对外函数：
 *   YawBackend_StartStep / YawBackend_Apply / YawBackend_HoldCurrent / YawBackend_SetParameter /
 *   YawBackend_OnFrame / YawBackend_Request / YawBackend_Init / YawBackend_GetState / YawBackend_Stop /
 *   YawBackend_PrepareStop / YawBackend_ReleasePreparation / YawBackend_SetLoopPid /
 *   YawBackend_SetLoopParameter / YawBackend_CaptureReference / YawBackend_GetStepId /
 *   YawBackend_GetStep / YawBackend_TakeStepReport / YawBackend_TakeRemoteStepEvent /
 *   YawBackend_TakeRearmRequest / YawBackend_SetViewLock / YawBackend_StepBlocks /
 *   YawBackend_ClearCommsFault
 * 内部函数：
 *   CaptureForStart / RemoteNotice / Online / StepBlocks
 * 文件级变量（函数局部变量在各函数内定义）：
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
 *   off_motion_faults：已经辨明发生在关闭状态下的运动故障位。
 *   actuation_started：本会话是否尝试过使能或观察到实际使能状态。
 */
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
static uint32_t off_motion_faults;
/* FC尝试或实际状态1即置位，STOP/OFF不清；健康失能静止参考捕获后开新会话。 */
static uint8_t actuation_started;

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static uint8_t Online(uint32_t now);
static uint32_t StepBlocks(uint32_t now);
static uint8_t CaptureForStart(void);
static void RemoteNotice(const TargetRequest *request, uint8_t result);

uint8_t YawBackend_StartStep(float degrees)
/*
 * 作用：检查Yaw阶跃门禁，接受目标并建立基于真实反馈的成绩记录。
 * 参数 degrees：软件坐标中的目标角度，deg。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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
            step.shape_ms = 0U;
            step.shape_velocity_gain = 0.0f;
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

void YawBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t inhibited)
/*
 * 作用：将 Yaw 请求与全局许可、使能握手、视图锁定和反馈状态结合，并提交控制量。
 * 参数 request：待执行的云台/发射目标请求。
 * 参数 allowed：本轮是否允许输出动力。
 * 参数 inhibited：调用方是否额外禁止本次动作。
 * 流程：先检查运行需求；尚未使能时发送固定使能帧，等待发送之后的新状态 1 反馈。
 * 流程：处理本轮阶跃或归位事件后调用控制器；正常力矩命令每 10 ms 提交一次。
 * 流程：不可执行的单次事件当轮拒绝，后续不会自行补发。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
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
    wanted = !preparation_hold && state.requested && allowed && !inhibited && request &&
        Online(now) && !state.controller.faults && !state.stop_pending && !clear_waiting_disable;
    if (!wanted) {
        if (new_step) { RemoteNotice(request, 3U); }
        if (state.active || state.enabling) { YawBackend_Stop(); }
        CanBus_SetYawPermission(0U);
        if (monitoring && (!sent || (uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS)) {
            result = CanBus_YawDisable(cancel_pending);
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
        } else if ((uint32_t)(now - enable_ms) >= 100U) {
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
                 && request->step_axis == GIMBAL_STEP_YAW
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

uint8_t YawBackend_HoldCurrent(void)
/*
 * 作用：满足原保持条件后锁定Yaw当前实测位置，保留原 PID 和观测器。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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

uint8_t YawBackend_SetParameter(char parameter, float value)
/*
 * 作用：处理 Yaw 直接目标参数；双环 PID 使用专用位置环/速度环接口。
 * 参数 parameter：待修改的参数选择符；P/I/D 对应增益，目标入口使用 T。
 * 参数 value：待处理的数值。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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

uint8_t YawBackend_OnFrame(const CanFrame *frame)
/*
 * 作用：接收Yaw有效新反馈，更新控制器、真实成绩和停止/故障状态。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 返回是否识别并处理了该设备的数据；电机反馈中的故障状态仍由上层检查。
 */
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

uint8_t YawBackend_Request(uint8_t enabled)
/*
 * 作用：更新Yaw会话请求；实际使能仍由实时许可、反馈及原握手条件决定。
 * 参数 enabled：请求开关，非零表示开启请求。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (!enabled) {
        preparation_hold = 0U;
        state.requested = 0U;
        YawBackend_Stop(); accepted = 1U;
    } else if (bus.started && !bus.fault_epoch && !state.active && !state.enabling &&
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

void YawBackend_Init(void)
/*
 * 作用：初始化Yaw控制、会话、动作和故障状态。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
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
    off_motion_faults = 0U; actuation_started = 0U;
    taskEXIT_CRITICAL();
}

void YawBackend_GetState(YawBackendState *out)
/*
 * 作用：在同步保护下复制Yaw实测反馈、会话和控制状态。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值，将当前快照写入输出参数；不会启动新的运动。
 */
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

static uint8_t CaptureForStart(void)
/*
 * 作用：在安全静止时捕获启动参考，并尽可能保留人工零点以保持当前位置。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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

void YawBackend_Stop(void)
/*
 * 作用：撤销Yaw当前动作与发送许可，并管理原停止确认流程。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值，撤销请求并更新停止状态；设备失能仍由后续反馈确认。
 */
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
/*
 * 作用：建立 Yaw 人工准备所需的停止监测和请求屏障，不直接使能电机。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (bus.started && !bus.fault_epoch) {
        monitoring = 1U; state.requested = 0U; state.zero_set = 0U;
        YawBackend_Stop(); preparation_hold = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}

void YawBackend_ReleasePreparation(void)
/*
 * 作用：释放 Yaw 准备流程门禁，仍需后续的新鲜遥控解锁。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    taskENTER_CRITICAL(); preparation_hold = 0U; taskEXIT_CRITICAL();
}

uint8_t YawBackend_SetLoopPid(char loop, float kp, float ki, float kd)
/*
 * 作用：校验并更新Yaw指定环 PID，参数变更时同步控制历史和真实阶跃状态。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 参数 kp：比例增益。
 * 参数 ki：积分增益，计算时再乘 dt。
 * 参数 kd：测量微分增益。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    uint8_t accepted;
    uint32_t revision;
    taskENTER_CRITICAL();
    revision = state.controller.cascade.revision;
    accepted = GimbalPid_SetPid(&state.controller.cascade, loop, kp, ki, kd);
    if (accepted && revision != state.controller.cascade.revision) {
        state.controller.torque_nm = 0.0f;
        YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_PID_CHANGE, 0U);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetLoopParameter(char loop, char parameter, float value)
/*
 * 作用：读取Yaw指定环参数，仅修改选中的 Kp、Ki 或 Kd 后走统一校验入口。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 参数 parameter：待修改的参数选择符；P/I/D 对应增益，目标入口使用 T。
 * 参数 value：待处理的数值。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    uint8_t accepted = 0U;
    GimbalPidGains pid;
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
/*
 * 作用：在关闭且满足原静止门禁时建立Yaw软件位置参考。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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

uint32_t YawBackend_GetStepId(void)
/*
 * 作用：读取Yaw最近接受的真实阶跃编号，用于双轴同步。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回当前序号、计数或故障代次，读取本身不触发运动。
 */
{
    uint32_t id;
    taskENTER_CRITICAL(); id = step.id; taskEXIT_CRITICAL();
    return id;
}

void YawBackend_GetStep(YawStep *out)
/*
 * 作用：复制Yaw真实阶跃统计快照，不改变运动状态。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值，将当前快照写入输出参数；不会启动新的运动。
 */
{
    if (!out) { return; }
    taskENTER_CRITICAL(); *out = step; taskEXIT_CRITICAL();
}

uint8_t YawBackend_TakeStepReport(YawStep *out)
/*
 * 作用：取出Yaw待报告的真实阶跃记录，避免重复发布同一结果。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 返回 1 表示取到了新事件/报告，并消费其待处理标志；0 表示当前没有。
 */
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (previous_step.pending) { *out = previous_step; previous_step.pending = 0U; ready = 1U; }
    else if (step.pending) { *out = step; step.pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}

uint8_t YawBackend_TakeRemoteStepEvent(YawRemoteStepEvent *out)
/*
 * 作用：取出并清除一条Yaw遥控动作接收或拒绝事件。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 返回 1 表示取到了新事件/报告，并消费其待处理标志；0 表示当前没有。
 */
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (remote_event_pending) { *out = remote_step_event; remote_event_pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}

static void RemoteNotice(const TargetRequest *request, uint8_t result)
/*
 * 作用：记录 Yaw 遥控动作的接收结果和门禁信息，供控制台报告。
 * 参数 request：待执行的云台/发射目标请求。
 * 参数 result：发送或业务动作的结果码，供上层处理及串口回执。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    remote_step_event.sequence = request->step_sequence;
    remote_step_event.target_deg = result == 1U || result == 2U ? state.controller.target_deg :
        state.controller.target_deg + request->step_delta_deg;
    remote_step_event.result = result; remote_event_pending = 1U;
    remote_step_event.blocks = result == 3U ? YawBackend_StepBlocks(request->step_delta_deg) : 0U;
    if (result == 3U && !remote_step_event.blocks) { remote_step_event.blocks = YAW_STEP_BLOCK_PERMISSION; }
}

uint8_t YawBackend_TakeRearmRequest(void)
/*
 * 作用：取出并清除Yaw后端要求重新解锁的单次标记。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示取到了新事件/报告，并消费其待处理标志；0 表示当前没有。
 */
{
    uint8_t out;
    taskENTER_CRITICAL(); out = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL();
    return out;
}

void YawBackend_SetViewLock(uint8_t locked)
/*
 * 作用：更新Yaw持续视图保持请求，实际锁点在健康且已解锁后应用。
 * 参数 locked：是否请求保持当前实测姿态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    taskENTER_CRITICAL(); locked = (uint8_t)(locked != 0U);
    if (state.view_locked != locked) {
        state.view_locked = state.view_hold_pending = locked;
        if (!locked) { state.step_hold = step_from_remote = 0U; }
    }
    taskEXIT_CRITICAL();
}

static uint8_t Online(uint32_t now)
/*
 * 作用：核对本轴反馈时间与当前 CAN 故障代次，判断设备是否在线。
 * 参数 now：当前毫秒时间。
 * 返回布尔判断结果，非零为满足条件，0 为不满足。
 */
{
    CanBusState bus;
    CanBus_GetState(&bus);
    return state.controller.have_feedback &&
        (uint32_t)(now - state.feedback.received_ms) < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        state.feedback.fault_epoch == bus.fault_epoch;
}

static uint32_t StepBlocks(uint32_t now)
/*
 * 作用：汇总 Yaw 正式阶跃所需的会话、位置、静止、限位和视图门禁。
 * 参数 now：当前毫秒时间。
 * 返回位掩码；每一位表示一个待发邮箱或阻塞原因，0 表示没有。
 */
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

uint32_t YawBackend_StepBlocks(float delta_deg)
/*
 * 作用：检查相对角度后返回 Yaw 阶跃所有阻塞条件的组合。
 * 参数 delta_deg：相邻有效位置增量，deg。
 * 返回位掩码；每一位表示一个待发邮箱或阻塞原因，0 表示没有。
 */
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

uint8_t YawBackend_ClearCommsFault(void)
/*
 * 作用：核对 Yaw 原清错门禁，允许时提交固定清错帧并跟踪恢复状态。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    uint32_t hold_ms = state.feedback.received_ms - clear_stationary_since;
    uint32_t required_ms = (state.controller.faults & YAW_OFF_MOTION_FAULTS) ?
        500U : YAW_CONTROLLER_STATIONARY_HOLD_MS;
    taskENTER_CRITICAL();
    if (monitoring && !state.requested && !state.active && !state.enabling &&
        !clear_waiting_disable && !CanBus_YawAllowed() && !CanBus_OutputAllowed() &&
        Online(now) && state.feedback.status == 13U && clear_stationary_samples >= 2U &&
        hold_ms >= required_ms && sent && !cancel_pending &&
        now - last_tx_ms < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        !(state.controller.faults & ~(YAW_CONTROLLER_FAULT_STATUS | off_motion_faults)) &&
        !(actuation_started && (state.controller.faults & YAW_OFF_MOTION_FAULTS)) &&
        CanBus_YawClearComms() == HAL_OK) {
        clear_waiting_disable = 1U;
        state.stop_pending = 1U;
        disable_ms = now;
        cancel_pending = 0U; sent = 0U; // FB 后继续等新失能反馈，不直接使能。
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
