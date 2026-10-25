/**
 * @file motor_backend.c
 * @brief 摩擦轮遥控执行与双轮停机确认
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（9）：
 *   MotorBackend_Init / MotorBackend_GetJogState / MotorBackend_GetHealth / MotorBackend_OnFrame /
 *   MotorBackend_Stop / MotorBackend_PrepareRemote / MotorBackend_SelectJog /
 *   MotorBackend_TakeRearmRequest / MotorBackend_Apply
 * 内部函数（0）：
 *   无。
 * 文件级变量/常量（13；局部变量见对应函数）：
 *   jog：两侧摩擦轮反馈、速度环及运行会话状态。
 *   initialized：后端初始化完成标记。
 *   owned：本后端是否已经拥有过输出，供停止重试使用。
 *   stop_pending：是否仍待完成原停止提交或确认。
 *   stop_attempted：是否已经尝试提交摩擦轮双零。
 *   rearm_requested：需要操作员重新解锁的待消费标记。
 *   last_stop_ms：最近一次停止重试时间。
 *   wheel_stable_since：两侧轮子连续稳速观察的起始时间。
 *   wheel_sample_ms：两侧最近一帧用于稳速检查的时间。
 *   wheel_stable_mask：已经满足真实稳速条件的轮子掩码。
 *   remote_primed：当前遥控周期是否已经准备好轮子会话。
 * 构建说明：模块关闭时，同名后端函数提供停用兼容实现。
 *   have_prepare_sequence：正常摩擦轮模式是否已有自动准备遥控序号基准。
 *   prepare_sequence：正常摩擦轮模式最近处理的自动准备遥控帧序号。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 双摩擦轮后端：遥控准备和应用层接口。 */
#include "motor_backend.h"
#include "app_config.h"
#include <string.h>

#if APP_FLYWHEEL_JOG_ENABLED
#include "can_bus.h"
#include "task.h"
static FlywheelJog jog;
/* owned保留到复位：撤权取消邮箱后仍须重发停机双零。 */
static uint8_t initialized, owned, stop_pending, stop_attempted, rearm_requested;
static uint32_t last_stop_ms;
#if APP_FEEDER_ENABLED && 1U
/* 独立拨盘互锁计时，不改轮子PI/斜坡/统计。 */
static uint32_t wheel_stable_since[2], wheel_sample_ms[2];
static uint8_t wheel_stable_mask;
#endif
static uint8_t remote_primed;
#if !APP_FEEDER_STANDALONE_TEST_ACTIVE
static uint8_t have_prepare_sequence;
static uint32_t prepare_sequence;
#endif

void MotorBackend_Init(void)
/* 功能：初始化摩擦轮执行状态、速度会话和双轮稳速互锁计时。 */
{
    taskENTER_CRITICAL();
    FlywheelJog_Init(&jog);
    initialized = 1U; owned = stop_pending = stop_attempted = rearm_requested = 0U;
    last_stop_ms = 0U;
#if APP_FEEDER_ENABLED && 1U
    memset(wheel_stable_since, 0, sizeof(wheel_stable_since));
    memset(wheel_sample_ms, 0, sizeof(wheel_sample_ms)); wheel_stable_mask = 0U;
#endif
    remote_primed = 0U;
#if !APP_FEEDER_STANDALONE_TEST_ACTIVE
    have_prepare_sequence = 0U;
    prepare_sequence = 0U;
#endif
    taskEXIT_CRITICAL();
}

void MotorBackend_GetJogState(MotorJogState *out)
/* 功能：复制两侧轮子反馈、控制请求、运行统计和停止状态。 */
{
    CanBusState bus;
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    memset(out, 0, sizeof(*out));
    CanBus_GetState(&bus);
    FlywheelJog_GetHealth(&jog, HAL_GetTick(), bus.fault_epoch, &out->health);
    out->selection = jog.selection; out->requested_mA = FlywheelJog_GetRequestedCurrent(&jog);
    out->active = jog.active; out->rearm_ready = jog.ready; out->stop_pending = stop_pending;
    out->speed = jog.speed; out->last_speed_selection = jog.last_speed_selection;
    out->speed_right = jog.last_speed_selection == FLYWHEEL_JOG_RIGHT ? jog.speed : jog.speed_right;
    out->requested_left_mA = FlywheelJog_GetNodeCurrent(&jog, APP_FLYWHEEL_LEFT_ID);
    out->requested_right_mA = FlywheelJog_GetNodeCurrent(&jog, APP_FLYWHEEL_RIGHT_ID);
    out->stop_reason = jog.stop_reason;
    taskEXIT_CRITICAL();
}

void MotorBackend_GetHealth(MotorHealth *health)
/* 功能：根据当前双轮会话和真实反馈生成系统所需的健康与稳速就绪状态。 */
{
    MotorJogState state;
    CanBusState bus;
    if (health == 0) { return; }
    taskENTER_CRITICAL();
    MotorBackend_GetJogState(&state); CanBus_GetState(&bus);
    memset(health, 0, sizeof(*health));
    health->ready = initialized && state.selection != FLYWHEEL_JOG_OFF && !stop_pending;
    health->feedback_ready = state.health.valid && bus.started && bus.fault_epoch == 0U;
    health->actuator_fault = state.selection != FLYWHEEL_JOG_OFF && state.health.faults != 0U;
    /* 实际双轮均连续入带500ms且最新原始反馈也在带内；不以目标值代替反馈。 */
    health->flywheels_ready = health->ready && health->feedback_ready &&
        !health->actuator_fault && state.selection == FLYWHEEL_JOG_BOTH && state.active &&
        state.speed.running && state.speed_right.running &&
        state.speed.in_band && state.speed_right.in_band &&
#if APP_FEEDER_ENABLED
        wheel_stable_mask == 3U &&
        HAL_GetTick() - wheel_stable_since[0] >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS &&
        HAL_GetTick() - wheel_stable_since[1] >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS &&
#endif
        !state.speed.coasting && !state.speed_right.coasting &&
        state.health.feedback[0].erpm >= APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM &&
        state.health.feedback[0].erpm <= APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM &&
        state.health.feedback[1].erpm >= APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM &&
        state.health.feedback[1].erpm <= APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM;
    taskEXIT_CRITICAL();
}

uint8_t MotorBackend_OnFrame(const CanFrame *frame)
/* 功能：把有效轮子反馈交给会话控制器，并同步双轮稳速和故障事件。 */
{
    uint8_t accepted;
    uint32_t previous_faults;
    CanBusState bus;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    previous_faults = jog.faults;
    accepted = FlywheelJog_OnFrame(&jog, frame, HAL_GetTick(), bus.fault_epoch);
#if APP_FEEDER_ENABLED && 1U
    /* 新鲜STATUS1短暂离带也打断独立500ms稳速窗口，不等20ms PI更新。 */
    if (accepted && frame && frame->extended && frame->bus == APP_FLYWHEEL_CAN_BUS &&
        (frame->id >> 8) == 9U) {
        uint8_t index;
        for (index = 0U; index < 2U; ++index) {
            uint8_t bit = (uint8_t)(1U << index);
            uint8_t node = index ? APP_FLYWHEEL_RIGHT_ID : APP_FLYWHEEL_LEFT_ID;
            int32_t erpm = jog.feedback[index].erpm;
            if ((frame->id & 255U) != node) { continue; }
            if (!jog.active || erpm < APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM ||
                erpm > APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM) {
                wheel_stable_mask &= (uint8_t)~bit;
            } else {
                if (!(wheel_stable_mask & bit) ||
                    frame->received_ms - wheel_sample_ms[index] >= FLYWHEEL_JOG_STATUS1_TIMEOUT_MS) {
                    wheel_stable_since[index] = frame->received_ms;
                }
                wheel_stable_mask |= bit;
            }
            wheel_sample_ms[index] = frame->received_ms;
        }
    }
#endif
    if (owned && jog.selection != FLYWHEEL_JOG_OFF && jog.faults != previous_faults) {
        CanBus_SetOutputPermission(0U);
        stop_pending = 1U; stop_attempted = 0U; rearm_requested = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void MotorBackend_Stop(void)
/* 功能：撤销摩擦轮运行请求，并管理双零提交及后续停止确认。 */
{
    taskENTER_CRITICAL();
#if APP_FEEDER_ENABLED && 1U
    wheel_stable_mask = 0U;
#endif
    remote_primed = 0U;
    if (owned) {
        FlywheelJog_Stop(&jog);
        stop_pending = 1U; stop_attempted = 0U;
    }
    taskEXIT_CRITICAL();
}

uint8_t MotorBackend_PrepareRemote(const RemoteFrame *remote,
    uint32_t barrier_ms, uint8_t barrier_valid)
/* 功能：依据新鲜安全 OFF 帧准备双轮会话；独立拨盘模式不开放轮动力。 */
{
#if 1U && !APP_FEEDER_STANDALONE_TEST_ACTIVE
    RemoteCommand command;
    uint32_t now = HAL_GetTick();
    uint8_t result = 0U;
    if (remote == 0 || !remote->has_frame ||
        (uint32_t)(now - remote->received_ms) >= REMOTE_TIMEOUT_MS ||
        (barrier_valid && (int32_t)(remote->received_ms - barrier_ms) <= 0)) { return 0U; }
    RemoteMapper_Map(remote, &command);
    if (!command.valid || !command.failsafe_ok || command.arm != RC_LOW || !command.neutral) {
        return 0U;
    }
    taskENTER_CRITICAL();
    if (!have_prepare_sequence || prepare_sequence != remote->sequence) {
        have_prepare_sequence = 1U; prepare_sequence = remote->sequence;
        CanBus_SetOutputPermission(0U);
        if (!jog.active && !jog.last_allowed && !stop_pending) {
            if (jog.selection != FLYWHEEL_JOG_OFF &&
                (jog.selection != FLYWHEEL_JOG_BOTH || jog.faults)) {
                result = MotorBackend_SelectJog(FLYWHEEL_JOG_OFF);
            } else if (jog.selection == FLYWHEEL_JOG_OFF) {
                result = MotorBackend_SelectJog(FLYWHEEL_JOG_BOTH);
            }
        }
    }
    taskEXIT_CRITICAL();
    return result;
#else
    (void)remote; (void)barrier_ms; (void)barrier_valid;
    return 0U;
#endif
}

uint8_t MotorBackend_SelectJog(FlywheelJogSelection selection)
/* 功能：受原门禁约束更新轮子选择；独立拨盘模式只接受停止。 */
{
    CanBusState bus;
    uint8_t accepted = 0U;
    /* 独立拨盘测试也禁止串口点动重新选中摩擦轮。OFF 保持可用。 */
    if (APP_FEEDER_STANDALONE_TEST_ACTIVE && selection != FLYWHEEL_JOG_OFF) { return 0U; }
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (selection == FLYWHEEL_JOG_OFF) {
        accepted = FlywheelJog_Select(&jog, selection, HAL_GetTick(), bus.fault_epoch);
        if (owned) { stop_pending = 1U; stop_attempted = 0U; }
    } else if (initialized && !stop_pending && bus.started && !bus.fault_epoch &&
               !CanBus_OutputAllowed()) {
        if (jog.faults) { (void)FlywheelJog_ClearFault(&jog, HAL_GetTick(), bus.fault_epoch); }
        accepted = FlywheelJog_Select(&jog, selection, HAL_GetTick(), bus.fault_epoch);
        if (accepted) { owned = 1U; stop_pending = 1U; stop_attempted = 0U; }
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint8_t MotorBackend_TakeRearmRequest(void)
/* 功能：取出并清除摩擦轮要求重新解锁的单次标记。 */
{
    uint8_t requested;
    taskENTER_CRITICAL();
    requested = rearm_requested; rearm_requested = 0U;
    taskEXIT_CRITICAL();
    return requested;
}

uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t output_allowed)
/* 功能：把遥控轮子请求转为受许可约束的速度控制和 CAN 电流输出。 */
{
    CanBusState bus;
    uint8_t was_active, nodes[2], count = 0U, index, submitted = 0U;
    int32_t currents[2];
    uint32_t now;
    uint8_t wanted, ordinary_off;
    if (request == 0) { MotorBackend_Stop(); return 0U; }
    /* 外层临界区核对遥控、STOP及UART故障代次。Step->Take->Send不排队，
     * 避免旧非零电流迟到发送；这里的HAL CAN操作不等待总线发送完成。 */
    taskENTER_CRITICAL();
    now = HAL_GetTick(); CanBus_GetState(&bus); was_active = jog.active;
    wanted = !APP_FEEDER_STANDALONE_TEST_ACTIVE && request->friction_requested &&
        request->gimbal_mode == GIMBAL_MANUAL;
    ordinary_off = output_allowed && !wanted;
    output_allowed = output_allowed && wanted && !stop_pending;
    if (!output_allowed) {
        remote_primed = 0U;
        FlywheelJog_Step(&jog, now, 0U, 0U, request->wheel_reset_allowed, bus.fault_epoch);
    } else if (!jog.active && !remote_primed) {
        /* 适配既有会话接口：ON边沿先以零请求确认一次就绪，下一周期才运行。
         * 未停稳时本次ON不会就绪，须用户CH6 OFF后再次ON，不能迟到自启。 */
        FlywheelJog_Step(&jog, now, 1U, 0U, 1U, bus.fault_epoch);
        remote_primed = 1U;
    } else {
        FlywheelJog_Step(&jog, now, 1U, 1U, 0U, bus.fault_epoch);
    }
    if (was_active && !jog.active) {
        /* CH6正常关闭或进入测试不取消总解锁；故障/撤权仍要求CH5重解锁。 */
        if (!ordinary_off || jog.faults) { rearm_requested = 1U; }
        stop_pending = owned; stop_attempted = 0U;
    }
    while (count < 2U && FlywheelJog_TakeCommand(&jog, &nodes[count], &currents[count])) { count++; }
#if APP_FEEDER_ENABLED && 1U
    if (!jog.active) { wheel_stable_mask = 0U; }
#endif
    if (stop_pending) {
        /* 停机提交失败保留pending，每20ms重试；此期间禁止非零输出。 */
        if (!stop_attempted || (uint32_t)(now - last_stop_ms) >= FLYWHEEL_JOG_COMMAND_PERIOD_MS) {
            stop_attempted = 1U; last_stop_ms = now;
            if (CanBus_StopFlywheels() == HAL_OK) { stop_pending = 0U; }
            else { rearm_requested = 1U; } /* 停机提交失败恢复后不得自动重新加力。 */
        }
    } else if (owned && !output_allowed && (!stop_attempted ||
               (uint32_t)(now - last_stop_ms) >= FLYWHEEL_JOG_COMMAND_PERIOD_MS)) {
        HAL_StatusTypeDef result;
        /* 初次STOP已取消旧正流。后续双零不abort，只读采集可正常完成。
         * 邮箱忙先跳过；真正发送/总线失败才重新进入严格STOP重试。 */
        stop_attempted = 1U; last_stop_ms = now;
        result = CanBus_KeepFlywheelsStopped();
        if (result != HAL_OK && result != HAL_BUSY) { stop_pending = 1U; }
    } else if (owned && jog.selection != FLYWHEEL_JOG_OFF && output_allowed) {
        for (index = 0U; index < count; ++index) {
            if (CanBus_JogVescCurrent(nodes[index], currents[index]) != HAL_OK) {
                FlywheelJog_Fault(&jog, FLYWHEEL_JOG_FAULT_EXTERNAL);
                stop_pending = 1U; stop_attempted = 1U; last_stop_ms = now; rearm_requested = 1U;
                CanBus_SetOutputPermission(0U);
                if (CanBus_StopFlywheels() == HAL_OK) { stop_pending = 0U; }
                submitted = 0U; break;
            }
            submitted = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return submitted;
}
#else
void MotorBackend_Init(void)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ }
void MotorBackend_Stop(void)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ }
uint8_t MotorBackend_SelectJog(FlywheelJogSelection selection)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ return selection == FLYWHEEL_JOG_OFF; }
uint8_t MotorBackend_PrepareRemote(const RemoteFrame *remote,
    uint32_t barrier_ms, uint8_t barrier_valid)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ (void)remote; (void)barrier_ms; (void)barrier_valid; return 0U; }
uint8_t MotorBackend_TakeRearmRequest(void)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ return 0U; }
void MotorBackend_GetJogState(MotorJogState *state)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ if (state) { memset(state, 0, sizeof(*state)); } }
void MotorBackend_GetHealth(MotorHealth *health)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ if (health) { memset(health, 0, sizeof(*health)); } }
uint8_t MotorBackend_OnFrame(const CanFrame *frame)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ (void)frame; return 0U; }
uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t allowed)
/* 功能：提供摩擦轮模块关闭时的停用兼容入口，不产生动力输出。 */
{ (void)request; (void)allowed; return 0U; }
#endif
