#include "motor_backend.h"
#include "app_config.h"
#include <string.h>

#if APP_FLYWHEEL_JOG_ENABLED
#include "can_bus.h"
#include "task.h"
static FlywheelJog jog;
/* owned保留到复位：DIAG取消邮箱后仍须重发停机双零。 */
static uint8_t initialized, owned, stop_pending, stop_attempted, rearm_requested;
static uint32_t last_stop_ms;
#if APP_REMOTE_PROFILE_ASSESSMENT
static uint8_t remote_primed, have_prepare_sequence;
static uint32_t prepare_sequence;
#endif

void MotorBackend_Init(void)
{
    taskENTER_CRITICAL();
    FlywheelJog_Init(&jog);
    initialized = 1U; owned = stop_pending = stop_attempted = rearm_requested = 0U;
    last_stop_ms = 0U;
#if APP_REMOTE_PROFILE_ASSESSMENT
    remote_primed = have_prepare_sequence = 0U;
    prepare_sequence = 0U;
#endif
    taskEXIT_CRITICAL();
}

void MotorBackend_GetJogState(MotorJogState *out)
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
    /* flywheels_ready保留0：6000rpm目标尚未实物验收，不自动授权拨盘。 */
    taskEXIT_CRITICAL();
}

uint8_t MotorBackend_OnFrame(const CanFrame *frame)
{
    uint8_t accepted;
    uint32_t previous_faults;
    CanBusState bus;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    previous_faults = jog.faults;
    accepted = FlywheelJog_OnFrame(&jog, frame, HAL_GetTick(), bus.fault_epoch);
    if (owned && jog.selection != FLYWHEEL_JOG_OFF && jog.faults != previous_faults) {
        CanBus_SetOutputPermission(0U);
        stop_pending = 1U; stop_attempted = 0U; rearm_requested = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void MotorBackend_Stop(void)
{
    taskENTER_CRITICAL();
#if APP_REMOTE_PROFILE_ASSESSMENT
    remote_primed = 0U;
#endif
    if (owned) {
        FlywheelJog_Stop(&jog);
        stop_pending = 1U; stop_attempted = 0U;
    }
    taskEXIT_CRITICAL();
}

uint8_t MotorBackend_PrepareRemote(const RemoteFrame *remote,
    uint32_t barrier_ms, uint8_t barrier_valid)
{
#if APP_REMOTE_PROFILE_ASSESSMENT
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
{
    CanBusState bus;
    uint8_t accepted = 0U;
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
{
    uint8_t requested;
    taskENTER_CRITICAL();
    requested = rearm_requested; rearm_requested = 0U;
    taskEXIT_CRITICAL();
    return requested;
}

uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t output_allowed)
{
    CanBusState bus;
    uint8_t was_active, nodes[2], count = 0U, index, submitted = 0U;
    int32_t currents[2];
    uint32_t now;
#if APP_REMOTE_PROFILE_ASSESSMENT
    uint8_t wanted, ordinary_off;
#endif
    if (request == 0) { MotorBackend_Stop(); return 0U; }
    /* 外层临界区核对遥控、STOP及UART故障代次。Step->Take->Send不排队，
     * 避免旧非零电流迟到发送；这里的HAL CAN操作不等待总线发送完成。 */
    taskENTER_CRITICAL();
    now = HAL_GetTick(); CanBus_GetState(&bus); was_active = jog.active;
#if APP_REMOTE_PROFILE_ASSESSMENT
    wanted = request->friction_requested && request->gimbal_mode == GIMBAL_MANUAL;
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
#else
    FlywheelJog_Step(&jog, now, output_allowed && !stop_pending,
                    request->yaw_rate_dps > 0.0f, request->yaw_rate_dps == 0.0f,
                    bus.fault_epoch);
#endif
    if (was_active && !jog.active) {
#if APP_REMOTE_PROFILE_ASSESSMENT
        /* CH6正常关闭或进入测试不取消总解锁；故障/撤权仍要求CH5重解锁。 */
        if (!ordinary_off || jog.faults) { rearm_requested = 1U; }
        stop_pending = owned; stop_attempted = 0U;
#else
        rearm_requested = 1U; stop_pending = owned; stop_attempted = 0U;
#endif
    }
    while (count < 2U && FlywheelJog_TakeCommand(&jog, &nodes[count], &currents[count])) { count++; }
    if (stop_pending) {
        /* 停机提交失败保留pending，每20ms重试；此期间禁止非零输出。 */
        if (!stop_attempted || (uint32_t)(now - last_stop_ms) >= FLYWHEEL_JOG_COMMAND_PERIOD_MS) {
            stop_attempted = 1U; last_stop_ms = now;
            if (CanBus_StopFlywheels() == HAL_OK) { stop_pending = 0U; }
#if APP_REMOTE_PROFILE_ASSESSMENT
            else { rearm_requested = 1U; } /* 停机提交失败恢复后不得自动重新加力。 */
#endif
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
void MotorBackend_Init(void) { }
void MotorBackend_Stop(void) { }
uint8_t MotorBackend_SelectJog(FlywheelJogSelection selection) { return selection == FLYWHEEL_JOG_OFF; }
uint8_t MotorBackend_PrepareRemote(const RemoteFrame *remote,
    uint32_t barrier_ms, uint8_t barrier_valid)
{ (void)remote; (void)barrier_ms; (void)barrier_valid; return 0U; }
uint8_t MotorBackend_TakeRearmRequest(void) { return 0U; }
void MotorBackend_GetJogState(MotorJogState *state) { if (state) { memset(state, 0, sizeof(*state)); } }
void MotorBackend_GetHealth(MotorHealth *health) { if (health) { memset(health, 0, sizeof(*health)); } }
uint8_t MotorBackend_OnFrame(const CanFrame *frame) { (void)frame; return 0U; }
uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t allowed) { (void)request; (void)allowed; return 0U; }
#endif
