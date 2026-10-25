/**
 * @file app_logic.c
 * @brief 遥控、安全与目标管理的应用协调
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（4）：
 *   AppLogic_CombineActuatorHealth / AppLogic_Init / AppLogic_RecordStop / AppLogic_Step
 * 内部函数（0）：
 *   无。
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 应用纯逻辑：组合轴健康、安全状态和目标请求。 */
#include "app_logic.h"
#include "app_config.h"
#include <string.h>
void AppLogic_CombineActuatorHealth(SafetyInput *input, const AppAxisHealth *yaw,
                                   const AppAxisHealth *pitch, uint8_t wheels_wanted)
/* 功能：组合已请求云台轴与必要轮子的健康状态，形成统一运行许可输入。 */
{
    uint8_t requested = 0U, healthy = 1U, faulted = 0U;
    const AppAxisHealth *axes[2] = { yaw, pitch };
    uint32_t i;
    if (!input) { return; }
#if APP_FEEDER_STANDALONE_TEST_ACTIVE
    /* 独立拨盘测试时摩擦轮不参与动力许可；已请求的云台轴仍逐项检查。
     * 不把缺失电调的在线/稳速状态伪装为正常。 */
    wheels_wanted = 0U;
#endif
    for (i = 0U; i < 2U; ++i) {
        const AppAxisHealth *axis = axes[i];
        if (!axis || !axis->requested) { continue; }
        requested = 1U;
        healthy = (uint8_t)(healthy && axis->online && !axis->stop_pending && !axis->faulted);
        faulted = (uint8_t)(faulted || axis->faulted);
    }
    if (requested) {
        input->feedback_ready = (uint8_t)(healthy &&
            (!wheels_wanted || (input->backend_ready && input->feedback_ready)));
        input->actuator_fault = (uint8_t)(faulted || (wheels_wanted && input->actuator_fault));
        input->backend_ready = 1U;
    }
}
void AppLogic_Init(AppLogic *logic)
/* 功能：初始化应用、安全管理和目标节拍状态。 */
{
    if (logic == 0) { return; }
    memset(logic, 0, sizeof(*logic));
    SafetyManager_Init(&logic->safety);
    TargetManager_Init(&logic->targets);
}
void AppLogic_RecordStop(AppLogic *logic, const SafetyInput *input, AppStopSource source)
/* 功能：保存本次解锁后的首次撤权来源、输入和故障证据。 */
{
    AppStopTrace *trace;
    if (!logic || !input || !logic->trace_armed) { return; }
    logic->trace_armed = 0U;
    trace = &logic->stop_trace;
    ++trace->id; trace->at_ms = input->now_ms;
    trace->source = (uint8_t)source; trace->reason = (uint8_t)logic->safety.reason;
    trace->remote_age_ms = input->remote.has_frame ? input->now_ms - input->remote.received_ms : 0xFFFFFFFFUL;
    trace->uart_epoch = input->uart_epoch; trace->can_epoch = input->can_epoch;
    trace->stop_sequence = input->stop_sequence; trace->triggers = logic->trace_triggers;
    trace->previous_mode = logic->trace_previous_mode;
    trace->mode = (uint8_t)logic->command.gimbal_mode;
    memcpy(trace->channels, input->remote.channels, sizeof(trace->channels));
}
void AppLogic_Step(AppLogic *logic, const SafetyInput *input, uint8_t flywheels_ready)
/* 功能：映射有效遥控，推进安全状态，再生成当前手动、阶跃或发射请求。 */
{
    uint8_t new_frame, was_armed;
    if (logic == 0 || input == 0) { return; }
    new_frame = input->remote.has_frame && (!logic->safety.have_sequence ||
        logic->safety.last_sequence != input->remote.sequence);
    was_armed = logic->safety.operator_armed;
    logic->trace_previous_mode = (uint8_t)logic->command.gimbal_mode;
    RemoteMapper_Map(&input->remote, &logic->command);
    /* 只记录输入证据；许可仍完全由下面的原SafetyManager决定。 */
    logic->trace_triggers = 0U;
    if (input->stop_sequence != logic->safety.stop_sequence) { logic->trace_triggers |= APP_STOP_TRIGGER_STOP; }
    if (input->uart_epoch != logic->safety.uart_epoch) { logic->trace_triggers |= APP_STOP_TRIGGER_UART; }
    if (input->can_epoch != logic->safety.can_epoch) { logic->trace_triggers |= APP_STOP_TRIGGER_CAN; }
    if (input->control_late) { logic->trace_triggers |= APP_STOP_TRIGGER_TIMING; }
    if (!logic->command.valid || !input->remote.has_frame ||
        (uint32_t)(input->now_ms - input->remote.received_ms) >= REMOTE_TIMEOUT_MS) {
        logic->trace_triggers |= APP_STOP_TRIGGER_REMOTE;
    }
    if (!logic->command.failsafe_ok) { logic->trace_triggers |= APP_STOP_TRIGGER_FAILSAFE; }
    if (input->actuator_fault) { logic->trace_triggers |= APP_STOP_TRIGGER_ACTUATOR; }
    if (input->backend_ready && logic->command.arm == RC_HIGH && !input->feedback_ready) {
        logic->trace_triggers |= APP_STOP_TRIGGER_FEEDBACK;
    }
    if (logic->command.arm == RC_LOW) { logic->trace_triggers |= APP_STOP_TRIGGER_CH5_OFF; }
    SafetyManager_Step(&logic->safety, input, &logic->command);
    if (logic->safety.operator_armed && !was_armed) { logic->trace_armed = 1U; }
    if (was_armed && !logic->safety.operator_armed) {
        AppLogic_RecordStop(logic, input, APP_STOP_SOURCE_SAFETY);
    }
    TargetManager_Step(&logic->targets, &logic->command, new_frame,
        logic->safety.operator_armed, flywheels_ready && logic->safety.output_allowed,
        input->now_ms);
}
