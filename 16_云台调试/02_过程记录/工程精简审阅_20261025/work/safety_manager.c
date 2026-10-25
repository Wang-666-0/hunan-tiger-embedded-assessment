/* 安全许可状态机与新 OFF 帧解锁屏障。 */
#include "safety_manager.h"
#include "app_config.h"
#include <string.h>

static void Lock(SafetyManager *manager, SafetyReason reason, uint32_t now)
{
    manager->state = SAFETY_FAULT;
    manager->reason = reason;
    manager->last_fault = reason;
    manager->operator_armed = 0U;
    manager->output_allowed = 0U;
    manager->off_seen = 0U;
    manager->barrier_ms = now;
    manager->barrier_valid = 1U;
}

void SafetyManager_Init(SafetyManager *manager)
{
    if (manager == 0) { return; }
    memset(manager, 0, sizeof(*manager));
    manager->reason = SAFETY_POWER_ON;
}

void SafetyManager_Step(SafetyManager *manager, const SafetyInput *input,
                        const RemoteCommand *command)
{
    SafetyReason fault = SAFETY_NONE;
    uint8_t new_frame;
    if (manager == 0 || input == 0 || command == 0) { return; }
    new_frame = input->remote.has_frame && (!manager->have_sequence ||
        input->remote.sequence != manager->last_sequence);
    if (new_frame) {
        manager->have_sequence = 1U;
        manager->last_sequence = input->remote.sequence;
    }
    /* 故障代次和STOP在每个控制周期检查，不能等下一帧遥控才撤销许可。 */
    if (input->stop_sequence != manager->stop_sequence) { fault = SAFETY_STOP; }
    else if (input->uart_epoch != manager->uart_epoch) { fault = SAFETY_UART; }
    else if (input->can_epoch != manager->can_epoch) { fault = SAFETY_CAN; }
    else if (input->control_late) { fault = SAFETY_TIMING; }
    else if (!command->valid || !input->remote.has_frame ||
        (uint32_t)(input->now_ms - input->remote.received_ms) >= REMOTE_TIMEOUT_MS) {
        fault = SAFETY_REMOTE;
    }
    else if (!command->failsafe_ok) { fault = SAFETY_FAILSAFE; }
    else if (input->actuator_fault) { fault = SAFETY_ACTUATOR; }
    else if (input->backend_ready && command->arm == RC_HIGH && !input->feedback_ready) {
        fault = SAFETY_FEEDBACK;
    }
    manager->stop_sequence = input->stop_sequence;
    manager->uart_epoch = input->uart_epoch;
    manager->can_epoch = input->can_epoch;
    manager->output_allowed = 0U;
    if (fault != SAFETY_NONE) { Lock(manager, fault, input->now_ms); return; }
    if (new_frame) {
        /* 排队的旧OFF帧，包括与STOP同毫秒的帧，都不能用于重新解锁。 */
        if (manager->barrier_valid &&
            (int32_t)(input->remote.received_ms - manager->barrier_ms) <= 0) { return; }
        if (command->arm == RC_LOW) {
            manager->operator_armed = 0U;
            manager->off_seen = 1U;
            manager->state = SAFETY_READY;
            manager->reason = SAFETY_NONE;
            manager->barrier_valid = 0U;
        }
        else if (command->arm == RC_MIDDLE) {
            Lock(manager, SAFETY_ARM_POSITION, input->now_ms);
            return;
        }
        else if (!manager->operator_armed && manager->off_seen) {
            if (!command->neutral) { Lock(manager, SAFETY_NOT_NEUTRAL, input->now_ms); return; }
            manager->operator_armed = 1U;
            manager->off_seen = 0U;
            manager->state = SAFETY_ARMED;
            manager->reason = SAFETY_NONE;
        }
    }
    /* 遥控解锁可以先演示；动力许可还要求正式驱动和真实反馈均就绪。 */
    manager->output_allowed = manager->operator_armed && input->backend_ready && input->feedback_ready;
}
