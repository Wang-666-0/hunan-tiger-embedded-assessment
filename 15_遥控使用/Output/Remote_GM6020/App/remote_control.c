/* 遥控许可状态机：健康且看到新的 OFF，再在中位时 OFF->ON，才可启动。
 * iBUS 无明确 RF 失联标志：CH6 高位是接收机 failsafe 的第二许可，
 * 必须在遥控器菜单把 failsafe 设置 CH1=1500、CH5=1000、CH6=1000。
 * 检查帧真实到达时间；不能用任务读取旧帧的时刻给链路续命。
 */
#include "remote_control.h"
#include "app_config.h"
#include <string.h>

void RemoteControl_Init(RemoteControl *control)// 初始化遥控控制结构体
{
    if (control == 0) { return; }
    memset(control, 0, sizeof(*control));
    control->previous_arm = 1U; /* 上电开关已 ON，不算新的 OFF->ON。 */
    control->fault = REMOTE_FAULT_POWER_ON;
}

void RemoteControl_ForceStop(RemoteControl *control, RemoteControl_Fault fault)
{
    if (control == 0) { return; }
    control->armed = 0U;
    control->off_seen = 0U;
    control->previous_arm = 1U;
    control->target_rpm = 0.0f;
    control->fault = (uint32_t)fault;
}

float RemoteControl_MapSpeed(uint16_t channel)
{
    int32_t delta = (int32_t)channel - (int32_t)REMOTE_CENTER;
    if (delta >= -(int32_t)REMOTE_DEADBAND && delta <= (int32_t)REMOTE_DEADBAND) {
        return 0.0f;
    }
    if (delta > 500) { delta = 500; }
    if (delta < -500) { delta = -500; }
    return (float)delta * REMOTE_SPEED_LIMIT_RPM / 500.0f;
}

void RemoteControl_Step(RemoteControl *control, const RemoteControl_Input *input)
{
    RemoteControl_Fault fault = REMOTE_FAULT_NONE;
    uint8_t new_frame, arm;
    uint16_t ch1, ch5, ch6;
    if (control == 0 || input == 0) { return; }
    new_frame = input->has_frame && (!control->have_sequence ||
        input->frame_sequence != control->last_sequence);
    if (new_frame) {
        control->have_sequence = 1U;
        control->last_sequence = input->frame_sequence;
    }
    ch1 = input->channels[0]; ch5 = input->channels[4]; ch6 = input->channels[5];
    if (input->uart_fault) { fault = REMOTE_FAULT_UART; }
    else if (input->can_tx_fault) { fault = REMOTE_FAULT_CAN_TX; }
    else if (input->control_late) { fault = REMOTE_FAULT_CONTROL_TIMING; }
    else if (!input->has_frame || (uint32_t)(input->now_ms - input->received_ms) >= REMOTE_TIMEOUT_MS) {
        fault = REMOTE_FAULT_LINK;
    }
    else if (ch1 < 800U || ch1 > 2200U || ch5 < 800U || ch5 > 2200U || ch6 < 800U || ch6 > 2200U) {
        fault = REMOTE_FAULT_LINK;
    }
    else if (ch6 <= REMOTE_SWITCH_HIGH) { fault = REMOTE_FAULT_FAILSAFE; }
    else if (!input->can_online) { fault = REMOTE_FAULT_CAN; }
    else if (input->can_temperature >= MOTOR_TEMPERATURE_LIMIT_C) { fault = REMOTE_FAULT_TEMPERATURE; }
    if (fault != REMOTE_FAULT_NONE) {
        RemoteControl_ForceStop(control, fault);
        return;
    }
    /* 只有新收到的有效帧才能确认开关动作，旧快照不能解除故障锁定。 */
    if (!new_frame) { return; }
    if (ch5 <= REMOTE_SWITCH_LOW) {
        control->armed = 0U;
        control->target_rpm = 0.0f;
        control->off_seen = 1U;
        control->previous_arm = 0U;
        control->fault = REMOTE_FAULT_DISARM;
        return;
    }
    if (ch5 <= REMOTE_SWITCH_HIGH) {
        RemoteControl_ForceStop(control, REMOTE_FAULT_DISARM);
        return;
    }
    arm = control->off_seen && !control->previous_arm;
    control->previous_arm = 1U;
    if (!control->armed && arm) {
        int32_t delta = (int32_t)ch1 - (int32_t)REMOTE_CENTER;
        control->off_seen = 0U;
        if (delta < -(int32_t)REMOTE_DEADBAND || delta > (int32_t)REMOTE_DEADBAND) {
            RemoteControl_ForceStop(control, REMOTE_FAULT_NOT_CENTERED);
            return;
        }
        control->armed = 1U;
        control->fault = REMOTE_FAULT_NONE;
    }
    if (control->armed) { control->target_rpm = RemoteControl_MapSpeed(ch1); }
}
