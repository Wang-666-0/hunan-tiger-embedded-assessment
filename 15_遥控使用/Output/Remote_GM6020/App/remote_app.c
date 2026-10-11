/* 把纯遥控状态机连接到 HAL/FreeRTOS 与电机控制。
 * guard 的读取/更新和启动提交处于同一短临界区；控制台 STOP 同时撤销
 * guard 和电机请求，motor_control 内部版本号丢弃在途旧计算。
 */
#include "remote_app.h"
#include "remote_input.h"
#include "motor_console.h"
#include "motor_control.h"
#include "can_debug.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static RemoteControl guard;
static uint32_t remote_epoch, console_epoch, can_error_epoch;
static uint32_t stop_cutoff_ms;
static uint8_t have_stop_cutoff;

void RemoteApp_Init(void) { RemoteControl_Init(&guard); }
void RemoteApp_Stop(RemoteControl_Fault fault)
{
    taskENTER_CRITICAL();
    stop_cutoff_ms = HAL_GetTick();
    have_stop_cutoff = 1U;
    RemoteControl_ForceStop(&guard, fault);
    guard.have_sequence = 1U;
    MotorControl_Stop(fault == REMOTE_FAULT_STOP ? MOTOR_STOP_USER : MOTOR_STOP_REMOTE);
    taskEXIT_CRITICAL();
}
void RemoteApp_GetState(RemoteControl *out)
{
    if (out == NULL) { return; }
    taskENTER_CRITICAL(); *out = guard; taskEXIT_CRITICAL();
}
void RemoteApp_Step(void)
{
    RemoteInput_State remote;
    RemoteControl_Input input;
    GM6020_Feedback feedback;
    MotorControl_State motor;
    memset(&input, 0, sizeof(input));
    RemoteInput_GetState(&remote);
    GM6020_GetFeedback(&feedback);
    MotorControl_GetState(&motor);
    input.now_ms = HAL_GetTick(); input.received_ms = remote.received_ms;
    input.frame_sequence = remote.valid_frames; input.has_frame = remote.has_frame;
    memcpy(input.channels, remote.channels, sizeof(input.channels));
    input.can_online = can_ready && GM6020_IsOnline(&feedback, input.now_ms);
    input.can_temperature = feedback.temperature;
    taskENTER_CRITICAL();
    /* STOP 之前排队的 OFF/ON 帧不能解锁。只接受停止后新到达的帧。 */
    if (have_stop_cutoff) {
        if (input.has_frame && (int32_t)(input.received_ms - stop_cutoff_ms) > 0) {
            have_stop_cutoff = 0U;
        } else {
            input.frame_sequence = guard.last_sequence;
        }
    }
    input.uart_fault = remote.fault_epoch != remote_epoch ||
        motor_console_debug.rx_fault_epoch != console_epoch;
    remote_epoch = remote.fault_epoch; console_epoch = motor_console_debug.rx_fault_epoch;
    input.can_tx_fault = can1_error_callback_count != can_error_epoch;
    can_error_epoch = can1_error_callback_count;
    /* 电机步骤上一轮自己检测到故障，也必须撤销遥控，不能下一轮自动重启。 */
    if (guard.armed && !motor.enabled) {
        if (motor.stop_reason == MOTOR_STOP_CONTROL_TIMING) { input.control_late = 1U; }
        else if (motor.stop_reason == MOTOR_STOP_CAN_TX) { input.can_tx_fault = 1U; }
        else { RemoteControl_ForceStop(&guard, REMOTE_FAULT_CAN); }
    }
    RemoteControl_Step(&guard, &input);
    if (guard.armed) {
        if (!MotorControl_SetRemoteSpeed(guard.target_rpm)) {
            RemoteControl_ForceStop(&guard, REMOTE_FAULT_CAN);
            MotorControl_Stop(MOTOR_STOP_REMOTE);
        }
    } else if (motor.enabled) {
        MotorControl_Stop(MOTOR_STOP_REMOTE);
    }
    taskEXIT_CRITICAL();
}
