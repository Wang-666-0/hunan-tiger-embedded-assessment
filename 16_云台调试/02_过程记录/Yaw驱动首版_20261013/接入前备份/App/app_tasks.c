#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#include "app_console.h"
#include "app_diagnostics.h"
#include "task.h"
#include <string.h>

static AppState published;
static volatile uint32_t stop_sequence;

void AppTasks_RequestStop(void)
{
    taskENTER_CRITICAL();
    stop_sequence++;
    CanBus_SetOutputPermission(0U);
    /* 先撤销诊断/旧邮箱，再排双零，不能让Cancel撤销刚提交的停机。 */
    AppDiagnostics_Cancel();
    MotorBackend_Stop();
    taskEXIT_CRITICAL();
}

void AppTasks_GetState(AppState *out)
{
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    *out = published;
    taskEXIT_CRITICAL();
}

static void RemoteTask(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    (void)argument;
    RemoteInput_Start();
    for (;;) {
        RemoteInput_Poll();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(APP_REMOTE_PERIOD_MS));
    }
}

static void CanTask(void *argument)
{
    CanFrame frame;
    CanBusState bus;
    (void)argument;
    CanBus_Start();
    for (;;) {
        AppDiagnostics_Poll(HAL_GetTick());
        if (CanBus_Receive(&frame, pdMS_TO_TICKS(2U)) == pdPASS) {
            CanBus_GetState(&bus);
            if (frame.fault_epoch == bus.fault_epoch) {
                AppDiagnostics_OnFrame(&frame);
                (void)MotorBackend_OnFrame(&frame);
            }
        }
    }
}

static void ControlTask(void *argument)
{
    AppState current;
    SafetyInput input;
    RemoteInput_State remote;
    CanBusState bus;
    TargetRequest jog_request;
    TickType_t wake = xTaskGetTickCount();
    uint32_t previous_ms = HAL_GetTick();
    (void)argument;
    memset(&current, 0, sizeof(current));
    AppLogic_Init(&current.logic);
    for (;;) {
        /* 快照、保护判定、许可和CAN提交在同一个短临界区完成，
         * 避免UART/CAN ISR撤权后旧快照重新开放输出。无串口阻塞发送。 */
        taskENTER_CRITICAL();
        memset(&input, 0, sizeof(input));
        RemoteInput_GetState(&remote);
        CanBus_GetState(&bus);
        input.now_ms = HAL_GetTick();
        current.control_gap_ms = (uint32_t)(input.now_ms - previous_ms);
        previous_ms = input.now_ms;
        memcpy(input.remote.channels, remote.channels, sizeof(input.remote.channels));
        input.remote.has_frame = remote.has_frame;
        input.remote.sequence = remote.valid_frames;
        input.remote.received_ms = remote.received_ms;
        input.uart_epoch = remote.fault_epoch + AppConsole_GetFaultEpoch();
        input.can_epoch = bus.fault_epoch;
#if APP_REMOTE_PROFILE_ASSESSMENT
        if (!AppConsole_IsDiagnosticMode() &&
            MotorBackend_PrepareRemote(&input.remote, current.logic.safety.barrier_ms,
                                       current.logic.safety.barrier_valid)) {
            AppTasks_RequestStop(); /* 自动准备也必须等待此后新OFF帧，禁止旧帧解锁。 */
        }
#endif
        MotorBackend_GetHealth(&current.motors);
        input.backend_ready = current.motors.ready;
        input.feedback_ready = current.motors.feedback_ready;
        input.actuator_fault = current.motors.actuator_fault;
        input.control_late = current.control_gap_ms > APP_CONTROL_MAX_GAP_MS;
        input.stop_sequence = stop_sequence;
        AppLogic_Step(&current.logic, &input, current.motors.flywheels_ready);
        jog_request = current.logic.targets.request;
#if APP_REMOTE_PROFILE_ASSESSMENT
        jog_request.wheel_reset_allowed = current.logic.command.valid &&
            current.logic.command.failsafe_ok && current.logic.command.wheel_switch == RC_LOW;
        CanBus_SetOutputPermission(current.logic.safety.output_allowed && jog_request.friction_requested);
#else
        CanBus_SetOutputPermission(current.logic.safety.output_allowed);
        /* 未解锁时TargetManager清请求，但点动重置仍须观察真实CH1回中。 */
        jog_request.yaw_rate_dps = current.logic.command.yaw_rate_dps;
#endif
        (void)MotorBackend_Apply(&jog_request, current.logic.safety.output_allowed);
        if (MotorBackend_TakeRearmRequest()) {
            /* 单次点动结束后保持正向也不能重启；重新要求新OFF回中帧。 */
            AppTasks_RequestStop();
            current.logic.safety.output_allowed = 0U;
            current.logic.safety.operator_armed = 0U;
        }
        if (!CanBus_OutputAllowed()) { current.logic.safety.output_allowed = 0U; }
        current.remote = input.remote;
        current.stop_sequence = input.stop_sequence;
        published = current;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
    }
}

static void ConsoleTask(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    (void)argument;
    AppConsole_Start();
    for (;;) {
        AppConsole_Poll();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(APP_CONSOLE_PERIOD_MS));
    }
}

void AppTasks_Create(void)
{
    MotorBackend_Init();
    RemoteInput_CreateQueue();
    CanBus_CreateQueue();
    AppConsole_CreateQueue();
    if (xTaskCreate(RemoteTask, "remote", APP_INPUT_STACK_WORDS, 0, APP_INPUT_PRIORITY, 0) != pdPASS ||
        xTaskCreate(CanTask, "can_rx", APP_CAN_STACK_WORDS, 0, APP_CAN_PRIORITY, 0) != pdPASS ||
        xTaskCreate(ControlTask, "control", APP_CONTROL_STACK_WORDS, 0, APP_CONTROL_PRIORITY, 0) != pdPASS ||
        xTaskCreate(ConsoleTask, "console", APP_CONSOLE_STACK_WORDS, 0, APP_CONSOLE_PRIORITY, 0) != pdPASS) {
        AppFault_Fatal(APP_FAULT_TASK);
    }
}
