#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#include "app_console.h"
#include "app_diagnostics.h"
#include "gimbal_startup.h"
#if APP_FEEDER_ENABLED
#include "feeder_backend.h"
#endif
#if APP_YAW_ENABLED
#include "yaw_backend.h"
#endif
#if APP_PITCH_ENABLED
#include "pitch_backend.h"
#endif
#include "task.h"
#include <string.h>

static AppState published;
static volatile uint32_t stop_sequence;
#if APP_YAW_ENABLED && APP_PITCH_ENABLED
static uint32_t YawStepId(void)
{ YawStep step; YawBackend_GetStep(&step); return step.id; }
#endif
#if APP_PITCH_ENABLED
static uint32_t PitchStepId(void)
{ PitchStep step; PitchBackend_GetStep(&step); return step.id; }
#endif

void AppTasks_RequireRearm(void)
{
    taskENTER_CRITICAL();
    stop_sequence++;
    taskEXIT_CRITICAL();
}

void AppTasks_RequestStop(void)
{
    taskENTER_CRITICAL();
    stop_sequence++;
    CanBus_SetOutputPermission(0U);
    /* 先撤销诊断/旧邮箱，再排双零，不能让Cancel撤销刚提交的停机。 */
    AppDiagnostics_Cancel();
    MotorBackend_Stop();
#if APP_FEEDER_ENABLED
    FeederBackend_Stop();
#endif
#if APP_YAW_ENABLED
    YawBackend_Stop();
#endif
#if APP_PITCH_ENABLED
    PitchBackend_Stop();
#endif
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
#if APP_FEEDER_ENABLED
                (void)FeederBackend_OnFrame(&frame);
#endif
#if APP_YAW_ENABLED
                (void)YawBackend_OnFrame(&frame);
#endif
#if APP_PITCH_ENABLED
                (void)PitchBackend_OnFrame(&frame);
#endif
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
#if APP_YAW_ENABLED
    YawBackendState yaw;
#if APP_PITCH_ENABLED
    uint32_t yaw_step_observed = 0U, yaw_step_current;
#endif
#endif
#if APP_PITCH_ENABLED
    PitchBackendState pitch;
    uint32_t pitch_step_observed = 0U, pitch_step_current;
#endif
#if APP_YAW_ENABLED || APP_PITCH_ENABLED
    RemoteCommand demanded;
    AppAxisHealth yaw_health, pitch_health;
    uint8_t wheels_wanted;
#endif
    TickType_t wake = xTaskGetTickCount();
    uint32_t previous_ms = HAL_GetTick();
    uint32_t guard_id;
    (void)argument;
    memset(&current, 0, sizeof(current));
    AppLogic_Init(&current.logic);
    for (;;) {
        /* 快照、保护判定、许可和CAN提交在同一个短临界区完成，
         * 避免UART/CAN ISR撤权后旧快照重新开放输出。无串口阻塞发送。 */
        taskENTER_CRITICAL();
        guard_id = current.logic.stop_trace.id;
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
        input.control_late = current.control_gap_ms > APP_CONTROL_MAX_GAP_MS;
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
        input.stop_sequence = stop_sequence;
        GimbalStartup_Update(&input, AppConsole_IsDiagnosticMode());
#endif
        MotorBackend_GetHealth(&current.motors);
        input.backend_ready = current.motors.ready;
        input.feedback_ready = current.motors.feedback_ready;
        input.actuator_fault = current.motors.actuator_fault;
#if APP_YAW_ENABLED || APP_PITCH_ENABLED
        RemoteMapper_Map(&input.remote, &demanded);
        memset(&yaw_health, 0, sizeof(yaw_health)); memset(&pitch_health, 0, sizeof(pitch_health));
        wheels_wanted = demanded.friction_on && demanded.gimbal_mode == GIMBAL_MANUAL;
#if APP_YAW_ENABLED
        YawBackend_GetState(&yaw);
        yaw_health.requested = yaw.requested; yaw_health.online = yaw.online;
        yaw_health.stop_pending = yaw.stop_pending; yaw_health.faulted = yaw.controller.faults != 0U;
#endif
#if APP_PITCH_ENABLED
        PitchBackend_GetState(&pitch);
        pitch_health.requested = pitch.requested; pitch_health.online = pitch.online;
        pitch_health.stop_pending = pitch.stop_pending; pitch_health.faulted = pitch.controller.faults != 0U;
#endif
        AppLogic_CombineActuatorHealth(&input, &yaw_health, &pitch_health, wheels_wanted);
#endif
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
        if (!GimbalStartup_OutputReady()) { input.feedback_ready = 0U; }
#endif
        input.stop_sequence = stop_sequence;
#if APP_YAW_ENABLED && APP_PITCH_ENABLED
        /* 串口和遥控真正接受的新阶跃均记住轴，先同步再消费CH7回参考。 */
        yaw_step_current = YawStepId();
        if (yaw_step_current != yaw_step_observed) {
            TargetManager_RecordAcceptedTestAxis(&current.logic.targets, GIMBAL_STEP_YAW);
            yaw_step_observed = yaw_step_current;
        }
#endif
#if APP_PITCH_ENABLED
        pitch_step_current = PitchStepId();
        if (pitch_step_current != pitch_step_observed) {
            TargetManager_RecordAcceptedTestAxis(&current.logic.targets, GIMBAL_STEP_PITCH);
            pitch_step_observed = pitch_step_current;
        }
#endif
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
#if APP_FEEDER_ENABLED
        FeederBackend_Apply(&jog_request, current.logic.safety.output_allowed,
                            current.motors.flywheels_ready, AppConsole_IsDiagnosticMode());
        if (FeederBackend_TakeRearmRequest()) {
            AppLogic_RecordStop(&current.logic, &input, APP_STOP_SOURCE_FEEDER);
            AppTasks_RequestStop();
            current.logic.safety.output_allowed = current.logic.safety.operator_armed = 0U;
        }
#endif
#if APP_YAW_ENABLED
        YawBackend_Apply(&jog_request, current.logic.safety.output_allowed, AppConsole_IsDiagnosticMode());
#if APP_PITCH_ENABLED
        yaw_step_current = YawStepId();
        if (yaw_step_current != yaw_step_observed) {
            TargetManager_RecordAcceptedTestAxis(&current.logic.targets, GIMBAL_STEP_YAW);
            yaw_step_observed = yaw_step_current;
        }
#endif
        if (YawBackend_TakeRearmRequest()) {
            AppLogic_RecordStop(&current.logic, &input, APP_STOP_SOURCE_YAW);
            AppTasks_RequestStop();
            current.logic.safety.output_allowed = current.logic.safety.operator_armed = 0U;
        }
#endif
#if APP_PITCH_ENABLED
        PitchBackend_Apply(&jog_request, current.logic.safety.output_allowed, AppConsole_IsDiagnosticMode());
        pitch_step_current = PitchStepId();
        if (pitch_step_current != pitch_step_observed) {
            TargetManager_RecordAcceptedTestAxis(&current.logic.targets, GIMBAL_STEP_PITCH);
            pitch_step_observed = pitch_step_current;
        }
        if (PitchBackend_TakeRearmRequest()) {
            AppLogic_RecordStop(&current.logic, &input, APP_STOP_SOURCE_PITCH);
            AppTasks_RequestStop();
            current.logic.safety.output_allowed = current.logic.safety.operator_armed = 0U;
        }
#endif
        if (MotorBackend_TakeRearmRequest()) {
            /* 单次点动结束后保持正向也不能重启；重新要求新OFF回中帧。 */
            AppLogic_RecordStop(&current.logic, &input, APP_STOP_SOURCE_WHEELS);
            AppTasks_RequestStop();
            current.logic.safety.output_allowed = 0U;
            current.logic.safety.operator_armed = 0U;
        }
        if (!CanBus_OutputAllowed()
#if APP_FEEDER_ENABLED
            && !CanBus_FeederAllowed()
#endif
#if APP_YAW_ENABLED
            && !CanBus_YawAllowed()
#endif
#if APP_PITCH_ENABLED
            && !CanBus_PitchAllowed()
#endif
            ) { current.logic.safety.output_allowed = 0U; }
        current.remote = input.remote;
        if (current.logic.stop_trace.id != guard_id) {
            current.guard_control_gap_ms = current.control_gap_ms;
            current.guard_remote_uart_epoch = remote.fault_epoch;
            current.guard_console_uart_epoch = AppConsole_GetFaultEpoch();
        }
        current.stop_sequence = input.stop_sequence;
        published = current;
#if APP_FEEDER_ENABLED
        if (GimbalStartup_OutputReady() && !AppConsole_IsDiagnosticMode()) {
            (void)FeederBackend_PrepareRemote();
        }
#endif
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
#if APP_FEEDER_ENABLED
    FeederBackend_Init();
#endif
#if APP_YAW_ENABLED
    YawBackend_Init();
#endif
#if APP_PITCH_ENABLED
    PitchBackend_Init();
#endif
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
    GimbalStartup_Init();
#endif
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
