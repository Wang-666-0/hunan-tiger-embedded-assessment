/**
 * @file app_tasks.c
 * 作用：RTOS 任务与运行调度
 * 对外函数：
 *   AppTasks_Create / AppTasks_GetState / AppTasks_RequireRearm / AppTasks_RequestStop
 * 内部函数：
 *   ControlTask / RemoteTask / CanTask / ConsoleTask
 * 文件级变量（函数局部变量在各函数内定义）：
 *   published：控制任务向其他任务发布的完整应用快照。
 *   stop_sequence：全局停止与重新解锁屏障序号。
 */
#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#include "app_console.h"
#include "gimbal_startup.h"
#include "feeder_backend.h"
#include "yaw_backend.h"
#include "pitch_backend.h"
#include "gimbal_step_sync.h"
#include "task.h"
#include <string.h>

static AppState published;
static volatile uint32_t stop_sequence;

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static void RemoteTask(void *argument);
static void CanTask(void *argument);
static void ControlTask(void *argument);
static void ConsoleTask(void *argument);

static void ControlTask(void *argument)
/*
 * 作用：按固定节拍执行系统控制，是遥控、安全逻辑和各电机后端的统一调用入口。
 * 参数 argument：FreeRTOS 任务入口参数，本工程未使用。
 * 流程：读取遥控、CAN 和双轴状态，检查反馈时效和控制间隔，更新上电准备与视图锁轴。
 * 流程：生成目标请求后依次执行摩擦轮、拨盘、Yaw、Pitch；后端请求重解锁时统一停止。
 * 流程：当前周期为 2 ms；Yaw/Pitch 自己的 CAN 命令提交周期分别为 10/2 ms。
 * 流程：快照、许可和提交在临界区内协调，串口格式化及发送由低优先级任务执行。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    AppState current;
    SafetyInput input;
    RemoteInput_State remote;
    CanBusState bus;
    TargetRequest jog_request;
    YawBackendState yaw;
    GimbalStepSync step_sync = {0U, 0U};
    PitchBackendState pitch;
    RemoteCommand demanded;
    AppAxisHealth yaw_health, pitch_health;
    uint8_t wheels_wanted;
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
        input.uart_epoch = remote.fault_epoch;
        input.can_epoch = bus.fault_epoch;
        if (MotorBackend_PrepareRemote(&input.remote, current.logic.safety.barrier_ms,
                                       current.logic.safety.barrier_valid)) {
            AppTasks_RequestStop(); /* 自动准备也必须等待此后新OFF帧，禁止旧帧解锁。 */
        }
        input.control_late = current.control_gap_ms > APP_CONTROL_MAX_GAP_MS;
        input.stop_sequence = stop_sequence;
        GimbalStartup_Update(&input, 0U);
        GimbalStepSync_ApplyViewHold(AppConsole_GetViewHoldMask());
        MotorBackend_GetHealth(&current.motors);
        input.backend_ready = current.motors.ready;
        input.feedback_ready = current.motors.feedback_ready;
        input.actuator_fault = current.motors.actuator_fault;
        RemoteMapper_Map(&input.remote, &demanded);
        memset(&yaw_health, 0, sizeof(yaw_health)); memset(&pitch_health, 0, sizeof(pitch_health));
        wheels_wanted = demanded.friction_on && demanded.gimbal_mode == GIMBAL_MANUAL;
        YawBackend_GetState(&yaw);
        yaw_health.requested = yaw.requested; yaw_health.online = yaw.online;
        yaw_health.stop_pending = yaw.stop_pending; yaw_health.faulted = yaw.controller.faults != 0U;
        PitchBackend_GetState(&pitch);
        pitch_health.requested = pitch.requested; pitch_health.online = pitch.online;
        pitch_health.stop_pending = pitch.stop_pending; pitch_health.faulted = pitch.controller.faults != 0U;
        AppLogic_CombineActuatorHealth(&input, &yaw_health, &pitch_health, wheels_wanted);
        if (!GimbalStartup_OutputReady()) { input.feedback_ready = 0U; }
        input.stop_sequence = stop_sequence;
        GimbalStepSync_Update(&step_sync, &current.logic.targets,
            demanded.gimbal_mode == GIMBAL_TEST && current.logic.safety.output_allowed);
        AppLogic_Step(&current.logic, &input, current.motors.flywheels_ready);
        jog_request = current.logic.targets.request;
        GimbalStepSync_FilterViewRequest(&jog_request, AppConsole_GetViewHoldMask());
        jog_request.wheel_reset_allowed = current.logic.command.valid &&
            current.logic.command.failsafe_ok && current.logic.command.wheel_switch == RC_LOW;
        CanBus_SetOutputPermission(current.logic.safety.output_allowed && jog_request.friction_requested);
        (void)MotorBackend_Apply(&jog_request, current.logic.safety.output_allowed);
        FeederBackend_Apply(&jog_request, current.logic.safety.output_allowed,
                            current.motors.flywheels_ready, 0U);
        if (FeederBackend_TakeRearmRequest()) {

            AppTasks_RequestStop();
            current.logic.safety.output_allowed = current.logic.safety.operator_armed = 0U;
        }
        YawBackend_Apply(&jog_request, current.logic.safety.output_allowed, 0U);
        GimbalStepSync_Update(&step_sync, &current.logic.targets,
            jog_request.gimbal_mode == GIMBAL_TEST && current.logic.safety.output_allowed);
        if (YawBackend_TakeRearmRequest()) {

            AppTasks_RequestStop();
            current.logic.safety.output_allowed = current.logic.safety.operator_armed = 0U;
        }
        PitchBackend_Apply(&jog_request, current.logic.safety.output_allowed, 0U);
        GimbalStepSync_Update(&step_sync, &current.logic.targets,
            jog_request.gimbal_mode == GIMBAL_TEST && current.logic.safety.output_allowed);
        if (PitchBackend_TakeRearmRequest()) {

            AppTasks_RequestStop();
            current.logic.safety.output_allowed = current.logic.safety.operator_armed = 0U;
        }
        if (MotorBackend_TakeRearmRequest()) {
            /* 单次点动结束后保持正向也不能重启；重新要求新OFF回中帧。 */

            AppTasks_RequestStop();
            current.logic.safety.output_allowed = 0U;
            current.logic.safety.operator_armed = 0U;
        }
        if (!CanBus_OutputAllowed()
            && !CanBus_FeederAllowed()
            && !CanBus_YawAllowed()
            && !CanBus_PitchAllowed()
            ) { current.logic.safety.output_allowed = 0U; }
        current.remote = input.remote;

        current.stop_sequence = input.stop_sequence;
        published = current;
        if (GimbalStartup_OutputReady()) {
            (void)FeederBackend_PrepareRemote();
        }
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
    }
}

void AppTasks_Create(void)
/*
 * 作用：初始化各后端和接收队列，创建遥控、CAN、控制及控制台任务。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    MotorBackend_Init();
    FeederBackend_Init();
    YawBackend_Init();
    PitchBackend_Init();
    GimbalStartup_Init();
    RemoteInput_CreateQueue();
    CanBus_CreateQueue();
    AppConsole_CreateQueue();
    if (xTaskCreate(RemoteTask, "remote", 384U, 0, 4U, 0) != pdPASS ||
        xTaskCreate(CanTask, "can_rx", 384U, 0, 4U, 0) != pdPASS ||
        xTaskCreate(ControlTask, "control", 768U, 0, 3U, 0) != pdPASS ||
        xTaskCreate(ConsoleTask, "console", 768U, 0, 2U, 0) != pdPASS) {
        AppFault_Fatal(APP_FAULT_TASK);
    }
}

static void RemoteTask(void *argument)
/*
 * 作用：启动 UART4 DMA 接收，并周期消费接收队列、解析和发布有效 iBUS 遥控帧。
 * 参数 argument：FreeRTOS 任务入口参数，本工程未使用。
 * 流程：中断负责搬运字节；校验和、通道解析和有效帧发布在本任务完成。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    TickType_t wake = xTaskGetTickCount();
    (void)argument;
    RemoteInput_Start();
    for (;;) {
        RemoteInput_Poll();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(2U));
    }
}

static void CanTask(void *argument)
/*
 * 作用：从 CAN 接收队列取出反馈，并交给各电机后端识别和更新状态。
 * 参数 argument：FreeRTOS 任务入口参数，本工程未使用。
 * 流程：只接收当前故障代次的帧，避免总线异常前排队的反馈恢复旧状态。
 * 流程：各后端按总线、帧格式和设备 ID 判断是否属于自己，帧不属于本设备时直接返回。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    CanFrame frame;
    CanBusState bus;
    (void)argument;
    CanBus_Start();
    for (;;) {
        if (CanBus_Receive(&frame, pdMS_TO_TICKS(2U)) == pdPASS) {
            CanBus_GetState(&bus);
            if (frame.fault_epoch == bus.fault_epoch) {
                (void)MotorBackend_OnFrame(&frame);
                (void)FeederBackend_OnFrame(&frame);
                (void)YawBackend_OnFrame(&frame);
                (void)PitchBackend_OnFrame(&frame);
            }
        }
    }
}

static void ConsoleTask(void *argument)
/*
 * 作用：周期处理串口命令、准备流程回执和当前视图的 VOFA 遥测。
 * 参数 argument：FreeRTOS 任务入口参数，本工程未使用。
 * 流程：控制台任务的优先级低于遥控、CAN 和控制任务；波形显示不参与 PID 的反馈计算。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    TickType_t wake = xTaskGetTickCount();
    (void)argument;
    AppConsole_Start();
    for (;;) {
        AppConsole_Poll();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5U));
    }
}

void AppTasks_GetState(AppState *out)
/*
 * 作用：在临界区内复制已发布的应用、遥控及安全状态。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值，将当前快照写入输出参数；不会启动新的运动。
 */
{
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    *out = published;
    taskEXIT_CRITICAL();
}

void AppTasks_RequireRearm(void)
/*
 * 作用：递增停机屏障序号，要求后续通过新的安全输入重新解锁。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    taskENTER_CRITICAL();
    stop_sequence++;
    taskEXIT_CRITICAL();
}

void AppTasks_RequestStop(void)
/*
 * 作用：统一撤销本轮动力请求，并通知摩擦轮、拨盘和双轴后端进入停止流程。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 流程：递增 stop_sequence，使安全状态机在后续周期要求新的 OFF 帧和人工解锁。
 * 流程：CAN 邮箱提交或取消不代表机械已经停稳，设备停止状态继续由反馈确认。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    taskENTER_CRITICAL();
    stop_sequence++;
    CanBus_SetOutputPermission(0U);
    MotorBackend_Stop();
    FeederBackend_Stop();
    YawBackend_Stop();
    PitchBackend_Stop();
    taskEXIT_CRITICAL();
}
