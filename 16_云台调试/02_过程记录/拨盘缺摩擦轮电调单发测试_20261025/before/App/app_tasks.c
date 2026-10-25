/**
 * @file app_tasks.c
 * @brief 应用调度与启动
 *
 * 职责：创建四个 RTOS 任务，组合安全许可，协调双轴准备并发布状态快照。
 * 主要函数：
 *   - AppTasks_Create：初始化后端、队列及遥控/CAN/控制/控制台任务。
 *   - AppLogic_Step：先执行安全判定，再更新遥控目标和执行器请求。
 *   - GimbalStartup_Update：按安全 OFF、反馈及使能握手推进双轴准备。
 *   - AppTasks_RequestStop / AppFault_Fatal：分别处理可恢复停机与致命故障。
 * 调用关系：main → AppTasks_Create；控制任务每 APP_CONTROL_PERIOD_MS 调用各后端 Apply。
 * 维护约束：保护判定、许可及命令提交仍在原临界区；不得让过期快照重新开放输出。
 * 参数入口：app_config.h；对外声明仍在原头文件中。
 */

/* APP_SECTION_BEGIN app_fault.c */
/* 致命故障入口：交由 Error_Handler，避免在异常环境依赖 RTOS。 */
#include "app_fault.h"
#include "main.h"
void AppFault_Fatal(AppFault fault)
{
    (void)fault;
    /* 致命错误进入Error_Handler后无法调度停机重试。电调自身通信超时
     * 是最后兜底；此处不能在堆损坏/ISR环境依赖RTOS或阻塞发送。 */
    Error_Handler();
}
/* APP_SECTION_END app_fault.c */

/* APP_SECTION_BEGIN app_logic.c */
/* 应用纯逻辑：组合轴健康、安全状态和目标请求。 */
#include "app_logic.h"
#include "app_config.h"
#include <string.h>
void AppLogic_CombineActuatorHealth(SafetyInput *input, const AppAxisHealth *yaw,
                                   const AppAxisHealth *pitch, uint8_t wheels_wanted)
{
    uint8_t requested = 0U, healthy = 1U, faulted = 0U;
    const AppAxisHealth *axes[2] = { yaw, pitch };
    uint32_t i;
    if (!input) { return; }
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
{
    if (logic == 0) { return; }
    memset(logic, 0, sizeof(*logic));
    SafetyManager_Init(&logic->safety);
    TargetManager_Init(&logic->targets);
}
void AppLogic_RecordStop(AppLogic *logic, const SafetyInput *input, AppStopSource source)
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
/* APP_SECTION_END app_logic.c */

/* APP_SECTION_BEGIN gimbal_startup.c */
/* 双轴启动协调：安全 OFF、建立基准和原使能握手。 */
#include "gimbal_startup.h"
#include "app_config.h"
#include "app_tasks.h"
#include "yaw_backend.h"
#include "pitch_backend.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>

static GimbalStartupState state;
static uint32_t remote_sequence, guard_stop, guard_uart, guard_can;
static uint32_t barrier_ms, off_since_ms, started_ms, query_ms;
static uint8_t have_sequence, have_guard, off_seen, pending_event;
static uint8_t begun, yaw_zero_done, pitch_zero_done, clear_sent, manual_rezero;

static void Status(uint8_t phase, uint8_t reason, uint8_t ready)
{
    if (state.phase != phase || state.reason != reason || state.ready != ready) {
        state.phase = phase; state.reason = reason; state.ready = ready;
        ++state.sequence; pending_event = 1U;
    }
}
void GimbalStartup_Init(void)
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    have_sequence = have_guard = off_seen = pending_event = 0U;
    begun = yaw_zero_done = pitch_zero_done = clear_sent = 0U;
    manual_rezero = 0U;
    remote_sequence = guard_stop = guard_uart = guard_can = 0U;
    barrier_ms = off_since_ms = started_ms = query_ms = 0U;
    taskEXIT_CRITICAL();
}
static uint8_t SafeOff(const RemoteFrame *frame, uint32_t now)
{
    RemoteCommand command;
    if (!frame || !frame->has_frame || (uint32_t)(now - frame->received_ms) >= REMOTE_TIMEOUT_MS) { return 0U; }
    RemoteMapper_Map(frame, &command);
    return (uint8_t)(command.valid && command.failsafe_ok && command.arm == RC_LOW && command.neutral);
}
static void Barrier(uint32_t now)
{
    AppTasks_RequireRearm(); ++guard_stop;
    barrier_ms = now; off_seen = 0U;
}
static void CancelInitial(void)
{
    if (begun && !state.reference_latched) {
        (void)YawBackend_Request(0U); (void)PitchBackend_Request(0U);
    }
    begun = yaw_zero_done = pitch_zero_done = clear_sent = 0U;
}
void GimbalStartup_ManualOverride(void)
{
    taskENTER_CRITICAL(); CancelInitial(); off_seen = 0U;
    Status(GIMBAL_STARTUP_MANUAL, GIMBAL_STARTUP_NONE, 0U);
    taskEXIT_CRITICAL();
}
uint8_t GimbalStartup_Request(const RemoteFrame *frame, uint32_t now, uint8_t diagnostic)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL(); CanBus_GetState(&bus);
    if (!diagnostic && SafeOff(frame, now) && bus.started && !bus.fault_epoch) {
        GimbalStartup_Init(); manual_rezero = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}
void GimbalStartup_GetState(GimbalStartupState *out)
{ if (out) { taskENTER_CRITICAL(); *out = state; taskEXIT_CRITICAL(); } }
uint8_t GimbalStartup_TakeEvent(GimbalStartupState *out)
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (out && pending_event) { *out = state; pending_event = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t GimbalStartup_OutputReady(void)
{ return (uint8_t)(state.phase == GIMBAL_STARTUP_MANUAL || state.ready); }

void GimbalStartup_Update(const SafetyInput *input, uint8_t diagnostic)
{
    CanBusState bus;
    YawBackendState yaw;
    PitchBackendState pitch;
    uint8_t fresh, off;
    uint32_t now;
    if (!input || state.phase == GIMBAL_STARTUP_MANUAL) { return; }
    now = input->now_ms;
    fresh = (uint8_t)(input->remote.has_frame && (!have_sequence || remote_sequence != input->remote.sequence));
    if (fresh) { remote_sequence = input->remote.sequence; have_sequence = 1U; }
    if (!have_guard) {
        guard_stop = input->stop_sequence; guard_uart = input->uart_epoch; guard_can = input->can_epoch;
        have_guard = 1U; barrier_ms = now;
    } else if (guard_stop != input->stop_sequence || guard_uart != input->uart_epoch || guard_can != input->can_epoch) {
        guard_stop = input->stop_sequence; guard_uart = input->uart_epoch; guard_can = input->can_epoch;
        CancelInitial(); barrier_ms = now; off_seen = 0U;
        Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_STOP, 0U);
    }
    CanBus_GetState(&bus);
    if (!bus.started) {
        off_seen = 0U; Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_CAN, 0U); return;
    }
    if (bus.fault_epoch || input->can_epoch) {
        CancelInitial(); off_seen = 0U;
        Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_CAN, 0U); return;
    }
    if (state.phase == GIMBAL_STARTUP_BLOCKED) {
        if (state.reference_latched) {
            YawBackend_GetState(&yaw); PitchBackend_GetState(&pitch);
            if (!yaw.zero_set || !pitch.zero_set || !pitch.controller.gravity_calibrated) {
                Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_REFERENCE_LOST, 0U);
            }
        }
        return;
    }
    if (diagnostic || input->control_late) {
        CancelInitial(); off_seen = 0U; barrier_ms = now;
        Status(GIMBAL_STARTUP_WAIT_OFF, diagnostic ? GIMBAL_STARTUP_DIAGNOSTIC : GIMBAL_STARTUP_STOP, 0U); return;
    }
    YawBackend_GetState(&yaw); PitchBackend_GetState(&pitch);
    off = SafeOff(&input->remote, now);
    if (state.reference_latched) {
        /* Never move the mechanical limits by silently capturing a new zero after use. */
        if (!yaw.zero_set || !pitch.zero_set || !pitch.coord_confirmed || !pitch.controller.gravity_calibrated) {
            Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_REFERENCE_LOST, 0U); return;
        }
        if (yaw.controller.faults || pitch.controller.faults) {
            Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_FAULT, 0U); return;
        }
        if (yaw.online && pitch.online && !yaw.stop_pending && !pitch.stop_pending && yaw.requested && pitch.requested) {
            if (!state.ready) {
                if (!off || !fresh || (int32_t)(input->remote.received_ms - barrier_ms) <= 0) { return; }
                Barrier(now);
            }
            Status(GIMBAL_STARTUP_READY, GIMBAL_STARTUP_NONE, 1U); return;
        }
        Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_NONE, 0U);
        if (!off || !fresh || (int32_t)(input->remote.received_ms - barrier_ms) <= 0 ||
            !yaw.online || !pitch.online || yaw.active || pitch.active || yaw.enabling || pitch.enabling ||
            yaw.stop_pending || pitch.stop_pending || yaw.feedback.status != 0U || pitch.feedback.status != 0U) { return; }
        if (!yaw.requested) { (void)YawBackend_Request(1U); }
        if (!pitch.requested) { (void)PitchBackend_Request(1U); }
        return;
    }
    if (!off || (int32_t)(input->remote.received_ms - barrier_ms) <= 0) {
        if (begun) { CancelInitial(); Barrier(now); }
        off_seen = 0U;
        Status(GIMBAL_STARTUP_WAIT_OFF, GIMBAL_STARTUP_REMOTE, 0U); return;
    }
    if (!off_seen) {
        if (!fresh) { return; }
        off_seen = 1U; off_since_ms = now;
    }
    if (!begun) {
        if (!fresh || (uint32_t)(now - off_since_ms) < APP_GIMBAL_STARTUP_OFF_HOLD_MS) { return; }
        if (!YawBackend_PrepareStop()) { return; }
        (void)PitchBackend_Request(0U); (void)PitchBackend_QueryFeedback();
        begun = 1U; started_ms = query_ms = now;
        yaw_zero_done = pitch_zero_done = clear_sent = 0U;
        Status(GIMBAL_STARTUP_PREPARING, GIMBAL_STARTUP_NONE, 0U); return;
    }
    if ((uint32_t)(now - started_ms) >= APP_GIMBAL_STARTUP_TIMEOUT_MS) {
        CancelInitial(); Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_TIMEOUT, 0U); return;
    }
    if (state.phase == GIMBAL_STARTUP_BLOCKED) { return; }
    if ((uint32_t)(now - query_ms) >= 20U) { (void)PitchBackend_QueryFeedback(); query_ms = now; }
    /* Only the existing, fully guarded status13 recovery may send a single FB at boot. */
    if (yaw.online && yaw.feedback.status == 13U && !clear_sent) {
        if (YawBackend_ClearCommsFault()) { clear_sent = 1U; }
    }
    if (!manual_rezero && ((yaw.controller.faults && !((yaw.feedback.status == 13U || (clear_sent && yaw.feedback.status == 0U)) &&
        (yaw.controller.faults & ~(uint32_t)(YAW_CONTROLLER_FAULT_STATUS | YAW_CONTROLLER_FAULT_SPEED | YAW_CONTROLLER_FAULT_JUMP)) == 0U)) ||
        pitch.controller.faults || (yaw.online && yaw.feedback.status != 0U && yaw.feedback.status != 13U) ||
        (pitch.online && pitch.feedback.status != 0U))) {
        /* A status1 still in flight immediately after FD is awaited, not enabled by this path. */
        if (yaw.feedback.status != 1U && pitch.feedback.status != 1U) {
            CancelInitial(); Status(GIMBAL_STARTUP_BLOCKED, GIMBAL_STARTUP_FAULT, 0U); return;
        }
    }
    if (!yaw_zero_done && yaw.online && !yaw.stop_pending && yaw.feedback.status == 0U &&
        (!yaw.controller.faults || manual_rezero || (clear_sent && (yaw.controller.faults &
        ~(uint32_t)(YAW_CONTROLLER_FAULT_STATUS | YAW_CONTROLLER_FAULT_SPEED | YAW_CONTROLLER_FAULT_JUMP)) == 0U))) {
        if (YawBackend_CaptureReference() && YawBackend_Request(1U)) { yaw_zero_done = 1U; }
    }
    if (!pitch_zero_done && pitch.online && !pitch.stop_pending && pitch.feedback.status == 0U &&
        (!pitch.controller.faults || manual_rezero)) {
        if ((!pitch.coord_confirmed && !PitchBackend_SetCoordinate(1U))) { return; }
        if (PitchBackend_CaptureReference() && PitchBackend_SetGravityOffset(0.0f, 1U) && PitchBackend_Request(1U)) {
            pitch_zero_done = 1U;
        }
    }
    YawBackend_GetState(&yaw); PitchBackend_GetState(&pitch);
    if (yaw_zero_done && pitch_zero_done && yaw.requested && pitch.requested &&
        yaw.online && pitch.online && !yaw.stop_pending && !pitch.stop_pending &&
        !yaw.controller.faults && !pitch.controller.faults && yaw.feedback.status == 0U && pitch.feedback.status == 0U) {
        YawBackend_ReleasePreparation(); state.reference_latched = 1U;
        Barrier(now); Status(GIMBAL_STARTUP_READY, GIMBAL_STARTUP_NONE, 1U);
    }
}
/* APP_SECTION_END gimbal_startup.c */

/* APP_SECTION_BEGIN app_tasks.c */
/* RTOS 调度：队列、任务、全局停机和控制周期。 */
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
#if APP_YAW_ENABLED && APP_PITCH_ENABLED
#include "gimbal_step_sync.h"
#endif
#include "task.h"
#include <string.h>

static AppState published;
static volatile uint32_t stop_sequence;

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
    GimbalStepSync step_sync = {0U, 0U};
#endif
#endif
#if APP_PITCH_ENABLED
    PitchBackendState pitch;
#if !APP_YAW_ENABLED
    uint32_t pitch_step_observed = 0U, pitch_step_current;
#endif
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
#if APP_YAW_ENABLED && APP_PITCH_ENABLED
        GimbalStepSync_ApplyViewHold(AppConsole_GetViewHoldMask());
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
        GimbalStepSync_Update(&step_sync, &current.logic.targets,
            demanded.gimbal_mode == GIMBAL_TEST && current.logic.safety.output_allowed &&
            !AppConsole_IsDiagnosticMode());
#elif APP_PITCH_ENABLED
        pitch_step_current = PitchBackend_GetStepId();
        if (pitch_step_current != pitch_step_observed) {
            TargetManager_RecordAcceptedTestAxis(&current.logic.targets, GIMBAL_STEP_PITCH);
            pitch_step_observed = pitch_step_current;
        }
#endif
        AppLogic_Step(&current.logic, &input, current.motors.flywheels_ready);
        jog_request = current.logic.targets.request;
#if APP_YAW_ENABLED && APP_PITCH_ENABLED
        GimbalStepSync_FilterViewRequest(&jog_request, AppConsole_GetViewHoldMask());
#endif
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
        GimbalStepSync_Update(&step_sync, &current.logic.targets,
            jog_request.gimbal_mode == GIMBAL_TEST && current.logic.safety.output_allowed &&
            !AppConsole_IsDiagnosticMode());
#endif
        if (YawBackend_TakeRearmRequest()) {
            AppLogic_RecordStop(&current.logic, &input, APP_STOP_SOURCE_YAW);
            AppTasks_RequestStop();
            current.logic.safety.output_allowed = current.logic.safety.operator_armed = 0U;
        }
#endif
#if APP_PITCH_ENABLED
        PitchBackend_Apply(&jog_request, current.logic.safety.output_allowed, AppConsole_IsDiagnosticMode());
#if APP_YAW_ENABLED
        GimbalStepSync_Update(&step_sync, &current.logic.targets,
            jog_request.gimbal_mode == GIMBAL_TEST && current.logic.safety.output_allowed &&
            !AppConsole_IsDiagnosticMode());
#else
        pitch_step_current = PitchBackend_GetStepId();
        if (pitch_step_current != pitch_step_observed) {
            TargetManager_RecordAcceptedTestAxis(&current.logic.targets, GIMBAL_STEP_PITCH);
            pitch_step_observed = pitch_step_current;
        }
#endif
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
/* APP_SECTION_END app_tasks.c */
