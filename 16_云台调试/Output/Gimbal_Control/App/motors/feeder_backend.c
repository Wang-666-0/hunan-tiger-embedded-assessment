/**
 * @file feeder_backend.c
 * @brief 拨盘准备、触发与 CAN 执行
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（14）：
 *   FeederBackend_Init / FeederBackend_Stop / FeederBackend_ManualOff / FeederBackend_OnFrame /
 *   FeederBackend_GetState / FeederBackend_TakeRearmRequest / FeederBackend_SetConfig /
 *   FeederBackend_Clear / FeederBackend_Zero / FeederBackend_Arm / FeederBackend_Ready /
 *   FeederBackend_PrepareRemote / FeederBackend_Single / FeederBackend_Apply
 * 内部函数（2）：
 *   Fresh / RemoteGuard
 * 文件级变量/常量（15；局部变量见对应函数）：
 *   state：本模块维护并向外发布的设备或流程状态快照。
 *   owned：本后端是否已经拥有过输出，供停止重试使用。
 *   rearm：需要操作员重新解锁的待消费标记。
 *   was_allowed：上一控制周期的拨盘许可状态。
 *   single_pending：待执行的一次串口单发请求。
 *   have_completed：是否已有可用于后续门禁的完成动作。
 *   last_tx_ms：最近一次控制命令的提交时间。
 *   last_success_ms：最近一次成功提交拨盘输出的时间。
 *   event_sequence：最近已处理的拨盘事件序号。
 *   single_ms：串口单发请求接受时间。
 *   completed_ms：最近一发动作完成时间。
 *   auto_quiet_since：自动准备所需静止观察的起始时间。
 *   auto_quiet：自动准备是否正在进行静止观察。
 *   reference_captured：本次自动准备是否已经捕获软件参考。
 *   manual_hold：显式人工关闭对自动准备的抑制标记。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 拨盘会话：发射互锁、单发/连发请求与 CAN 输出。 */
#include "feeder_backend.h"
#include "app_config.h"
#include "app_tasks.h"
#include "app_console.h"
#include "can_bus.h"
#include "remote_input.h"
#include "motor_backend.h"
#include "task.h"
#include <string.h>

static FeederBackendState state;
static uint8_t owned, rearm, was_allowed, single_pending, have_completed;
static uint32_t last_tx_ms, last_success_ms, event_sequence, single_ms, completed_ms;
static uint32_t auto_quiet_since;
static uint8_t auto_quiet, reference_captured, manual_hold;

static uint8_t Fresh(void)
/* 功能：检查拨盘反馈是否新鲜，并与当前 CAN 故障代次一致。 */
{
    return state.controller.have_feedback &&
        HAL_GetTick() - state.controller.last_feedback_ms < FEEDER_FEEDBACK_TIMEOUT_MS;
}

/* Caller holds the same critical section as mutation: a stale UI snapshot never authorizes motion. */
static uint8_t RemoteGuard(uint8_t off)
/* 功能：按操作类型核对拨盘所需的遥控 OFF/解锁、回中和输入时效条件。 */
{
    RemoteInput_State remote;
    RemoteFrame frame;
    RemoteCommand command;
    AppState app;
    CanBusState bus;
    RemoteInput_GetState(&remote); AppTasks_GetState(&app); CanBus_GetState(&bus);
    memset(&frame, 0, sizeof(frame)); memcpy(frame.channels, remote.channels, sizeof(frame.channels));
    frame.has_frame = remote.has_frame;
    RemoteMapper_Map(&frame, &command);
    if (!remote.has_frame || HAL_GetTick() - remote.received_ms >= REMOTE_TIMEOUT_MS ||
        !command.valid || !command.failsafe_ok || command.gimbal_mode != GIMBAL_MANUAL ||
        0U || remote.needs_rearm ||
        app.logic.safety.uart_epoch != remote.fault_epoch + AppConsole_GetFaultEpoch() ||
        !bus.started || bus.fault_epoch) { return 0U; }
    if (off) {
        return command.arm == RC_LOW && command.neutral &&
            app.logic.safety.state == SAFETY_READY && !app.logic.safety.barrier_valid &&
            !app.logic.safety.operator_armed &&
            !app.logic.safety.output_allowed && !CanBus_FeederAllowed();
    }
    return command.arm == RC_HIGH && command.trigger == RC_LOW &&
        (APP_FEEDER_STANDALONE_TEST_ACTIVE ? command.fire_mode == FIRE_SINGLE : command.friction_on) &&
        app.logic.safety.operator_armed && app.logic.safety.output_allowed;
}

void FeederBackend_Init(void)
/* 功能：初始化拨盘控制器、发送时序、准备流程和单次触发锁存。 */
{
    memset(&state, 0, sizeof(state)); Feeder_Init(&state.controller);
    owned = rearm = was_allowed = single_pending = have_completed = 0U;
    last_tx_ms = last_success_ms = event_sequence = single_ms = completed_ms = 0U;
    auto_quiet_since = 0U; auto_quiet = reference_captured = manual_hold = 0U;
}

void FeederBackend_Stop(void)
/* 功能：撤销拨盘动力许可和待执行请求，启动原零电流停止流程。 */
{
    taskENTER_CRITICAL();
    state.armed = was_allowed = single_pending = 0U;
    if (state.controller.busy && state.controller.outstanding) { state.controller.have_zero = 0U; }
    Feeder_Stop(&state.controller);
    CanBus_SetFeederPermission(0U);
    if (owned) { state.stop_pending = 1U; }
    taskEXIT_CRITICAL();
}

void FeederBackend_ManualOff(void)
/* 功能：处理拨盘人工关闭，防止自动准备立即覆盖显式停止意图。 */
{
    taskENTER_CRITICAL(); manual_hold = 1U; FeederBackend_Stop(); taskEXIT_CRITICAL();
}

uint8_t FeederBackend_OnFrame(const CanFrame *frame)
/* 功能：接收拨盘有效反馈，并同步零点、故障和已完成动作信息。 */
{
    CanBusState bus;
    uint8_t accepted;
    uint32_t faults;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus); faults = state.controller.faults;
    accepted = Feeder_OnFrame(&state.controller, frame, HAL_GetTick(), bus.fault_epoch);
    if (state.armed && faults != state.controller.faults) { FeederBackend_Stop(); rearm = 1U; }
    taskEXIT_CRITICAL(); return accepted;
}

void FeederBackend_GetState(FeederBackendState *out)
/* 功能：复制拨盘控制器、反馈、许可和动作状态快照。 */
{
    if (!out) { return; }
    taskENTER_CRITICAL(); *out = state; out->online = Fresh(); taskEXIT_CRITICAL();
}

uint8_t FeederBackend_TakeRearmRequest(void)
/* 功能：取出并清除拨盘要求重新解锁的单次标记。 */
{
    uint8_t value;
    taskENTER_CRITICAL(); value = rearm; rearm = 0U; taskEXIT_CRITICAL(); return value;
}

uint8_t FeederBackend_SetConfig(const FeederConfig *config)
/* 功能：在安全关闭条件下校验并设置拨盘运行配置。 */
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.armed && !state.stop_pending && RemoteGuard(1U)) {
        result = Feeder_SetConfig(&state.controller, config);
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Clear(void)
/* 功能：在原安全门禁允许时清除拨盘可恢复故障。 */
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.armed && !state.stop_pending && RemoteGuard(1U)) {
        result = Feeder_Clear(&state.controller, HAL_GetTick());
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Zero(void)
/* 功能：在原静止且安全关闭条件下建立拨盘软件零点。 */
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.armed && !state.stop_pending && RemoteGuard(1U)) {
        result = Feeder_Zero(&state.controller, HAL_GetTick());
        if (result) { reference_captured = 1U; }
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Arm(void)
/* 功能：核对准备条件后请求拨盘待命，实际输出仍受实时许可约束。 */
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    if (!state.stop_pending && RemoteGuard(1U) && Fresh() && !state.controller.faults &&
        state.controller.have_zero && state.controller.config.calibrated &&
        (state.controller.config.direction == 1 || state.controller.config.direction == -1)) {
        state.armed = owned = state.stop_pending = 1U;
        rearm = was_allowed = single_pending = 0U; result = 1U;
        reference_captured = 1U;
        manual_hold = 0U;
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Ready(void)
/* 功能：按原安全关闭流程完成拨盘清错、软件设零和待命准备。 */
{
    uint8_t result = 0U;
    taskENTER_CRITICAL();
    /* Explicit human reference acknowledgement at safe OFF; never a boot action.
     * All original clear/zero/arm guards still apply, including quiet feedback. */
    if (!state.armed && !state.stop_pending && RemoteGuard(1U) && Fresh() &&
        state.controller.config.calibrated && state.controller.config.direction &&
        Feeder_Clear(&state.controller, HAL_GetTick()) &&
        Feeder_Zero(&state.controller, HAL_GetTick())) { result = FeederBackend_Arm(); }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_PrepareRemote(void)
/* 功能：在新鲜安全 OFF 输入及静止反馈下自动准备拨盘，尊重人工关闭意图。 */
{
    uint8_t result = 0U;
    uint32_t now;
    taskENTER_CRITICAL(); now = HAL_GetTick();
    /* Called only after the original gimbal preparation is ready. No motor
     * movement during OFF; one initial reference per boot, never silent fault
     * clearing or recapturing a half-shot reference after an interruption. */
    if (manual_hold || state.armed || state.stop_pending || !RemoteGuard(1U) || !Fresh() ||
        state.controller.faults || !state.controller.config.calibrated ||
        !state.controller.config.direction ||
        (reference_captured && !state.controller.have_zero) ||
        state.controller.raw_rpm > state.controller.config.settle_rpm ||
        state.controller.raw_rpm < -state.controller.config.settle_rpm) { auto_quiet = 0U; }
    else {
        if (!auto_quiet) { auto_quiet = 1U; auto_quiet_since = now; }
        if (now - auto_quiet_since >= 100U && state.controller.recovery_samples >= 3U &&
            (state.controller.have_zero || FeederBackend_Zero())) { result = FeederBackend_Arm(); }
    }
    taskEXIT_CRITICAL(); return result;
}

uint8_t FeederBackend_Single(void)
/* 功能：核对单发门禁并锁存一次串口触发，不排队补发。 */
{
    uint8_t result = 0U;
    MotorHealth wheels;
    taskENTER_CRITICAL(); MotorBackend_GetHealth(&wheels);
    if (state.armed && !state.stop_pending && !single_pending && !state.controller.busy &&
        Fresh() && !state.controller.faults &&
        (APP_FEEDER_STANDALONE_TEST_ACTIVE || wheels.flywheels_ready) && RemoteGuard(0U)) {
        single_pending = 1U; single_ms = HAL_GetTick(); result = 1U;
    }
    taskEXIT_CRITICAL(); return result;
}

void FeederBackend_Apply(const TargetRequest *request, uint8_t output_allowed,
                         uint8_t wheels_ready, uint8_t inhibited)
/* 功能：根据真实反馈、运行许可和请求序号执行拨盘单发/连发，并提交受限电流。 */
{
    uint32_t now, accepted, completed, aborted, faults;
    uint8_t eligible, allowed, event, was_busy, console_single, shot_ready;
    int16_t current;
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL(); now = HAL_GetTick();
    was_busy = state.controller.busy; faults = state.controller.faults;
    eligible = request && output_allowed && !inhibited && request->gimbal_mode == GIMBAL_MANUAL &&
        (APP_FEEDER_STANDALONE_TEST_ACTIVE ?
            (request->fire_mode == FIRE_SINGLE || request->fire_mode == FIRE_CONTINUOUS) : request->friction_requested);
    /* 独立拨盘测试不使用摩擦轮稳速门禁；真实 wheels_ready 遥测保持原值。
     * allowed、拨盘反馈、故障、零点、会话及 CAN 保护均按原流程执行。 */
    shot_ready = APP_FEEDER_STANDALONE_TEST_ACTIVE || wheels_ready;
    /* Speed readiness authorizes a new shot, not a half-shot abort. Actual
     * wheel feedback loss/fault still revokes the shared output permission. */
    allowed = state.armed && eligible && (shot_ready || was_busy) && !state.stop_pending;
    console_single = single_pending;
    event = request && request->shot_event;
    if (single_pending) {
        if (now - single_ms <= APP_CONTROL_MAX_GAP_MS && allowed) { event = 1U; }
        else { state.controller.rejected++; }
        single_pending = 0U;
    }
    if (event) { event_sequence++; }
    accepted = state.controller.accepted; completed = state.controller.completed;
    aborted = state.controller.aborted;
    current = Feeder_Run(&state.controller, now, allowed, shot_ready, event, event_sequence,
                        request && request->continuous_fire && !console_single);
    if (state.controller.accepted != accepted) {
        if (!console_single && request && request->fire_mode == FIRE_CONTINUOUS) { state.continuous_accepted++; }
        else { state.single_accepted++; }
    }
    if (state.controller.completed != completed) {
        state.completed_interval_ms = have_completed ? now - completed_ms : 0U;
        completed_ms = now; have_completed = 1U;
    }
    if (state.controller.aborted != aborted ||
        (state.armed && state.controller.faults != faults)) {
        if (state.controller.aborted != aborted) { state.controller.have_zero = 0U; }
        FeederBackend_Stop(); rearm = 1U; allowed = 0U;
    } else if (was_allowed && !eligible) {
        if (request && output_allowed && !inhibited && request->gimbal_mode == GIMBAL_MANUAL) {
            /* 空闲CH6普通关闭保留拨盘会话，沿用原遥控开关习惯；
             * 目标和串口事件已清，重新开轮仍需稳速与新鲜释放/触发。 */
            single_pending = 0U; CanBus_SetFeederPermission(0U); state.stop_pending = owned;
        } else { FeederBackend_Stop(); }
        allowed = 0U;
    }
    if (!allowed) {
        if (was_allowed) { CanBus_SetFeederPermission(0U); state.stop_pending = owned; }
        if (owned && (state.stop_pending || now - last_tx_ms >= 20U)) {
            result = CanBus_StopFeeder(); last_tx_ms = now;
            state.stop_pending = result != HAL_OK;
        }
    } else {
        if (!was_allowed) { last_success_ms = now; }
        CanBus_SetFeederPermission(1U);
        if (now - last_tx_ms >= APP_FEEDER_COMMAND_PERIOD_MS) {
            result = CanBus_FeederCurrent(current); last_tx_ms = now;
            if (result == HAL_OK) { last_success_ms = now; }
            if (result == HAL_ERROR || (result == HAL_BUSY && now - last_success_ms > 10U)) {
                state.controller.faults |= FEEDER_FAULT_CAN;
                state.controller.have_zero = 0U; FeederBackend_Stop(); rearm = 1U; allowed = 0U;
            }
        }
    }
    was_allowed = allowed;
    taskEXIT_CRITICAL();
}
