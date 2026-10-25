/**
 * @file yaw_console.c
 * @brief Yaw 串口命令和真实成绩
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（3）：
 *   YawConsole_CancelPreparation / YawConsole_Poll / YawConsole_Handle
 * 内部函数（9）：
 *   SafeOffReason / SafeOff / PreparationTick / SafeStep / StepBlockReason / yaw_console_StepReport
 *   / ClearReport / BusReport / yaw_console_BriefStatus
 * 文件级变量/常量（3；局部变量见对应函数）：
 *   ready_phase：Yaw 人工准备流程当前阶段。
 *   ready_clear_sent：本次准备是否已经发送过固定清错帧。
 *   ready_started_ms：Yaw 本次准备流程开始时间。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* YAW 命令：独立准备流程、配置查询和实测报告。 */
#include "yaw_console.h"
#include "yaw_backend.h"
#include "app_config.h"
#include "app_tasks.h"
#include "app_console.h"
#include "console_parse.h"
#include "remote_input.h"
#include "can_bus.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/* READY是一次有时限的人工会话；每个阶段重新验证安全OFF，绝不自动使能。 */
static uint8_t ready_phase, ready_clear_sent;
static uint32_t ready_started_ms;

static const char *SafeOffReason(RemoteInput_State *remote_out)
/* 功能：核对安全 OFF、新鲜反馈和遥控回中条件，返回准备操作的阻塞原因。 */
{
    RemoteInput_State remote;
    RemoteFrame frame;
    RemoteCommand command;
    AppState app;
    RemoteInput_GetState(&remote); AppTasks_GetState(&app);
    if (remote_out) { *remote_out = remote; }
    memset(&frame, 0, sizeof(frame));
    memcpy(frame.channels, remote.channels, sizeof(frame.channels));
    frame.has_frame = remote.has_frame; frame.received_ms = remote.received_ms;
    RemoteMapper_Map(&frame, &command);
    if (!remote.has_frame || (uint32_t)(HAL_GetTick() - remote.received_ms) >= REMOTE_TIMEOUT_MS) { return "STALE"; }
    if (!command.valid) { return "INVALID"; }
    if (!command.failsafe_ok) { return "CH8_LOCK"; }
    if (command.arm != RC_LOW) { return "CH5_NOT_OFF"; }
    if (!command.neutral) { return "NOT_NEUTRAL"; }
    if (app.logic.safety.operator_armed) { return "ARMED"; }

    return "OK";
}

static uint8_t SafeOff(void)
/* 功能：判断 Yaw 当前是否满足准备流程要求的安全 OFF 条件。 */
{ return strcmp(SafeOffReason(0), "OK") == 0; }

void YawConsole_CancelPreparation(void)
/* 功能：取消尚未完成的 Yaw 准备流程，避免旧操作继续接管会话。 */
{
    taskENTER_CRITICAL();
    if (ready_phase) { ready_phase = 0U; (void)YawBackend_Request(0U); }
    taskEXIT_CRITICAL();
}

static const char *PreparationTick(void)
/* 功能：按反馈和超时推进 Yaw 停止、监测、参考捕获及请求准备流程。 */
{
    YawBackendState yaw;
    CanBusDiagnostics bus;
    const char *reason;
    uint32_t now = HAL_GetTick();
    if (!ready_phase) { return 0; }
    reason = SafeOffReason(0);
    if (strcmp(reason, "OK") != 0) { goto failed; }
    CanBus_GetDiagnostics(&bus);
    if (!bus.state.started || bus.state.fault_epoch || bus.hal_error[0] || bus.hal_error[1]) {
        reason = "CAN_FAULT_USE_YAW_BUS"; goto failed;
    }
    YawBackend_GetState(&yaw);
    if (yaw.controller.faults & (YAW_CONTROLLER_FAULT_CAN | (1UL << 17))) {
        reason = "CAN_OR_TX_FAULT_USE_YAW_BUS"; goto failed;
    }
    if ((uint32_t)(now - ready_started_ms) >= YAW_READY_TIMEOUT_MS) {
        reason = "TIMEOUT_USE_YAW_STATUS"; goto failed;
    }
    if (!yaw.online) { return 0; }
    if (ready_phase == 2U) {
        if (!yaw.requested || !yaw.zero_set || yaw.controller.faults || yaw.feedback.status != 0U) {
            reason = "PREPARE_STATE_CHANGED"; goto failed;
        }
        if (yaw.stop_pending) { return 0; }
        YawBackend_ReleasePreparation(); ready_phase = 0U; return "OK";
    }
    if (yaw.feedback.status == 13U) {
        /* 仅人工READY这一次会话允许单次FB；沿用既有全部清错门禁。 */
        if (!ready_clear_sent && YawBackend_ClearCommsFault()) { ready_clear_sent = 1U; }
        return 0;
    }
    if (yaw.feedback.status > 1U) { reason = "DRIVER_STATUS_USE_YAW_STATUS"; goto failed; }
    if (yaw.feedback.status != 0U || yaw.stop_pending || !YawBackend_CaptureReference()) { return 0; }
    AppTasks_RequestStop();
    if (!YawBackend_Request(1U)) { reason = "PREPARE_REJECTED"; goto failed; }
    ready_phase = 2U;
    return 0;
failed:
    ready_phase = 0U;
    (void)YawBackend_Request(0U); AppTasks_RequestStop();
    return reason;
}

static uint8_t SafeStep(void)
/* 功能：核对当前遥控、解锁与回中条件是否允许串口阶跃。 */
{
    RemoteInput_State remote;
    RemoteFrame frame;
    RemoteCommand command;
    AppState app;
    RemoteInput_GetState(&remote); AppTasks_GetState(&app);
    memset(&frame, 0, sizeof(frame)); memcpy(frame.channels, remote.channels, sizeof(frame.channels));
    frame.has_frame = remote.has_frame; frame.received_ms = remote.received_ms;
    RemoteMapper_Map(&frame, &command);
    return (uint8_t)(remote.has_frame && (uint32_t)(HAL_GetTick() - remote.received_ms) < REMOTE_TIMEOUT_MS &&
        command.valid && command.failsafe_ok && command.neutral && command.arm == RC_HIGH &&
        (command.gimbal_mode == GIMBAL_MANUAL || command.gimbal_mode == GIMBAL_TEST) &&
        app.logic.safety.operator_armed);
}

static const char *StepBlockReason(uint32_t blocks)
/* 功能：把 Yaw 阶跃门禁位转换为优先显示的拒绝原因。 */
{
    if (blocks & YAW_STEP_BLOCK_VIEW) { return "VIEW_LOCKED_USE_YAW_VIEW"; }
    if (blocks & YAW_STEP_BLOCK_FAULT) { return "FAULT"; }
    if (blocks & YAW_STEP_BLOCK_REFERENCE) { return "NEED_ZERO"; }
    if (blocks & YAW_STEP_BLOCK_FEEDBACK) { return "FEEDBACK"; }
    if (blocks & YAW_STEP_BLOCK_TRANSITION) { return "WAIT_ENABLE_OR_STOP"; }
    if (blocks & YAW_STEP_BLOCK_INACTIVE) { return "NOT_ACTIVE"; }
    if (blocks & YAW_STEP_BLOCK_PERMISSION) { return "REMOTE_OR_OUTPUT_GUARD"; }
    if (blocks & YAW_STEP_BLOCK_TARGET_LIMIT) { return "TARGET_LIMIT"; }
    if (blocks & YAW_STEP_BLOCK_WINDOW) { return "WAIT_5S_WINDOW"; }
    if (blocks & YAW_STEP_BLOCK_HOME) { return "SEND_HOME_FIRST"; }
    if (blocks & YAW_STEP_BLOCK_POSITION) { return "WAIT_TARGET_BAND"; }
    if (blocks & YAW_STEP_BLOCK_MOVING) { return "WAIT_STILL"; }
    if (blocks & YAW_STEP_BLOCK_STILL) { return "WAIT_STILL_HOLD"; }
    return "READY";
}

static uint8_t yaw_console_StepReport(const YawStep *step, char *response, size_t capacity)
/* 功能：格式化Yaw真实阶跃成绩、控制参数和终止原因。 */
{
    static const char *phases[] = { "IDLE", "RUNNING", "DONE", "ABORTED" };
    static const char *reasons[] = { "NONE", "STOP", "FAULT", "PID_CHANGE", "TARGET_CHANGE", "MANUAL", "GAP", "DATA", "FF_CHANGE" };
    uint32_t rise = step->t10_ms != YAW_STEP_UNREACHED && step->t90_ms != YAW_STEP_UNREACHED ?
        step->t90_ms - step->t10_ms : YAW_STEP_UNREACHED;
    int length = snprintf(response, capacity,
        "Y|STEP|ID|%lu|STATE|%s|REASON|%s|VALID|%u|START|%.3f|TARGET|%.3f|WINDOW_MS|%lu|RESP_MS|%ld|T90_MS|%ld|RISE_MS|%ld|SETTLE_MS|%ld|OS_DEG|%.6f|OS_PCT|%.3f|SSE_DEG|%.6f|SSE_VALID|%u|PASS|%u|FAULT|%lu|VMAX_DPS|%.2f|TMAX_NM|%.5f|N|%lu|GAP_MS|%lu|%s|%lu|%s|%.4f|%s|%.4f|%s|%.4f|CMD_N|%lu|FIRST_NM|%.5f",
        (unsigned long)step->id, step->phase <= YAW_STEP_ABORTED ? phases[step->phase] : "UNKNOWN",
        step->reason <= YAW_STEP_FF_CHANGE ? reasons[step->reason] : "UNKNOWN",
        (unsigned)(step->phase == YAW_STEP_DONE), (double)step->start_deg, (double)step->target_deg,
        (unsigned long)step->elapsed_ms, (long)(int32_t)step->response_ms, (long)(int32_t)step->t90_ms,
        (long)(int32_t)rise, (long)(int32_t)step->settle_ms, (double)step->overshoot_deg,
        (double)step->overshoot_pct, (double)step->steady_error_deg, (unsigned)step->sse_valid,
        (unsigned)step->pass, (unsigned long)step->faults, (double)step->peak_velocity_dps,
        (double)step->peak_torque_nm, (unsigned long)step->samples, (unsigned long)step->max_gap_ms,
        step->cascade ? "SAT_N" : "LIMIT_N", (unsigned long)step->limited_samples,
        step->cascade ? "PKP" : "KP", (double)step->kp,
        step->cascade ? "PKI" : "KI", (double)step->ki,
        step->cascade ? "PKD" : "KD", (double)step->kd,
        (unsigned long)step->command_samples, (double)step->first_command_nm);
    if (length <= 0 || (size_t)length >= capacity) { return 0U; }
    {
        size_t used = (size_t)length;
        if (step->cascade) {
            length = snprintf(response + used, capacity - used,
                "|CTRL|CASCADE|VKP|%.6f|VKI|%.6f|VKD|%.6f|VREF_MAX_DPS|%.1f|VFILTER_MS|%u|OUTPUT_LIMITS|%u"
                "|BAND02|0.2|RESP02_MS|%ld|SETTLE02_MS|%ld|PASS02|%u"
                "|TIME_LIMIT_MS|250|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2|SSE_PEAK_DEG|%.6g|ASSESSMENT|FINAL_SETTLE"
                "|FF_ON|%u|FF_TYPE|ACCEL|FKA|%.6g|FMAX|%.6g|SHAPE_MS|%lu|SHAPE_V|%.6g\r\n",
                (double)step->vkp, (double)step->vki, (double)step->vkd,
                (double)step->vmax_dps, (unsigned)YAW_CASCADE_VELOCITY_FILTER_MS, (unsigned)step->output_limits,
                (long)(int32_t)step->response_ms, (long)(int32_t)step->settle_ms, (unsigned)step->pass,
                (double)step->ss_max_error,
                (unsigned)step->ff_enabled, (double)step->ff_acceleration_gain,
                (double)step->ff_limit_nm, (unsigned long)step->shape_ms,
                (double)step->shape_velocity_gain);
        } else { length = snprintf(response + used, capacity - used, "\r\n"); }
        return (uint8_t)(length > 0 && (size_t)length < capacity - used);
    }
}

uint8_t YawConsole_Poll(char *response, size_t capacity)
/* 功能：推进Yaw准备操作和异步事件，按需生成一条回执或真实结果。 */
{
    YawStep snapshot, report;
    YawRemoteStepEvent event;
    if (!response || !capacity) { return 0U; }
    if (capacity >= 128U) {
        const char *result;
        taskENTER_CRITICAL(); result = PreparationTick(); taskEXIT_CRITICAL();
        if (result) {
            if (strcmp(result, "OK") == 0) {
                (void)snprintf(response, capacity, "Y|OK|READY|ZERO_SET|1|REQUESTED|1|ACTIVE|0|NEXT|NEW_CH5_OFF_THEN_MANUAL_OR_TEST\r\n");
            } else { (void)snprintf(response, capacity, "Y|ERROR|READY|%s\r\n", result); }
            return 1U;
        }
    }
    if (capacity >= 128U && YawBackend_TakeRemoteStepEvent(&event)) {
        static const char *results[] = { "NONE", "STARTED", "BACK_30", "START_GUARD",
#if APP_PITCH_ENABLED
            "OTHER_AXIS"
#else
            "PITCH_NOT_IMPLEMENTED"
#endif
        };
        (void)snprintf(response, capacity, "Y|REMOTE_STEP|SEQ|%lu|RESULT|%s|TARGET|%.1f|WHY|%s|BLOCKS|%lu\r\n",
            (unsigned long)event.sequence, event.result <= 4U ? results[event.result] : "UNKNOWN",
            (double)event.target_deg, StepBlockReason(event.blocks), (unsigned long)event.blocks);
        return 1U;
    }
    YawBackend_GetStep(&snapshot);
    if (!snapshot.pending || !yaw_console_StepReport(&snapshot, response, capacity)) { return 0U; }
    if (!YawBackend_TakeStepReport(&report)) { return 0U; }
    if (report.id != snapshot.id) { return yaw_console_StepReport(&report, response, capacity); }
    return 1U;
}

static void ClearReport(const char *prefix, const char *remote_override, char *response, size_t capacity)
/* 功能：格式化 Yaw 清错条件、停止确认和最近一次清错结果。 */
{
    RemoteInput_State remote;
    YawBackendState yaw;
    YawClearCheck check;
    const char *remote_reason;
    taskENTER_CRITICAL();
    remote_reason = SafeOffReason(&remote);
    if (remote_override) { remote_reason = remote_override; }
    YawBackend_GetState(&yaw); YawBackend_GetClearCheck(&check);
    taskEXIT_CRITICAL();
    (void)snprintf(response, capacity,
        "%s|REMOTE|%s|BLOCKS|%lu|LAST_BLOCKS|%lu|SAMPLES|%u|HOLD_MS|%lu|VEL_DPS|%.2f|D_TEMP|%d|M_TEMP|%d|MONITOR|%u|FD_ATTEMPTED|%u|FD_AGE_MS|%lu|FD_RESULT|%u|FD_CANCEL|%u|MAILBOX|%lu|CAN_EPOCH|%lu|CLEAR_RESULT|%u|OFF_MOTION|%lu|ACTUATION_STARTED|%u|REQUIRED_HOLD_MS|%lu|CH|%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        prefix, remote_reason, (unsigned long)check.blocks, (unsigned long)check.last_blocks,
        (unsigned)check.samples, (unsigned long)check.hold_ms, (double)yaw.controller.velocity_dps,
        (int)yaw.feedback.driver_temp_c, (int)yaw.feedback.motor_temp_c, (unsigned)check.monitoring,
        (unsigned)check.fd_sent, (unsigned long)check.fd_age_ms, (unsigned)check.fd_result,
        (unsigned)check.fd_cancel, (unsigned long)check.mailbox_pending, (unsigned long)check.can_epoch,
        (unsigned)check.clear_result, (unsigned long)check.off_motion_faults,
        (unsigned)check.actuation_started, (unsigned long)check.required_hold_ms,
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3], (unsigned)remote.channels[4],
        (unsigned)remote.channels[5], (unsigned)remote.channels[6], (unsigned)remote.channels[7]);
}

static void BusReport(char *response, size_t capacity)
/* 功能：发布双 CAN 收发计数、邮箱占用及首次错误证据，不改变输出许可。 */
{
    CanBusDiagnostics bus;
    size_t used;
    uint32_t i;
    int count;
    CanBus_GetDiagnostics(&bus);
    count = snprintf(response, capacity, "Y|BUS|%s|EPOCH|%lu|DROP|%lu",
        APP_YAW_VERSION, (unsigned long)bus.state.fault_epoch, (unsigned long)bus.state.dropped);
    if (count < 0 || (size_t)count >= capacity) { return; }
    used = (size_t)count;
    for (i = 0U; i < 2U; ++i) {
        count = snprintf(response + used, capacity - used,
            "|C%u_RX|%lu|TX|%lu|ERR|%lu|LAST|0x%08lX|HAL|0x%08lX|FIRST|0x%08lX|SOURCE|%u|FIRST_MS|%lu|PENDING|%lu%s",
            (unsigned)(i + 1U), (unsigned long)bus.state.rx_frames[i],
            (unsigned long)bus.state.tx_frames[i], (unsigned long)bus.state.errors[i],
            (unsigned long)bus.state.last_error[i], (unsigned long)bus.hal_error[i],
            (unsigned long)bus.state.first_error[i], (unsigned)bus.state.first_error_source[i],
            (unsigned long)bus.state.first_error_ms[i], (unsigned long)bus.pending[i],
            i == 1U ? "\r\n" : "");
        if (count < 0 || (size_t)count >= capacity - used) { return; }
        used += (size_t)count;
    }
}

static void yaw_console_BriefStatus(char *response, size_t capacity)
/* 功能：格式化Yaw精简状态，供高频查询读取且不改变运行状态。 */
{
    YawBackendState state;
    YawBackend_GetState(&state);
    (void)snprintf(response, capacity,
        "Y|STATE|%s|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|ZERO_SET|%u|FAULT|%lu|AGE_MS|%lu|ANGLE|%.3f|TARGET|%.3f|VEL_DPS|%.2f|TORQUE_NM|%.5f\r\n",
        APP_YAW_VERSION, (unsigned)state.online, (unsigned)state.active, (unsigned)state.requested,
        (unsigned)state.zero_set, (unsigned long)state.controller.faults, (unsigned long)state.feedback_age_ms,
        (double)state.controller.angle_deg, (double)state.controller.target_deg,
        (double)state.controller.cascade.velocity_dps, (double)state.controller.torque_nm);
}

uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity)
/* 功能：解析Yaw命令，核对格式和操作条件后查询状态或调用对应后端。 */
{
    YawBackendState yaw;
    uint8_t accepted = 0U;
    char parameter = 0;
    const char *number = 0;
    float value = 0.0f;
    if (!line || !response || !capacity ||
        (strcmp(line, "YAW") != 0 && strncmp(line, "YAW ", 4U) != 0)) { return 0U; }
    if (strcmp(line, "YAW") == 0) { yaw_console_BriefStatus(response, capacity); return 1U; }
    if (strcmp(line, "YAW HELP") == 0 || strcmp(line, "YAW ?") == 0) {
        (void)snprintf(response, capacity,
            "Y|HELP|CTRL|FIXED_CASCADE_PID|READ|YAW;PID;RESULT;STATUS;STEP CHECK|PREPARE|READY|STOP|OFF|VIEW|YAW VIEW|TUNE|POS kp ki kd;VEL kp ki kd;PKP/PKI/PKD/VKP/VKI/VKD value|TELEM|TELEM OFF/ON|TEST|STEP [+/-30_RELATIVE];HOME;REMOTE_CH5_CH7|DETAIL|INFO;CHECK;BUS;ZERO;CLEAR\r\n");
        return 1U;
    }
    if (strcmp(line, "YAW BUS") == 0) {
        BusReport(response, capacity); return 1U;
    }
    if (strcmp(line, "YAW RESULT") == 0) {
        YawStep snapshot; YawBackend_GetStep(&snapshot);
        (void)yaw_console_StepReport(&snapshot, response, capacity); return 1U;
    }
    if (ready_phase && strcmp(line, "YAW READY") != 0 && strcmp(line, "YAW OFF") != 0 &&
        strcmp(line, "YAW CHECK") != 0 && strcmp(line, "YAW STATUS") != 0 && strcmp(line, "YAW INFO") != 0 &&
        strcmp(line, "YAW PID") != 0) {
        (void)snprintf(response, capacity, "Y|ERROR|READY_BUSY_USE_YAW_OFF\r\n"); return 1U;
    }
    if (strcmp(line, "YAW HOME") == 0) {
        taskENTER_CRITICAL();
        if (SafeStep()) { accepted = YawBackend_SetParameter('T', 0.0f); }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "Y|OK|HOME|TARGET|0\r\n" :
            "Y|ERROR|HOME_REQUIRES_ARMED_ACTIVE_NEUTRAL\r\n"); return 1U;
    }
    if (strcmp(line, "YAW STEP CHECK") == 0) {
        uint8_t remote_ok = SafeStep();
        YawBackend_GetState(&yaw);
        (void)snprintf(response, capacity,
            "Y|STEP_CHECK|READY|%u|WHY|%s|BLOCKS|%lu|ANGLE|%.3f|TARGET|%.3f|ENC_DPS|%.2f|RAW_DPS|%.2f|STILL_MS|%lu|WINDOW_LEFT_MS|%lu|CONSOLE_READY|%u|PLUS30_BLOCKS|%lu|MINUS30_BLOCKS|%lu|COUNT_LIMIT|0\r\n",
            (unsigned)(!yaw.step_blocks), StepBlockReason(yaw.step_blocks),
            (unsigned long)yaw.step_blocks, (double)yaw.controller.angle_deg, (double)yaw.controller.target_deg,
            (double)yaw.controller.cascade.velocity_dps, (double)yaw.controller.velocity_dps,
            (unsigned long)yaw.step_still_ms, (unsigned long)yaw.step_window_left_ms, (unsigned)(!yaw.step_blocks && remote_ok),
            (unsigned long)YawBackend_StepBlocks(30.0f), (unsigned long)YawBackend_StepBlocks(-30.0f));
        return 1U;
    }
    if (strcmp(line, "YAW STEP") == 0 || strncmp(line, "YAW STEP ", 9U) == 0) {
        uint8_t target_ok, remote_ok;
        value = 30.0f;
        taskENTER_CRITICAL();
        target_ok = (uint8_t)((strcmp(line, "YAW STEP") == 0 || ConsoleParse_Number(line + 9U, &value)) &&
            (value == 30.0f || value == -30.0f));
        remote_ok = SafeStep();
        if (target_ok && remote_ok) {
            accepted = YawBackend_StartStep(value);
        }
        taskEXIT_CRITICAL();
        if (accepted) {
            YawBackend_GetState(&yaw);
            (void)snprintf(response, capacity, "Y|OK|STEP_STARTED|TARGET|%.1f|DELTA|%.1f|MODE|RELATIVE|WINDOW_MS|5000|HOLD|1\r\n",
                (double)yaw.controller.target_deg, (double)value);
        } else {
            YawBackend_GetState(&yaw);
            yaw.step_blocks = YawBackend_StepBlocks(value);
            (void)snprintf(response, capacity, "Y|ERROR|STEP_REQUIRES_ARMED_SETTLED_HEALTHY|WHY|%s|BLOCKS|%lu\r\n",
                !target_ok ? "TARGET_OR_FORMAT" : (remote_ok ? StepBlockReason(yaw.step_blocks) : "REMOTE_OR_OUTPUT_GUARD"),
                (unsigned long)yaw.step_blocks);
        }
        return 1U;
    }
    if (strcmp(line, "YAW READY") == 0) {
        const char *reason;
        CanBusDiagnostics bus;
        if (ready_phase) { (void)snprintf(response, capacity, "Y|BUSY|READY\r\n"); return 1U; }
        taskENTER_CRITICAL();
        reason = SafeOffReason(0); CanBus_GetDiagnostics(&bus);
        if (strcmp(reason, "OK") == 0) {
            if (!bus.state.started || bus.state.fault_epoch || bus.hal_error[0] || bus.hal_error[1]) {
                reason = "CAN_FAULT_USE_YAW_BUS";
            } else if (YawBackend_PrepareStop()) {
                AppTasks_RequestStop(); ready_phase = 1U; ready_clear_sent = 0U;
                ready_started_ms = HAL_GetTick(); accepted = 1U;
            } else { reason = "PREPARE_REJECTED"; }
        }
        taskEXIT_CRITICAL();
        if (accepted) { (void)snprintf(response, capacity, "Y|BUSY|READY\r\n"); }
        else { (void)snprintf(response, capacity, "Y|ERROR|READY|%s\r\n", reason); }
        return 1U;
    }
    if (strcmp(line, "YAW CHECK") == 0) {
        ClearReport("Y|CHECK", 0, response, capacity); return 1U;
    }
    if (strcmp(line, "YAW STATUS") == 0 || strcmp(line, "YAW INFO") == 0) {
        YawBackend_GetState(&yaw);
        (void)snprintf(response, capacity,
            "Y|STATUS|%s|CTRL|CASCADE|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|STATUS|%u|FAULT|%lu|AGE_MS|%lu|STOP_PENDING|%u|ANGLE|%.3f|TARGET|%.3f|VREF_DPS|%.2f|VEL_DPS|%.2f|PKP|%.4f|PKI|%.4f|PKD|%.4f|VKP|%.6f|VKI|%.6f|VKD|%.6f|VREF_MAX_DPS|%.1f|TORQUE_SAT|%u|VEL_SAT|%u|%s|%.3f|OUTPUT_LIMITS|%u|TARGET_LIMIT_DEG|%.1f|HARD_LIMIT_DEG|%.1f|SPEED_GUARD|%u|ZERO_SET|%u|TELEM_MS|%u|BAUD|%lu|RX|%lu\r\n",
            APP_YAW_VERSION, (unsigned)yaw.online, (unsigned)yaw.active, (unsigned)yaw.requested,
            (unsigned)yaw.feedback.status, (unsigned long)yaw.controller.faults,
            (unsigned long)yaw.feedback_age_ms, (unsigned)yaw.stop_pending,
            (double)yaw.controller.angle_deg, (double)yaw.controller.target_deg,
            (double)yaw.controller.cascade.target_velocity_dps, (double)yaw.controller.cascade.velocity_dps,
            (double)yaw.controller.cascade.position.kp, (double)yaw.controller.cascade.position.ki,
            (double)yaw.controller.cascade.position.kd, (double)yaw.controller.cascade.speed.kp,
            (double)yaw.controller.cascade.speed.ki, (double)yaw.controller.cascade.speed.kd,
            (double)yaw.controller.cascade.velocity_limit_dps,
            (unsigned)yaw.controller.cascade.torque_limited, (unsigned)yaw.controller.cascade.velocity_limited,
            0U ? "LIMIT_NM" : "PROTOCOL_NM",
            (double)YAW_CONTROLLER_TORQUE_LIMIT_NM, (unsigned)0U,
            (double)YAW_CONTROLLER_TARGET_LIMIT_DEG,
            (double)YAW_CONTROLLER_HARD_LIMIT_DEG, (unsigned)YAW_CONTROLLER_SPEED_GUARD_ENABLED,
            (unsigned)yaw.zero_set, (unsigned)APP_TELEMETRY_PERIOD_MS,
            (unsigned long)APP_CONSOLE_BAUDRATE, (unsigned long)yaw.rx_count);
        return 1U;
    }
    if (strcmp(line, "YAW OFF") == 0) {
        taskENTER_CRITICAL();
        ready_phase = 0U;
        (void)YawBackend_Request(0U); AppTasks_RequestStop();
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, "Y|OK|OFF\r\n"); return 1U;
    }
    if (strcmp(line, "YAW ON") == 0) {
        taskENTER_CRITICAL();
        if (SafeOff()) {
            /* 新STOP屏障取消已有目标；ON仅准备，旧CH5 ON不得使能。 */
            AppTasks_RequestStop(); accepted = YawBackend_Request(1U);
        }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "Y|OK|PREPARE\r\n" : "Y|ERROR|SAFE_OFF_REQUIRED\r\n");
        return 1U;
    }
    if (strcmp(line, "YAW PID") == 0) {
        YawBackend_GetState(&yaw);
        (void)snprintf(response, capacity,
            "Y|PID|POS|%.4f,%.4f,%.4f|VEL|%.6f,%.6f,%.6f|VMAX_DPS|%.1f|OUTPUT_LIMITS|%u\r\n",
            (double)yaw.controller.cascade.position.kp, (double)yaw.controller.cascade.position.ki,
            (double)yaw.controller.cascade.position.kd, (double)yaw.controller.cascade.speed.kp,
            (double)yaw.controller.cascade.speed.ki, (double)yaw.controller.cascade.speed.kd,
            (double)yaw.controller.cascade.velocity_limit_dps, (unsigned)0U);
        return 1U;
    }
    if (strncmp(line, "YAW POS ", 8U) == 0 || strncmp(line, "YAW VEL ", 8U) == 0) {
        float kp, ki, kd;
        char which = line[4] == 'P' ? 'P' : 'V';
        if (ConsoleParse_PidNumbers(line + 8U, &kp, &ki, &kd)) {
            accepted = YawBackend_SetLoopPid(which, kp, ki, kd);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|%s\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n",
            which == 'P' ? "POS" : "VEL"); return 1U;
    }
    if (strncmp(line, "YAW PID ", 8U) == 0 || strncmp(line, "YAW KP ", 7U) == 0 ||
        strncmp(line, "YAW KI ", 7U) == 0 || strncmp(line, "YAW KD ", 7U) == 0) {
        (void)snprintf(response, capacity, "Y|ERROR|USE_YAW_POS_OR_VEL\r\n"); return 1U;
    }
    if (strlen(line) >= 8U && (line[4] == 'P' || line[4] == 'V') && line[5] == 'K' &&
        (line[6] == 'P' || line[6] == 'I' || line[6] == 'D') && line[7] == ' ') {
        if (ConsoleParse_Number(line + 8U, &value)) {
            accepted = YawBackend_SetLoopParameter(line[4], line[6], value);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|PARAMETER\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n"); return 1U;
    }
    if (strcmp(line, "YAW CLEAR") == 0) {
        const char *remote_reason;
        taskENTER_CRITICAL();
        remote_reason = SafeOffReason(0);
        if (strcmp(remote_reason, "OK") == 0) { accepted = YawBackend_ClearCommsFault(); }
        taskEXIT_CRITICAL();
        if (accepted) { (void)snprintf(response, capacity, "Y|OK|CLEAR_SENT\r\n"); }
        else { ClearReport("Y|ERROR|CLEAR", remote_reason, response, capacity); }
        return 1U;
    }
    if (strcmp(line, "YAW ZERO") == 0) {
        taskENTER_CRITICAL();
        if (SafeOff()) { accepted = YawBackend_CaptureReference(); }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "Y|OK|SOFTWARE_ZERO\r\n" : "Y|ERROR|STOP_AND_STATIONARY_REQUIRED\r\n");
        return 1U;
    }
    if (strncmp(line, "YAW KP ", 7U) == 0) { parameter = 'P'; number = line + 7U; }
    else if (strncmp(line, "YAW KI ", 7U) == 0) { parameter = 'I'; number = line + 7U; }
    else if (strncmp(line, "YAW KD ", 7U) == 0) { parameter = 'D'; number = line + 7U; }
    else if (strncmp(line, "YAW TARGET ", 11U) == 0) { parameter = 'T'; number = line + 11U; }
    if (!number) {
        if (strcmp(line, "YAW KP") == 0 || strcmp(line, "YAW KI") == 0 ||
            strcmp(line, "YAW KD") == 0 || strcmp(line, "YAW TARGET") == 0) {
            (void)snprintf(response, capacity, "Y|ERROR|PARAMETER_OR_STATE\r\n");
        } else { (void)snprintf(response, capacity, "Y|ERROR|UNKNOWN_COMMAND|USE|YAW HELP\r\n"); }
        return 1U;
    }
    if (ConsoleParse_Number(number, &value)) {
        accepted = YawBackend_SetParameter(parameter, value);
    }
    (void)snprintf(response, capacity, accepted ? "Y|OK|PARAMETER\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n");
    return 1U;
}
