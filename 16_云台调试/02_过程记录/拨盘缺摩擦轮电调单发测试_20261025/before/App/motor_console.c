/**
 * @file motor_console.c
 * @brief 执行器串口命令
 *
 * 职责：实现 Yaw、Pitch、拨盘的命令解析、准备流程及状态报告。
 * 主要函数：
 *   - YawConsole_Handle / Poll：处理 YAW 命令、准备和阶跃报告。
 *   - PitchConsole_Handle / Poll：处理 PITCH 命令、坐标/重力查询及报告。
 *   - FeederConsole_Handle：处理 FEEDER 配置、单发/连发和状态查询。
 * 调用关系：app_console 分发命令 → 本文件校验参数 → 对应 Backend 执行。
 * 维护约束：三种设备的握手条件分别保留；查询不应隐式改变使能、零点或积分。
 * 参数入口：app_config.h；对外声明仍在原头文件中。
 */

/* APP_SECTION_BEGIN yaw_console.c */
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
    if (AppConsole_IsDiagnosticMode()) { return "DIAGNOSTIC"; }
    return "OK";
}

static uint8_t SafeOff(void) { return strcmp(SafeOffReason(0), "OK") == 0; }

void YawConsole_CancelPreparation(void)
{
    taskENTER_CRITICAL();
    if (ready_phase) { ready_phase = 0U; (void)YawBackend_Request(0U); }
    taskEXIT_CRITICAL();
}

static const char *PreparationTick(void)
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
        app.logic.safety.operator_armed && !AppConsole_IsDiagnosticMode());
}

static const char *StepBlockReason(uint32_t blocks)
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

#if YAW_CONTROLLER_CASCADE_ENABLED
static void FeedforwardReport(char *response, size_t capacity)
{
    YawBackendState yaw;
    YawBackend_GetState(&yaw);
    (void)snprintf(response, capacity,
        "Y|FF|ON|%u|TYPE|ACCEL|FKA|%.6g|FMAX|%.6g|SHAPE_MS|%lu|SHAPE_ACTIVE|%u|REF_DPS2|%.6g"
        "|FF_NM|%.6g|PID_NM|%.6g|SUM_NM|%.6g|OUT_NM|%.6g|VI_NM|%.6g|PI_DPS|%.6g"
        "|FF_SAT|%u|TOTAL_SAT|%u|REV|%lu\r\n",
        (unsigned)yaw.controller.ff.enabled, (double)yaw.controller.ff.acceleration_gain,
        (double)yaw.controller.ff.limit_nm, (unsigned long)yaw.controller.shape.duration_ms,
        (unsigned)yaw.controller.shape.active, (double)yaw.controller.shape.acceleration_dps2,
        (double)yaw.controller.cascade.feedforward_nm, (double)yaw.controller.pid_torque_nm,
        (double)yaw.controller.cascade.unsaturated_torque_nm, (double)yaw.controller.torque_nm,
        (double)yaw.controller.cascade.speed_i, (double)yaw.controller.cascade.position_i,
        (unsigned)yaw.controller.cascade.feedforward_limited, (unsigned)yaw.controller.cascade.torque_limited,
        (unsigned long)yaw.controller.ff.revision);
}
#endif

uint8_t YawConsole_Poll(char *response, size_t capacity)
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
{
    YawBackendState state;
    YawBackend_GetState(&state);
    (void)snprintf(response, capacity,
        "Y|STATE|%s|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|ZERO_SET|%u|FAULT|%lu|AGE_MS|%lu|ANGLE|%.3f|TARGET|%.3f|VEL_DPS|%.2f|TORQUE_NM|%.5f\r\n",
        APP_YAW_VERSION, (unsigned)state.online, (unsigned)state.active, (unsigned)state.requested,
        (unsigned)state.zero_set, (unsigned long)state.controller.faults, (unsigned long)state.feedback_age_ms,
        (double)state.controller.angle_deg, (double)state.controller.target_deg,
        (double)(YAW_CONTROLLER_CASCADE_ENABLED ? state.controller.cascade.velocity_dps : state.controller.velocity_dps), (double)state.controller.torque_nm);
}

uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity)
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
#if APP_YAW_EXPERIMENTAL_TRACKING
            "Y|HELP|READ|YAW;PID;RESULT;STATUS|PREPARE|READY|STOP|OFF|VIEW|YAW VIEW|TUNE|POS kp ki kd;VEL kp ki kd;PKP/PKI/PKD/VKP/VKI/VKD value|FF|FF;FF ON/OFF;FKA [0_to_0.001];FMAX [0_to_1]|SHAPE|SHAPE [0_or_80_to_500_ms];SHAPEV [0_to_1]|TELEM|TELEM OFF/ON|TEST|STEP [+/-30_RELATIVE];HOME;REMOTE_CH5_CH7|DETAIL|INFO;CHECK;BUS;ZERO;CLEAR\r\n");
#else
            "Y|HELP|CTRL|FIXED_CASCADE_PID|READ|YAW;PID;RESULT;STATUS;STEP CHECK|PREPARE|READY|STOP|OFF|VIEW|YAW VIEW|TUNE|POS kp ki kd;VEL kp ki kd;PKP/PKI/PKD/VKP/VKI/VKD value|TELEM|TELEM OFF/ON|TEST|STEP [+/-30_RELATIVE];HOME;REMOTE_CH5_CH7|DETAIL|INFO;CHECK;BUS;ZERO;CLEAR\r\n");
#endif
        return 1U;
    }
    if (strcmp(line, "YAW BUS") == 0) {
        BusReport(response, capacity); return 1U;
    }
    if (strcmp(line, "YAW RESULT") == 0) {
        YawStep snapshot; YawBackend_GetStep(&snapshot);
        (void)yaw_console_StepReport(&snapshot, response, capacity); return 1U;
    }
#if YAW_CONTROLLER_CASCADE_ENABLED
    if (strcmp(line, "YAW FF") == 0) { FeedforwardReport(response, capacity); return 1U; }
#endif
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
#if YAW_CONTROLLER_CASCADE_ENABLED
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
            YAW_CASCADE_OUTPUT_LIMITS_ENABLED ? "LIMIT_NM" : "PROTOCOL_NM",
            (double)YAW_CONTROLLER_TORQUE_LIMIT_NM, (unsigned)YAW_CASCADE_OUTPUT_LIMITS_ENABLED,
            (double)YAW_CONTROLLER_TARGET_LIMIT_DEG,
            (double)YAW_CONTROLLER_HARD_LIMIT_DEG, (unsigned)YAW_CONTROLLER_SPEED_GUARD_ENABLED,
            (unsigned)yaw.zero_set, (unsigned)APP_TELEMETRY_PERIOD_MS,
            (unsigned long)APP_CONSOLE_BAUDRATE, (unsigned long)yaw.rx_count);
#else
        (void)snprintf(response, capacity,
            "Y|STATUS|%s|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|STATUS|%u|FAULT|%lu|AGE_MS|%lu|STOP_PENDING|%u|ANGLE|%.3f|TARGET|%.3f|KP|%.4f|KI|%.4f|KD|%.4f|LIMIT_NM|%.3f|TARGET_LIMIT_DEG|%.1f|HARD_LIMIT_DEG|%.1f|SOFT_DPS|%.1f|HARD_DPS|%.1f|SPEED_GUARD|%u|LIMITED|%u|ZERO_SET|%u|TELEM_MS|%u|RX|%lu\r\n",
            APP_YAW_VERSION, (unsigned)yaw.online, (unsigned)yaw.active,
            (unsigned)yaw.requested, (unsigned)yaw.feedback.status,
            (unsigned long)yaw.controller.faults, (unsigned long)yaw.feedback_age_ms,
            (unsigned)yaw.stop_pending, (double)yaw.controller.angle_deg,
            (double)yaw.controller.target_deg, (double)yaw.controller.pid.kp,
            (double)yaw.controller.pid.ki, (double)yaw.controller.pid.kd,
            (double)YAW_CONTROLLER_TORQUE_LIMIT_NM,
            (double)YAW_CONTROLLER_TARGET_LIMIT_DEG, (double)YAW_CONTROLLER_HARD_LIMIT_DEG,
            (double)(YAW_CONTROLLER_SPEED_GUARD_ENABLED ? YAW_CONTROLLER_SOFT_SPEED_DPS : 0.0f),
            (double)(YAW_CONTROLLER_SPEED_GUARD_ENABLED ? YAW_CONTROLLER_SPEED_LIMIT_DPS : 0.0f),
            (unsigned)YAW_CONTROLLER_SPEED_GUARD_ENABLED,
            (unsigned)yaw.controller.speed_limited, (unsigned)yaw.zero_set, (unsigned)APP_TELEMETRY_PERIOD_MS,
            (unsigned long)yaw.rx_count);
#endif
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
#if YAW_CONTROLLER_CASCADE_ENABLED
    if (strcmp(line, "YAW SHAPE") == 0) {
        YawBackend_GetState(&yaw);
        (void)snprintf(response, capacity,
            "Y|SHAPE|ENABLED|%u|MS|%lu|ACTIVE|%u|REF_DEG|%.6g|REF_DPS|%.6g|REF_DPS2|%.6g|GOAL_DEG|%.6g|VEL_FF|%.6g|PI_FREEZE|%u\r\n",
            (unsigned)(yaw.controller.shape.duration_ms != 0U), (unsigned long)yaw.controller.shape.duration_ms,
            (unsigned)yaw.controller.shape.active, (double)yaw.controller.shape.position_deg,
            (double)yaw.controller.shape.velocity_dps, (double)yaw.controller.shape.acceleration_dps2,
            (double)yaw.controller.target_deg, (double)yaw.controller.shape.velocity_gain, (unsigned)yaw.controller.shape.active);
        return 1U;
    }
    if (strcmp(line, "YAW SHAPEV") == 0) {
        YawBackend_GetState(&yaw);
        (void)snprintf(response, capacity, "Y|SHAPEV|%.6g\r\n", (double)yaw.controller.shape.velocity_gain);
        return 1U;
    }
    if (strncmp(line, "YAW SHAPEV ", 11U) == 0) {
        if (!APP_YAW_EXPERIMENTAL_TRACKING && ConsoleParse_Number(line + 11U, &value) && value != 0.0f) {
            (void)snprintf(response, capacity, "Y|ERROR|REMOVED|SHAPEV|FIXED_PID\r\n"); return 1U;
        }
        if (ConsoleParse_Number(line + 11U, &value) && !AppConsole_IsDiagnosticMode()) {
            accepted = YawBackend_SetShapeVelocityGain(value);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|SHAPEV|%.6g\r\n" :
            "Y|ERROR|SHAPEV|USE_0_TO_1|FINISH_STEP_FIRST\r\n", (double)value);
        return 1U;
    }
    if (strncmp(line, "YAW SHAPE ", 10U) == 0) {
        if (!APP_YAW_EXPERIMENTAL_TRACKING && ConsoleParse_Number(line + 10U, &value) && value != 0.0f) {
            (void)snprintf(response, capacity, "Y|ERROR|REMOVED|SHAPE|FIXED_PID\r\n"); return 1U;
        }
        if (ConsoleParse_Number(line + 10U, &value) && value >= 0.0f && value <= 500.0f &&
            value == (float)(uint32_t)value && !AppConsole_IsDiagnosticMode()) {
            accepted = YawBackend_SetShape((uint32_t)value);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|SHAPE|MS|%.0f\r\n" :
            "Y|ERROR|SHAPE|USE_0_OR_80_TO_500_MS|FINISH_STEP_FIRST\r\n", (double)value);
        return 1U;
    }
    if (strcmp(line, "YAW FF ON") == 0 || strcmp(line, "YAW FF OFF") == 0 ||
        strcmp(line, "YAW FF 1") == 0 || strcmp(line, "YAW FF 0") == 0) {
        value = (strcmp(line, "YAW FF ON") == 0 || strcmp(line, "YAW FF 1") == 0) ? 1.0f : 0.0f;
        if (!APP_YAW_EXPERIMENTAL_TRACKING && value != 0.0f) {
            (void)snprintf(response, capacity, "Y|ERROR|REMOVED|FF|FIXED_PID\r\n"); return 1U;
        }
        if (value == 0.0f || !AppConsole_IsDiagnosticMode()) { accepted = YawBackend_SetFeedforward('E', value); }
        (void)snprintf(response, capacity, accepted ? "Y|OK|FF|ON|%u\r\n" :
            "Y|ERROR|FF_REQUIRES_PREPARED_HEALTHY_OR_VALID_VALUE\r\n", (unsigned)value); return 1U;
    }
    {
        static const char *removed[] = { "FBIAS", "FANG", "FKV", "FC", "FVS" };
        unsigned i;
        for (i = 0U; i < sizeof(removed) / sizeof(removed[0]); ++i) {
            size_t length = strlen(removed[i]);
            if (strncmp(line + 4U, removed[i], length) == 0 &&
                (line[4U + length] == ' ' || line[4U + length] == '\0')) {
                (void)snprintf(response, capacity,
#if APP_YAW_EXPERIMENTAL_TRACKING
                    "Y|ERROR|REMOVED|%s|USE|FKA|FMAX\r\n",
#else
                    "Y|ERROR|REMOVED|%s|FIXED_PID\r\n",
#endif
                    removed[i]);
                return 1U;
            }
        }
    }
    {
        static const char *names[] = { "FKA", "FMAX" };
        static const char parameters[] = { 'A', 'L' };
        unsigned i;
        for (i = 0U; i < sizeof(parameters); ++i) {
            size_t length = strlen(names[i]);
            if (strcmp(line + 4U, names[i]) == 0) {
                YawBackend_GetState(&yaw);
                (void)snprintf(response, capacity, "Y|%s|%.6g\r\n", names[i],
                    (double)(parameters[i] == 'A' ? yaw.controller.ff.acceleration_gain : yaw.controller.ff.limit_nm));
                return 1U;
            }
            if (strncmp(line + 4U, names[i], length) == 0 && line[4U + length] == ' ') {
#if !APP_YAW_EXPERIMENTAL_TRACKING
                (void)snprintf(response, capacity, "Y|ERROR|REMOVED|%s|FIXED_PID\r\n", names[i]); return 1U;
#else
                if (ConsoleParse_Number(line + 5U + length, &value) && !AppConsole_IsDiagnosticMode()) {
                    accepted = YawBackend_SetFeedforward(parameters[i], value);
                }
                (void)snprintf(response, capacity, accepted ? "Y|OK|%s\r\n" :
                    "Y|ERROR|FF_PARAMETER_OR_STATE|%s\r\n", names[i]); return 1U;
#endif
            }
        }
    }
    if (strcmp(line, "YAW PID") == 0) {
        YawBackend_GetState(&yaw);
        (void)snprintf(response, capacity,
            "Y|PID|POS|%.4f,%.4f,%.4f|VEL|%.6f,%.6f,%.6f|VMAX_DPS|%.1f|OUTPUT_LIMITS|%u\r\n",
            (double)yaw.controller.cascade.position.kp, (double)yaw.controller.cascade.position.ki,
            (double)yaw.controller.cascade.position.kd, (double)yaw.controller.cascade.speed.kp,
            (double)yaw.controller.cascade.speed.ki, (double)yaw.controller.cascade.speed.kd,
            (double)yaw.controller.cascade.velocity_limit_dps, (unsigned)YAW_CASCADE_OUTPUT_LIMITS_ENABLED);
        return 1U;
    }
    if (strncmp(line, "YAW POS ", 8U) == 0 || strncmp(line, "YAW VEL ", 8U) == 0) {
        float kp, ki, kd;
        char which = line[4] == 'P' ? 'P' : 'V';
        if (ConsoleParse_PidNumbers(line + 8U, &kp, &ki, &kd) && !AppConsole_IsDiagnosticMode()) {
            accepted = YawBackend_SetLoopPid(which, kp, ki, kd);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|%s\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n",
            which == 'P' ? "POS" : "VEL"); return 1U;
    }
    if (strncmp(line, "YAW PID ", 8U) == 0 || strncmp(line, "YAW KP ", 7U) == 0 ||
        strncmp(line, "YAW KI ", 7U) == 0 || strncmp(line, "YAW KD ", 7U) == 0) {
        (void)snprintf(response, capacity, "Y|ERROR|USE_YAW_POS_OR_VEL\r\n"); return 1U;
    }
    if (strncmp(line, "YAW VMAX ", 9U) == 0) {
#if YAW_CASCADE_OUTPUT_LIMITS_ENABLED
        if (ConsoleParse_Number(line + 9U, &value) && !AppConsole_IsDiagnosticMode()) { accepted = YawBackend_SetVelocityLimit(value); }
        (void)snprintf(response, capacity, accepted ? "Y|OK|VMAX\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n"); return 1U;
#else
        (void)snprintf(response, capacity, "Y|ERROR|OUTPUT_LIMITS_DISABLED\r\n"); return 1U;
#endif
    }
    if (strlen(line) >= 8U && (line[4] == 'P' || line[4] == 'V') && line[5] == 'K' &&
        (line[6] == 'P' || line[6] == 'I' || line[6] == 'D') && line[7] == ' ') {
        if (ConsoleParse_Number(line + 8U, &value) && !AppConsole_IsDiagnosticMode()) {
            accepted = YawBackend_SetLoopParameter(line[4], line[6], value);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|PARAMETER\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n"); return 1U;
    }
#else
    if (strncmp(line, "YAW PID ", 8U) == 0) {
        float kp, ki, kd;
        if (ConsoleParse_PidNumbers(line + 8U, &kp, &ki, &kd) && !AppConsole_IsDiagnosticMode()) {
            accepted = YawBackend_SetPid(kp, ki, kd);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|PID\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n");
        return 1U;
    }
#endif
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
    if (ConsoleParse_Number(number, &value) && !AppConsole_IsDiagnosticMode()) {
        accepted = YawBackend_SetParameter(parameter, value);
    }
    (void)snprintf(response, capacity, accepted ? "Y|OK|PARAMETER\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n");
    return 1U;
}
/* APP_SECTION_END yaw_console.c */

/* APP_SECTION_BEGIN pitch_console.c */
/* PITCH 命令：坐标/重力、独立准备流程和实测报告。 */
#include "pitch_console.h"
#include "pitch_backend.h"
#include "app_config.h"
#include "app_tasks.h"
#include "app_console.h"
#include "console_parse.h"
#include "remote_input.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/* Low-priority formatting only. CHECK sends CC; READY never sends FC or FD. */
static uint8_t pending_operation; /* 1=CHECK, 2=READY */
static uint32_t operation_ms, query_ms, query_rx;
static uint32_t reported_guard_id, reported_stop_id;

void PitchConsole_Init(void)
{
    pending_operation = 0U; operation_ms = query_ms = query_rx = 0U;
    reported_guard_id = reported_stop_id = 0U;
}
void PitchConsole_CancelPreparation(void)
{ taskENTER_CRITICAL(); pending_operation = 0U; taskEXIT_CRITICAL(); }

static const char *RemoteReason(uint8_t off)
{
    RemoteInput_State remote;
    RemoteFrame frame;
    RemoteCommand command;
    AppState app;
    RemoteInput_GetState(&remote); AppTasks_GetState(&app);
    memset(&frame, 0, sizeof(frame)); memcpy(frame.channels, remote.channels, sizeof(frame.channels));
    frame.has_frame = remote.has_frame; frame.received_ms = remote.received_ms;
    RemoteMapper_Map(&frame, &command);
    if (!remote.has_frame || HAL_GetTick() - remote.received_ms >= REMOTE_TIMEOUT_MS) { return "STALE"; }
    if (!command.valid) { return "INVALID"; }
    if (!command.failsafe_ok) { return "CH8_LOCK"; }
    if (AppConsole_IsDiagnosticMode()) { return "DIAGNOSTIC"; }
    if (off) {
        if (command.arm != RC_LOW) { return "CH5_NOT_OFF"; }
        if (!command.neutral) { return "NOT_NEUTRAL_OR_CH6_ON"; }
        if (app.logic.safety.operator_armed) { return "ARMED"; }
    } else {
        if (command.arm != RC_HIGH || !app.logic.safety.operator_armed) { return "NOT_ARMED"; }
        if (!command.neutral) { return "NOT_NEUTRAL_OR_CH6_ON"; }
    }
    return "OK";
}

static const char *StopName(PitchStopReason reason)
{
    static const char *names[] = { "NONE", "EXTERNAL", "PERMISSION", "FEEDBACK",
        "CAN_PERMISSION", "ENABLE_TX", "START_LIMIT", "ENABLE_TIMEOUT", "MODE_EXIT",
        "MANUAL_OVERRIDE", "CONTROL_FAULT", "TORQUE_TX", "FEEDBACK_FAULT" };
    return (unsigned)reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "UNKNOWN";
}

static uint8_t GuardReport(char *response, size_t capacity, uint8_t only_new)
{
    static const char *sources[] = { "NONE", "SAFETY", "YAW", "PITCH", "WHEELS", "FEEDER" };
    static const char *reasons[] = { "NONE", "POWER_ON", "REMOTE", "CH8_LOCK", "UART",
        "CAN", "TIMING", "STOP", "NOT_NEUTRAL", "ARM_POSITION", "FEEDBACK", "ACTUATOR" };
    AppState app;
    PitchBackendState pitch;
    const AppStopTrace *trace;
    int length;
    AppTasks_GetState(&app); PitchBackend_GetState(&pitch); trace = &app.logic.stop_trace;
    if (only_new && trace->id == reported_guard_id && pitch.stop_id == reported_stop_id) { return 0U; }
    length = snprintf(response, capacity,
        "P|GUARD|%s|TRACE|%lu|SOURCE|%s|REASON|%s|FLAGS|0x%08lX|AT_MS|%lu|REMOTE_AGE_MS|%lu|UART_EPOCH|%lu|RC_UART_EPOCH|%lu|CONSOLE_EPOCH|%lu|CAN_EPOCH|%lu|STOP_SEQ|%lu|GAP_MS|%lu|MODE|%u,%u|CH|%u,%u,%u,%u,%u,%u,%u,%u|NOW_ARMED|%u|NOW_REASON|%u|P_STOP_ID|%lu|P_STOP|%s|P_STOP_MS|%lu|P_FAULT|%lu\r\n",
        APP_PITCH_VERSION, (unsigned long)trace->id,
        trace->source < sizeof(sources) / sizeof(sources[0]) ? sources[trace->source] : "UNKNOWN",
        trace->reason <= SAFETY_ACTUATOR ? reasons[trace->reason] : "UNKNOWN",
        (unsigned long)trace->triggers, (unsigned long)trace->at_ms, (unsigned long)trace->remote_age_ms,
        (unsigned long)trace->uart_epoch, (unsigned long)app.guard_remote_uart_epoch,
        (unsigned long)app.guard_console_uart_epoch, (unsigned long)trace->can_epoch,
        (unsigned long)trace->stop_sequence, (unsigned long)app.guard_control_gap_ms,
        (unsigned)trace->previous_mode, (unsigned)trace->mode,
        (unsigned)trace->channels[0], (unsigned)trace->channels[1], (unsigned)trace->channels[2],
        (unsigned)trace->channels[3], (unsigned)trace->channels[4], (unsigned)trace->channels[5],
        (unsigned)trace->channels[6], (unsigned)trace->channels[7],
        (unsigned)app.logic.safety.operator_armed, (unsigned)app.logic.safety.reason,
        (unsigned long)pitch.stop_id, StopName(pitch.stop_reason), (unsigned long)pitch.stopped_ms,
        (unsigned long)pitch.controller.faults);
    if (length <= 0 || (size_t)length >= capacity) { return 0U; }
    if (only_new) { reported_guard_id = trace->id; reported_stop_id = pitch.stop_id; }
    return 1U;
}

static uint8_t Summary(const char *kind, const char *query_result, char *response, size_t capacity)
{
    PitchBackendState pitch;
    CanBusState bus;
    int length;
    PitchBackend_GetState(&pitch); CanBus_GetState(&bus);
    length = snprintf(response, capacity,
        "P|%s|%s|QUERY|%s|CTRL|CASCADE|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|ENABLING|%u|STATUS|%u|FAULT|%lu|AGE_MS|%lu|RX|%lu|STOP_PENDING|%u|COORD_CONFIRMED|%u|FEEDBACK_DIVISOR|%u|RAW_RAD|%.6f|RAW_RAD_S|%.5f|ANGLE|%.3f|TARGET|%.3f|VREF_DPS|%.2f|VEL_DPS|%.2f|TORQUE_NM|%.5f|FB_NM|%.5f|PKP|%.6g|PKI|%.6g|PKD|%.6g|VKP|%.6g|VKI|%.6g|VKD|%.6g|OUTPUT_LIMITS|0|PROTOCOL_NM|10|MIN_DEG|%.0f|MAX_DEG|%.0f|HARD_MIN_DEG|%.0f|HARD_MAX_DEG|%.0f|ZERO_SET|%u|D_TEMP|%u|M_TEMP|%u|CAN_EPOCH|%lu|REMOTE|%s|TELEM_MS|%u|BAUD|%lu|READ_ONLY_MONITOR|%u|P_STOP_ID|%lu|P_STOP|%s|KG|%.6g|GOFF_DEG|%.6g|G_CAL|%u|G_REQUIRED|%u|MANUAL_DPS|%.0f|SUPPORT_REQUIRED|1\r\n",
        kind, APP_PITCH_VERSION, query_result, (unsigned)pitch.online, (unsigned)pitch.active,
        (unsigned)pitch.requested, (unsigned)pitch.enabling, (unsigned)pitch.feedback.status,
        (unsigned long)pitch.controller.faults, (unsigned long)pitch.feedback_age_ms,
        (unsigned long)pitch.rx_count, (unsigned)pitch.stop_pending,
        (unsigned)pitch.coord_confirmed, (unsigned)pitch.controller.feedback_divisor,
        (double)pitch.feedback.position_rad, (double)pitch.feedback.velocity_rad_s,
        (double)pitch.controller.angle_deg, (double)pitch.controller.target_deg,
        (double)pitch.controller.cascade.target_velocity_dps,
        (double)pitch.controller.cascade.velocity_dps, (double)pitch.controller.torque_nm,
        (double)pitch.feedback.torque_nm, (double)pitch.controller.cascade.position.kp,
        (double)pitch.controller.cascade.position.ki, (double)pitch.controller.cascade.position.kd,
        (double)pitch.controller.cascade.speed.kp, (double)pitch.controller.cascade.speed.ki,
        (double)pitch.controller.cascade.speed.kd,
        (double)PITCH_CONTROLLER_TARGET_MIN_DEG, (double)PITCH_CONTROLLER_TARGET_MAX_DEG,
        (double)PITCH_CONTROLLER_HARD_MIN_DEG, (double)PITCH_CONTROLLER_HARD_MAX_DEG, (unsigned)pitch.zero_set,
        (unsigned)pitch.feedback.temp_mos, (unsigned)pitch.feedback.temp_rotor,
        (unsigned long)bus.fault_epoch, RemoteReason(1U), (unsigned)APP_TELEMETRY_PERIOD_MS,
        (unsigned long)APP_CONSOLE_BAUDRATE, (unsigned)pitch.monitoring,
        (unsigned long)pitch.stop_id, StopName(pitch.stop_reason), (double)pitch.controller.gravity_kg_nm,
        (double)pitch.controller.gravity_offset_deg, (unsigned)pitch.controller.gravity_calibrated,
        (unsigned)(pitch.controller.gravity_kg_nm != 0.0f && !pitch.controller.gravity_calibrated),
        (double)APP_PITCH_RATE_LIMIT_DPS);
    return (uint8_t)(length > 0 && (size_t)length < capacity);
}

static uint8_t GravityReport(const char *kind, char *response, size_t capacity)
{
    PitchBackendState pitch;
    const PitchController *p;
    const YawCascade *c;
    float speed_error, proportional, derivative;
    int length;
    PitchBackend_GetState(&pitch); p = &pitch.controller; c = &p->cascade;
    speed_error = c->target_velocity_dps - c->velocity_dps;
    proportional = p->active ? c->speed.kp * speed_error : 0.0f;
    derivative = p->active ? -c->speed.kd * c->acceleration_dps2 : 0.0f;
    length = snprintf(response, capacity,
        "P|%s|KG|%.6g|GOFF_DEG|%.6g|G_CAL|%u|G_REQUIRED|%u|PHYSICAL_DEG|%.3f|GFF_NM|%.6g|VP_NM|%.6g|VI_NM|%.6g|VD_NM|%.6g|PID_NM|%.6g|SUM_NM|%.6g|OUT_NM|%.6g|SAT|%u|I_TRANSFER|%u|RAM_ONLY|1|SHAPE_ACTIVE|%u|REF_DEG|%.6g|REF_DPS|%.6g|TERMINAL_BRAKE_NM|%.6g\r\n",
        kind, (double)p->gravity_kg_nm, (double)p->gravity_offset_deg, (unsigned)p->gravity_calibrated,
        (unsigned)(p->gravity_kg_nm != 0.0f && !p->gravity_calibrated),
        (double)(p->angle_deg + p->gravity_offset_deg), (double)p->gravity_torque_nm,
        (double)proportional, (double)c->speed_i, (double)derivative, (double)p->pid_torque_nm,
        (double)c->unsaturated_torque_nm, (double)p->torque_nm, (unsigned)c->torque_limited,
        (unsigned)p->gravity_i_transferred, (unsigned)p->shape.active,
        (double)p->shape.position_deg, (double)p->shape.velocity_dps, (double)p->brake.torque_nm);
    return (uint8_t)(length > 0 && (size_t)length < capacity);
}

static uint8_t pitch_console_StepReport(const PitchStep *value, char *response, size_t capacity)
{
    static const char *phases[] = { "IDLE", "RUNNING", "DONE", "ABORTED" };
    static const char *reasons[] = { "NONE", "STOP", "FAULT", "PID_CHANGE", "TARGET_CHANGE", "MANUAL", "GAP", "DATA", "FF_CHANGE" };
    uint32_t rise = value->t10_ms != PITCH_STEP_UNREACHED && value->t90_ms != PITCH_STEP_UNREACHED ?
        value->t90_ms - value->t10_ms : PITCH_STEP_UNREACHED;
    int length = snprintf(response, capacity,
        "P|STEP|ID|%lu|STATE|%s|REASON|%s|VALID|%u|START|%.3f|TARGET|%.3f|WINDOW_MS|%lu|RESP_MS|%ld|T90_MS|%ld|RISE_MS|%ld|SETTLE_MS|%ld|OS_DEG|%.6f|OS_PCT|%.3f|SSE_DEG|%.6f|SSE_VALID|%u|PASS|%u|FAULT|%lu|VMAX_DPS|%.2f|TMAX_NM|%.5f|N|%lu|GAP_MS|%lu|SAT_N|%lu|CMD_N|%lu|FIRST_NM|%.5f|CTRL|CASCADE|PKP|%.6g|PKI|%.6g|PKD|%.6g|VKP|%.6g|VKI|%.6g|VKD|%.6g|OUTPUT_LIMITS|0|RESP_LIMIT_MS|300|RESPONSE|FIRST_IN_BAND|BAND02|0.2|RESP02_MS|%ld|SETTLE02_MS|%ld|PASS02|%u|TIME_LIMIT_MS|300|OS_LIMIT_DEG|0.2|SSE_LIMIT_DEG|0.2|SSE_PEAK_DEG|%.6g|ASSESSMENT|FINAL_SETTLE|KG|%.6g|GOFF_DEG|%.6g|G_CAL|%u|SHAPE_MS|%lu|SHAPEV|%.6g|TERMINAL_BRAKE|%u\r\n",
        (unsigned long)value->id, value->phase <= PITCH_STEP_ABORTED ? phases[value->phase] : "UNKNOWN",
        value->reason <= PITCH_STEP_FF_CHANGE ? reasons[value->reason] : "UNKNOWN",
        (unsigned)(value->phase == PITCH_STEP_DONE), (double)value->start_deg, (double)value->target_deg,
        (unsigned long)value->elapsed_ms, (long)(int32_t)value->response_ms, (long)(int32_t)value->t90_ms,
        (long)(int32_t)rise, (long)(int32_t)value->settle_ms, (double)value->overshoot_deg,
        (double)value->overshoot_pct, (double)value->steady_error_deg, (unsigned)value->sse_valid,
        (unsigned)value->pass, (unsigned long)value->faults, (double)value->peak_velocity_dps,
        (double)value->peak_torque_nm, (unsigned long)value->samples, (unsigned long)value->max_gap_ms,
        (unsigned long)value->limited_samples, (unsigned long)value->command_samples,
        (double)value->first_command_nm, (double)value->kp, (double)value->ki, (double)value->kd,
        (double)value->vkp, (double)value->vki, (double)value->vkd,
        (long)(int32_t)value->response_ms, (long)(int32_t)value->settle_ms, (unsigned)value->pass,
        (double)value->ss_max_error,
        (double)value->gravity_kg_nm, (double)value->gravity_offset_deg, (unsigned)value->gravity_calibrated,
        (unsigned long)value->shape_ms, (double)value->shape_velocity_gain, (unsigned)value->pitch_brake_enabled);
    return (uint8_t)(length > 0 && (size_t)length < capacity);
}

uint8_t PitchConsole_Poll(char *response, size_t capacity)
{
    PitchBackendState pitch;
    PitchStep snapshot, report;
    PitchRemoteStepEvent event;
    uint8_t operation;
    uint32_t started, first_rx, last_query;
    if (!response || capacity < 128U) { return 0U; }
    taskENTER_CRITICAL();
    operation = pending_operation; started = operation_ms; first_rx = query_rx; last_query = query_ms;
    taskEXIT_CRITICAL();
    if (operation) {
        uint32_t now = HAL_GetTick();
        const char *reason = 0;
        PitchBackend_GetState(&pitch);
        if (operation == 1U) {
            const char *result = 0;
            if (pitch.rx_count != first_rx && pitch.online &&
                (int32_t)(pitch.feedback.received_ms - started) >= 0) { result = "NEW_FEEDBACK"; }
            else if (now - started >= PITCH_PREPARATION_TIMEOUT_MS) { result = "TIMEOUT"; }
            if (result && Summary("CHECK", result, response, capacity)) {
                uint8_t deliver;
                taskENTER_CRITICAL(); deliver = (uint8_t)(pending_operation == operation && operation_ms == started);
                if (deliver) { pending_operation = 0U; }
                taskEXIT_CRITICAL(); return deliver;
            }
            return 0U;
        }
        reason = RemoteReason(1U);
        if (strcmp(reason, "OK") == 0) { reason = 0; }
        if (!reason && now - started >= PITCH_PREPARATION_TIMEOUT_MS) { reason = "FEEDBACK_OR_STATIONARY_TIMEOUT"; }
        if (!reason && pitch.online && pitch.rx_count != first_rx &&
            (int32_t)(pitch.feedback.received_ms - started) > 0) {
            if (pitch.feedback.status != 0U) { reason = "DRIVER_NOT_DISABLED"; }
            else if (pitch.feedback.temp_mos >= PITCH_CONTROLLER_DRIVER_LIMIT_C ||
                pitch.feedback.temp_rotor >= PITCH_CONTROLLER_MOTOR_LIMIT_C) { reason = "TEMPERATURE"; }
            else {
                uint8_t prepared = 0U;
                /* Recheck the operation and OFF permission before the short mutation.
                 * All snprintf/float formatting remains outside this critical section. */
                taskENTER_CRITICAL();
                if (pending_operation != operation || operation_ms != started) {
                    taskEXIT_CRITICAL(); return 0U;
                }
                reason = RemoteReason(1U);
                if (strcmp(reason, "OK") == 0) {
                    reason = 0;
                    if (PitchBackend_CaptureReference()) {
                        AppTasks_RequireRearm();
                        prepared = PitchBackend_Request(1U);
                        if (prepared) { pending_operation = 0U; }
                        else { reason = "PREPARE_REJECTED_USE_PITCH_CHECK"; }
                    }
                }
                taskEXIT_CRITICAL();
                if (prepared) {
                    (void)snprintf(response, capacity,
                        "P|OK|READY|ZERO_SET|1|REQUESTED|1|ACTIVE|0|FC_SENT|0|FD_SENT|0|NEXT|GHORIZ_IF_KG_NONZERO_THEN_NEW_CH5_OFF_THEN_MANUAL_OR_TEST|SUPPORT_REQUIRED|1\r\n");
                    return 1U;
                }
            }
        }
        if (reason) {
            uint8_t deliver;
            taskENTER_CRITICAL(); deliver = (uint8_t)(pending_operation == operation && operation_ms == started);
            if (deliver) { pending_operation = 0U; }
            taskEXIT_CRITICAL();
            if (!deliver) { return 0U; }
            (void)snprintf(response, capacity, "P|ERROR|READY|%s|SUPPORT_REQUIRED|1\r\n", reason);
            return 1U;
        }
        if (now - last_query >= 20U) {
            taskENTER_CRITICAL();
            if (pending_operation == operation && operation_ms == started && PitchBackend_QueryFeedback()) { query_ms = now; }
            taskEXIT_CRITICAL();
        }
        return 0U;
    }
    if (PitchBackend_TakeRemoteStepEvent(&event)) {
        static const char *results[] = { "NONE", "STARTED", "BACK_30", "START_GUARD" };
        (void)snprintf(response, capacity, "P|REMOTE_STEP|SEQ|%lu|RESULT|%s|TARGET|%.1f\r\n",
            (unsigned long)event.sequence, event.result <= 3U ? results[event.result] : "UNKNOWN",
            (double)event.target_deg);
        return 1U;
    }
    PitchBackend_GetStep(&snapshot);
    if (snapshot.pending && pitch_console_StepReport(&snapshot, response, capacity)) {
        if (!PitchBackend_TakeStepReport(&report)) { return 0U; }
        if (report.id != snapshot.id) { (void)pitch_console_StepReport(&report, response, capacity); }
        return 1U;
    }
    return GuardReport(response, capacity, 1U);
}

static void pitch_console_BriefStatus(char *response, size_t capacity)
{
    PitchBackendState state;
    PitchBackend_GetState(&state);
    (void)snprintf(response, capacity,
        "P|STATE|%s|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|ZERO_SET|%u|FAULT|%lu|AGE_MS|%lu|ANGLE|%.3f|TARGET|%.3f|VEL_DPS|%.2f|TORQUE_NM|%.5f|KG|%.6g|G_CAL|%u\r\n",
        APP_PITCH_VERSION, (unsigned)state.online, (unsigned)state.active, (unsigned)state.requested,
        (unsigned)state.zero_set, (unsigned long)state.controller.faults, (unsigned long)state.feedback_age_ms,
        (double)state.controller.angle_deg, (double)state.controller.target_deg,
        (double)state.controller.cascade.velocity_dps, (double)state.controller.torque_nm, (double)state.controller.gravity_kg_nm, (unsigned)state.controller.gravity_calibrated);
}

uint8_t PitchConsole_Handle(const char *line, char *response, size_t capacity)
{
    PitchBackendState pitch;
    uint8_t accepted = 0U;
    float value = 0.0f;
    if (!line || !response || !capacity ||
        (strcmp(line, "PITCH") != 0 && strncmp(line, "PITCH ", 6U) != 0)) { return 0U; }
    if (strcmp(line, "PITCH") == 0) { pitch_console_BriefStatus(response, capacity); return 1U; }
    if (strcmp(line, "PITCH HELP") == 0 || strcmp(line, "PITCH ?") == 0) {
        (void)snprintf(response, capacity,
            "P|HELP|READ|PITCH;PID;RESULT;STATUS|PREPARE|READY|STOP|OFF|VIEW|PITCH VIEW|TUNE|POS kp ki kd;VEL kp ki kd;PKP/PKI/PKD/VKP/VKI/VKD value|TELEM|TELEM OFF/ON|TEST|STEP [30];HOME;REMOTE_CH5_CH7|PITCH_ONLY|COORD 1;GHORIZ;FF;KG value;GOFF degrees;SHAPE [ms];SHAPEV [gain];BRAKE [ON/OFF]\r\n");
        return 1U;
    }
    if (strcmp(line, "PITCH OFF") == 0) {
        taskENTER_CRITICAL(); pending_operation = 0U;
        (void)PitchBackend_Request(0U); AppTasks_RequestStop(); taskEXIT_CRITICAL();
        (void)snprintf(response, capacity,
            "P|OK|OFF_REQUESTED|SUPPORT_REQUIRED|1|DISABLE_IS_NOT_BRAKE|1\r\n"); return 1U;
    }
    if (strcmp(line, "PITCH INFO") == 0 || strcmp(line, "PITCH STATUS") == 0) {
        (void)Summary("STATUS", "SNAPSHOT_ONLY", response, capacity); return 1U;
    }
    if (strcmp(line, "PITCH GUARD") == 0) { (void)GuardReport(response, capacity, 0U); return 1U; }
    if (strcmp(line, "PITCH RESULT") == 0) {
        PitchStep snapshot; PitchBackend_GetStep(&snapshot);
        (void)pitch_console_StepReport(&snapshot, response, capacity); return 1U;
    }
    if (strcmp(line, "PITCH PID") == 0) {
        PitchBackend_GetState(&pitch);
        (void)snprintf(response, capacity,
            "P|PID|POS|%.6g,%.6g,%.6g|VEL|%.6g,%.6g,%.6g|KG|%.6g|G_CAL|%u|OUTPUT_LIMITS|0|RUNTIME_RAM_ONLY|1|BOOT_DEFAULTS_SAVED|1\r\n",
            (double)pitch.controller.cascade.position.kp, (double)pitch.controller.cascade.position.ki,
            (double)pitch.controller.cascade.position.kd, (double)pitch.controller.cascade.speed.kp,
            (double)pitch.controller.cascade.speed.ki, (double)pitch.controller.cascade.speed.kd,
            (double)pitch.controller.gravity_kg_nm, (unsigned)pitch.controller.gravity_calibrated);
        return 1U;
    }
    if (strcmp(line, "PITCH FF") == 0) {
        (void)GravityReport("FF", response, capacity); return 1U;
    }
    if (strcmp(line, "PITCH SHAPE") == 0) {
        PitchBackend_GetState(&pitch);
        (void)snprintf(response, capacity,
            "P|SHAPE|MS|%lu|VEL_FF|%.6g|ACTIVE|%u|REF_DEG|%.6g|REF_DPS|%.6g|REF_DPS2|%.6g|GOAL_DEG|%.6g|PI_FREEZE|%u|T0|STEP_ACCEPTED|ASSESSMENT|FINAL_GOAL\r\n",
            (unsigned long)pitch.controller.shape.duration_ms, (double)pitch.controller.shape.velocity_gain,
            (unsigned)pitch.controller.shape.active, (double)pitch.controller.shape.position_deg,
            (double)pitch.controller.shape.velocity_dps, (double)pitch.controller.shape.acceleration_dps2,
            (double)pitch.controller.target_deg, (unsigned)pitch.controller.shape.active);
        return 1U;
    }
    if (strcmp(line, "PITCH SHAPEV") == 0) {
        PitchBackend_GetState(&pitch);
        (void)snprintf(response, capacity, "P|SHAPEV|%.6g\r\n", (double)pitch.controller.shape.velocity_gain);
        return 1U;
    }
    if (strcmp(line, "PITCH BRAKE") == 0) {
        PitchBackend_GetState(&pitch);
        (void)snprintf(response, capacity,
            "P|BRAKE|MODE|TERMINAL_AND_RETURN|ENABLED|%u|TRACKING|%u|ACTIVE|%u|NM|%.6g|WINDOW_DEG|%.6g|GAIN|%.6g|MAX_NM|%.6g|DECEL_DPS2|%.6g|FLOOR_DPS|%.6g|TRIGGER_DPS|%.6g|RETURN_MS|%lu|RETURN_GAIN|%.6g|RETURN_MAX_NM|%.6g|PEAK_NM|%.6g|RETURN_PEAK_NM|%.6g|N|%lu|RETURN_N|%lu|LAND_SEEN|%u|LAND_MS|%lu|VI_START_NM|%.6g|LAND_RAW_DPS|%.6g|LAND_FILT_DPS|%.6g|LAND_VREF_DPS|%.6g|LAND_VI_NM|%.6g|LAND_G_NM|%.6g|LAND_BRAKE_NM|%.6g|LAND_OUT_NM|%.6g|VI_MIN_AFTER_NM|%.6g|VI_MAX_AFTER_NM|%.6g|DIP_DEG|%.6g|DIP_MS|%lu|V_FILTER_MS|%lu|A_FILTER_MS|%lu|DIRECT_STEP_DEFAULT|1\r\n",
            (unsigned)pitch.controller.brake.enabled, (unsigned)pitch.controller.brake.tracking,
            (unsigned)pitch.controller.brake.active, (double)pitch.controller.brake.torque_nm,
            (double)APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG, (double)APP_PITCH_TERMINAL_BRAKE_GAIN,
            (double)APP_PITCH_TERMINAL_BRAKE_LIMIT_NM, (double)APP_PITCH_TERMINAL_BRAKE_DECEL_DPS2,
            (double)APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS, (double)pitch.controller.brake.safe_velocity_dps,
            (unsigned long)APP_PITCH_CAPTURE_BRAKE_WINDOW_MS, (double)APP_PITCH_CAPTURE_BRAKE_GAIN,
            (double)APP_PITCH_CAPTURE_BRAKE_LIMIT_NM, (double)pitch.controller.brake.peak_abs_nm,
            (double)pitch.controller.brake.return_peak_nm, (unsigned long)pitch.controller.brake.active_frames,
            (unsigned long)pitch.controller.brake.return_frames,
            (unsigned)pitch.controller.landing.landed, (unsigned long)pitch.controller.landing.land_ms,
            (double)pitch.controller.landing.start_i_nm, (double)pitch.controller.landing.land_raw_dps,
            (double)pitch.controller.landing.land_filtered_dps, (double)pitch.controller.landing.land_target_dps,
            (double)pitch.controller.landing.land_i_nm, (double)pitch.controller.landing.land_gravity_nm,
            (double)pitch.controller.landing.land_brake_nm, (double)pitch.controller.landing.land_torque_nm,
            (double)pitch.controller.landing.min_i_nm, (double)pitch.controller.landing.max_i_nm,
            (double)pitch.controller.landing.dip_deg, (unsigned long)pitch.controller.landing.dip_ms,
            (unsigned long)APP_PITCH_VELOCITY_FILTER_MS, (unsigned long)APP_PITCH_ACCEL_FILTER_MS);
        return 1U;
    }
    if (pending_operation) {
        (void)snprintf(response, capacity, "P|BUSY|%s|USE_PITCH_OFF_TO_CANCEL\r\n",
            pending_operation == 1U ? "CHECK" : "READY"); return 1U;
    }
    if (strncmp(line, "PITCH BRAKE ", 12U) == 0) {
        if (!AppConsole_IsDiagnosticMode() && (!strcmp(line + 12U, "ON") || !strcmp(line + 12U, "OFF"))) {
            accepted = PitchBackend_SetBrakeEnabled((uint8_t)!strcmp(line + 12U, "ON"));
        }
        (void)snprintf(response, capacity, accepted ? "P|OK|BRAKE|%s|MODE|TERMINAL_AND_RETURN|ON_NEXT_MOVE\r\n" :
            "P|ERROR|BRAKE|USE_ON_OFF|FINISH_STEP_FIRST\r\n", line + 12U);
        return 1U;
    }
    if (strncmp(line, "PITCH SHAPE ", 12U) == 0) {
        if (ConsoleParse_Number(line + 12U, &value) && value >= 0.0f && value <= 250.0f &&
            value == (float)(uint32_t)value && !AppConsole_IsDiagnosticMode()) {
            PitchBackend_GetState(&pitch);
            accepted = PitchBackend_SetShape((uint32_t)value, pitch.controller.shape.velocity_gain);
        }
        (void)snprintf(response, capacity, accepted ? "P|OK|SHAPE|MS|%.0f\r\n" :
            "P|ERROR|SHAPE|USE_0_OR_100_TO_250_MS|FINISH_STEP_FIRST\r\n", (double)value);
        return 1U;
    }
    if (strncmp(line, "PITCH SHAPEV ", 13U) == 0) {
        if (ConsoleParse_Number(line + 13U, &value) && !AppConsole_IsDiagnosticMode()) {
            PitchBackend_GetState(&pitch);
            accepted = PitchBackend_SetShape(pitch.controller.shape.duration_ms, value);
        }
        (void)snprintf(response, capacity, accepted ? "P|OK|SHAPEV|%.6g\r\n" :
            "P|ERROR|SHAPEV|USE_0_TO_1|FINISH_STEP_FIRST\r\n", (double)value);
        return 1U;
    }
    if (strcmp(line, "PITCH CHECK") == 0 || strcmp(line, "PITCH READY") == 0) {
        const char *reason = "QUERY_UNAVAILABLE_CAN_OR_MAILBOX";
        uint8_t ready = (uint8_t)(strcmp(line, "PITCH READY") == 0);
        uint8_t already = 0U;
        taskENTER_CRITICAL(); PitchBackend_GetState(&pitch);
        if (ready) {
            reason = RemoteReason(1U);
            already = (uint8_t)(strcmp(reason, "OK") == 0 && pitch.requested && pitch.zero_set &&
                pitch.coord_confirmed && pitch.online && pitch.feedback.status == 0U &&
                !pitch.controller.faults && !pitch.active && !pitch.enabling && !pitch.stop_pending);
            if (strcmp(reason, "OK") == 0 && !pitch.coord_confirmed) { reason = "COORD_UNCONFIRMED_USE_PITCH_COORD_1_OR_7"; }
            if (strcmp(reason, "OK") == 0 &&
                (pitch.requested || pitch.active || pitch.enabling || pitch.stop_pending)) { reason = "OFF_REQUIRED_SUPPORT_LOAD"; }
        }
        if (!already && (!ready || strcmp(reason, "OK") == 0) && PitchBackend_QueryFeedback()) {
            pending_operation = ready ? 2U : 1U; query_rx = pitch.rx_count;
            operation_ms = query_ms = HAL_GetTick(); accepted = 1U;
        } else if (ready && strcmp(reason, "OK") == 0) { reason = "QUERY_UNAVAILABLE_CAN_OR_MAILBOX"; }
        taskEXIT_CRITICAL();
        if (already) {
            (void)snprintf(response, capacity,
                "P|OK|ALREADY_READY|ZERO_SET|1|REQUESTED|1|ACTIVE|0|REZEROED|0|FC_SENT|0|FD_SENT|0|SUPPORT_REQUIRED|1\r\n");
            return 1U;
        }
        if (accepted) {
            (void)snprintf(response, capacity, "P|BUSY|%s|QUERY|CC|FC_SENT|0|FD_SENT|0|READ_ONLY_MONITOR|1\r\n", ready ? "READY" : "CHECK");
        } else { (void)snprintf(response, capacity, "P|ERROR|%s|%s\r\n", ready ? "READY" : "CHECK", reason); }
        return 1U;
    }
    if (strncmp(line, "PITCH KG ", 9U) == 0 || strcmp(line, "PITCH FF OFF") == 0) {
        uint8_t parsed = (uint8_t)(strcmp(line, "PITCH FF OFF") == 0 || ConsoleParse_Number(line + 9U, &value));
        taskENTER_CRITICAL();
        if (parsed && !AppConsole_IsDiagnosticMode()) { accepted = PitchBackend_SetGravityGain(value); }
        taskEXIT_CRITICAL();
        if (accepted) { (void)GravityReport("OK|KG_SET", response, capacity); }
        else { (void)snprintf(response, capacity, "P|ERROR|KG_REQUIRES_CALIBRATED_HEALTHY_ZERO_AND_RANGE_MINUS10_PLUS10\r\n"); }
        return 1U;
    }
    if (strncmp(line, "PITCH GOFF ", 11U) == 0 || strcmp(line, "PITCH GHORIZ") == 0) {
        uint8_t horizontal = (uint8_t)(strcmp(line, "PITCH GHORIZ") == 0);
        uint8_t parsed = (uint8_t)(horizontal || ConsoleParse_Number(line + 11U, &value));
        taskENTER_CRITICAL();
        if (parsed && strcmp(RemoteReason(1U), "OK") == 0) {
            accepted = PitchBackend_SetGravityOffset(value, horizontal);
        }
        taskEXIT_CRITICAL();
        if (accepted) { (void)GravityReport("OK|HORIZONTAL_SET", response, capacity); }
        else { (void)snprintf(response, capacity, "P|ERROR|GOFF_REQUIRES_OFF_ZERO_FRESH_DISABLED_AND_RANGE_MINUS180_PLUS180\r\n"); }
        return 1U;
    }
    if (strncmp(line, "PITCH COORD ", 12U) == 0) {
        uint8_t parsed = (uint8_t)(ConsoleParse_Number(line + 12U, &value) && (value == 1.0f || value == 7.0f));
        taskENTER_CRITICAL();
        if (parsed &&
            strcmp(RemoteReason(1U), "OK") == 0) { accepted = PitchBackend_SetCoordinate((uint8_t)value); }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "P|OK|COORD|DIVISOR|%.0f|ZERO_SET|0\r\n" :
            "P|ERROR|COORD_REQUIRES_OFF_FRESH_DISABLED_AND_1_OR_7\r\n", (double)value); return 1U;
    }
    if (strcmp(line, "PITCH ZERO") == 0) {
        taskENTER_CRITICAL();
        if (strcmp(RemoteReason(1U), "OK") == 0) { accepted = PitchBackend_CaptureReference(); }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "P|OK|SOFTWARE_ZERO|PERMANENT_ZERO_WRITTEN|0\r\n" :
            "P|ERROR|ZERO_REQUIRES_OFF_COORD_AND_SUPPORTED_STATIONARY_FEEDBACK\r\n"); return 1U;
    }
    if (strncmp(line, "PITCH STEP ", 11U) == 0 || strcmp(line, "PITCH STEP") == 0) {
        uint8_t parsed;
        value = 30.0f;
        parsed = (uint8_t)(strcmp(line, "PITCH STEP") == 0 || ConsoleParse_Number(line + 11U, &value));
        if (parsed && (value < PITCH_CONTROLLER_TARGET_MIN_DEG || value > PITCH_CONTROLLER_TARGET_MAX_DEG)) {
            (void)snprintf(response, capacity, "P|ERROR|STEP_OUTSIDE_MECHANICAL_RANGE|MIN_DEG|%.0f|MAX_DEG|%.0f\r\n",
                (double)PITCH_CONTROLLER_TARGET_MIN_DEG, (double)PITCH_CONTROLLER_TARGET_MAX_DEG);
            return 1U;
        }
        taskENTER_CRITICAL();
        if (parsed &&
            strcmp(RemoteReason(0U), "OK") == 0) { accepted = PitchBackend_StartStep(value); }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "P|OK|STEP_STARTED|TARGET|%.1f|WINDOW_MS|5000|HOLD|1\r\n" :
            "P|ERROR|STEP_REQUIRES_ARMED_READY_STATIONARY_ZERO_AND_PLUS_MINUS_30\r\n", (double)value); return 1U;
    }
    if (strcmp(line, "PITCH HOME") == 0) {
        taskENTER_CRITICAL();
        if (strcmp(RemoteReason(0U), "OK") == 0) { accepted = PitchBackend_ReturnTo(0.0f); }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "P|OK|HOME|TARGET|0\r\n" :
            "P|ERROR|HOME_REQUIRES_ARMED_ACTIVE_HEALTHY_UNLOCKED_AXIS\r\n"); return 1U;
    }
    if (strncmp(line, "PITCH TARGET ", 13U) == 0) {
        uint8_t parsed = ConsoleParse_Number(line + 13U, &value);
        taskENTER_CRITICAL();
        if (parsed && strcmp(RemoteReason(0U), "OK") == 0) {
            accepted = PitchBackend_SetTarget(value);
        }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "P|OK|TARGET|%.3f|HOLD|1\r\n" :
            "P|ERROR|TARGET_REQUIRES_ARMED_ACTIVE_IDLE_PROFILE_AND_STATIONARY_WITHIN_RANGE\r\n", (double)value); return 1U;
    }
    if (strncmp(line, "PITCH POS ", 10U) == 0 || strncmp(line, "PITCH VEL ", 10U) == 0) {
        float kp, ki, kd;
        char loop = line[6] == 'P' ? 'P' : 'V';
        if (ConsoleParse_PidNumbers(line + 10U, &kp, &ki, &kd) && !AppConsole_IsDiagnosticMode()) {
            accepted = PitchBackend_SetLoopPid(loop, kp, ki, kd);
        }
        (void)snprintf(response, capacity, accepted ? "P|OK|%s\r\n" : "P|ERROR|PID_RANGE_OR_FORMAT\r\n",
            loop == 'P' ? "POS" : "VEL"); return 1U;
    }
    if (strlen(line) >= 10U && (line[6] == 'P' || line[6] == 'V') && line[7] == 'K' &&
        (line[8] == 'P' || line[8] == 'I' || line[8] == 'D') && line[9] == ' ') {
        if (ConsoleParse_Number(line + 10U, &value) && !AppConsole_IsDiagnosticMode()) {
            accepted = PitchBackend_SetLoopParameter(line[6], line[8], value);
        }
        (void)snprintf(response, capacity, accepted ? "P|OK|PARAMETER\r\n" : "P|ERROR|PID_RANGE_OR_FORMAT\r\n"); return 1U;
    }
    if (strncmp(line, "PITCH VMAX", 10U) == 0) {
        (void)snprintf(response, capacity, "P|ERROR|OUTPUT_LIMITS_DISABLED\r\n"); return 1U;
    }
    (void)snprintf(response, capacity, "P|ERROR|UNKNOWN_COMMAND|USE|PITCH HELP\r\n"); return 1U;
}
/* APP_SECTION_END pitch_console.c */

/* APP_SECTION_BEGIN feeder_console.c */
/* FEEDER 命令：单发、连发、配置及角度/速度遥测。 */
#include "feeder_console.h"
#include "feeder_backend.h"
#include "motor_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static void Status(const char *operation, uint8_t ok, char *out, size_t capacity)
{
    FeederBackendState state;
    MotorHealth wheels;
    const FeederController *c;
    FeederBackend_GetState(&state); MotorBackend_GetHealth(&wheels); c = &state.controller;
    (void)snprintf(out, capacity,
        "F|%s|%s|%s|ONLINE|%u|RX|%lu|AGE|%lu|RAW|%u|ROTOR_DEG|%.4f|POS_DEG|%.4f|TARGET_DEG|%.4f|RPM|%d|VREF|%.2f|CMD_RAW|%d|FB_TORQUE_RAW|%d|ARMED|%u|ZERO|%u|BUSY|%u|FAULT|%lu|STOP_PENDING|%u|WHEELS_READY|%u|STEP|%.6g|DIR|%d|GEAR|%.6g|CAL|%u|PKP|%.6g|VKP|%.6g|VKI|%.6g|MAX_RPM|%.6g|LIMIT|%.6g|TOL|%.6g|SETTLE_RPM|%.6g|SETTLE_MS|%lu|TIMEOUT_MS|%lu|JAM_RAW|%.6g|JAM_MS|%lu|JUMP_SLACK|%u|GAP_MARGIN_MS|%u|TS_ALLOW|%lu|ACCEPT|%lu|DONE|%lu|REJECT|%lu|ABORT|%lu|SINGLE|%lu|CONT|%lu|SHOT_MS|%lu|RATE_HZ|%.3f|PENDING|%u|CANCEL|%lu|DONE_DT_MS|%lu\r\n",
        operation, ok ? "OK" : "REJECTED", APP_FEEDER_VERSION,
        (unsigned)state.online, (unsigned long)c->feedback_sequence,
        c->have_feedback ? (unsigned long)(HAL_GetTick() - c->last_feedback_ms) : 0xFFFFFFFFUL,
        (unsigned)c->raw_angle, c->position_deg, c->axis_position_deg, c->axis_target_deg,
        (int)c->raw_rpm, (double)c->target_rpm, (int)c->command_raw, (int)c->raw_current,
        (unsigned)state.armed, (unsigned)c->have_zero, (unsigned)c->busy, (unsigned long)c->faults,
        (unsigned)state.stop_pending, (unsigned)wheels.flywheels_ready,
        (double)c->config.step_deg, (int)c->config.direction, (double)c->config.gear_ratio,
        (unsigned)c->config.calibrated, (double)c->config.pos_kp, (double)c->config.speed_kp,
        (double)c->config.speed_ki, (double)c->config.max_rpm, (double)c->config.current_limit,
        (double)c->config.tolerance_deg, (double)c->config.settle_rpm,
        (unsigned long)c->config.settle_ms, (unsigned long)c->config.motion_timeout_ms,
        (double)c->config.jam_current_raw, (unsigned long)c->config.jam_ms,
        (unsigned)FEEDER_JUMP_SLACK_COUNTS, (unsigned)FEEDER_GAP_UNCERTAINTY_MS,
        (unsigned long)c->timestamp_allowances,
        (unsigned long)c->accepted, (unsigned long)c->completed, (unsigned long)c->rejected,
        (unsigned long)c->aborted, (unsigned long)state.single_accepted,
        (unsigned long)state.continuous_accepted, (unsigned long)c->last_motion_ms,
        (double)c->measured_hz, (unsigned)c->outstanding, (unsigned long)c->cancelled,
        (unsigned long)state.completed_interval_ms);
}

static void Result(char *out, size_t capacity)
{
    FeederBackendState state;
    const FeederController *c;
    FeederBackend_GetState(&state); c = &state.controller;
    (void)snprintf(out, capacity,
        "F|RESULT|%s|COUNT_SOURCE|ENCODER_40DEG|ACCEPT|%lu|DONE|%lu|SINGLE_DONE|%lu|CONT_DONE|%lu|REJECT|%lu|ABORT|%lu|CANCEL|%lu|PENDING|%u|KIND|%u|SHOT_MS|%lu|SHOT_ERR_DEG|%.4f|BURST_DONE|%lu|WINDOW_MS|%lu|RATE_HZ|%.3f|RATE_VALID|%u|REQUEST_MS|%u|REQUIRED_RPM|%.2f|CONT_MAX_RPM|%.6g|PHASE_MS|%.6g|BRAKE_KP|%.6g|RATE_BLOCK|%u|FAULT|%lu|SINGLE_DECEL_RPM_S|%.6g|BRAKE_DELAY_MS|%u|PREVIEW_MS|%u|I_RELAX_MS|%u|SINGLE_BRAKE|%u|SINGLE_BRAKE_N|%lu|SINGLE_OVERSHOOT_DEG|%.4f\r\n",
        APP_FEEDER_VERSION, (unsigned long)c->accepted, (unsigned long)c->completed,
        (unsigned long)c->single_completed, (unsigned long)c->continuous_completed,
        (unsigned long)c->rejected, (unsigned long)c->aborted, (unsigned long)c->cancelled,
        (unsigned)c->outstanding, (unsigned)c->motion_kind, (unsigned long)c->last_motion_ms,
        (double)c->last_motion_error_deg, (unsigned long)c->burst_completed,
        (unsigned long)c->burst_elapsed_ms, (double)c->measured_hz,
        (unsigned)(c->burst_completed >= 2U && c->burst_elapsed_ms != 0U),
        (unsigned)APP_FIRE_INTERVAL_MS, (double)Feeder_ContinuousRpm(c),
        (double)c->config.continuous_max_rpm, (double)c->config.continuous_phase_ms,
        (double)c->config.continuous_brake_kp, (unsigned)c->rate_blocked,
        (unsigned long)c->faults, (double)c->config.single_decel_rpm_s,
        (unsigned)FEEDER_SINGLE_BRAKE_DELAY_MS, (unsigned)FEEDER_SINGLE_PREVIEW_MS,
        (unsigned)FEEDER_SINGLE_I_RELAX_MS,
        (unsigned)c->single_braking, (unsigned long)c->single_brake_frames,
        (double)c->single_overshoot_deg);
}

static void Check(char *out, size_t capacity)
{
    FeederBackendState state;
    RemoteInput_State remote;
    AppState app;
    CanBusState bus;
    const FeederJumpDiagnostic *jump;
    const char *cause;
    char data[17];
    uint32_t i, now, uart_epoch;
    static const char hex[] = "0123456789ABCDEF";
    /* Read one coherent snapshot, format only after interrupts are unmasked. */
    taskENTER_CRITICAL();
    FeederBackend_GetState(&state); RemoteInput_GetState(&remote);
    AppTasks_GetState(&app); CanBus_GetState(&bus);
    now = HAL_GetTick(); uart_epoch = remote.fault_epoch + AppConsole_GetFaultEpoch();
    taskEXIT_CRITICAL();
    jump = &state.controller.jump;
    cause = !jump->valid ? "NONE" : jump->flags == FEEDER_JUMP_DELTA ? "DELTA" :
        jump->flags == FEEDER_JUMP_HALF_TURN ? "HALF_TURN" : "DELTA_AND_HALF_TURN";
    for (i = 0U; i < 8U; ++i) {
        data[i * 2U] = hex[jump->data[i] >> 4];
        data[i * 2U + 1U] = hex[jump->data[i] & 15U];
    }
    data[16] = '\0';
    (void)snprintf(out, capacity,
        "F|CHECK|%s|FAULT|%lu|ONLINE|%u|ARMED|%u|BUSY|%u|CAL|%u|DIR|%d|STEP|%.6g|JUMP_VALID|%u|JUMP_COUNT|%lu|CAUSE|%s|FLAGS|%lu|AT_MS|%lu|SEQ|%lu|PREV_RAW|%u|RAW|%u|DELTA_COUNTS|%ld|GAP_MS|%lu|GAP_BOUND_MS|%lu|GAP_MARGIN_MS|%u|TS_ALLOW|%lu|FRAME_AGE_MS|%lu|PREV_RPM|%d|RPM|%d|LIMIT_COUNTS|%.6g|EXPECTED_COUNTS|%.6g|FB_TORQUE_RAW|%d|BEFORE_CMD|%d|BEFORE_BUSY|%u|BEFORE_ZERO|%u|FRAME|%s|RC_AGE_MS|%lu|CH|%u,%u,%u,%u,%u,%u,%u,%u|SAFETY|%u|SAFETY_REASON|%u|BARRIER|%u|OP_ARMED|%u|UART_CURRENT|%lu|UART_SEEN|%lu|CAN_EPOCH|%lu\r\n",
        APP_FEEDER_VERSION, (unsigned long)state.controller.faults,
        (unsigned)state.online, (unsigned)state.armed, (unsigned)state.controller.busy,
        (unsigned)state.controller.config.calibrated, (int)state.controller.config.direction,
        (double)state.controller.config.step_deg, (unsigned)jump->valid,
        (unsigned long)jump->events, cause, (unsigned long)jump->flags,
        (unsigned long)jump->at_ms, (unsigned long)jump->sequence,
        (unsigned)jump->previous_angle, (unsigned)jump->angle, (long)jump->delta_counts,
        (unsigned long)jump->gap_ms, (unsigned long)jump->bound_gap_ms,
        (unsigned)FEEDER_GAP_UNCERTAINTY_MS, (unsigned long)state.controller.timestamp_allowances,
        (unsigned long)jump->age_ms,
        (int)jump->previous_rpm, (int)jump->rpm, (double)jump->limit_counts,
        (double)jump->expected_counts, (int)jump->feedback_raw, (int)jump->command_raw,
        (unsigned)jump->busy, (unsigned)jump->zero_valid, data,
        remote.has_frame ? (unsigned long)(now - remote.received_ms) : 0xFFFFFFFFUL,
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3],
        (unsigned)remote.channels[4], (unsigned)remote.channels[5],
        (unsigned)remote.channels[6], (unsigned)remote.channels[7],
        (unsigned)app.logic.safety.state, (unsigned)app.logic.safety.reason,
        (unsigned)app.logic.safety.barrier_valid, (unsigned)app.logic.safety.operator_armed,
        (unsigned long)uart_epoch, (unsigned long)app.logic.safety.uart_epoch,
        (unsigned long)bus.fault_epoch);
}

uint8_t FeederConsole_Handle(const char *line, char *out, size_t capacity)
{
    const char *argument;
    char *end;
    char key[16];
    float value;
    size_t length;
    uint8_t ok = 0U;
    FeederBackendState state;
    FeederConfig config;
    if (strncmp(line, "FEEDER", 6U) != 0 || (line[6] && line[6] != ' ')) { return 0U; }
    if (!strcmp(line, "FEEDER HELP")) {
        (void)snprintf(out, capacity,
            "F|HELP|READ|FEEDER INFO;STATUS;RESULT;PID;VIEW;ANGLE VIEW;SPEED VIEW;CHECK|ANGLE_I|0_ACTUAL_1_TARGET_SHAFT_DEG|REQUEST_MS|50|CONTROL_MS|2|CHECK|FIRST_JUMP_SINCE_CLEAR|SAFE_OFF|AUTO_PREPARE_OR_READY;OFF;CLEAR;ZERO;ARM|ACTION|SINGLE_REQUIRES_NORMAL_AND_REAL_WHEELS_READY|TUNE|PKP;VKP;VKI;RPM;CONT_RPM;PHASE_MS;BRAKE_KP;SINGLE_DECEL;LIMIT;TOL;SETTLE;TIMEOUT;STEP;DIR;GEAR;CAL|BOOT|CAL1_DIR-1_GEAR35.68_SAFE_OFF_AUTO_ZERO_ARM|SHAFT|40DEG_CLOCKWISE_FROM_TOP|RAM_TUNING|1\r\n"); return 1U;
    }
    if (!strcmp(line, "FEEDER RESULT")) { Result(out, capacity); return 1U; }
    if (!strcmp(line, "FEEDER CHECK")) { Check(out, capacity); return 1U; }
    if (!strcmp(line, "FEEDER") || !strcmp(line, "FEEDER INFO") ||
        !strcmp(line, "FEEDER STATUS") || !strcmp(line, "FEEDER PID")) {
        Status("STATUS", 1U, out, capacity); return 1U;
    }
    if (!strcmp(line, "FEEDER OFF")) { FeederBackend_ManualOff(); ok = 1U; }
    else if (!strcmp(line, "FEEDER CLEAR")) { ok = FeederBackend_Clear(); }
    else if (!strcmp(line, "FEEDER ZERO")) { ok = FeederBackend_Zero(); }
    else if (!strcmp(line, "FEEDER ARM")) { ok = FeederBackend_Arm(); }
    else if (!strcmp(line, "FEEDER READY")) { ok = FeederBackend_Ready(); }
    else if (!strcmp(line, "FEEDER SINGLE")) { ok = FeederBackend_Single(); }
    else {
        argument = strchr(line + 7U, ' ');
        if (!argument) { Status("BAD_COMMAND", 0U, out, capacity); return 1U; }
        length = (size_t)(argument - (line + 7U));
        if (!length || length >= sizeof(key)) { Status("BAD_COMMAND", 0U, out, capacity); return 1U; }
        memcpy(key, line + 7U, length); key[length] = '\0';
        value = strtof(argument + 1U, &end);
        if (end == argument + 1U || *end || !isfinite(value)) {
            Status("BAD_VALUE", 0U, out, capacity); return 1U;
        }
        FeederBackend_GetState(&state); config = state.controller.config;
        if (!strcmp(key, "PKP")) { config.pos_kp = value; }
        else if (!strcmp(key, "VKP")) { config.speed_kp = value; }
        else if (!strcmp(key, "VKI")) { config.speed_ki = value; }
        else if (!strcmp(key, "RPM")) { config.max_rpm = value; }
        else if (!strcmp(key, "CONT_RPM")) { config.continuous_max_rpm = value; }
        else if (!strcmp(key, "PHASE_MS")) { config.continuous_phase_ms = value; }
        else if (!strcmp(key, "BRAKE_KP")) { config.continuous_brake_kp = value; }
        else if (!strcmp(key, "SINGLE_DECEL")) { config.single_decel_rpm_s = value; }
        else if (!strcmp(key, "LIMIT")) { config.current_limit = value; }
        else if (!strcmp(key, "TOL")) { config.tolerance_deg = value; }
        else if (!strcmp(key, "STEP")) { config.step_deg = value; }
        else if (!strcmp(key, "GEAR")) { config.gear_ratio = value; }
        else if (!strcmp(key, "SETTLE") && value >= 0 && value <= 1000 && floorf(value) == value) {
            config.settle_ms = (uint32_t)value;
        } else if (!strcmp(key, "TIMEOUT") && value >= 20 && value <= 10000 && floorf(value) == value) {
            config.motion_timeout_ms = (uint32_t)value;
        } else if (!strcmp(key, "DIR") && (value == -1 || value == 1 || value == 0)) {
            config.direction = (int8_t)value;
        } else if (!strcmp(key, "CAL") && (value == 0 || value == 1)) {
            config.calibrated = (uint8_t)value;
        } else { Status("BAD_KEY_OR_VALUE", 0U, out, capacity); return 1U; }
        ok = FeederBackend_SetConfig(&config);
    }
    Status(line + 7U, ok, out, capacity); return 1U;
}

void FeederConsole_Telemetry(char *out, size_t capacity)
{
    FeederBackendState state;
    MotorHealth wheels;
    const FeederController *c;
    double display_position, display_target;
    FeederBackend_GetState(&state); MotorBackend_GetHealth(&wheels); c = &state.controller;
    /* VOFA I0/I1 use the opposite angle convention only at the serial output.
     * Keep the accepted 40-degree target staircase, not the stream ramp.
     * Normalize exact zero so the initial display does not print "-0". */
    display_position = c->axis_position_deg == 0.0 ? 0.0 : -c->axis_position_deg;
    display_target = c->axis_target_deg == 0.0 ? 0.0 : -c->axis_target_deg;
    (void)snprintf(out, capacity,
        "feeder:%.7g,%.7g,%d,%.6g,%d,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%.6g,%u,%u,%u,%u,%u,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,%.6g,%.6g,%lu,%lu,%lu,%.8g,%.8g\r\n",
        display_position, display_target, (int)c->raw_rpm, (double)c->target_rpm,
        (int)c->command_raw, (int)c->raw_current,
        (double)c->config.pos_kp, (double)c->config.speed_kp, (double)c->config.speed_ki,
        (double)c->config.max_rpm, (double)c->config.current_limit, (double)c->config.step_deg,
        (int)c->config.direction, (double)c->config.gear_ratio, (unsigned)c->config.calibrated,
        (unsigned)c->have_zero, (unsigned)state.online, (unsigned)state.armed, (unsigned)c->busy,
        (unsigned long)c->faults, (unsigned)wheels.flywheels_ready,
        (unsigned long)c->accepted, (unsigned long)c->completed, (unsigned long)c->rejected,
        (unsigned long)c->aborted, (unsigned long)c->feedback_sequence,
        c->have_feedback ? (unsigned long)(HAL_GetTick() - c->last_feedback_ms) : 0xFFFFFFFFUL,
        (unsigned)c->raw_angle, (unsigned)state.stop_pending,
        (unsigned long)state.single_accepted, (unsigned long)state.continuous_accepted,
        (unsigned long)state.completed_interval_ms, (double)c->config.tolerance_deg,
        (double)c->config.settle_rpm, (unsigned long)c->config.settle_ms,
        (unsigned long)c->config.motion_timeout_ms, (unsigned long)c->last_motion_ms,
        c->position_deg, c->target_deg);
}

void FeederConsole_SpeedTelemetry(char *out, size_t capacity)
{
    FeederBackendState state;
    const FeederController *c;
    FeederBackend_GetState(&state); c = &state.controller;
    /* Signed rotor rpm matches the existing controller coordinates; shaft rpm
     * divides by the configured gearbox ratio, without changing direction. */
    (void)snprintf(out, capacity, "feeder_speed:%d,%.6g,%.6g,%.6g,%.6g,%u,%u,%lu,%lu\r\n",
        (int)c->raw_rpm, (double)c->target_rpm,
        (double)c->raw_rpm / c->config.gear_ratio,
        (double)c->target_rpm / c->config.gear_ratio, (double)c->config.gear_ratio,
        (unsigned)state.online, (unsigned)state.armed, (unsigned long)c->faults,
        c->have_feedback ? (unsigned long)(HAL_GetTick() - c->last_feedback_ms) : 0xFFFFFFFFUL);
}
/* APP_SECTION_END feeder_console.c */
