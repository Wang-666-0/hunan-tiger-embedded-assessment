#include "yaw_console.h"
#include "yaw_backend.h"
#include "app_config.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"
#include "can_bus.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

/* READY是一次有时限的人工会话；每个阶段重新验证安全OFF，绝不自动使能。 */
static uint8_t ready_phase, ready_clear_sent;
static uint32_t ready_started_ms;
#define READY_TIMEOUT_MS 5000U

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
    if ((uint32_t)(now - ready_started_ms) >= READY_TIMEOUT_MS) {
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

static uint8_t StepReport(const YawStep *step, char *response, size_t capacity)
{
    static const char *phases[] = { "IDLE", "RUNNING", "DONE", "ABORTED" };
    static const char *reasons[] = { "NONE", "STOP", "FAULT", "PID_CHANGE", "TARGET_CHANGE", "MANUAL", "GAP", "DATA", "FF_CHANGE" };
    uint32_t rise = step->t10_ms != YAW_STEP_UNREACHED && step->t90_ms != YAW_STEP_UNREACHED ?
        step->t90_ms - step->t10_ms : YAW_STEP_UNREACHED;
    int length = snprintf(response, capacity,
        "Y|STEP|ID|%lu|STATE|%s|REASON|%s|VALID|%u|START|%.3f|TARGET|%.3f|WINDOW_MS|%lu|RESP_MS|%ld|T90_MS|%ld|RISE_MS|%ld|SETTLE_MS|%ld|OS_DEG|%.3f|OS_PCT|%.3f|SSE_DEG|%.3f|SSE_VALID|%u|PASS|%u|FAULT|%lu|VMAX_DPS|%.2f|TMAX_NM|%.5f|N|%lu|GAP_MS|%lu|%s|%lu|%s|%.4f|%s|%.4f|%s|%.4f|CMD_N|%lu|FIRST_NM|%.5f",
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
                "|BAND05|0.5|RESP05_MS|%ld|SETTLE05_MS|%ld|PASS05|%u"
                "|FF_ON|%u|FF_TYPE|ACCEL|FKA|%.6g|FMAX|%.6g|SHAPE_MS|%lu|SHAPE_V|%.6g\r\n",
                (double)step->vkp, (double)step->vki, (double)step->vkd,
                (double)step->vmax_dps, (unsigned)YAW_CASCADE_VELOCITY_FILTER_MS, (unsigned)step->output_limits,
                (long)(int32_t)step->response05_ms, (long)(int32_t)step->settle05_ms, (unsigned)step->pass05,
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
        static const char *results[] = { "NONE", "STARTED", "RETURN_ZERO", "START_GUARD",
#if APP_PITCH_ENABLED
            "OTHER_AXIS"
#else
            "PITCH_NOT_IMPLEMENTED"
#endif
        };
        (void)snprintf(response, capacity, "Y|REMOTE_STEP|SEQ|%lu|RESULT|%s|TARGET|%.1f\r\n",
            (unsigned long)event.sequence, event.result <= 4U ? results[event.result] : "UNKNOWN",
            (double)event.target_deg);
        return 1U;
    }
    YawBackend_GetStep(&snapshot);
    if (!snapshot.pending || !StepReport(&snapshot, response, capacity)) { return 0U; }
    if (!YawBackend_TakeStepReport(&report)) { return 0U; }
    if (report.id != snapshot.id) { return StepReport(&report, response, capacity); }
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

static uint8_t Number(const char *text, float *value)
{
    char *end;
    float parsed = strtof(text, &end);
    if (end == text || *end || !(parsed >= -FLT_MAX && parsed <= FLT_MAX)) { return 0U; }
    *value = parsed; return 1U;
}

static uint8_t PidNumbers(const char *text, float *kp, float *ki, float *kd)
{
    char *end;
    float *values[3] = { kp, ki, kd };
    uint32_t i;
    for (i = 0U; i < 3U; ++i) {
        *values[i] = strtof(text, &end);
        if (end == text || !(*values[i] >= -FLT_MAX && *values[i] <= FLT_MAX)) { return 0U; }
        if (i == 2U) { return *end == '\0'; }
        if (*end != ' ') { return 0U; }
        text = end + 1;
    }
    return 0U;
}

static void BriefStatus(char *response, size_t capacity)
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
    if (strcmp(line, "YAW") == 0) { BriefStatus(response, capacity); return 1U; }
    if (strcmp(line, "YAW HELP") == 0 || strcmp(line, "YAW ?") == 0) {
        (void)snprintf(response, capacity,
            "Y|HELP|READ|YAW;PID;RESULT;STATUS|PREPARE|READY|STOP|OFF|VIEW|YAW VIEW|TUNE|POS kp ki kd;VEL kp ki kd;PKP/PKI/PKD/VKP/VKI/VKD value|FF|FF;FF ON/OFF;FKA [0_to_0.001];FMAX [0_to_1]|SHAPE|SHAPE [0_or_80_to_500_ms];SHAPEV [0_to_1]|TELEM|TELEM OFF/ON|TEST|STEP [30];HOME;REMOTE_CH5_CH7|DETAIL|INFO;CHECK;BUS;ZERO;CLEAR\r\n");
        return 1U;
    }
    if (strcmp(line, "YAW BUS") == 0) {
        BusReport(response, capacity); return 1U;
    }
    if (strcmp(line, "YAW RESULT") == 0) {
        YawStep snapshot; YawBackend_GetStep(&snapshot);
        (void)StepReport(&snapshot, response, capacity); return 1U;
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
    if (strcmp(line, "YAW STEP") == 0 || strncmp(line, "YAW STEP ", 9U) == 0) {
        value = 30.0f;
        taskENTER_CRITICAL();
        if ((strcmp(line, "YAW STEP") == 0 || Number(line + 9U, &value)) && SafeStep()) {
            accepted = YawBackend_StartStep(value);
        }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "Y|OK|STEP_STARTED|TARGET|%.1f|WINDOW_MS|5000|HOLD|1\r\n" :
            "Y|ERROR|STEP_REQUIRES_ARMED_ZERO_STATIONARY_HEALTHY\r\n", (double)value);
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
        if (Number(line + 11U, &value) && !AppConsole_IsDiagnosticMode()) {
            accepted = YawBackend_SetShapeVelocityGain(value);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|SHAPEV|%.6g\r\n" :
            "Y|ERROR|SHAPEV|USE_0_TO_1|FINISH_STEP_FIRST\r\n", (double)value);
        return 1U;
    }
    if (strncmp(line, "YAW SHAPE ", 10U) == 0) {
        if (Number(line + 10U, &value) && value >= 0.0f && value <= 500.0f &&
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
                (void)snprintf(response, capacity, "Y|ERROR|REMOVED|%s|USE|FKA|FMAX\r\n", removed[i]);
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
                if (Number(line + 5U + length, &value) && !AppConsole_IsDiagnosticMode()) {
                    accepted = YawBackend_SetFeedforward(parameters[i], value);
                }
                (void)snprintf(response, capacity, accepted ? "Y|OK|%s\r\n" :
                    "Y|ERROR|FF_PARAMETER_OR_STATE|%s\r\n", names[i]); return 1U;
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
        if (PidNumbers(line + 8U, &kp, &ki, &kd) && !AppConsole_IsDiagnosticMode()) {
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
        if (Number(line + 9U, &value) && !AppConsole_IsDiagnosticMode()) { accepted = YawBackend_SetVelocityLimit(value); }
        (void)snprintf(response, capacity, accepted ? "Y|OK|VMAX\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n"); return 1U;
#else
        (void)snprintf(response, capacity, "Y|ERROR|OUTPUT_LIMITS_DISABLED\r\n"); return 1U;
#endif
    }
    if (strlen(line) >= 8U && (line[4] == 'P' || line[4] == 'V') && line[5] == 'K' &&
        (line[6] == 'P' || line[6] == 'I' || line[6] == 'D') && line[7] == ' ') {
        if (Number(line + 8U, &value) && !AppConsole_IsDiagnosticMode()) {
            accepted = YawBackend_SetLoopParameter(line[4], line[6], value);
        }
        (void)snprintf(response, capacity, accepted ? "Y|OK|PARAMETER\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n"); return 1U;
    }
#else
    if (strncmp(line, "YAW PID ", 8U) == 0) {
        float kp, ki, kd;
        if (PidNumbers(line + 8U, &kp, &ki, &kd) && !AppConsole_IsDiagnosticMode()) {
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
    if (Number(number, &value) && !AppConsole_IsDiagnosticMode()) {
        accepted = YawBackend_SetParameter(parameter, value);
    }
    (void)snprintf(response, capacity, accepted ? "Y|OK|PARAMETER\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n");
    return 1U;
}
