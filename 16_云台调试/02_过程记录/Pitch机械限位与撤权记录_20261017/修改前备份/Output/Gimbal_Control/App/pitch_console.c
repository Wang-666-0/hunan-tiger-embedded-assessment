#include "pitch_console.h"
#include "pitch_backend.h"
#include "app_config.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

/* Low-priority formatting only. CHECK sends CC; READY never sends FC or FD. */
static uint8_t pending_operation; /* 1=CHECK, 2=READY */
static uint32_t operation_ms, query_ms, query_rx;
#define PITCH_PREPARATION_TIMEOUT_MS 1000U

void PitchConsole_Init(void)
{ pending_operation = 0U; operation_ms = query_ms = query_rx = 0U; }
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
    } else if (command.arm != RC_HIGH || !app.logic.safety.operator_armed) { return "NOT_ARMED"; }
    return "OK";
}

static uint8_t Summary(const char *kind, const char *query_result, char *response, size_t capacity)
{
    PitchBackendState pitch;
    CanBusState bus;
    int length;
    PitchBackend_GetState(&pitch); CanBus_GetState(&bus);
    length = snprintf(response, capacity,
        "P|%s|%s|QUERY|%s|CTRL|CASCADE|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|ENABLING|%u|STATUS|%u|FAULT|%lu|AGE_MS|%lu|RX|%lu|STOP_PENDING|%u|COORD_CONFIRMED|%u|FEEDBACK_DIVISOR|%u|RAW_RAD|%.6f|RAW_RAD_S|%.5f|ANGLE|%.3f|TARGET|%.3f|VREF_DPS|%.2f|VEL_DPS|%.2f|TORQUE_NM|%.5f|FB_NM|%.5f|PKP|%.6g|PKI|%.6g|PKD|%.6g|VKP|%.6g|VKI|%.6g|VKD|%.6g|OUTPUT_LIMITS|0|PROTOCOL_NM|10|LIMIT_DEG|50|ZERO_SET|%u|D_TEMP|%u|M_TEMP|%u|CAN_EPOCH|%lu|REMOTE|%s|TELEM_MS|%u|BAUD|%lu|READ_ONLY_MONITOR|%u|SUPPORT_REQUIRED|1\r\n",
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
        (double)pitch.controller.cascade.speed.kd, (unsigned)pitch.zero_set,
        (unsigned)pitch.feedback.temp_mos, (unsigned)pitch.feedback.temp_rotor,
        (unsigned long)bus.fault_epoch, RemoteReason(1U), (unsigned)APP_TELEMETRY_PERIOD_MS,
        (unsigned long)APP_CONSOLE_BAUDRATE, (unsigned)pitch.monitoring);
    return (uint8_t)(length > 0 && (size_t)length < capacity);
}

static uint8_t StepReport(const PitchStep *value, char *response, size_t capacity)
{
    static const char *phases[] = { "IDLE", "RUNNING", "DONE", "ABORTED" };
    static const char *reasons[] = { "NONE", "STOP", "FAULT", "PID_CHANGE", "TARGET_CHANGE", "MANUAL", "GAP", "DATA" };
    uint32_t rise = value->t10_ms != PITCH_STEP_UNREACHED && value->t90_ms != PITCH_STEP_UNREACHED ?
        value->t90_ms - value->t10_ms : PITCH_STEP_UNREACHED;
    int length = snprintf(response, capacity,
        "P|STEP|ID|%lu|STATE|%s|REASON|%s|VALID|%u|START|%.3f|TARGET|%.3f|WINDOW_MS|%lu|RESP_MS|%ld|T90_MS|%ld|RISE_MS|%ld|SETTLE_MS|%ld|OS_DEG|%.3f|OS_PCT|%.3f|SSE_DEG|%.3f|SSE_VALID|%u|PASS|%u|FAULT|%lu|VMAX_DPS|%.2f|TMAX_NM|%.5f|N|%lu|GAP_MS|%lu|SAT_N|%lu|CMD_N|%lu|FIRST_NM|%.5f|CTRL|CASCADE|PKP|%.6g|PKI|%.6g|PKD|%.6g|VKP|%.6g|VKI|%.6g|VKD|%.6g|OUTPUT_LIMITS|0|RESP_LIMIT_MS|300|RESPONSE|FIRST_IN_BAND\r\n",
        (unsigned long)value->id, value->phase <= PITCH_STEP_ABORTED ? phases[value->phase] : "UNKNOWN",
        value->reason <= PITCH_STEP_DATA ? reasons[value->reason] : "UNKNOWN",
        (unsigned)(value->phase == PITCH_STEP_DONE), (double)value->start_deg, (double)value->target_deg,
        (unsigned long)value->elapsed_ms, (long)(int32_t)value->response_ms, (long)(int32_t)value->t90_ms,
        (long)(int32_t)rise, (long)(int32_t)value->settle_ms, (double)value->overshoot_deg,
        (double)value->overshoot_pct, (double)value->steady_error_deg, (unsigned)value->sse_valid,
        (unsigned)value->pass, (unsigned long)value->faults, (double)value->peak_velocity_dps,
        (double)value->peak_torque_nm, (unsigned long)value->samples, (unsigned long)value->max_gap_ms,
        (unsigned long)value->limited_samples, (unsigned long)value->command_samples,
        (double)value->first_command_nm, (double)value->kp, (double)value->ki, (double)value->kd,
        (double)value->vkp, (double)value->vki, (double)value->vkd);
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
                        "P|OK|READY|ZERO_SET|1|REQUESTED|1|ACTIVE|0|FC_SENT|0|FD_SENT|0|NEXT|NEW_CH5_OFF_THEN_MANUAL_OR_TEST|SUPPORT_REQUIRED|1\r\n");
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
        static const char *results[] = { "NONE", "STARTED", "RETURN_ZERO", "START_GUARD" };
        (void)snprintf(response, capacity, "P|REMOTE_STEP|SEQ|%lu|RESULT|%s|TARGET|%.1f\r\n",
            (unsigned long)event.sequence, event.result <= 3U ? results[event.result] : "UNKNOWN",
            (double)event.target_deg);
        return 1U;
    }
    PitchBackend_GetStep(&snapshot);
    if (!snapshot.pending || !StepReport(&snapshot, response, capacity)) { return 0U; }
    if (!PitchBackend_TakeStepReport(&report)) { return 0U; }
    if (report.id != snapshot.id) { (void)StepReport(&report, response, capacity); }
    return 1U;
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
    float *values[3] = { kp, ki, kd };
    char *end;
    uint32_t i;
    for (i = 0U; i < 3U; ++i) {
        *values[i] = strtof(text, &end);
        if (end == text || !(*values[i] >= -FLT_MAX && *values[i] <= FLT_MAX)) { return 0U; }
        if (i == 2U) { return (uint8_t)(*end == '\0'); }
        if (*end != ' ') { return 0U; }
        text = end + 1;
    }
    return 0U;
}

uint8_t PitchConsole_Handle(const char *line, char *response, size_t capacity)
{
    PitchBackendState pitch;
    uint8_t accepted = 0U;
    float value = 0.0f;
    if (!line || !response || !capacity || strncmp(line, "PITCH ", 6U) != 0) { return 0U; }
    if (strcmp(line, "PITCH OFF") == 0) {
        taskENTER_CRITICAL(); pending_operation = 0U;
        (void)PitchBackend_Request(0U); AppTasks_RequestStop(); taskEXIT_CRITICAL();
        (void)snprintf(response, capacity,
            "P|OK|OFF_REQUESTED|SUPPORT_REQUIRED|1|DISABLE_IS_NOT_BRAKE|1\r\n"); return 1U;
    }
    if (strcmp(line, "PITCH INFO") == 0 || strcmp(line, "PITCH STATUS") == 0) {
        (void)Summary("STATUS", "SNAPSHOT_ONLY", response, capacity); return 1U;
    }
    if (strcmp(line, "PITCH RESULT") == 0) {
        PitchStep snapshot; PitchBackend_GetStep(&snapshot);
        (void)StepReport(&snapshot, response, capacity); return 1U;
    }
    if (strcmp(line, "PITCH PID") == 0) {
        PitchBackend_GetState(&pitch);
        (void)snprintf(response, capacity,
            "P|PID|POS|%.6g,%.6g,%.6g|VEL|%.6g,%.6g,%.6g|OUTPUT_LIMITS|0|RAM_ONLY|1|INITIAL_NOT_TUNED|1\r\n",
            (double)pitch.controller.cascade.position.kp, (double)pitch.controller.cascade.position.ki,
            (double)pitch.controller.cascade.position.kd, (double)pitch.controller.cascade.speed.kp,
            (double)pitch.controller.cascade.speed.ki, (double)pitch.controller.cascade.speed.kd);
        return 1U;
    }
    if (pending_operation) {
        (void)snprintf(response, capacity, "P|BUSY|%s|USE_PITCH_OFF_TO_CANCEL\r\n",
            pending_operation == 1U ? "CHECK" : "READY"); return 1U;
    }
    if (strcmp(line, "PITCH CHECK") == 0 || strcmp(line, "PITCH READY") == 0) {
        const char *reason = "QUERY_UNAVAILABLE_CAN_OR_MAILBOX";
        uint8_t ready = (uint8_t)(strcmp(line, "PITCH READY") == 0);
        taskENTER_CRITICAL(); PitchBackend_GetState(&pitch);
        if (ready) {
            reason = RemoteReason(1U);
            if (strcmp(reason, "OK") == 0 && !pitch.coord_confirmed) { reason = "COORD_UNCONFIRMED_USE_PITCH_COORD_1_OR_7"; }
            if (strcmp(reason, "OK") == 0 &&
                (pitch.requested || pitch.active || pitch.enabling || pitch.stop_pending)) { reason = "OFF_REQUIRED_SUPPORT_LOAD"; }
        }
        if ((!ready || strcmp(reason, "OK") == 0) && PitchBackend_QueryFeedback()) {
            pending_operation = ready ? 2U : 1U; query_rx = pitch.rx_count;
            operation_ms = query_ms = HAL_GetTick(); accepted = 1U;
        } else if (ready && strcmp(reason, "OK") == 0) { reason = "QUERY_UNAVAILABLE_CAN_OR_MAILBOX"; }
        taskEXIT_CRITICAL();
        if (accepted) {
            (void)snprintf(response, capacity, "P|BUSY|%s|QUERY|CC|FC_SENT|0|FD_SENT|0|READ_ONLY_MONITOR|1\r\n", ready ? "READY" : "CHECK");
        } else { (void)snprintf(response, capacity, "P|ERROR|%s|%s\r\n", ready ? "READY" : "CHECK", reason); }
        return 1U;
    }
    if (strncmp(line, "PITCH COORD ", 12U) == 0) {
        uint8_t parsed = (uint8_t)(Number(line + 12U, &value) && (value == 1.0f || value == 7.0f));
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
        parsed = (uint8_t)(strcmp(line, "PITCH STEP") == 0 || Number(line + 11U, &value));
        taskENTER_CRITICAL();
        if (parsed &&
            strcmp(RemoteReason(0U), "OK") == 0) { accepted = PitchBackend_StartStep(value); }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "P|OK|STEP_STARTED|TARGET|%.1f|WINDOW_MS|5000|HOLD|1\r\n" :
            "P|ERROR|STEP_REQUIRES_ARMED_READY_STATIONARY_ZERO_AND_PLUS_MINUS_30\r\n", (double)value); return 1U;
    }
    if (strncmp(line, "PITCH TARGET ", 13U) == 0) {
        uint8_t parsed = Number(line + 13U, &value);
        taskENTER_CRITICAL();
        if (parsed && strcmp(RemoteReason(0U), "OK") == 0) {
            accepted = PitchBackend_SetTarget(value);
        }
        taskEXIT_CRITICAL();
        (void)snprintf(response, capacity, accepted ? "P|OK|TARGET|%.3f|HOLD|1\r\n" :
            "P|ERROR|TARGET_REQUIRES_ARMED_ACTIVE_AND_WITHIN_50\r\n", (double)value); return 1U;
    }
    if (strncmp(line, "PITCH POS ", 10U) == 0 || strncmp(line, "PITCH VEL ", 10U) == 0) {
        float kp, ki, kd;
        char loop = line[6] == 'P' ? 'P' : 'V';
        if (PidNumbers(line + 10U, &kp, &ki, &kd) && !AppConsole_IsDiagnosticMode()) {
            accepted = PitchBackend_SetLoopPid(loop, kp, ki, kd);
        }
        (void)snprintf(response, capacity, accepted ? "P|OK|%s\r\n" : "P|ERROR|PID_RANGE_OR_FORMAT\r\n",
            loop == 'P' ? "POS" : "VEL"); return 1U;
    }
    if (strlen(line) >= 10U && (line[6] == 'P' || line[6] == 'V') && line[7] == 'K' &&
        (line[8] == 'P' || line[8] == 'I' || line[8] == 'D') && line[9] == ' ') {
        if (Number(line + 10U, &value) && !AppConsole_IsDiagnosticMode()) {
            accepted = PitchBackend_SetLoopParameter(line[6], line[8], value);
        }
        (void)snprintf(response, capacity, accepted ? "P|OK|PARAMETER\r\n" : "P|ERROR|PID_RANGE_OR_FORMAT\r\n"); return 1U;
    }
    if (strncmp(line, "PITCH VMAX", 10U) == 0) {
        (void)snprintf(response, capacity, "P|ERROR|OUTPUT_LIMITS_DISABLED\r\n"); return 1U;
    }
    (void)snprintf(response, capacity, "P|ERROR|UNKNOWN_COMMAND_USE_PITCH_STATUS_OR_PID\r\n"); return 1U;
}
