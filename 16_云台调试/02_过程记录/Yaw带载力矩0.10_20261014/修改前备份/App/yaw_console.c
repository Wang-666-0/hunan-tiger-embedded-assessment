#include "yaw_console.h"
#include "yaw_backend.h"
#include "app_config.h"
#include "app_tasks.h"
#include "app_console.h"
#include "remote_input.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

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
        "%s|REMOTE|%s|BLOCKS|%lu|LAST_BLOCKS|%lu|SAMPLES|%u|HOLD_MS|%lu|VEL_DPS|%.2f|D_TEMP|%d|M_TEMP|%d|MONITOR|%u|FD_ATTEMPTED|%u|FD_AGE_MS|%lu|FD_RESULT|%u|FD_CANCEL|%u|MAILBOX|%lu|CAN_EPOCH|%lu|CLEAR_RESULT|%u|CH|%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        prefix, remote_reason, (unsigned long)check.blocks, (unsigned long)check.last_blocks,
        (unsigned)check.samples, (unsigned long)check.hold_ms, (double)yaw.controller.velocity_dps,
        (int)yaw.feedback.driver_temp_c, (int)yaw.feedback.motor_temp_c, (unsigned)check.monitoring,
        (unsigned)check.fd_sent, (unsigned long)check.fd_age_ms, (unsigned)check.fd_result,
        (unsigned)check.fd_cancel, (unsigned long)check.mailbox_pending, (unsigned long)check.can_epoch,
        (unsigned)check.clear_result, (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3], (unsigned)remote.channels[4],
        (unsigned)remote.channels[5], (unsigned)remote.channels[6], (unsigned)remote.channels[7]);
}

static uint8_t Number(const char *text, float *value)
{
    char *end;
    float parsed = strtof(text, &end);
    if (end == text || *end || !(parsed >= -FLT_MAX && parsed <= FLT_MAX)) { return 0U; }
    *value = parsed; return 1U;
}

uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity)
{
    YawBackendState yaw;
    uint8_t accepted = 0U;
    char parameter = 0;
    const char *number = 0;
    float value;
    if (!line || strncmp(line, "YAW ", 4U) != 0) { return 0U; }
    if (strcmp(line, "YAW CHECK") == 0) {
        ClearReport("Y|CHECK", 0, response, capacity); return 1U;
    }
    if (strcmp(line, "YAW STATUS") == 0 || strcmp(line, "YAW INFO") == 0) {
        YawBackend_GetState(&yaw);
        (void)snprintf(response, capacity,
            "Y|STATUS|%s|ONLINE|%u|ACTIVE|%u|REQUESTED|%u|STATUS|%u|FAULT|%lu|AGE_MS|%lu|STOP_PENDING|%u|ANGLE|%.3f|TARGET|%.3f|KP|%.4f|KI|%.4f|KD|%.4f|RX|%lu\r\n",
            APP_YAW_VERSION, (unsigned)yaw.online, (unsigned)yaw.active,
            (unsigned)yaw.requested, (unsigned)yaw.feedback.status,
            (unsigned long)yaw.controller.faults, (unsigned long)yaw.feedback_age_ms,
            (unsigned)yaw.stop_pending, (double)yaw.controller.angle_deg,
            (double)yaw.controller.target_deg, (double)yaw.controller.pid.kp,
            (double)yaw.controller.pid.ki, (double)yaw.controller.pid.kd, (unsigned long)yaw.rx_count);
        return 1U;
    }
    if (strcmp(line, "YAW OFF") == 0) {
        taskENTER_CRITICAL();
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
    if (number && Number(number, &value) && !AppConsole_IsDiagnosticMode()) {
        accepted = YawBackend_SetParameter(parameter, value);
    }
    (void)snprintf(response, capacity, accepted ? "Y|OK|PARAMETER\r\n" : "Y|ERROR|PARAMETER_OR_STATE\r\n");
    return 1U;
}
