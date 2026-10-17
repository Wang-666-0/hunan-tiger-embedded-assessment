#include "app_console.h"
#include "gimbal_startup.h"
#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#include "app_diagnostics.h"
#include "dm_diag.h"
#include "vesc_diag.h"
#if APP_YAW_ENABLED
#include "yaw_console.h"
#include "yaw_backend.h"
#endif
#if APP_PITCH_ENABLED
#include "pitch_console.h"
#include "pitch_backend.h"
#endif
#include "usart.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

typedef struct { uint32_t received_ms, epoch; uint8_t byte; } ConsoleByte;
static QueueHandle_t rx_queue;
static volatile uint32_t fault_epoch;
static volatile uint8_t needs_rearm;
#if APP_CONSOLE_RX_DMA_ENABLED
static uint8_t rx_dma[APP_CONSOLE_RX_DMA_BYTES];
static volatile uint32_t rx_bytes_seen, rx_bursts, rx_uart_errors, rx_queue_drops, tx_errors, last_uart_error;
#else
static uint8_t rx_byte;
#endif
static char line[64], telemetry[1024];
static uint32_t observed_epoch, last_byte_ms, telemetry_ms;
static uint8_t line_length, discard_line;
static uint8_t diagnostic_mode, status_pending, status_index;
static uint8_t telemetry_enabled = 1U;
static uint32_t reported_remote_epoch;
#if APP_PITCH_ENABLED
static uint8_t pitch_view;
#endif

static void TransmitFailed(void)
{
    taskENTER_CRITICAL();
#if APP_CONSOLE_RX_DMA_ENABLED
    tx_errors++;
#endif
    fault_epoch++;
    taskEXIT_CRITICAL();
    AppTasks_RequestStop();
}

static void SendText(const char *message)
{
    /* 只在低优先级Console任务调用。CAN接收、遥控保护继续运行。 */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)message, (uint16_t)strlen(message), APP_YAW_ENABLED ? 120U : 60U) != HAL_OK) {
        TransmitFailed();
    }
}

static void PublishRemoteRx(uint8_t only_fault)
{
    RemoteInput_State remote;
    uint32_t age;
    RemoteInput_GetState(&remote);
    if (only_fault && remote.fault_epoch == reported_remote_epoch) { return; }
    reported_remote_epoch = remote.fault_epoch;
    age = remote.has_frame ? HAL_GetTick() - remote.received_ms : 0xFFFFFFFFUL;
    (void)snprintf(telemetry, sizeof(telemetry),
        "R|RX|EPOCH|%lu|LAST|0x%08lX|RX_ERR|%lu|DROP|%lu|REARM_ERR|%lu|REARM_PENDING|%u|RX_BYTES|%lu|BURSTS|%lu|VALID|%lu|BAD|%lu|AGE_MS|%lu|RX_DMA|%u\r\n",
        (unsigned long)remote.fault_epoch, (unsigned long)remote.last_uart_error,
        (unsigned long)remote.rx_errors, (unsigned long)remote.rx_drops,
        (unsigned long)remote.rearm_errors, (unsigned)remote.needs_rearm,
        (unsigned long)remote.rx_bytes, (unsigned long)remote.rx_bursts,
        (unsigned long)remote.valid_frames, (unsigned long)remote.bad_frames,
        (unsigned long)age, (unsigned)APP_REMOTE_RX_DMA_ENABLED);
    SendText(telemetry);
}

static void HexBytes(const uint8_t *data, uint8_t length, char *out)
{
    static const char digits[] = "0123456789ABCDEF";
    uint8_t i;
    for (i = 0U; i < length; ++i) {
        out[i * 2U] = digits[data[i] >> 4];
        out[i * 2U + 1U] = digits[data[i] & 15U];
    }
    out[length * 2U] = '\0';
}

static uint8_t ReadNumber(const char **cursor, uint8_t *out)
{
    uint32_t value = 0U;
    const char *p = *cursor;
    if (*p < '0' || *p > '9') { return 0U; }
    do {
        value = value * 10U + (uint32_t)(*p - '0');
        if (value > 255U) { return 0U; }
        p++;
    } while (*p >= '0' && *p <= '9');
    *cursor = p; *out = (uint8_t)value;
    return 1U;
}

#if APP_FLYWHEEL_JOG_ENABLED
#if !APP_REMOTE_PROFILE_ASSESSMENT
static uint8_t CanSelectWheel(void)
{
    RemoteInput_State remote;
    AppState app;
    uint32_t i, age;
    RemoteInput_GetState(&remote);
    AppTasks_GetState(&app);
    age = (uint32_t)(HAL_GetTick() - remote.received_ms);
    if (!remote.has_frame || age >= REMOTE_TIMEOUT_MS ||
        app.logic.safety.operator_armed || app.logic.safety.output_allowed ||
        CanBus_OutputAllowed()) { return 0U; }
#if APP_REMOTE_PROFILE_LEGACY
    const uint8_t required[] = { RC_YAW_CHANNEL, RC_ARM_CHANNEL, RC_FAILSAFE_CHANNEL };
    for (i = 0U; i < sizeof(required); ++i) {
        uint16_t value = remote.channels[required[i]];
        if (value < 800U || value > 2200U) { return 0U; }
    }
#else
    for (i = 0U; i < 10U; ++i) {
        if (remote.channels[i] < 800U || remote.channels[i] > 2200U) { return 0U; }
    }
#endif
    return (uint8_t)(remote.channels[RC_YAW_CHANNEL] >= RC_CENTER - RC_DEADBAND &&
        remote.channels[RC_YAW_CHANNEL] <= RC_CENTER + RC_DEADBAND &&
        remote.channels[RC_ARM_CHANNEL] <= RC_SWITCH_LOW);
}
#endif

static void PublishWheelStatus(void)
{
    MotorJogState jog;
    MotorBackend_GetJogState(&jog);
    (void)snprintf(telemetry, sizeof(telemetry),
        "W|STATUS|%ld|%ld|%ld|%ld|%u|%ld|%u|%lu|%u|%u\r\n",
        (long)jog.health.feedback[0].erpm, (long)jog.health.feedback[1].erpm,
        (long)jog.health.feedback[0].motor_current_mA,
        (long)jog.health.feedback[1].motor_current_mA, (unsigned)jog.selection,
        (long)jog.requested_mA, (unsigned)jog.active, (unsigned long)jog.health.faults,
        (unsigned)jog.stop_pending, (unsigned)jog.rearm_ready);
    SendText(telemetry);
}

#if APP_FLYWHEEL_SPEED_ACTIVE
static void PublishOneSpeed(const FlywheelSpeed *speed, unsigned node)
{
    (void)snprintf(telemetry, sizeof(telemetry),
        "W|SPEED_NODE|%u|TARGET_ERPM|%ld|RAMP_ERPM|%ld|FILTER_ERPM|%ld|RPM_EST|%.1f|AT_LIMIT|%u|IN_BAND|%u|RUN_MS|%lu|PEAK_ERPM|%lu|BAND_MAX_MS|%lu|LIMIT_MS|%lu|COAST_MS|%lu|ZERO_MS|%lu|LOW_MS|%lu|T90_MS|%ld|BAND_FIRST_MS|%ld\r\n",
        node, (long)APP_FLYWHEEL_SPEED_TARGET_ERPM, (long)speed->target_erpm,
        (long)speed->filtered_erpm,
        (double)speed->filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (unsigned)speed->at_limit, (unsigned)speed->in_band,
        (unsigned long)speed->run_ms, (unsigned long)speed->peak_erpm,
        (unsigned long)speed->longest_band_ms, (unsigned long)speed->limited_ms,
        (unsigned long)speed->coast_ms, (unsigned long)speed->zero_current_ms,
        (unsigned long)speed->low_speed_ms,
        speed->rise90_seen ? (long)speed->rise90_ms : -1L,
        speed->band_first_seen ? (long)speed->band_first_ms : -1L);
    SendText(telemetry);
}

static void PublishWheelSpeed(void)
{
    MotorJogState jog;
    AppState app;
    MotorBackend_GetJogState(&jog);
    AppTasks_GetState(&app);
    (void)snprintf(telemetry, sizeof(telemetry),
        "W|SPEED|LAST_WHEEL|%u|ACTIVE|%u|STOP_REASON|%u|FAULT|%lu|SAFETY|%u|LEFT_MA|%ld|RIGHT_MA|%ld\r\n",
        (unsigned)jog.last_speed_selection, (unsigned)jog.active,
        (unsigned)jog.stop_reason, (unsigned long)jog.health.faults,
        (unsigned)app.logic.safety.reason,
        (long)jog.requested_left_mA, (long)jog.requested_right_mA);
    SendText(telemetry);
    PublishOneSpeed(&jog.speed, jog.last_speed_selection == FLYWHEEL_JOG_RIGHT ?
        APP_FLYWHEEL_RIGHT_ID : APP_FLYWHEEL_LEFT_ID);
    if (jog.last_speed_selection == FLYWHEEL_JOG_BOTH) {
        PublishOneSpeed(&jog.speed_right, APP_FLYWHEEL_RIGHT_ID);
    }
}
#endif

static void SelectWheel(FlywheelJogSelection selection)
{
    uint8_t accepted = 0U;
#if APP_REMOTE_PROFILE_ASSESSMENT
    if (selection != FLYWHEEL_JOG_OFF) {
        SendText("W|ERROR|REMOTE_CONTROL\r\n");
        return;
    }
#endif
    if (selection == FLYWHEEL_JOG_OFF) {
        taskENTER_CRITICAL();
        AppTasks_RequestStop();
        accepted = MotorBackend_SelectJog(FLYWHEEL_JOG_OFF);
        taskEXIT_CRITICAL();
    } else {
#if !APP_REMOTE_PROFILE_ASSESSMENT
        if (diagnostic_mode) { SendText("W|ERROR|DIAGNOSTIC\r\n"); return; }
        if (!CanSelectWheel()) { SendText("W|ERROR|UNSAFE\r\n"); return; }
        taskENTER_CRITICAL();
        /* 检查和选择之间不得让新遥控帧、故障或Control任务改变条件。
         * 选择后STOP建立新帧屏障，旧OFF/ON输入不能解锁本会话。 */
        accepted = CanSelectWheel() && MotorBackend_SelectJog(selection);
        if (accepted) { AppTasks_RequestStop(); }
        taskEXIT_CRITICAL();
#endif
    }
    if (!accepted) { SendText("W|ERROR|UNSAFE\r\n"); return; }
    SendText(selection == FLYWHEEL_JOG_LEFT ? "W|OK|LEFT\r\n" :
        selection == FLYWHEEL_JOG_RIGHT ? "W|OK|RIGHT\r\n" :
        selection == FLYWHEEL_JOG_BOTH ? "W|OK|BOTH\r\n" : "W|OK|OFF\r\n");
}
#endif

#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
static void PublishStartup(uint8_t event_only)
{
    static const char *phases[] = { "WAIT_OFF", "PREPARING", "READY", "BLOCKED", "MANUAL" };
    static const char *reasons[] = { "NONE", "REMOTE", "CAN", "FAULT", "REFERENCE_LOST", "DIAGNOSTIC", "STOP", "TIMEOUT" };
    GimbalStartupState state;
    if (event_only) { if (!GimbalStartup_TakeEvent(&state)) { return; } }
    else { GimbalStartup_GetState(&state); }
    (void)snprintf(telemetry, sizeof(telemetry),
        "G|%s|PHASE|%s|REASON|%s|READY|%u|REFERENCE|%u|SEQ|%lu\r\n",
        event_only ? "AUTO" : "STATE", state.phase <= GIMBAL_STARTUP_MANUAL ? phases[state.phase] : "UNKNOWN",
        state.reason <= GIMBAL_STARTUP_TIMEOUT ? reasons[state.reason] : "UNKNOWN", (unsigned)state.ready,
        (unsigned)state.reference_latched, (unsigned long)state.sequence);
    SendText(telemetry);
}
static uint8_t StartupRequest(void)
{
    RemoteInput_State remote;
    RemoteFrame frame;
    RemoteInput_GetState(&remote); memset(&frame, 0, sizeof(frame));
    memcpy(frame.channels, remote.channels, sizeof(frame.channels));
    frame.has_frame = remote.has_frame; frame.received_ms = remote.received_ms;
    frame.sequence = remote.valid_frames;
    if (!GimbalStartup_Request(&frame, HAL_GetTick(), diagnostic_mode)) { return 0U; }
    AppTasks_RequestStop(); return 1U;
}
#endif

static void HandleLine(void)
{
    const char *cursor;
    uint8_t node, field;
    DiagKind kind;
    CanFrame check;
    if (line[0] == '\0') { return; }
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
    if (strcmp(line, "GIMBAL") == 0) { PublishStartup(0U); return; }
    if (strcmp(line, "GIMBAL READY") == 0) {
        SendText(StartupRequest() ? "G|BUSY|READY|BASELINE|CURRENT_SUPPORTED_HORIZONTAL\r\n" :
            "G|ERROR|READY_REQUIRES_FRESH_SAFE_OFF_AND_CAN\r\n"); return;
    }
    if (strcmp(line, "GIMBAL HELP") == 0) {
        SendText("G|HELP|BOOT|SAFE_OFF_HORIZONTAL_THEN_CH5_MIDDLE|REZERO|GIMBAL READY|STEP|YAW STEP;PITCH STEP|RETURN|YAW HOME;PITCH HOME|RESULT|YAW RESULT;PITCH RESULT\r\n"); return;
    }
    /* Explicit legacy preparation/stop takes ownership; auto startup must not overwrite it. */
    if (strcmp(line, "STOP") == 0 || strcmp(line, "DIAG ON") == 0 ||
        strcmp(line, "YAW READY") == 0 || strcmp(line, "YAW ZERO") == 0 ||
        strcmp(line, "YAW ON") == 0 || strcmp(line, "YAW OFF") == 0 || strcmp(line, "YAW CLEAR") == 0 ||
        strcmp(line, "PITCH READY") == 0 || strcmp(line, "PITCH ZERO") == 0 || strcmp(line, "PITCH OFF") == 0 ||
        strcmp(line, "PITCH GHORIZ") == 0 || strncmp(line, "PITCH GOFF ", 11U) == 0 ||
        strncmp(line, "PITCH COORD ", 12U) == 0) { GimbalStartup_ManualOverride(); }
#endif
    if (strcmp(line, "HELP") == 0) {
        SendText("C|HELP|AXIS|YAW;PITCH|AUTO|GIMBAL;GIMBAL READY|READ|axis;axis PID;axis RESULT|ACTION|axis READY;axis OFF|VIEW|axis VIEW|TELEM|TELEM OFF;TELEM ON|DETAIL|axis HELP\r\n"); return;
    }
    if (strcmp(line, "TELEM OFF") == 0 || strcmp(line, "TELEM ON") == 0) {
        telemetry_enabled = (uint8_t)(strcmp(line, "TELEM ON") == 0);
        SendText(telemetry_enabled ? "C|OK|TELEM|ON\r\n" : "C|OK|TELEM|OFF\r\n"); return;
    }
#if APP_PITCH_ENABLED
    if (strcmp(line, "TELEM PITCH") == 0 || strcmp(line, "PITCH VIEW") == 0) {
        pitch_view = 1U;
        SendText("P|VIEW|PITCH|GROUP|pitch|COLS|58|PID_I|7_12|GFF_I|52_57\r\n"); return;
    }
    if (strcmp(line, "TELEM YAW") == 0 || strcmp(line, "YAW VIEW") == 0) {
        pitch_view = 0U;
        SendText(strcmp(line, "YAW VIEW") == 0 ? "Y|VIEW|YAW|GROUP|gimbal|COLS|70|PID_I|7_12|FF_I|48_59|BAND05_I|60_62\r\n" :
            "P|VIEW|YAW|GROUP|gimbal|COLS|70|PID_I|7_12|FF_I|48_59|BAND05_I|60_62\r\n"); return;
    }
    if (PitchConsole_Handle(line, telemetry, sizeof(telemetry))) { SendText(telemetry); return; }
#endif
#if APP_YAW_ENABLED
#if !APP_PITCH_ENABLED
    if (strcmp(line, "YAW VIEW") == 0 || strcmp(line, "TELEM YAW") == 0) {
        SendText("Y|VIEW|YAW|GROUP|gimbal|COLS|70|PID_I|7_12|FF_I|48_59|BAND05_I|60_62\r\n"); return;
    }
#endif
    if (YawConsole_Handle(line, telemetry, sizeof(telemetry))) { SendText(telemetry); return; }
#endif
    if (strcmp(line, "REMOTE RX") == 0) { PublishRemoteRx(0U); return; }
    if (strcmp(line, "REMOTE INFO") == 0) {
        SendText(APP_REMOTE_PROFILE_ASSESSMENT ?
            "R|INFO|ASSESSMENT_CH5_CH8_V1|MODE_CH|5|WHEEL_CH|6|STEP_CH|7|TRIGGER_CH|8|FAILSAFE_CH|8|FAILSAFE|LOW|WHEEL|AUTO_BOTH|OTHER_DRIVERS|0"
#if APP_YAW_ENABLED
            "|YAW_DRIVER|1\r\n" :
#else
            "\r\n" :
#endif
            "R|INFO|HISTORICAL_PROFILE\r\n");
        return;
    }
    if (strcmp(line, "WHEEL INFO") == 0) {
        /* 只读编译配置，不选轮、不发CAN、不改变诊断/遥控许可。 */
        (void)snprintf(telemetry, sizeof(telemetry),
            "W|INFO|%s|LEFT_MA|%d|RIGHT_MA|%d|LEFT_SIGN|%d|RIGHT_SIGN|%d|MS|%u|ERPM|%ld|ENABLED|%u|HOLD_TO_RUN|%u|COAST_ERPM|%ld|RESUME_ERPM|%ld|MODE|%s|POLE_PAIRS_ASSUMED|%u|TARGET_RPM|%u|TARGET_ERPM|%ld|BOTH|%u|FB_MAX_MA|%ld|RAMP_ERPMS|%ld|BAND_ERPM|%ld\r\n",
            APP_FIRMWARE_VERSION, APP_FLYWHEEL_JOG_LEFT_MAX_MA, APP_FLYWHEEL_JOG_RIGHT_MAX_MA,
            APP_FLYWHEEL_LEFT_CURRENT_SIGN, APP_FLYWHEEL_RIGHT_CURRENT_SIGN,
            (unsigned)FLYWHEEL_JOG_MAX_DURATION_MS, (long)FLYWHEEL_JOG_MAX_ERPM,
            (unsigned)APP_FLYWHEEL_JOG_ENABLED,
            (unsigned)(FLYWHEEL_JOG_MAX_DURATION_MS == 0U),
            (long)(APP_FLYWHEEL_SPEED_ACTIVE ? APP_FLYWHEEL_SPEED_COAST_ERPM :
                (FLYWHEEL_JOG_MAX_DURATION_MS == 0U ? FLYWHEEL_OBSERVE_COAST_ERPM : 0)),
            (long)(APP_FLYWHEEL_SPEED_ACTIVE ? APP_FLYWHEEL_SPEED_RESUME_ERPM :
                (FLYWHEEL_JOG_MAX_DURATION_MS == 0U ? FLYWHEEL_OBSERVE_RESUME_ERPM : 0)),
            !APP_FLYWHEEL_JOG_ENABLED ? "OFF" : (APP_FLYWHEEL_SPEED_ACTIVE ? "CURRENT_PI" :
                (FLYWHEEL_JOG_MAX_DURATION_MS == 0U ? "OBSERVE" : "PULSE")),
            (unsigned)APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
            (unsigned)(APP_FLYWHEEL_SPEED_ACTIVE ? APP_FLYWHEEL_SPEED_TARGET_RPM : 0),
            (long)(APP_FLYWHEEL_SPEED_ACTIVE ? APP_FLYWHEEL_SPEED_TARGET_ERPM : 0),
            (unsigned)APP_FLYWHEEL_SPEED_ACTIVE, (long)FLYWHEEL_JOG_MAX_FEEDBACK_CURRENT_MA,
            (long)(APP_FLYWHEEL_SPEED_ACTIVE ? APP_FLYWHEEL_SPEED_RAMP_ERPMS : 0),
            (long)(APP_FLYWHEEL_SPEED_ACTIVE ? APP_FLYWHEEL_SPEED_BAND_ERPM : 0));
        SendText(telemetry);
        return;
    }
    if (strcmp(line, "STOP") == 0) {
#if APP_PITCH_ENABLED
        PitchConsole_CancelPreparation();
#endif
#if APP_YAW_ENABLED
        YawConsole_CancelPreparation();
#endif
        AppTasks_RequestStop();
        status_pending = 0U;
        if (diagnostic_mode) { SendText("D|ERROR|CANCELLED\r\n"); }
        return;
    }
    if (strcmp(line, "DIAG ON") == 0) {
#if APP_PITCH_ENABLED
        PitchConsole_CancelPreparation();
#endif
#if APP_YAW_ENABLED
        YawConsole_CancelPreparation();
#endif
#if APP_FLYWHEEL_JOG_ENABLED
        taskENTER_CRITICAL();
        AppTasks_RequestStop();
        (void)MotorBackend_SelectJog(FLYWHEEL_JOG_OFF);
        taskEXIT_CRITICAL();
#endif
        diagnostic_mode = 1U;
        SendText("D|HELLO|GIMBAL_DIAG_1\r\n");
        return;
    }
#if APP_FLYWHEEL_JOG_ENABLED
    if (strcmp(line, "WHEEL LEFT") == 0) { SelectWheel(FLYWHEEL_JOG_LEFT); return; }
    if (strcmp(line, "WHEEL RIGHT") == 0) { SelectWheel(FLYWHEEL_JOG_RIGHT); return; }
    if (strcmp(line, "WHEEL OFF") == 0) { SelectWheel(FLYWHEEL_JOG_OFF); return; }
    if (strcmp(line, "WHEEL STATUS") == 0) { PublishWheelStatus(); return; }
#if APP_FLYWHEEL_SPEED_ACTIVE
    if (strcmp(line, "WHEEL SPEED") == 0) { PublishWheelSpeed(); return; }
    if (strcmp(line, "WHEEL BOTH") == 0) { SelectWheel(FLYWHEEL_JOG_BOTH); return; }
#endif
#endif
    if (strcmp(line, "DIAG OFF") == 0) {
#if APP_PITCH_ENABLED
        PitchConsole_CancelPreparation();
#endif
#if APP_YAW_ENABLED
        YawConsole_CancelPreparation();
#endif
        AppDiagnostics_Cancel();
        status_pending = 0U;
        diagnostic_mode = 0U;
        return;
    }
    if (!diagnostic_mode) { return; }
    if (strcmp(line, "STATUS") == 0) { status_pending = 1U; status_index = 0U; return; }
    if (strncmp(line, "READ DM ", 8U) == 0) {
        cursor = line + 8U; kind = DIAG_DM; node = APP_PITCH_DEVICE_ID;
        if (!ReadNumber(&cursor, &field) || *cursor || !DmDiag_BuildRead(field, &check)) {
            SendText("D|ERROR|BAD_COMMAND\r\n"); return;
        }
    }
    else if (strncmp(line, "READ VESC ", 10U) == 0) {
        cursor = line + 10U; kind = DIAG_VESC;
        if (!ReadNumber(&cursor, &node) || *cursor++ != ' ' ||
            !ReadNumber(&cursor, &field) || *cursor || !VescDiag_BuildRead(node, field, &check)) {
            SendText("D|ERROR|BAD_COMMAND\r\n"); return;
        }
    }
    else { SendText("D|ERROR|BAD_COMMAND\r\n"); return; }
    if (status_pending || !AppDiagnostics_Request(kind, node, field)) {
        SendText("D|ERROR|BUSY\r\n"); return;
    }
    if (kind == DIAG_DM) {
        (void)snprintf(telemetry, sizeof(telemetry), "D|BEGIN|DM|%u\r\n", (unsigned)field);
    }
    else {
        (void)snprintf(telemetry, sizeof(telemetry), "D|BEGIN|VESC|%u|%u\r\n", (unsigned)node, (unsigned)field);
    }
    SendText(telemetry);
}

static void PublishDiagnostics(uint32_t now)
{
    DiagEvent event;
    DiagPassiveFrame passive;
    RemoteInput_State remote;
    CanBusState bus;
    char hex[65];
    uint32_t i;
    static const char *errors[] = {"NONE", "TIMEOUT", "CAN_FAULT", "TX_FAILED", "BAD_REPLY", "CANCELLED"};
    for (i = 0U; i < 4U && AppDiagnostics_TakeEvent(&event); ++i) {
        if (event.event == DIAG_EVENT_ERROR) {
            (void)snprintf(telemetry, sizeof(telemetry), "D|ERROR|%s\r\n",
                event.error <= DIAG_ERROR_CANCELLED ? errors[event.error] : "BAD_REPLY");
        }
        else if (event.event == DIAG_EVENT_DATA) {
            HexBytes(event.data, event.length, hex);
            if (event.kind == DIAG_DM) {
                (void)snprintf(telemetry, sizeof(telemetry), "D|DM|%u|%s\r\n", (unsigned)event.field, hex);
            }
            else {
                (void)snprintf(telemetry, sizeof(telemetry), "D|VESC|%u|%u|%u|%u|%s\r\n",
                    (unsigned)event.node, (unsigned)event.field, (unsigned)event.total, (unsigned)event.offset, hex);
            }
        }
        else if (event.kind == DIAG_DM) {
            (void)snprintf(telemetry, sizeof(telemetry), "D|DONE|DM|%u\r\n", (unsigned)event.field);
        }
        else {
            (void)snprintf(telemetry, sizeof(telemetry), "D|DONE|VESC|%u|%u\r\n", (unsigned)event.node, (unsigned)event.field);
        }
        SendText(telemetry);
    }
    if (!status_pending) { return; }
    if (status_index == 0U) {
        CanBus_GetState(&bus); RemoteInput_GetState(&remote);
        (void)snprintf(telemetry, sizeof(telemetry),
            "D|STATUS|%lu|%lu|%lu|%lu|%lu|%lu|%u|%u|%u|%u|%u\r\n",
            (unsigned long)bus.rx_frames[0], (unsigned long)bus.rx_frames[1],
            (unsigned long)bus.errors[0], (unsigned long)bus.errors[1],
            (unsigned long)bus.dropped, (unsigned long)bus.fault_epoch,
            (unsigned)(remote.has_frame && (uint32_t)(now - remote.received_ms) < REMOTE_TIMEOUT_MS),
            (unsigned)remote.channels[RC_YAW_CHANNEL], (unsigned)remote.channels[RC_ARM_CHANNEL],
            (unsigned)remote.channels[RC_FAILSAFE_CHANNEL], (unsigned)CanBus_OutputAllowed());
        SendText(telemetry); status_index = 1U;
    }
    else if (status_index <= APP_DIAG_PASSIVE_SLOTS) {
        if (AppDiagnostics_GetPassive(status_index - 1U, &passive)) {
            HexBytes(passive.frame.data, passive.frame.dlc, hex);
            (void)snprintf(telemetry, sizeof(telemetry), "D|CAN|%u|%u|%lX|%u|%s|%lu|%lu\r\n",
                (unsigned)passive.frame.bus, (unsigned)passive.frame.extended,
                (unsigned long)passive.frame.id, (unsigned)passive.frame.dlc, hex,
                (unsigned long)passive.count, (unsigned long)(now - passive.frame.received_ms));
            SendText(telemetry);
        }
        status_index++;
    }
    else { SendText("D|DONE|STATUS\r\n"); status_pending = 0U; }
}

void AppConsole_CreateQueue(void)
{
    telemetry_enabled = 1U;
#if APP_PITCH_ENABLED
    PitchConsole_Init();
#endif
    rx_queue = xQueueCreate(APP_CONSOLE_RX_QUEUE_LENGTH, sizeof(ConsoleByte));
    if (rx_queue == 0) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "CONSOLE_BYTES");
}
static HAL_StatusTypeDef StartConsoleRx(void)
{
#if APP_CONSOLE_RX_DMA_ENABLED
    return ConsoleUart_StartReceive(rx_dma, sizeof(rx_dma));
#else
    return HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
#endif
}
void AppConsole_Start(void)
{ if (StartConsoleRx() != HAL_OK) { AppFault_Fatal(APP_FAULT_UART_START); } }
uint32_t AppConsole_GetFaultEpoch(void) { return fault_epoch; }
uint8_t AppConsole_IsDiagnosticMode(void) { return diagnostic_mode; }

/* 全工程只有这一份UART回调；UART4交给遥控模块，USART1交给控制台。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if !APP_CONSOLE_RX_DMA_ENABLED
    ConsoleByte item;
    BaseType_t woken = pdFALSE;
#endif
    if (huart == &huart4) { RemoteInput_RxCplt(huart); return; }
#if !APP_CONSOLE_RX_DMA_ENABLED
    if (huart != &huart1 || HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    item.byte = rx_byte;
    item.received_ms = HAL_GetTick();
    item.epoch = fault_epoch;
    if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) { fault_epoch++; }
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK) { fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
#endif
}

#if APP_CONSOLE_RX_DMA_ENABLED || APP_REMOTE_RX_DMA_ENABLED
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
#if APP_CONSOLE_RX_DMA_ENABLED
    ConsoleByte item;
    uint16_t i;
    BaseType_t woken = pdFALSE;
#endif
#if APP_REMOTE_RX_DMA_ENABLED
    if (huart == &huart4) { RemoteInput_RxEvent(huart, size); return; }
#endif
#if APP_CONSOLE_RX_DMA_ENABLED
    if (huart != &huart1 || HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    /* HT is disabled. Normal DMA IDLE/TC has already stopped reception.
     * Enqueue this stable bounded burst; parsing and TX remain in the task. */
    if (!size || size > sizeof(rx_dma)) { fault_epoch++; needs_rearm = 1U; return; }
    rx_bursts++; rx_bytes_seen += size;
    item.received_ms = HAL_GetTick(); item.epoch = fault_epoch;
    for (i = 0U; i < size; ++i) {
        item.byte = rx_dma[i];
        if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) {
            rx_queue_drops++; fault_epoch++; break;
        }
    }
    if (StartConsoleRx() != HAL_OK) { fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
#else
    (void)huart; (void)size;
#endif
}
#endif

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4) { RemoteInput_Error(huart); return; }
    if (huart != &huart1) { return; }
#if APP_CONSOLE_RX_DMA_ENABLED
    rx_uart_errors++; last_uart_error = HAL_UART_GetError(huart);
#endif
    fault_epoch++;
    (void)HAL_UART_AbortReceive(huart);
    needs_rearm = 1U;
}

#if APP_PITCH_ENABLED
static void PublishPitchTelemetry(uint32_t now)
{
    PitchBackendState pitch;
    PitchStep step;
    RemoteInput_State remote;
    AppState app;
    CanBusState bus;
    int length;
    taskENTER_CRITICAL();
    PitchBackend_GetState(&pitch); PitchBackend_GetStep(&step);
    taskEXIT_CRITICAL();
    RemoteInput_GetState(&remote); AppTasks_GetState(&app); CanBus_GetState(&bus);
    /* Pitch原I0~I51保持；I52~I57追加Kg、水平偏置、前馈、PID、速度积分、标定。Yaw仍48列。
     * 同时只发当前选择的一组，保持921600/5ms的线路预算。 */
    length = snprintf(telemetry, sizeof(telemetry),
        "pitch:%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g"
        ",%u,%u,%u,%u,%lu,%lu,%u,%u,%u,%u,%lu,%u,%ld,%ld,%.6g,%.6g,%.6g,%u,%u,%ld,%ld"
        ",%.6g,%.6g,%u,%lu,%u,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%u,%.6g,%.6g,%.6g,%.6g,%.6g,%u\r\n",
        (double)pitch.controller.angle_deg, (double)pitch.controller.target_deg,
        (double)pitch.controller.cascade.target_velocity_dps, (double)pitch.controller.cascade.velocity_dps,
        (double)pitch.controller.velocity_dps, (double)pitch.controller.torque_nm, (double)pitch.feedback.torque_nm,
        (double)pitch.controller.cascade.position.kp, (double)pitch.controller.cascade.position.ki,
        (double)pitch.controller.cascade.position.kd, (double)pitch.controller.cascade.speed.kp,
        (double)pitch.controller.cascade.speed.ki, (double)pitch.controller.cascade.speed.kd,
        (double)(pitch.feedback.position_rad * 57.295779513f / pitch.controller.feedback_divisor),
        (unsigned)pitch.online, (unsigned)pitch.active, (unsigned)pitch.requested, (unsigned)pitch.feedback.status,
        (unsigned long)pitch.controller.faults, (unsigned long)pitch.feedback_age_ms,
        (unsigned)pitch.stop_pending, (unsigned)pitch.zero_set,
        (unsigned)pitch.controller.cascade.torque_limited, (unsigned)pitch.coord_confirmed,
        (unsigned long)step.id, (unsigned)step.phase,
        (long)(int32_t)step.response_ms, (long)(int32_t)step.settle_ms,
        (double)step.overshoot_deg, (double)step.overshoot_pct, (double)step.steady_error_deg,
        (unsigned)step.sse_valid, (unsigned)step.pass, (long)(int32_t)step.t90_ms,
        (long)(int32_t)(step.t10_ms != PITCH_STEP_UNREACHED && step.t90_ms != PITCH_STEP_UNREACHED ?
            step.t90_ms - step.t10_ms : PITCH_STEP_UNREACHED),
        (double)pitch.feedback.position_rad, (double)pitch.feedback.velocity_rad_s,
        (unsigned)pitch.controller.feedback_divisor, (unsigned long)pitch.rx_count,
        (unsigned)(remote.has_frame && (uint32_t)(now - remote.received_ms) < REMOTE_TIMEOUT_MS),
        (unsigned)app.logic.safety.operator_armed,
        (unsigned)remote.channels[4], (unsigned)remote.channels[5], (unsigned)remote.channels[6],
        (unsigned)remote.channels[7], (unsigned long)bus.fault_epoch,
        (unsigned)pitch.feedback.temp_mos, (unsigned)pitch.feedback.temp_rotor,
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3],
        (double)pitch.controller.gravity_kg_nm, (double)pitch.controller.gravity_offset_deg,
        (double)pitch.controller.gravity_torque_nm, (double)pitch.controller.pid_torque_nm,
        (double)pitch.controller.cascade.speed_i, (unsigned)pitch.controller.gravity_calibrated);
    if (length > 0 && (size_t)length < sizeof(telemetry)) { SendText(telemetry); }
}
#endif

static void PublishYawTelemetry(uint32_t now)
{
    AppState app;
    CanBusState bus;
    RemoteInput_State remote;
#if !APP_YAW_ENABLED
    TargetRequest *request;
#endif
    uint32_t age;
    int length;
#if APP_FLYWHEEL_JOG_ENABLED
    MotorJogState jog;
#endif
#if APP_YAW_ENABLED
    YawBackendState yaw;
    YawStep step;
#endif
    AppTasks_GetState(&app);
    CanBus_GetState(&bus);
    RemoteInput_GetState(&remote);
#if APP_FLYWHEEL_JOG_ENABLED
    MotorBackend_GetJogState(&jog);
#endif
#if APP_YAW_ENABLED
    taskENTER_CRITICAL();
    YawBackend_GetState(&yaw); YawBackend_GetStep(&step);
    taskEXIT_CRITICAL();
#endif
    age = remote.has_frame ? (uint32_t)(now - remote.received_ms) : 0U;
#if APP_YAW_ENABLED
    /* 原I0~I47不变；追加前馈参数/积分/新±0.5指标和CAN毫秒戳，共70列；I64~I69为规划时间/活动/位置/速度/加速度/跟踪误差。
     * 前馈参数4位有效数字，诊断5位，减少5ms/921600线路开销。 */
    length = snprintf(telemetry, sizeof(telemetry),
        "gimbal:%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g"
        ",%u,%u,%u,%u,%lu,%lu,%u,%u,%u,%u,%lu,%u,%ld,%ld,%.6g,%.6g,%.6g,%u,%u,%ld,%ld"
        ",%u,%.6g,%.6g,%lu,%u,%u,%u,%u,%u,%u,%lu,%d,%d"
        ",%u,%.4g,%.4g,%.4g,%.4g,%.4g,%.4g,%.5g,%.5g,%.5g,%.5g,%u,%ld,%ld,%u,%lu"
        ",%lu,%u,%.5g,%.5g,%.5g,%.5g\r\n",
        (double)yaw.controller.angle_deg, (double)yaw.controller.target_deg,
        (double)yaw.controller.cascade.target_velocity_dps, (double)yaw.controller.cascade.velocity_dps,
        (double)yaw.controller.velocity_dps, (double)yaw.controller.torque_nm, (double)yaw.feedback.torque_nm,
        (double)yaw.controller.cascade.position.kp, (double)yaw.controller.cascade.position.ki,
        (double)yaw.controller.cascade.position.kd, (double)yaw.controller.cascade.speed.kp,
        (double)yaw.controller.cascade.speed.ki, (double)yaw.controller.cascade.speed.kd,
        (double)yaw.controller.cascade.velocity_limit_dps,
        (unsigned)yaw.online, (unsigned)yaw.active, (unsigned)yaw.requested, (unsigned)yaw.feedback.status,
        (unsigned long)yaw.controller.faults, (unsigned long)yaw.feedback_age_ms,
        (unsigned)yaw.stop_pending, (unsigned)yaw.zero_set,
        (unsigned)yaw.controller.cascade.torque_limited, (unsigned)yaw.controller.cascade.velocity_limited,
        (unsigned long)step.id, (unsigned)step.phase,
        (long)(int32_t)step.response_ms, (long)(int32_t)step.settle_ms,
        (double)step.overshoot_deg, (double)step.overshoot_pct, (double)step.steady_error_deg,
        (unsigned)step.sse_valid, (unsigned)step.pass, (long)(int32_t)step.t90_ms,
        (long)(int32_t)(step.t10_ms != YAW_STEP_UNREACHED && step.t90_ms != YAW_STEP_UNREACHED ?
            step.t90_ms - step.t10_ms : YAW_STEP_UNREACHED),
#if APP_FLYWHEEL_JOG_ENABLED && APP_FLYWHEEL_SPEED_ACTIVE
        (unsigned)jog.active, (double)jog.speed.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (double)jog.speed_right.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED, (unsigned long)jog.health.faults,
#else
        0U, 0.0, 0.0, 0UL,
#endif
        (unsigned)(remote.has_frame && age < REMOTE_TIMEOUT_MS), (unsigned)app.logic.safety.operator_armed,
        (unsigned)remote.channels[4], (unsigned)remote.channels[5], (unsigned)remote.channels[6],
        (unsigned)remote.channels[7], (unsigned long)bus.fault_epoch,
        (int)yaw.feedback.driver_temp_c, (int)yaw.feedback.motor_temp_c,
        (unsigned)yaw.controller.ff.enabled, (double)yaw.controller.ff.bias_nm,
        (double)yaw.controller.ff.angle_gain, (double)yaw.controller.ff.velocity_gain,
        (double)yaw.controller.ff.friction_nm, (double)yaw.controller.ff.friction_velocity_dps,
        (double)yaw.controller.ff.limit_nm, (double)yaw.controller.cascade.feedforward_nm,
        (double)yaw.controller.pid_torque_nm, (double)yaw.controller.cascade.speed_i,
        (double)yaw.controller.cascade.position_i, (unsigned)yaw.controller.cascade.feedforward_limited,
        (long)(int32_t)step.response05_ms, (long)(int32_t)step.settle05_ms, (unsigned)step.pass05,
        (unsigned long)yaw.controller.last_feedback_ms,
        (unsigned long)yaw.controller.shape.duration_ms, (unsigned)yaw.controller.shape.active,
        (double)yaw.controller.shape.position_deg, (double)yaw.controller.shape.velocity_dps,
        (double)yaw.controller.shape.acceleration_dps2,
        (double)(yaw.controller.shape.position_deg - yaw.controller.angle_deg));
#else
    request = &app.logic.targets.request;
    length = snprintf(telemetry, sizeof(telemetry),
        "gimbal:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%.2f,%.2f,%u,%u,%lu,%u,%.1f,%lu,%lu,%lu,%u,%lu,%lu"
#if APP_FLYWHEEL_JOG_ENABLED
        ",%ld,%ld,%ld,%ld,%u,%ld,%u,%lu,%u,%u"
#if APP_FLYWHEEL_SPEED_ACTIVE
        ",%ld,%ld,%.1f,%u,%u,%ld,%ld,%ld,%.1f,%u,%u,%u"
#endif
#endif
#if APP_YAW_ENABLED
        ",%.3f,%.3f,%.2f,%.5f,%.5f,%.4f,%.4f,%.4f,%u,%u,%u,%lu,%lu,%u,%u,%d,%d,%.6f,%lu"
#endif
        "\r\n",
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3],
        (unsigned)remote.channels[4], (unsigned)remote.channels[5],
        (unsigned)remote.channels[6], (unsigned)remote.channels[7],
        (unsigned)remote.channels[8], (unsigned)remote.channels[9],
        (unsigned)(remote.has_frame && age < REMOTE_TIMEOUT_MS),
        (unsigned)app.logic.safety.operator_armed, (unsigned)CanBus_OutputAllowed(),
        (unsigned)app.logic.safety.state, (unsigned)app.logic.safety.reason, (unsigned long)age,
        (double)request->yaw_rate_dps, (double)request->pitch_rate_dps,
        (unsigned)request->friction_requested, (unsigned)request->fire_mode,
        (unsigned long)request->shot_sequence, (unsigned)request->gimbal_mode,
        (double)request->step_delta_deg, (unsigned long)request->step_sequence,
        (unsigned long)bus.rx_frames[0], (unsigned long)bus.rx_frames[1],
        (unsigned)app.motors.ready, (unsigned long)remote.fault_epoch, (unsigned long)remote.rx_bytes
#if APP_FLYWHEEL_JOG_ENABLED
        , (long)jog.health.feedback[0].erpm, (long)jog.health.feedback[1].erpm,
        (long)jog.health.feedback[0].motor_current_mA,
        (long)jog.health.feedback[1].motor_current_mA, (unsigned)jog.selection,
        (long)jog.requested_mA, (unsigned)jog.active, (unsigned long)jog.health.faults,
        (unsigned)jog.stop_pending, (unsigned)jog.rearm_ready
#if APP_FLYWHEEL_SPEED_ACTIVE
        , (long)jog.speed.target_erpm, (long)jog.speed.filtered_erpm,
        (double)jog.speed.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (unsigned)jog.speed.at_limit, (unsigned)jog.speed.in_band,
        (long)jog.requested_right_mA, (long)jog.speed_right.target_erpm,
        (long)jog.speed_right.filtered_erpm,
        (double)jog.speed_right.filtered_erpm / APP_FLYWHEEL_POLE_PAIRS_ASSUMED,
        (unsigned)jog.speed_right.at_limit, (unsigned)jog.speed_right.in_band,
        (unsigned)jog.stop_reason
#endif
#endif
#if APP_YAW_ENABLED
        , (double)yaw.controller.angle_deg, (double)yaw.controller.target_deg,
        (double)yaw.controller.velocity_dps, (double)yaw.controller.torque_nm,
        (double)yaw.feedback.torque_nm, (double)yaw.controller.pid.kp,
        (double)yaw.controller.pid.ki, (double)yaw.controller.pid.kd,
        (unsigned)yaw.online, (unsigned)yaw.active, (unsigned)yaw.feedback.status,
        (unsigned long)yaw.controller.faults, (unsigned long)yaw.feedback_age_ms,
        (unsigned)yaw.requested, (unsigned)yaw.stop_pending,
        (int)yaw.feedback.driver_temp_c, (int)yaw.feedback.motor_temp_c,
        (double)yaw.feedback.position_rad, (unsigned long)yaw.rx_count
#endif
        );
#endif
    if (length > 0 && (size_t)length < sizeof(telemetry)) {
        if (HAL_UART_Transmit(&huart1, (uint8_t *)telemetry, (uint16_t)length,
                             APP_YAW_ENABLED ? 90U : 40U) != HAL_OK) {
            TransmitFailed();
        }
    }
}

/* 独立调度帧，不在Yaw的大快照栈内嵌套Pitch格式化调用。
 * AC5/O0会为整个函数预留栈，分派后各轴只使用自身快照。 */
static void PublishTelemetry(uint32_t now)
{
    /* Pause only FireWater snapshots, never command replies, safety or step reports. */
    if (!telemetry_enabled) { telemetry_ms = now; return; }
    if ((uint32_t)(now - telemetry_ms) < APP_TELEMETRY_PERIOD_MS) { return; }
#if APP_YAW_ENABLED || APP_PITCH_ENABLED
    /* 迟到只发送当前一帧，保留固定相位，不追发旧快照。 */
    telemetry_ms += (uint32_t)(now - telemetry_ms) / APP_TELEMETRY_PERIOD_MS * APP_TELEMETRY_PERIOD_MS;
#else
    telemetry_ms = now;
#endif
#if APP_PITCH_ENABLED
    if (pitch_view) { PublishPitchTelemetry(now); return; }
#endif
    PublishYawTelemetry(now);
}

void AppConsole_Poll(void)
{
    ConsoleByte item;
    uint32_t i;
    PublishRemoteRx(1U);
    if (needs_rearm) {
        taskENTER_CRITICAL();
        (void)HAL_UART_AbortReceive(&huart1);
        if (StartConsoleRx() == HAL_OK) { needs_rearm = 0U; }
        taskEXIT_CRITICAL();
    }
    if (observed_epoch != fault_epoch) {
#if APP_PITCH_ENABLED
        PitchConsole_CancelPreparation();
#endif
        observed_epoch = fault_epoch;
        line_length = 0U;
        discard_line = 1U;
        AppDiagnostics_Cancel();
        status_pending = 0U;
        if (diagnostic_mode) { SendText("D|ERROR|UART_FAULT\r\n"); }
#if APP_CONSOLE_RX_DMA_ENABLED
        (void)snprintf(telemetry, sizeof(telemetry),
            "C|UART_FAULT|EPOCH|%lu|LAST|0x%08lX|RX_ERR|%lu|DROP|%lu|TX_ERR|%lu|RX_BYTES|%lu|BURSTS|%lu|RX_DMA|1\r\n",
            (unsigned long)observed_epoch, (unsigned long)last_uart_error,
            (unsigned long)rx_uart_errors, (unsigned long)rx_queue_drops, (unsigned long)tx_errors,
            (unsigned long)rx_bytes_seen, (unsigned long)rx_bursts);
        SendText(telemetry);
#endif
    }
    for (i = 0U; i < APP_CONSOLE_RX_QUEUE_LENGTH; ++i) {
        if (xQueueReceive(rx_queue, &item, 0U) != pdPASS) { break; }
        if (item.epoch != fault_epoch || (uint32_t)(HAL_GetTick() - item.received_ms) >= APP_CONSOLE_INPUT_TIMEOUT_MS) {
            line_length = 0U; discard_line = 1U; continue;
        }
        if (line_length && (uint32_t)(item.received_ms - last_byte_ms) >= APP_CONSOLE_INPUT_TIMEOUT_MS) {
            line_length = 0U; discard_line = 1U;
        }
        last_byte_ms = item.received_ms;
        if (item.byte == '\r' || item.byte == '\n') {
            line[line_length] = '\0';
            if (!discard_line) { HandleLine(); }
            line_length = 0U; discard_line = 0U;
        }
        else if (!discard_line) {
            /* 命令只接受可打印ASCII，防止NUL把带尾部垃圾的行截成合法前缀。 */
            if (item.byte < 0x20U || item.byte > 0x7EU) { line_length = 0U; discard_line = 1U; }
            else if (line_length < sizeof(line) - 1U) { line[line_length++] = (char)item.byte; }
            else { line_length = 0U; discard_line = 1U; }
        }
    }
#if APP_YAW_ENABLED
    if (!diagnostic_mode && YawConsole_Poll(telemetry, sizeof(telemetry))) { SendText(telemetry); }
#endif
#if APP_PITCH_ENABLED
    if (!diagnostic_mode && PitchConsole_Poll(telemetry, sizeof(telemetry))) { SendText(telemetry); }
#endif
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
    PublishStartup(1U);
#endif
    if (diagnostic_mode) { PublishDiagnostics(HAL_GetTick()); }
    else { PublishTelemetry(HAL_GetTick()); }
}
