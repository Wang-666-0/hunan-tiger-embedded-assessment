#include "app_console.h"
#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#include "app_diagnostics.h"
#include "dm_diag.h"
#include "vesc_diag.h"
#include "usart.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

typedef struct { uint32_t received_ms, epoch; uint8_t byte; } ConsoleByte;
static QueueHandle_t rx_queue;
static volatile uint32_t fault_epoch;
static volatile uint8_t needs_rearm;
static uint8_t rx_byte;
static char line[40], telemetry[512];
static uint32_t observed_epoch, last_byte_ms, telemetry_ms;
static uint8_t line_length, discard_line;
static uint8_t diagnostic_mode, status_pending, status_index;

static void TransmitFailed(void)
{
    taskENTER_CRITICAL();
    fault_epoch++;
    taskEXIT_CRITICAL();
    AppTasks_RequestStop();
}

static void SendText(const char *message)
{
    /* 只在低优先级Console任务调用。CAN接收、遥控保护继续运行。 */
    if (HAL_UART_Transmit(&huart1, (uint8_t *)message, (uint16_t)strlen(message), 60U) != HAL_OK) {
        TransmitFailed();
    }
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
    uint8_t accepted;
    if (selection == FLYWHEEL_JOG_OFF) {
        taskENTER_CRITICAL();
        AppTasks_RequestStop();
        accepted = MotorBackend_SelectJog(FLYWHEEL_JOG_OFF);
        taskEXIT_CRITICAL();
    } else {
        if (diagnostic_mode) { SendText("W|ERROR|DIAGNOSTIC\r\n"); return; }
        if (!CanSelectWheel()) { SendText("W|ERROR|UNSAFE\r\n"); return; }
        taskENTER_CRITICAL();
        /* 检查和选择之间不得让新遥控帧、故障或Control任务改变条件。
         * 选择后STOP建立新帧屏障，旧OFF/ON输入不能解锁本会话。 */
        accepted = CanSelectWheel() && MotorBackend_SelectJog(selection);
        if (accepted) { AppTasks_RequestStop(); }
        taskEXIT_CRITICAL();
    }
    if (!accepted) { SendText("W|ERROR|UNSAFE\r\n"); return; }
    SendText(selection == FLYWHEEL_JOG_LEFT ? "W|OK|LEFT\r\n" :
        selection == FLYWHEEL_JOG_RIGHT ? "W|OK|RIGHT\r\n" :
        selection == FLYWHEEL_JOG_BOTH ? "W|OK|BOTH\r\n" : "W|OK|OFF\r\n");
}
#endif

static void HandleLine(void)
{
    const char *cursor;
    uint8_t node, field;
    DiagKind kind;
    CanFrame check;
    if (line[0] == '\0') { return; }
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
        AppTasks_RequestStop();
        status_pending = 0U;
        if (diagnostic_mode) { SendText("D|ERROR|CANCELLED\r\n"); }
        return;
    }
    if (strcmp(line, "DIAG ON") == 0) {
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
    rx_queue = xQueueCreate(APP_CONSOLE_RX_QUEUE_LENGTH, sizeof(ConsoleByte));
    if (rx_queue == 0) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "CONSOLE_BYTES");
}
void AppConsole_Start(void)
{
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK) { AppFault_Fatal(APP_FAULT_UART_START); }
}
uint32_t AppConsole_GetFaultEpoch(void) { return fault_epoch; }

/* 全工程只有这一份UART回调；UART4交给遥控模块，USART1交给控制台。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ConsoleByte item;
    BaseType_t woken = pdFALSE;
    if (huart == &huart4) { RemoteInput_RxCplt(huart); return; }
    if (huart != &huart1 || HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    item.byte = rx_byte;
    item.received_ms = HAL_GetTick();
    item.epoch = fault_epoch;
    if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) { fault_epoch++; }
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK) { fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4) { RemoteInput_Error(huart); return; }
    if (huart != &huart1) { return; }
    fault_epoch++;
    (void)HAL_UART_AbortReceive(huart);
    needs_rearm = 1U;
}

static void PublishTelemetry(uint32_t now)
{
    AppState app;
    CanBusState bus;
    RemoteInput_State remote;
    TargetRequest *request;
    uint32_t age;
    int length;
#if APP_FLYWHEEL_JOG_ENABLED
    MotorJogState jog;
#endif
    if ((uint32_t)(now - telemetry_ms) < APP_TELEMETRY_PERIOD_MS) { return; }
    telemetry_ms = now;
    AppTasks_GetState(&app);
    CanBus_GetState(&bus);
    RemoteInput_GetState(&remote);
#if APP_FLYWHEEL_JOG_ENABLED
    MotorBackend_GetJogState(&jog);
#endif
    request = &app.logic.targets.request;
    age = remote.has_frame ? (uint32_t)(now - remote.received_ms) : 0U;
    length = snprintf(telemetry, sizeof(telemetry),
        "gimbal:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%.2f,%.2f,%u,%u,%lu,%u,%.1f,%lu,%lu,%lu,%u,%lu,%lu"
#if APP_FLYWHEEL_JOG_ENABLED
        ",%ld,%ld,%ld,%ld,%u,%ld,%u,%lu,%u,%u"
#if APP_FLYWHEEL_SPEED_ACTIVE
        ",%ld,%ld,%.1f,%u,%u,%ld,%ld,%ld,%.1f,%u,%u,%u"
#endif
#endif
        "\r\n",
        (unsigned)remote.channels[0], (unsigned)remote.channels[1],
        (unsigned)remote.channels[2], (unsigned)remote.channels[3],
        (unsigned)remote.channels[4], (unsigned)remote.channels[5],
        (unsigned)remote.channels[6], (unsigned)remote.channels[7],
        (unsigned)remote.channels[8], (unsigned)remote.channels[9],
        (unsigned)(remote.has_frame && age < REMOTE_TIMEOUT_MS),
        (unsigned)app.logic.safety.operator_armed, (unsigned)app.logic.safety.output_allowed,
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
        );
    if (length > 0 && (size_t)length < sizeof(telemetry)) {
        if (HAL_UART_Transmit(&huart1, (uint8_t *)telemetry, (uint16_t)length, 40U) != HAL_OK) {
            TransmitFailed();
        }
    }
}

void AppConsole_Poll(void)
{
    ConsoleByte item;
    uint32_t i, now = HAL_GetTick();
    if (needs_rearm) {
        taskENTER_CRITICAL();
        (void)HAL_UART_AbortReceive(&huart1);
        if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) == HAL_OK) { needs_rearm = 0U; }
        taskEXIT_CRITICAL();
    }
    if (observed_epoch != fault_epoch) {
        observed_epoch = fault_epoch;
        line_length = 0U;
        discard_line = 1U;
        AppDiagnostics_Cancel();
        status_pending = 0U;
        if (diagnostic_mode) { SendText("D|ERROR|UART_FAULT\r\n"); }
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
    if (diagnostic_mode) { PublishDiagnostics(HAL_GetTick()); }
    else { PublishTelemetry(now); }
}
