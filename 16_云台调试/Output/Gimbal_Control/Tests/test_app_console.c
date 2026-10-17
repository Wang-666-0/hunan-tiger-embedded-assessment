#include "app_console.h"
#include "app_diagnostics.h"
#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#include "dm_diag.h"
#include "vesc_diag.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

#include "gimbal_startup.h"
#if APP_GIMBAL_REMOTE_STARTUP_ENABLED
static uint32_t now;
static uint32_t startup_manual_calls, startup_requests;
static uint8_t startup_request_allowed = 1U;
static GimbalStartupState startup_sample;
static uint8_t startup_event_pending;
void GimbalStartup_ManualOverride(void) { ++startup_manual_calls; }
uint8_t GimbalStartup_Request(const RemoteFrame *frame, uint32_t timestamp, uint8_t diagnostic)
{ CHECK(frame && frame->has_frame && timestamp == now && !diagnostic); ++startup_requests; return startup_request_allowed; }
void GimbalStartup_GetState(GimbalStartupState *out) { *out = startup_sample; }
uint8_t GimbalStartup_TakeEvent(GimbalStartupState *out)
{ if (!startup_event_pending) { return 0U; } *out = startup_sample; startup_event_pending = 0U; return 1U; }
#endif

UART_HandleTypeDef huart1 = {1U, 0U}, huart4 = {4U, 0U};
static UART_HandleTypeDef other_uart = {2U, 0U};
static uint32_t now = 10U, queue_size, queue_head, queue_count;
static uint8_t bytes[APP_CONSOLE_RX_QUEUE_LENGTH][16], *rx_data;
static uint8_t fail_queue, fail_rearm, fail_transmit, request_allowed = 1U;
static uint32_t requests, stops, cancels, rearms, aborts, remote_rx, remote_errors;
static uint32_t remote_events, remote_event_size;
static DiagKind requested_kind;
static uint8_t requested_node, requested_field;
static char output[4096];
static uint32_t output_length;
static DiagEvent events[8];
static uint8_t event_read, event_count;
static uint8_t output_allowed, app_armed, remote_present = 1U;
static uint32_t remote_age = 5U, remote_reads, change_remote_at_read;
static uint16_t remote_yaw = RC_CENTER, remote_arm = 1000U, remote_failsafe = 2000U;
static uint16_t remote_other_sticks[3] = { RC_CENTER, RC_CENTER, RC_CENTER };
#if APP_FLYWHEEL_JOG_ENABLED
static MotorJogState jog_state;
static uint8_t select_allowed = 1U;
static uint32_t selections;
static FlywheelJogSelection selected;
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle);

void AppFault_Fatal(AppFault fault) { (void)fault; CHECK(0); }
QueueHandle_t xQueueCreate(uint32_t length, uint32_t size)
{
    CHECK(length == APP_CONSOLE_RX_QUEUE_LENGTH && size <= sizeof(bytes[0]));
    queue_size = size;
    return bytes;
}
void vQueueAddToRegistry(QueueHandle_t queue, const char *name)
{ CHECK(queue == bytes && strcmp(name, "CONSOLE_BYTES") == 0); }
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken)
{
    CHECK(queue == bytes); (void)woken;
    if (fail_queue || queue_count == APP_CONSOLE_RX_QUEUE_LENGTH) { return pdFALSE; }
    memcpy(bytes[(queue_head + queue_count) % APP_CONSOLE_RX_QUEUE_LENGTH], data, queue_size);
    ++queue_count;
    return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t wait)
{
    CHECK(queue == bytes && wait == 0U);
    if (!queue_count) { return pdFALSE; }
    memcpy(data, bytes[queue_head], queue_size);
    queue_head = (queue_head + 1U) % APP_CONSOLE_RX_QUEUE_LENGTH;
    --queue_count;
    return pdPASS;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length)
{
    CHECK(handle == &huart1 && data != 0 && length == 1U);
    rx_data = data; ++rearms;
    return fail_rearm ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{ CHECK(handle == &huart1); ++aborts; return HAL_OK; }
uint32_t HAL_UART_GetError(UART_HandleTypeDef *handle) { return handle->error; }
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, uint8_t *data,
    uint16_t length, uint32_t timeout)
{
    CHECK(handle == &huart1 && length < (APP_YAW_ENABLED ? 1024U : 512U) &&
        (timeout == (APP_YAW_ENABLED ? 90U : 40U) || timeout == (APP_YAW_ENABLED ? 120U : 60U)));
    CHECK(output_length + length < sizeof(output));
    if (fail_transmit) { return HAL_ERROR; }
    memcpy(output + output_length, data, length);
    output_length += length; output[output_length] = '\0';
    return HAL_OK;
}
void RemoteInput_RxCplt(UART_HandleTypeDef *handle) { CHECK(handle == &huart4); ++remote_rx; }
void RemoteInput_Error(UART_HandleTypeDef *handle) { CHECK(handle == &huart4); ++remote_errors; }
void RemoteInput_RxEvent(UART_HandleTypeDef *handle, uint16_t size)
{ CHECK(handle == &huart4); ++remote_events; remote_event_size = size; }
void RemoteInput_GetState(RemoteInput_State *state)
{
    uint32_t i;
    ++remote_reads;
    if (change_remote_at_read == remote_reads) { remote_arm = 2000U; }
    memset(state, 0, sizeof(*state));
    for (i = 0U; i < 10U; ++i) { state->channels[i] = 1500U; }
    for (i = 0U; i < 3U; ++i) { state->channels[i + 1U] = remote_other_sticks[i]; }
    state->has_frame = remote_present; state->received_ms = now - remote_age;
    state->channels[RC_YAW_CHANNEL] = remote_yaw;
    state->channels[RC_ARM_CHANNEL] = remote_arm;
    state->channels[RC_FAILSAFE_CHANNEL] = remote_failsafe;
}
void CanBus_GetState(CanBusState *state)
{
    memset(state, 0, sizeof(*state));
    state->rx_frames[0] = 12U; state->rx_frames[1] = 34U;
    state->errors[0] = 1U; state->errors[1] = 2U;
    state->dropped = 3U; state->fault_epoch = 6U; state->started = 1U;
}
uint8_t CanBus_OutputAllowed(void) { return output_allowed; }
void AppTasks_GetState(AppState *state)
{ memset(state, 0, sizeof(*state)); state->logic.safety.operator_armed = app_armed; }
void AppDiagnostics_Cancel(void) { ++cancels; event_count = 0U; event_read = 0U; }
void AppTasks_RequestStop(void) { ++stops; output_allowed = 0U; AppDiagnostics_Cancel(); }
#if APP_FLYWHEEL_JOG_ENABLED
void MotorBackend_GetJogState(MotorJogState *state) { *state = jog_state; }
uint8_t MotorBackend_SelectJog(FlywheelJogSelection selection)
{
    ++selections; selected = selection;
    if (!select_allowed && selection != FLYWHEEL_JOG_OFF) { return 0U; }
    jog_state.selection = selection;
    if (selection == FLYWHEEL_JOG_OFF) {
        jog_state.requested_mA = 0; jog_state.active = 0U;
    }
    return 1U;
}
#endif
uint8_t AppDiagnostics_Request(DiagKind kind, uint8_t node, uint8_t field)
{
    ++requests; requested_kind = kind; requested_node = node; requested_field = field;
    return request_allowed;
}
uint8_t AppDiagnostics_TakeEvent(DiagEvent *event)
{
    if (event_read >= event_count) { return 0U; }
    *event = events[event_read++];
    return 1U;
}
uint8_t AppDiagnostics_GetPassive(uint32_t index, DiagPassiveFrame *out)
{
    uint8_t i;
    CHECK(index < APP_DIAG_PASSIVE_SLOTS);
    if (index != 0U) { return 0U; }
    memset(out, 0, sizeof(*out));
    out->count = 9U; out->frame.bus = 2U; out->frame.id = 0x205U;
    out->frame.dlc = 8U; out->frame.received_ms = now - 6U;
    for (i = 0U; i < 8U; ++i) { out->frame.data[i] = i; }
    return 1U;
}

static void clear_output(void) { output_length = 0U; output[0] = '\0'; }
static void feed(const char *text)
{
    while (*text) {
        CHECK(rx_data != 0);
        *rx_data = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(&huart1);
    }
}
static void command_line(const char *text)
{ feed(text); feed("\r\n"); AppConsole_Poll(); }
static void expect_rejected(const char *text)
{
    uint32_t previous = requests;
    clear_output(); command_line(text);
    CHECK(requests == previous && strcmp(output, "D|ERROR|BAD_COMMAND\r\n") == 0);
}
static void event_init(uint8_t count)
{ memset(events, 0, sizeof(events)); event_read = 0U; event_count = count; }

void test_run(void)
{
    char text[80];
    uint32_t previous, epoch, previous_rearms, i;
    test_stage = 1U;
    AppConsole_CreateQueue(); AppConsole_Start(); CHECK(rx_data != 0 && rearms == 1U);
    HAL_UART_RxCpltCallback(&huart4); HAL_UART_ErrorCallback(&huart4);
    CHECK(remote_rx == 1U && remote_errors == 1U && rearms == 1U);
    HAL_UART_RxCpltCallback(&other_uart); HAL_UART_ErrorCallback(&other_uart);
    CHECK(remote_rx == 1U && remote_errors == 1U && AppConsole_GetFaultEpoch() == 0U);
    command_line("READ DM 7"); CHECK(requests == 0U && !output_length);
    command_line("STOP"); CHECK(stops == 1U && cancels == 1U && !output_length);
    command_line("DIAG ON"); CHECK(strcmp(output, "D|HELLO|GIMBAL_DIAG_1\r\n") == 0);
    clear_output(); command_line("READ DM 7");
    CHECK(requests == 1U && requested_kind == DIAG_DM && requested_node == 2U && requested_field == 7U);
    CHECK(strcmp(output, "D|BEGIN|DM|7\r\n") == 0);
    clear_output(); command_line("READ VESC 70 91");
    CHECK(requests == 2U && requested_kind == DIAG_VESC && requested_node == 70U && requested_field == 91U);
    CHECK(strcmp(output, "D|BEGIN|VESC|70|91\r\n") == 0);
    clear_output(); command_line("READ VESC 71 4");
    CHECK(requests == 3U && requested_kind == DIAG_VESC && requested_node == 71U && requested_field == 4U);
    CHECK(strcmp(output, "D|BEGIN|VESC|71|4\r\n") == 0);

    test_stage = 2U;
    for (i = 0U; i < 256U; ++i) {
        uint8_t allowed = DmDiag_IsReadableRid((uint8_t)i);
        previous = requests; clear_output();
        (void)snprintf(text, sizeof(text), "READ DM %lu", (unsigned long)i); command_line(text);
        CHECK(requests == previous + allowed);
        CHECK(strstr(output, allowed ? "D|BEGIN|DM|" : "D|ERROR|BAD_COMMAND\r\n") != 0);
        if (allowed) { CHECK(requested_field == i && requested_node == 2U); }
    }
    for (i = 0U; i < 256U; ++i) {
        uint8_t allowed = (uint8_t)(i == 0U || i == 4U || i == 14U || i == 17U || i == 91U);
        previous = requests; clear_output();
        (void)snprintf(text, sizeof(text), "READ VESC 70 %lu", (unsigned long)i); command_line(text);
        CHECK(requests == previous + allowed);
        CHECK(strstr(output, allowed ? "D|BEGIN|VESC|70|" : "D|ERROR|BAD_COMMAND\r\n") != 0);
    }
    for (i = 0U; i < 256U; ++i) {
        uint8_t allowed = (uint8_t)(i == 70U || i == 71U);
        previous = requests; clear_output();
        (void)snprintf(text, sizeof(text), "READ VESC %lu 4", (unsigned long)i); command_line(text);
        CHECK(requests == previous + allowed);
        CHECK(strstr(output, allowed ? "D|BEGIN|VESC|" : "D|ERROR|BAD_COMMAND\r\n") != 0);
    }
    expect_rejected("READ DM -7"); expect_rejected("READ DM +7");
    expect_rejected("READ DM 256"); expect_rejected("READ DM 7x"); expect_rejected("READ DM 7 ");
    expect_rejected("READ DM"); expect_rejected("READ DM 4294967296");
    expect_rejected("READ VESC 70"); expect_rejected("READ VESC 70 ");
    expect_rejected("READ VESC 70 256"); expect_rejected("READ VESC 70 4x");
    expect_rejected("READ VESC 70  4"); expect_rejected("READ VESC -70 4");
    expect_rejected("READ VESC 70 4 5"); expect_rejected("READ VESC 70 4294967296");
    expect_rejected("READ VESC 70 30"); /* ALIVE不能借读接口保活。 */
    expect_rejected("SET RPM 6000"); expect_rejected("ENABLE"); expect_rejected("RAW CAN 200 0000");
    expect_rejected("WRITE DM 7 0"); expect_rejected("READ GL 1"); expect_rejected("SAVE");
    clear_output(); request_allowed = 0U; previous = requests; command_line("READ DM 7");
    CHECK(requests == previous + 1U && strcmp(output, "D|ERROR|BUSY\r\n") == 0);
    request_allowed = 1U;

    test_stage = 3U;
    clear_output(); command_line("STATUS");
    CHECK(strcmp(output, "D|STATUS|12|34|1|2|3|6|1|1500|1000|2000|0\r\n") == 0);
    clear_output(); previous = requests; command_line("READ DM 7");
    CHECK(requests == previous);
    CHECK(strcmp(output, "D|ERROR|BUSY\r\nD|CAN|2|0|205|8|0001020304050607|9|6\r\n") == 0);
    clear_output();
    for (i = 0U; i < APP_DIAG_PASSIVE_SLOTS; ++i) { AppConsole_Poll(); }
    CHECK(strcmp(output, "D|DONE|STATUS\r\n") == 0);
    clear_output(); event_init(3U);
    events[0].event = DIAG_EVENT_DATA; events[0].kind = DIAG_DM; events[0].field = 7U;
    events[0].length = 4U; events[0].data[0] = 0x01U; events[0].data[1] = 0x23U;
    events[0].data[2] = 0xABU; events[0].data[3] = 0xCDU;
    events[1].event = DIAG_EVENT_DONE; events[1].kind = DIAG_DM; events[1].field = 7U;
    events[2].event = DIAG_EVENT_ERROR; events[2].error = DIAG_ERROR_TIMEOUT;
    AppConsole_Poll();
    CHECK(strcmp(output, "D|DM|7|0123ABCD\r\nD|DONE|DM|7\r\nD|ERROR|TIMEOUT\r\n") == 0);
    clear_output(); event_init(2U);
    events[0].event = DIAG_EVENT_DATA; events[0].kind = DIAG_VESC;
    events[0].node = 70U; events[0].field = 14U; events[0].total = 600U; events[0].offset = 256U;
    events[0].length = 32U;
    for (i = 0U; i < 32U; ++i) { events[0].data[i] = (uint8_t)i; }
    events[1].event = DIAG_EVENT_DONE; events[1].kind = DIAG_VESC; events[1].node = 70U; events[1].field = 14U;
    AppConsole_Poll();
    CHECK(strcmp(output, "D|VESC|70|14|600|256|000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F\r\nD|DONE|VESC|70|14\r\n") == 0);
    clear_output(); event_init(5U);
    for (i = 0U; i < 5U; ++i) { events[i].event = DIAG_EVENT_ERROR; events[i].error = (DiagError)(i + 1U); }
    AppConsole_Poll(); CHECK(event_read == 4U);
    CHECK(strcmp(output, "D|ERROR|TIMEOUT\r\nD|ERROR|CAN_FAULT\r\nD|ERROR|TX_FAILED\r\nD|ERROR|BAD_REPLY\r\n") == 0);
    clear_output(); AppConsole_Poll(); CHECK(strcmp(output, "D|ERROR|CANCELLED\r\n") == 0);

    test_stage = 4U;
    clear_output(); previous = requests;
    memset(text, 'x', sizeof(text) - 1U); text[sizeof(text) - 1U] = '\0';
    command_line(text); CHECK(requests == previous && !output_length);
    command_line("READ DM 7"); CHECK(requests == previous + 1U);
    /* 在合法前缀后注入NUL、控制字节或非ASCII，整行必须丢弃；
     * 下一条完整ASCII命令仍可正常执行，不能永久卡在丢弃状态。 */
    for (i = 0U; i < 256U; ++i) {
        if ((i < 0x20U || i > 0x7EU) && i != '\r' && i != '\n') {
            clear_output(); previous = requests;
            feed("READ DM 7");
            *rx_data = (uint8_t)i;
            HAL_UART_RxCpltCallback(&huart1);
            feed("junk\r\n"); AppConsole_Poll();
            CHECK(requests == previous && !output_length);
            command_line("READ DM 7");
            CHECK(requests == previous + 1U && strcmp(output, "D|BEGIN|DM|7\r\n") == 0);
        }
    }
    clear_output(); previous = requests;
    feed("READ DM "); AppConsole_Poll(); now += APP_CONSOLE_INPUT_TIMEOUT_MS;
    command_line("7"); CHECK(requests == previous && !output_length);
    command_line("READ DM 7"); CHECK(requests == previous + 1U);
    clear_output(); previous = requests;
    feed("READ DM 7\n"); now += APP_CONSOLE_INPUT_TIMEOUT_MS; AppConsole_Poll();
    CHECK(requests == previous && !output_length);
    command_line("READ DM 7"); CHECK(requests == previous); /* 丢弃至一个新换行边界。 */
    command_line("READ DM 7"); CHECK(requests == previous + 1U);

    test_stage = 5U;
    clear_output(); previous = requests; epoch = AppConsole_GetFaultEpoch();
    feed("READ DM "); AppConsole_Poll();
    HAL_UART_ErrorCallback(&huart1); CHECK(AppConsole_GetFaultEpoch() == epoch + 1U);
    command_line("7"); CHECK(requests == previous);
    CHECK(strcmp(output, "D|ERROR|UART_FAULT\r\n") == 0);
    clear_output(); command_line("READ DM 7"); CHECK(requests == previous + 1U);
    clear_output(); previous = requests; epoch = AppConsole_GetFaultEpoch();
    feed("READ DM 7\n"); HAL_UART_ErrorCallback(&huart1); AppConsole_Poll();
    CHECK(requests == previous && AppConsole_GetFaultEpoch() == epoch + 1U);
    command_line(""); clear_output(); command_line("READ DM 7"); CHECK(requests == previous + 1U);
    clear_output(); previous = requests; epoch = AppConsole_GetFaultEpoch();
    fail_queue = 1U; feed("R"); fail_queue = 0U; command_line("EAD DM 7");
    CHECK(requests == previous && AppConsole_GetFaultEpoch() == epoch + 1U);
    clear_output(); command_line("READ DM 7"); CHECK(requests == previous + 1U);
    clear_output(); previous = requests; epoch = AppConsole_GetFaultEpoch();
    fail_rearm = 1U; feed("R"); fail_rearm = 0U; previous_rearms = rearms;
    command_line("EAD DM 7");
    CHECK(requests == previous && AppConsole_GetFaultEpoch() == epoch + 1U);
    CHECK(rearms > previous_rearms && aborts > 0U);
    clear_output(); command_line("READ DM 7"); CHECK(requests == previous + 1U);
    clear_output(); epoch = AppConsole_GetFaultEpoch(); previous = requests;
    huart1.error = 1U; feed("READ DM 7\n"); huart1.error = 0U; AppConsole_Poll();
    CHECK(requests == previous && AppConsole_GetFaultEpoch() == epoch);

    test_stage = 6U;
    clear_output(); previous = cancels; fail_transmit = 1U; epoch = AppConsole_GetFaultEpoch();
    command_line("DIAG ON"); fail_transmit = 0U;
    CHECK(AppConsole_GetFaultEpoch() == epoch + 1U &&
        cancels == previous + 1U + APP_FLYWHEEL_JOG_ENABLED);
    command_line(""); clear_output(); command_line("READ DM 7");
    CHECK(strcmp(output, "D|BEGIN|DM|7\r\n") == 0);
    clear_output(); previous = stops; command_line("STOP");
    CHECK(stops == previous + 1U && strcmp(output, "D|ERROR|CANCELLED\r\n") == 0);
    clear_output(); previous = cancels; command_line("DIAG OFF");
    CHECK(cancels == previous + 1U && strstr(output, "gimbal:") == output);
    clear_output(); previous = requests; command_line("READ DM 7"); CHECK(requests == previous);
    command_line("DIAG ON"); CHECK(strstr(output, "D|HELLO|GIMBAL_DIAG_1\r\n") != 0);

    test_stage = 7U;
    /* 状态尾字段必须读取运行许可，两种编译模式都不能返回编译宏。 */
    clear_output(); output_allowed = 1U; command_line("STATUS");
    CHECK(strcmp(output, "D|STATUS|12|34|1|2|3|6|1|1500|1000|2000|1\r\n") == 0);
    output_allowed = 0U; command_line("DIAG OFF"); clear_output();
    {
        uint32_t before_requests = requests, before_stops = stops, before_cancels = cancels;
        command_line("WHEEL INFO");
#if APP_FLYWHEEL_SPEED_ACTIVE
        CHECK(strcmp(output, "W|INFO|DUAL_RPM6000_I2000_V1|LEFT_MA|2000|RIGHT_MA|2000|LEFT_SIGN|1|RIGHT_SIGN|1|MS|0|ERPM|46200|ENABLED|1|HOLD_TO_RUN|1|COAST_ERPM|44100|RESUME_ERPM|42000|MODE|CURRENT_PI|POLE_PAIRS_ASSUMED|7|TARGET_RPM|6000|TARGET_ERPM|42000|BOTH|1|FB_MAX_MA|3000|RAMP_ERPMS|42000|BAND_ERPM|700\r\n") == 0);
#elif APP_FLYWHEEL_JOG_ENABLED
        CHECK(strcmp(output, "W|INFO|DUAL_RPM6000_I2000_V1|LEFT_MA|500|RIGHT_MA|500|LEFT_SIGN|1|RIGHT_SIGN|1|MS|0|ERPM|2000|ENABLED|1|HOLD_TO_RUN|1|COAST_ERPM|1000|RESUME_ERPM|600|MODE|OBSERVE|POLE_PAIRS_ASSUMED|7|TARGET_RPM|0|TARGET_ERPM|0|BOTH|0|FB_MAX_MA|1000|RAMP_ERPMS|0|BAND_ERPM|0\r\n") == 0);
#else
        CHECK(strcmp(output, "W|INFO|DUAL_RPM6000_I2000_V1|LEFT_MA|500|RIGHT_MA|500|LEFT_SIGN|1|RIGHT_SIGN|1|MS|0|ERPM|2000|ENABLED|0|HOLD_TO_RUN|1|COAST_ERPM|1000|RESUME_ERPM|600|MODE|OFF|POLE_PAIRS_ASSUMED|7|TARGET_RPM|0|TARGET_ERPM|0|BOTH|0|FB_MAX_MA|1000|RAMP_ERPMS|0|BAND_ERPM|0\r\n") == 0);
#endif
        CHECK(requests == before_requests && stops == before_stops && cancels == before_cancels);
        CHECK(!output_allowed);
        clear_output();
    }
#if APP_FLYWHEEL_JOG_ENABLED
    previous = selections; i = stops; command_line("WHEEL LEFT");
    CHECK(selections == previous + 1U && selected == FLYWHEEL_JOG_LEFT && stops == i + 1U);
    CHECK(strcmp(output, "W|OK|LEFT\r\n") == 0);
    clear_output(); command_line("WHEEL RIGHT");
    CHECK(selected == FLYWHEEL_JOG_RIGHT && strcmp(output, "W|OK|RIGHT\r\n") == 0);
#if APP_FLYWHEEL_SPEED_ACTIVE
    clear_output(); previous = selections; i = stops; command_line("WHEEL BOTH");
    CHECK(selections == previous + 1U && stops == i + 1U && selected == FLYWHEEL_JOG_BOTH);
    CHECK(strcmp(output, "W|OK|BOTH\r\n") == 0 && !output_allowed);
    clear_output(); previous = selections; i = stops;
    change_remote_at_read = remote_reads + 2U; command_line("WHEEL BOTH");
    CHECK(selections == previous && stops == i && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0);
    change_remote_at_read = 0U; remote_arm = 1000U;
#endif

    /* 初次检查和最终临界区内的检查都必须通过。 */
    clear_output(); previous = selections; i = stops;
    change_remote_at_read = remote_reads + 2U; command_line("WHEEL LEFT");
    CHECK(selections == previous && stops == i && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0);
    change_remote_at_read = 0U; remote_arm = 1000U;
    clear_output(); remote_present = 0U; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); remote_present = 1U;
    clear_output(); remote_age = REMOTE_TIMEOUT_MS; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); remote_age = 5U;
    clear_output(); remote_age = 0xFFFFFFFFU; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); remote_age = 5U;
    clear_output(); remote_yaw = RC_CENTER + RC_DEADBAND + 1U; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); remote_yaw = RC_CENTER;
    clear_output(); remote_yaw = RC_CENTER - RC_DEADBAND - 1U; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); remote_yaw = RC_CENTER;
    clear_output(); remote_arm = RC_SWITCH_LOW + 1U; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); remote_arm = 1000U;
    clear_output(); remote_failsafe = 799U; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); remote_failsafe = 2000U;
    clear_output(); app_armed = 1U; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); app_armed = 0U;
    clear_output(); output_allowed = 1U; command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0); output_allowed = 0U;
    clear_output(); select_allowed = 0U; i = stops; command_line("WHEEL LEFT");
    CHECK(selections == previous + 1U && stops == i && strcmp(output, "W|ERROR|UNSAFE\r\n") == 0);
    select_allowed = 1U;

    clear_output(); remote_yaw = RC_CENTER + RC_DEADBAND; remote_arm = RC_SWITCH_LOW;
    command_line("WHEEL RIGHT"); CHECK(strcmp(output, "W|OK|RIGHT\r\n") == 0);
    clear_output(); remote_yaw = RC_CENTER - RC_DEADBAND;
    command_line("WHEEL LEFT"); CHECK(strcmp(output, "W|OK|LEFT\r\n") == 0);
    remote_yaw = RC_CENTER; remote_arm = 1000U;

    /* OFF即使缺遥控、已解锁或在诊断中也必须停止；DIAG ON取消选轮。 */
    clear_output(); remote_present = 0U; app_armed = 1U; output_allowed = 1U; i = stops;
    command_line("WHEEL OFF");
    CHECK(selected == FLYWHEEL_JOG_OFF && stops == i + 1U && output_allowed == 0U);
    CHECK(strcmp(output, "W|OK|OFF\r\n") == 0);
    remote_present = 1U; app_armed = 0U;
    clear_output(); command_line("WHEEL RIGHT"); clear_output(); i = stops;
    command_line("DIAG ON");
    CHECK(stops == i + 1U && jog_state.selection == FLYWHEEL_JOG_OFF);
    CHECK(strcmp(output, "D|HELLO|GIMBAL_DIAG_1\r\n") == 0);
    clear_output(); previous = selections; i = stops;
    command_line("WHEEL INFO");
    CHECK(strstr(output, "W|INFO|DUAL_RPM6000_I2000_V1|") == output);
    CHECK(selections == previous && stops == i);
    previous = selections; clear_output(); command_line("WHEEL LEFT");
    CHECK(selections == previous && strcmp(output, "W|ERROR|DIAGNOSTIC\r\n") == 0);
#if APP_FLYWHEEL_SPEED_ACTIVE
    clear_output(); command_line("WHEEL BOTH");
    CHECK(selections == previous && strcmp(output, "W|ERROR|DIAGNOSTIC\r\n") == 0);
#endif
    clear_output(); command_line("WHEEL OFF");
    CHECK(strcmp(output, "W|OK|OFF\r\n") == 0);
    command_line("DIAG OFF"); clear_output();

    jog_state.health.feedback[0].erpm = -20; jog_state.health.feedback[1].erpm = 30;
    jog_state.health.feedback[0].motor_current_mA = -100;
    jog_state.health.feedback[1].motor_current_mA = 200;
    jog_state.selection = FLYWHEEL_JOG_RIGHT; jog_state.requested_mA = 500;
    jog_state.active = 1U; jog_state.health.faults = 32U;
    jog_state.stop_pending = 1U; jog_state.rearm_ready = 0U;
    command_line("WHEEL STATUS");
    CHECK(strcmp(output, "W|STATUS|-20|30|-100|200|2|500|1|32|1|0\r\n") == 0);
    clear_output(); now += APP_TELEMETRY_PERIOD_MS; AppConsole_Poll();
    CHECK(strstr(output, "gimbal:") == output);
    CHECK(strstr(output, ",-20,30,-100,200,2,500,1,32,1,0") != 0);
    {
        uint32_t commas = 0U; const char *p;
        for (p = output; *p; ++p) { if (*p == ',') { ++commas; } }
        CHECK(commas == (APP_FLYWHEEL_SPEED_ACTIVE ? 50U : 38U));
    }
    clear_output(); jog_state.selection = FLYWHEEL_JOG_LEFT;
    jog_state.requested_mA = 500; jog_state.health.faults = 0U;
    jog_state.stop_pending = 0U;
    command_line("WHEEL STATUS");
    CHECK(strcmp(output, "W|STATUS|-20|30|-100|200|1|500|1|0|0|0\r\n") == 0);
    clear_output(); now += APP_TELEMETRY_PERIOD_MS; AppConsole_Poll();
    CHECK(strstr(output, ",-20,30,-100,200,1,500,1,0,0,0") != 0);
#if APP_FLYWHEEL_SPEED_ACTIVE
    clear_output();
    jog_state.speed.target_erpm = 42000.0f; jog_state.speed.filtered_erpm = 41300.0f;
    jog_state.last_speed_selection = FLYWHEEL_JOG_LEFT;
    jog_state.speed.at_limit = 1U; jog_state.speed.in_band = 1U;
    jog_state.speed.run_ms = 5000U; jog_state.speed.peak_erpm = 43100U;
    jog_state.speed.longest_band_ms = 1300U; jog_state.speed.limited_ms = 200U;
    jog_state.speed.coast_ms = 20U; jog_state.speed.zero_current_ms = 40U;
    jog_state.speed.low_speed_ms = 60U;
    jog_state.speed.rise90_seen = jog_state.speed.band_first_seen = 1U;
    jog_state.speed.rise90_ms = 1234U; jog_state.speed.band_first_ms = 2345U;
    jog_state.requested_mA = jog_state.requested_left_mA = 2000;
    previous = stops; i = selections;
    command_line("WHEEL SPEED");
    CHECK(strcmp(output, "W|SPEED|LAST_WHEEL|1|ACTIVE|1|STOP_REASON|0|FAULT|0|SAFETY|0|LEFT_MA|2000|RIGHT_MA|0\r\n"
        "W|SPEED_NODE|70|TARGET_ERPM|42000|RAMP_ERPM|42000|FILTER_ERPM|41300|RPM_EST|5900.0|AT_LIMIT|1|IN_BAND|1|RUN_MS|5000|PEAK_ERPM|43100|BAND_MAX_MS|1300|LIMIT_MS|200|COAST_MS|20|ZERO_MS|40|LOW_MS|60|T90_MS|1234|BAND_FIRST_MS|2345\r\n") == 0);
    CHECK(stops == previous && selections == i);
    clear_output(); now += APP_TELEMETRY_PERIOD_MS; AppConsole_Poll();
    CHECK(strstr(output, ",42000,41300,5900.0,1,1,0,0,0,0.0,0,0,0\r\n") != 0);
    /* 双轮状态必须有两组独立统计，查询不能改变选择、许可或电流。 */
    clear_output(); jog_state.selection = jog_state.last_speed_selection = FLYWHEEL_JOG_BOTH;
    jog_state.requested_right_mA = 1500;
    jog_state.speed_right.target_erpm = 42000.0f; jog_state.speed_right.filtered_erpm = 37800.0f;
    jog_state.speed_right.run_ms = 5000U; jog_state.speed_right.peak_erpm = 39000U;
    jog_state.speed_right.rise90_seen = 1U; jog_state.speed_right.rise90_ms = 2100U;
    command_line("WHEEL SPEED");
    CHECK(strstr(output, "W|SPEED|LAST_WHEEL|3|ACTIVE|1|STOP_REASON|0|FAULT|0|SAFETY|0|LEFT_MA|2000|RIGHT_MA|1500\r\n") == output);
    CHECK(strstr(output, "W|SPEED_NODE|70|TARGET_ERPM|42000|") != 0);
    CHECK(strstr(output, "W|SPEED_NODE|71|TARGET_ERPM|42000|RAMP_ERPM|42000|FILTER_ERPM|37800|RPM_EST|5400.0|AT_LIMIT|0|IN_BAND|0|RUN_MS|5000|PEAK_ERPM|39000|BAND_MAX_MS|0|LIMIT_MS|0|COAST_MS|0|ZERO_MS|0|LOW_MS|0|T90_MS|2100|BAND_FIRST_MS|-1\r\n") != 0);
    CHECK(stops == previous && selections == i && jog_state.requested_right_mA == 1500);
    clear_output(); now += APP_TELEMETRY_PERIOD_MS; AppConsole_Poll();
    CHECK(strstr(output, ",42000,41300,5900.0,1,1,1500,42000,37800,5400.0,0,0,0\r\n") != 0);
    clear_output(); jog_state.active = 0U; jog_state.stop_reason = FLYWHEEL_STOP_INPUT;
    jog_state.requested_left_mA = jog_state.requested_right_mA = 0;
    command_line("WHEEL SPEED");
    CHECK(strstr(output, "|ACTIVE|0|STOP_REASON|1|FAULT|0|SAFETY|0|LEFT_MA|0|RIGHT_MA|0\r\n") != 0);
    CHECK(strstr(output, "|RUN_MS|5000|PEAK_ERPM|43100|") != 0);
    CHECK(strstr(output, "|RUN_MS|5000|PEAK_ERPM|39000|") != 0);
    CHECK(strstr(output, "|T90_MS|1234|BAND_FIRST_MS|2345\r\n") != 0);
#endif
#else
    previous = stops; command_line("WHEEL LEFT");
    CHECK(stops == previous && !output_length);
#endif

    test_stage = 8U;
    clear_output(); now += APP_TELEMETRY_PERIOD_MS; fail_transmit = 1U;
    epoch = AppConsole_GetFaultEpoch(); previous = stops;
    AppConsole_Poll(); fail_transmit = 0U;
    CHECK(AppConsole_GetFaultEpoch() == epoch + 1U && stops == previous + 1U);
    test_done = 1U;
}
