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

UART_HandleTypeDef huart1 = {1U, 0U}, huart4 = {4U, 0U};
static UART_HandleTypeDef other_uart = {2U, 0U};
static uint32_t now = 10U, queue_size, queue_head, queue_count;
static uint8_t bytes[APP_CONSOLE_RX_QUEUE_LENGTH][16], *rx_data;
static uint8_t fail_queue, fail_rearm, fail_transmit, request_allowed = 1U;
static uint32_t requests, stops, cancels, rearms, aborts, remote_rx, remote_errors;
static DiagKind requested_kind;
static uint8_t requested_node, requested_field;
static char output[4096];
static uint32_t output_length;
static DiagEvent events[8];
static uint8_t event_read, event_count;

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
    CHECK(handle == &huart1 && length < 512U && (timeout == 40U || timeout == 60U));
    CHECK(output_length + length < sizeof(output));
    if (fail_transmit) { return HAL_ERROR; }
    memcpy(output + output_length, data, length);
    output_length += length; output[output_length] = '\0';
    return HAL_OK;
}
void RemoteInput_RxCplt(UART_HandleTypeDef *handle) { CHECK(handle == &huart4); ++remote_rx; }
void RemoteInput_Error(UART_HandleTypeDef *handle) { CHECK(handle == &huart4); ++remote_errors; }
void RemoteInput_GetState(RemoteInput_State *state)
{
    memset(state, 0, sizeof(*state));
    state->has_frame = 1U; state->received_ms = now - 5U;
    state->channels[RC_YAW_CHANNEL] = 1500U;
    state->channels[RC_ARM_CHANNEL] = 1000U;
    state->channels[RC_FAILSAFE_CHANNEL] = 2000U;
}
void CanBus_GetState(CanBusState *state)
{
    memset(state, 0, sizeof(*state));
    state->rx_frames[0] = 12U; state->rx_frames[1] = 34U;
    state->errors[0] = 1U; state->errors[1] = 2U;
    state->dropped = 3U; state->fault_epoch = 6U; state->started = 1U;
}
void AppTasks_GetState(AppState *state) { memset(state, 0, sizeof(*state)); }
void AppDiagnostics_Cancel(void) { ++cancels; event_count = 0U; event_read = 0U; }
void AppTasks_RequestStop(void) { ++stops; AppDiagnostics_Cancel(); }
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
    CHECK(AppConsole_GetFaultEpoch() == epoch + 1U && cancels == previous + 1U);
    command_line(""); clear_output(); command_line("READ DM 7");
    CHECK(strcmp(output, "D|BEGIN|DM|7\r\n") == 0);
    clear_output(); previous = stops; command_line("STOP");
    CHECK(stops == previous + 1U && strcmp(output, "D|ERROR|CANCELLED\r\n") == 0);
    clear_output(); previous = cancels; command_line("DIAG OFF");
    CHECK(cancels == previous + 1U && strstr(output, "gimbal:") == output);
    clear_output(); previous = requests; command_line("READ DM 7"); CHECK(requests == previous);
    command_line("DIAG ON"); CHECK(strstr(output, "D|HELLO|GIMBAL_DIAG_1\r\n") != 0);
    test_done = 1U;
}
