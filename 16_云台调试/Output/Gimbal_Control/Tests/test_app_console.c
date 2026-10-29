#include "app_console.h"
#include "app_tasks.h"
#include "app_config.h"
#include "app_fault.h"
#include "remote_input.h"
#include "can_bus.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

#include "gimbal_startup.h"
#if 1U
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
static UART_HandleTypeDef __attribute__((unused)) other_uart = {2U, 0U};
static uint32_t now = 10U, queue_size, queue_head, queue_count;
static uint8_t bytes[APP_CONSOLE_RX_QUEUE_LENGTH][16], *rx_data;
static uint8_t fail_queue, fail_rearm, fail_transmit;
static uint32_t stops, rearms, aborts, remote_rx, remote_errors;
static uint32_t remote_events, remote_event_size;
static char output[4096];
static uint32_t output_length;
static uint8_t output_allowed, app_armed, remote_present = 1U;
static uint32_t remote_age = 5U, remote_reads, change_remote_at_read;
static uint16_t remote_yaw = RC_CENTER, remote_arm = 1000U, remote_failsafe = 2000U;
static uint16_t remote_other_sticks[3] = { RC_CENTER, RC_CENTER, RC_CENTER };
#if 1U
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
#ifndef TEST_CUSTOM_CONSOLE_DMA
HAL_StatusTypeDef ConsoleUart_StartReceive(uint8_t *data, uint16_t length)
{
    CHECK(data != 0 && length == 128U);
    rx_data = data;
    ++rearms;
    return fail_rearm ? HAL_ERROR : HAL_OK;
}
#endif
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{ CHECK(handle == &huart1); ++aborts; return HAL_OK; }
uint32_t HAL_UART_GetError(UART_HandleTypeDef *handle) { return handle->error; }
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, uint8_t *data,
    uint16_t length, uint32_t timeout)
{
    CHECK(handle == &huart1 && length < (1U ? 1024U : 512U) &&
        (timeout == (1U ? 90U : 40U) || timeout == (1U ? 120U : 60U)));
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
    state->channels[0U] = remote_yaw;
    state->channels[4U] = remote_arm;
    state->channels[7U] = remote_failsafe;
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
{ memset(state, 0, sizeof(*state)); state->logic.safety.operator_armed = app_armed;
  state->logic.safety.output_allowed = output_allowed; }
void AppTasks_RequestStop(void) { ++stops; output_allowed = 0U;  }
#if 1U
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
static void clear_output(void) { output_length = 0U; output[0] = '\0'; }
static void feed(const char *text)
{
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
    while (*text) {
        size_t length = strlen(text);
        if (length > 128U) { length = 128U; }
        CHECK(rx_data != 0);
        memcpy(rx_data, text, length);
        HAL_UARTEx_RxEventCallback(&huart1, (uint16_t)length);
        text += length;
    }
}
static void command_line(const char *text)
{ feed(text); feed("\r\n"); AppConsole_Poll(); }
