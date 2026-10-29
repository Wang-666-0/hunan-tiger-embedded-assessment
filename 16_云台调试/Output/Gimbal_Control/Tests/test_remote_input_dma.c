/* Real UART4 input/parser and safety logic; DMA/HAL/RTOS are deterministic mocks. */
#include "remote_input.h"
#include "app_config.h"
#include "app_logic.h"
#include "app_fault.h"
#include "queue.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
UART_HandleTypeDef huart1 = {1U, 0U}, huart4 = {4U, 0U};
static uint32_t now = 100U, queue_size, head, count, rearms, aborts;
static uint8_t queue_bytes[REMOTE_RX_QUEUE_LENGTH][16], *dma_data, fail_rearm;
static RemoteInput_State remote;
static uint16_t channels[14];
static AppLogic logic;

void AppFault_Fatal(AppFault fault) { (void)fault; CHECK(0); }
uint32_t HAL_GetTick(void) { return now; }
uint32_t HAL_UART_GetError(UART_HandleTypeDef *handle) { return handle->error; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{ CHECK(handle == &huart4); ++aborts; return HAL_OK; }
HAL_StatusTypeDef RemoteUart_StartReceive(uint8_t *data, uint16_t length)
{
    CHECK(data && length == 64U && length == 64U);
    dma_data = data; ++rearms;
    if (!fail_rearm) { huart4.error = 0U; }
    return fail_rearm ? HAL_ERROR : HAL_OK;
}
QueueHandle_t xQueueCreate(uint32_t length, uint32_t size)
{ CHECK(length == REMOTE_RX_QUEUE_LENGTH && size <= 16U); queue_size = size; return queue_bytes; }
void vQueueAddToRegistry(QueueHandle_t queue, const char *name)
{ CHECK(queue == queue_bytes && !strcmp(name, "IBUS_BYTES")); }
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken)
{
    CHECK(queue == queue_bytes); (void)woken;
    if (count == REMOTE_RX_QUEUE_LENGTH) { return pdFALSE; }
    memcpy(queue_bytes[(head + count) % REMOTE_RX_QUEUE_LENGTH], data, queue_size);
    ++count; return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t wait)
{
    CHECK(queue == queue_bytes && wait == 0U);
    if (!count) { return pdFALSE; }
    memcpy(data, queue_bytes[head], queue_size); head = (head + 1U) % REMOTE_RX_QUEUE_LENGTH;
    --count; return pdPASS;
}
static void Read(void) { RemoteInput_GetState(&remote); }
static void Poll(void) { RemoteInput_Poll(); Read(); }
static void Burst(const uint8_t *data, uint16_t length)
{
    CHECK(length && length <= 64U);
    memcpy(dma_data, data, length); RemoteInput_RxEvent(&huart4, length);
}
static void FrameBytes(uint8_t data[32])
{
    uint32_t i; uint16_t sum = 0U;
    data[0] = 0x20U; data[1] = 0x40U;
    for (i = 0U; i < 14U; ++i) { data[2U + 2U*i] = (uint8_t)channels[i]; data[3U + 2U*i] = (uint8_t)(channels[i] >> 8U); }
    for (i = 0U; i < 30U; ++i) { sum = (uint16_t)(sum + data[i]); }
    sum = (uint16_t)(0xFFFFU - sum); data[30] = (uint8_t)sum; data[31] = (uint8_t)(sum >> 8U);
}
static void Frame(void)
{ uint8_t data[32]; now += 7U; FrameBytes(data); Burst(data, sizeof(data)); Poll(); }
static void Safety(void)
{
    SafetyInput input;
    memset(&input, 0, sizeof(input)); Read();
    memcpy(input.remote.channels, remote.channels, sizeof(remote.channels));
    input.remote.has_frame = remote.has_frame; input.remote.sequence = remote.valid_frames;
    input.remote.received_ms = remote.received_ms;
    input.uart_epoch = remote.fault_epoch; input.now_ms = now;
    input.backend_ready = input.feedback_ready = 1U;
    AppLogic_Step(&logic, &input, 0U);
}
void test_run(void)
{
    uint32_t i, before, epoch, timestamp;
    uint8_t data[64];
    test_stage = 1U;
    CHECK(1U && 1U && 64U == 64U);
    for (i = 0U; i < 14U; ++i) { channels[i] = 1500U; }
    channels[4] = channels[5] = 1000U;
    RemoteInput_CreateQueue(); RemoteInput_Start(); Read();
    CHECK(rearms == 1U && dma_data && !remote.has_frame && remote.fault_epoch == 0U);
    FrameBytes(data); memcpy(dma_data, data, 32U); now += 2U;
    RemoteInput_RxEvent(&huart4, 32U); Poll();
    CHECK(remote.has_frame && remote.received_ms == now && remote.valid_frames == 1U);
    CHECK(remote.rx_bytes == 32U && remote.rx_bursts == 1U && remote.fault_epoch == 0U);
    before = remote.valid_frames;
    for (i = 0U; i < 14U; ++i) { CHECK(remote.channels[i] == channels[i]); }
    now += 7U; Burst(data, 9U); Poll(); CHECK(remote.valid_frames == before);
    Burst(data + 9U, 23U); Poll(); CHECK(remote.valid_frames == before + 1U);
    memcpy(data + 32U, data, 32U); now += 7U; Burst(data, 64U); Poll();
    CHECK(remote.valid_frames == before + 3U);
    before = remote.rx_bytes; RemoteInput_RxEvent(&huart1, 32U);
    RemoteInput_Error(&huart1); Read(); CHECK(remote.rx_bytes == before && remote.fault_epoch == 0U);

    test_stage = 2U;
    AppLogic_Init(&logic); Frame(); Safety(); channels[4] = 2000U; Frame(); Safety();
    CHECK(logic.safety.operator_armed && logic.safety.output_allowed);
    /* A UART error invalidates the published frame even before parser polling. */
    FrameBytes(data); Burst(data, 12U); Poll(); epoch = remote.fault_epoch;
    huart4.error = 8U; RemoteInput_Error(&huart4); Read();
    CHECK(!remote.has_frame && remote.fault_epoch == epoch + 1U && remote.last_uart_error == 8U);
    CHECK(remote.rx_errors == 1U && remote.needs_rearm && aborts == 1U);
    Safety(); CHECK(!logic.safety.operator_armed && logic.safety.reason == SAFETY_UART);
    Poll(); CHECK(!remote.needs_rearm && !remote.has_frame && remote.last_uart_error == 8U);
    Burst(data + 12U, 20U); Poll(); CHECK(!remote.has_frame);
    Frame(); Safety(); CHECK(remote.has_frame && !logic.safety.operator_armed);
    Frame(); Safety(); CHECK(!logic.safety.operator_armed);
    channels[4] = 1000U; Frame(); Safety(); channels[4] = 2000U; Frame(); Safety();
    CHECK(logic.safety.operator_armed && logic.safety.output_allowed);

    test_stage = 3U;
    /* Entire pre-fault backlog is rejected; recovery cannot reuse old permission. */
    FrameBytes(data); memcpy(data + 32U, data, 32U);
    for (i = 0U; i < 4U; ++i) { Burst(data, 64U); }
    Read(); epoch = remote.fault_epoch; before = remote.valid_frames; Burst(data, 64U); Read();
    CHECK(remote.rx_drops == 1U && remote.fault_epoch == epoch + 1U && !remote.has_frame);
    Poll(); CHECK(remote.valid_frames == before && !remote.has_frame && count == 0U);
    Safety(); CHECK(!logic.safety.operator_armed);
    Frame(); Safety(); CHECK(remote.has_frame && !logic.safety.operator_armed);
    /* Timestamp records receive, never parser drain time. */
    FrameBytes(data); now += 7U; timestamp = now; Burst(data, 32U);
    now += REMOTE_TIMEOUT_MS; before = remote.valid_frames; Poll();
    CHECK(remote.valid_frames == before && remote.received_ms != now && now - timestamp == REMOTE_TIMEOUT_MS);
    Safety(); CHECK(!logic.safety.operator_armed);

    test_stage = 4U;
    fail_rearm = 1U; epoch = remote.fault_epoch; now += 7U; Burst(data, 32U); Read();
    CHECK(remote.fault_epoch == epoch + 1U && remote.rearm_errors == 1U && remote.needs_rearm);
    Poll(); CHECK(remote.needs_rearm && !remote.has_frame);
    fail_rearm = 0U; Poll(); CHECK(!remote.needs_rearm && !remote.has_frame);
    Frame(); Safety(); CHECK(remote.has_frame && !logic.safety.operator_armed);
    /* Bad checksum and partial bursts resynchronise only on a validated frame. */
    FrameBytes(data); data[31] ^= 1U; before = remote.valid_frames; now += 7U;
    Burst(data, 32U); Poll(); CHECK(remote.valid_frames == before && remote.bad_frames >= 1U);
    Frame(); CHECK(remote.valid_frames == before + 1U);
    epoch = remote.fault_epoch; RemoteInput_RxEvent(&huart4, 0U); Read();
    CHECK(remote.fault_epoch == epoch + 1U && remote.needs_rearm && !remote.has_frame); Poll();
    epoch = remote.fault_epoch; RemoteInput_RxEvent(&huart4, 65U); Read();
    CHECK(remote.fault_epoch == epoch + 1U && remote.needs_rearm); Poll();
    huart4.error = 6U; RemoteInput_Error(&huart4); Read();
    CHECK(remote.last_uart_error == 6U && remote.rx_errors == 2U); Poll(); Safety();
    channels[4] = 1000U; Frame(); Safety(); channels[4] = 2000U; Frame(); Safety();
    CHECK(logic.safety.operator_armed && logic.safety.output_allowed);
    test_done = 1U;
}
