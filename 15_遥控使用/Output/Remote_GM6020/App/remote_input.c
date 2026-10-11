/* UART4 ISR 只搬运字节和实际时间；独立任务完成 iBUS 校验。
 * 丢字节/USART 错误产生故障代次：旧队列字节不能复活旧许可。
 */
#include "remote_input.h"
#include "ibus.h"
#include "app_config.h"
#include "can_debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

typedef struct { uint32_t received_ms, epoch; uint8_t byte; } RemoteByte;
static QueueHandle_t rx_queue;
static ibus_parser_t parser;
static RemoteInput_State state;
static volatile uint32_t fault_epoch, rx_bytes, rx_drops, rx_errors, last_uart_error;
static volatile uint8_t needs_rearm, last_rx_byte;
static uint32_t observed_epoch;
static uint8_t rx_byte;

void RemoteInput_CreateQueue(void)// 创建接收队列和 iBUS 解析器
{
    rx_queue = xQueueCreate(REMOTE_RX_QUEUE_LENGTH, sizeof(RemoteByte));
    if (rx_queue == NULL) { CanFail(CAN_FAULT_UART_QUEUE); }
    vQueueAddToRegistry(rx_queue, "IBUS_BYTES");
    ibus_init(&parser);
    memset(&state, 0, sizeof(state));
}
void RemoteInput_Start(void)// 启动 UART 接收中断
{
    if (HAL_UART_Receive_IT(&huart4, &rx_byte, 1U) != HAL_OK) { CanFail(CAN_FAULT_UART_START); }
}
void RemoteInput_RxCplt(UART_HandleTypeDef *huart)// UART 接收完成中断回调
{
    RemoteByte item;
    BaseType_t woken = pdFALSE;
    if (huart != &huart4) { return; }
    rx_bytes++;
    last_rx_byte = rx_byte;
    /* HAL 的错误 IRQ 可能先调用 RxCplt、随后才调用 ErrorCallback。
     * 错误字节不入队，也不能在这里重启接收（重启会清掉 ErrorCode）。
     * 交给随后执行的 ErrorCallback 保存错误、撤销许可并安排任务恢复。 */
    if (HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    item.byte = rx_byte; item.received_ms = HAL_GetTick(); item.epoch = fault_epoch;
    if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) { rx_drops++; fault_epoch++; }
    if (HAL_UART_Receive_IT(&huart4, &rx_byte, 1U) != HAL_OK) { fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
}
void RemoteInput_Error(UART_HandleTypeDef *huart)// UART 错误中断回调
{
    if (huart != &huart4) { return; }
    /* 后续 HAL 错误分支或重启接收会清除错误码，在恢复前保存诊断。 */
    last_uart_error = HAL_UART_GetError(huart);
    rx_errors++; fault_epoch++;
    (void)HAL_UART_AbortReceive(huart);
    needs_rearm = 1U;
}
static void RemoteInput_SyncFault(void)
{
    if (observed_epoch != fault_epoch) {
        observed_epoch = fault_epoch;
        parser.buffered = 0U; parser.has_frame = 0U;
        taskENTER_CRITICAL(); state.has_frame = 0U; taskEXIT_CRITICAL();
    }
}
void RemoteInput_Poll(void)// 轮询处理接收队列和 iBUS 解析
{
    RemoteByte item;
    ibus_frame_t frame;
    RemoteInput_SyncFault();
    if (needs_rearm) {
        taskENTER_CRITICAL();
        (void)HAL_UART_AbortReceive(&huart4);
        if (HAL_UART_Receive_IT(&huart4, &rx_byte, 1U) == HAL_OK) { needs_rearm = 0U; }
        taskEXIT_CRITICAL();
    }
    for (uint32_t i = 0U; i < REMOTE_RX_QUEUE_LENGTH; i++) {
        if (xQueueReceive(rx_queue, &item, 0U) != pdPASS) { break; }
        RemoteInput_SyncFault();
        if (item.epoch != observed_epoch) { continue; }
        /* 队列积压的旧帧只允许被丢弃，不能当成新的控制指令。 */
        if ((uint32_t)(HAL_GetTick() - item.received_ms) >= REMOTE_TIMEOUT_MS) {
            parser.buffered = 0U;
            continue;
        }
        if (ibus_feed_byte(&parser, item.byte, item.received_ms, &frame)) {
            taskENTER_CRITICAL();
            memcpy(state.channels, frame.channels, sizeof(state.channels));
            state.received_ms = frame.received_ms;
            state.valid_frames = parser.stats.valid_frames;
            state.has_frame = 1U;
            taskEXIT_CRITICAL();
        }
    }
    (void)ibus_poll_timeout(&parser, HAL_GetTick());
    taskENTER_CRITICAL();
    state.bad_frames = parser.stats.bad_frames;
    state.partial_timeouts = parser.stats.partial_timeouts;
    taskEXIT_CRITICAL();
}
void RemoteInput_GetState(RemoteInput_State *out)// 获取当前遥控输入状态
{
    if (out == NULL) { return; }
    taskENTER_CRITICAL();
    *out = state;
    /* 中断故障无需等解析任务执行即可撤销控制许可。 */
    out->fault_epoch = fault_epoch; out->rx_bytes = rx_bytes;
    out->rx_drops = rx_drops; out->rx_errors = rx_errors;
    out->last_uart_error = last_uart_error;
    out->last_rx_byte = last_rx_byte;
    taskEXIT_CRITICAL();
}
