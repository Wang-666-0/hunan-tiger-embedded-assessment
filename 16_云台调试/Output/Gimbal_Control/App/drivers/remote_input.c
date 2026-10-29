/**
 * @file remote_input.c
 * 作用：UART4 遥控接收与有效帧发布
 * 对外函数：
 *   RemoteInput_Poll / RemoteInput_RxEvent / RemoteInput_Start / RemoteInput_CreateQueue /
 *   RemoteInput_GetState / RemoteInput_Error
 * 内部函数：
 *   StartRemoteRx / RemoteInput_SyncFault
 * 文件级变量（函数局部变量在各函数内定义）：
 *   rx_queue：接收队列，连接中断生产者与任务消费者。
 *   parser：iBUS 持续解析器及有效帧统计状态。
 *   state：本模块维护并向外发布的设备或流程状态快照。
 *   fault_epoch：累计接收/发送故障代次，用于隔离旧数据。
 *   rx_bytes：累计接收字节数。
 *   rx_drops：接收队列丢字节计数。
 *   rx_errors：累计串口接收错误数。
 *   last_uart_error：最近一次串口硬件错误码。
 *   rx_bursts：累计 DMA 接收批次数。
 *   rearm_errors：重新启动遥控接收失败的计数。
 *   needs_rearm：接收需要重新启动的标记。
 *   last_rx_byte：最近一个接收字节的时间戳。
 *   observed_epoch：任务已处理的故障代次。
 *   rx_dma：DMA 接收缓冲区。
 */
#include "remote_input.h"
#include "ibus.h"
#include "app_config.h"
#include "app_fault.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

typedef struct { uint32_t received_ms, epoch; uint8_t byte; } RemoteByte;
static QueueHandle_t rx_queue;
static ibus_parser_t parser;
static RemoteInput_State state;
static volatile uint32_t fault_epoch, rx_bytes, rx_drops, rx_errors, last_uart_error;
static volatile uint32_t rx_bursts, rearm_errors;
static volatile uint8_t needs_rearm, last_rx_byte;
static uint32_t observed_epoch;
static uint8_t rx_dma[64U];

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static HAL_StatusTypeDef StartRemoteRx(void);
static void RemoteInput_SyncFault(void);

void RemoteInput_Poll(void)
/*
 * 作用：恢复遥控接收并解析队列字节，只发布通过校验的新鲜遥控帧。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    RemoteByte item;
    ibus_frame_t frame;
    RemoteInput_SyncFault();
    if (needs_rearm) {
        taskENTER_CRITICAL();
        (void)HAL_UART_AbortReceive(&huart4);
        if (StartRemoteRx() == HAL_OK) { needs_rearm = 0U; }
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
            if (item.epoch == fault_epoch) {
                memcpy(state.channels, frame.channels, sizeof(state.channels));
                state.received_ms = frame.received_ms;
                state.valid_frames = parser.stats.valid_frames;
                state.has_frame = 1U;
            }
            taskEXIT_CRITICAL();
        }
    }
    (void)ibus_poll_timeout(&parser, HAL_GetTick());
    taskENTER_CRITICAL();
    state.bad_frames = parser.stats.bad_frames;
    state.partial_timeouts = parser.stats.partial_timeouts;
    taskEXIT_CRITICAL();
}

void RemoteInput_RxEvent(UART_HandleTypeDef *huart, uint16_t size)
/*
 * 作用：把一批遥控 DMA 字节附加时间和故障代次后入队。
 * 参数 huart：产生事件的 UART 句柄。
 * 参数 size：DMA 本批接收字节数。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    RemoteByte item;
    uint16_t i;
    BaseType_t woken = pdFALSE;
    if (huart != &huart4 || HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    /* HT is disabled; normal DMA is stopped by HAL before IDLE/TC callback.
     * Copy a stable bounded burst. Parsing and safety remain in their tasks. */
    if (!size || size > sizeof(rx_dma)) { fault_epoch++; needs_rearm = 1U; return; }
    rx_bursts++; rx_bytes += size; last_rx_byte = rx_dma[size - 1U];
    item.received_ms = HAL_GetTick(); item.epoch = fault_epoch;
    for (i = 0U; i < size; ++i) {
        item.byte = rx_dma[i];
        if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) {
            rx_drops++; fault_epoch++; break;
        }
    }
    if (StartRemoteRx() != HAL_OK) { rearm_errors++; fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
}

void RemoteInput_Start(void)
/*
 * 作用：启动遥控接收，失败时记录错误并安排恢复。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (StartRemoteRx() != HAL_OK) { AppFault_Fatal(APP_FAULT_UART_START); }
}

void RemoteInput_CreateQueue(void)
/*
 * 作用：创建遥控字节队列并初始化解析器与接收状态。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    rx_queue = xQueueCreate(REMOTE_RX_QUEUE_LENGTH, sizeof(RemoteByte));
    if (rx_queue == NULL) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "IBUS_BYTES");
    ibus_init(&parser);
    memset(&state, 0, sizeof(state));
}

void RemoteInput_GetState(RemoteInput_State *out)
/*
 * 作用：复制当前遥控通道、有效帧时间和接收故障统计。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值，将当前快照写入输出参数；不会启动新的运动。
 */
{
    if (out == NULL) { return; }
    taskENTER_CRITICAL();
    *out = state;
    /* 中断故障无需等解析任务执行即可撤销控制许可。 */
    out->fault_epoch = fault_epoch; out->rx_bytes = rx_bytes;
    out->rx_drops = rx_drops; out->rx_errors = rx_errors;
    out->last_uart_error = last_uart_error;
    out->last_rx_byte = last_rx_byte;
    out->rx_bursts = rx_bursts; out->rearm_errors = rearm_errors;
    out->needs_rearm = needs_rearm;
    if (observed_epoch != fault_epoch) { out->has_frame = 0U; }
    taskEXIT_CRITICAL();
}

static HAL_StatusTypeDef StartRemoteRx(void)
/*
 * 作用：按配置启动 UART4 的 DMA 或单字节中断接收。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    return RemoteUart_StartReceive(rx_dma, sizeof(rx_dma));
}

void RemoteInput_Error(UART_HandleTypeDef *huart)
/*
 * 作用：记录遥控串口错误，撤销旧数据可信度并安排重新接收。
 * 参数 huart：产生事件的 UART 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (huart != &huart4) { return; }
    /* 后续 HAL 错误分支或重启接收会清除错误码，在恢复前保存诊断。 */
    last_uart_error = HAL_UART_GetError(huart);
    rx_errors++; fault_epoch++;
    (void)HAL_UART_AbortReceive(huart);
    needs_rearm = 1U;
}

static void RemoteInput_SyncFault(void)
/*
 * 作用：同步接收故障代次，使旧队列字节不能重新形成有效遥控输入。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (observed_epoch != fault_epoch) {
        observed_epoch = fault_epoch;
        parser.buffered = 0U; parser.has_frame = 0U;
        taskENTER_CRITICAL(); state.has_frame = 0U; taskEXIT_CRITICAL();
    }
}
