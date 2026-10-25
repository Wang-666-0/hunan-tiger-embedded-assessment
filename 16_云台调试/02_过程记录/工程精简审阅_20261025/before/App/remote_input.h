/**
 * @file remote_input.h
 * @brief 遥控传输与快照
 * 职责：声明 UART4 DMA/字节回调、接收轮询、错误统计和只读状态。
 * 主要接口：RemoteInput_CreateQueue, RemoteInput_Start, RemoteInput_Poll, RemoteInput_GetState, RemoteInput_RxCplt 等。
 * 实现/调用：remote_input.c；错误先撤权，恢复接收后仍需新安全帧解锁。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H
#include "app_config.h"
#include "usart.h"
#include <stdint.h>
typedef struct {
    uint16_t channels[14];
    uint32_t received_ms, valid_frames, bad_frames, partial_timeouts;
    uint32_t fault_epoch, rx_bytes, rx_drops, rx_errors, last_uart_error;
    uint32_t rx_bursts, rearm_errors;
    uint8_t has_frame, last_rx_byte;
    uint8_t needs_rearm;
} RemoteInput_State;
void RemoteInput_CreateQueue(void);
void RemoteInput_Start(void);
void RemoteInput_Poll(void);
void RemoteInput_GetState(RemoteInput_State *out);
void RemoteInput_RxCplt(UART_HandleTypeDef *huart);
void RemoteInput_RxEvent(UART_HandleTypeDef *huart, uint16_t size);
void RemoteInput_Error(UART_HandleTypeDef *huart);
#endif
