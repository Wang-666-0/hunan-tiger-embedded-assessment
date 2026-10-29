/* 遥控传输与快照 */
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
void RemoteInput_RxEvent(UART_HandleTypeDef *huart, uint16_t size);
void RemoteInput_Error(UART_HandleTypeDef *huart);
#endif
