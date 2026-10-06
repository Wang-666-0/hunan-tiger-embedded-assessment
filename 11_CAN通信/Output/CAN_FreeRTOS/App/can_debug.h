#ifndef CAN_DEBUG_H
#define CAN_DEBUG_H

#include "can_protocol.h"

/* 可在 Keil Watch 中通过 can_fault_code 定位停止原因。 */
#define CAN_FAULT_QUEUE 1U
#define CAN_FAULT_RX_TASK 2U
#define CAN_FAULT_TX_TASK 3U
#define CAN_FAULT_DEFAULT 4U
#define CAN_FAULT_FILTER1 11U
#define CAN_FAULT_FILTER2 12U
#define CAN_FAULT_START1 13U
#define CAN_FAULT_START2 14U
#define CAN_FAULT_NOTIFY1 15U
#define CAN_FAULT_NOTIFY2 16U
#define CAN_FAULT_STACK 21U
#define CAN_FAULT_MALLOC 22U

extern volatile uint32_t can_ready;
extern volatile uint32_t can_fault_code;
extern volatile uint32_t can_tx_task_cycle_count;
extern volatile uint32_t can1_tx_queued_count;
extern volatile uint32_t can2_tx_queued_count;
extern volatile uint32_t can1_tx_count;
extern volatile uint32_t can2_tx_count;
extern volatile uint32_t can1_tx_busy_count;
extern volatile uint32_t can2_tx_busy_count;
extern volatile uint32_t can1_tx_submit_error_count;
extern volatile uint32_t can2_tx_submit_error_count;
extern volatile uint32_t can1_rx_count;
extern volatile uint32_t can2_rx_count;
extern volatile uint32_t can1_rx_valid_count;
extern volatile uint32_t can2_rx_valid_count;
extern volatile uint32_t can1_rx_invalid_count;
extern volatile uint32_t can2_rx_invalid_count;
extern volatile uint32_t can1_last_rx_id;
extern volatile uint32_t can2_last_rx_id;
extern volatile uint8_t can1_last_rx_dlc;
extern volatile uint8_t can2_last_rx_dlc;
extern volatile uint8_t can1_last_rx_data[8];
extern volatile uint8_t can2_last_rx_data[8];
extern volatile uint32_t can1_last_rx_sequence;
extern volatile uint32_t can2_last_rx_sequence;
extern volatile uint32_t can_rx_read_error_count;
extern volatile uint32_t can_rx_queue_full_count;
extern volatile uint32_t can_rx_ignored_count;
extern volatile uint32_t can1_error_callback_count;
extern volatile uint32_t can2_error_callback_count;
extern volatile uint32_t can1_last_hal_error;
extern volatile uint32_t can2_last_hal_error;
extern volatile uint32_t can1_error_status;
extern volatile uint32_t can2_error_status;
extern volatile uint32_t can1_bus_off;
extern volatile uint32_t can2_bus_off;
extern volatile uint32_t can1_free_mailboxes;
extern volatile uint32_t can2_free_mailboxes;

void CanFail(uint32_t code);
void CanDebug_SampleHardware(void);
void CanDebug_RecordRx(const CanRxMessage *message);

#endif /* CAN_DEBUG_H */
