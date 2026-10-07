#ifndef CAN_DEBUG_H
#define CAN_DEBUG_H

#include "can_protocol.h"
#include "app_config.h"

/* 非零故障码表示初始化/RTOS 创建失败，可在 Error_Handler 暂停后观察。 */
#define CAN_FAULT_QUEUE 1U
#define CAN_FAULT_RX_TASK 2U
#define CAN_FAULT_CONTROL_TASK 3U
#define CAN_FAULT_DEFAULT 4U
#define CAN_FAULT_CONSOLE_TASK 5U
#define CAN_FAULT_UART_QUEUE 6U
#define CAN_FAULT_FILTER1 11U
#define CAN_FAULT_START1 13U
#define CAN_FAULT_NOTIFY1 15U
#define CAN_FAULT_UART_START 17U
#define CAN_FAULT_STACK 21U
#define CAN_FAULT_MALLOC 22U

extern volatile uint32_t can_ready;
extern volatile uint32_t can_fault_code;
extern volatile uint32_t can1_tx_queued_count;
extern volatile uint32_t can1_tx_count;
extern volatile uint32_t can1_tx_busy_count;
extern volatile uint32_t can1_tx_abort_count;
extern volatile uint32_t can1_tx_submit_error_count;
extern volatile uint32_t can1_rx_count;
extern volatile uint32_t can1_rx_valid_count;
extern volatile uint32_t can1_rx_invalid_count;
#if APP_ENABLE_DIAGNOSTICS
extern volatile uint32_t can1_last_rx_id;
extern volatile uint8_t can1_last_rx_dlc;
extern volatile uint8_t can1_last_rx_data[8];
#endif
extern volatile uint32_t can_rx_read_error_count;
extern volatile uint32_t can_rx_queue_full_count;
extern volatile uint32_t can_rx_ignored_count;
extern volatile uint32_t can1_error_callback_count;
extern volatile uint32_t can1_last_hal_error;
#if APP_ENABLE_DIAGNOSTICS
extern volatile uint32_t can1_error_status;
extern volatile uint32_t can1_bus_off;
extern volatile uint32_t can1_free_mailboxes;
/* 栈剩余最小值，单位 word，用于验收运行时的栈余量。 */
extern volatile uint32_t can_rx_stack_free_words;
extern volatile uint32_t motor_control_stack_free_words;
extern volatile uint32_t motor_console_stack_free_words;
#endif

void CanFail(uint32_t code);
#if APP_ENABLE_DIAGNOSTICS
void CanDebug_SampleHardware(void);
/* 可选记录原始帧；不解析电机、不改变反馈或收发计数。 */
void CanDebug_RecordRx(const CanRxMessage *message);
#endif

#endif /* CAN_DEBUG_H */
