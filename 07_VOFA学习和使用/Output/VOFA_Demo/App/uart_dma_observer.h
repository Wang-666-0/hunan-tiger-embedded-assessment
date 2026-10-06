#ifndef UART_DMA_OBSERVER_H
#define UART_DMA_OBSERVER_H

#include <stdint.h>

extern uint8_t uart_buffer[256];
extern volatile uint16_t rx_length;
extern volatile uint8_t rx_ready;
extern volatile uint8_t tx_done;

#endif /* UART_DMA_OBSERVER_H */
