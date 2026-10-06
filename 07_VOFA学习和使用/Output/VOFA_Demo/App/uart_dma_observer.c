/* DMA 回调观察模块：保留原有调试变量与回调。
 * 当前主程序没有开启 ReceiveToIdle DMA，这些接收标志仅在启用后更新。
 * 回调只记录状态；不要在中断里进行长时间打印或等待。
 */
#include "uart_dma_observer.h"
#include "usart.h"

uint8_t uart_buffer[256];

volatile uint16_t rx_length = 0;
volatile uint8_t rx_ready = 0;
volatile uint8_t tx_done = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        rx_length = Size;
        rx_ready = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_done = 1;
    }
}
