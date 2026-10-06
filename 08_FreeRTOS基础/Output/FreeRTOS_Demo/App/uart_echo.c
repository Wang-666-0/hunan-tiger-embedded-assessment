#include "uart_echo.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>

void UartEcho_Task(void *argument)
{
    (void)argument;
    uint8_t rx_byte;
    uint8_t welcome[] = "INFO FreeRTOS UART ready\r\n";
    char firewater_text[16];

    HAL_UART_Transmit(&huart1, welcome, sizeof(welcome) - 1, 100);

    for (;;)
    {
        /* 尝试接收一个字节，最多等待 1 ms */
        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 1) == HAL_OK)
        {
            /* Send the received byte as one numeric FireWater channel. */
            /* snprintf 返回“本来需要的长度”，可能大于缓冲区。
             * 必须同时检查 length > 0 和 length < sizeof(buffer)，再交给 HAL。
             */
            int length = snprintf(firewater_text, sizeof(firewater_text), "rx:%u\r\n",
                                  (unsigned int)rx_byte);
            if (length > 0 && length < (int)sizeof(firewater_text))
            {
                HAL_UART_Transmit(&huart1, (uint8_t *)firewater_text, (uint16_t)length, 10);
            }
        }

        /* 让出 CPU，其他任务可以运行 */
        osDelay(1);
    }
}
