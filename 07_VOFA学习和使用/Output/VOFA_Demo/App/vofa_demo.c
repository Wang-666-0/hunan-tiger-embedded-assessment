#include "vofa_demo.h"
#include "usart.h"

/* FireWater：前缀 + 冒号 + 逗号分隔的数值 + 换行。
 * sizeof(...)-1 去掉 C 字符串的结尾 '\0'；它不属于发送协议。
 */
uint8_t vofa_data_1[] = "channels:10,90\r\n";
uint8_t vofa_data_2[] = "channels:90,10\r\n";

void VofaDemo_RunCycle(void)
{
    /* 保留原来的阻塞式演示：两个通道每 500 ms 交换数值。
     * DMA 的硬件配置仍在 dma.c；此处原来使用的就是 HAL_UART_Transmit。
     */
    HAL_UART_Transmit(&huart1, vofa_data_1, sizeof(vofa_data_1) - 1, 100);
    HAL_Delay(500);
    HAL_UART_Transmit(&huart1, vofa_data_2, sizeof(vofa_data_2) - 1, 100);
    HAL_Delay(500);
}
