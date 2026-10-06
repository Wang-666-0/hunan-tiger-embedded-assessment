#include "imu_telemetry.h"
#include "attitude_filter.h"
#include "usart.h"
#include <stdio.h>

char uart_text[128];
uint32_t last_print_tick = 0;

void ImuTelemetry_ReportInit(uint8_t dmp_result)
{
    int len = snprintf(uart_text, sizeof(uart_text), "INFO DMP init result=%u\r\n",
                       (unsigned int)dmp_result);

    if (len > 0 && len < (int)sizeof(uart_text))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_text, (uint16_t)len, 100);
    }
}

void ImuTelemetry_Process(void)
{
    /* 获取样本和发送曲线是两种频率：发送较慢不应降低传感器读取频率。
     * 使用无符号 tick 差值，跨 HAL_GetTick 的自然回绕也可正常判断间隔。
     */
    /* 每 100 ms 打印一次，读取数据仍然保持较高频率 */
    if (attitude_ready && HAL_GetTick() - last_print_tick >= 100)
    {
        last_print_tick = HAL_GetTick();

        int len = snprintf(uart_text, sizeof(uart_text), "attitude:%.2f,%.2f\r\n", (double)roll_deg,
                           (double)pitch_deg);

        if (len > 0 && len < (int)sizeof(uart_text))
        {
            HAL_UART_Transmit(&huart1, (uint8_t *)uart_text, (uint16_t)len, 100);
        }
    }
}
