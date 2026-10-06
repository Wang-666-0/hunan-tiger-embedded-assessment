#include "uart_service.h"
#include "command_parser.h"
#include "imu_acquisition.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>

static QueueHandle_t uart_rx_queue;
static uint8_t uart_rx_byte;
static SemaphoreHandle_t uart_tx_mutex;
volatile uint32_t uart_rx_dropped = 0;

void UartService_Init(void)
{
    uart_tx_mutex = xSemaphoreCreateMutex();
    uart_rx_queue = xQueueCreate(128, sizeof(uint8_t));
    if (uart_tx_mutex == NULL || uart_rx_queue == NULL)
        Error_Handler();
    /* 接收中断在 UartService_Task 中启动，确保队列和调度器已准备好。 */
}

HAL_StatusTypeDef UartSend(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size,
                           uint32_t timeout)
{
    HAL_StatusTypeDef result;

    /* 同一 UART 被 IMU 错误提示和 UART 任务共用，用锁防止帧交叉。
     * 锁等待为 RTOS tick；HAL 发送 timeout 的单位为 ms。本函数仅供任务调用。 */
    if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return HAL_TIMEOUT;
    }

    result = HAL_UART_Transmit(huart, data, size, timeout);

    xSemaphoreGive(uart_tx_mutex);

    return result;
}

void UartService_Task(void *argument)
{
    (void)argument;
    uint8_t received_byte;
    char line[32];
    uint32_t length = 0;
    uint8_t overflow = 0;
    ImuMessage imu;
    char text[128];
    uint32_t last_print_tick = HAL_GetTick();

    if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1) != HAL_OK)
        Error_Handler();

    for (;;)
    {

        /* 一轮最多消费 32 字节，避免命令流长期占住任务、推迟曲线发送。 */
        for (uint32_t i = 0; i < 32; i++)
        {
            if (xQueueReceive(uart_rx_queue, &received_byte, 0) != pdPASS)
                break;
            /* LF 完成一条命令；CR 忽略，兼容 LF 和 CRLF。溢出后丢到 LF。 */
            if (received_byte == '\n')
            {
                if (overflow)
                {
                    const char reply[] = "ERR line too long\r\n";
                    UartSend(&huart1, (uint8_t *)reply, sizeof(reply) - 1, 100);
                }
                else if (length > 0)
                {
                    line[length] = '\0';
                    CommandParser_Process(line);
                }
                length = 0;
                overflow = 0;
            }
            else if (received_byte == '\r')
            {
            }
            else if (!overflow)
            {
                if (length < sizeof(line) - 1)
                    line[length++] = (char)received_byte;
                else
                    overflow = 1;
            }
        }

        if (HAL_GetTick() - last_print_tick >= 20)
        {
            last_print_tick = HAL_GetTick();
            if (ImuAcquisition_TakeLatest(&imu) == pdPASS)
            {
                /* FireWater 数值顺序：ax,ay,az,gx,gy,gz,采样数,读取错误数。 */
                int text_length =
                    snprintf(text, sizeof(text), "imu:%d,%d,%d,%d,%d,%d,%lu,%lu\r\n", (int)imu.ax,
                             (int)imu.ay, (int)imu.az, (int)imu.gx, (int)imu.gy, (int)imu.gz,
                             (unsigned long)imu.sample_count, (unsigned long)imu.read_errors);
                /* snprintf 返回所需长度；截断时返回值也可能大于缓冲区，必须检查。 */
                if (text_length > 0 && text_length < (int)sizeof(text))
                    UartSend(&huart1, (uint8_t *)text, (uint16_t)text_length, 20);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;

        /* 中断不能等待互斥锁或阻塞队列，只复制字节给 UART 任务处理。 */
        if (xQueueSendFromISR(uart_rx_queue, &uart_rx_byte, &higher_priority_task_woken) != pdPASS)
        {
            uart_rx_dropped++;
        }

        /* 单字节中断接收完成后，立即重新挂接下一字节。 */
        if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1) != HAL_OK)
        {
            Error_Handler();
        }

        /* 若唤醒更高优先级任务，退出中断后立即切换；不是在中断中跑任务。 */
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
