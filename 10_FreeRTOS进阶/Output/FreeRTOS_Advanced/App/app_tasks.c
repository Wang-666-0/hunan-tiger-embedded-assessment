#include "app_tasks.h"
#include "pwm_control.h"
#include "uart_service.h"
#include "imu_acquisition.h"
#include "main.h"
#include "task.h"

void AppTasks_Init(void)
{
    /* 先创建全部共享资源，再创建任务；MX_FREERTOS_Init 在启动调度器前调用。 */
    UartService_Init();
    PwmControl_Init();
    ImuAcquisition_Init();

    /* xTaskCreate 栈参数单位是 word，STM32 上 1 word=4 字节。
     * PWM 优先级 4，IMU 3，UART 2；每个任务主动阻塞，让其他任务运行。
     */
    if (xTaskCreate(PwmControl_Task, "PWM", 256, NULL, 4, NULL) != pdPASS)
        Error_Handler();
    if (xTaskCreate(UartService_Task, "UART", 384, NULL, 2, NULL) != pdPASS)
        Error_Handler();
    if (xTaskCreate(ImuAcquisition_Task, "IMU", 512, NULL, 3, NULL) != pdPASS)
        Error_Handler();
}
