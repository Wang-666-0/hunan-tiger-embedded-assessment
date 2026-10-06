#include "imu_acquisition.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "uart_service.h"
#include "task.h"
#include "queue.h"

static QueueHandle_t imu_data_queue;

void ImuAcquisition_Init(void)
{
    imu_data_queue = xQueueCreate(1, sizeof(ImuMessage));
    if (imu_data_queue == NULL)
        Error_Handler();
}

BaseType_t ImuAcquisition_TakeLatest(ImuMessage *message)
{
    /* 没有新数据时立即返回，不阻塞 UART 的命令处理。 */
    return xQueueReceive(imu_data_queue, message, 0);
}

void ImuAcquisition_Task(void *argument)
{
    (void)argument;

    /* HAL 要求左移后的地址；0x68 是 MPU6050 的 7 位地址。 */
    const uint16_t address = 0x68 << 1;
    uint8_t data[14];
    uint8_t value;
    uint8_t id = 0;

    uint32_t sample_count = 0;
    uint32_t read_errors = 0;

    vTaskDelay(pdMS_TO_TICKS(100));

    if (HAL_I2C_Mem_Read(&hi2c1, address, 0x75, I2C_MEMADD_SIZE_8BIT, &id, 1, 20) != HAL_OK ||
        id != 0x68)
    {
        const char message[] = "ERR IMU identity\r\n";

        UartSend(&huart1, (uint8_t *)message, sizeof(message) - 1, 100);

        /* 身份检查失败只挂起采集任务，让呼吸灯和串口命令继续工作。 */
        vTaskSuspend(NULL);
    }

    /* PWR_MGMT_1：退出睡眠并使用 X 轴陀螺仪 PLL 作时钟。 */
    value = 0x01;

    if (HAL_I2C_Mem_Write(&hi2c1, address, 0x6B, I2C_MEMADD_SIZE_8BIT, &value, 1, 20) != HAL_OK)
    {
        Error_Handler();
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* CONFIG=3 开启数字低通；SMPLRT_DIV=9：1 kHz/(1+9)=100 Hz。
     * GYRO_CONFIG=0：±250 deg/s；ACCEL_CONFIG=0：±2 g。
     * 本工程输出原始 int16 数据，不在这里换算物理单位。 */
    const uint8_t registers[] = {0x1A, 0x19, 0x1B, 0x1C};
    const uint8_t settings[] = {0x03, 9, 0x00, 0x00};

    for (uint32_t i = 0; i < 4; i++)
    {
        value = settings[i];

        if (HAL_I2C_Mem_Write(&hi2c1, address, registers[i], I2C_MEMADD_SIZE_8BIT, &value, 1, 20) !=
            HAL_OK)
        {
            Error_Handler();
        }
    }

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        if (HAL_I2C_Mem_Read(&hi2c1, address, 0x3B, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 5) ==
            HAL_OK)
        {
            /* 连读 14 字节：加速度 0..5、温度 6..7、陀螺仪 8..13。
             * 每轴高字节在前；先用无符号数拼接，再转有符号 int16。 */
            int16_t ax = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
            int16_t ay = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
            int16_t az = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

            int16_t gx = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
            int16_t gy = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
            int16_t gz = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

            sample_count++;

            ImuMessage message = {.ax = ax,
                                  .ay = ay,
                                  .az = az,
                                  .gx = gx,
                                  .gy = gy,
                                  .gz = gz,
                                  .sample_count = sample_count,
                                  .read_errors = read_errors};

            /* 只保留最新样本；UART 每 20 ms 发送，IMU 仍每 10 ms 读取。
             * 覆盖写只能用于长度为 1 的队列，且复制结构体而非保存指针。 */
            xQueueOverwrite(imu_data_queue, &message);
        }
        else
        {
            read_errors++;
        }

        /* 初始化之后建立基准；按 10 ms 绝对周期读取。 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
