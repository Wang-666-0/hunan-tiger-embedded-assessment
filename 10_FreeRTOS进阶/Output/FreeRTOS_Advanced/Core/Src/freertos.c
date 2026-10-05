/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "queue.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include "i2c.h"
#include "semphr.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    PWM_SET_MAX_BRIGHTNESS,  /* 设置最大亮度 */
    PWM_SET_BREATH_PERIOD    /* 设置完整呼吸周期 */
} PwmCommandType;

typedef struct
{
    PwmCommandType type;
    uint32_t value;
} PwmCommand;

typedef struct
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t sample_count;
    uint32_t read_errors;
} ImuMessage;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static QueueHandle_t pwm_command_queue = NULL;
static QueueHandle_t imu_data_queue = NULL;
static QueueHandle_t uart_rx_queue = NULL;
static uint8_t uart_rx_byte;

volatile uint32_t uart_rx_dropped = 0;

static SemaphoreHandle_t uart_tx_mutex = NULL;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void PwmTask(void *argument);
static void UartTask(void *argument);
static void ImuTask(void *argument);
static void ProcessCommand(const char *line);

static HAL_StatusTypeDef UartSend(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
uart_tx_mutex = xSemaphoreCreateMutex();

if (uart_tx_mutex == NULL)
{
    Error_Handler();
}
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
pwm_command_queue = xQueueCreate(8, sizeof(PwmCommand));

if (pwm_command_queue == NULL)
{
    Error_Handler();
}

uart_rx_queue = xQueueCreate(128, sizeof(uint8_t));

if (uart_rx_queue == NULL)
{
    Error_Handler();
}
imu_data_queue = xQueueCreate(1, sizeof(ImuMessage));
if (imu_data_queue == NULL)
{
    Error_Handler();
}
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

if (xTaskCreate(PwmTask, "PWM", 256, NULL, 4, NULL) != pdPASS)
{
    Error_Handler();
}

if (xTaskCreate(UartTask, "UART", 384, NULL, 2, NULL) != pdPASS)
{
    Error_Handler();
}

if (xTaskCreate(ImuTask, "IMU", 512, NULL, 3, NULL) != pdPASS)
{
    Error_Handler();
}

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void PwmTask(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t update_period = pdMS_TO_TICKS(5);

    uint32_t max_brightness = 1000;
    uint32_t breath_period_ms = 2000;
    uint32_t phase_ms = 0;

    PwmCommand command;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    for (;;)
    {
        /* 每轮最多处理 8 条消息，避免持续收消息耽误 PWM 更新 */
        for (uint32_t i = 0; i < 8; i++)
        {
            if (xQueueReceive(
                    pwm_command_queue, &command, 0) != pdPASS)
            {
                break;
            }

            switch (command.type)
            {
                case PWM_SET_MAX_BRIGHTNESS:
                    if (command.value <= 1000)
                    {
                        max_brightness = command.value;
                    }
                    break;

                case PWM_SET_BREATH_PERIOD:
                    if (command.value >= 200 &&
                        command.value <= 10000)
                    {
                        breath_period_ms = command.value;
                        phase_ms = 0;
                    }
                    break;

                default:
                    break;
            }
        }

        /* 将完整周期分成变亮和变暗两个阶段 */
        uint32_t half_period_ms = breath_period_ms / 2;
        uint32_t brightness;

        if (phase_ms < half_period_ms)
        {
            brightness =
                max_brightness * phase_ms / half_period_ms;
        }
        else
        {
            brightness =
                max_brightness *
                (breath_period_ms - phase_ms) /
                (breath_period_ms - half_period_ms);
        }

        __HAL_TIM_SET_COMPARE(
            &htim1, TIM_CHANNEL_1, brightness);

        /* 每次更新推进 5 ms 的呼吸进度 */
        phase_ms += 5;

        if (phase_ms >= breath_period_ms)
        {
            phase_ms = 0;
        }

        vTaskDelayUntil(&last_wake, update_period);
    }
}

static void UartTask(void *argument)
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
        /* Bound command work so telemetry also gets serviced. */
        for (uint32_t i = 0; i < 32; i++)
        {
            if (xQueueReceive(uart_rx_queue, &received_byte, 0) != pdPASS)
                break;
            if (received_byte == '\n')
            {
                if (overflow)
                {
                    const char reply[] = "ERR line too long\r\n";
                    UartSend(&huart1, (uint8_t *)reply, sizeof(reply)-1, 100);
                }
                else if (length > 0)
                {
                    line[length] = '\0';
                    ProcessCommand(line);
                }
                length = 0;
                overflow = 0;
            }
            else if (received_byte == '\r')
            {
                /* Accept CRLF as well as LF. */
            }
            else if (!overflow)
            {
                if (length < sizeof(line)-1)
                    line[length++] = (char)received_byte;
                else
                    overflow = 1;
            }
        }

        if (HAL_GetTick() - last_print_tick >= 200)
        {
            last_print_tick = HAL_GetTick();
            if (xQueueReceive(imu_data_queue, &imu, 0) == pdPASS)
            {
                int text_length = snprintf(text, sizeof(text),
                    "IMU n=%lu err=%lu A:%d,%d,%d G:%d,%d,%d\r\n",
                    (unsigned long)imu.sample_count,
                    (unsigned long)imu.read_errors,
                    (int)imu.ax, (int)imu.ay, (int)imu.az,
                    (int)imu.gx, (int)imu.gy, (int)imu.gz);
                if (text_length > 0 && text_length < (int)sizeof(text))
                    UartSend(&huart1, (uint8_t *)text, (uint16_t)text_length, 20);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
static void ImuTask(void *argument)
{
    (void)argument;

    const uint16_t address = 0x68 << 1;
    uint8_t data[14];
    uint8_t value;
    uint8_t id = 0;


    uint32_t sample_count = 0;
    uint32_t read_errors = 0;

    /* 等待模块上电稳定 */
    vTaskDelay(pdMS_TO_TICKS(100));

    if (HAL_I2C_Mem_Read(
            &hi2c1, address, 0x75,
            I2C_MEMADD_SIZE_8BIT,
            &id, 1, 20) != HAL_OK ||
        id != 0x68)
    {
        const char message[] = "ERR IMU identity\r\n";

        UartSend(&huart1, (uint8_t *)message,
                 sizeof(message) - 1, 100);

        /* 只停止 IMU 任务，保留灯光和串口控制 */
        vTaskSuspend(NULL);
    }

    /* 唤醒，使用 X 轴陀螺仪 PLL */
    value = 0x01;

    if (HAL_I2C_Mem_Write(
            &hi2c1, address, 0x6B,
            I2C_MEMADD_SIZE_8BIT,
            &value, 1, 20) != HAL_OK)
    {
        Error_Handler();
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    const uint8_t registers[] = {0x1A, 0x19, 0x1B, 0x1C};
    const uint8_t settings[]  = {0x03, 9,    0x00, 0x00};

    for (uint32_t i = 0; i < 4; i++)
    {
        value = settings[i];

        if (HAL_I2C_Mem_Write(
                &hi2c1, address, registers[i],
                I2C_MEMADD_SIZE_8BIT,
                &value, 1, 20) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* 初始化结束后再建立周期基准 */
    TickType_t last_wake = xTaskGetTickCount();


    for (;;)
    {
        if (HAL_I2C_Mem_Read(
                &hi2c1, address, 0x3B,
                I2C_MEMADD_SIZE_8BIT,
                data, sizeof(data), 5) == HAL_OK)
        {
            int16_t ax = (int16_t)(
                ((uint16_t)data[0] << 8) | data[1]);
            int16_t ay = (int16_t)(
                ((uint16_t)data[2] << 8) | data[3]);
            int16_t az = (int16_t)(
                ((uint16_t)data[4] << 8) | data[5]);

            int16_t gx = (int16_t)(
                ((uint16_t)data[8] << 8) | data[9]);
            int16_t gy = (int16_t)(
                ((uint16_t)data[10] << 8) | data[11]);
            int16_t gz = (int16_t)(
                ((uint16_t)data[12] << 8) | data[13]);

            sample_count++;

            /* 每 200 ms 显示一次，读取仍按 10 ms 周期 */
            ImuMessage message = {
                .ax = ax, .ay = ay, .az = az,
                .gx = gx, .gy = gy, .gz = gz,
                .sample_count = sample_count,
                .read_errors = read_errors
            };
            /* Single-slot queue: publish latest sample without waiting. */
            xQueueOverwrite(imu_data_queue, &message);
        }
        else
        {
            read_errors++;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;

        if (xQueueSendFromISR(
                uart_rx_queue,
                &uart_rx_byte,
                &higher_priority_task_woken) != pdPASS)
        {
            uart_rx_dropped++;
        }

        /* 收完一个字节后，重新启动下一次接收 */
        if (HAL_UART_Receive_IT(
                &huart1, &uart_rx_byte, 1) != HAL_OK)
        {
            Error_Handler();
        }

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void ProcessCommand(const char *line)
{
    PwmCommand command;
    const char *reply;

    if (line[0] != 'B' && line[0] != 'T')
    {
        reply = "ERR command\r\n";
    }
    else
    {
        const char *number = &line[1];
        size_t digits = strlen(number);

        /* 限制数字长度，并拒绝空数字、负号和其他字符 */
        uint8_t valid = (digits >= 1 && digits <= 5);

        for (size_t i = 0; i < digits && valid; i++)
        {
            if (number[i] < '0' || number[i] > '9')
            {
                valid = 0;
            }
        }

        if (!valid)
        {
            reply = "ERR number\r\n";
        }
        else
        {
            uint32_t value = (uint32_t)strtoul(number, NULL, 10);

            if ((line[0] == 'B' && value > 1000) ||
                (line[0] == 'T' &&
                 (value < 200 || value > 10000)))
            {
                reply = "ERR range\r\n";
            }
            else
            {
                command.type = (line[0] == 'B')
                    ? PWM_SET_MAX_BRIGHTNESS
                    : PWM_SET_BREATH_PERIOD;

                command.value = value;

                if (xQueueSend(
                        pwm_command_queue,
                        &command,
                        0) == pdPASS)
                {
                    reply = "OK queued\r\n";
                }
                else
                {
                    reply = "ERR queue full\r\n";
                }
            }
        }
    }

    if (UartSend(
            &huart1,
            (uint8_t *)reply,
            (uint16_t)strlen(reply),
            100) != HAL_OK)
    {
        Error_Handler();
    }
}

static HAL_StatusTypeDef UartSend(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout)
{
    HAL_StatusTypeDef result;

    if (xSemaphoreTake(
            uart_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return HAL_TIMEOUT;
    }

    result = HAL_UART_Transmit(huart, data, size, timeout);

    xSemaphoreGive(uart_tx_mutex);

    return result;
}
/* USER CODE END Application */

