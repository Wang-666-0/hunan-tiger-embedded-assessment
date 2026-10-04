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
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for pwmTask */
osThreadId_t pwmTaskHandle;
const osThreadAttr_t pwmTask_attributes = {
  .name = "pwmTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartPwmTask(void *argument);
void StartUartTask(void *argument);

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
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of pwmTask */
  pwmTaskHandle = osThreadNew(StartPwmTask, NULL, &pwmTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(StartUartTask, NULL, &uartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartPwmTask */
/**
  * @brief  Function implementing the pwmTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPwmTask */
void StartPwmTask(void *argument)
{
  /* USER CODE BEGIN StartPwmTask */
      uint16_t brightness = 0;
      int16_t step = 5;
 /* 启动 TIM1 通道1的 PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  for (;;)
  {
    /* 设置占空比，范围为 0～1000 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, brightness);

    /* 等待期间，让串口任务也能运行 */
    osDelay(5);

    /* 到达最亮或最暗时，改变方向 */
    if (brightness >= 1000)
    {
      step = -5;
    }
    else if (brightness == 0)
    {
      step = 5;
    }

    brightness = (uint16_t)((int16_t)brightness + step);
  }
  /* USER CODE END StartPwmTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */

  uint8_t rx_byte;
  uint8_t welcome[] = "FreeRTOS UART ready\r\n";

  HAL_UART_Transmit(&huart1,
                    welcome,
                    sizeof(welcome) - 1,
                    100);

  for (;;)
  {
    /* 尝试接收一个字节，最多等待 1 ms */
    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 1) == HAL_OK)
    {
      /* 收到什么，就回复什么 */
      HAL_UART_Transmit(&huart1, &rx_byte, 1, 10);
    }

    /* 让出 CPU，其他任务可以运行 */
    osDelay(1);
  }


  /* USER CODE END StartUartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

