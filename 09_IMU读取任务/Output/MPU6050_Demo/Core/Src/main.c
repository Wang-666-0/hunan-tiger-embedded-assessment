/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

/* USER CODE BEGIN PV */
uint8_t mpu_id = 0;
uint8_t mpu_data[14];
uint8_t mpu_config;

int16_t ax, ay, az;
int16_t gx, gy, gz;

char uart_text[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//HAL_Delay(100);

//HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
//    &hi2c1,
//    (0x68 << 1),
//    0x75,
//    I2C_MEMADD_SIZE_8BIT,
//    &mpu_id,
//    1,
//    100
//);

//int length = snprintf(
//    uart_text,
//    sizeof(uart_text),
//    "I2C status=%d, MPU ID=0x%02X\r\n",
//    (int)result,
//    (unsigned int)mpu_id
//);

//if (length > 0 && length < (int)sizeof(uart_text))
//{
//    HAL_UART_Transmit(
//        &huart1,
//        (uint8_t *)uart_text,
//        (uint16_t)length,
//        100
//    );
//}

/* 唤醒，使用 X 轴陀螺仪 PLL 作为时钟 */
mpu_config = 0x01;
if (HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B,
                     I2C_MEMADD_SIZE_8BIT,
                     &mpu_config, 1, 100) != HAL_OK)
{
    Error_Handler();
}

HAL_Delay(100);

/* 设置低通滤波配置 */
mpu_config = 0x03;
if (HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1A,
                     I2C_MEMADD_SIZE_8BIT,
                     &mpu_config, 1, 100) != HAL_OK)
{
    Error_Handler();
}

/* 采样率：1 kHz / (1 + 9) = 100 Hz */
mpu_config = 9;
if (HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x19,
                     I2C_MEMADD_SIZE_8BIT,
                     &mpu_config, 1, 100) != HAL_OK)
{
    Error_Handler();
}

/* 陀螺仪量程：±250 °/s */
mpu_config = 0x00;
if (HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1B,
                     I2C_MEMADD_SIZE_8BIT,
                     &mpu_config, 1, 100) != HAL_OK)
{
    Error_Handler();
}

/* 加速度量程：±2 g */
if (HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1C,
                     I2C_MEMADD_SIZE_8BIT,
                     &mpu_config, 1, 100) != HAL_OK)
{
    Error_Handler();
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B,
                    I2C_MEMADD_SIZE_8BIT,
                    mpu_data, sizeof(mpu_data), 100) == HAL_OK)
{
    ax = (int16_t)(((uint16_t)mpu_data[0] << 8) | mpu_data[1]);
    ay = (int16_t)(((uint16_t)mpu_data[2] << 8) | mpu_data[3]);
    az = (int16_t)(((uint16_t)mpu_data[4] << 8) | mpu_data[5]);

    /* 第 6、7 字节是温度，本次跳过 */
    gx = (int16_t)(((uint16_t)mpu_data[8]  << 8) | mpu_data[9]);
    gy = (int16_t)(((uint16_t)mpu_data[10] << 8) | mpu_data[11]);
    gz = (int16_t)(((uint16_t)mpu_data[12] << 8) | mpu_data[13]);

    int len = snprintf(
        uart_text, sizeof(uart_text),
        "A:%d,%d,%d G:%d,%d,%d\r\n",
        (int)ax, (int)ay, (int)az,
        (int)gx, (int)gy, (int)gz
    );

    if (len > 0 && len < (int)sizeof(uart_text))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_text,
                          (uint16_t)len, 100);
    }
}
else
{
    uint8_t error_text[] = "MPU read failed\r\n";
    HAL_UART_Transmit(&huart1, error_text,
                      sizeof(error_text) - 1, 100);
}

HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
