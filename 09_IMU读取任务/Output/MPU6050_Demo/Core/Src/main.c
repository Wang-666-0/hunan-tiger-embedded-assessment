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
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
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

float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;

float gx_bias = 0.0f;
float gy_bias = 0.0f;
float gz_bias = 0.0f;

short dmp_gyro[3];
short dmp_accel[3];
long dmp_quat[4];

unsigned long dmp_timestamp;
short dmp_sensors;
unsigned char dmp_more;

float gyro_sensitivity;
unsigned short accel_sensitivity;

uint32_t last_print_tick = 0;

float roll_deg = 0.0f;
float pitch_deg = 0.0f;

float roll_acc_deg = 0.0f;
float pitch_acc_deg = 0.0f;

uint8_t attitude_ready = 0;
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

HAL_Delay(100);

uint8_t dmp_result = MPU6050_DMP_Init();

int len = snprintf(
    uart_text,
    sizeof(uart_text),
    "INFO DMP init result=%u\r\n",
    (unsigned int)dmp_result
);

if (len > 0 && len < (int)sizeof(uart_text))
{
    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)uart_text,
        (uint16_t)len,
        100
    );
}

if (dmp_result != 0)
{
    Error_Handler();
}

/* 获取当前量程对应的换算系数 */
if (mpu_get_gyro_sens(&gyro_sensitivity) != 0 ||
    mpu_get_accel_sens(&accel_sensitivity) != 0)
{
    Error_Handler();
}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    do
    {
        dmp_more = 0;

        int result = dmp_read_fifo(
            dmp_gyro,
            dmp_accel,
            dmp_quat,
            &dmp_timestamp,
            &dmp_sensors,
            &dmp_more
        );

        /* 暂时没有新数据或读取失败，退出本轮读取 */
        if (result != 0)
        {
            break;
        }

        /* 确认这一包同时包含加速度和角速度 */
        if ((dmp_sensors & INV_XYZ_ACCEL) &&
            ((dmp_sensors & INV_XYZ_GYRO) == INV_XYZ_GYRO))
        {
            ax_g = (float)dmp_accel[0] / accel_sensitivity;
            ay_g = (float)dmp_accel[1] / accel_sensitivity;
            az_g = (float)dmp_accel[2] / accel_sensitivity;

            gx_dps = (float)dmp_gyro[0] / gyro_sensitivity;
            gy_dps = (float)dmp_gyro[1] / gyro_sensitivity;
            gz_dps = (float)dmp_gyro[2] / gyro_sensitivity;

            /* 根据重力方向计算倾斜角，弧度转换为度 */
              roll_acc_deg = atan2f(ay_g, az_g) * 57.2957795f;

              pitch_acc_deg = atan2f(
                  -ax_g,
                  sqrtf(ay_g * ay_g + az_g * az_g)
              ) * 57.2957795f;

              /* 第一包数据直接建立初始角度 */
              if (attitude_ready == 0)
              {
                  roll_deg = roll_acc_deg;
                  pitch_deg = pitch_acc_deg;
                  attitude_ready = 1;
              }
              else
              {
                  /* 每个 FIFO 数据包对应的采样间隔，单位为秒 */
                  const float dt = 1.0f / DEFAULT_MPU_HZ;
                  const float alpha = 0.98f;

                  roll_deg = alpha * (roll_deg + gx_dps * dt)
                          + (1.0f - alpha) * roll_acc_deg;

                  pitch_deg = alpha * (pitch_deg + gy_dps * dt)
                            + (1.0f - alpha) * pitch_acc_deg;
              }
        }

    } while (dmp_more != 0);

    /* 每 100 ms 打印一次，读取数据仍然保持较高频率 */
    if (attitude_ready &&
    HAL_GetTick() - last_print_tick >= 100)
    {
        last_print_tick = HAL_GetTick();

        int len = snprintf(
            uart_text,
            sizeof(uart_text),
            "attitude:%.2f,%.2f\r\n",
            (double)roll_deg,
            (double)pitch_deg
        );

        if (len > 0 && len < (int)sizeof(uart_text))
        {
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)uart_text,
                (uint16_t)len,
                100
            );
        }
    }

    HAL_Delay(1);
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
