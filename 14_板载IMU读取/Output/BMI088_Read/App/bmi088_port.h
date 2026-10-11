#ifndef BMI088_PORT_H
#define BMI088_PORT_H

#include <stdbool.h>
#include <stdint.h>

/* 将平台相关内容隔离在 port 中；bmi088.c 不直接依赖 STM32 HAL，方便主机测试。 */
typedef enum
{
    BMI088_PORT_ACCEL = 0, /* 使用 PC4 片选的加速度计。 */
    BMI088_PORT_GYRO = 1   /* 使用 PA4 片选的陀螺仪。 */
} bmi088_port_device_t;

bool BMI088_PortInit(void);
bool BMI088_PortTransfer(bmi088_port_device_t device,
                         const uint8_t *tx, uint8_t *rx, uint16_t length);
void BMI088_PortDelayUs(uint32_t microseconds);
void BMI088_PortDelayMs(uint32_t milliseconds);

#endif
