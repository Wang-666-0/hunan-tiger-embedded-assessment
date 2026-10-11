#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>

/* 每一帧同时携带六轴物理量；算法直接使用 rad/s 和 m/s²，不再转换原始整数。 */
typedef struct
{
    float accel_m_s2[3]; /* X/Y/Z 加速度，包含重力，单位 m/s²。 */
    float gyro_rad_s[3]; /* X/Y/Z 角速度，单位 rad/s；这里尚未减去启动校准的零偏。 */
    float temperature_c; /* 摄氏温度；首次温度转换未完成时为 NAN，不影响六轴姿态计算。 */
} bmi088_sample_t;

/* 必须从已启动的 FreeRTOS 任务调用；SPI1/GPIO 必须先由 CubeMX 初始化。 */
bool BMI088_Init(void);

/* 读取成功才更新 *sample；失败返回 false，调用方不得把旧数据当成新数据继续积分。 */
bool BMI088_Read(bmi088_sample_t *sample);

#endif
