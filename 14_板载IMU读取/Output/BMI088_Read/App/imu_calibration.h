#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_CALIBRATION_SAMPLES 500U /* 200 Hz 下连续静止 500 次，约 2.5 秒。 */

typedef struct
{
    uint32_t count;             /* 当前连续有效的静止样本数。 */
    float gyro_sum[3];          /* 三轴角速度之和，单位 rad/s。 */
    float gyro_square_sum[3];   /* 用于判断窗口内是否发生了转动。 */
    float accel_sum[3];         /* 保留重力，供初始 Roll/Pitch 使用。 */
    float accel_square_sum[3];  /* 用于判断窗口内是否发生了晃动。 */
} imu_calibration_t;

void ImuCalibration_Reset(imu_calibration_t *state);
/* 有转动/加速时自动重新计数；凑齐一个稳定窗口才返回 true。 */
bool ImuCalibration_Push(imu_calibration_t *state, const float gyro[3], const float accel[3]);
/* 输出陀螺仪零偏和含重力的平均加速度，两个数组均有三个元素。 */
bool ImuCalibration_GetResult(const imu_calibration_t *state, float gyro_bias[3], float gravity[3]);

#endif
