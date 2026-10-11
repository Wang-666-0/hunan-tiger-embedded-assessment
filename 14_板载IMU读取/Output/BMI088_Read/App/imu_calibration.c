#include "imu_calibration.h"
#include <math.h>
#include <string.h>

#define CAL_GYRO_LIMIT_RAD_S 0.04f /* 约 2.3 度/秒，给常温 ±1 度/秒零偏留余量，明显转动则拒绝。 */
#define CAL_ACCEL_MIN_M_S2 8.8f   /* 静止时加速度模长应接近重力。 */
#define CAL_ACCEL_MAX_M_S2 10.8f
#define CAL_GYRO_VARIANCE_MAX 0.0001f /* 各轴标准差不超过约 0.01 rad/s；仍必须实际放稳板子。 */
#define CAL_ACCEL_VARIANCE_MAX 0.01f  /* 各轴标准差不超过约 0.1 m/s²。 */

void ImuCalibration_Reset(imu_calibration_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state)); /* 清空计数和统计值，开始新的静止窗口。 */
    }
}

bool ImuCalibration_Push(imu_calibration_t *state, const float gyro[3], const float accel[3])
{
    float accel_norm_squared = 0.0f;
    uint32_t axis;

    if ((state == NULL) || (gyro == NULL) || (accel == NULL))
    {
        return false; /* 空指针不能成为有效校准数据。 */
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        if (!isfinite(gyro[axis]) || !isfinite(accel[axis]) ||
            (fabsf(gyro[axis]) > CAL_GYRO_LIMIT_RAD_S))
        {
            ImuCalibration_Reset(state); /* 板子转动或数据异常时，必须重新静止等待。 */
            return false;
        }
        accel_norm_squared += accel[axis] * accel[axis]; /* 用平方模长判断重力，无须开平方。 */
    }
    if ((accel_norm_squared < CAL_ACCEL_MIN_M_S2 * CAL_ACCEL_MIN_M_S2) ||
        (accel_norm_squared > CAL_ACCEL_MAX_M_S2 * CAL_ACCEL_MAX_M_S2))
    {
        ImuCalibration_Reset(state); /* 明显线性加速/失重时不校准。 */
        return false;
    }
    if (state->count >= IMU_CALIBRATION_SAMPLES)
    {
        return true; /* 已完成的稳定窗口不再累加，避免计数和求和溢出。 */
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        state->gyro_sum[axis] += gyro[axis]; /* 静止平均角速度就是此温度下的陀螺仪零偏。 */
        state->gyro_square_sum[axis] += gyro[axis] * gyro[axis];
        state->accel_sum[axis] += accel[axis]; /* 不减掉重力，否则 Mahony 无法校正倾斜。 */
        state->accel_square_sum[axis] += accel[axis] * accel[axis];
    }
    ++state->count; /* 每个成功读取的完整六轴样本只计一次。 */
    if (state->count < IMU_CALIBRATION_SAMPLES)
    {
        return false;
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        float gyro_mean = state->gyro_sum[axis] / (float)state->count;
        float accel_mean = state->accel_sum[axis] / (float)state->count;
        float gyro_variance = state->gyro_square_sum[axis] / (float)state->count - gyro_mean * gyro_mean;
        float accel_variance = state->accel_square_sum[axis] / (float)state->count - accel_mean * accel_mean;
        if ((gyro_variance > CAL_GYRO_VARIANCE_MAX) || (accel_variance > CAL_ACCEL_VARIANCE_MAX))
        {
            ImuCalibration_Reset(state); /* 慢晃动可能模长正常，仍须通过整段方差检查。 */
            return false;
        }
    }
    return true;
}

bool ImuCalibration_GetResult(const imu_calibration_t *state, float gyro_bias[3], float gravity[3])
{
    uint32_t axis;
    if ((state == NULL) || (gyro_bias == NULL) || (gravity == NULL) ||
        (state->count != IMU_CALIBRATION_SAMPLES))
    {
        return false; /* 样本不够时不能发布未完成的校准结果。 */
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        gyro_bias[axis] = state->gyro_sum[axis] / (float)state->count; /* 单位保持 rad/s。 */
        gravity[axis] = state->accel_sum[axis] / (float)state->count; /* 单位保持 m/s²。 */
    }
    return true;
}
