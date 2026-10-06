#include "attitude_filter.h"
#include <math.h>

/* 保留原来的两角互补滤波。此模块不使用 DMP 四元数解算，也没有新增 yaw。
 * 加速度计根据重力方向修正长期偏差，陀螺仪积分负责短时变化。
 * 模块独立于 I2C、UART，复盘时只需关注角度、单位和滤波权重。
 */
float roll_deg = 0.0f;
float pitch_deg = 0.0f;

float roll_acc_deg = 0.0f;
float pitch_acc_deg = 0.0f;

uint8_t attitude_ready = 0;

void AttitudeFilter_Update(float ax_g, float ay_g, float az_g, float gx_dps, float gy_dps, float dt)
{
    /* 根据重力方向计算倾斜角，弧度转换为度 */
    roll_acc_deg = atan2f(ay_g, az_g) * 57.2957795f;

    pitch_acc_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.2957795f;

    /* 第一包数据直接建立初始角度 */
    if (attitude_ready == 0)
    {
        roll_deg = roll_acc_deg;
        pitch_deg = pitch_acc_deg;
        attitude_ready = 1;
    }
    else
    {
        /* dt 由调用者按每个 FIFO 包的采样间隔传入，单位是秒。 */
        /* alpha=0.98：陀螺仪预测占 98%，重力倾角修正占 2%。
         * gx_dps*dt 是本采样间隔增加的角度，单位 deg。
         * 加速度倾角假定重力占主导；快速平移会暂时影响修正结果。
         */
        const float alpha = 0.98f;

        roll_deg = alpha * (roll_deg + gx_dps * dt) + (1.0f - alpha) * roll_acc_deg;

        pitch_deg = alpha * (pitch_deg + gy_dps * dt) + (1.0f - alpha) * pitch_acc_deg;
    }
}
