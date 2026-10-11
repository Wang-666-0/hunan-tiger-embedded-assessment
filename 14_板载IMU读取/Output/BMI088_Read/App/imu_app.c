#include "imu_app.h"
#include "bmi088.h"
#include "imu_calibration.h"
#include "mahony.h"
#include "vofa_telemetry.h"
#include "cmsis_os2.h"
#include <stdint.h>

#define IMU_PERIOD_MS 5U           /* 读取、姿态更新和串口发送均为 200 Hz。 */
#define IMU_POWER_SETTLE_MS 200U    /* 上电后先等待传感器电源和内部电路稳定。 */
#define IMU_RETRY_MS 500U           /* 初始化/读取异常后不忙轮询，也不发旧姿态。 */
#define IMU_MAX_GAP_MS 25U          /* 断帧太久不猜测期间运动，重新校准建立姿态。 */

static bool WaitForNextPeriod(uint32_t *deadline, uint32_t period_ticks)
{
    uint32_t now;
    *deadline += period_ticks; /* 以绝对时间推进，计算和 UART 耗时包含在 5 ms 内。 */
    now = osKernelGetTickCount();
    if ((int32_t)(*deadline - now) <= 0)
    {
        *deadline = now; /* 超期时重新建立基准，不能连续补发多帧追赶旧 deadline。 */
        (void)osDelay(period_ticks); /* 异常过载时主动让出 CPU；不承诺此时仍为 200 Hz。 */
        *deadline = osKernelGetTickCount();
        return false;
    }
    return osDelayUntil(*deadline) == osOK; /* 正常路径下一轮起点保持 5 ms 间隔。 */
}

static bool CollectStationaryCalibration(float bias[3], float gravity[3], uint32_t period_ticks)
{
    imu_calibration_t calibration;
    bmi088_sample_t sample;
    uint32_t deadline = osKernelGetTickCount();
    ImuCalibration_Reset(&calibration);
    for (;;)
    {
        if (!BMI088_Read(&sample))
        {
            return false; /* 总线或 ID 异常时退出，交给外层重新初始化。 */
        }
        if (ImuCalibration_Push(&calibration, sample.gyro_rad_s, sample.accel_m_s2))
        {
            return ImuCalibration_GetResult(&calibration, bias, gravity); /* 只有静止窗口才成功。 */
        }
        if (!WaitForNextPeriod(&deadline, period_ticks))
        {
            ImuCalibration_Reset(&calibration); /* 周期异常不能算作连续 2.5 秒静止采样。 */
        }
    }
}

void ImuApp_Run(void)
{
    uint32_t tick_hz = osKernelGetTickFreq();
    uint32_t period_ticks = tick_hz / (1000U / IMU_PERIOD_MS);
    uint32_t axis;
    float bias[3];
    float gravity[3];
    float euler_deg[3];
    mahony_t attitude;
    bmi088_sample_t sample;

    if ((tick_hz != 1000U) || (period_ticks == 0U))
    {
        for (;;)
        {
            (void)osDelay(1U); /* 本工程明确要求 1 kHz RTOS tick，防止错误配置悄悄改变周期。 */
        }
    }
    (void)osDelay(IMU_POWER_SETTLE_MS); /* 在调度器启动后等待，此处没有 HAL_Delay 忙等。 */
    for (;;)
    {
        uint32_t deadline;
        uint32_t last_sample_tick;
        if (!BMI088_Init() || !CollectStationaryCalibration(bias, gravity, period_ticks) ||
            !Mahony_Init(&attitude, gravity))
        {
            (void)osDelay(IMU_RETRY_MS); /* 失败后保持无有效帧，稍后重新尝试。 */
            continue;
        }
        deadline = osKernelGetTickCount(); /* 校准结束后才建立正式输出的周期基准。 */
        last_sample_tick = deadline - period_ticks; /* 首次正常更新使用 0.005 秒。 */
        for (;;)
        {
            uint32_t sample_tick;
            uint32_t elapsed_ticks;
            float dt_s;
            if (!BMI088_Read(&sample))
            {
                break; /* 失败的样本不进入融合、不发送上一帧的角度。 */
            }
            sample_tick = osKernelGetTickCount();
            elapsed_ticks = sample_tick - last_sample_tick; /* 无符号相减支持 tick 自然回绕。 */
            if ((elapsed_ticks == 0U) || (elapsed_ticks > (IMU_MAX_GAP_MS * tick_hz / 1000U)))
            {
                break; /* 大间隔不能按固定 5 ms 积分，重新建立相对姿态。 */
            }
            dt_s = (float)elapsed_ticks / (float)tick_hz; /* 实际间隔换成秒，而非把 5 当作秒。 */
            last_sample_tick = sample_tick;
            for (axis = 0U; axis < 3U; ++axis)
            {
                sample.gyro_rad_s[axis] -= bias[axis]; /* 只扣除静止陀螺仪零偏，不扣掉重力。 */
            }
            if (!Mahony_UpdateIMU(&attitude, sample.gyro_rad_s, sample.accel_m_s2, dt_s) ||
                !Mahony_GetEulerDegrees(&attitude, euler_deg))
            {
                break; /* 非法数据/四元数不发布到 VOFA。 */
            }
            (void)VOFA_SendEuler(euler_deg); /* 三个 float 的完整新帧，每个正常周期发送一次。 */
            (void)WaitForNextPeriod(&deadline, period_ticks);
        }
        (void)osDelay(IMU_RETRY_MS); /* 异常恢复后重新静止校准，Yaw 相对零点也会重新建立。 */
    }
}
