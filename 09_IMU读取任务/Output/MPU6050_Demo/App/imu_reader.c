#include "imu_reader.h"
#include "imu_telemetry.h"
#include "attitude_filter.h"
#include "main.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* 传感器与 DMP FIFO 状态归本模块；原有全局名称保留供调试。
 * Mpu6050/ 内的 I2C 适配和 DMP 库已经分文件，继续使用原库。
 */
uint8_t mpu_id = 0;
uint8_t mpu_data[14];
uint8_t mpu_config;

int16_t ax, ay, az;
int16_t gx, gy, gz;

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

void ImuReader_Init(void)
{
    HAL_Delay(100);

    uint8_t dmp_result = MPU6050_DMP_Init();

    ImuTelemetry_ReportInit(dmp_result);

    if (dmp_result != 0)
    {
        Error_Handler();
    }

    /* 获取当前量程对应的换算系数 */
    if (mpu_get_gyro_sens(&gyro_sensitivity) != 0 || mpu_get_accel_sens(&accel_sensitivity) != 0)
    {
        Error_Handler();
    }
}

void ImuReader_Process(void)
{
    /* more 表示 FIFO 仍有后续包；逐包读取，避免主循环慢时积压旧样本。 */
    do
    {
        dmp_more = 0;

        int result =
            dmp_read_fifo(dmp_gyro, dmp_accel, dmp_quat, &dmp_timestamp, &dmp_sensors, &dmp_more);

        /* 暂时没有新数据或读取失败，退出本轮读取 */
        if (result != 0)
        {
            break;
        }

        /* 确认这一包同时包含加速度和角速度 */
        if ((dmp_sensors & INV_XYZ_ACCEL) && ((dmp_sensors & INV_XYZ_GYRO) == INV_XYZ_GYRO))
        {
            ax_g = (float)dmp_accel[0] / accel_sensitivity;
            ay_g = (float)dmp_accel[1] / accel_sensitivity;
            az_g = (float)dmp_accel[2] / accel_sensitivity;

            gx_dps = (float)dmp_gyro[0] / gyro_sensitivity;
            gy_dps = (float)dmp_gyro[1] / gyro_sensitivity;
            gz_dps = (float)dmp_gyro[2] / gyro_sensitivity;

            /* 每个 FIFO 包对应一次滤波更新，不能用 UART 打印间隔作为 dt。 */
            AttitudeFilter_Update(ax_g, ay_g, az_g, gx_dps, gy_dps, 1.0f / DEFAULT_MPU_HZ);
        }

    } while (dmp_more != 0);
}
