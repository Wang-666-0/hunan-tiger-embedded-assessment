#include "imu_app.h"
#include "imu_reader.h"
#include "imu_telemetry.h"
#include "main.h"

void ImuApp_Init(void)
{
    ImuReader_Init();
}

void ImuApp_Process(void)
{
    /* 保留原顺序：先更新姿态，再尝试打印，最后短暂延时。 */
    ImuReader_Process();
    ImuTelemetry_Process();
    HAL_Delay(1);
}
