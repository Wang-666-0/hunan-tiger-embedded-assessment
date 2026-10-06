#ifndef IMU_TELEMETRY_H
#define IMU_TELEMETRY_H

#include <stdint.h>

/* 初始化结果是日志；数值数据使用 attitude:roll,pitch 的 FireWater 格式。 */
void ImuTelemetry_ReportInit(uint8_t dmp_result);
/* 主循环调用，内部限制为每 100 ms 最多输出一次。 */
void ImuTelemetry_Process(void);

#endif /* IMU_TELEMETRY_H */
