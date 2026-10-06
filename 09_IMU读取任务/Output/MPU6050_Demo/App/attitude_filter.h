#ifndef ATTITUDE_FILTER_H
#define ATTITUDE_FILTER_H

#include <stdint.h>

/* 加速度单位 g，角速度单位 deg/s，dt 单位 s；按每个有效样本调用。 */
void AttitudeFilter_Update(float ax_g, float ay_g, float az_g, float gx_dps, float gy_dps,
                           float dt);
/* 保留原有变量名，便于继续使用 Watch 验收。 */
extern float roll_deg, pitch_deg;
extern float roll_acc_deg, pitch_acc_deg;
extern uint8_t attitude_ready;

#endif /* ATTITUDE_FILTER_H */
