#ifndef APP_MAHONY_H
#define APP_MAHONY_H

#include <stdbool.h>  // 使用明确的 true / false 表示一次操作是否成功。

/*
 * 六轴 Mahony 姿态融合的教学实现，不依赖 STM32 HAL 或 FreeRTOS。
 * 算法结构参考（本文件不是直接复制的第三方源码）：
 * https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/components/algorithm/MahonyAHRS.c
 *
 * 坐标约定：使用右手系；四元数 q = [w, x, y, z] 表示机体到世界的旋转。
 * 静止、正放时加速度应约为 [0, 0, +9.80665] m/s²；先统一陀螺仪/加速度轴方向。
 * 角速度单位必须为 rad/s；加速度单位为 m/s²；dt_s 是本次更新间隔，单位秒。
 * 六轴方案不能测量绝对航向，Yaw 以初始化时方向为零点，长期漂移是正常限制。
 */
typedef struct
{
    float q[4];        // 保存连续的姿态状态：w、x、y、z；禁止在每个任务循环里重新清零。
    float kp;          // 比例反馈增益，完整单位重力叉积的默认增益为 2.0。
    float ki;          // 积分反馈增益，默认 0；启动时由应用模块先完成陀螺仪零偏校准。
    float integral[3]; // 三轴积分反馈角速度，单位 rad/s，每个实例各自保存。
    bool initialized;  // 仅在成功完成初始化后才能调用更新和读取接口。
} mahony_t;

/* 用静止加速度初始化倾斜姿态和增益；失败时保持原 state 不变。 */
bool Mahony_Init(mahony_t *state, const float accel[3]);

/* 更新一个采样周期；0 < dt_s <= 0.1，任何无效输入均返回 false 且不破坏旧状态。 */
bool Mahony_UpdateIMU(mahony_t *state, const float gyro[3], const float accel[3], float dt_s);

/* 输出 ZYX 欧拉角，数组顺序为 Roll(X)、Pitch(Y)、Yaw(Z)，单位度。 */
bool Mahony_GetEulerDegrees(const mahony_t *state, float euler_deg[3]);

#endif
