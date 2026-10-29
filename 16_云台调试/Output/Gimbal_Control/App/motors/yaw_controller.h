/* Yaw 控制器数据与接口 */
#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H
#include "gl60_ii.h"
#include "gimbal_pid.h"
#include "app_config.h"
#include <stdint.h>

typedef enum {
    YAW_CONTROLLER_FAULT_NONE = 0U,
    YAW_CONTROLLER_FAULT_FEEDBACK = 1U << 0,
    YAW_CONTROLLER_FAULT_DATA = 1U << 1,
    YAW_CONTROLLER_FAULT_STATUS = 1U << 2,
    YAW_CONTROLLER_FAULT_TEMPERATURE = 1U << 3,
    YAW_CONTROLLER_FAULT_SPEED = 1U << 4,
    YAW_CONTROLLER_FAULT_JUMP = 1U << 5,
    YAW_CONTROLLER_FAULT_LIMIT = 1U << 6,
    YAW_CONTROLLER_FAULT_CAN = 1U << 7,
    YAW_CONTROLLER_FAULT_TIMING = 1U << 8
} YawControllerFault;

typedef struct {
    GimbalPid cascade;
    float pid_torque_nm;
    float reference_rad, last_position_rad, continuous_rad;
    float angle_deg, target_deg, velocity_dps, torque_nm;
    uint32_t last_feedback_ms, last_sequence, faults;
    uint8_t have_feedback, have_reference, active;
    /* 默认严格；后端只在健康失能参考捕获/初始化后放宽，FC前恢复严格。
     * STOP不清；实际状态1也锁回严格，避免滑行被当作人工碰触。 */
    uint8_t motion_guard_active;
    /* 所有调用必须由上层串行化；没有HAL、RTOS或共享静态控制状态。 */
    uint32_t last_control_ms, stationary_started_ms;
    uint8_t have_control_tick, stationary_samples;
    uint8_t last_status;
    int8_t last_driver_temp_c, last_motor_temp_c;
} YawController;

void YawController_Init(YawController *yaw);
/* 收到的实际接收时间须单调；旧/重复包不能延长反馈寿命。达到100ms空窗
 * 重新建立角度基准并锁故障，不能把断线前后的角度直接跨圈拼接。 */
uint8_t YawController_OnFeedback(YawController *yaw, const Gl60Feedback *feedback);
/* 上层还须验证反馈新鲜、总许可已撤销及机械活动空间。本函数要求位置
 * 连续静止20ms、状态/温度正常；只清已知可恢复故障，CAN/未知故障保留。 */
uint8_t YawController_CaptureReference(YawController *yaw);
uint8_t YawController_SetTarget(YawController *yaw, float degrees);
void YawController_Stop(YawController *yaw);
/* allowed由后端综合遥控、CAN、驱动使能和停机提交状态。无新反馈不积分，
 * 新反馈dt最大100ms；手动目标以控制tick积分，不追补迟到时间。 */
float YawController_Update(YawController *yaw, float rate_dps,
                           uint32_t now_ms, uint8_t allowed);
#endif
