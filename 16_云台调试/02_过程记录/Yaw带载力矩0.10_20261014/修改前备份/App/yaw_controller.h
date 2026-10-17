#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H
#include "gl60_ii.h"
#include <stdint.h>

/* 首次上板仅允许小角度、低转矩；这些不是GL电机的协议量程。 */
#define YAW_CONTROLLER_TARGET_LIMIT_DEG 5.0f
#define YAW_CONTROLLER_HARD_LIMIT_DEG 8.0f
#define YAW_CONTROLLER_TORQUE_LIMIT_NM 0.03f
#define YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS 10.0f
#define YAW_CONTROLLER_SPEED_LIMIT_DPS 60.0f
#define YAW_CONTROLLER_STATIONARY_SPEED_DPS 6.0f
#define YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS 100U
#define YAW_CONTROLLER_CONTROL_MAX_GAP_MS 10U
#define YAW_CONTROLLER_STATIONARY_HOLD_MS 20U
#define YAW_CONTROLLER_DRIVER_LIMIT_C 60
#define YAW_CONTROLLER_MOTOR_LIMIT_C 70

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

typedef struct { float kp, ki, kd; } YawPid;
typedef struct {
    YawPid pid;
    float reference_rad, last_position_rad, continuous_rad;
    float angle_deg, target_deg, velocity_dps, torque_nm, integral;
    uint32_t last_feedback_ms, last_sequence, faults;
    uint8_t have_feedback, have_reference, active;
    /* 所有调用必须由上层串行化；没有HAL、RTOS或共享静态控制状态。 */
    uint32_t last_control_ms, pid_feedback_ms, pid_sequence, stationary_started_ms;
    uint8_t have_control_tick, have_pid_feedback, stationary_samples;
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
/* 单位Nm/rad、Nm/(rad*s)、Nm*s/rad；调参清积分，不自动使能。 */
uint8_t YawController_SetPid(YawController *yaw, float kp, float ki, float kd);
uint8_t YawController_SetTarget(YawController *yaw, float degrees);
void YawController_Stop(YawController *yaw);
/* allowed由后端综合遥控、CAN、驱动使能和停机提交状态。无新反馈不积分，
 * 新反馈dt最大100ms；手动目标以控制tick积分，不追补迟到时间。 */
float YawController_Update(YawController *yaw, float rate_dps,
                           uint32_t now_ms, uint8_t allowed);
#endif
