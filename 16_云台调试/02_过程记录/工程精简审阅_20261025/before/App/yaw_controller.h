/**
 * @file yaw_controller.h
 * @brief Yaw 控制器数据与接口
 * 职责：定义位置基准、PID、轨迹/前馈和反馈连续性、机械边界保护。
 * 主要接口：YawController_Init, YawController_OnFeedback, YawController_CaptureReference, YawController_SetPid, YawController_SetTarget 等。
 * 实现/调用：yaw_controller.c；支持既有单环对照和生产串级 PID，开关/限位见 app_config.h。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H
#include "gl60_ii.h"
#include "yaw_cascade.h"
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

typedef struct { float kp, ki, kd; } YawPid;
typedef struct {
    /* 协议Nm/(deg/s^2)，仅乘规划加速度；不对编码器加速度作正反馈。 */
    float acceleration_gain, limit_nm;
    uint32_t revision;
    uint8_t enabled;
} YawFeedforward;
typedef struct {
    uint32_t duration_ms, start_ms;
    float start_deg, position_deg, velocity_dps, acceleration_dps2, velocity_gain;
    uint8_t active;
} YawShape;
typedef struct {
    YawPid pid;
    YawCascade cascade;
    YawFeedforward ff;
    YawShape shape;
    float pid_torque_nm;
    float reference_rad, last_position_rad, continuous_rad;
    float angle_deg, target_deg, velocity_dps, torque_nm, integral;
    uint32_t last_feedback_ms, last_sequence, faults;
    uint8_t have_feedback, have_reference, active;
    /* 默认严格；后端只在健康失能参考捕获/初始化后放宽，FC前恢复严格。
     * STOP不清；实际状态1也锁回严格，避免滑行被当作人工碰触。 */
    uint8_t motion_guard_active;
    uint8_t speed_limited;
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
/* 历史单环接口，单位Nm/rad、Nm/(rad*s)、Nm*s/rad。
 * 双环由yaw_cascade接口调参，不能复用这组三个数字。 */
uint8_t YawController_SetPid(YawController *yaw, float kp, float ki, float kd);
uint8_t YawController_SetTarget(YawController *yaw, float degrees);
/* V2P生产编译关闭实验规划/前馈；仅接受显式关闭和零规划配置。
 * 以下接口保留历史离线回归，APP_YAW_EXPERIMENTAL_TRACKING=1才可启用。
 * A=规划加速度系数(0~0.001)，L=前馈限幅(0~1)，E=使能。
 * 运动规划中只允许关闭；不再把前馈转存到积分中。旧B/V/C/S已删除。 */
uint8_t YawController_SetFeedforward(YawController *yaw, char parameter, float value);
/* 默认duration=0关闭。只在正式STEP入口启动；配置不启动电机。
 * 80~500ms为五次多项式全程时间，0恢复原始阶跃；计时仍从原指令开始。 */
uint8_t YawController_SetShape(YawController *yaw, uint32_t duration_ms);
/* 规划速度前馈权重0~1，默认0；与加速度力矩前馈的FF开关独立。 */
uint8_t YawController_SetShapeVelocityGain(YawController *yaw, float gain);
void YawController_StartShape(YawController *yaw, uint32_t now_ms);
void YawController_Stop(YawController *yaw);
/* allowed由后端综合遥控、CAN、驱动使能和停机提交状态。无新反馈不积分，
 * 新反馈dt最大100ms；手动目标以控制tick积分，不追补迟到时间。 */
float YawController_Update(YawController *yaw, float rate_dps,
                           uint32_t now_ms, uint8_t allowed);
#endif
