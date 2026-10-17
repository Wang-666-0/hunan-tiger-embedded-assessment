#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H
#include "gl60_ii.h"
#include "yaw_cascade.h"
#include "app_config.h"
#include <stdint.h>

/* 生产双环；0仅保留既有单环对照回归，不提供运行时自动切环。 */
#ifndef YAW_CONTROLLER_CASCADE_ENABLED
#define YAW_CONTROLLER_CASCADE_ENABLED 1U
#endif

/* 手动目标±50度，相对本次软件零点；阶跃入口仍只接受±30度。
 * 保留原10度反馈越界余量，避免原40度门限阻断手动正常范围。
 * #ifndef仅供明确的历史离线用例覆盖。 */
#ifndef YAW_CONTROLLER_TARGET_LIMIT_DEG
#define YAW_CONTROLLER_TARGET_LIMIT_DEG 50.0f
#endif
#ifndef YAW_CONTROLLER_HARD_LIMIT_DEG
#define YAW_CONTROLLER_HARD_LIMIT_DEG 60.0f
#endif
/* V2B生产仅裁到MIT协议端点；0.10只保留历史单环/V2A对照。 */
#if YAW_CONTROLLER_CASCADE_ENABLED && !YAW_CASCADE_OUTPUT_LIMITS_ENABLED
#define YAW_CONTROLLER_TORQUE_LIMIT_NM GL60_TORQUE_MAX_NM
#else
#define YAW_CONTROLLER_TORQUE_LIMIT_NM 0.10f
#endif
/* 跟随原CH1满杆60deg/s映射，保持完整比例及死区，回中保持目标。 */
#ifndef YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS
#define YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS APP_YAW_RATE_LIMIT_DPS
#endif
/* 用户2026-10-14明确要求考核档取消60deg/s停机及40deg/s抑制。
 * 1仅供原受限单环档回归；生产默认0，速度参与双环控制及测量。 */
#ifndef YAW_CONTROLLER_SPEED_GUARD_ENABLED
#define YAW_CONTROLLER_SPEED_GUARD_ENABLED 0U
#endif
#define YAW_CONTROLLER_SPEED_LIMIT_DPS 60.0f
/* 速度过软阈值时抑制加速并给反向制动，不以放宽硬保护掩盖超速。 */
#define YAW_CONTROLLER_SOFT_SPEED_DPS 40.0f
#define YAW_CONTROLLER_SPEED_BRAKE_KD 0.20f
/* 通信连续性检查独立于原60deg/s门限，不作为转速控制器。
 * 10ms允许13deg；正常30deg/250ms轨迹不会被原60门槛代替拒绝。 */
#define YAW_CONTROLLER_CONTINUITY_DPS 1200.0f
#define YAW_CONTROLLER_CONTINUITY_MARGIN_DEG 1.0f
/* 仅未尝试使能、状态0/13的无输出观察阶段；不扩大闭环活动空间。 */
#define YAW_CONTROLLER_OFF_SPEED_LIMIT_DPS 360.0f
#define YAW_CONTROLLER_OFF_JUMP_MARGIN_DEG 3.0f
#ifndef YAW_CONTROLLER_OFF_ANGLE_LIMIT_DEG
#define YAW_CONTROLLER_OFF_ANGLE_LIMIT_DEG 65.0f
#endif
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
    YawCascade cascade;
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
void YawController_Stop(YawController *yaw);
/* allowed由后端综合遥控、CAN、驱动使能和停机提交状态。无新反馈不积分，
 * 新反馈dt最大100ms；手动目标以控制tick积分，不追补迟到时间。 */
float YawController_Update(YawController *yaw, float rate_dps,
                           uint32_t now_ms, uint8_t allowed);
#endif
