#ifndef PITCH_CONTROLLER_H
#define PITCH_CONTROLLER_H
#include "dm3507.h"
#include "yaw_cascade.h"
#include "app_config.h"
#include <stdint.h>

/* 相对READY时的实测行程基准：机械抬头-20/低头+35；目标留2度、撤权界留1度。
 * 2026-10-17用户更正了此前±50无遮挡条件。每次重设零都须回到同一基准姿态。 */
#define PITCH_CONTROLLER_TARGET_MIN_DEG (-18.0f)
#define PITCH_CONTROLLER_TARGET_MAX_DEG 33.0f
#define PITCH_CONTROLLER_HARD_MIN_DEG (-19.0f)
#define PITCH_CONTROLLER_HARD_MAX_DEG 34.0f
#define PITCH_CONTROLLER_TORQUE_LIMIT_NM DM3507_TORQUE_MAX_NM
#define PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS 100U
#define PITCH_CONTROLLER_CONTROL_MAX_GAP_MS 10U
#define PITCH_CONTROLLER_STATIONARY_SPEED_DPS 6.0f
#define PITCH_CONTROLLER_STATIONARY_HOLD_MS 20U
#define PITCH_CONTROLLER_DRIVER_LIMIT_C 60U
#define PITCH_CONTROLLER_MOTOR_LIMIT_C 70U

typedef enum {
    PITCH_CONTROLLER_FAULT_NONE = 0U,
    PITCH_CONTROLLER_FAULT_FEEDBACK = 1U << 0,
    PITCH_CONTROLLER_FAULT_DATA = 1U << 1,
    PITCH_CONTROLLER_FAULT_STATUS = 1U << 2,
    PITCH_CONTROLLER_FAULT_TEMPERATURE = 1U << 3,
    PITCH_CONTROLLER_FAULT_JUMP = 1U << 5,
    PITCH_CONTROLLER_FAULT_LIMIT = 1U << 6,
    PITCH_CONTROLLER_FAULT_CAN = 1U << 7,
    PITCH_CONTROLLER_FAULT_TIMING = 1U << 8
} PitchControllerFault;

typedef struct {
    uint32_t duration_ms, start_ms;
    float velocity_gain, start_deg, position_deg, velocity_dps, acceleration_dps2;
    uint8_t active;
} PitchShape;

typedef struct {
    float torque_nm, direction, target_deg, previous_angle_deg, safe_velocity_dps;
    float peak_abs_nm, return_peak_nm;
    uint32_t sequence, feedback_ms, arrival_ms, active_frames, return_frames;
    uint8_t enabled, tracking, arrived, active, approach_done;
} PitchTerminalBrake;

/* 首次到位与回退快照，只读诊断；不参与控制，不自动改写积分。 */
typedef struct {
    float target_deg, direction, start_i_nm, land_i_nm, min_i_nm, max_i_nm;
    float land_raw_dps, land_filtered_dps, land_target_dps;
    float land_gravity_nm, land_brake_nm, land_torque_nm, dip_deg;
    uint32_t start_ms, sequence, arrival_ms, land_ms, dip_ms;
    uint8_t active, arrived, landed;
} PitchLandingTrace;

typedef struct {
    /* 复用已验证双环数学；参数与状态完全独立于Yaw实例。 */
    YawCascade cascade;
    float reference_rad, last_position_rad, continuous_rad;
    float angle_deg, target_deg, velocity_dps, torque_nm;
    /* 实际水平角=angle_deg+gravity_offset_deg。Kg载入固化默认，在线改值仅RAM。
     * 未标定时保留系数，禁止使能；不保存跨重启的软件零点/相位。 */
    float gravity_kg_nm, gravity_offset_deg, gravity_torque_nm, pid_torque_nm;
    uint8_t gravity_calibrated, gravity_i_transferred;
    /* target_deg始终是最终目标；shape.position_deg仅供内环跟踪。 */
    PitchShape shape;
    /* 独立的末段超速阻尼；只在SHAPE=0的离散目标运动中启用。 */
    PitchTerminalBrake brake;
    PitchLandingTrace landing;
    uint32_t last_feedback_ms, last_sequence, faults, last_control_ms;
    uint32_t stationary_started_ms;
    uint8_t have_feedback, have_reference, active, have_control_tick;
    uint8_t last_status, temp_mos, temp_rotor, stationary_samples, feedback_divisor;
} PitchController;

void PitchController_Init(PitchController *pitch);
uint8_t PitchController_OnFeedback(PitchController *pitch, const Dm3507Feedback *feedback);
/* 支撑实测行程基准姿态、关闭且健康静止反馈满20ms才捕获；只改RAM参考，不发送FE。 */
uint8_t PitchController_CaptureReference(PitchController *pitch);
/* 1=协议已是输出轴，7=协议电机轴；关闭后人工确认，改动清软件零点。 */
uint8_t PitchController_SetFeedbackDivisor(PitchController *pitch, uint8_t divisor);
/* P:deg->deg/s；V:deg/s->协议Nm。参数输入范围并非PID输出限幅。 */
uint8_t PitchController_SetPid(PitchController *pitch, char which, float kp, float ki, float kd);
/* 即时目标入口，无条件取消规划；离散业务运动用StartMove。 */
uint8_t PitchController_SetTarget(PitchController *pitch, float degrees);
/* SHAPE=0直接目标+末段阻尼；可选轨迹按静止到静止处理。同目标不重启，不清积分。 */
uint8_t PitchController_StartMove(PitchController *pitch, float degrees, uint32_t now_ms);
/* Immediate return target; may interrupt an active plan without resetting PID. */
uint8_t PitchController_ReturnTo(PitchController *pitch, float degrees, uint32_t now_ms);
/* duration=0关闭规划，或100..250ms；速度参考权重0..1。 */
uint8_t PitchController_SetShape(PitchController *pitch, uint32_t duration_ms, float velocity_gain);
/* OFF立即撤去附加力矩；ON在下一次离散目标运动生效，不清积分。 */
uint8_t PitchController_SetBrakeEnabled(PitchController *pitch, uint8_t enabled);
void PitchController_InvalidateGravity(PitchController *pitch);
uint8_t PitchController_SetGravityGain(PitchController *pitch, float kg_nm);
/* 关闭且已建立软件零点时声明水平偏置；不改变软件零点。 */
uint8_t PitchController_SetGravityOffset(PitchController *pitch, float offset_deg);
uint8_t PitchController_CaptureHorizontal(PitchController *pitch);
void PitchController_Stop(PitchController *pitch);
/* 新CAN位置差分按Pitch独立2ms滤速/20ms滤加速度，无新CAN不推进积分/估计。
 * 手动rate限幅仅CH2映射，阶跃无速度业务限幅。 */
float PitchController_Update(PitchController *pitch, float rate_dps, uint32_t now_ms, uint8_t allowed);
/* Fixed-target hold with the original PID, observer, gravity and protections.
 * This entry never temporarily edits or scales the user's loop gains. */
float PitchController_UpdateViewHold(PitchController *pitch, uint32_t now_ms, uint8_t allowed);
#endif
