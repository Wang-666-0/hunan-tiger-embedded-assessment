/* Pitch 控制器数据与接口 */
#ifndef PITCH_CONTROLLER_H
#define PITCH_CONTROLLER_H
#include "dm3507.h"
#include "gimbal_pid.h"
#include "app_config.h"
#include <stdint.h>

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
    float torque_nm, direction, target_deg, previous_angle_deg, safe_velocity_dps;
    float peak_abs_nm, return_peak_nm;
    uint32_t sequence, feedback_ms, arrival_ms, active_frames, return_frames;
    uint8_t enabled, tracking, arrived, active, approach_done;
} PitchTerminalBrake;


typedef struct {
    /* 复用已验证双环数学；参数与状态完全独立于Yaw实例。 */
    GimbalPid cascade;
    float reference_rad, last_position_rad, continuous_rad;
    float angle_deg, target_deg, velocity_dps, torque_nm;
    /* 实际水平角=angle_deg+gravity_offset_deg。Kg载入固化默认，在线改值仅RAM。
     * 未标定时保留系数，禁止使能；不保存跨重启的软件零点/相位。 */
    float gravity_kg_nm, gravity_offset_deg, gravity_torque_nm, pid_torque_nm;
    uint8_t gravity_calibrated, gravity_i_transferred;
    /* 直接阶跃接近终点时施加附加阻尼，实际目标不经过轨迹整形。 */
    PitchTerminalBrake brake;
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
/* 直接设置最终角度目标，同时清理旧动作的制动状态。 */
uint8_t PitchController_SetTarget(PitchController *pitch, float degrees);
/* 接受离散目标并建立制动方向；相同目标不重新开始动作，也不清积分。 */
uint8_t PitchController_StartMove(PitchController *pitch, float degrees, uint32_t now_ms);
/* 归位入口可接管正在进行的动作，PID 参数和积分状态保留。 */
uint8_t PitchController_ReturnTo(PitchController *pitch, float degrees, uint32_t now_ms);
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
#endif
