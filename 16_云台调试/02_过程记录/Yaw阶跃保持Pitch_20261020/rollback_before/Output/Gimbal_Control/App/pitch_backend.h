#ifndef PITCH_BACKEND_H
#define PITCH_BACKEND_H
#include "pitch_controller.h"
#include "pitch_step.h"
#include "target_manager.h"

#define PITCH_BACKEND_FAULT_ENABLE (1UL << 16)
#define PITCH_BACKEND_FAULT_TX (1UL << 17)
#define PITCH_BACKEND_FAULT_DISABLED (1UL << 18)
typedef enum {
    PITCH_STOP_NONE = 0U, PITCH_STOP_EXTERNAL, PITCH_STOP_PERMISSION,
    PITCH_STOP_FEEDBACK, PITCH_STOP_CAN_PERMISSION, PITCH_STOP_ENABLE_TX,
    PITCH_STOP_START_LIMIT, PITCH_STOP_ENABLE_TIMEOUT, PITCH_STOP_MODE_EXIT,
    PITCH_STOP_MANUAL_OVERRIDE, PITCH_STOP_CONTROL_FAULT, PITCH_STOP_TORQUE_TX,
    PITCH_STOP_FEEDBACK_FAULT
} PitchStopReason;
typedef struct {
    PitchController controller;
    Dm3507Feedback feedback;
    uint32_t rx_count, feedback_age_ms;
    uint32_t stop_id, stopped_ms;
    PitchStopReason stop_reason;
    uint8_t online, requested, active, enabling, stop_pending;
    uint8_t zero_set, step_hold, coord_confirmed, monitoring, yaw_step_hold;
} PitchBackendState;
typedef struct { uint32_t sequence; float target_deg; uint8_t result; } PitchRemoteStepEvent;
void PitchBackend_Init(void);
uint8_t PitchBackend_OnFrame(const CanFrame *frame);
void PitchBackend_GetState(PitchBackendState *out);
/* CC is the official status query. It does not enable, disable or set permanent zero. */
uint8_t PitchBackend_QueryFeedback(void);
/* Divisor is explicitly selected by the user, never inferred from Gr=7. */
uint8_t PitchBackend_SetCoordinate(uint8_t divisor);
uint8_t PitchBackend_CaptureReference(void);
/* READY/Request(1) records preparation only. A new remote unlock is still required. */
uint8_t PitchBackend_Request(uint8_t enabled);
void PitchBackend_Stop(void);
void PitchBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic);
uint8_t PitchBackend_TakeRearmRequest(void);
uint8_t PitchBackend_SetTarget(float degrees);
uint8_t PitchBackend_SetLoopPid(char loop, float kp, float ki, float kd);
uint8_t PitchBackend_SetLoopParameter(char loop, char parameter, float value);
uint8_t PitchBackend_SetGravityGain(float kg_nm);
uint8_t PitchBackend_SetGravityOffset(float offset_deg, uint8_t capture_horizontal);
/* +/-30 is an absolute software-zero target, measured for five seconds and then held. */
uint8_t PitchBackend_StartStep(float degrees);
/* 仅Yaw STEP使用：首次捕获Pitch实测角，保留闭环/重力补偿；重复调用不重捕获。
 * 释放后保持捕获角，不追赶旧目标；停机和故障仍优先。 */
uint8_t PitchBackend_CanHoldForYawStep(void);
uint8_t PitchBackend_HoldForYawStep(uint8_t hold);
/* 控制任务只需编号时避免复制完整测量快照；不消费报告。 */
uint32_t PitchBackend_GetStepId(void);
void PitchBackend_GetStep(PitchStep *out);
uint8_t PitchBackend_TakeStepReport(PitchStep *out);
uint8_t PitchBackend_TakeRemoteStepEvent(PitchRemoteStepEvent *out);
#endif
