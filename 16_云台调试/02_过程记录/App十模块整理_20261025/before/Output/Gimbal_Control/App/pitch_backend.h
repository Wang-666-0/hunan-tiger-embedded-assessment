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
    uint8_t zero_set, step_hold, coord_confirmed, monitoring;
    uint8_t view_locked, view_hold_pending;
    uint8_t view_prepare_pending;
    uint8_t view_hold_suspended;
    float view_hold_deg;
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
/* Active hold at the measured pose; preserve gravity calibration and speed I. */
uint8_t PitchBackend_HoldCurrent(void);
void PitchBackend_SetViewLock(uint8_t locked);
/* Renew selected-view intent after an isolated stop; no CAN/enable here. */
void PitchBackend_PrepareViewHold(void);
void PitchBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic);
uint8_t PitchBackend_TakeRearmRequest(void);
uint8_t PitchBackend_SetTarget(float degrees);
/* HOME / CH7 return can interrupt motion; ordinary STEP guards are unchanged. */
uint8_t PitchBackend_ReturnTo(float degrees);
uint8_t PitchBackend_SetLoopPid(char loop, float kp, float ki, float kd);
uint8_t PitchBackend_SetLoopParameter(char loop, char parameter, float value);
uint8_t PitchBackend_SetGravityGain(float kg_nm);
uint8_t PitchBackend_SetGravityOffset(float offset_deg, uint8_t capture_horizontal);
/* 五秒测量窗口中拒绝改变轨迹配置；相同配置幂等，不清积分。 */
uint8_t PitchBackend_SetShape(uint32_t duration_ms, float velocity_gain);
uint8_t PitchBackend_SetBrakeEnabled(uint8_t enabled);
/* +/-30 is an absolute software-zero target, measured for five seconds and then held. */
uint8_t PitchBackend_StartStep(float degrees);
/* 控制任务只需编号时避免复制完整测量快照；不消费报告。 */
uint32_t PitchBackend_GetStepId(void);
void PitchBackend_GetStep(PitchStep *out);
uint8_t PitchBackend_TakeStepReport(PitchStep *out);
uint8_t PitchBackend_TakeRemoteStepEvent(PitchRemoteStepEvent *out);
#endif
