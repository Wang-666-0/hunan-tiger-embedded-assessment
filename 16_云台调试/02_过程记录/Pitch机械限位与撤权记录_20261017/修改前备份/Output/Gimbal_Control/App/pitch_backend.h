#ifndef PITCH_BACKEND_H
#define PITCH_BACKEND_H
#include "pitch_controller.h"
#include "pitch_step.h"
#include "target_manager.h"

#define PITCH_BACKEND_FAULT_ENABLE (1UL << 16)
#define PITCH_BACKEND_FAULT_TX (1UL << 17)
#define PITCH_BACKEND_FAULT_DISABLED (1UL << 18)
typedef struct {
    PitchController controller;
    Dm3507Feedback feedback;
    uint32_t rx_count, feedback_age_ms;
    uint8_t online, requested, active, enabling, stop_pending;
    uint8_t zero_set, step_hold, coord_confirmed, monitoring;
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
/* +/-30 is an absolute software-zero target, measured for five seconds and then held. */
uint8_t PitchBackend_StartStep(float degrees);
void PitchBackend_GetStep(PitchStep *out);
uint8_t PitchBackend_TakeStepReport(PitchStep *out);
uint8_t PitchBackend_TakeRemoteStepEvent(PitchRemoteStepEvent *out);
#endif
