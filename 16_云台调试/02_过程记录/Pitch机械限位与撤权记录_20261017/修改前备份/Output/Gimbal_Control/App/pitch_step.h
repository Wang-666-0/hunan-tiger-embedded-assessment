#ifndef PITCH_STEP_H
#define PITCH_STEP_H
#include "yaw_step.h"

/* 共用纯测量实现，实例完全独立；Yaw继续250ms，Pitch独立300ms。 */
typedef YawStep PitchStep;
typedef YawStepPhase PitchStepPhase;
typedef YawStepReason PitchStepReason;
#define PITCH_STEP_WINDOW_MS YAW_STEP_WINDOW_MS
#define PITCH_STEP_STEADY_MS YAW_STEP_STEADY_MS
#define PITCH_STEP_BAND_DEG YAW_STEP_BAND_DEG
#define PITCH_STEP_SETTLE_HOLD_MS YAW_STEP_SETTLE_HOLD_MS
#define PITCH_STEP_RESPONSE_LIMIT_MS 300U
#define PITCH_STEP_UNREACHED YAW_STEP_UNREACHED
#define PITCH_STEP_IDLE YAW_STEP_IDLE
#define PITCH_STEP_RUNNING YAW_STEP_RUNNING
#define PITCH_STEP_DONE YAW_STEP_DONE
#define PITCH_STEP_ABORTED YAW_STEP_ABORTED
#define PITCH_STEP_NONE YAW_STEP_NONE
#define PITCH_STEP_STOP YAW_STEP_STOP
#define PITCH_STEP_FAULT YAW_STEP_FAULT
#define PITCH_STEP_PID_CHANGE YAW_STEP_PID_CHANGE
#define PITCH_STEP_TARGET_CHANGE YAW_STEP_TARGET_CHANGE
#define PITCH_STEP_MANUAL YAW_STEP_MANUAL
#define PITCH_STEP_GAP YAW_STEP_GAP
#define PITCH_STEP_DATA YAW_STEP_DATA
void PitchStep_Init(PitchStep *step);
uint8_t PitchStep_Start(PitchStep *step, uint32_t now_ms, float start_deg, float target_deg);
void PitchStep_Sample(PitchStep *step, uint32_t received_ms, float angle_deg,
    float velocity_dps, float torque_nm, uint8_t limited);
void PitchStep_Command(PitchStep *step, uint32_t now_ms, float torque_nm);
void PitchStep_Abort(PitchStep *step, uint32_t now_ms, PitchStepReason reason, uint32_t faults);
#endif
