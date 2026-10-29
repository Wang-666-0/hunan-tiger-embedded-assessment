/* Pitch 实测阶跃接口 */
#ifndef PITCH_STEP_H
#define PITCH_STEP_H
#include "app_config.h"
#include "yaw_step.h"

/* 共用纯测量实现，实例完全独立；Yaw继续250ms，Pitch独立300ms。 */
typedef YawStep PitchStep;
typedef YawStepPhase PitchStepPhase;
typedef YawStepReason PitchStepReason;
void PitchStep_Init(PitchStep *step);
uint8_t PitchStep_Start(PitchStep *step, uint32_t now_ms, float start_deg, float target_deg);
void PitchStep_Sample(PitchStep *step, uint32_t received_ms, float angle_deg,
    float velocity_dps, float torque_nm, uint8_t limited);
void PitchStep_Command(PitchStep *step, uint32_t now_ms, float torque_nm);
void PitchStep_Abort(PitchStep *step, uint32_t now_ms, PitchStepReason reason, uint32_t faults);
#endif
