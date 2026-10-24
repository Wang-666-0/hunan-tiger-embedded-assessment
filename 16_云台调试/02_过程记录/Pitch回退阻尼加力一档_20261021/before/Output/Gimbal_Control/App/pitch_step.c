#include "pitch_step.h"

static float absolute(float x) { return x < 0.0f ? -x : x; }
void PitchStep_Init(PitchStep *step) { YawStep_Init(step); }
uint8_t PitchStep_Start(PitchStep *step, uint32_t now_ms, float start_deg, float target_deg)
{
    if (absolute(start_deg) > 50.0f || absolute(target_deg) > 50.0f) { return 0U; }
    return YawStep_Start(step, now_ms, start_deg, target_deg);
}
void PitchStep_Sample(PitchStep *step, uint32_t received_ms, float angle_deg,
    float velocity_dps, float torque_nm, uint8_t limited)
{
    uint8_t running = (uint8_t)(step && step->phase == PITCH_STEP_RUNNING);
    YawStep_Sample(step, received_ms, angle_deg, velocity_dps, torque_nm, limited);
    if (running && step->phase == PITCH_STEP_DONE) {
        step->pass = YawStep_Passed(step, PITCH_STEP_RESPONSE_LIMIT_MS);
    }
}
void PitchStep_Command(PitchStep *step, uint32_t now_ms, float torque_nm)
{ YawStep_Command(step, now_ms, torque_nm); }
void PitchStep_Abort(PitchStep *step, uint32_t now_ms, PitchStepReason reason, uint32_t faults)
{ YawStep_Abort(step, now_ms, reason, faults); }
