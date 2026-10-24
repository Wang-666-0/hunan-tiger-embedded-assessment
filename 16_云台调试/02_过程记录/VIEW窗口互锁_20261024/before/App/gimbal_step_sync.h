#ifndef GIMBAL_STEP_SYNC_H
#define GIMBAL_STEP_SYNC_H
#include "yaw_backend.h"
#include "pitch_backend.h"

typedef struct { uint32_t yaw_id, pitch_id; } GimbalStepSync;

/* Shared by the real control task and its integration fixture. Console STEP
 * and remote STEP use the same accepted IDs; rejected requests do not count.
 * A companion is latched once per new move, never to each measured sample. */
static void GimbalStepSync_Update(GimbalStepSync *sync, TargetManager *targets,
                                  uint8_t hold_companion)
{
    uint32_t yaw_id = YawBackend_GetStepId(), pitch_id = PitchBackend_GetStepId();
    if (yaw_id != sync->yaw_id) {
        if (hold_companion) { (void)PitchBackend_HoldCurrent(); }
        TargetManager_RecordAcceptedTestAxis(targets, GIMBAL_STEP_YAW);
        sync->yaw_id = yaw_id;
    }
    if (pitch_id != sync->pitch_id) {
        if (hold_companion) { (void)YawBackend_HoldCurrent(); }
        TargetManager_RecordAcceptedTestAxis(targets, GIMBAL_STEP_PITCH);
        sync->pitch_id = pitch_id;
    }
}
#endif
