/**
 * @file gimbal_step_sync.h
 * @brief 双轴阶跃与视图同步
 * 职责：同步实测阶跃编号、最近测试轴和视图保持请求。
 * 主要接口：GimbalStepSync_ApplyViewHold, YawBackend_SetViewLock, PitchBackend_SetViewLock, GimbalStepSync_FilterViewRequest, GimbalStepSync_Update 等。
 * 实现/调用：头文件内协调函数；调用时序保留在控制任务的各原同步位置。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef GIMBAL_STEP_SYNC_H
#define GIMBAL_STEP_SYNC_H
#include "app_config.h"
#include "yaw_backend.h"
#include "pitch_backend.h"
#include "app_console.h"

typedef struct { uint32_t yaw_id, pitch_id; } GimbalStepSync;

static void GimbalStepSync_ApplyViewHold(uint8_t mask)
/* 功能：把当前视图保持掩码分别提交给 Yaw 和 Pitch 后端。 */
{
    YawBackend_SetViewLock((uint8_t)(mask & APP_VIEW_HOLD_YAW));
    PitchBackend_SetViewLock((uint8_t)(mask & APP_VIEW_HOLD_PITCH));
}

static void GimbalStepSync_FilterViewRequest(TargetRequest *request, uint8_t mask)
/* 功能：屏蔽被锁轴的手动速度和阶跃事件，保留可见轴动作。 */
{
    if (mask & APP_VIEW_HOLD_YAW) { request->yaw_rate_dps = 0.0f; }
    if (mask & APP_VIEW_HOLD_PITCH) { request->pitch_rate_dps = 0.0f; }
    if (((mask & APP_VIEW_HOLD_YAW) && request->step_axis == GIMBAL_STEP_YAW) ||
        ((mask & APP_VIEW_HOLD_PITCH) && request->step_axis == GIMBAL_STEP_PITCH)) {
        /* Consume in TargetManager, but neither backend sees a wrong-axis
         * event. It must not interrupt the visible axis through companion hold. */
        request->step_event = request->step_return_event = 0U;
    }
}

/* Shared by the real control task and its integration fixture. Console STEP
 * and remote STEP use the same accepted IDs; rejected requests do not count.
 * A companion is latched once per new move, never to each measured sample. */
static void GimbalStepSync_Update(GimbalStepSync *sync, TargetManager *targets,
                                  uint8_t hold_companion)
/* 功能：识别新接受的真实阶跃编号，更新最近测试轴并按需锁住另一轴。 */
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
