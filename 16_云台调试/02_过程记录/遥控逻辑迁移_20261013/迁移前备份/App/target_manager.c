#include "target_manager.h"
#include "app_config.h"
#include <string.h>

void TargetManager_Init(TargetManager *manager)
{
    if (manager != 0) { memset(manager, 0, sizeof(*manager)); }
}

static void Cancel(TargetManager *manager)
{
    uint32_t shots = manager->request.shot_sequence, steps = manager->request.step_sequence;
    memset(&manager->request, 0, sizeof(manager->request));
    manager->request.shot_sequence = shots;
    manager->request.step_sequence = steps;
    manager->trigger_low_seen = 0U;
    manager->step_center_seen = 0U;
    manager->firing = 0U;
    manager->last_fire_mode = FIRE_SAFE;
    manager->last_gimbal_mode = GIMBAL_MANUAL;
}

void TargetManager_Step(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms)
{
    TargetRequest *request;
    if (manager == 0 || command == 0) { return; }
    request = &manager->request;
    request->shot_event = 0U;
    request->step_event = 0U;
    if (!operator_armed || !command->valid) { Cancel(manager); return; }
    request->gimbal_mode = command->gimbal_mode;
    request->fire_mode = command->fire_mode;
    request->yaw_rate_dps = 0.0f;
    request->pitch_rate_dps = 0.0f;
    request->friction_requested = 0U;
    request->flywheel_rpm = 0.0f;
    if (command->gimbal_mode != manager->last_gimbal_mode) {
        request->step_delta_deg = 0.0f; /* 换轴或回手动时不沿用上一轴的偏移。 */
        manager->step_center_seen = 0U;
        manager->trigger_low_seen = 0U;
        manager->firing = 0U;
        manager->last_gimbal_mode = command->gimbal_mode;
    }
    if (command->gimbal_mode != GIMBAL_MANUAL) {
        manager->trigger_low_seen = 0U;
        manager->firing = 0U;
        if (new_frame && command->step == RC_MIDDLE) {
            manager->step_center_seen = 1U;
            request->step_delta_deg = 0.0f;
        }
        else if (new_frame && manager->step_center_seen) {
            request->step_delta_deg = command->step == RC_HIGH ? APP_STEP_DEGREES : -APP_STEP_DEGREES;
            request->step_event = 1U;
            request->step_sequence++;
            manager->step_center_seen = 0U;
        }
        return; /* 阶跃模式禁止摩擦轮和拨盘，防止辅助旋钮误触发发射。 */
    }
    request->yaw_rate_dps = command->yaw_rate_dps;
    request->pitch_rate_dps = command->pitch_rate_dps;
    request->friction_requested = command->friction_on;
    request->flywheel_rpm = command->friction_on ? APP_FLYWHEEL_TARGET_RPM : 0.0f;
    if (command->fire_mode != manager->last_fire_mode) {
        manager->trigger_low_seen = 0U;
        manager->firing = 0U;
        manager->last_fire_mode = command->fire_mode;
    }
    if (!command->friction_on || !flywheels_ready || command->fire_mode == FIRE_SAFE) {
        manager->trigger_low_seen = 0U;
        manager->firing = 0U;
        return;
    }
    if (new_frame && command->trigger == RC_LOW) {
        manager->trigger_low_seen = 1U;
        manager->firing = 0U;
    }
    else if (new_frame && command->trigger == RC_MIDDLE) {
        manager->trigger_low_seen = 0U;
        manager->firing = 0U;
    }
    else if (new_frame && command->trigger == RC_HIGH && manager->trigger_low_seen) {
        request->shot_event = 1U;
        request->shot_sequence++;
        manager->trigger_low_seen = 0U;
        manager->firing = command->fire_mode == FIRE_CONTINUOUS;
        manager->next_shot_ms = now_ms + APP_FIRE_INTERVAL_MS;
    }
    else if (manager->firing && command->trigger == RC_HIGH &&
             (int32_t)(now_ms - manager->next_shot_ms) >= 0) {
        request->shot_event = 1U;
        request->shot_sequence++;
        /* 最小间隔50ms；迟到只产生一次事件，不追赶积压的发射请求。 */
        manager->next_shot_ms = now_ms + APP_FIRE_INTERVAL_MS;
    }
}
