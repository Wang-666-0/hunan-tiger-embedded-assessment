#include "target_manager.h"
#include "app_config.h"
#include <string.h>

void TargetManager_Init(TargetManager *manager)
{
    if (manager != 0) {
        memset(manager, 0, sizeof(*manager));
        manager->last_test_axis = GIMBAL_STEP_YAW;
    }
}

void TargetManager_RecordAcceptedTestAxis(TargetManager *manager, GimbalMode axis)
{
    if (manager != 0 && (axis == GIMBAL_STEP_YAW || axis == GIMBAL_STEP_PITCH)) {
        manager->last_test_axis = axis;
    }
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
    manager->wheel_off_seen = 0U;
    manager->last_step_switch = RC_MIDDLE;
}

static void RepeatShot(TargetManager *manager, uint32_t now_ms)
{
    uint32_t late_ms = now_ms - manager->next_shot_ms;
    manager->request.shot_event = 1U;
    manager->request.shot_sequence++;
    /* Keep the 50ms phase through normal one-control-tick jitter. A genuinely
     * late control call emits only one event and starts a new interval; it
     * must never squeeze missed requests into a catch-up burst. */
    manager->next_shot_ms = late_ms <= APP_CONTROL_PERIOD_MS ?
        manager->next_shot_ms + APP_FIRE_INTERVAL_MS : now_ms + APP_FIRE_INTERVAL_MS;
}

#if APP_REMOTE_PROFILE_ASSESSMENT
static void AssessmentStep(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms)
{
    TargetRequest *request = &manager->request;
    request->shot_event = request->step_event = request->step_return_event = request->continuous_fire = 0U;
    if (!operator_armed || !command->valid || !command->failsafe_ok) { Cancel(manager); return; }
    if (command->gimbal_mode != manager->last_gimbal_mode) {
        request->step_delta_deg = 0.0f;
        request->step_axis = GIMBAL_MANUAL;
        manager->step_center_seen = manager->trigger_low_seen = manager->firing = 0U;
        manager->wheel_off_seen = 0U;
        manager->last_step_switch = command->step;
        manager->last_gimbal_mode = command->gimbal_mode;
    }
    request->gimbal_mode = command->gimbal_mode;
    request->fire_mode = command->fire_mode;
    request->yaw_rate_dps = request->pitch_rate_dps = 0.0f;
    request->friction_requested = 0U;
    request->flywheel_rpm = 0.0f;
    if (command->gimbal_mode == GIMBAL_TEST) {
        manager->wheel_off_seen = manager->trigger_low_seen = manager->firing = 0U;
        if (new_frame) {
            if (command->step == RC_MIDDLE) { manager->step_center_seen = 1U; }
            else if (command->step == RC_LOW) {
                if (manager->last_step_switch != RC_LOW) {
#if APP_PITCH_ENABLED
                    /* 回中使用最后实际接受轴；斜向不能猜轴或回两轴。 */
                    if (command->step_valid || command->step_selection_centered) {
                        request->step_return_event = request->step_event = 1U;
                        request->step_delta_deg = 0.0f;
                        request->step_axis = command->step_valid ? command->step_axis : manager->last_test_axis;
                        request->step_sequence++;
                    }
#else
                    request->step_return_event = request->step_event = 1U;
                    request->step_delta_deg = 0.0f;
                    request->step_axis = GIMBAL_MANUAL;
                    request->step_sequence++;
#endif
                }
                manager->step_center_seen = 0U;
            } else if (manager->step_center_seen) {
                if (command->step_valid) {
                    /* 未选轴时不消耗确认；保持CH7高位后选好单轴即可。
                     * 一旦发布即消耗，长按不会重复发起，下一次仍需新中位。 */
                    manager->step_center_seen = 0U;
                    request->step_axis = command->step_axis;
                    request->step_delta_deg = command->step_delta_deg;
                    request->step_event = 1U;
                    request->step_sequence++;
                } else if (!command->step_selection_centered) {
                    /* 斜向仍拒绝并消耗确认，改为单轴后须重新经过中位。 */
                    manager->step_center_seen = 0U;
                }
            }
            manager->last_step_switch = command->step;
        }
        return; /* 此模块只发布单次请求，由相应轴后端执行，测试禁止射击。 */
    }
    request->yaw_rate_dps = command->yaw_rate_dps;
    request->pitch_rate_dps = command->pitch_rate_dps;
    if (new_frame && command->wheel_switch == RC_LOW) { manager->wheel_off_seen = 1U; }
    request->friction_requested = manager->wheel_off_seen && command->friction_on;
    request->flywheel_rpm = request->friction_requested ? APP_FLYWHEEL_TARGET_RPM : 0.0f;
    if (command->fire_mode != manager->last_fire_mode) {
        manager->trigger_low_seen = manager->firing = 0U;
        manager->last_fire_mode = command->fire_mode;
    }
    if (!request->friction_requested || !flywheels_ready || command->fire_mode == FIRE_SAFE) {
        manager->trigger_low_seen = manager->firing = 0U;
        return;
    }
    if (new_frame && command->trigger == RC_LOW) {
        manager->trigger_low_seen = 1U; manager->firing = 0U;
    } else if (new_frame && command->trigger == RC_HIGH && manager->trigger_low_seen) {
        request->shot_event = 1U; request->shot_sequence++;
        manager->trigger_low_seen = 0U;
        manager->firing = command->fire_mode == FIRE_CONTINUOUS;
        manager->next_shot_ms = now_ms + APP_FIRE_INTERVAL_MS;
    } else if (manager->firing && command->trigger == RC_HIGH &&
               (int32_t)(now_ms - manager->next_shot_ms) >= 0) {
        RepeatShot(manager, now_ms);
    }
    /* Observation of the existing release/trigger latch; no new shot source. */
    request->continuous_fire = manager->firing && command->trigger == RC_HIGH;
}
#endif

void TargetManager_Step(TargetManager *manager, const RemoteCommand *command,
    uint8_t new_frame, uint8_t operator_armed, uint8_t flywheels_ready, uint32_t now_ms)
{
#if APP_REMOTE_PROFILE_ASSESSMENT
    if (manager == 0 || command == 0) { return; }
    AssessmentStep(manager, command, new_frame, operator_armed, flywheels_ready, now_ms);
#else
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
        RepeatShot(manager, now_ms);
    }
#endif
}
