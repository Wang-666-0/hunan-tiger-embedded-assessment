#include "remote_mapper.h"
#include "app_config.h"
#include <string.h>

static RcSwitch SwitchPosition(uint16_t value)
{
    if (value <= RC_SWITCH_LOW) { return RC_LOW; }
    if (value > RC_SWITCH_HIGH) { return RC_HIGH; }
    return RC_MIDDLE;
}

static float MapRate(uint16_t value, float limit)
{
    int32_t delta = (int32_t)value - (int32_t)RC_CENTER;
    if (delta > 500) { delta = 500; }
    if (delta < -500) { delta = -500; }
    /* 扣掉死区再缩放，使死区边缘连续，不产生突然的小速度。 */
    if (delta > (int32_t)RC_DEADBAND) {
        return (float)(delta - (int32_t)RC_DEADBAND) * limit / (500.0f - RC_DEADBAND);
    }
    if (delta < -(int32_t)RC_DEADBAND) {
        return (float)(delta + (int32_t)RC_DEADBAND) * limit / (500.0f - RC_DEADBAND);
    }
    return 0.0f;
}

void RemoteMapper_Map(const RemoteFrame *frame, RemoteCommand *command)
{
    uint32_t i;
#if APP_REMOTE_PROFILE_LEGACY
    const uint8_t channels[] = {RC_YAW_CHANNEL, RC_ARM_CHANNEL, RC_FAILSAFE_CHANNEL};
#endif
    if (command == 0) { return; }
    memset(command, 0, sizeof(*command));
    if (frame == 0 || !frame->has_frame) { return; }
#if APP_REMOTE_PROFILE_LEGACY
    /* 第15项没有配置的辅助通道不参与有效性、解锁或功能判断。 */
    for (i = 0U; i < sizeof(channels); ++i) {
        if (frame->channels[channels[i]] < 800U || frame->channels[channels[i]] > 2200U) { return; }
    }
#else
    for (i = 0U; i < 10U; ++i) {
        if (frame->channels[i] < 800U || frame->channels[i] > 2200U) { return; }
    }
#endif
    command->valid = 1U;
    command->yaw_rate_dps = MapRate(frame->channels[RC_YAW_CHANNEL], APP_YAW_RATE_LIMIT_DPS);
    command->arm = SwitchPosition(frame->channels[RC_ARM_CHANNEL]);
    command->failsafe_ok = SwitchPosition(frame->channels[RC_FAILSAFE_CHANNEL]) == RC_HIGH;
#if APP_REMOTE_PROFILE_LEGACY
    command->step = RC_MIDDLE;
    command->neutral = command->yaw_rate_dps == 0.0f;
#else
    command->pitch_rate_dps = MapRate(frame->channels[RC_PITCH_CHANNEL], APP_PITCH_RATE_LIMIT_DPS);
    command->trigger = SwitchPosition(frame->channels[RC_TRIGGER_CHANNEL]);
    command->step = SwitchPosition(frame->channels[RC_STEP_CHANNEL]);
    command->friction_on = SwitchPosition(frame->channels[RC_FRICTION_CHANNEL]) == RC_HIGH;
    command->fire_mode = (FireMode)SwitchPosition(frame->channels[RC_FIRE_MODE_CHANNEL]);
    command->gimbal_mode = (GimbalMode)SwitchPosition(frame->channels[RC_TEST_MODE_CHANNEL]);
    command->neutral = command->yaw_rate_dps == 0.0f && command->pitch_rate_dps == 0.0f &&
        command->trigger == RC_LOW && command->fire_mode == FIRE_SAFE &&
        SwitchPosition(frame->channels[RC_FRICTION_CHANNEL]) == RC_LOW &&
        command->gimbal_mode == GIMBAL_MANUAL && command->step == RC_MIDDLE;
#endif
}
