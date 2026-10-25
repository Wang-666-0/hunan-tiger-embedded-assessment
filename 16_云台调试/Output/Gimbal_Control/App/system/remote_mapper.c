/**
 * @file remote_mapper.c
 * @brief 现用 CH5～CH8 遥控映射
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（1）：
 *   RemoteMapper_Map
 * 内部函数（2）：
 *   SwitchPosition / MapRate
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 遥控通道到业务语义的纯映射。 */
#include "remote_mapper.h"
#include "app_config.h"
#include <string.h>

static RcSwitch SwitchPosition(uint16_t value)
/* 功能：把遥控通道值映射为上、中、下三个业务档位。 */
{
    if (value <= RC_SWITCH_LOW) { return RC_LOW; }
    if (value > RC_SWITCH_HIGH) { return RC_HIGH; }
    return RC_MIDDLE;
}

static float MapRate(uint16_t value, float limit)
/* 功能：按死区和速度标尺把摇杆通道转换为连续角速度请求。 */
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
/* 功能：校验现用遥控通道，并映射 CH5 模式、CH6 发射、CH7 阶跃和 CH8 安全扳机。 */
{
    uint32_t i;
    if (command == 0) { return; }
    memset(command, 0, sizeof(*command));
    if (frame == 0 || !frame->has_frame) { return; }
    for (i = 0U; i < 8U; ++i) {
        if (frame->channels[i] < 800U || frame->channels[i] > 2200U) { return; }
    }
    command->valid = 1U;
    command->yaw_rate_dps = MapRate(frame->channels[RC_YAW_CHANNEL], APP_YAW_RATE_LIMIT_DPS);
    command->arm = SwitchPosition(frame->channels[RC_ARM_CHANNEL]);
    {
        RcSwitch mode = command->arm;
        RcSwitch trigger = SwitchPosition(frame->channels[RC_TRIGGER_CHANNEL]);
        RcSwitch left_x = SwitchPosition(frame->channels[RC_STEP_YAW_CHANNEL]);
        RcSwitch left_y = SwitchPosition(frame->channels[RC_STEP_PITCH_CHANNEL]);
        command->arm = mode == RC_LOW ? RC_LOW : RC_HIGH;
        command->gimbal_mode = mode == RC_HIGH ? GIMBAL_TEST : GIMBAL_MANUAL;
        command->failsafe_ok = trigger != RC_LOW;
        /* 中档是释放，不能按历史三档扳机把中档当成无效输入。 */
        command->trigger = trigger == RC_MIDDLE ? RC_LOW :
            (trigger == RC_HIGH ? RC_HIGH : RC_MIDDLE);
        command->step = SwitchPosition(frame->channels[RC_STEP_CHANNEL]);
        command->wheel_switch = SwitchPosition(frame->channels[RC_FRICTION_CHANNEL]);
        command->friction_on = command->wheel_switch != RC_LOW;
        command->fire_mode = (FireMode)command->wheel_switch;
        command->pitch_rate_dps = MapRate(frame->channels[RC_PITCH_CHANNEL], APP_PITCH_RATE_LIMIT_DPS);
        command->step_selection_centered = (uint8_t)(
            frame->channels[RC_STEP_YAW_CHANNEL] >= RC_CENTER - RC_DEADBAND &&
            frame->channels[RC_STEP_YAW_CHANNEL] <= RC_CENTER + RC_DEADBAND &&
            frame->channels[RC_STEP_PITCH_CHANNEL] >= RC_CENTER - RC_DEADBAND &&
            frame->channels[RC_STEP_PITCH_CHANNEL] <= RC_CENTER + RC_DEADBAND);
        if ((left_x != RC_MIDDLE) != (left_y != RC_MIDDLE)) {
            command->step_valid = 1U;
            command->step_axis = left_x != RC_MIDDLE ? GIMBAL_STEP_YAW : GIMBAL_STEP_PITCH;
            command->step_delta_deg = (left_x != RC_MIDDLE ? left_x : left_y) == RC_HIGH ?
                APP_STEP_DEGREES : -APP_STEP_DEGREES;
        }
        command->neutral = command->yaw_rate_dps == 0.0f && command->pitch_rate_dps == 0.0f &&
            command->wheel_switch == RC_LOW && command->trigger == RC_LOW &&
            command->step == RC_MIDDLE && command->step_selection_centered;
    }
}
