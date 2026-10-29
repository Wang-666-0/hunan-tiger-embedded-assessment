/**
 * @file remote_mapper.c
 * 作用：现用 CH5～CH8 遥控映射
 * 对外函数：
 *   RemoteMapper_Map
 * 内部函数：
 *   MapRate / SwitchPosition
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "remote_mapper.h"
#include "app_config.h"
#include <string.h>

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static RcSwitch SwitchPosition(uint16_t value);
static float MapRate(uint16_t value, float limit);

void RemoteMapper_Map(const RemoteFrame *frame, RemoteCommand *command)
/*
 * 作用：校验遥控通道，并把通道值映射成云台模式、角速度、发射及阶跃操作。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 参数 command：通道映射后的操作命令。
 * 流程：CH5 低值档为安全 OFF，中值档为手动，高值档为测试。
 * 流程：逻辑 arm 将中/高值档统一为解锁请求，gimbal_mode 再区分手动与测试。
 * 流程：CH3/CH4 只允许单轴阶跃选择；neutral 还检查摇杆、CH6、CH7 和扳机。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint32_t i;
    if (command == 0) { return; }
    memset(command, 0, sizeof(*command));
    if (frame == 0 || !frame->has_frame) { return; }
    for (i = 0U; i < 8U; ++i) {
        if (frame->channels[i] < 800U || frame->channels[i] > 2200U) { return; }
    }
    command->valid = 1U;
    command->yaw_rate_dps = MapRate(frame->channels[0U], APP_YAW_RATE_LIMIT_DPS);
    command->arm = SwitchPosition(frame->channels[4U]);
    {
        RcSwitch mode = command->arm;
        RcSwitch trigger = SwitchPosition(frame->channels[7U]);
        RcSwitch left_x = SwitchPosition(frame->channels[RC_STEP_YAW_CHANNEL]);
        RcSwitch left_y = SwitchPosition(frame->channels[RC_STEP_PITCH_CHANNEL]);
        command->arm = mode == RC_LOW ? RC_LOW : RC_HIGH;
        command->gimbal_mode = mode == RC_HIGH ? GIMBAL_TEST : GIMBAL_MANUAL;
        command->failsafe_ok = trigger != RC_LOW;
        /* 中档是释放，不能按历史三档扳机把中档当成无效输入。 */
        command->trigger = trigger == RC_MIDDLE ? RC_LOW :
            (trigger == RC_HIGH ? RC_HIGH : RC_MIDDLE);
        command->step = SwitchPosition(frame->channels[6U]);
        command->wheel_switch = SwitchPosition(frame->channels[5U]);
        command->friction_on = command->wheel_switch != RC_LOW;
        command->fire_mode = (FireMode)command->wheel_switch;
        command->pitch_rate_dps = MapRate(frame->channels[1U], APP_PITCH_RATE_LIMIT_DPS);
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

static float MapRate(uint16_t value, float limit)
/*
 * 作用：将摇杆通道值转换为连续角速度请求。
 * 参数 value：待处理的数值。
 * 参数 limit：该判断/换算所使用的上限。
 * 流程：以 1500 为中心，限制最大偏移为 ±500；中心 ±30 为死区。
 * 流程：先扣掉死区再按比例缩放，使刚离开死区时的请求从零连续增加。
 * 返回角速度请求，deg/s；死区内为 0。
 */
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

static RcSwitch SwitchPosition(uint16_t value)
/*
 * 作用：把遥控通道值映射为上、中、下三个业务档位。
 * 参数 value：待处理的数值。
 * 返回转换或计算后的整数值，供调用方继续处理。
 */
{
    if (value <= 1300U) { return RC_LOW; }
    if (value > 1700U) { return RC_HIGH; }
    return RC_MIDDLE;
}
