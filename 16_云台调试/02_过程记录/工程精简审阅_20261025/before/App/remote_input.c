/**
 * @file remote_input.c
 * @brief 遥控输入、安全判定与目标管理
 *
 * 职责：解析 iBUS，映射通道，形成安全许可及单发、连发、阶跃目标请求。
 * 主要函数：
 *   - ibus_feed*：寻找帧头，校验长度、通道、校验和及字节超时。
 *   - RemoteInput_Poll / GetState：消费接收队列并发布有效遥控快照。
 *   - RemoteMapper_Map：将物理通道映射为模式、方向和开关状态。
 *   - SafetyManager_Step / TargetManager_Step：先判保护，再生成边沿事件和目标。
 * 调用关系：UART4 → iBUS 队列/解析 → RemoteMapper → SafetyManager → TargetManager。
 * 维护约束：CH7 为边沿事件，长按不重复；故障后的解锁依赖新的安全 OFF 帧。
 * 参数入口：app_config.h；对外声明仍在原头文件中。
 */

/* APP_SECTION_BEGIN ibus.c */
/* iBUS 帧解析与错误后重同步。 */
#include "ibus.h"

#include <string.h>

/* 丢弃一个候选起点后仍保留后续字节，坏帧/丢字节不会跳过下一帧。 */
static void ibus_drop_first(ibus_parser_t *parser)
{
    if (parser->buffered != 0U)
    {
        parser->buffered--;
        if (parser->buffered != 0U)
        {
            memmove(parser->buffer, parser->buffer + 1U, parser->buffered);
        }
        parser->stats.discarded_bytes++;
    }
}

/* 即使块在 0x20 后结束，也保留该字节等待下一次的 0x40。 */
static void ibus_find_header(ibus_parser_t *parser)
{
    while (parser->buffered != 0U)
    {
        if (parser->buffer[0] == 0x20U)
        {
            if ((parser->buffered == 1U) || (parser->buffer[1] == 0x40U))
            {
                break;
            }
        }
        ibus_drop_first(parser);
    }
}

void ibus_init(ibus_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->inter_byte_timeout_ms = IBUS_DEFAULT_BYTE_TIMEOUT_MS;
    parser->checked_channels = IBUS_DEFAULT_CHECKED_CHANNELS;
    parser->channel_min = IBUS_DEFAULT_CHANNEL_MIN;
    parser->channel_max = IBUS_DEFAULT_CHANNEL_MAX;
}

uint8_t ibus_set_channel_validation(ibus_parser_t *parser,
                                    uint8_t count, uint16_t min, uint16_t max)
{
    if ((parser == NULL) || (count > IBUS_CHANNEL_COUNT) || (min > max))
    {
        return 0U;
    }
    parser->checked_channels = count;
    parser->channel_min = min;
    parser->channel_max = max;
    return 1U;
}

uint8_t ibus_poll_timeout(ibus_parser_t *parser, uint32_t now_ms)
{
    if ((parser == NULL) || (parser->buffered == 0U) ||
        (parser->inter_byte_timeout_ms == 0U))
    {
        return 0U;
    }
    /* 无符号减法支持毫秒计数从 UINT32_MAX 回卷到 0。 */
    if ((uint32_t)(now_ms - parser->last_byte_ms) < parser->inter_byte_timeout_ms)
    {
        return 0U;
    }
    parser->stats.discarded_bytes += parser->buffered;
    parser->buffered = 0U;
    parser->stats.partial_timeouts++;
    return 1U;
}

uint8_t ibus_feed_byte(ibus_parser_t *parser, uint8_t byte, uint32_t now_ms,
                       ibus_frame_t *out)
{
    uint16_t sum;
    uint16_t checksum;
    uint16_t received_checksum;
    uint16_t value;
    uint8_t index;
    ibus_frame_t candidate;

    if (parser == NULL)
    {
        return 0U;
    }
    (void)ibus_poll_timeout(parser, now_ms);
    parser->last_byte_ms = now_ms;
    parser->buffer[parser->buffered++] = byte;
    ibus_find_header(parser);
    if (parser->buffered < IBUS_FRAME_SIZE)
    {
        return 0U;
    }

    sum = 0U;
    for (index = 0U; index < IBUS_FRAME_SIZE - 2U; index++)
    {
        sum = (uint16_t)(sum + parser->buffer[index]);
    }
    /* 校验和为前 30 字节之和的 16 位反码：0xFFFF - sum。 */
    checksum = (uint16_t)(0xFFFFU - sum);
    received_checksum = (uint16_t)((uint16_t)parser->buffer[30] |
                                   ((uint16_t)parser->buffer[31] << 8U));
    if (checksum != received_checksum)
    {
        parser->stats.checksum_errors++;
        parser->stats.bad_frames++;
        ibus_drop_first(parser);
        ibus_find_header(parser);
        return 0U;
    }

    for (index = 0U; index < IBUS_CHANNEL_COUNT; index++)
    {
        value = (uint16_t)((uint16_t)parser->buffer[2U + 2U * index] |
                          ((uint16_t)parser->buffer[3U + 2U * index] << 8U));
        if ((index < parser->checked_channels) &&
            ((value < parser->channel_min) || (value > parser->channel_max)))
        {
            parser->stats.range_errors++;
            parser->stats.bad_frames++;
            ibus_drop_first(parser);
            ibus_find_header(parser);
            return 0U;
        }
        candidate.channels[index] = value;
    }
    candidate.received_ms = now_ms;
    parser->latest = candidate;
    parser->has_frame = 1U;
    parser->stats.valid_frames++;
    parser->buffered = 0U;
    if (out != NULL)
    {
        *out = candidate;
    }
    return 1U;
}

size_t ibus_feed(ibus_parser_t *parser, const uint8_t *data, size_t length,
                 uint32_t now_ms, ibus_frame_t *out)
{
    size_t index;
    size_t frames;

    if ((parser == NULL) || (data == NULL))
    {
        return 0U;
    }
    frames = 0U;
    for (index = 0U; index < length; index++)
    {
        frames += ibus_feed_byte(parser, data[index], now_ms, out);
    }
    return frames;
}
/* APP_SECTION_END ibus.c */

/* APP_SECTION_BEGIN remote_mapper.c */
/* 遥控通道到业务语义的纯映射。 */
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
#elif APP_REMOTE_PROFILE_ASSESSMENT
    for (i = 0U; i < 8U; ++i) {
        if (frame->channels[i] < 800U || frame->channels[i] > 2200U) { return; }
    }
#else
    for (i = 0U; i < 10U; ++i) {
        if (frame->channels[i] < 800U || frame->channels[i] > 2200U) { return; }
    }
#endif
    command->valid = 1U;
    command->yaw_rate_dps = MapRate(frame->channels[RC_YAW_CHANNEL], APP_YAW_RATE_LIMIT_DPS);
    command->arm = SwitchPosition(frame->channels[RC_ARM_CHANNEL]);
#if APP_REMOTE_PROFILE_ASSESSMENT
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
#else
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
#endif
}
/* APP_SECTION_END remote_mapper.c */

/* APP_SECTION_BEGIN safety_manager.c */
/* 安全许可状态机与新 OFF 帧解锁屏障。 */
#include "safety_manager.h"
#include "app_config.h"
#include <string.h>

static void Lock(SafetyManager *manager, SafetyReason reason, uint32_t now)
{
    manager->state = SAFETY_FAULT;
    manager->reason = reason;
    manager->last_fault = reason;
    manager->operator_armed = 0U;
    manager->output_allowed = 0U;
    manager->off_seen = 0U;
    manager->barrier_ms = now;
    manager->barrier_valid = 1U;
}

void SafetyManager_Init(SafetyManager *manager)
{
    if (manager == 0) { return; }
    memset(manager, 0, sizeof(*manager));
    manager->reason = SAFETY_POWER_ON;
}

void SafetyManager_Step(SafetyManager *manager, const SafetyInput *input,
                        const RemoteCommand *command)
{
    SafetyReason fault = SAFETY_NONE;
    uint8_t new_frame;
    if (manager == 0 || input == 0 || command == 0) { return; }
    new_frame = input->remote.has_frame && (!manager->have_sequence ||
        input->remote.sequence != manager->last_sequence);
    if (new_frame) {
        manager->have_sequence = 1U;
        manager->last_sequence = input->remote.sequence;
    }
    /* 故障代次和STOP在每个控制周期检查，不能等下一帧遥控才撤销许可。 */
    if (input->stop_sequence != manager->stop_sequence) { fault = SAFETY_STOP; }
    else if (input->uart_epoch != manager->uart_epoch) { fault = SAFETY_UART; }
    else if (input->can_epoch != manager->can_epoch) { fault = SAFETY_CAN; }
    else if (input->control_late) { fault = SAFETY_TIMING; }
    else if (!command->valid || !input->remote.has_frame ||
        (uint32_t)(input->now_ms - input->remote.received_ms) >= REMOTE_TIMEOUT_MS) {
        fault = SAFETY_REMOTE;
    }
    else if (!command->failsafe_ok) { fault = SAFETY_FAILSAFE; }
    else if (input->actuator_fault) { fault = SAFETY_ACTUATOR; }
    else if (input->backend_ready && command->arm == RC_HIGH && !input->feedback_ready) {
        fault = SAFETY_FEEDBACK;
    }
    manager->stop_sequence = input->stop_sequence;
    manager->uart_epoch = input->uart_epoch;
    manager->can_epoch = input->can_epoch;
    manager->output_allowed = 0U;
    if (fault != SAFETY_NONE) { Lock(manager, fault, input->now_ms); return; }
    if (new_frame) {
        /* 排队的旧OFF帧，包括与STOP同毫秒的帧，都不能用于重新解锁。 */
        if (manager->barrier_valid &&
            (int32_t)(input->remote.received_ms - manager->barrier_ms) <= 0) { return; }
        if (command->arm == RC_LOW) {
            manager->operator_armed = 0U;
            manager->off_seen = 1U;
            manager->state = SAFETY_READY;
            manager->reason = SAFETY_NONE;
            manager->barrier_valid = 0U;
        }
        else if (command->arm == RC_MIDDLE) {
            Lock(manager, SAFETY_ARM_POSITION, input->now_ms);
            return;
        }
        else if (!manager->operator_armed && manager->off_seen) {
            if (!command->neutral) { Lock(manager, SAFETY_NOT_NEUTRAL, input->now_ms); return; }
            manager->operator_armed = 1U;
            manager->off_seen = 0U;
            manager->state = SAFETY_ARMED;
            manager->reason = SAFETY_NONE;
        }
    }
    /* 遥控解锁可以先演示；动力许可还要求正式驱动和真实反馈均就绪。 */
    manager->output_allowed = manager->operator_armed && input->backend_ready && input->feedback_ready;
}
/* APP_SECTION_END safety_manager.c */

/* APP_SECTION_BEGIN target_manager.c */
/* 目标节拍、CH7 边沿与最近测试轴记忆。 */
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
    uint8_t feed_requested;
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
                    /* Physical CH7 up (1000): one relative -30 request.
                     * Centered selection uses the last accepted axis. */
                    if (command->step_valid || command->step_selection_centered) {
                        request->step_return_event = request->step_event = 1U;
                        request->step_delta_deg = -APP_STEP_DEGREES;
                        request->step_axis = command->step_valid ? command->step_axis : manager->last_test_axis;
                        request->step_sequence++;
                    }
#else
                    request->step_return_event = request->step_event = 1U;
                    request->step_delta_deg = -APP_STEP_DEGREES;
                    request->step_axis = GIMBAL_STEP_YAW;
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
#if APP_FEEDER_STANDALONE_TEST_ACTIVE
    /* CH6 中档单发/高档连发；摩擦轮始终为零请求。
     * 仍需本轮解锁后的 CH6 OFF，以及选好模式后的新鲜 CH8 释放/扣动。 */
    feed_requested = manager->wheel_off_seen && command->friction_on;
    (void)flywheels_ready;
#else
    request->friction_requested = manager->wheel_off_seen && command->friction_on;
    feed_requested = request->friction_requested && flywheels_ready;
#endif
    request->flywheel_rpm = request->friction_requested ? APP_FLYWHEEL_TARGET_RPM : 0.0f;
    if (command->fire_mode != manager->last_fire_mode) {
        manager->trigger_low_seen = manager->firing = 0U;
        manager->last_fire_mode = command->fire_mode;
    }
    if (!feed_requested || command->fire_mode == FIRE_SAFE) {
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
/* APP_SECTION_END target_manager.c */

/* APP_SECTION_BEGIN remote_input.c */
/* UART4 接收、队列消费及有效帧快照。 */
/* UART4 ISR 只搬运字节和实际时间；独立任务完成 iBUS 校验。
 * 丢字节/USART 错误产生故障代次：旧队列字节不能复活旧许可。
 */
#include "remote_input.h"
#include "ibus.h"
#include "app_config.h"
#include "app_fault.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

typedef struct { uint32_t received_ms, epoch; uint8_t byte; } RemoteByte;
static QueueHandle_t rx_queue;
static ibus_parser_t parser;
static RemoteInput_State state;
static volatile uint32_t fault_epoch, rx_bytes, rx_drops, rx_errors, last_uart_error;
static volatile uint32_t rx_bursts, rearm_errors;
static volatile uint8_t needs_rearm, last_rx_byte;
static uint32_t observed_epoch;
#if APP_REMOTE_RX_DMA_ENABLED
static uint8_t rx_dma[APP_REMOTE_RX_DMA_BYTES];
#else
static uint8_t rx_byte;
#endif

static HAL_StatusTypeDef StartRemoteRx(void)
{
#if APP_REMOTE_RX_DMA_ENABLED
    return RemoteUart_StartReceive(rx_dma, sizeof(rx_dma));
#else
    return HAL_UART_Receive_IT(&huart4, &rx_byte, 1U);
#endif
}

void RemoteInput_CreateQueue(void)// 创建接收队列和 iBUS 解析器
{
    rx_queue = xQueueCreate(REMOTE_RX_QUEUE_LENGTH, sizeof(RemoteByte));
    if (rx_queue == NULL) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "IBUS_BYTES");
    ibus_init(&parser);
    memset(&state, 0, sizeof(state));
}
void RemoteInput_Start(void)// 启动UART4接收，生产配置为DMA+IDLE
{
    if (StartRemoteRx() != HAL_OK) { AppFault_Fatal(APP_FAULT_UART_START); }
}
void RemoteInput_RxCplt(UART_HandleTypeDef *huart)// UART 接收完成中断回调
{
#if !APP_REMOTE_RX_DMA_ENABLED
    RemoteByte item;
    BaseType_t woken = pdFALSE;
    if (huart != &huart4) { return; }
    rx_bytes++;
    last_rx_byte = rx_byte;
    /* HAL 的错误 IRQ 可能先调用 RxCplt、随后才调用 ErrorCallback。
     * 错误字节不入队，也不能在这里重启接收（重启会清掉 ErrorCode）。
     * 交给随后执行的 ErrorCallback 保存错误、撤销许可并安排任务恢复。 */
    if (HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    item.byte = rx_byte; item.received_ms = HAL_GetTick(); item.epoch = fault_epoch;
    if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) { rx_drops++; fault_epoch++; }
    if (StartRemoteRx() != HAL_OK) { rearm_errors++; fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
#else
    (void)huart; /* ReceiveToIdle routes UART4 through RxEvent, not RxCplt. */
#endif
}

void RemoteInput_RxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
#if APP_REMOTE_RX_DMA_ENABLED
    RemoteByte item;
    uint16_t i;
    BaseType_t woken = pdFALSE;
    if (huart != &huart4 || HAL_UART_GetError(huart) != HAL_UART_ERROR_NONE) { return; }
    /* HT is disabled; normal DMA is stopped by HAL before IDLE/TC callback.
     * Copy a stable bounded burst. Parsing and safety remain in their tasks. */
    if (!size || size > sizeof(rx_dma)) { fault_epoch++; needs_rearm = 1U; return; }
    rx_bursts++; rx_bytes += size; last_rx_byte = rx_dma[size - 1U];
    item.received_ms = HAL_GetTick(); item.epoch = fault_epoch;
    for (i = 0U; i < size; ++i) {
        item.byte = rx_dma[i];
        if (xQueueSendFromISR(rx_queue, &item, &woken) != pdPASS) {
            rx_drops++; fault_epoch++; break;
        }
    }
    if (StartRemoteRx() != HAL_OK) { rearm_errors++; fault_epoch++; needs_rearm = 1U; }
    portYIELD_FROM_ISR(woken);
#else
    (void)huart; (void)size;
#endif
}
void RemoteInput_Error(UART_HandleTypeDef *huart)// UART 错误中断回调
{
    if (huart != &huart4) { return; }
    /* 后续 HAL 错误分支或重启接收会清除错误码，在恢复前保存诊断。 */
    last_uart_error = HAL_UART_GetError(huart);
    rx_errors++; fault_epoch++;
    (void)HAL_UART_AbortReceive(huart);
    needs_rearm = 1U;
}
static void RemoteInput_SyncFault(void)
{
    if (observed_epoch != fault_epoch) {
        observed_epoch = fault_epoch;
        parser.buffered = 0U; parser.has_frame = 0U;
        taskENTER_CRITICAL(); state.has_frame = 0U; taskEXIT_CRITICAL();
    }
}
void RemoteInput_Poll(void)// 轮询处理接收队列和 iBUS 解析
{
    RemoteByte item;
    ibus_frame_t frame;
    RemoteInput_SyncFault();
    if (needs_rearm) {
        taskENTER_CRITICAL();
        (void)HAL_UART_AbortReceive(&huart4);
        if (StartRemoteRx() == HAL_OK) { needs_rearm = 0U; }
        taskEXIT_CRITICAL();
    }
    for (uint32_t i = 0U; i < REMOTE_RX_QUEUE_LENGTH; i++) {
        if (xQueueReceive(rx_queue, &item, 0U) != pdPASS) { break; }
        RemoteInput_SyncFault();
        if (item.epoch != observed_epoch) { continue; }
        /* 队列积压的旧帧只允许被丢弃，不能当成新的控制指令。 */
        if ((uint32_t)(HAL_GetTick() - item.received_ms) >= REMOTE_TIMEOUT_MS) {
            parser.buffered = 0U;
            continue;
        }
        if (ibus_feed_byte(&parser, item.byte, item.received_ms, &frame)) {
            taskENTER_CRITICAL();
            if (item.epoch == fault_epoch) {
                memcpy(state.channels, frame.channels, sizeof(state.channels));
                state.received_ms = frame.received_ms;
                state.valid_frames = parser.stats.valid_frames;
                state.has_frame = 1U;
            }
            taskEXIT_CRITICAL();
        }
    }
    (void)ibus_poll_timeout(&parser, HAL_GetTick());
    taskENTER_CRITICAL();
    state.bad_frames = parser.stats.bad_frames;
    state.partial_timeouts = parser.stats.partial_timeouts;
    taskEXIT_CRITICAL();
}
void RemoteInput_GetState(RemoteInput_State *out)// 获取当前遥控输入状态
{
    if (out == NULL) { return; }
    taskENTER_CRITICAL();
    *out = state;
    /* 中断故障无需等解析任务执行即可撤销控制许可。 */
    out->fault_epoch = fault_epoch; out->rx_bytes = rx_bytes;
    out->rx_drops = rx_drops; out->rx_errors = rx_errors;
    out->last_uart_error = last_uart_error;
    out->last_rx_byte = last_rx_byte;
    out->rx_bursts = rx_bursts; out->rearm_errors = rearm_errors;
    out->needs_rearm = needs_rearm;
    if (observed_epoch != fault_epoch) { out->has_frame = 0U; }
    taskEXIT_CRITICAL();
}
/* APP_SECTION_END remote_input.c */
