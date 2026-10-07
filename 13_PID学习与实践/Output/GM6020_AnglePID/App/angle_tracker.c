/*
 * 文件功能：把连续的单圈编码器反馈累加成相对零点的多圈角度。
 * 每次先用 ±4096 修正 8191/0 回绕，再累计真实编码差；不对目标角度
 * 做 360 度取模，因此目标 720 度确实代表正向两圈。
 * 时间差使用无符号减法，兼容 HAL 毫秒计数器回绕。20 ms 断帧、
 * 不可信跳变或超出累计保护范围都会锁存失效，即使重新在线也不
 * 恢复累计位置。只有停止、低速且反馈新鲜时的 Z 操作建立新零点。
 */
#include "angle_tracker.h"
#include "app_config.h"
#include <stddef.h>

#define ENCODER_COUNTS_PER_TURN 8192
#define ENCODER_HALF_TURN 4096
#define ENCODER_JUMP_MARGIN_COUNTS 8.0f

void AngleTracker_Init(AngleTracker *tracker)
{
    if (tracker != NULL)
    {
        tracker->accumulated_counts = 0;
        tracker->last_encoder = 0U;
        tracker->last_rx_ms = 0U;
        tracker->epoch = 0U;
        tracker->initialized = 0U;
        tracker->valid = 0U;
    }
}

void AngleTracker_CheckAge(AngleTracker *tracker, uint32_t now_ms)
{
    if (tracker != NULL && tracker->initialized != 0U &&
        (uint32_t)(now_ms - tracker->last_rx_ms) >= ANGLE_TRACK_MAX_GAP_MS)
    {
        tracker->valid = 0U;
    }
}

uint8_t AngleTracker_Update(AngleTracker *tracker, uint16_t encoder,
                            int16_t speed_rpm, uint32_t now_ms)
{
    uint32_t elapsed_ms;
    int32_t delta;
    int32_t speed_magnitude = speed_rpm;
    int64_t next_counts;
    float max_delta;

    if (tracker == NULL)
    {
        return 0U;
    }
    if (encoder >= ENCODER_COUNTS_PER_TURN)
    {
        tracker->valid = 0U;
        return 0U;
    }
    if (speed_magnitude < 0)
    {
        speed_magnitude = -speed_magnitude;
    }
    if (tracker->initialized == 0U)
    {
        /* 第一条反馈只定义零点，不能把开机时的绝对编码位置当位移。 */
        tracker->last_encoder = encoder;
        tracker->last_rx_ms = now_ms;
        tracker->accumulated_counts = 0;
        tracker->initialized = 1U;
        tracker->epoch++;
        tracker->valid = (uint8_t)((float)speed_magnitude <=
                                   MOTOR_PHYSICAL_SPEED_LIMIT_RPM);
        return tracker->valid;
    }

    elapsed_ms = (uint32_t)(now_ms - tracker->last_rx_ms);
    delta = (int32_t)encoder - (int32_t)tracker->last_encoder;
    if (delta > ENCODER_HALF_TURN)
    {
        delta -= ENCODER_COUNTS_PER_TURN;
    }
    else if (delta < -ENCODER_HALF_TURN)
    {
        delta += ENCODER_COUNTS_PER_TURN;
    }

    /* 即使角度已失效也保存最新帧，以便反馈在线后可以执行显式 Z。
     * 保留 counts 能显示失效前位置，但 valid=0 时禁止用于角度闭环。 */
    tracker->last_encoder = encoder;
    tracker->last_rx_ms = now_ms;
    if (elapsed_ms >= ANGLE_TRACK_MAX_GAP_MS ||
        (float)speed_magnitude > MOTOR_PHYSICAL_SPEED_LIMIT_RPM)
    {
        tracker->valid = 0U;
    }
    if (tracker->valid == 0U)
    {
        return 0U;
    }

    /* 毫秒时间戳有量化误差，真实间隔可能略小于 elapsed_ms+1。
     * 因此同一毫秒中的两帧仍允许合理移动；额外 8 格容忍编码量化。
     * 这里只用物理最高转速判定可信度，不用反馈速度推算圈数。 */
    max_delta = MOTOR_PHYSICAL_SPEED_LIMIT_RPM *
                (float)ENCODER_COUNTS_PER_TURN * (float)(elapsed_ms + 1U) /
                60000.0f + ENCODER_JUMP_MARGIN_COUNTS;
    if ((float)delta > max_delta || (float)delta < -max_delta)
    {
        tracker->valid = 0U;
        return 0U;
    }

    next_counts = tracker->accumulated_counts + (int64_t)delta;
    if (next_counts > ANGLE_TRACK_MAX_COUNTS ||
        next_counts < -ANGLE_TRACK_MAX_COUNTS)
    {
        tracker->valid = 0U;
        return 0U;
    }
    tracker->accumulated_counts = next_counts;
    return 1U;
}

uint8_t AngleTracker_Zero(AngleTracker *tracker, uint16_t encoder,
                          uint32_t now_ms)
{
    if (tracker == NULL || tracker->initialized == 0U ||
        encoder >= ENCODER_COUNTS_PER_TURN)
    {
        return 0U;
    }
    /* 允许从 invalid 恢复，但必须有新鲜帧；速度和停机状态由调用层
     * 在同一临界区或其配置事务中校验，不能用过期位置建立零点。 */
    if ((uint32_t)(now_ms - tracker->last_rx_ms) >= ANGLE_TRACK_MAX_GAP_MS)
    {
        tracker->valid = 0U;
        return 0U;
    }
    tracker->last_encoder = encoder;
    tracker->accumulated_counts = 0;
    tracker->valid = 1U;
    tracker->epoch++;
    return 1U;
}

float AngleTracker_AngleDeg(const AngleTracker *tracker)
{
    if (tracker == NULL)
    {
        return 0.0f;
    }
    return (float)tracker->accumulated_counts * (360.0f / 8192.0f);
}
