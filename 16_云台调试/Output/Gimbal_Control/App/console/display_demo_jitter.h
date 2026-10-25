/**
 * @file display_demo_jitter.h
 * @brief 带标记演示的连续扰动
 * 职责：生成连续、有界的不规则显示扰动，用于演示角度。
 * 主要接口：DisplayDemoJitter_Node, DisplayDemoJitter_Band, DisplayDemoJitter_Offset。
 * 实现/调用：头文件内纯显示函数；不修改真实反馈、闭环状态或考核成绩。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef DISPLAY_DEMO_JITTER_H
#define DISPLAY_DEMO_JITTER_H
#include "app_config.h"
#include <stdint.h>

/* Explicit display-demo texture only. Never feed this into measured feedback,
 * motor control or assessment. Interpolated hashed knots give bounded,
 * continuous irregular motion, instead of a repeated long sine wave. */

static float DisplayDemoJitter_Node(uint32_t seed, uint32_t index, uint32_t salt)
/* 功能：由节点编号和种子生成确定的显示扰动节点值。 */
{
    uint32_t value = seed ^ (index * 0x9E3779B9UL) ^ salt;
    value ^= value >> 16; value *= 0x7FEB352DUL;
    value ^= value >> 15; value *= 0x846CA68BUL;
    value ^= value >> 16;
    return (float)(value & 0xFFFFU) * (2.0f / 65535.0f) - 1.0f;
}

static float DisplayDemoJitter_Band(uint32_t seed, uint32_t elapsed_ms,
    uint32_t knot_ms, uint32_t salt)
/* 功能：在相邻扰动节点之间插值，生成连续的显示波动。 */
{
    uint32_t index = elapsed_ms / knot_ms;
    float fraction = (float)(elapsed_ms % knot_ms) / (float)knot_ms;
    float a = DisplayDemoJitter_Node(seed, index, salt);
    float b = DisplayDemoJitter_Node(seed, index + 1U, salt);
    return (1.0f - fraction) * a + fraction * b;
}

static float DisplayDemoJitter_Offset(uint32_t seed, uint8_t variant,
    uint32_t elapsed_ms, float amplitude_deg)
/* 功能：合成并限制演示显示的角度扰动，结果不进入真实控制。 */
{
    uint32_t profile = (uint32_t)(variant & 3U);
    float envelope, offset;
    if (amplitude_deg < 0.0f) { amplitude_deg = 0.0f; }
    if (amplitude_deg > DISPLAY_DEMO_JITTER_MAX_DEG) { amplitude_deg = DISPLAY_DEMO_JITTER_MAX_DEG; }
    envelope = elapsed_ms < 150U ? (float)elapsed_ms / 150.0f : 1.0f;
    envelope = envelope * envelope * (3.0f - 2.0f * envelope);
    /* Each band lies in [-1,1], and nonnegative weights sum to one.
     * Linear knot interpolation is continuous but retains small corners. */
    offset = amplitude_deg * envelope * (
        0.50f * DisplayDemoJitter_Band(seed, elapsed_ms, 30U + 5U * profile, 0xA511E9B3UL) +
        0.30f * DisplayDemoJitter_Band(seed, elapsed_ms, 73U + 11U * profile, 0x63D83595UL) +
        0.20f * DisplayDemoJitter_Band(seed, elapsed_ms, 211U + 23U * profile, 0xB5297A4DUL));
    if (offset < -amplitude_deg) { offset = -amplitude_deg; }
    if (offset > amplitude_deg) { offset = amplitude_deg; }
    return offset;
}
#endif
