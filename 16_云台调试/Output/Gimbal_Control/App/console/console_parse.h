/**
 * @file console_parse.h
 * @brief 共用命令数字解析
 * 职责：实现不依赖设备状态的浮点数及 PID 三参数解析。
 * 主要接口：ConsoleParse_Number, ConsoleParse_PidNumbers, for。
 * 实现/调用：头文件内 static 函数；Yaw/Pitch 命令共用，拒绝非有限值和尾随垃圾。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef CONSOLE_PARSE_H
#define CONSOLE_PARSE_H
#include "app_config.h"

#include <stdint.h>
#include <stdlib.h>
#include <float.h>

static inline uint8_t ConsoleParse_Number(const char *text, float *value)
/* 功能：严格解析一个有限浮点数，拒绝空输入和数值后的多余字符。 */
{
    char *end;
    float parsed = strtof(text, &end);
    if (end == text || *end || !(parsed >= -FLT_MAX && parsed <= FLT_MAX)) { return 0U; }
    *value = parsed; return 1U;
}

static inline uint8_t ConsoleParse_PidNumbers(const char *text, float *kp, float *ki, float *kd)
/* 功能：严格解析一组 Kp、Ki、Kd，供串口参数范围校验使用。 */
{
    char *end;
    float *values[3] = { kp, ki, kd };
    uint32_t i;
    for (i = 0U; i < 3U; ++i) {
        *values[i] = strtof(text, &end);
        if (end == text || !(*values[i] >= -FLT_MAX && *values[i] <= FLT_MAX)) { return 0U; }
        if (i == 2U) { return (uint8_t)(*end == '\0'); }
        if (*end != ' ') { return 0U; }
        text = end + 1;
    }
    return 0U;
}

#endif
