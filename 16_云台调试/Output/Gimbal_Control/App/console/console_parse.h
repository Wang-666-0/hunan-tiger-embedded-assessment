/* 共用命令数字解析 */
#ifndef CONSOLE_PARSE_H
#define CONSOLE_PARSE_H
#include "app_config.h"

#include <stdint.h>
#include <stdlib.h>
#include <float.h>

static inline uint8_t ConsoleParse_Number(const char *text, float *value)
{
    char *end;
    float parsed = strtof(text, &end);
    if (end == text || *end || !(parsed >= -FLT_MAX && parsed <= FLT_MAX)) { return 0U; }
    *value = parsed; return 1U;
}

static inline uint8_t ConsoleParse_PidNumbers(const char *text, float *kp, float *ki, float *kd)
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
