#ifndef DISPLAY_STEP_WATCH_H
#define DISPLAY_STEP_WATCH_H
#include "yaw_step.h"
#include <math.h>
/* Display trigger only, sampled with the 5ms telemetry clock. Tracks accepted
 * 30deg target changes, including HOME, without changing backend assessment.
 * RESP here has telemetry resolution; backend RESULT remains CAN-timestamped. */
typedef struct {
    float last_target_deg, last_real_deg, target_deg, direction, overshoot_deg;
    uint32_t start_ms, response_ms, elapsed_ms;
    uint8_t seen, active, failed;
} DisplayStepWatch;

static uint8_t DisplayStepWatch_Update(DisplayStepWatch *watch, float real_deg,
    float target_deg, uint32_t now_ms, uint32_t limit_ms)
{
    uint8_t failed = watch->failed;
    float delta, overshoot;
    if (watch->seen && target_deg != watch->last_target_deg) {
        delta = target_deg - watch->last_target_deg;
        watch->active = (uint8_t)(fabsf(fabsf(delta) - 30.0f) <= 0.2f);
        watch->start_ms = now_ms; watch->target_deg = target_deg;
        watch->direction = delta >= 0.0f ? 1.0f : -1.0f;
        watch->response_ms = YAW_STEP_UNREACHED;
        watch->elapsed_ms = 0U; watch->overshoot_deg = 0.0f; watch->failed = 0U;
    }
    if (watch->active) {
        watch->elapsed_ms = (uint32_t)(now_ms - watch->start_ms);
        if (watch->response_ms == YAW_STEP_UNREACHED &&
            fabsf(target_deg - real_deg) <= YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG) {
            watch->response_ms = watch->elapsed_ms;
        }
        overshoot = watch->direction * (real_deg - watch->target_deg);
        if (overshoot > watch->overshoot_deg) { watch->overshoot_deg = overshoot; }
        if ((watch->response_ms == YAW_STEP_UNREACHED && watch->elapsed_ms > limit_ms) ||
            (watch->response_ms != YAW_STEP_UNREACHED && watch->response_ms > limit_ms) ||
            watch->overshoot_deg > YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG) {
            watch->failed = 1U;
        }
    }
    watch->seen = 1U; watch->last_target_deg = target_deg; watch->last_real_deg = real_deg;
    return (uint8_t)(failed || watch->failed);
}
#endif
