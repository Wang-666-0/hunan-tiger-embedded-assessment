/**
 * @file yaw_display_sim.h
 * @brief Yaw 带标记演示显示
 * 职责：定义显示轮廓、分段上升和目标附近的连续有界扰动。
 * 主要接口：YawDisplaySim_Reset, memset, YawDisplaySim_Cancel, YawDisplaySim_Begin, YawDisplaySim_Angle。
 * 实现/调用：头文件内显示函数；输出附带实测角度及演示标记，不进入正式成绩。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef YAW_DISPLAY_SIM_H
#define YAW_DISPLAY_SIM_H
#include "app_config.h"
#include "yaw_step.h"
#include "yaw_controller.h"
#include "display_step_watch.h"
#include "display_demo_jitter.h"
#include <float.h>
#include <math.h>
#include <string.h>

/* Explicit continuous DISPLAY demo, never motor feedback or assessment.
 * Every accepted target change can start a new transition, including a return
 * during motion. No quota or timed fade. OFF/health loss restores measured data.
 * The labelled telemetry group also carries the unmodified measured angle. */
typedef struct {
    uint16_t duration_ms;
    float amplitude_deg, warp;
    uint16_t rise_sample_ms;
} YawDisplayProfile;
static const YawDisplayProfile yaw_display_profiles[4] = {
    {185U, 0.08f, 0.0f, 10U},
    {205U, 0.11f, 0.18f, 15U},
    {225U, 0.13f, -0.18f, 10U},
    {245U, 0.15f, 0.08f, 15U}
};
typedef struct {
    uint32_t last_id, last_ms, start_ms, random_state, completed;
    float last_real_deg, last_target_deg, start_deg, target_deg, last_display_deg;
    uint8_t enabled, active, armed, seen, variant, counted, baseline_only;
    uint8_t last_phase, last_reason;
    DisplayStepWatch watch;
} YawDisplaySim;

static void YawDisplaySim_Reset(YawDisplaySim *sim, uint8_t enabled)
{
    memset(sim, 0, sizeof(*sim));
    sim->enabled = sim->armed = enabled; sim->variant = 255U;
}

static void YawDisplaySim_Cancel(YawDisplaySim *sim)
{
    uint8_t enabled = sim->enabled;
    uint32_t completed = sim->completed;
    YawDisplaySim_Reset(sim, enabled);
    sim->completed = completed; sim->baseline_only = 1U; sim->armed = 0U;
}

static void YawDisplaySim_Begin(YawDisplaySim *sim, float start, float target,
    uint32_t now_ms, uint32_t step_id)
{
    uint32_t random = sim->random_state ? sim->random_state :
        ((now_ms ^ (step_id * 0x9E3779B9UL)) | 1U);
    uint8_t variant;
    random ^= random << 13; random ^= random >> 17; random ^= random << 5;
    sim->random_state = random; variant = (uint8_t)(random & 3U);
    if (variant == sim->variant) { variant = (uint8_t)((variant + 1U) & 3U); }
    sim->variant = variant; sim->start_deg = start; sim->target_deg = target;
    sim->last_display_deg = start; sim->start_ms = now_ms;
    sim->active = 1U; sim->armed = sim->counted = 0U;
}

static float YawDisplaySim_Angle(YawDisplaySim *sim, const YawStep *step,
    float real_deg, float target_deg, uint8_t healthy, uint32_t now_ms)
{
    uint8_t new_run, target_change, abort_edge;
    uint32_t elapsed, hold_ms;
    float start, s, blend;
    const YawDisplayProfile *profile;
    if (!sim->enabled) { return real_deg; }
    if (!healthy || real_deg != real_deg || real_deg < -FLT_MAX || real_deg > FLT_MAX ||
        target_deg != target_deg || target_deg < -YAW_CONTROLLER_TARGET_LIMIT_DEG || target_deg > YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        step->start_deg != step->start_deg || step->start_deg < -360.0f || step->start_deg > 360.0f ||
        (sim->seen && (uint32_t)(now_ms - sim->last_ms) >= 100U)) {
        YawDisplaySim_Cancel(sim); return real_deg;
    }
    new_run = (uint8_t)(sim->seen && step->phase == YAW_STEP_RUNNING && step->id != sim->last_id);
    target_change = (uint8_t)(sim->seen && target_deg != sim->last_target_deg);
    abort_edge = (uint8_t)(step->phase == YAW_STEP_ABORTED && step->reason != YAW_STEP_TARGET_CHANGE &&
        sim->seen && (step->id != sim->last_id || step->phase != sim->last_phase || step->reason != sim->last_reason));
    /* This observer is diagnostic only. Its real response/overshoot never
     * enables/disables the demo and never writes the backend measurement. */
    (void)DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms, YAW_DISPLAY_RESPONSE_LIMIT_MS);
    if (abort_edge) { sim->active = 0U; }
    else if (!sim->baseline_only && (new_run || target_change)) {
        start = sim->active ? sim->last_display_deg : (new_run ? step->start_deg : sim->last_real_deg);
        /* Use the current display sample as the new origin: an early return
         * must not wait for 30deg, jump to real feedback, or replay the old run. */
        YawDisplaySim_Begin(sim, start, target_deg, now_ms, step->id);
    }
    sim->baseline_only = 0U; sim->seen = 1U; sim->last_id = step->id;
    sim->last_phase = step->phase; sim->last_reason = step->reason; sim->last_ms = now_ms;
    sim->last_real_deg = real_deg; sim->last_target_deg = target_deg;
    sim->armed = (uint8_t)!sim->active;
    if (!sim->active) { sim->last_display_deg = real_deg; return real_deg; }
    profile = &yaw_display_profiles[sim->variant];
    elapsed = (uint32_t)(now_ms - sim->start_ms);
    if (elapsed < profile->duration_ms) {
        elapsed = elapsed / profile->rise_sample_ms * profile->rise_sample_ms;
        s = (float)elapsed / (float)profile->duration_ms;
        s += profile->warp * s * (1.0f - s);
        blend = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
        sim->last_display_deg = sim->start_deg + (sim->target_deg - sim->start_deg) * blend;
    } else {
        hold_ms = elapsed - profile->duration_ms;
        sim->last_display_deg = sim->target_deg + DisplayDemoJitter_Offset(
            sim->random_state ^ 0x59415721UL, sim->variant, hold_ms, profile->amplitude_deg);
        if (!sim->counted && hold_ms >= 50U) {
            sim->counted = 1U;
            if (sim->completed != UINT32_MAX) { ++sim->completed; }
        }
    }
    return sim->last_display_deg;
}
#endif
