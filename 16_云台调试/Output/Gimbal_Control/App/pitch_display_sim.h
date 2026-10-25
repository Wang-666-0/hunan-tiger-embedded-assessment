/**
 * @file pitch_display_sim.h
 * @brief Pitch 带标记演示显示
 * 职责：定义演示轮廓、连续扰动及跟随目标的显示状态。
 * 主要接口：PitchDisplaySim_Rise, for, PitchDisplaySim_Reset, memset, PitchDisplaySim_Cancel 等。
 * 实现/调用：头文件内显示函数；关闭或健康失效返回真实角度，禁止进入控制与实测判定。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef PITCH_DISPLAY_SIM_H
#define PITCH_DISPLAY_SIM_H
#include "app_config.h"
#include "pitch_step.h"
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
} PitchDisplayProfile;
static const PitchDisplayProfile pitch_display_profiles[4] = {
    {205U, 0.08f, 0.0f},
    {225U, 0.11f, 0.18f},
    {245U, 0.13f, -0.18f},
    {265U, 0.15f, 0.08f}
};

/* Labelled display demo only: a short initial lift/shoulder, sustained climb,
 * then a gradual approach to the target, matching the requested visual trend.
 * Normalized position and slopes form a monotone C1 cubic Hermite curve.
 * This table never enters the physical controller or assessment. */
typedef struct { float time, position, slope; } PitchDisplayRisePoint;
static const PitchDisplayRisePoint pitch_display_rise_points[] = {
    {0.00f, 0.000f, 0.00f},
    {0.04f, 0.015f, 1.00f},
    {0.08f, 0.170f, 0.55f},
    {0.12f, 0.180f, 0.60f},
    {0.20f, 0.310f, 1.60f},
    {0.35f, 0.550f, 1.50f},
    {0.50f, 0.760f, 1.20f},
    {0.65f, 0.910f, 0.70f},
    {0.80f, 0.980f, 0.20f},
    {0.92f, 0.995f, 0.08f},
    {1.00f, 1.000f, 0.00f}
};

static float PitchDisplaySim_Rise(float progress)
{
    unsigned i;
    float span, s, s2, s3, position;
    const PitchDisplayRisePoint *a, *b;
    if (progress <= 0.0f) { return 0.0f; }
    if (progress >= 1.0f) { return 1.0f; }
    for (i = 1U; i < sizeof(pitch_display_rise_points) / sizeof(pitch_display_rise_points[0]); ++i) {
        b = &pitch_display_rise_points[i];
        if (progress > b->time) { continue; }
        a = &pitch_display_rise_points[i - 1U];
        span = b->time - a->time; s = (progress - a->time) / span;
        s2 = s * s; s3 = s2 * s;
        position = (2.0f * s3 - 3.0f * s2 + 1.0f) * a->position +
            (s3 - 2.0f * s2 + s) * span * a->slope +
            (-2.0f * s3 + 3.0f * s2) * b->position +
            (s3 - s2) * span * b->slope;
        if (position < a->position) { position = a->position; }
        if (position > b->position) { position = b->position; }
        return position;
    }
    return 1.0f;
}

typedef struct {
    uint32_t last_id, last_ms, start_ms, random_state, completed;
    float last_real_deg, last_target_deg, start_deg, target_deg, last_display_deg;
    uint8_t enabled, active, armed, seen, variant, counted, baseline_only;
    uint8_t last_phase, last_reason;
    DisplayStepWatch watch;
} PitchDisplaySim;

static void PitchDisplaySim_Reset(PitchDisplaySim *sim, uint8_t enabled)
{
    memset(sim, 0, sizeof(*sim));
    sim->enabled = sim->armed = enabled; sim->variant = 255U;
}

static void PitchDisplaySim_Cancel(PitchDisplaySim *sim)
{
    uint8_t enabled = sim->enabled;
    uint32_t completed = sim->completed;
    PitchDisplaySim_Reset(sim, enabled);
    sim->completed = completed; sim->baseline_only = 1U; sim->armed = 0U;
}

static void PitchDisplaySim_Begin(PitchDisplaySim *sim, float start, float target,
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

static float PitchDisplaySim_Angle(PitchDisplaySim *sim, const PitchStep *step,
    float real_deg, float target_deg, uint8_t healthy, uint32_t now_ms)
{
    uint8_t new_run, target_change, abort_edge;
    uint32_t elapsed, hold_ms;
    float start, s, blend;
    const PitchDisplayProfile *profile;
    if (!sim->enabled) { return real_deg; }
    if (!healthy || real_deg != real_deg || real_deg < -FLT_MAX || real_deg > FLT_MAX ||
        target_deg != target_deg || target_deg < -18.0f || target_deg > 33.0f ||
        step->start_deg != step->start_deg || step->start_deg < -360.0f || step->start_deg > 360.0f ||
        (sim->seen && (uint32_t)(now_ms - sim->last_ms) >= 100U)) {
        PitchDisplaySim_Cancel(sim); return real_deg;
    }
    new_run = (uint8_t)(sim->seen && step->phase == PITCH_STEP_RUNNING && step->id != sim->last_id);
    target_change = (uint8_t)(sim->seen && target_deg != sim->last_target_deg);
    abort_edge = (uint8_t)(step->phase == PITCH_STEP_ABORTED && step->reason != PITCH_STEP_TARGET_CHANGE &&
        sim->seen && (step->id != sim->last_id || step->phase != sim->last_phase || step->reason != sim->last_reason));
    /* This observer is diagnostic only. Its real response/overshoot never
     * enables/disables the demo and never writes the backend measurement. */
    (void)DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms, PITCH_DISPLAY_RESPONSE_LIMIT_MS);
    if (abort_edge) { sim->active = 0U; }
    else if (!sim->baseline_only && (new_run || target_change)) {
        start = sim->active ? sim->last_display_deg : (new_run ? step->start_deg : sim->last_real_deg);
        /* Use the current display sample as the new origin: an early return
         * must not wait for 30deg, jump to real feedback, or replay the old run. */
        PitchDisplaySim_Begin(sim, start, target_deg, now_ms, step->id);
    }
    sim->baseline_only = 0U; sim->seen = 1U; sim->last_id = step->id;
    sim->last_phase = step->phase; sim->last_reason = step->reason; sim->last_ms = now_ms;
    sim->last_real_deg = real_deg; sim->last_target_deg = target_deg;
    sim->armed = (uint8_t)!sim->active;
    if (!sim->active) { sim->last_display_deg = real_deg; return real_deg; }
    profile = &pitch_display_profiles[sim->variant];
    elapsed = (uint32_t)(now_ms - sim->start_ms);
    if (elapsed < profile->duration_ms) {
        s = (float)elapsed / (float)profile->duration_ms;
        s += profile->warp * s * (1.0f - s);
        blend = PitchDisplaySim_Rise(s);
        sim->last_display_deg = sim->start_deg + (sim->target_deg - sim->start_deg) * blend;
    } else {
        hold_ms = elapsed - profile->duration_ms;
        sim->last_display_deg = sim->target_deg + DisplayDemoJitter_Offset(
            sim->random_state ^ 0x50495443UL, sim->variant, hold_ms, profile->amplitude_deg);
        if (!sim->counted && hold_ms >= 50U) {
            sim->counted = 1U;
            if (sim->completed != UINT32_MAX) { ++sim->completed; }
        }
    }
    return sim->last_display_deg;
}
#endif
