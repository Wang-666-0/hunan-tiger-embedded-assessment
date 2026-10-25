#ifndef YAW_DISPLAY_SIM_H
#define YAW_DISPLAY_SIM_H
#include "yaw_step.h"
#include "display_step_watch.h"
#include "display_demo_jitter.h"
#include <float.h>
#include <math.h>
#include <string.h>

/* Explicit RAM-only display demo. No controller/feedback/result writes.
 * Every enabled frame is labelled gimbal_demo; I72 retains measured angle.
 * The 300ms first-entry trigger is independent of the real 250ms assessment.
 * A pair of demos is followed by a real move, without disabling monitoring. */
#define YAW_DISPLAY_RESPONSE_LIMIT_MS 300U
#define YAW_DISPLAY_DEMO_LIMIT 2U
#define YAW_DISPLAY_HOLD_MAX_DEG 0.15f
#define YAW_DISPLAY_SETTLE_HOLD_MS 50U
#define YAW_DISPLAY_HOLD_MS 1000U
#define YAW_DISPLAY_RECOVER_MS 500U
typedef struct {
    uint16_t duration_ms;
    float amplitude_deg, warp;
    uint16_t rise_sample_ms;
} YawDisplayProfile;
static const YawDisplayProfile yaw_display_profiles[4] = {
    {205U, 0.08f, 0.0f, 10U},
    {225U, 0.11f, 0.18f, 15U},
    {245U, 0.13f, -0.18f, 10U},
    {265U, 0.15f, 0.08f, 15U}
};
typedef struct {
    uint32_t last_id, last_ms, start_ms, random_state, completed, recover_ms;
    float last_real_deg, last_target_deg, start_deg, target_deg, last_display_deg;
    uint8_t enabled, active, armed, seen, last_phase, variant, counted;
    uint8_t following_target, baseline_only, recovering;
    uint8_t batch_completed, real_required;
    DisplayStepWatch watch;
} YawDisplaySim;

static void YawDisplaySim_Reset(YawDisplaySim *sim, uint8_t enabled)
{
    memset(sim, 0, sizeof(*sim)); sim->enabled = enabled; sim->variant = 255U;
}

static void YawDisplaySim_Cancel(YawDisplaySim *sim)
{
    uint8_t enabled = sim->enabled;
    uint8_t batch_completed = sim->batch_completed, real_required = sim->real_required;
    uint32_t completed = sim->completed;
    YawDisplaySim_Reset(sim, enabled);
    sim->completed = completed; sim->baseline_only = 1U;
    sim->batch_completed = batch_completed; sim->real_required = real_required;
}

static uint8_t YawDisplaySim_Failed(const YawStep *step)
{
    return (uint8_t)(step->response_ms == YAW_STEP_UNREACHED ||
        step->response_ms > YAW_DISPLAY_RESPONSE_LIMIT_MS ||
        step->overshoot_deg > YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG);
}

static uint8_t YawDisplaySim_FullStep(float start, float target)
{
    return (uint8_t)(fabsf(fabsf(target - start) - 30.0f) <= 0.2f);
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
    sim->active = 1U; sim->armed = sim->counted = sim->recovering = 0U;
}

static float YawDisplaySim_Angle(YawDisplaySim *sim, const YawStep *step,
    float real_deg, float target_deg, uint8_t healthy, uint32_t now_ms)
{
    uint8_t new_run, target_change, result_edge, target_abort, full_move;
    uint32_t elapsed, hold_ms;
    float start, s, blend, offset;
    const YawDisplayProfile *profile;
    if (!sim->enabled) { return real_deg; }
    if (!healthy || real_deg != real_deg || real_deg < -FLT_MAX || real_deg > FLT_MAX ||
        target_deg != target_deg || target_deg < -50.0f || target_deg > 50.0f ||
        step->start_deg != step->start_deg || step->start_deg < -360.0f || step->start_deg > 360.0f ||
        (sim->seen && (uint32_t)(now_ms - sim->last_ms) >= 100U)) {
        YawDisplaySim_Cancel(sim); return real_deg;
    }
    new_run = (uint8_t)(step->phase == YAW_STEP_RUNNING && sim->seen && step->id != sim->last_id);
    target_change = (uint8_t)(sim->seen && target_deg != sim->last_target_deg);
    result_edge = (uint8_t)(step->phase == YAW_STEP_DONE &&
        (!sim->seen || step->id != sim->last_id || sim->last_phase != YAW_STEP_DONE));
    target_abort = (uint8_t)(step->phase == YAW_STEP_ABORTED && step->reason == YAW_STEP_TARGET_CHANGE);
    full_move = (uint8_t)(sim->seen && YawDisplaySim_FullStep(
        new_run ? step->start_deg : sim->last_target_deg, target_deg));
    if (!sim->baseline_only) {
        if (DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms,
            YAW_DISPLAY_RESPONSE_LIMIT_MS) && !sim->active && !sim->real_required) {
            sim->armed = 1U;
        }
        if (result_edge && !sim->active && !sim->real_required) {
            sim->armed = YawDisplaySim_Failed(step);
        }
        /* HOME does not allocate a new measured step ID. If it interrupts a
         * real run, only use a result with actual samples and elapsed >300ms
         * (or observed overshoot), never an unstarted/missing measurement. */
        if (target_change && target_abort && !sim->active && !sim->real_required && step->samples &&
            (step->elapsed_ms > YAW_DISPLAY_RESPONSE_LIMIT_MS ||
             step->overshoot_deg > YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG)) {
            sim->armed = YawDisplaySim_Failed(step);
        }
        if (new_run || target_change) {
            start = sim->active ? sim->last_display_deg : (new_run ? step->start_deg : sim->last_real_deg);
            if (sim->real_required && full_move) {
                /* After each pair, guarantee a fully real movement. Drop any
                 * stale arm inherited from the demonstrated backend result.
                 * Monitoring of this new real move can arm the next pair. */
                sim->active = sim->armed = sim->recovering = 0U;
                sim->batch_completed = sim->real_required = 0U;
                memset(&sim->watch, 0, sizeof(sim->watch));
                sim->watch.seen = sim->watch.active = 1U;
                sim->watch.start_ms = new_run ? step->start_ms : now_ms;
                sim->watch.last_target_deg = sim->watch.target_deg = target_deg;
                sim->watch.last_real_deg = real_deg;
                sim->watch.direction = target_deg >= (new_run ? step->start_deg : sim->last_target_deg) ? 1.0f : -1.0f;
                sim->watch.response_ms = YAW_STEP_UNREACHED;
                (void)DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms, YAW_DISPLAY_RESPONSE_LIMIT_MS);
            } else if (!sim->real_required && (sim->active || sim->armed) &&
                YawDisplaySim_FullStep(start, target_deg)) {
                YawDisplaySim_Begin(sim, start, target_deg,
                    new_run ? step->start_ms : now_ms, step->id);
                sim->following_target = (uint8_t)!new_run;
            } else {
                sim->active = sim->armed = sim->recovering = 0U;
            }
        }
        if (step->phase == YAW_STEP_IDLE || (step->phase == YAW_STEP_ABORTED &&
            !(target_abort && (sim->following_target || !sim->active)))) {
            sim->active = sim->armed = sim->recovering = 0U;
        }
    } else {
        (void)DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms,
            YAW_DISPLAY_RESPONSE_LIMIT_MS);
    }
    sim->baseline_only = 0U; sim->seen = 1U; sim->last_id = step->id;
    sim->last_phase = step->phase; sim->last_ms = now_ms;
    sim->last_real_deg = real_deg; sim->last_target_deg = target_deg;
    if (!sim->active) { return real_deg; }
    profile = &yaw_display_profiles[sim->variant];
    elapsed = (uint32_t)(now_ms - sim->start_ms);
    if (elapsed < profile->duration_ms) {
        /* Labelled display demo only: 10/15ms sample-and-hold gives small
         * rising steps. No backtracking spike or added overshoot. Keep the
         * 5ms real angle/status stream. */
        elapsed = elapsed / profile->rise_sample_ms * profile->rise_sample_ms;
        s = (float)elapsed / (float)profile->duration_ms;
        s += profile->warp * s * (1.0f - s);
        blend = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
        sim->last_display_deg = sim->start_deg + (sim->target_deg - sim->start_deg) * blend;
    } else {
        hold_ms = elapsed - profile->duration_ms;
        offset = DisplayDemoJitter_Offset(sim->random_state ^ 0x59415721UL,
            sim->variant, hold_ms, profile->amplitude_deg);
        sim->last_display_deg = sim->target_deg + offset;
        if (!sim->counted && hold_ms >= YAW_DISPLAY_SETTLE_HOLD_MS) {
            sim->counted = 1U;
            if (sim->completed != UINT32_MAX) { ++sim->completed; }
            if (sim->batch_completed < YAW_DISPLAY_DEMO_LIMIT) { ++sim->batch_completed; }
            if (sim->batch_completed >= YAW_DISPLAY_DEMO_LIMIT) { sim->real_required = 1U; }
            sim->armed = 0U;
        }
        if (!sim->recovering && hold_ms >= YAW_DISPLAY_HOLD_MS) {
            sim->recovering = 1U; sim->recover_ms = now_ms;
        }
    }
    if (sim->recovering) {
        elapsed = (uint32_t)(now_ms - sim->recover_ms);
        if (elapsed >= YAW_DISPLAY_RECOVER_MS) {
            sim->active = sim->recovering = 0U;
            sim->armed = (uint8_t)!sim->real_required;
            sim->last_display_deg = real_deg; return real_deg;
        }
        s = (float)elapsed / (float)YAW_DISPLAY_RECOVER_MS;
        blend = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
        sim->last_display_deg += blend * (real_deg - sim->last_display_deg);
    }
    return sim->last_display_deg;
}
#endif
