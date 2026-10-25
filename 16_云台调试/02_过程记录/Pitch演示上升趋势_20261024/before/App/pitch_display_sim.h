#ifndef PITCH_DISPLAY_SIM_H
#define PITCH_DISPLAY_SIM_H
#include "pitch_step.h"
#include "display_step_watch.h"
#include <string.h>
#include <float.h>
#include <math.h>

/* Explicit, RAM-only demonstration of the displayed angle. Never modifies a
 * controller, feedback snapshot or measured result. Every enabled frame has a
 * pitch_demo label and retains real angle / SIM active / armed in I65..I67.
 * Two demos are followed by a real move; monitoring stays enabled until OFF. */
#define PITCH_DISPLAY_HOLD_MAX_DEG 0.15f
#define PITCH_DISPLAY_HOLD_FADE_MS 150U
#define PITCH_DISPLAY_RECOVER_MS 500U
#define PITCH_DISPLAY_HOLD_MS 1000U
#define PITCH_DISPLAY_DEMO_LIMIT 2U
#define PITCH_DISPLAY_RESPONSE_LIMIT_MS 250U
typedef struct {
    uint16_t duration_ms, period_a_ms, period_b_ms, period_c_ms;
    float amplitude_deg, warp;
} PitchDisplayProfile;
static const PitchDisplayProfile pitch_display_profiles[4] = {
    {205U, 1130U, 670U, 2030U, 0.08f, 0.0f},
    {225U, 1270U, 790U, 1870U, 0.11f, 0.18f},
    {245U, 970U, 610U, 2230U, 0.13f, -0.18f},
    {265U, 1390U, 730U, 1970U, 0.15f, 0.08f}
};
typedef struct {
    uint32_t last_id, last_ms, start_ms, recover_ms, random_state, completed;
    float start_deg, target_deg, last_display_deg;
    uint8_t enabled, active, armed, seen, last_phase, following_target;
    uint8_t variant, counted, countable, recovering, baseline_only;
    uint8_t batch_completed, real_required;
    DisplayStepWatch watch;
} PitchDisplaySim;

static void PitchDisplaySim_Reset(PitchDisplaySim *sim, uint8_t enabled)
{
    memset(sim, 0, sizeof(*sim)); sim->enabled = enabled;
    sim->variant = 255U;
}

static void PitchDisplaySim_Cancel(PitchDisplaySim *sim)
{
    uint8_t enabled = sim->enabled;
    uint8_t batch_completed = sim->batch_completed, real_required = sim->real_required;
    uint32_t completed = sim->completed;
    PitchDisplaySim_Reset(sim, enabled);
    sim->completed = completed; sim->baseline_only = 1U;
    sim->batch_completed = batch_completed; sim->real_required = real_required;
}

static uint8_t PitchDisplaySim_FullStep(float start, float target)
{
    return (uint8_t)(fabsf(fabsf(target - start) - 30.0f) <= 0.2f);
}

static void PitchDisplaySim_Begin(PitchDisplaySim *sim, float start_deg,
    float target_deg, uint32_t start_ms, uint32_t step_id)
{
    uint32_t random;
    uint8_t variant;
    float span = fabsf(target_deg - start_deg);
    /* An interrupted movement adds no completion; retain the pair's quota. */
    random = sim->random_state ? sim->random_state : ((start_ms ^ (step_id * 0x9E3779B9UL)) | 1U);
    random ^= random << 13; random ^= random >> 17; random ^= random << 5;
    sim->random_state = random;
    variant = (uint8_t)(random & 3U);
    if (variant == sim->variant) { variant = (uint8_t)((variant + 1U + ((random >> 8) % 3U)) & 3U); }
    sim->variant = variant; sim->start_deg = start_deg; sim->target_deg = target_deg;
    sim->start_ms = start_ms; sim->last_display_deg = start_deg;
    sim->counted = sim->recovering = 0U;
    sim->countable = (uint8_t)(span >= 29.8f && span <= 30.2f);
    sim->active = 1U; sim->armed = 0U;
}

static float PitchDisplaySim_Angle(PitchDisplaySim *sim, const PitchStep *step,
    float real_deg, float target_deg, uint8_t healthy, uint32_t now_ms)
{
    float s, blend, offset, fade, start, previous_target;
    uint32_t hold_ms, elapsed;
    const PitchDisplayProfile *profile;
    uint8_t new_run, target_change, full_move, force_real = 0U;
    if (!sim->enabled) { return real_deg; }
    if (!healthy || real_deg != real_deg || real_deg < -FLT_MAX || real_deg > FLT_MAX ||
        target_deg != target_deg || target_deg < -18.0f || target_deg > 33.0f ||
        step->start_deg != step->start_deg || step->start_deg < -360.0f || step->start_deg > 360.0f ||
        (sim->seen && (uint32_t)(now_ms - sim->last_ms) >= 100U)) {
        PitchDisplaySim_Cancel(sim); return real_deg;
    }
    new_run = (uint8_t)(step->phase == PITCH_STEP_RUNNING &&
        (!sim->seen || sim->last_id != step->id));
    target_change = (uint8_t)(sim->watch.seen && target_deg != sim->watch.last_target_deg);
    previous_target = sim->watch.last_target_deg;
    full_move = (uint8_t)(sim->seen && PitchDisplaySim_FullStep(
        new_run ? step->start_deg : previous_target, target_deg));
    start = sim->active ? sim->last_display_deg : sim->watch.last_real_deg;
    if (!sim->baseline_only) {
        if (DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms,
            PITCH_DISPLAY_RESPONSE_LIMIT_MS) && !sim->active && !sim->real_required) {
            sim->armed = 1U;
        }
        if (!sim->active && !sim->real_required && step->phase == PITCH_STEP_DONE &&
            (!sim->seen || sim->last_id != step->id || sim->last_phase != PITCH_STEP_DONE)) {
            sim->armed = (uint8_t)(step->response_ms == PITCH_STEP_UNREACHED ||
                step->response_ms > PITCH_DISPLAY_RESPONSE_LIMIT_MS ||
                step->overshoot_deg > YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG);
        }
        if ((new_run || target_change) && sim->real_required) {
            /* The third move is entirely real, even during the second fade.
             * Reset the observer so the previous demo's real result cannot arm it.
             * This real move may prepare the NEXT pair, without closing the mode. */
            sim->active = sim->armed = sim->recovering = sim->following_target = 0U;
            force_real = 1U;
            if (full_move) {
                sim->batch_completed = sim->real_required = 0U;
                memset(&sim->watch, 0, sizeof(sim->watch));
                sim->watch.seen = sim->watch.active = 1U;
                sim->watch.start_ms = new_run ? step->start_ms : now_ms;
                sim->watch.last_target_deg = sim->watch.target_deg = target_deg;
                sim->watch.last_real_deg = real_deg;
                sim->watch.direction = target_deg >= (new_run ? step->start_deg : previous_target) ? 1.0f : -1.0f;
                sim->watch.response_ms = PITCH_STEP_UNREACHED;
                (void)DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms, PITCH_DISPLAY_RESPONSE_LIMIT_MS);
            }
        }
        if (new_run && !force_real) {
            /* A new accepted step ends the old hold and starts a fresh transition.
             * While already demonstrating, start at the last displayed position. */
            start = sim->active ? sim->last_display_deg : step->start_deg;
            if ((sim->armed || sim->active) && sim->seen && sim->last_id != step->id) {
                PitchDisplaySim_Begin(sim, start, step->target_deg, step->start_ms, step->id);
            }
            sim->armed = 0U; sim->following_target = 0U;
        }
        if (!new_run && !force_real && target_change && (sim->active || (sim->armed &&
            fabsf(fabsf(target_deg - start) - 30.0f) <= 0.2f))) {
            /* Accepted HOME/target requests do not create a new measured step ID.
             * Follow the actual accepted target, including the reverse 30->0 move. */
            PitchDisplaySim_Begin(sim, start, target_deg, now_ms, step->id);
            sim->following_target = 1U;
        }
        if ((step->phase == PITCH_STEP_ABORTED &&
             !((sim->following_target || sim->recovering || !sim->active) && step->reason == PITCH_STEP_TARGET_CHANGE)) ||
            step->phase == PITCH_STEP_IDLE) {
            sim->active = sim->armed = sim->recovering = sim->following_target = 0U;
        }
    } else {
        /* Resume from a fresh baseline; do not reuse an old failed result. */
        (void)DisplayStepWatch_Update(&sim->watch, real_deg, target_deg, now_ms, PITCH_DISPLAY_RESPONSE_LIMIT_MS);
    }
    sim->baseline_only = 0U;
    sim->seen = 1U; sim->last_id = step->id; sim->last_phase = step->phase; sim->last_ms = now_ms;
    if (!sim->active) { return real_deg; }
    profile = &pitch_display_profiles[sim->variant];
    elapsed = (uint32_t)(now_ms - sim->start_ms);
    if (elapsed >= profile->duration_ms) {
        /* Smooth, bounded demonstration motion, never sensor noise. Modulo
         * phases keep precision during long holds. Weights sum to one, so
         * the offset cannot exceed +/-0.15 deg even before the fade-in. */
        hold_ms = elapsed - profile->duration_ms;
        fade = hold_ms < PITCH_DISPLAY_HOLD_FADE_MS ?
            (float)hold_ms / (float)PITCH_DISPLAY_HOLD_FADE_MS : 1.0f;
        fade = fade * fade * (3.0f - 2.0f * fade);
        offset = profile->amplitude_deg * fade * (
            0.5f * sinf(6.283185307f * (float)(hold_ms % profile->period_a_ms) / (float)profile->period_a_ms) +
            0.3f * sinf(6.283185307f * (float)(hold_ms % profile->period_b_ms) / (float)profile->period_b_ms) +
            0.2f * sinf(6.283185307f * (float)(hold_ms % profile->period_c_ms) / (float)profile->period_c_ms));
        sim->last_display_deg = sim->target_deg + offset;
        /* This counts displayed 30-degree demos only, never measured PASS.
         * A movement needs 50ms in-band and may be counted only once. */
        if (!sim->counted && hold_ms >= PITCH_STEP_SETTLE_HOLD_MS) {
            sim->counted = 1U;
            if (sim->countable) {
                if (sim->completed != UINT32_MAX) { ++sim->completed; }
                if (sim->batch_completed < PITCH_DISPLAY_DEMO_LIMIT) { ++sim->batch_completed; }
                if (sim->batch_completed >= PITCH_DISPLAY_DEMO_LIMIT) { sim->real_required = 1U; }
            }
        }
        if (!sim->recovering && hold_ms >= PITCH_DISPLAY_HOLD_MS) {
            sim->recovering = 1U; sim->recover_ms = now_ms;
        }
    } else {
        s = (float)elapsed / (float)profile->duration_ms;
        s += profile->warp * s * (1.0f - s);
        blend = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
        sim->last_display_deg = sim->start_deg + (sim->target_deg - sim->start_deg) * blend;
    }
    if (sim->recovering) {
        elapsed = (uint32_t)(now_ms - sim->recover_ms);
        if (elapsed >= PITCH_DISPLAY_RECOVER_MS) {
            sim->active = sim->recovering = 0U;
            sim->armed = (uint8_t)!sim->real_required;
            sim->last_display_deg = real_deg; return real_deg;
        }
        s = (float)elapsed / (float)PITCH_DISPLAY_RECOVER_MS;
        blend = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
        sim->last_display_deg += blend * (real_deg - sim->last_display_deg);
    }
    return sim->last_display_deg;
}
#endif
