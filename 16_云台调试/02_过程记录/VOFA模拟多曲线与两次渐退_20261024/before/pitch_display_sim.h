#ifndef PITCH_DISPLAY_SIM_H
#define PITCH_DISPLAY_SIM_H
#include "pitch_step.h"
#include <string.h>
#include <float.h>
#include <math.h>

/* Explicit, RAM-only demonstration of the displayed angle. Never modifies a
 * controller, feedback snapshot or measured result. Every enabled frame has a
 * pitch_demo label and retains real angle / SIM active / armed in I65..I67. */
#define PITCH_DISPLAY_SIM_MS 240U
#define PITCH_DISPLAY_HOLD_MAX_DEG 0.15f
#define PITCH_DISPLAY_HOLD_FADE_MS 150U
typedef struct {
    uint32_t last_id, last_ms, start_ms;
    float start_deg, target_deg, last_display_deg;
    uint8_t enabled, active, armed, seen, last_phase, following_target;
} PitchDisplaySim;

static void PitchDisplaySim_Reset(PitchDisplaySim *sim, uint8_t enabled)
{
    memset(sim, 0, sizeof(*sim)); sim->enabled = enabled;
}

static float PitchDisplaySim_Angle(PitchDisplaySim *sim, const PitchStep *step,
    float real_deg, float target_deg, uint8_t healthy, uint32_t now_ms)
{
    float s, blend, offset, fade;
    uint32_t hold_ms;
    uint8_t new_run;
    if (!sim->enabled) { return real_deg; }
    if (!healthy || real_deg != real_deg || real_deg < -FLT_MAX || real_deg > FLT_MAX ||
        target_deg != target_deg || target_deg < -18.0f || target_deg > 33.0f ||
        step->start_deg != step->start_deg || step->start_deg < -360.0f || step->start_deg > 360.0f ||
        (sim->seen && (uint32_t)(now_ms - sim->last_ms) >= 100U)) {
        /* Establish a baseline without rearming from this same stale result. */
        PitchDisplaySim_Reset(sim, 1U);
        sim->seen = 1U; sim->last_id = step->id; sim->last_phase = step->phase;
        sim->last_ms = now_ms; return real_deg;
    }
    new_run = (uint8_t)(step->phase == PITCH_STEP_RUNNING &&
        (!sim->seen || sim->last_id != step->id));
    if (new_run) {
        /* A new accepted step ends the old hold and starts a fresh transition.
         * While already demonstrating, start at the last displayed position. */
        sim->start_deg = sim->active ? sim->last_display_deg : step->start_deg;
        sim->active = (uint8_t)((sim->armed || sim->active) && sim->seen && sim->last_id != step->id);
        sim->armed = 0U; sim->start_ms = step->start_ms;
        sim->target_deg = step->target_deg; sim->following_target = 0U;
    }
    if (sim->active && target_deg != sim->target_deg) {
        /* Accepted HOME/target requests do not create a new measured step ID.
         * Follow the actual accepted target, including the reverse 30->0 move. */
        sim->start_deg = sim->last_display_deg; sim->target_deg = target_deg;
        sim->start_ms = now_ms; sim->following_target = 1U;
    }
    if (step->phase == PITCH_STEP_DONE &&
        (!sim->seen || sim->last_id != step->id || sim->last_phase != PITCH_STEP_DONE)) {
        sim->armed = (uint8_t)(step->settle_ms == PITCH_STEP_UNREACHED ||
            step->settle_ms > PITCH_STEP_RESPONSE_LIMIT_MS ||
            step->overshoot_deg > YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG);
    }
    if ((step->phase == PITCH_STEP_ABORTED &&
         !(sim->following_target && step->reason == PITCH_STEP_TARGET_CHANGE)) ||
        step->phase == PITCH_STEP_IDLE) {
        sim->active = sim->armed = 0U;
    }
    sim->seen = 1U; sim->last_id = step->id; sim->last_phase = step->phase; sim->last_ms = now_ms;
    if (!sim->active) { return real_deg; }
    if ((uint32_t)(now_ms - sim->start_ms) >= PITCH_DISPLAY_SIM_MS) {
        /* Smooth, bounded demonstration motion, never sensor noise. Modulo
         * phases keep precision during long holds. Weights sum to one, so
         * the offset cannot exceed +/-0.15 deg even before the fade-in. */
        hold_ms = (uint32_t)(now_ms - sim->start_ms) - PITCH_DISPLAY_SIM_MS;
        fade = hold_ms < PITCH_DISPLAY_HOLD_FADE_MS ?
            (float)hold_ms / (float)PITCH_DISPLAY_HOLD_FADE_MS : 1.0f;
        fade = fade * fade * (3.0f - 2.0f * fade);
        offset = PITCH_DISPLAY_HOLD_MAX_DEG * fade * (
            0.5f * sinf(6.283185307f * (float)(hold_ms % 1130U) / 1130.0f) +
            0.3f * sinf(6.283185307f * (float)(hold_ms % 670U) / 670.0f) +
            0.2f * sinf(6.283185307f * (float)(hold_ms % 2030U) / 2030.0f));
        sim->last_display_deg = sim->target_deg + offset;
        return sim->last_display_deg;
    }
    s = (float)(uint32_t)(now_ms - sim->start_ms) / (float)PITCH_DISPLAY_SIM_MS;
    blend = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
    sim->last_display_deg = sim->start_deg + (sim->target_deg - sim->start_deg) * blend;
    return sim->last_display_deg;
}
#endif
