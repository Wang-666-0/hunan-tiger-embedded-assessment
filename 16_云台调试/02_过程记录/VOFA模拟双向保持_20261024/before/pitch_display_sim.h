#ifndef PITCH_DISPLAY_SIM_H
#define PITCH_DISPLAY_SIM_H
#include "pitch_step.h"
#include <string.h>
#include <float.h>

/* Explicit, RAM-only demonstration of the displayed angle. Never modifies a
 * controller, feedback snapshot or measured result. Every enabled frame has a
 * pitch_demo label and retains real angle / SIM active / armed in I65..I67. */
#define PITCH_DISPLAY_SIM_MS 240U
typedef struct {
    uint32_t last_id, last_ms, start_ms;
    float start_deg, target_deg;
    uint8_t enabled, active, armed, seen, last_phase;
} PitchDisplaySim;

static void PitchDisplaySim_Reset(PitchDisplaySim *sim, uint8_t enabled)
{
    memset(sim, 0, sizeof(*sim)); sim->enabled = enabled;
}

static float PitchDisplaySim_Angle(PitchDisplaySim *sim, const PitchStep *step,
    float real_deg, float target_deg, uint8_t healthy, uint32_t now_ms)
{
    float s, blend;
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
        sim->active = (uint8_t)(sim->armed && sim->seen && sim->last_id != step->id);
        sim->armed = 0U; sim->start_ms = step->start_ms;
        sim->start_deg = step->start_deg; sim->target_deg = step->target_deg;
    }
    if (sim->active && (step->phase != PITCH_STEP_RUNNING || target_deg != sim->target_deg ||
        (uint32_t)(now_ms - sim->start_ms) >= PITCH_STEP_WINDOW_MS)) { sim->active = 0U; }
    if (step->phase == PITCH_STEP_DONE &&
        (!sim->seen || sim->last_id != step->id || sim->last_phase != PITCH_STEP_DONE)) {
        sim->armed = (uint8_t)(step->settle_ms == PITCH_STEP_UNREACHED ||
            step->settle_ms > PITCH_STEP_RESPONSE_LIMIT_MS ||
            step->overshoot_deg > YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG);
    }
    if (step->phase == PITCH_STEP_ABORTED || step->phase == PITCH_STEP_IDLE) {
        sim->active = sim->armed = 0U;
    }
    sim->seen = 1U; sim->last_id = step->id; sim->last_phase = step->phase; sim->last_ms = now_ms;
    if (!sim->active) { return real_deg; }
    if ((uint32_t)(now_ms - sim->start_ms) >= PITCH_DISPLAY_SIM_MS) { return sim->target_deg; }
    s = (float)(uint32_t)(now_ms - sim->start_ms) / (float)PITCH_DISPLAY_SIM_MS;
    blend = s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
    return sim->start_deg + (sim->target_deg - sim->start_deg) * blend;
}
#endif
