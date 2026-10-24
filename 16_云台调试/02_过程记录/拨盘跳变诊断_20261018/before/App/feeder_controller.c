#include "feeder_controller.h"
#include <float.h>
#include <string.h>

#define HALF_TICK_RANGE 0x80000000UL
#define COUNTS_TO_DEGREES (360.0 / (double)FEEDER_ENCODER_COUNTS)

static float absolute(float x) { return x < 0.0f ? -x : x; }
static double abs_double(double x) { return x < 0.0 ? -x : x; }
static float clamp(float x, float low, float high)
{ return x < low ? low : (x > high ? high : x); }
static uint8_t finite_value(float x)
{ return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
static uint8_t within(float x, float low, float high)
{ return (uint8_t)(finite_value(x) && x >= low && x <= high); }
static void positions(FeederController *c)
{
    c->position_deg = (double)(c->accumulated_counts - c->zero_counts) * COUNTS_TO_DEGREES;
    c->axis_position_deg = c->position_deg / (double)c->config.gear_ratio;
    c->axis_target_deg = c->target_deg / (double)c->config.gear_ratio;
}
static void reset_output(FeederController *c)
{
    c->command_raw = 0; c->target_rpm = c->integral = 0.0f;
    c->settling = c->jamming = c->have_control_tick = 0U;
}
void Feeder_Stop(FeederController *c)
{
    if (!c) { return; }
    if (c->busy) { ++c->aborted; }
    c->busy = 0U; reset_output(c);
    c->target_deg = c->position_deg;
    c->axis_target_deg = c->axis_position_deg;
}
static void fault(FeederController *c, uint32_t reason)
{
    c->faults |= reason; Feeder_Stop(c); c->have_zero = 0U;
    c->recovery_samples = 0U;
}
static uint8_t fresh(const FeederController *c, uint32_t now)
{
    uint32_t age = now - c->last_feedback_ms;
    return (uint8_t)(c->have_feedback && age <= FEEDER_FEEDBACK_TIMEOUT_MS);
}
uint8_t Feeder_ConfigValid(const FeederConfig *p)
{
    if (!p) { return 0U; }
    return (uint8_t)(within(p->step_deg, 0.1f, 360.0f) &&
        within(p->gear_ratio, 0.1f, 100.0f) &&
        (p->direction == -1 || p->direction == 0 || p->direction == 1) &&
        p->calibrated <= 1U && (!p->calibrated || p->direction != 0) &&
        within(p->pos_kp, 0.001f, 100.0f) &&
        within(p->speed_kp, 0.001f, 100.0f) && within(p->speed_ki, 0.0f, 1000.0f) &&
        within(p->max_rpm, 1.0f, 6000.0f) &&
        within(p->max_feedback_rpm, p->max_rpm, 6000.0f) &&
        within(p->current_limit, 1.0f, (float)FEEDER_CURRENT_CONFIG_LIMIT) &&
        within(p->tolerance_deg, 0.05f, 5.0f) &&
        within(p->settle_rpm, 0.0f, 100.0f) &&
        within(p->jam_rpm, 0.0f, 100.0f) &&
        within(p->jam_current_raw, 1.0f, (float)FEEDER_CURRENT_CONFIG_LIMIT) &&
        p->settle_ms >= 5U && p->settle_ms <= 500U &&
        p->motion_timeout_ms >= 50U && p->motion_timeout_ms <= 5000U &&
        p->jam_ms >= 20U && p->jam_ms <= 2000U && p->jam_ms < p->motion_timeout_ms);
}
void Feeder_Init(FeederController *c)
{
    if (!c) { return; }
    memset(c, 0, sizeof(*c));
    c->config.step_deg = 40.0f; c->config.gear_ratio = 1.0f;
    c->config.pos_kp = 5.0f; c->config.speed_kp = 2.0f; c->config.speed_ki = 5.0f;
    c->config.max_rpm = 300.0f; c->config.max_feedback_rpm = 6000.0f;
    c->config.current_limit = 1000.0f; c->config.tolerance_deg = 1.0f;
    c->config.settle_rpm = 10.0f; c->config.settle_ms = 20U;
    c->config.motion_timeout_ms = 1000U;
    c->config.jam_rpm = 10.0f; c->config.jam_current_raw = 800.0f; c->config.jam_ms = 150U;
}
uint8_t Feeder_SetConfig(FeederController *c, const FeederConfig *p)
{
    uint8_t invalidate;
    if (!c || c->busy || !Feeder_ConfigValid(p)) { return 0U; }
    invalidate = (uint8_t)(p->gear_ratio != c->config.gear_ratio ||
        p->direction != c->config.direction || p->calibrated != c->config.calibrated);
    c->config = *p; Feeder_Stop(c);
    if (invalidate) { c->have_zero = 0U; }
    positions(c); c->axis_target_deg = c->axis_position_deg;
    return 1U;
}
uint8_t Feeder_OnFrame(FeederController *c, const CanFrame *f, uint32_t now, uint32_t epoch)
{
    uint16_t angle;
    int16_t rpm, current;
    int32_t delta = 0;
    uint32_t gap = 0U, reasons = 0U;
    float rpm_bound, counts_bound, expected_counts;
    uint8_t first, rebase = 0U;
    if (!c || !f || f->bus != 2U || f->id != 0x205U || f->extended) { return 0U; }
    if (f->dlc != 8U) { fault(c, FEEDER_FAULT_DATA); return 0U; }
    if (c->have_feedback && c->bus_epoch != epoch) {
        fault(c, FEEDER_FAULT_CAN); rebase = 1U;
    }
    if (f->fault_epoch != epoch) { return 0U; }
    if (now - f->received_ms > FEEDER_FEEDBACK_TIMEOUT_MS) {
        fault(c, FEEDER_FAULT_FEEDBACK); return 0U;
    }
    angle = (uint16_t)(((uint16_t)f->data[0] << 8) | f->data[1]);
    rpm = (int16_t)(((uint16_t)f->data[2] << 8) | f->data[3]);
    current = (int16_t)(((uint16_t)f->data[4] << 8) | f->data[5]);
    if (angle >= FEEDER_ENCODER_COUNTS) { fault(c, FEEDER_FAULT_DATA); return 0U; }
    if (absolute((float)rpm) > c->config.max_feedback_rpm) { reasons |= FEEDER_FAULT_OVERSPEED; }
    if (absolute((float)current) > (float)FEEDER_CURRENT_PROTOCOL_LIMIT) { reasons |= FEEDER_FAULT_CURRENT; }
    first = (uint8_t)!c->have_feedback;
    if (!first) {
        gap = f->received_ms - c->last_feedback_ms;
        if (gap >= HALF_TICK_RANGE || (!gap && !rebase)) { return 0U; }
        if (gap > FEEDER_FEEDBACK_TIMEOUT_MS) { reasons |= FEEDER_FAULT_FEEDBACK; rebase = 1U; }
        delta = (int32_t)angle - (int32_t)c->raw_angle;
        if (delta > 4096) { delta -= 8192; }
        else if (delta < -4096) { delta += 8192; }
        rpm_bound = absolute((float)rpm);
        if (absolute((float)c->raw_rpm) > rpm_bound) { rpm_bound = absolute((float)c->raw_rpm); }
        expected_counts = rpm_bound * (float)gap * ((float)FEEDER_ENCODER_COUNTS / 60000.0f);
        /* At >= half a rotor revolution between samples, shortest-path unwrap
         * is ambiguous. Lose the zero rather than guessing unseen revolutions. */
        counts_bound = (rpm_bound + 120.0f) * (float)gap *
            ((float)FEEDER_ENCODER_COUNTS / 60000.0f) + 32.0f;
        if (!rebase && (expected_counts >= 4064.0f || absolute((float)delta) > counts_bound)) {
            reasons |= FEEDER_FAULT_JUMP; rebase = 1U;
        }
    }
    if (reasons) { fault(c, reasons); rebase = 1U; }
    if (!first && !rebase) { c->accumulated_counts += delta; }
    c->raw_angle = angle; c->raw_rpm = rpm; c->raw_current = current;
    c->last_feedback_ms = f->received_ms; c->bus_epoch = epoch;
    c->have_feedback = 1U; ++c->feedback_sequence;
    positions(c);
    if (!c->busy && !c->have_zero) {
        c->target_deg = c->position_deg; c->axis_target_deg = c->axis_position_deg;
    }
    if (!reasons && !rebase && absolute((float)rpm) <= c->config.settle_rpm &&
        absolute((float)delta) <= 2.0f) {
        if (c->recovery_samples < 255U) { ++c->recovery_samples; }
    } else { c->recovery_samples = 0U; }
    return 1U;
}
uint8_t Feeder_Clear(FeederController *c, uint32_t now)
{
    if (!c || c->busy || !fresh(c, now) || c->recovery_samples < 3U ||
        absolute((float)c->raw_rpm) > c->config.settle_rpm ||
        absolute((float)c->raw_current) > (float)FEEDER_CURRENT_PROTOCOL_LIMIT) { return 0U; }
    Feeder_Stop(c); c->faults = 0U; c->have_zero = 0U;
    return 1U;
}
uint8_t Feeder_Zero(FeederController *c, uint32_t now)
{
    if (!c || c->busy || c->faults || !fresh(c, now) || c->recovery_samples < 3U ||
        !c->config.calibrated || !c->config.direction ||
        absolute((float)c->raw_rpm) > c->config.settle_rpm) { return 0U; }
    Feeder_Stop(c); c->zero_counts = c->accumulated_counts;
    c->position_deg = c->target_deg = c->axis_position_deg = c->axis_target_deg = 0.0;
    c->have_zero = 1U; return 1U;
}
static uint8_t consume_event(FeederController *c, uint8_t event, uint32_t sequence)
{
    uint32_t distance;
    if (!event) { return 0U; }
    distance = sequence - c->last_shot_sequence;
    if (c->have_shot_sequence && (!distance || distance >= HALF_TICK_RANGE)) { return 0U; }
    c->have_shot_sequence = 1U; c->last_shot_sequence = sequence; return 1U;
}
int16_t Feeder_Step(FeederController *c, uint32_t now, uint8_t allowed,
                    uint8_t wheels_ready, uint8_t shot_event, uint32_t sequence)
{
    uint8_t event, new_feedback;
    uint32_t tick_gap = 0U, feedback_gap;
    float error, speed_error, p, candidate, output, dt;
    if (!c) { return 0; }
    event = consume_event(c, shot_event, sequence);
    if (c->have_feedback && !fresh(c, now)) { fault(c, FEEDER_FAULT_FEEDBACK); }
    if (!allowed || !wheels_ready || c->faults || !c->have_zero ||
        !c->config.calibrated || !c->config.direction || !fresh(c, now)) {
        Feeder_Stop(c); if (event) { ++c->rejected; } return 0;
    }
    if (c->busy && c->have_control_tick) {
        tick_gap = now - c->last_control_ms;
        if (tick_gap > FEEDER_CONTROL_MAX_GAP_MS) {
            fault(c, FEEDER_FAULT_TIMING); if (event) { ++c->rejected; } return 0;
        }
    }
    c->last_control_ms = now; c->have_control_tick = 1U;
    if (event) {
        if (c->busy) { ++c->rejected; }
        else {
            c->busy = 1U; ++c->accepted; c->motion_started_ms = now;
            c->target_deg += (double)c->config.step_deg * (double)c->config.gear_ratio *
                (double)c->config.direction;
            c->axis_target_deg = c->target_deg / (double)c->config.gear_ratio;
            c->integral = 0.0f; c->settling = c->jamming = 0U;
            c->last_control_sequence = c->feedback_sequence - 1U;
            c->last_pi_feedback_ms = c->last_feedback_ms;
        }
    }
    if (!c->busy) { c->command_raw = 0; return 0; }
    if (now - c->motion_started_ms >= c->config.motion_timeout_ms) {
        fault(c, FEEDER_FAULT_TIMEOUT); return 0;
    }
    error = (float)(c->target_deg - c->position_deg);
    new_feedback = (uint8_t)(c->last_control_sequence != c->feedback_sequence);
    if (new_feedback) {
        c->last_control_sequence = c->feedback_sequence;
        if (abs_double(c->target_deg - c->position_deg) <=
            (double)c->config.tolerance_deg * (double)c->config.gear_ratio &&
            absolute((float)c->raw_rpm) <= c->config.settle_rpm) {
            if (!c->settling) { c->settling = 1U; c->settle_started_ms = c->last_feedback_ms; }
            else if (c->last_feedback_ms - c->settle_started_ms >= c->config.settle_ms) {
                c->busy = 0U; ++c->completed; reset_output(c); return 0;
            }
        } else { c->settling = 0U; }
        c->target_rpm = clamp(error * c->config.pos_kp, -c->config.max_rpm, c->config.max_rpm);
        speed_error = c->target_rpm - (float)c->raw_rpm;
        feedback_gap = c->last_feedback_ms - c->last_pi_feedback_ms;
        c->last_pi_feedback_ms = c->last_feedback_ms;
        dt = (float)feedback_gap * 0.001f;
        p = c->config.speed_kp * speed_error;
        candidate = clamp(c->integral + c->config.speed_ki * speed_error * dt,
            -c->config.current_limit, c->config.current_limit);
        output = p + candidate;
        if ((output <= c->config.current_limit && output >= -c->config.current_limit) ||
            (output > c->config.current_limit && speed_error < 0.0f) ||
            (output < -c->config.current_limit && speed_error > 0.0f)) { c->integral = candidate; }
        output = clamp(p + c->integral, -c->config.current_limit, c->config.current_limit);
        c->command_raw = (int16_t)output;
        if (!c->settling && absolute((float)c->raw_rpm) <= c->config.jam_rpm &&
            (absolute(output) >= c->config.jam_current_raw ||
             absolute((float)c->raw_current) >= c->config.jam_current_raw)) {
            if (!c->jamming) { c->jamming = 1U; c->jam_started_ms = c->last_feedback_ms; }
            else if (c->last_feedback_ms - c->jam_started_ms >= c->config.jam_ms) {
                fault(c, FEEDER_FAULT_JAM); return 0;
            }
        } else { c->jamming = 0U; }
    }
    return c->command_raw;
}
