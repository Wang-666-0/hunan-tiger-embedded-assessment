#include "pitch_controller.h"
#include <float.h>
#include <math.h>
#include <string.h>

#if YAW_CASCADE_OUTPUT_LIMITS_ENABLED
#error Pitch_requires_protocol_only_cascade_output_limits
#endif
#define RAD_TO_DEG (180.0f / DM3507_POSITION_MAX_RAD)
#define HALF_TICK_RANGE 0x80000000UL
#define RECOVERABLE_FAULTS (PITCH_CONTROLLER_FAULT_FEEDBACK | PITCH_CONTROLLER_FAULT_DATA | \
    PITCH_CONTROLLER_FAULT_STATUS | PITCH_CONTROLLER_FAULT_TEMPERATURE | \
    PITCH_CONTROLLER_FAULT_JUMP | PITCH_CONTROLLER_FAULT_LIMIT | PITCH_CONTROLLER_FAULT_TIMING)

static uint8_t finite_value(float x) { return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
static float absolute(float x) { return x < 0.0f ? -x : x; }
static float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float normalize_degrees(float x)
{ x = fmodf(x, 360.0f); if (x > 180.0f) { x -= 360.0f; } else if (x < -180.0f) { x += 360.0f; } return x; }
static float gravity_at(const PitchController *pitch, float kg)
{ return kg * cosf((pitch->angle_deg + pitch->gravity_offset_deg) / RAD_TO_DEG); }
static void reset_shape(PitchController *pitch)
{
    pitch->shape.active = 0U;
    pitch->shape.position_deg = pitch->target_deg;
    pitch->shape.velocity_dps = pitch->shape.acceleration_dps2 = 0.0f;
}
static void update_shape(PitchController *pitch, uint32_t now_ms)
{
    uint32_t elapsed;
    float u, u2, u3, span, seconds, remaining;
    if (!pitch->shape.active) { reset_shape(pitch); return; }
    elapsed = now_ms - pitch->shape.start_ms;
    if (elapsed >= pitch->shape.duration_ms) { reset_shape(pitch); return; }
    seconds = (float)pitch->shape.duration_ms * 0.001f;
    u = (float)elapsed / (float)pitch->shape.duration_ms;
    u2 = u * u; u3 = u2 * u; remaining = 1.0f - u;
    span = pitch->target_deg - pitch->shape.start_deg;
    pitch->shape.position_deg = pitch->shape.start_deg + span * u3 * (10.0f + u * (-15.0f + 6.0f * u));
    pitch->shape.velocity_dps = span * 30.0f * u2 * remaining * remaining / seconds;
    pitch->shape.acceleration_dps2 = span * 60.0f * u * remaining * (1.0f - 2.0f * u) / (seconds * seconds);
}
void PitchController_InvalidateGravity(PitchController *pitch)
{
    if (!pitch) { return; }
    /* 坐标失效只取消物理相位，不丢用户的力矩系数。 */
    pitch->gravity_offset_deg = pitch->gravity_torque_nm = 0.0f;
    pitch->pid_torque_nm = 0.0f; pitch->gravity_calibrated = pitch->gravity_i_transferred = 0U;
}
static void fault(PitchController *pitch, uint32_t reasons)
{ pitch->faults |= reasons; PitchController_Stop(pitch); }
static uint8_t stationary_safe(const PitchController *pitch)
{
    return (uint8_t)(pitch->have_feedback && pitch->last_status <= 1U &&
        pitch->temp_mos < PITCH_CONTROLLER_DRIVER_LIMIT_C &&
        pitch->temp_rotor < PITCH_CONTROLLER_MOTOR_LIMIT_C &&
        absolute(pitch->velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS);
}
void PitchController_Init(PitchController *pitch)
{
    if (!pitch) { return; }
    memset(pitch, 0, sizeof(*pitch)); pitch->feedback_divisor = 1U;
    pitch->shape.duration_ms = APP_PITCH_SHAPE_MS;
    pitch->shape.velocity_gain = APP_PITCH_SHAPE_VELOCITY_GAIN;
    YawCascade_Init(&pitch->cascade);
    /* 用户确认写入的上电默认；不改变Yaw默认增益。 */
    pitch->cascade.position.kp = APP_PITCH_POSITION_KP;
    pitch->cascade.position.ki = APP_PITCH_POSITION_KI;
    pitch->cascade.position.kd = APP_PITCH_POSITION_KD;
    pitch->cascade.speed.kp = APP_PITCH_SPEED_KP;
    pitch->cascade.speed.ki = APP_PITCH_SPEED_KI;
    pitch->cascade.speed.kd = APP_PITCH_SPEED_KD;
    pitch->gravity_kg_nm = APP_PITCH_GRAVITY_KG_NM;
    pitch->cascade.velocity_limit_dps = 0.0f;
}
void PitchController_Stop(PitchController *pitch)
{
    if (!pitch) { return; }
    pitch->active = pitch->have_control_tick = 0U; pitch->torque_nm = 0.0f;
    pitch->gravity_torque_nm = pitch->pid_torque_nm = 0.0f;
    pitch->gravity_i_transferred = 0U;
    YawCascade_Stop(&pitch->cascade);
    pitch->target_deg = finite_value(pitch->angle_deg) ?
        clamp(pitch->angle_deg, PITCH_CONTROLLER_TARGET_MIN_DEG, PITCH_CONTROLLER_TARGET_MAX_DEG) : 0.0f;
    reset_shape(pitch);
}
uint8_t PitchController_OnFeedback(PitchController *pitch, const Dm3507Feedback *feedback)
{
    float delta = 0.0f, divisor;
    uint32_t gap = 0U;
    uint8_t first, rebase = 0U, was_active;
    if (!pitch || !feedback) { return 0U; }
    if (!finite_value(feedback->position_rad) || !finite_value(feedback->velocity_rad_s) ||
        !finite_value(feedback->torque_nm) || feedback->motor_id != DM3507_MOTOR_ID ||
        absolute(feedback->position_rad) > DM3507_POSITION_MAX_RAD + 0.00001f ||
        absolute(feedback->velocity_rad_s) > DM3507_VELOCITY_MAX_RAD_S + 0.00001f ||
        absolute(feedback->torque_nm) > DM3507_TORQUE_MAX_NM + 0.00001f) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); pitch->stationary_samples = 0U; return 0U;
    }
    divisor = (float)pitch->feedback_divisor;
    if (divisor != 1.0f && divisor != 7.0f) { fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0U; }
    first = (uint8_t)!pitch->have_feedback; was_active = pitch->active;
    if (!first) {
        gap = feedback->received_ms - pitch->last_feedback_ms;
        if (!gap || gap >= HALF_TICK_RANGE) { return 0U; }
        delta = feedback->position_rad - pitch->last_position_rad;
        if (delta > DM3507_POSITION_MAX_RAD) { delta -= 2.0f * DM3507_POSITION_MAX_RAD; }
        else if (delta < -DM3507_POSITION_MAX_RAD) { delta += 2.0f * DM3507_POSITION_MAX_RAD; }
        if (gap >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
            rebase = 1U; fault(pitch, PITCH_CONTROLLER_FAULT_FEEDBACK);
        }
    }
    pitch->last_position_rad = feedback->position_rad; pitch->last_feedback_ms = feedback->received_ms;
    ++pitch->last_sequence; pitch->have_feedback = 1U;
    pitch->velocity_dps = feedback->velocity_rad_s * RAD_TO_DEG / divisor;
    pitch->last_status = feedback->status; pitch->temp_mos = feedback->temp_mos;
    pitch->temp_rotor = feedback->temp_rotor;
    if (first || rebase) {
        PitchController_InvalidateGravity(pitch);
        pitch->continuous_rad = pitch->reference_rad = feedback->position_rad;
        pitch->angle_deg = pitch->target_deg = 0.0f;
        reset_shape(pitch);
        pitch->have_reference = 0U; pitch->stationary_samples = 0U;
        YawCascade_ResetObserver(&pitch->cascade);
    } else {
        pitch->continuous_rad += delta;
        pitch->angle_deg = (pitch->continuous_rad - pitch->reference_rad) * RAD_TO_DEG / divisor;
        YawCascade_Observe(&pitch->cascade, delta * RAD_TO_DEG / divisor, gap);
    }
    if (stationary_safe(pitch) && (first || rebase ||
        absolute(delta) * RAD_TO_DEG / divisor <= 0.05f + 0.5f * (float)gap * 0.001f)) {
        if (!pitch->stationary_samples) { pitch->stationary_started_ms = feedback->received_ms; }
        if (pitch->stationary_samples < 255U) { ++pitch->stationary_samples; }
    } else { pitch->stationary_samples = 0U; }
    if (feedback->fault_epoch) { fault(pitch, PITCH_CONTROLLER_FAULT_CAN); }
    if (feedback->status > 1U || (was_active && feedback->status != 1U)) {
        fault(pitch, PITCH_CONTROLLER_FAULT_STATUS);
    }
    if (feedback->temp_mos >= PITCH_CONTROLLER_DRIVER_LIMIT_C ||
        feedback->temp_rotor >= PITCH_CONTROLLER_MOTOR_LIMIT_C) {
        fault(pitch, PITCH_CONTROLLER_FAULT_TEMPERATURE);
    }
    /* 未捕获软件零点时允许观察绝对反馈，不猜水平或套GL的±60。 */
    if (pitch->have_reference && (pitch->angle_deg < PITCH_CONTROLLER_HARD_MIN_DEG ||
        pitch->angle_deg > PITCH_CONTROLLER_HARD_MAX_DEG)) {
        fault(pitch, PITCH_CONTROLLER_FAULT_LIMIT);
    }
    return 1U;
}
uint8_t PitchController_CaptureReference(PitchController *pitch)
{
    if (!pitch || pitch->active || pitch->last_status != 0U || !stationary_safe(pitch) || pitch->stationary_samples < 2U ||
        pitch->last_feedback_ms - pitch->stationary_started_ms < PITCH_CONTROLLER_STATIONARY_HOLD_MS ||
        (pitch->faults & ~(uint32_t)RECOVERABLE_FAULTS)) { return 0U; }
    /* 改软件零时保持已标定的物理水平，不把新零误认为水平。 */
    if (pitch->have_reference && pitch->gravity_calibrated) {
        pitch->gravity_offset_deg = normalize_degrees(pitch->gravity_offset_deg + pitch->angle_deg);
    } else { PitchController_InvalidateGravity(pitch); }
    PitchController_Stop(pitch); pitch->reference_rad = pitch->continuous_rad;
    pitch->angle_deg = pitch->target_deg = 0.0f; pitch->have_reference = 1U;
    reset_shape(pitch);
    YawCascade_ResetObserver(&pitch->cascade); pitch->faults &= ~(uint32_t)RECOVERABLE_FAULTS;
    return 1U;
}
uint8_t PitchController_SetFeedbackDivisor(PitchController *pitch, uint8_t divisor)
{
    if (!pitch || (divisor != 1U && divisor != 7U) || pitch->active ||
        (pitch->have_feedback && pitch->last_status != 0U)) { return 0U; }
    if (pitch->feedback_divisor == divisor) { return 1U; }
    PitchController_Stop(pitch); pitch->feedback_divisor = divisor; pitch->have_reference = 0U;
    PitchController_InvalidateGravity(pitch);
    pitch->reference_rad = pitch->continuous_rad; pitch->angle_deg = pitch->target_deg = 0.0f;
    reset_shape(pitch);
    pitch->velocity_dps = pitch->velocity_dps * (divisor == 7U ? 1.0f / 7.0f : 7.0f);
    pitch->stationary_samples = 0U; YawCascade_ResetObserver(&pitch->cascade);
    return 1U;
}
uint8_t PitchController_SetPid(PitchController *pitch, char which, float kp, float ki, float kd)
{
    YawCascadePid *pid;
    if (!pitch || !finite_value(kp) || !finite_value(ki) || !finite_value(kd) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f) { return 0U; }
    if (which == 'P') {
        if (kp > 100.0f || ki > 30.0f || kd > 10.0f) { return 0U; }
        pid = &pitch->cascade.position;
    } else if (which == 'V') {
        if (kp > 1.0f || ki > 10.0f || kd > 0.02f) { return 0U; }
        pid = &pitch->cascade.speed;
    } else { return 0U; }
    if (pid->kp == kp && pid->ki == ki && pid->kd == kd) { return 1U; }
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    YawCascade_Stop(&pitch->cascade); ++pitch->cascade.revision;
    return 1U;
}
uint8_t PitchController_SetTarget(PitchController *pitch, float degrees)
{
    if (!pitch || !finite_value(degrees) || degrees < PITCH_CONTROLLER_TARGET_MIN_DEG ||
        degrees > PITCH_CONTROLLER_TARGET_MAX_DEG || !pitch->have_reference || pitch->faults) { return 0U; }
    pitch->target_deg = degrees; reset_shape(pitch);
    return 1U;
}
uint8_t PitchController_SetShape(PitchController *pitch, uint32_t duration_ms, float velocity_gain)
{
    if (!pitch || (duration_ms != 0U && (duration_ms < 100U || duration_ms > 250U)) ||
        !finite_value(velocity_gain) || velocity_gain < 0.0f || velocity_gain > 1.0f) { return 0U; }
    if (pitch->shape.duration_ms == duration_ms && pitch->shape.velocity_gain == velocity_gain) { return 1U; }
    if (pitch->shape.active) { return 0U; }
    pitch->shape.duration_ms = duration_ms; pitch->shape.velocity_gain = velocity_gain;
    return 1U;
}
uint8_t PitchController_StartMove(PitchController *pitch, float degrees, uint32_t now_ms)
{
    if (!pitch || !finite_value(degrees) || degrees < PITCH_CONTROLLER_TARGET_MIN_DEG ||
        degrees > PITCH_CONTROLLER_TARGET_MAX_DEG || !pitch->active || !pitch->have_reference ||
        !pitch->have_feedback || pitch->faults || pitch->last_status != 1U ||
        now_ms - pitch->last_feedback_ms >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) { return 0U; }
    if (pitch->shape.active) { return (uint8_t)(degrees == pitch->target_deg); }
    if (pitch->shape.duration_ms && absolute(degrees - pitch->angle_deg) >= 1.0f) {
        if (absolute(pitch->velocity_dps) > PITCH_CONTROLLER_STATIONARY_SPEED_DPS ||
            absolute(pitch->cascade.velocity_dps) > PITCH_CONTROLLER_STATIONARY_SPEED_DPS) { return 0U; }
        pitch->target_deg = degrees; reset_shape(pitch);
        pitch->shape.start_ms = now_ms; pitch->shape.start_deg = pitch->angle_deg;
        pitch->shape.position_deg = pitch->angle_deg; pitch->shape.active = 1U;
        return 1U;
    }
    return PitchController_SetTarget(pitch, degrees);
}
uint8_t PitchController_SetGravityOffset(PitchController *pitch, float offset_deg)
{
    if (!pitch || !finite_value(offset_deg) || absolute(offset_deg) > 180.0f || pitch->active ||
        !pitch->have_reference || !pitch->have_feedback || pitch->last_status != 0U || pitch->faults) { return 0U; }
    pitch->gravity_offset_deg = offset_deg; pitch->gravity_calibrated = 1U;
    pitch->gravity_i_transferred = 0U; return 1U;
}
uint8_t PitchController_CaptureHorizontal(PitchController *pitch)
{
    if (!pitch || !stationary_safe(pitch) || pitch->stationary_samples < 2U ||
        pitch->last_feedback_ms - pitch->stationary_started_ms < PITCH_CONTROLLER_STATIONARY_HOLD_MS) { return 0U; }
    return PitchController_SetGravityOffset(pitch, -pitch->angle_deg);
}
uint8_t PitchController_SetGravityGain(PitchController *pitch, float kg_nm)
{
    float delta, previous_i, next_i;
    if (!pitch || !finite_value(kg_nm) || absolute(kg_nm) > PITCH_CONTROLLER_TORQUE_LIMIT_NM ||
        (kg_nm != 0.0f && (!pitch->have_reference || !pitch->gravity_calibrated || pitch->faults ||
         !finite_value(pitch->gravity_offset_deg) || !finite_value(pitch->angle_deg)))) { return 0U; }
    if (pitch->gravity_kg_nm == kg_nm) { return 1U; }
    pitch->gravity_i_transferred = 0U;
    if (pitch->active && pitch->cascade.speed.ki > 0.0f) {
        delta = gravity_at(pitch, pitch->gravity_kg_nm) - gravity_at(pitch, kg_nm);
        previous_i = pitch->cascade.speed_i;
        next_i = clamp(previous_i + delta, -PITCH_CONTROLLER_TORQUE_LIMIT_NM, PITCH_CONTROLLER_TORQUE_LIMIT_NM);
        pitch->cascade.speed_i = next_i;
        pitch->gravity_i_transferred = (uint8_t)(absolute((next_i - previous_i) - delta) < 0.00001f);
    }
    pitch->gravity_kg_nm = kg_nm;
    return 1U;
}
float PitchController_Update(PitchController *pitch, float rate_dps, uint32_t now_ms, uint8_t allowed)
{
    uint32_t age, elapsed;
    float torque, velocity_ff;
    if (!pitch) { return 0.0f; }
    if (!finite_value(rate_dps) || !finite_value(pitch->gravity_kg_nm) ||
        !finite_value(pitch->gravity_offset_deg) || absolute(pitch->gravity_kg_nm) > PITCH_CONTROLLER_TORQUE_LIMIT_NM ||
        absolute(pitch->gravity_offset_deg) > 180.0f) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0.0f;
    }
    age = now_ms - pitch->last_feedback_ms;
    if (!pitch->have_feedback || age >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
        if (allowed || pitch->active) { fault(pitch, PITCH_CONTROLLER_FAULT_FEEDBACK); }
        else { PitchController_Stop(pitch); }
        return 0.0f;
    }
    if (!allowed || !pitch->have_reference || pitch->faults ||
        (pitch->gravity_kg_nm != 0.0f && !pitch->gravity_calibrated)) {
        PitchController_Stop(pitch); return 0.0f;
    }
    if (pitch->last_status != 1U) { fault(pitch, PITCH_CONTROLLER_FAULT_STATUS); return 0.0f; }
    if (pitch->angle_deg < PITCH_CONTROLLER_HARD_MIN_DEG || pitch->angle_deg > PITCH_CONTROLLER_HARD_MAX_DEG) {
        fault(pitch, PITCH_CONTROLLER_FAULT_LIMIT); return 0.0f;
    }
    elapsed = pitch->have_control_tick ? now_ms - pitch->last_control_ms : 0U;
    pitch->last_control_ms = now_ms; pitch->have_control_tick = 1U;
    if (elapsed > PITCH_CONTROLLER_CONTROL_MAX_GAP_MS) { fault(pitch, PITCH_CONTROLLER_FAULT_TIMING); return 0.0f; }
    pitch->active = 1U;
    rate_dps = clamp(rate_dps, -APP_PITCH_RATE_LIMIT_DPS, APP_PITCH_RATE_LIMIT_DPS);
    if (rate_dps != 0.0f && pitch->shape.active) {
        /* 手动接管从实测位置继续，避免跳到尚未到达的最终目标。 */
        pitch->target_deg = clamp(pitch->angle_deg, PITCH_CONTROLLER_TARGET_MIN_DEG, PITCH_CONTROLLER_TARGET_MAX_DEG);
        reset_shape(pitch);
    }
    pitch->target_deg = clamp(pitch->target_deg + rate_dps * (float)elapsed * 0.001f,
        PITCH_CONTROLLER_TARGET_MIN_DEG, PITCH_CONTROLLER_TARGET_MAX_DEG);
    update_shape(pitch, now_ms);
    velocity_ff = pitch->shape.velocity_gain * pitch->shape.velocity_dps;
    pitch->gravity_torque_nm = gravity_at(pitch, pitch->gravity_kg_nm);
    torque = YawCascade_UpdateReference(&pitch->cascade, pitch->shape.position_deg - pitch->angle_deg,
        pitch->last_feedback_ms, pitch->last_sequence, PITCH_CONTROLLER_TORQUE_LIMIT_NM,
        pitch->gravity_torque_nm, velocity_ff, pitch->shape.active);
    pitch->pid_torque_nm = pitch->cascade.unsaturated_torque_nm - pitch->gravity_torque_nm;
    if (!finite_value(torque) || !finite_value(pitch->cascade.position_i) ||
        !finite_value(pitch->cascade.speed_i) || !finite_value(pitch->cascade.target_velocity_dps) ||
        !finite_value(pitch->cascade.unsaturated_torque_nm)) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0.0f;
    }
    pitch->torque_nm = torque; return torque;
}
