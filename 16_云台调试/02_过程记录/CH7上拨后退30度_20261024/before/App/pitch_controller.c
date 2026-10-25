#include "pitch_controller.h"
#include <float.h>
#include <math.h>
#include <string.h>

#if YAW_CASCADE_OUTPUT_LIMITS_ENABLED
#error Pitch_requires_protocol_only_cascade_output_limits
#endif
#define RAD_TO_DEG (180.0f / DM3507_POSITION_MAX_RAD)
#define HALF_TICK_RANGE 0x80000000UL
#define LANDING_TRACE_WINDOW_MS 600U
#define RECOVERABLE_FAULTS (PITCH_CONTROLLER_FAULT_FEEDBACK | PITCH_CONTROLLER_FAULT_DATA | \
    PITCH_CONTROLLER_FAULT_STATUS | PITCH_CONTROLLER_FAULT_TEMPERATURE | \
    PITCH_CONTROLLER_FAULT_JUMP | PITCH_CONTROLLER_FAULT_LIMIT | PITCH_CONTROLLER_FAULT_TIMING)

static uint8_t finite_value(float x) { return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
static float absolute(float x) { return x < 0.0f ? -x : x; }
static float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float normalize_degrees(float x)
{ x = fmodf(x, 360.0f); if (x > 180.0f) { x -= 360.0f; } else if (x < -180.0f) { x += 360.0f; } return x; }
static void observe_pitch(PitchController *pitch, float delta_deg, uint32_t gap_ms)
{
    YawCascade *loop = &pitch->cascade;
    float previous, dt, alpha;
    if (!gap_ms || gap_ms >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS || !finite_value(delta_deg)) {
        YawCascade_ResetObserver(loop); return;
    }
    dt = (float)gap_ms * 0.001f;
    previous = loop->velocity_dps;
    alpha = (float)gap_ms / ((float)APP_PITCH_VELOCITY_FILTER_MS + (float)gap_ms);
    loop->velocity_dps += alpha * (delta_deg / dt - loop->velocity_dps);
    /* 保留测量微分的平滑，短速度滤波不等于直接放大速度D噪声。 */
    alpha = (float)gap_ms / ((float)APP_PITCH_ACCEL_FILTER_MS + (float)gap_ms);
    loop->acceleration_dps2 += alpha * ((loop->velocity_dps - previous) / dt - loop->acceleration_dps2);
}
static float gravity_at(const PitchController *pitch, float kg)
{ return kg * cosf((pitch->angle_deg + pitch->gravity_offset_deg) / RAD_TO_DEG); }
static void reset_shape(PitchController *pitch)
{
    pitch->shape.active = 0U;
    pitch->shape.position_deg = pitch->target_deg;
    pitch->shape.velocity_dps = pitch->shape.acceleration_dps2 = 0.0f;
}
static void reset_brake(PitchController *pitch)
{
    pitch->brake.torque_nm = pitch->brake.safe_velocity_dps = 0.0f;
    pitch->brake.tracking = pitch->brake.arrived = pitch->brake.active = pitch->brake.approach_done = 0U;
    pitch->brake.sequence = pitch->last_sequence;
    pitch->brake.feedback_ms = pitch->last_feedback_ms;
    pitch->brake.previous_angle_deg = pitch->angle_deg;
}
static void clear_brake_statistics(PitchController *pitch)
{
    pitch->brake.peak_abs_nm = pitch->brake.return_peak_nm = 0.0f;
    pitch->brake.active_frames = pitch->brake.return_frames = 0U;
}
static void start_landing_trace(PitchController *pitch, float span, uint32_t now_ms)
{
    PitchLandingTrace *trace = &pitch->landing;
    memset(trace, 0, sizeof(*trace));
    trace->active = (uint8_t)(absolute(span) >= 1.0f);
    trace->direction = span > 0.0f ? 1.0f : -1.0f;
    trace->target_deg = pitch->target_deg; trace->start_ms = now_ms;
    trace->sequence = pitch->last_sequence;
    trace->start_i_nm = trace->min_i_nm = trace->max_i_nm = pitch->cascade.speed_i;
}
static void sample_landing_trace(PitchController *pitch)
{
    PitchLandingTrace *trace = &pitch->landing;
    uint32_t elapsed;
    float remaining;
    if (!trace->active) { return; }
    if (pitch->target_deg != trace->target_deg) { trace->active = 0U; return; }
    if (pitch->last_sequence == trace->sequence) { return; }
    trace->sequence = pitch->last_sequence;
    elapsed = pitch->last_feedback_ms - trace->start_ms;
    /* 忽略命令接受前已入队的旧反馈，32位毫秒回绕正常处理。 */
    if (elapsed >= HALF_TICK_RANGE) { return; }
    if (elapsed > LANDING_TRACE_WINDOW_MS) { trace->active = 0U; return; }
    remaining = trace->direction * (trace->target_deg - pitch->angle_deg);
    if (!trace->arrived && remaining <= 0.2f) {
        trace->arrived = 1U; trace->arrival_ms = pitch->last_feedback_ms;
        trace->min_i_nm = trace->max_i_nm = pitch->cascade.speed_i;
    }
    if (!trace->arrived) { return; }
    if (pitch->cascade.speed_i < trace->min_i_nm) { trace->min_i_nm = pitch->cascade.speed_i; }
    if (pitch->cascade.speed_i > trace->max_i_nm) { trace->max_i_nm = pitch->cascade.speed_i; }
    if (!trace->landed && trace->direction * pitch->velocity_dps <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS) {
        trace->landed = 1U; trace->land_ms = elapsed;
        trace->land_i_nm = pitch->cascade.speed_i;
        trace->land_raw_dps = pitch->velocity_dps;
        trace->land_filtered_dps = pitch->cascade.velocity_dps;
        trace->land_target_dps = pitch->cascade.target_velocity_dps;
        trace->land_gravity_nm = pitch->gravity_torque_nm;
        trace->land_brake_nm = pitch->brake.torque_nm;
        trace->land_torque_nm = pitch->torque_nm;
    }
    if (remaining > trace->dip_deg) { trace->dip_deg = remaining; trace->dip_ms = elapsed; }
}
static float terminal_brake(PitchController *pitch, uint32_t age)
{
    PitchTerminalBrake *brake = &pitch->brake;
    float remaining, closing, raw_closing, delta, weight, wanted = 0.0f, rise, previous;
    float output_direction, return_speed;
    uint32_t gap;
    uint8_t returning = 0U;
    if (!brake->enabled || !brake->tracking || pitch->shape.active ||
        pitch->target_deg != brake->target_deg) { reset_brake(pitch); return 0.0f; }
    if (age > APP_PITCH_TERMINAL_BRAKE_MAX_AGE_MS) {
        brake->torque_nm = 0.0f; brake->active = 0U; return 0.0f;
    }
    if (brake->sequence == pitch->last_sequence) { return brake->torque_nm; }
    gap = pitch->last_feedback_ms - brake->feedback_ms;
    delta = brake->direction * (pitch->angle_deg - brake->previous_angle_deg);
    brake->sequence = pitch->last_sequence; brake->feedback_ms = pitch->last_feedback_ms;
    brake->previous_angle_deg = pitch->angle_deg;
    remaining = brake->direction * (pitch->target_deg - pitch->angle_deg);
    closing = brake->direction * pitch->cascade.velocity_dps;
    raw_closing = brake->direction * pitch->velocity_dps;
    if (!brake->arrived && remaining <= 0.2f) {
        brake->arrived = 1U; brake->arrival_ms = pitch->last_feedback_ms;
    }
    if (brake->arrived) {
        if (pitch->last_feedback_ms - brake->arrival_ms > APP_PITCH_CAPTURE_BRAKE_WINDOW_MS) {
            reset_brake(pitch); return 0.0f;
        }
        if (closing <= APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS ||
            raw_closing <= APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS || delta < -0.0001f) {
            brake->approach_done = 1U;
        }
    }
    output_direction = -brake->direction;
    if (brake->approach_done && remaining > APP_PITCH_CAPTURE_BRAKE_ERROR_DEG &&
        remaining < APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG && delta < -0.0001f &&
        closing < -APP_PITCH_CAPTURE_BRAKE_FLOOR_DPS &&
        raw_closing < -APP_PITCH_CAPTURE_BRAKE_FLOOR_DPS) {
        /* 只抵抗实际远离目标的回退；两路速度与位置增量必须一致。 */
        return_speed = -closing < -raw_closing ? -closing : -raw_closing;
        weight = clamp(APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG - remaining, 0.0f, 1.0f);
        weight = weight * weight * (3.0f - 2.0f * weight);
        wanted = clamp(APP_PITCH_CAPTURE_BRAKE_GAIN *
            (return_speed - APP_PITCH_CAPTURE_BRAKE_FLOOR_DPS) * weight,
            0.0f, APP_PITCH_CAPTURE_BRAKE_LIMIT_NM);
        output_direction = brake->direction; returning = 1U;
    }
    /* 两路速度均指向目标才加阻尼；不沿用滤波滞后在静止/反向时继续加力。 */
    if (raw_closing < closing) { closing = raw_closing; }
    brake->safe_velocity_dps = sqrtf(2.0f * APP_PITCH_TERMINAL_BRAKE_DECEL_DPS2 *
        (remaining > 0.0f ? remaining : 0.0f));
    if (brake->safe_velocity_dps < APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS) {
        brake->safe_velocity_dps = APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS;
    }
    if (!brake->approach_done && absolute(remaining) < APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG &&
        delta >= -0.0001f && closing > brake->safe_velocity_dps) {
        weight = 1.0f - absolute(remaining) / APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG;
        weight = weight * weight * (3.0f - 2.0f * weight);
        wanted = clamp(APP_PITCH_TERMINAL_BRAKE_GAIN * (closing - brake->safe_velocity_dps) * weight,
            0.0f, APP_PITCH_TERMINAL_BRAKE_LIMIT_NM);
    }
    if (wanted <= 0.0f) {
        brake->torque_nm = 0.0f; brake->active = 0U; return 0.0f;
    }
    rise = (returning ? APP_PITCH_CAPTURE_BRAKE_RISE_NM_S : APP_PITCH_TERMINAL_BRAKE_RISE_NM_S) *
        (float)(gap > 10U ? 10U : gap) * 0.001f;
    /* 换向时先撤掉旧方向力矩，再从0缓升，避免跨零突跳。 */
    previous = brake->torque_nm * output_direction > 0.0f ? absolute(brake->torque_nm) : 0.0f;
    if (wanted > previous + rise) { wanted = previous + rise; }
    brake->torque_nm = output_direction * wanted;
    brake->active = (uint8_t)(wanted > 0.0f);
    if (wanted > brake->peak_abs_nm) { brake->peak_abs_nm = wanted; }
    ++brake->active_frames;
    if (returning) {
        if (wanted > brake->return_peak_nm) { brake->return_peak_nm = wanted; }
        ++brake->return_frames;
    }
    return brake->torque_nm;
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
    pitch->brake.enabled = APP_PITCH_TERMINAL_BRAKE_ENABLED;
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
    pitch->landing.active = 0U;
    pitch->gravity_torque_nm = pitch->pid_torque_nm = 0.0f;
    pitch->gravity_i_transferred = 0U;
    YawCascade_Stop(&pitch->cascade);
    pitch->target_deg = finite_value(pitch->angle_deg) ?
        clamp(pitch->angle_deg, PITCH_CONTROLLER_TARGET_MIN_DEG, PITCH_CONTROLLER_TARGET_MAX_DEG) : 0.0f;
    reset_shape(pitch);
    reset_brake(pitch);
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
        observe_pitch(pitch, delta * RAD_TO_DEG / divisor, gap);
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
    pitch->landing.active = 0U;
    reset_brake(pitch);
    YawCascade_Stop(&pitch->cascade); ++pitch->cascade.revision;
    return 1U;
}
uint8_t PitchController_SetTarget(PitchController *pitch, float degrees)
{
    if (!pitch || !finite_value(degrees) || degrees < PITCH_CONTROLLER_TARGET_MIN_DEG ||
        degrees > PITCH_CONTROLLER_TARGET_MAX_DEG || !pitch->have_reference || pitch->faults) { return 0U; }
    pitch->target_deg = degrees; reset_shape(pitch);
    reset_brake(pitch);
    clear_brake_statistics(pitch);
    memset(&pitch->landing, 0, sizeof(pitch->landing));
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
    float span;
    if (!pitch || !finite_value(degrees) || degrees < PITCH_CONTROLLER_TARGET_MIN_DEG ||
        degrees > PITCH_CONTROLLER_TARGET_MAX_DEG || !pitch->active || !pitch->have_reference ||
        !pitch->have_feedback || pitch->faults || pitch->last_status != 1U ||
        now_ms - pitch->last_feedback_ms >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) { return 0U; }
    if (degrees == pitch->target_deg) { return 1U; }
    if (pitch->shape.active) { return 0U; }
    if (pitch->shape.duration_ms && absolute(degrees - pitch->angle_deg) >= 1.0f) {
        if (absolute(pitch->velocity_dps) > PITCH_CONTROLLER_STATIONARY_SPEED_DPS ||
            absolute(pitch->cascade.velocity_dps) > PITCH_CONTROLLER_STATIONARY_SPEED_DPS) { return 0U; }
        pitch->target_deg = degrees; reset_shape(pitch); reset_brake(pitch); clear_brake_statistics(pitch);
        pitch->shape.start_ms = now_ms; pitch->shape.start_deg = pitch->angle_deg;
        pitch->shape.position_deg = pitch->angle_deg; pitch->shape.active = 1U;
        start_landing_trace(pitch, degrees - pitch->angle_deg, now_ms);
        return 1U;
    }
    span = degrees - pitch->angle_deg;
    if (!PitchController_SetTarget(pitch, degrees)) { return 0U; }
    start_landing_trace(pitch, span, now_ms);
    if (!pitch->shape.duration_ms && pitch->brake.enabled && absolute(span) >= 1.0f) {
        pitch->brake.direction = span > 0.0f ? 1.0f : -1.0f;
        pitch->brake.target_deg = degrees; pitch->brake.tracking = 1U;
    }
    return 1U;
}
uint8_t PitchController_SetBrakeEnabled(PitchController *pitch, uint8_t enabled)
{
    if (!pitch || enabled > 1U) { return 0U; }
    if (pitch->brake.enabled != enabled) { reset_brake(pitch); pitch->brake.enabled = enabled; }
    return 1U;
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
    pitch->landing.active = 0U;
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
    float torque, velocity_ff, feedforward;
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
    if (rate_dps != 0.0f) { reset_brake(pitch); pitch->landing.active = 0U; }
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
    feedforward = pitch->gravity_torque_nm + terminal_brake(pitch, age);
    torque = YawCascade_UpdateReference(&pitch->cascade, pitch->shape.position_deg - pitch->angle_deg,
        pitch->last_feedback_ms, pitch->last_sequence, PITCH_CONTROLLER_TORQUE_LIMIT_NM,
        feedforward, velocity_ff, pitch->shape.active);
    pitch->pid_torque_nm = pitch->cascade.unsaturated_torque_nm - feedforward;
    if (!finite_value(torque) || !finite_value(pitch->cascade.position_i) ||
        !finite_value(pitch->cascade.speed_i) || !finite_value(pitch->cascade.target_velocity_dps) ||
        !finite_value(pitch->cascade.unsaturated_torque_nm)) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0.0f;
    }
    pitch->torque_nm = torque;
    sample_landing_trace(pitch);
    return torque;
}

float PitchController_UpdateViewHold(PitchController *pitch, uint32_t now_ms, uint8_t allowed)
{
    return PitchController_Update(pitch, 0.0f, now_ms, allowed);
}
