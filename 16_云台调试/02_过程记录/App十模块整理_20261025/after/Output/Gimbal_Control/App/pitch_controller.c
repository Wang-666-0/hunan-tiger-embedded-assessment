/**
 * @file pitch_controller.c
 * @brief Pitch 轴控制与执行
 *
 * 职责：实现 Pitch 串级 PID 接入、重力补偿、末段制动、阶跃实测及会话后端。
 * 主要函数：
 *   - PitchController_Update：叠加 PID、重力和制动，输出限幅力矩。
 *   - PitchController_StartMove / ReturnTo：普通运动与可中断归位分别处理。
 *   - PitchStep_*：复用 YawStep 统计核心，保留 Pitch 判定参数。
 *   - PitchBackend_Apply / SetViewLock：处理坐标、使能、锁轴与反馈保护。
 * 调用关系：控制任务 → PitchBackend → PitchController → 共用 YawCascade → CanBus。
 * 维护约束：重力补偿依赖角度坐标和水平标定；锁轴保持锁定时目标，不随受扰反馈重取。
 * 参数入口：app_config.h；对外声明仍在原头文件中。
 */

/* APP_SECTION_BEGIN pitch_controller.c */
/* Pitch 控制数学：坐标、重力补偿、制动和轨迹。 */
#include "pitch_controller.h"
#include <float.h>
#include <math.h>
#include <string.h>

#if YAW_CASCADE_OUTPUT_LIMITS_ENABLED
#error Pitch_requires_protocol_only_cascade_output_limits
#endif

static uint8_t finite_value(float x) { return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
static float pitch_controller_absolute(float x) { return x < 0.0f ? -x : x; }
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
{ return kg * cosf((pitch->angle_deg + pitch->gravity_offset_deg) / PITCH_RAD_TO_DEG); }
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
    trace->active = (uint8_t)(pitch_controller_absolute(span) >= 1.0f);
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
    if (elapsed >= APP_TICK_HALF_RANGE) { return; }
    if (elapsed > PITCH_LANDING_TRACE_WINDOW_MS) { trace->active = 0U; return; }
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
    if (!brake->approach_done && pitch_controller_absolute(remaining) < APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG &&
        delta >= -0.0001f && closing > brake->safe_velocity_dps) {
        weight = 1.0f - pitch_controller_absolute(remaining) / APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG;
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
    previous = brake->torque_nm * output_direction > 0.0f ? pitch_controller_absolute(brake->torque_nm) : 0.0f;
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
        pitch_controller_absolute(pitch->velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS);
}
void PitchController_Init(PitchController *pitch)
{
    if (!pitch) { return; }
    memset(pitch, 0, sizeof(*pitch)); pitch->feedback_divisor = 1U;
    pitch->shape.duration_ms = APP_PITCH_SHAPE_MS;
    pitch->shape.velocity_gain = APP_PITCH_SHAPE_VELOCITY_GAIN;
    pitch->brake.enabled = APP_PITCH_TERMINAL_BRAKE_ENABLED;
    YawCascade_Init(&pitch->cascade);
    /* 共用 PID 状态初始化后，加载 Pitch 自身的默认增益。 */
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
        pitch_controller_absolute(feedback->position_rad) > DM3507_POSITION_MAX_RAD + 0.00001f ||
        pitch_controller_absolute(feedback->velocity_rad_s) > DM3507_VELOCITY_MAX_RAD_S + 0.00001f ||
        pitch_controller_absolute(feedback->torque_nm) > DM3507_TORQUE_MAX_NM + 0.00001f) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); pitch->stationary_samples = 0U; return 0U;
    }
    divisor = (float)pitch->feedback_divisor;
    if (divisor != 1.0f && divisor != 7.0f) { fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0U; }
    first = (uint8_t)!pitch->have_feedback; was_active = pitch->active;
    if (!first) {
        gap = feedback->received_ms - pitch->last_feedback_ms;
        if (!gap || gap >= APP_TICK_HALF_RANGE) { return 0U; }
        delta = feedback->position_rad - pitch->last_position_rad;
        if (delta > DM3507_POSITION_MAX_RAD) { delta -= 2.0f * DM3507_POSITION_MAX_RAD; }
        else if (delta < -DM3507_POSITION_MAX_RAD) { delta += 2.0f * DM3507_POSITION_MAX_RAD; }
        if (gap >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
            rebase = 1U; fault(pitch, PITCH_CONTROLLER_FAULT_FEEDBACK);
        }
    }
    pitch->last_position_rad = feedback->position_rad; pitch->last_feedback_ms = feedback->received_ms;
    ++pitch->last_sequence; pitch->have_feedback = 1U;
    pitch->velocity_dps = feedback->velocity_rad_s * PITCH_RAD_TO_DEG / divisor;
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
        pitch->angle_deg = (pitch->continuous_rad - pitch->reference_rad) * PITCH_RAD_TO_DEG / divisor;
        observe_pitch(pitch, delta * PITCH_RAD_TO_DEG / divisor, gap);
    }
    if (stationary_safe(pitch) && (first || rebase ||
        pitch_controller_absolute(delta) * PITCH_RAD_TO_DEG / divisor <= 0.05f + 0.5f * (float)gap * 0.001f)) {
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
        (pitch->faults & ~(uint32_t)PITCH_RECOVERABLE_FAULTS)) { return 0U; }
    /* 改软件零时保持已标定的物理水平，不把新零误认为水平。 */
    if (pitch->have_reference && pitch->gravity_calibrated) {
        pitch->gravity_offset_deg = normalize_degrees(pitch->gravity_offset_deg + pitch->angle_deg);
    } else { PitchController_InvalidateGravity(pitch); }
    PitchController_Stop(pitch); pitch->reference_rad = pitch->continuous_rad;
    pitch->angle_deg = pitch->target_deg = 0.0f; pitch->have_reference = 1U;
    reset_shape(pitch);
    YawCascade_ResetObserver(&pitch->cascade); pitch->faults &= ~(uint32_t)PITCH_RECOVERABLE_FAULTS;
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
static uint8_t StartMove(PitchController *pitch, float degrees, uint32_t now_ms, uint8_t preempt)
{
    float span;
    if (!pitch || !finite_value(degrees) || degrees < PITCH_CONTROLLER_TARGET_MIN_DEG ||
        degrees > PITCH_CONTROLLER_TARGET_MAX_DEG || !pitch->active || !pitch->have_reference ||
        !pitch->have_feedback || pitch->faults || pitch->last_status != 1U ||
        now_ms - pitch->last_feedback_ms >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) { return 0U; }
    if (degrees == pitch->target_deg) { return 1U; }
    if (!preempt && pitch->shape.active) { return 0U; }
    if (!preempt && pitch->shape.duration_ms && pitch_controller_absolute(degrees - pitch->angle_deg) >= 1.0f) {
        if (pitch_controller_absolute(pitch->velocity_dps) > PITCH_CONTROLLER_STATIONARY_SPEED_DPS ||
            pitch_controller_absolute(pitch->cascade.velocity_dps) > PITCH_CONTROLLER_STATIONARY_SPEED_DPS) { return 0U; }
        pitch->target_deg = degrees; reset_shape(pitch); reset_brake(pitch); clear_brake_statistics(pitch);
        pitch->shape.start_ms = now_ms; pitch->shape.start_deg = pitch->angle_deg;
        pitch->shape.position_deg = pitch->angle_deg; pitch->shape.active = 1U;
        start_landing_trace(pitch, degrees - pitch->angle_deg, now_ms);
        return 1U;
    }
    span = degrees - pitch->angle_deg;
    if (!PitchController_SetTarget(pitch, degrees)) { return 0U; }
    start_landing_trace(pitch, span, now_ms);
    if ((preempt || !pitch->shape.duration_ms) && pitch->brake.enabled && pitch_controller_absolute(span) >= 1.0f) {
        pitch->brake.direction = span > 0.0f ? 1.0f : -1.0f;
        pitch->brake.target_deg = degrees; pitch->brake.tracking = 1U;
    }
    return 1U;
}
uint8_t PitchController_StartMove(PitchController *pitch, float degrees, uint32_t now_ms)
{ return StartMove(pitch, degrees, now_ms, 0U); }
uint8_t PitchController_ReturnTo(PitchController *pitch, float degrees, uint32_t now_ms)
{
    /* Explicit return preempts only the motion plan. Keep PID state, torque
     * limits, feedback/fault guards and the configured shape for future tests. */
    return StartMove(pitch, degrees, now_ms, 1U);
}
uint8_t PitchController_SetBrakeEnabled(PitchController *pitch, uint8_t enabled)
{
    if (!pitch || enabled > 1U) { return 0U; }
    if (pitch->brake.enabled != enabled) { reset_brake(pitch); pitch->brake.enabled = enabled; }
    return 1U;
}
uint8_t PitchController_SetGravityOffset(PitchController *pitch, float offset_deg)
{
    if (!pitch || !finite_value(offset_deg) || pitch_controller_absolute(offset_deg) > 180.0f || pitch->active ||
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
    if (!pitch || !finite_value(kg_nm) || pitch_controller_absolute(kg_nm) > PITCH_CONTROLLER_TORQUE_LIMIT_NM ||
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
        pitch->gravity_i_transferred = (uint8_t)(pitch_controller_absolute((next_i - previous_i) - delta) < 0.00001f);
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
        !finite_value(pitch->gravity_offset_deg) || pitch_controller_absolute(pitch->gravity_kg_nm) > PITCH_CONTROLLER_TORQUE_LIMIT_NM ||
        pitch_controller_absolute(pitch->gravity_offset_deg) > 180.0f) {
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
    /* 重力项和末段制动在力矩层叠加；串级 PID 内统一限幅并处理抗饱和。
     * 此处角度来自电机反馈，绝不使用控制台的演示角度。 */
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
/* APP_SECTION_END pitch_controller.c */

/* APP_SECTION_BEGIN pitch_step.c */
/* Pitch 实测统计适配：使用独立判定参数与实例。 */
#include "pitch_step.h"

static float pitch_step_absolute(float x) { return x < 0.0f ? -x : x; }
void PitchStep_Init(PitchStep *step) { YawStep_Init(step); }
uint8_t PitchStep_Start(PitchStep *step, uint32_t now_ms, float start_deg, float target_deg)
{
    if (pitch_step_absolute(start_deg) > 50.0f || pitch_step_absolute(target_deg) > 50.0f) { return 0U; }
    return YawStep_Start(step, now_ms, start_deg, target_deg);
}
void PitchStep_Sample(PitchStep *step, uint32_t received_ms, float angle_deg,
    float velocity_dps, float torque_nm, uint8_t limited)
{
    uint8_t running = (uint8_t)(step && step->phase == PITCH_STEP_RUNNING);
    YawStep_Sample(step, received_ms, angle_deg, velocity_dps, torque_nm, limited);
    if (running && step->phase == PITCH_STEP_DONE) {
        step->pass = YawStep_Passed(step, PITCH_STEP_RESPONSE_LIMIT_MS);
    }
}
void PitchStep_Command(PitchStep *step, uint32_t now_ms, float torque_nm)
{ YawStep_Command(step, now_ms, torque_nm); }
void PitchStep_Abort(PitchStep *step, uint32_t now_ms, PitchStepReason reason, uint32_t faults)
{ YawStep_Abort(step, now_ms, reason, faults); }
/* APP_SECTION_END pitch_step.c */

/* APP_SECTION_BEGIN pitch_backend.c */
/* Pitch 会话：使能、水平标定、锁轴和可中断归位。 */
#include "pitch_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "task.h"
#include <math.h>
#include <string.h>

static PitchBackendState state;
static PitchStep step;
static PitchRemoteStepEvent remote_event;
static uint32_t last_step_sequence, last_tx_ms, enable_ms, disable_ms, last_query_ms;
static uint8_t rearm_request, remote_pending, cancel_pending;
static uint8_t disable_submitted, actuation_started, last_mode;
static float submitted_torque;
static uint8_t submitted_limited;

static uint8_t Online(uint32_t now)
{
    CanBusState bus;
    CanBus_GetState(&bus);
    return (uint8_t)(state.controller.have_feedback && bus.started && !bus.fault_epoch &&
        (uint32_t)(now - state.feedback.received_ms) < PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        state.feedback.fault_epoch == bus.fault_epoch);
}

void PitchBackend_Init(void)
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    PitchController_Init(&state.controller); PitchStep_Init(&step);
    memset(&remote_event, 0, sizeof(remote_event));
    last_step_sequence = last_tx_ms = enable_ms = disable_ms = last_query_ms = 0U;
    rearm_request = remote_pending = cancel_pending = 0U;
    disable_submitted = actuation_started = 0U; last_mode = GIMBAL_MANUAL;
    submitted_torque = 0.0f; submitted_limited = 0U;
    /* Do not transmit FC or FD merely because the MCU was reset. */
    taskEXIT_CRITICAL();
}

void PitchBackend_GetState(PitchBackendState *out)
{
    uint32_t now;
    if (!out) { return; }
    taskENTER_CRITICAL(); now = HAL_GetTick();
    state.online = Online(now);
    state.feedback_age_ms = state.controller.have_feedback ? now - state.feedback.received_ms : 0U;
    *out = state; taskEXIT_CRITICAL();
}

uint8_t PitchBackend_QueryFeedback(void)
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = (uint8_t)(CanBus_PitchReadFeedback() == HAL_OK);
    if (result) { state.monitoring = 1U; last_query_ms = HAL_GetTick(); }
    taskEXIT_CRITICAL(); return result;
}

static void StopWithReason(PitchStopReason reason)
{
    uint8_t had_session;
    taskENTER_CRITICAL();
    had_session = (uint8_t)(state.active || state.enabling || actuation_started);
    PitchStep_Abort(&step, HAL_GetTick(), state.controller.faults ? PITCH_STEP_FAULT : PITCH_STEP_STOP,
        state.controller.faults);
    CanBus_SetPitchPermission(0U);
    PitchController_Stop(&state.controller);
    state.requested = state.active = state.enabling = state.step_hold = 0U;
    state.view_hold_pending = state.view_locked;
    state.view_prepare_pending = 0U;
    state.view_hold_suspended = state.view_locked;
    last_mode = GIMBAL_MANUAL;
    if (had_session && !state.stop_pending) {
        ++state.stop_id; state.stopped_ms = HAL_GetTick(); state.stop_reason = reason;
        state.stop_pending = 1U; cancel_pending = 1U; disable_submitted = 0U;
        disable_ms = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_Stop(void) { StopWithReason(PITCH_STOP_EXTERNAL); }

uint8_t PitchBackend_Request(uint8_t enabled)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (!enabled) {
        /* Explicit OFF requests FD even if the user has only queried the motor. */
        actuation_started = 1U; PitchBackend_Stop(); accepted = 1U;
    } else if (APP_PITCH_ENABLED && state.coord_confirmed && state.zero_set &&
        state.controller.have_reference && Online(HAL_GetTick()) && state.feedback.status == 0U &&
        !state.controller.faults && !state.active && !state.enabling && !state.stop_pending) {
        state.requested = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetCoordinate(uint8_t divisor)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((divisor == 1U || divisor == 7U) && !state.requested && !state.active &&
        !state.enabling && !state.stop_pending && Online(HAL_GetTick()) && state.feedback.status == 0U) {
        accepted = PitchController_SetFeedbackDivisor(&state.controller, divisor);
        if (accepted) {
            state.coord_confirmed = 1U; state.zero_set = 0U;
            state.controller.have_reference = 0U;
            PitchController_InvalidateGravity(&state.controller);
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_TARGET_CHANGE, 0U);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_CaptureReference(void)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (state.coord_confirmed && !state.requested && !state.active && !state.enabling &&
        !state.stop_pending && Online(HAL_GetTick()) && state.feedback.status == 0U) {
        uint32_t faults = state.controller.faults;
        /* Healthy disabled feedback can recover a prior enable timeout/disabled report.
         * TX/CAN faults remain latched; changing coordinates is not a bus recovery. */
        state.controller.faults &= ~(PITCH_BACKEND_FAULT_ENABLE | PITCH_BACKEND_FAULT_DISABLED);
        accepted = PitchController_CaptureReference(&state.controller);
        if (accepted) { state.zero_set = 1U; actuation_started = 0U; }
        else { state.controller.faults |= faults; }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_OnFrame(const CanFrame *frame)
{
    Dm3507Feedback feedback;
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    if (!Dm3507_Decode(frame, &feedback)) { return 0U; }
    taskENTER_CRITICAL();
    if ((uint32_t)(now - feedback.received_ms) < PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        (!state.controller.have_feedback || (int32_t)(feedback.received_ms - state.feedback.received_ms) > 0)) {
        CanBusState bus;
        CanBus_GetState(&bus);
        if (feedback.fault_epoch != bus.fault_epoch) { taskEXIT_CRITICAL(); return 0U; }
        state.feedback = feedback; ++state.rx_count;
        accepted = PitchController_OnFeedback(&state.controller, &feedback);
        if (feedback.status == 1U) { actuation_started = 1U; }
        if (!state.controller.have_reference) { state.zero_set = 0U; }
        /* Status0 confirms FD, not a brake or stationary mechanical support. */
        if (state.stop_pending && disable_submitted && !cancel_pending && feedback.status == 0U &&
            (int32_t)(feedback.received_ms - disable_ms) > 0) {
            state.stop_pending = 0U; actuation_started = 0U;
        }
        if (state.active && feedback.status == 0U) { state.controller.faults |= PITCH_BACKEND_FAULT_DISABLED; }
        if (step.phase == PITCH_STEP_RUNNING) {
            if (state.controller.faults) {
                PitchStep_Abort(&step, feedback.received_ms, PITCH_STEP_FAULT, state.controller.faults);
            } else if (state.active && feedback.status == 1U && CanBus_PitchAllowed()) {
                PitchStep_Sample(&step, feedback.received_ms, state.controller.angle_deg,
                    state.controller.velocity_dps, submitted_torque, submitted_limited);
            } else { PitchStep_Abort(&step, feedback.received_ms, PITCH_STEP_STOP, 0U); }
        }
        if (state.controller.faults && (state.active || state.enabling)) {
            StopWithReason(PITCH_STOP_FEEDBACK_FAULT); rearm_request = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetLoopPid(char loop, float kp, float ki, float kd)
{
    uint8_t accepted;
    uint32_t revision;
    taskENTER_CRITICAL(); revision = state.controller.cascade.revision;
    accepted = PitchController_SetPid(&state.controller, loop, kp, ki, kd);
    if (accepted && revision != state.controller.cascade.revision) {
        PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_PID_CHANGE, 0U);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetLoopParameter(char loop, char parameter, float value)
{
    YawCascadePid pid;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (loop == 'P' || loop == 'V') {
        pid = loop == 'P' ? state.controller.cascade.position : state.controller.cascade.speed;
        if (parameter == 'P') { pid.kp = value; }
        else if (parameter == 'I') { pid.ki = value; }
        else if (parameter == 'D') { pid.kd = value; }
        if (parameter == 'P' || parameter == 'I' || parameter == 'D') {
            accepted = PitchBackend_SetLoopPid(loop, pid.kp, pid.ki, pid.kd);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

static uint8_t SetTarget(float degrees, uint8_t returning)
{
    uint8_t accepted = 0U;
    uint32_t now;
    float previous;
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    if (!state.view_locked && state.active && state.requested && state.zero_set &&
        !state.enabling && !state.stop_pending && Online(now) &&
        state.feedback.status == 1U && !state.controller.faults && CanBus_PitchAllowed()) {
        previous = state.controller.target_deg;
        accepted = returning ? PitchController_ReturnTo(&state.controller, degrees, now) :
            PitchController_StartMove(&state.controller, degrees, now);
        if (accepted) {
            if (previous != degrees) { PitchStep_Abort(&step, now, PITCH_STEP_TARGET_CHANGE, 0U); }
            state.step_hold = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetTarget(float degrees) { return SetTarget(degrees, 0U); }
uint8_t PitchBackend_ReturnTo(float degrees) { return SetTarget(degrees, 1U); }

uint8_t PitchBackend_SetGravityGain(float kg_nm)
{
    uint8_t accepted = 0U;
    float previous;
    taskENTER_CRITICAL(); previous = state.controller.gravity_kg_nm;
    if (kg_nm == 0.0f || (state.coord_confirmed && state.zero_set && Online(HAL_GetTick()) &&
        !state.enabling && !state.stop_pending && !state.controller.faults &&
        ((!state.active && state.feedback.status == 0U) ||
         (state.active && state.feedback.status == 1U && CanBus_PitchAllowed())))) {
        accepted = PitchController_SetGravityGain(&state.controller, kg_nm);
        if (accepted && previous != state.controller.gravity_kg_nm) {
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_FF_CHANGE, 0U);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetGravityOffset(float offset_deg, uint8_t capture_horizontal)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (state.coord_confirmed && state.zero_set && Online(HAL_GetTick()) &&
        !state.active && !state.enabling && !state.stop_pending && state.feedback.status == 0U &&
        !state.controller.faults) {
        accepted = capture_horizontal ? PitchController_CaptureHorizontal(&state.controller) :
            PitchController_SetGravityOffset(&state.controller, offset_deg);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_StartStep(float degrees)
{
    uint8_t accepted = 0U;
    uint32_t now;
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    if (!state.view_locked && (degrees == 30.0f || degrees == -30.0f) && state.coord_confirmed && state.zero_set &&
        state.requested && state.active && state.controller.active && !state.enabling &&
        !state.stop_pending && Online(now) && state.feedback.status == 1U &&
        !state.controller.faults && CanBus_PitchAllowed() &&
        fabsf(state.controller.angle_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(state.controller.target_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(state.controller.velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        state.controller.stationary_samples >= 2U &&
        state.controller.last_feedback_ms - state.controller.stationary_started_ms >=
            PITCH_CONTROLLER_STATIONARY_HOLD_MS && step.phase != PITCH_STEP_RUNNING) {
        if (PitchController_StartMove(&state.controller, degrees, now)) {
            accepted = PitchStep_Start(&step, now, state.controller.angle_deg, degrees);
            if (accepted) {
                step.kp = state.controller.cascade.position.kp;
                step.ki = state.controller.cascade.position.ki;
                step.kd = state.controller.cascade.position.kd;
                step.vkp = state.controller.cascade.speed.kp;
                step.vki = state.controller.cascade.speed.ki;
                step.vkd = state.controller.cascade.speed.kd;
                step.gravity_kg_nm = state.controller.gravity_kg_nm;
                step.gravity_offset_deg = state.controller.gravity_offset_deg;
                step.gravity_calibrated = state.controller.gravity_calibrated;
                step.shape_ms = state.controller.shape.duration_ms;
                step.shape_velocity_gain = state.controller.shape.velocity_gain;
                step.pitch_brake_enabled = state.controller.brake.enabled;
                step.cascade = 1U; step.output_limits = 0U; step.vmax_dps = 0.0f;
                step.last_velocity_dps = state.controller.velocity_dps;
                step.last_torque_nm = submitted_torque;
                state.step_hold = 1U;
            }
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t PitchBackend_SetShape(uint32_t duration_ms, float velocity_gain)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (step.phase != PITCH_STEP_RUNNING ||
        (duration_ms == state.controller.shape.duration_ms &&
         velocity_gain == state.controller.shape.velocity_gain)) {
        accepted = PitchController_SetShape(&state.controller, duration_ms, velocity_gain);
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t PitchBackend_SetBrakeEnabled(uint8_t enabled)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (step.phase != PITCH_STEP_RUNNING || enabled == state.controller.brake.enabled) {
        accepted = PitchController_SetBrakeEnabled(&state.controller, enabled);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint32_t PitchBackend_GetStepId(void)
{
    uint32_t id;
    taskENTER_CRITICAL(); id = step.id; taskEXIT_CRITICAL();
    return id;
}
void PitchBackend_GetStep(PitchStep *out)
{ if (out) { taskENTER_CRITICAL(); *out = step; taskEXIT_CRITICAL(); } }
uint8_t PitchBackend_TakeStepReport(PitchStep *out)
{
    uint8_t result = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (step.pending) { *out = step; step.pending = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t PitchBackend_TakeRemoteStepEvent(PitchRemoteStepEvent *out)
{
    uint8_t result = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (remote_pending) { *out = remote_event; remote_pending = 0U; result = 1U; }
    taskEXIT_CRITICAL(); return result;
}
uint8_t PitchBackend_TakeRearmRequest(void)
{
    uint8_t result;
    taskENTER_CRITICAL(); result = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL(); return result;
}
static void Notice(const TargetRequest *request, uint8_t result)
{
    remote_event.sequence = request->step_sequence;
    remote_event.target_deg = result == 1U || result == 2U ? state.controller.target_deg :
        (request->step_return_event ? state.controller.target_deg + request->step_delta_deg : request->step_delta_deg);
    remote_event.result = result; remote_pending = 1U;
}

static uint8_t StationarySupport(void)
{
    const PitchController *pitch = &state.controller;
    return (uint8_t)(fabsf(pitch->target_deg - pitch->angle_deg) <= PITCH_STEP_BAND_DEG &&
        fabsf(pitch->velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        fabsf(pitch->cascade.velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS &&
        pitch->stationary_samples >= 2U &&
        pitch->last_feedback_ms - pitch->stationary_started_ms >= PITCH_CONTROLLER_STATIONARY_HOLD_MS);
}

static void TransferPositionSupport(void)
{
    float next = state.controller.cascade.speed_i +
        state.controller.cascade.speed.kp * state.controller.cascade.position_i;
    if (next > PITCH_CONTROLLER_TORQUE_LIMIT_NM) { next = PITCH_CONTROLLER_TORQUE_LIMIT_NM; }
    else if (next < -PITCH_CONTROLLER_TORQUE_LIMIT_NM) { next = -PITCH_CONTROLLER_TORQUE_LIMIT_NM; }
    state.controller.cascade.speed_i = next;
    state.controller.cascade.position_i = 0.0f;
}

uint8_t PitchBackend_HoldCurrent(void)
{
    uint8_t accepted = 0U;
    uint8_t carry_support;
    float target;
    taskENTER_CRITICAL();
    carry_support = StationarySupport();
    target = state.controller.angle_deg;
    if (target > PITCH_CONTROLLER_TARGET_MAX_DEG &&
        target <= PITCH_CONTROLLER_TARGET_MAX_DEG + PITCH_STEP_BAND_DEG) {
        target = PITCH_CONTROLLER_TARGET_MAX_DEG;
    } else if (target < PITCH_CONTROLLER_TARGET_MIN_DEG &&
        target >= PITCH_CONTROLLER_TARGET_MIN_DEG - PITCH_STEP_BAND_DEG) {
        target = PITCH_CONTROLLER_TARGET_MIN_DEG;
    }
    if (state.active && state.requested && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 1U && CanBus_PitchAllowed() &&
        !state.controller.faults) {
        if (state.view_locked && !state.view_hold_pending) { accepted = 1U; }
        else if (PitchController_SetTarget(&state.controller, target)) {
            PitchStep_Abort(&step, HAL_GetTick(), PITCH_STEP_MANUAL, 0U);
            if (state.view_locked) {
                /* Only a settled near-target outer I is treated as load
                 * support. Motion I must not become a persistent torque bias
                 * after cancelling that movement and latching a new pose. */
                if (carry_support) { TransferPositionSupport(); }
                else { state.controller.cascade.position_i = 0.0f; }
                state.view_hold_deg = target;
            } else { state.controller.cascade.position_i = 0.0f; }
            /* Gravity and existing inner load compensation remain active. */
            state.step_hold = 1U; state.view_hold_pending = state.view_prepare_pending = 0U; accepted = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

void PitchBackend_SetViewLock(uint8_t locked)
{
    taskENTER_CRITICAL(); locked = (uint8_t)(locked != 0U);
    if (state.view_locked != locked) {
        if (!locked && state.active && !state.view_hold_pending && !state.controller.faults) {
            if (StationarySupport()) { TransferPositionSupport(); }
            else { state.controller.cascade.position_i = 0.0f; }
        }
        state.view_locked = state.view_hold_pending = locked;
        state.view_prepare_pending = locked;
        state.view_hold_suspended = 0U;
        if (!locked) { state.step_hold = 0U; }
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_PrepareViewHold(void)
{
    taskENTER_CRITICAL();
    if (!state.active && !state.enabling) {
        state.view_prepare_pending = 1U;
        state.view_hold_suspended = 0U;
        /* Refresh stopped-motor feedback through the existing read-only CC
         * path, rather than waiting forever on an expired last frame. */
        state.monitoring = 1U;
    }
    taskEXIT_CRITICAL();
}

void PitchBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic)
{
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    uint8_t wanted, event = 0U, new_step = 0U;
    taskENTER_CRITICAL();
    if (request && request->step_event && request->step_sequence != last_step_sequence) {
        last_step_sequence = request->step_sequence;
        new_step = 1U;
        event = (uint8_t)(request->step_axis == GIMBAL_STEP_PITCH);
    }
    if ((state.active || state.enabling) && !Online(now)) {
        state.controller.faults |= PITCH_CONTROLLER_FAULT_FEEDBACK;
        StopWithReason(PITCH_STOP_FEEDBACK); rearm_request = 1U;
    }
    /* Selected view is durable intent, unlike V3's one-shot ticket which READY
     * could clear permanently. A stop made while permission is still live
     * suspends reconciliation. Observe blocked output (normal global STOP/OFF
     * and rearm path), or receive a fresh VIEW, before joining again. */
    if (!allowed) { state.view_hold_suspended = 0U; }
    if (state.view_locked && state.view_hold_pending && !state.requested &&
        !state.active && !state.enabling && !state.view_hold_suspended) {
        state.view_prepare_pending = 1U; state.monitoring = 1U;
    }
    /* Reuse Request + original FC/new status1 handshake. Existing reference,
     * calibration, faults, fresh feedback and global permission remain gates. */
    if (state.view_locked && state.view_hold_pending && !state.view_hold_suspended &&
        !state.requested && allowed && !diagnostic && request &&
        state.coord_confirmed && state.zero_set && state.controller.have_reference &&
        !state.active && !state.enabling && !state.stop_pending && !state.controller.faults &&
        Online(now) && state.feedback.status == 0U &&
        (state.controller.gravity_kg_nm == 0.0f || state.controller.gravity_calibrated)) {
        if (PitchBackend_Request(1U)) { state.view_prepare_pending = 0U; }
    }
    wanted = (uint8_t)(APP_PITCH_ENABLED && state.requested && state.coord_confirmed &&
        state.zero_set && allowed && !diagnostic && request && Online(now) &&
        !state.controller.faults && !state.stop_pending &&
        (state.controller.gravity_kg_nm == 0.0f || state.controller.gravity_calibrated));
    if (!wanted) {
        if (event) { Notice(request, 3U); }
        if (state.active || state.enabling) { StopWithReason(PITCH_STOP_PERMISSION); rearm_request = 1U; }
        CanBus_SetPitchPermission(0U);
        if (state.stop_pending && (!disable_submitted || now - last_tx_ms >= 10U)) {
            result = CanBus_PitchDisable(cancel_pending); last_tx_ms = now;
            if (result == HAL_OK) {
                if (!disable_submitted) { disable_ms = now; }
                disable_submitted = 1U; cancel_pending = 0U;
                submitted_torque = 0.0f; submitted_limited = 0U;
            }
        }
        /* The user's explicit CHECK/READY starts read-only monitoring. This keeps
         * disabled/ready feedback current while the user reads a summary or moves CH5.
         * No monitoring is started automatically on MCU reset. */
        if (state.monitoring && now - last_query_ms >= 10U) {
            if (CanBus_PitchReadFeedback() == HAL_OK) { last_query_ms = now; }
        }
        taskEXIT_CRITICAL(); return;
    }
    CanBus_SetPitchPermission(1U);
    if (!CanBus_PitchAllowed()) {
        StopWithReason(PITCH_STOP_CAN_PERMISSION); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (!state.active && !state.enabling) {
        if (event) { Notice(request, 3U); }
        /* Preserve the measured reference, but enable holding the current supported pose. */
        if (state.controller.angle_deg < PITCH_CONTROLLER_TARGET_MIN_DEG ||
            state.controller.angle_deg > PITCH_CONTROLLER_TARGET_MAX_DEG ||
            !PitchController_SetTarget(&state.controller, state.controller.angle_deg)) {
            state.controller.faults |= PITCH_CONTROLLER_FAULT_LIMIT;
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        result = CanBus_PitchEnable(); last_tx_ms = now;
        if (result == HAL_OK) {
            state.enabling = 1U; actuation_started = 1U; enable_ms = now;
        } else {
            state.controller.faults |= PITCH_BACKEND_FAULT_TX;
            StopWithReason(PITCH_STOP_ENABLE_TX); rearm_request = 1U;
        }
        taskEXIT_CRITICAL(); return;
    }
    if (state.enabling) {
        if (event) { Notice(request, 3U); }
        if (state.feedback.status == 1U && (int32_t)(state.feedback.received_ms - enable_ms) > 0) {
            state.enabling = 0U; state.active = state.controller.active = 1U;
        } else if (now - enable_ms >= APP_PITCH_ENABLE_TIMEOUT_MS) {
            state.controller.faults |= PITCH_BACKEND_FAULT_ENABLE;
            StopWithReason(PITCH_STOP_ENABLE_TIMEOUT); rearm_request = 1U;
        } else if (now - last_tx_ms >= 10U) {
            result = CanBus_PitchEnable(); last_tx_ms = now;
            if (result != HAL_OK) {
                state.controller.faults |= PITCH_BACKEND_FAULT_TX;
                StopWithReason(PITCH_STOP_ENABLE_TX); rearm_request = 1U;
            }
        }
        taskEXIT_CRITICAL(); return;
    }
    if (last_mode == GIMBAL_TEST && request->gimbal_mode != GIMBAL_TEST) {
        PitchStep_Abort(&step, now, PITCH_STEP_MANUAL, 0U);
        StopWithReason(PITCH_STOP_MODE_EXIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked && state.view_hold_pending && !PitchBackend_HoldCurrent()) {
        StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (event && state.view_locked) { Notice(request, 3U); event = 0U; }
#if APP_REMOTE_PROFILE_ASSESSMENT
    if (request->gimbal_mode == GIMBAL_TEST &&
        (last_mode != GIMBAL_TEST || (new_step && request->step_axis == GIMBAL_STEP_YAW))) {
        if (!PitchBackend_HoldCurrent()) {
            if (event) { Notice(request, 3U); }
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
    }
#endif
    last_mode = (uint8_t)request->gimbal_mode;
    if (event) {
        if (request->gimbal_mode != GIMBAL_TEST) { Notice(request, 3U); }
        else if (request->step_return_event) {
            Notice(request, PitchBackend_ReturnTo(state.controller.target_deg + request->step_delta_deg) ? 2U : 3U);
        } else if (PitchBackend_StartStep(request->step_delta_deg)) {
            Notice(request, 1U);
        } else { Notice(request, 3U); }
    }
    if (!state.view_locked && state.step_hold && request->gimbal_mode == GIMBAL_MANUAL && request->pitch_rate_dps != 0.0f) {
        PitchStep_Abort(&step, now, PITCH_STEP_MANUAL, 0U);
        StopWithReason(PITCH_STOP_MANUAL_OVERRIDE); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked) {
        /* The latched target is independent of disturbed feedback. Never
         * capture again until unlock/rearm; leave all original safety gates. */
        if ((state.controller.target_deg != state.view_hold_deg || state.controller.shape.active ||
             state.controller.brake.tracking) && !PitchController_SetTarget(&state.controller, state.view_hold_deg)) {
            StopWithReason(PITCH_STOP_START_LIMIT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        state.step_hold = 1U;
        (void)PitchController_UpdateViewHold(&state.controller, now, 1U);
    } else {
        (void)PitchController_Update(&state.controller,
            request->gimbal_mode == GIMBAL_MANUAL && !state.step_hold ? request->pitch_rate_dps : 0.0f, now, 1U);
    }
    if (state.controller.faults) {
        StopWithReason(PITCH_STOP_CONTROL_FAULT); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (now - last_tx_ms >= APP_PITCH_COMMAND_PERIOD_MS) {
        result = CanBus_PitchTorque(state.controller.torque_nm); last_tx_ms = now;
        if (result != HAL_OK) {
            state.controller.faults |= PITCH_BACKEND_FAULT_TX;
            StopWithReason(PITCH_STOP_TORQUE_TX); rearm_request = 1U;
        } else {
            submitted_torque = state.controller.torque_nm;
            submitted_limited = state.controller.cascade.torque_limited;
            PitchStep_Command(&step, now, submitted_torque);
        }
    }
    taskEXIT_CRITICAL();
}
/* APP_SECTION_END pitch_backend.c */
