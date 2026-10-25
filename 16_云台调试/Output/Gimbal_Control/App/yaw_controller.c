/**
 * @file yaw_controller.c
 * @brief Yaw 轴及共用串级 PID
 *
 * 职责：实现共用串级 PID、Yaw 控制、阶跃实测和 GL60 II 会话后端。
 * 主要函数：
 *   - YawCascade_*：两轴共用的速度观测、双环 PID 与抗积分饱和。
 *   - YawController_Update：Yaw 位置目标、轨迹/前馈和力矩计算。
 *   - YawStep_*：统计真实反馈的首次到位、稳定时间、超调及稳态误差。
 *   - YawBackend_Apply / StartStep：处理许可、使能、锁轴及相对阶跃。
 * 调用关系：控制任务 → YawBackend → YawController/YawCascade → CanBus；Pitch 也调用 YawCascade。
 * 维护约束：共用 PID 不是 Yaw 专属；实测结果不读取演示角度，限位和旧模式开关保留。
 * 参数入口：app_config.h；对外声明仍在原头文件中。
 */

/* APP_SECTION_BEGIN yaw_cascade.c */
/* 两轴共用 PID 数学：观测、双环、前馈与抗饱和。 */
#include "yaw_cascade.h"
#include "app_config.h"
#include <float.h>
#include <string.h>

static float yaw_cascade_clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static uint8_t yaw_cascade_finite_value(float x) { return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
static uint8_t winds_up(float wanted, float change, float limit)
{ return (uint8_t)((wanted > limit && change > 0.0f) || (wanted < -limit && change < 0.0f)); }

void YawCascade_Stop(YawCascade *loop)
{
    loop->position_i = loop->speed_i = 0.0f;
    loop->torque_nm = loop->unsaturated_torque_nm = loop->target_velocity_dps = 0.0f;
    loop->feedforward_nm = 0.0f;
    loop->have_update = loop->torque_limited = loop->velocity_limited = loop->feedforward_limited = 0U;
}
void YawCascade_ResetObserver(YawCascade *loop)
{ loop->velocity_dps = loop->acceleration_dps2 = 0.0f; YawCascade_Stop(loop); }
void YawCascade_Init(YawCascade *loop)
{
    memset(loop, 0, sizeof(*loop));
    loop->position.kp = APP_YAW_POSITION_KP;
    loop->position.ki = APP_YAW_POSITION_KI;
    loop->position.kd = APP_YAW_POSITION_KD;
    loop->speed.kp = APP_YAW_SPEED_KP;
    loop->speed.ki = APP_YAW_SPEED_KI;
    loop->speed.kd = APP_YAW_SPEED_KD;
    loop->velocity_limit_dps = YAW_CASCADE_OUTPUT_LIMITS_ENABLED ? APP_YAW_CASCADE_LEGACY_VELOCITY_DPS : 0.0f;
}
void YawCascade_Observe(YawCascade *loop, float delta_deg, uint32_t gap_ms)
{
    float previous, dt, alpha;
    if (!gap_ms || gap_ms >= APP_CASCADE_OBSERVER_MAX_GAP_MS || !yaw_cascade_finite_value(delta_deg)) { YawCascade_ResetObserver(loop); return; }
    dt = (float)gap_ms * 0.001f;
    alpha = (float)gap_ms / ((float)YAW_CASCADE_VELOCITY_FILTER_MS + (float)gap_ms);
    previous = loop->velocity_dps;
    loop->velocity_dps += alpha * (delta_deg / dt - loop->velocity_dps);
    alpha = (float)gap_ms / ((float)YAW_CASCADE_ACCEL_FILTER_MS + (float)gap_ms);
    loop->acceleration_dps2 += alpha * ((loop->velocity_dps - previous) / dt - loop->acceleration_dps2);
}
uint8_t YawCascade_SetPid(YawCascade *loop, char which, float kp, float ki, float kd)
{
    YawCascadePid *pid;
    if (!yaw_cascade_finite_value(kp) || !yaw_cascade_finite_value(ki) || !yaw_cascade_finite_value(kd) || kp < 0.0f || ki < 0.0f || kd < 0.0f) { return 0U; }
    if (which == 'P') {
        if (kp > 30.0f || ki > 10.0f || kd > 2.0f) { return 0U; }
        pid = &loop->position;
    } else if (which == 'V') {
        if (kp > 0.02f || ki > 0.2f || kd > 0.001f) { return 0U; }
        pid = &loop->speed;
    } else { return 0U; }
    if (pid->kp == kp && pid->ki == ki && pid->kd == kd) { return 1U; }
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    YawCascade_Stop(loop); ++loop->revision;
    return 1U;
}
uint8_t YawCascade_SetVelocityLimit(YawCascade *loop, float dps)
{
#if YAW_CASCADE_OUTPUT_LIMITS_ENABLED
    if (!yaw_cascade_finite_value(dps) || dps < 10.0f || dps > 360.0f) { return 0U; }
    if (loop->velocity_limit_dps != dps) {
        loop->velocity_limit_dps = dps; YawCascade_Stop(loop); ++loop->revision;
    }
    return 1U;
#else
    (void)loop; (void)dps;
    return 0U;
#endif
}
float YawCascade_Update(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm)
{
    return YawCascade_UpdateWithFeedforward(loop, error_deg, feedback_ms, sequence, torque_limit_nm, 0.0f);
}
static float update(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm, float feedforward_nm,
                        const YawCascadeFeedforward *ff, const YawCascadeReference *reference)
{
    float dt = 0.0f, wanted, candidate, change, speed_error;
    uint32_t gap = feedback_ms - loop->feedback_ms;
    if (loop->have_update && sequence != loop->sequence && gap < APP_CASCADE_OBSERVER_MAX_GAP_MS) { dt = (float)gap * 0.001f; }
    loop->feedback_ms = feedback_ms; loop->sequence = sequence; loop->have_update = 1U;
    if (loop->position.ki == 0.0f) { loop->position_i = 0.0f; }
    change = loop->position.ki * error_deg * dt;
    if (reference && reference->freeze_position_i) { change = 0.0f; }
#if YAW_CASCADE_OUTPUT_LIMITS_ENABLED
    candidate = yaw_cascade_clamp(loop->position_i + change, -loop->velocity_limit_dps, loop->velocity_limit_dps);
    wanted = loop->position.kp * error_deg + candidate - loop->position.kd * loop->velocity_dps;
    /* 外环同时跟踪速度参考及内环力矩饱和；允许反向误差解积分。 */
    if (!winds_up(wanted, change, loop->velocity_limit_dps) &&
        !winds_up(loop->unsaturated_torque_nm, change, torque_limit_nm)) { loop->position_i = candidate; }
    wanted = loop->position.kp * error_deg + loop->position_i - loop->position.kd * loop->velocity_dps;
    loop->target_velocity_dps = yaw_cascade_clamp(wanted, -loop->velocity_limit_dps, loop->velocity_limit_dps);
    loop->velocity_limited = (uint8_t)(wanted > loop->velocity_limit_dps || wanted < -loop->velocity_limit_dps);
#else
    /* 不截断外环速度参考或位置积分；仅在内环协议不可表达时防积累。 */
    candidate = loop->position_i + change;
    if (!winds_up(loop->unsaturated_torque_nm, change, torque_limit_nm)) { loop->position_i = candidate; }
    loop->target_velocity_dps = loop->position.kp * error_deg + loop->position_i - loop->position.kd * loop->velocity_dps;
    loop->velocity_limited = 0U;
#endif
    if (reference) {
        loop->target_velocity_dps += reference->velocity_dps;
#if YAW_CASCADE_OUTPUT_LIMITS_ENABLED
        wanted = loop->target_velocity_dps;
        loop->target_velocity_dps = yaw_cascade_clamp(wanted, -loop->velocity_limit_dps, loop->velocity_limit_dps);
        loop->velocity_limited = (uint8_t)(wanted > loop->velocity_limit_dps || wanted < -loop->velocity_limit_dps);
#endif
    }
    loop->feedforward_limited = 0U;
    if (ff) {
        feedforward_nm = ff->torque_nm;
        loop->feedforward_limited = ff->limited;
    }
    loop->feedforward_nm = feedforward_nm;
    speed_error = loop->target_velocity_dps - loop->velocity_dps;
    if (loop->speed.ki == 0.0f) { loop->speed_i = 0.0f; }
    change = loop->speed.ki * speed_error * dt;
    candidate = yaw_cascade_clamp(loop->speed_i + change, -torque_limit_nm, torque_limit_nm);
    wanted = loop->speed.kp * speed_error + candidate - loop->speed.kd * loop->acceleration_dps2 + feedforward_nm;
    if (!winds_up(wanted, change, torque_limit_nm)) { loop->speed_i = candidate; }
    loop->unsaturated_torque_nm = loop->speed.kp * speed_error + loop->speed_i - loop->speed.kd * loop->acceleration_dps2 + feedforward_nm;
    loop->torque_nm = yaw_cascade_clamp(loop->unsaturated_torque_nm, -torque_limit_nm, torque_limit_nm);
    loop->torque_limited = (uint8_t)(loop->unsaturated_torque_nm > torque_limit_nm || loop->unsaturated_torque_nm < -torque_limit_nm);
    return loop->torque_nm;
}
float YawCascade_UpdateWithFeedforward(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm, float feedforward_nm)
{
    return update(loop, error_deg, feedback_ms, sequence, torque_limit_nm, feedforward_nm, 0, 0);
}
float YawCascade_UpdateTracking(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm, const YawCascadeFeedforward *ff,
                        const YawCascadeReference *reference)
{
    YawCascadeReference legacy;
    if (!reference) { return update(loop, error_deg, feedback_ms, sequence, torque_limit_nm, 0.0f, ff, 0); }
    /* 保持旧Yaw跟踪接口的D前馈语义。 */
    legacy = *reference;
    legacy.velocity_dps *= 1.0f + loop->position.kd;
    return update(loop, error_deg, feedback_ms, sequence, torque_limit_nm, 0.0f, ff, &legacy);
}
float YawCascade_UpdateReference(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm, float feedforward_nm,
                        float velocity_ff_dps, uint8_t freeze_position_i)
{
    YawCascadeReference reference;
    reference.velocity_dps = velocity_ff_dps;
    reference.freeze_position_i = freeze_position_i;
    return update(loop, error_deg, feedback_ms, sequence, torque_limit_nm, feedforward_nm, 0, &reference);
}
/* APP_SECTION_END yaw_cascade.c */

/* APP_SECTION_BEGIN yaw_controller.c */
/* Yaw 控制数学：软件零点、目标、轨迹与保护。 */
#include "yaw_controller.h"
#include <float.h>
#include <string.h>


static uint8_t yaw_controller_finite_value(float value)
{
    return (uint8_t)(value == value && value >= -FLT_MAX && value <= FLT_MAX);
}

static float yaw_controller_absolute(float value) { return value < 0.0f ? -value : value; }
static float yaw_controller_clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static uint8_t valid_ff(const YawFeedforward *ff)
{
    return (uint8_t)(ff->enabled <= 1U && yaw_controller_finite_value(ff->acceleration_gain) &&
        ff->acceleration_gain >= 0.0f && ff->acceleration_gain <= 0.001f &&
        yaw_controller_finite_value(ff->limit_nm) && ff->limit_nm >= 0.0f && ff->limit_nm <= 1.0f);
}

static void fault(YawController *yaw, uint32_t reasons)
{
    yaw->faults |= reasons;
    YawController_Stop(yaw);
}

static uint8_t sample_safe(const YawController *yaw)
{
    return (uint8_t)(yaw->have_feedback && yaw->last_status <= 1U &&
        yaw->last_driver_temp_c < YAW_CONTROLLER_DRIVER_LIMIT_C &&
        yaw->last_motor_temp_c < YAW_CONTROLLER_MOTOR_LIMIT_C &&
        yaw_controller_absolute(yaw->velocity_dps) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS);
}

void YawController_Init(YawController *yaw)
{
    if (yaw == 0) { return; }
    memset(yaw, 0, sizeof(*yaw));
    yaw->pid.kp = APP_YAW_LEGACY_KP;
    yaw->pid.kd = APP_YAW_LEGACY_KD;
    YawCascade_Init(&yaw->cascade);
    yaw->ff.limit_nm = APP_YAW_FF_LIMIT_NM;
    yaw->motion_guard_active = 1U;
}

void YawController_Stop(YawController *yaw)
{
    if (yaw == 0) { return; }
    yaw->active = 0U;
    YawCascade_Stop(&yaw->cascade);
    yaw->speed_limited = 0U;
    yaw->torque_nm = 0.0f;
    yaw->pid_torque_nm = 0.0f;
    yaw->integral = 0.0f;
    yaw->have_pid_feedback = 0U;
    yaw->have_control_tick = 0U;
    yaw->target_deg = yaw_controller_finite_value(yaw->angle_deg) ?
        yaw_controller_clamp(yaw->angle_deg, -YAW_CONTROLLER_TARGET_LIMIT_DEG,
              YAW_CONTROLLER_TARGET_LIMIT_DEG) : 0.0f;
    yaw->shape.active = 0U;
    yaw->shape.position_deg = yaw->target_deg;
    yaw->shape.velocity_dps = yaw->shape.acceleration_dps2 = 0.0f;
}

uint8_t YawController_OnFeedback(YawController *yaw, const Gl60Feedback *feedback)
{
    uint32_t gap = 0U;
    float delta = 0.0f;
    float speed_limit, jump_margin, angle_limit;
    uint8_t first, strict, rebase = 0U;
    if (yaw == 0 || feedback == 0) { return 0U; }
    if (!yaw_controller_finite_value(feedback->position_rad) ||
        !yaw_controller_finite_value(feedback->velocity_rad_s) ||
        !yaw_controller_finite_value(feedback->torque_nm) ||
        yaw_controller_absolute(feedback->position_rad) > GL60_POSITION_MAX_RAD + 0.00001f ||
        feedback->motor_id != 1U) {
        fault(yaw, YAW_CONTROLLER_FAULT_DATA);
        yaw->stationary_samples = 0U;
        return 0U;
    }
    if (yaw->active || feedback->status == 1U) { yaw->motion_guard_active = 1U; }
    strict = (uint8_t)(yaw->motion_guard_active ||
        (feedback->status != 0U && feedback->status != 13U));
    speed_limit = strict ? (YAW_CONTROLLER_SPEED_GUARD_ENABLED ? YAW_CONTROLLER_SPEED_LIMIT_DPS :
        YAW_CONTROLLER_CONTINUITY_DPS) : YAW_CONTROLLER_OFF_SPEED_LIMIT_DPS;
    jump_margin = strict ? (YAW_CONTROLLER_SPEED_GUARD_ENABLED ? 0.1f :
        YAW_CONTROLLER_CONTINUITY_MARGIN_DEG) : YAW_CONTROLLER_OFF_JUMP_MARGIN_DEG;
    angle_limit = strict ? YAW_CONTROLLER_HARD_LIMIT_DEG : YAW_CONTROLLER_OFF_ANGLE_LIMIT_DEG;
    first = (uint8_t)!yaw->have_feedback;
    if (!first) {
        gap = feedback->received_ms - yaw->last_feedback_ms;
        if (gap == 0U || gap >= APP_TICK_HALF_RANGE) { return 0U; }
        delta = feedback->position_rad - yaw->last_position_rad;
        if (delta > YAW_PI_RAD) { delta -= YAW_TWO_PI_RAD; }
        else if (delta < -YAW_PI_RAD) { delta += YAW_TWO_PI_RAD; }
        if (gap >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
            rebase = 1U;
            fault(yaw, YAW_CONTROLLER_FAULT_FEEDBACK);
        } else if (yaw_controller_absolute(delta) * YAW_RAD_TO_DEG >
                   speed_limit * (float)gap / 1000.0f + jump_margin) {
            rebase = 1U;
            fault(yaw, YAW_CONTROLLER_FAULT_JUMP);
        }
    }
    yaw->last_position_rad = feedback->position_rad;
    yaw->last_feedback_ms = feedback->received_ms;
    ++yaw->last_sequence;
    yaw->velocity_dps = feedback->velocity_rad_s * YAW_RAD_TO_DEG;
    yaw->last_status = feedback->status;
    yaw->last_driver_temp_c = feedback->driver_temp_c;
    yaw->last_motor_temp_c = feedback->motor_temp_c;
    yaw->have_feedback = 1U;
    if (first || rebase) {
        if (yaw->ff.enabled) { yaw->ff.enabled = 0U; ++yaw->ff.revision; }
        yaw->continuous_rad = feedback->position_rad;
        yaw->reference_rad = feedback->position_rad;
        yaw->angle_deg = 0.0f;
        yaw->target_deg = 0.0f;
        yaw->shape.active = 0U;
        yaw->shape.position_deg = 0.0f;
        yaw->shape.velocity_dps = yaw->shape.acceleration_dps2 = 0.0f;
        /* 首次静止反馈可显示相对零位。断线/跳变后必须人工重新捕获。 */
        yaw->have_reference = (uint8_t)(first && sample_safe(yaw));
        yaw->stationary_samples = 0U;
    } else {
        yaw->continuous_rad += delta;
        yaw->angle_deg = (yaw->continuous_rad - yaw->reference_rad) * YAW_RAD_TO_DEG;
    }
    if (first || rebase) { YawCascade_ResetObserver(&yaw->cascade); }
    else { YawCascade_Observe(&yaw->cascade, delta * YAW_RAD_TO_DEG, gap); }
    if (sample_safe(yaw) && (first || rebase || yaw_controller_absolute(delta) * YAW_RAD_TO_DEG <=
        0.05f + 0.5f * (float)gap / 1000.0f)) {
        if (yaw->stationary_samples == 0U) {
            yaw->stationary_started_ms = feedback->received_ms;
        }
        if (yaw->stationary_samples < 255U) { ++yaw->stationary_samples; }
    } else { yaw->stationary_samples = 0U; }
    if (feedback->fault_epoch != 0U) { fault(yaw, YAW_CONTROLLER_FAULT_CAN); }
    if (feedback->status > 1U) { fault(yaw, YAW_CONTROLLER_FAULT_STATUS); }
    if (feedback->driver_temp_c >= YAW_CONTROLLER_DRIVER_LIMIT_C ||
        feedback->motor_temp_c >= YAW_CONTROLLER_MOTOR_LIMIT_C) {
        fault(yaw, YAW_CONTROLLER_FAULT_TEMPERATURE);
    }
    if ((!strict || YAW_CONTROLLER_SPEED_GUARD_ENABLED) && yaw_controller_absolute(yaw->velocity_dps) > speed_limit) {
        fault(yaw, YAW_CONTROLLER_FAULT_SPEED);
    }
    if (yaw->have_reference && yaw_controller_absolute(yaw->angle_deg) > angle_limit) {
        fault(yaw, YAW_CONTROLLER_FAULT_LIMIT);
    }
    return 1U;
}

uint8_t YawController_CaptureReference(YawController *yaw)
{
    if (yaw == 0 || yaw->active || !sample_safe(yaw) ||
        yaw->stationary_samples < 2U ||
        yaw->last_feedback_ms - yaw->stationary_started_ms <
            YAW_CONTROLLER_STATIONARY_HOLD_MS ||
        (yaw->faults & ~(uint32_t)YAW_RECOVERABLE_FAULTS) != 0U) { return 0U; }
    YawController_Stop(yaw);
    /* 显式重新设零后保留参数、关闭加速度前馈，要求重新确认配置。
     * 后端普通OFF重启保留原软件坐标时会恢复原开关。 */
    if (yaw->ff.enabled) { yaw->ff.enabled = 0U; ++yaw->ff.revision; }
    yaw->reference_rad = yaw->continuous_rad;
    yaw->angle_deg = 0.0f;
    yaw->target_deg = 0.0f;
    yaw->shape.position_deg = 0.0f;
    yaw->have_reference = 1U;
    YawCascade_ResetObserver(&yaw->cascade);
    yaw->faults &= ~(uint32_t)YAW_RECOVERABLE_FAULTS;
    return 1U;
}

uint8_t YawController_SetPid(YawController *yaw, float kp, float ki, float kd)
{
    if (yaw == 0 || !yaw_controller_finite_value(kp) || !yaw_controller_finite_value(ki) || !yaw_controller_finite_value(kd) ||
        kp < 0.0f || kp > 2.0f || ki < 0.0f || ki > 1.0f || kd < 0.0f || kd > 0.2f) {
        return 0U;
    }
    yaw->pid.kp = kp; yaw->pid.ki = ki; yaw->pid.kd = kd;
    yaw->integral = 0.0f;
    yaw->torque_nm = 0.0f;
    yaw->have_pid_feedback = 0U;
    return 1U;
}

uint8_t YawController_SetTarget(YawController *yaw, float degrees)
{
    if (yaw == 0 || !yaw_controller_finite_value(degrees) ||
        degrees < -YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        degrees > YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        !yaw->have_reference || yaw->faults != 0U) { return 0U; }
    yaw->target_deg = degrees;
    yaw->shape.active = 0U;
    yaw->shape.position_deg = degrees;
    yaw->shape.velocity_dps = yaw->shape.acceleration_dps2 = 0.0f;
    return 1U;
}

uint8_t YawController_SetShape(YawController *yaw, uint32_t duration_ms)
{
#if APP_YAW_EXPERIMENTAL_TRACKING && YAW_CONTROLLER_CASCADE_ENABLED && !YAW_CASCADE_OUTPUT_LIMITS_ENABLED
    if (!yaw || yaw->shape.active || (duration_ms && (duration_ms < 80U || duration_ms > 500U))) { return 0U; }
    yaw->shape.duration_ms = duration_ms;
    return 1U;
#else
    (void)yaw; (void)duration_ms; return 0U;
#endif
}

uint8_t YawController_SetShapeVelocityGain(YawController *yaw, float gain)
{
#if APP_YAW_EXPERIMENTAL_TRACKING && YAW_CONTROLLER_CASCADE_ENABLED && !YAW_CASCADE_OUTPUT_LIMITS_ENABLED
    if (!yaw || yaw->shape.active || !yaw_controller_finite_value(gain) || gain < 0.0f || gain > 1.0f) { return 0U; }
    yaw->shape.velocity_gain = gain;
    return 1U;
#else
    (void)yaw; (void)gain; return 0U;
#endif
}

void YawController_StartShape(YawController *yaw, uint32_t now_ms)
{
    if (!APP_YAW_EXPERIMENTAL_TRACKING || !yaw || !yaw->shape.duration_ms || !yaw->active || !yaw->have_reference || yaw->faults) { return; }
    yaw->shape.start_ms = now_ms;
    yaw->shape.start_deg = yaw->shape.position_deg = yaw->angle_deg;
    yaw->shape.velocity_dps = yaw->shape.acceleration_dps2 = 0.0f;
    yaw->shape.active = 1U;
}

#if APP_YAW_EXPERIMENTAL_TRACKING && YAW_CONTROLLER_CASCADE_ENABLED
static void shape_update(YawController *yaw, uint32_t now_ms)
{
    uint32_t elapsed;
    float x, x2, x3, remain, span, seconds;
    if (!yaw->shape.active) {
        yaw->shape.position_deg = yaw->target_deg;
        yaw->shape.velocity_dps = yaw->shape.acceleration_dps2 = 0.0f;
        return;
    }
    elapsed = now_ms - yaw->shape.start_ms;
    if (elapsed >= yaw->shape.duration_ms) {
        yaw->shape.active = 0U;
        yaw->shape.position_deg = yaw->target_deg;
        yaw->shape.velocity_dps = yaw->shape.acceleration_dps2 = 0.0f;
        return;
    }
    x = (float)elapsed / (float)yaw->shape.duration_ms;
    x2 = x * x; x3 = x2 * x; remain = 1.0f - x;
    seconds = (float)yaw->shape.duration_ms * 0.001f;
    span = yaw->target_deg - yaw->shape.start_deg;
    yaw->shape.position_deg = yaw->shape.start_deg + span * x3 * (10.0f + x * (-15.0f + 6.0f * x));
    yaw->shape.velocity_dps = span * 30.0f * x2 * remain * remain / seconds;
    yaw->shape.acceleration_dps2 = span * 60.0f * x * remain * (1.0f - 2.0f * x) / (seconds * seconds);
}
#endif

uint8_t YawController_SetFeedforward(YawController *yaw, char parameter, float value)
{
#if APP_YAW_EXPERIMENTAL_TRACKING && YAW_CONTROLLER_CASCADE_ENABLED
    YawFeedforward next;
    float *field = 0;
    if (!yaw || !yaw_controller_finite_value(value)) { return 0U; }
    next = yaw->ff;
    if (parameter == 'A') { field = &next.acceleration_gain; }
    else if (parameter == 'L') { field = &next.limit_nm; }
    else if (parameter == 'E' && (value == 0.0f || value == 1.0f)) {
        if (next.enabled == (uint8_t)value) { return 1U; }
        next.enabled = (uint8_t)value;
    } else { return 0U; }
    if (field) { if (*field == value) { return 1U; } *field = value; }
    if (!valid_ff(&next) || (next.enabled && (!yaw->have_reference || yaw->faults))) { return 0U; }
    if (yaw->shape.active && !(parameter == 'E' && value == 0.0f)) { return 0U; }
    /* 前馈只存在于规划段；段外改参数不改积分，避免把制动脉冲留作保持力矩。 */
    ++next.revision; yaw->ff = next;
    return 1U;
#else
    if (yaw && parameter == 'E' && value == 0.0f) { yaw->ff.enabled = 0U; return 1U; }
    (void)yaw; (void)parameter; (void)value; return 0U;
#endif
}

float YawController_Update(YawController *yaw, float rate_dps,
                           uint32_t now_ms, uint8_t allowed)
{
    uint32_t elapsed, age;
#if !YAW_CONTROLLER_CASCADE_ENABLED
    float error, candidate, wanted, damping, brake;
#endif
    if (yaw == 0) { return 0.0f; }
    if (allowed) { yaw->motion_guard_active = 1U; }
    if (!yaw_controller_finite_value(rate_dps) || !valid_ff(&yaw->ff)) { fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f; }
    if (!yaw_controller_finite_value(yaw->shape.velocity_gain) || yaw->shape.velocity_gain < 0.0f || yaw->shape.velocity_gain > 1.0f) {
        fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f;
    }
    if (yaw->shape.active && (yaw->shape.duration_ms < 80U || yaw->shape.duration_ms > 500U ||
        !yaw_controller_finite_value(yaw->shape.start_deg))) {
        fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f;
    }
    age = now_ms - yaw->last_feedback_ms;
    if (!yaw->have_feedback || age >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS ||
        age >= APP_TICK_HALF_RANGE) {
        if (allowed || yaw->active) { fault(yaw, YAW_CONTROLLER_FAULT_FEEDBACK); }
        else { YawController_Stop(yaw); }
        return 0.0f;
    }
    if (!allowed || !yaw->have_reference || yaw->faults != 0U) {
        YawController_Stop(yaw);
        return 0.0f;
    }
    /* 放宽的观察样本不能直接进入闭环；许可转换时重查运行速度。 */
    if (YAW_CONTROLLER_SPEED_GUARD_ENABLED && yaw_controller_absolute(yaw->velocity_dps) > YAW_CONTROLLER_SPEED_LIMIT_DPS) {
        fault(yaw, YAW_CONTROLLER_FAULT_SPEED); return 0.0f;
    }
    if (yaw_controller_absolute(yaw->angle_deg) > YAW_CONTROLLER_HARD_LIMIT_DEG) {
        fault(yaw, YAW_CONTROLLER_FAULT_LIMIT); return 0.0f;
    }
    elapsed = yaw->have_control_tick ? now_ms - yaw->last_control_ms : 0U;
    yaw->last_control_ms = now_ms;
    yaw->have_control_tick = 1U;
    if (elapsed > YAW_CONTROLLER_CONTROL_MAX_GAP_MS) {
        fault(yaw, YAW_CONTROLLER_FAULT_TIMING); return 0.0f;
    }
    yaw->active = 1U;
    rate_dps = yaw_controller_clamp(rate_dps, -YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS,
                     YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS);
    yaw->target_deg = yaw_controller_clamp(yaw->target_deg + rate_dps * (float)elapsed / 1000.0f,
        -YAW_CONTROLLER_TARGET_LIMIT_DEG, YAW_CONTROLLER_TARGET_LIMIT_DEG);
#if YAW_CONTROLLER_CASCADE_ENABLED
    if (rate_dps != 0.0f) { yaw->shape.active = 0U; }
#if APP_YAW_EXPERIMENTAL_TRACKING
    shape_update(yaw, now_ms);
    if (yaw->shape.active) {
        YawCascadeReference reference;
        YawCascadeFeedforward model;
        float wanted_ff = yaw->ff.enabled ? yaw->ff.acceleration_gain * yaw->shape.acceleration_dps2 : 0.0f;
        reference.velocity_dps = yaw->shape.velocity_gain * yaw->shape.velocity_dps;
        reference.freeze_position_i = 1U;
        model.torque_nm = yaw_controller_clamp(wanted_ff, -yaw->ff.limit_nm, yaw->ff.limit_nm);
        model.limited = (uint8_t)(wanted_ff > yaw->ff.limit_nm || wanted_ff < -yaw->ff.limit_nm);
        yaw->torque_nm = YawCascade_UpdateTracking(&yaw->cascade, yaw->shape.position_deg - yaw->angle_deg,
            yaw->last_feedback_ms, yaw->last_sequence, YAW_CONTROLLER_TORQUE_LIMIT_NM,
            &model, &reference);
    } else {
#else
    yaw->shape.position_deg = yaw->target_deg;
    yaw->shape.active = 0U;
    yaw->shape.velocity_dps = yaw->shape.acceleration_dps2 = 0.0f;
    {
#endif
        yaw->torque_nm = YawCascade_Update(&yaw->cascade, yaw->target_deg - yaw->angle_deg,
            yaw->last_feedback_ms, yaw->last_sequence, YAW_CONTROLLER_TORQUE_LIMIT_NM);
    }
    yaw->pid_torque_nm = yaw->cascade.unsaturated_torque_nm - yaw->cascade.feedforward_nm;
    if (!yaw_controller_finite_value(yaw->torque_nm) || !yaw_controller_finite_value(yaw->pid_torque_nm) ||
        !yaw_controller_finite_value(yaw->cascade.position_i) || !yaw_controller_finite_value(yaw->cascade.speed_i) ||
        !yaw_controller_finite_value(yaw->cascade.target_velocity_dps)) { fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f; }
    return yaw->torque_nm;
#else
    error = (yaw->target_deg - yaw->angle_deg) * YAW_DEG_TO_RAD;
    damping = yaw->pid.kd * yaw->velocity_dps * YAW_DEG_TO_RAD;
    yaw->speed_limited = (uint8_t)(YAW_CONTROLLER_SPEED_GUARD_ENABLED &&
        yaw_controller_absolute(yaw->velocity_dps) > YAW_CONTROLLER_SOFT_SPEED_DPS);
    if (yaw->pid.ki == 0.0f) { yaw->integral = 0.0f; }
    else if (!yaw->speed_limited && yaw->have_pid_feedback && yaw->pid_sequence != yaw->last_sequence) {
        elapsed = yaw->last_feedback_ms - yaw->pid_feedback_ms;
        if (elapsed <= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
            candidate = yaw_controller_clamp(yaw->integral + error * (float)elapsed / 1000.0f,
                -YAW_CONTROLLER_TORQUE_LIMIT_NM / yaw->pid.ki,
                 YAW_CONTROLLER_TORQUE_LIMIT_NM / yaw->pid.ki);
            wanted = yaw->pid.kp * error + yaw->pid.ki * candidate - damping;
            if (!((wanted > YAW_CONTROLLER_TORQUE_LIMIT_NM && error > 0.0f) ||
                  (wanted < -YAW_CONTROLLER_TORQUE_LIMIT_NM && error < 0.0f))) {
                yaw->integral = candidate;
            }
        }
    }
    yaw->pid_feedback_ms = yaw->last_feedback_ms;
    yaw->pid_sequence = yaw->last_sequence;
    yaw->have_pid_feedback = 1U;
    wanted = yaw->pid.kp * error + yaw->pid.ki * yaw->integral - damping;
    if (YAW_CONTROLLER_SPEED_GUARD_ENABLED && yaw->velocity_dps > YAW_CONTROLLER_SOFT_SPEED_DPS) {
        brake = -(yaw->velocity_dps - YAW_CONTROLLER_SOFT_SPEED_DPS) * YAW_DEG_TO_RAD * YAW_CONTROLLER_SPEED_BRAKE_KD;
        if (wanted > brake) { wanted = brake; }
    } else if (YAW_CONTROLLER_SPEED_GUARD_ENABLED && yaw->velocity_dps < -YAW_CONTROLLER_SOFT_SPEED_DPS) {
        brake = -(yaw->velocity_dps + YAW_CONTROLLER_SOFT_SPEED_DPS) * YAW_DEG_TO_RAD * YAW_CONTROLLER_SPEED_BRAKE_KD;
        if (wanted < brake) { wanted = brake; }
    }
    yaw->torque_nm = yaw_controller_clamp(wanted, -YAW_CONTROLLER_TORQUE_LIMIT_NM,
                           YAW_CONTROLLER_TORQUE_LIMIT_NM);
    return yaw->torque_nm;
#endif
}
/* APP_SECTION_END yaw_controller.c */

/* APP_SECTION_BEGIN yaw_step.c */
/* 共用真实反馈阶跃统计；不依赖显示演示。 */
#include "yaw_step.h"
#include <float.h>
#include <string.h>
static float yaw_step_absolute(float value) { return value < 0.0f ? -value : value; }
static uint8_t yaw_step_finite_value(float value)
{ return (uint8_t)(value == value && value >= -FLT_MAX && value <= FLT_MAX); }
void YawStep_Init(YawStep *step)
{
    if (!step) { return; }
    memset(step, 0, sizeof(*step));
    step->t10_ms = step->t90_ms = step->response_ms = step->settle_ms = YAW_STEP_UNREACHED;
}
uint8_t YawStep_Start(YawStep *step, uint32_t now_ms, float start_deg, float target_deg)
{
    uint32_t id;
    if (!step || step->phase == YAW_STEP_RUNNING || !yaw_step_finite_value(start_deg) ||
        !yaw_step_finite_value(target_deg) || yaw_step_absolute(target_deg - start_deg) < 1.0f ||
        yaw_step_absolute(target_deg - start_deg) > 180.0f) { return 0U; }
    id = step->id + 1U; YawStep_Init(step); step->id = id;
    step->start_ms = step->last_ms = now_ms;
    step->start_deg = step->last_angle_deg = start_deg; step->target_deg = target_deg;
    step->span_deg = yaw_step_absolute(target_deg - start_deg);
    step->direction = target_deg > start_deg ? 1.0f : -1.0f;
    step->phase = YAW_STEP_RUNNING;
    return 1U;
}
void YawStep_Command(YawStep *step, uint32_t now_ms, float torque_nm)
{
    if (!step || step->phase != YAW_STEP_RUNNING ||
        (uint32_t)(now_ms - step->start_ms) >= YAW_STEP_WINDOW_MS ||
        !yaw_step_finite_value(torque_nm) || yaw_step_absolute(torque_nm) > 10.0f) { return; }
    if (!step->command_samples) { step->first_command_nm = torque_nm; }
    ++step->command_samples;
    if (yaw_step_absolute(torque_nm) > step->peak_torque_nm) { step->peak_torque_nm = yaw_step_absolute(torque_nm); }
}

void YawStep_Abort(YawStep *step, uint32_t now_ms, YawStepReason reason, uint32_t faults)
{
    if (!step || step->phase != YAW_STEP_RUNNING) { return; }
    step->phase = YAW_STEP_ABORTED; step->reason = (uint8_t)reason;
    step->faults = faults; step->elapsed_ms = now_ms - step->start_ms;
    step->pending = 1U; step->sse_valid = step->pass = 0U;
}
uint8_t YawStep_Passed(const YawStep *step, uint32_t limit_ms)
{
    return (uint8_t)(step && step->phase == YAW_STEP_DONE && !step->faults &&
        yaw_step_absolute(step->span_deg - 30.0f) <= YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG &&
        step->settle_ms != YAW_STEP_UNREACHED && step->settle_ms <= limit_ms &&
        step->overshoot_deg <= YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG &&
        step->sse_valid && yaw_step_absolute(step->steady_error_deg) <= YAW_STEP_SSE_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG);
}
void YawStep_Sample(YawStep *step, uint32_t received_ms, float angle_deg,
                    float velocity_dps, float torque_nm, uint8_t limited)
{
    uint32_t gap, elapsed, previous, end, low, duration;
    float angle, velocity, torque, progress, overshoot, left, fraction;
    if (!step || step->phase != YAW_STEP_RUNNING) { return; }
    gap = received_ms - step->last_ms;
    if (!gap || gap >= 0x80000000UL) { return; }
    if (gap >= 100U) { YawStep_Abort(step, received_ms, YAW_STEP_GAP, 0U); return; }
    if (!yaw_step_finite_value(angle_deg) || !yaw_step_finite_value(velocity_dps) || !yaw_step_finite_value(torque_nm) ||
        yaw_step_absolute(angle_deg) > 1000.0f || yaw_step_absolute(velocity_dps) > 20000.0f ||
        yaw_step_absolute(torque_nm) > 10.0f) {
        YawStep_Abort(step, received_ms, YAW_STEP_DATA, 0U); return;
    }
    elapsed = received_ms - step->start_ms; previous = step->last_ms - step->start_ms;
    end = elapsed > YAW_STEP_WINDOW_MS ? YAW_STEP_WINDOW_MS : elapsed;
    fraction = (float)(end - previous) / (float)gap;
    angle = step->last_angle_deg + (angle_deg - step->last_angle_deg) * fraction;
    velocity = step->last_velocity_dps + (velocity_dps - step->last_velocity_dps) * fraction;
    torque = step->last_torque_nm + (torque_nm - step->last_torque_nm) * fraction;
    if (gap > step->max_gap_ms) { step->max_gap_ms = gap; }
    ++step->samples; if (limited) { ++step->limited_samples; }
    if (yaw_step_absolute(velocity) > step->peak_velocity_dps) { step->peak_velocity_dps = yaw_step_absolute(velocity); }
    if (yaw_step_absolute(torque) > step->peak_torque_nm) { step->peak_torque_nm = yaw_step_absolute(torque); }
    progress = (angle - step->start_deg) * step->direction;
    if (step->t10_ms == YAW_STEP_UNREACHED && progress >= step->span_deg * 0.1f) { step->t10_ms = end; }
    if (step->t90_ms == YAW_STEP_UNREACHED && progress >= step->span_deg * 0.9f) { step->t90_ms = end; }
    overshoot = progress - step->span_deg;
    if (overshoot > step->overshoot_deg) { step->overshoot_deg = overshoot; }
    step->overshoot_pct = step->overshoot_deg * 100.0f / step->span_deg;
    if (yaw_step_absolute(step->target_deg - angle) <= YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG) {
        if (!step->band_active) { step->band_active = 1U; step->band_since_ms = end; }
        if (step->response_ms == YAW_STEP_UNREACHED) {
            step->response_ms = end;
        }
    } else { step->band_active = 0U; }
    low = previous > YAW_STEP_WINDOW_MS - YAW_STEP_STEADY_MS ?
        previous : YAW_STEP_WINDOW_MS - YAW_STEP_STEADY_MS;
    if (end > low) {
        left = step->last_angle_deg + (angle - step->last_angle_deg) *
            (float)(low - previous) / (float)(end - previous);
        duration = end - low;
        step->ss_integral += (left + angle) * 0.5f * (float)duration;
        step->ss_duration_ms += duration; ++step->ss_samples;
        if (yaw_step_absolute(step->target_deg - left) > step->ss_max_error) { step->ss_max_error = yaw_step_absolute(step->target_deg - left); }
        if (yaw_step_absolute(step->target_deg - angle) > step->ss_max_error) { step->ss_max_error = yaw_step_absolute(step->target_deg - angle); }
        step->steady_error_deg = step->target_deg - step->ss_integral / (float)step->ss_duration_ms;
    }
    step->last_ms = received_ms; step->last_angle_deg = angle_deg;
    step->last_velocity_dps = velocity_dps; step->last_torque_nm = torque_nm; step->elapsed_ms = end;
    if (end == YAW_STEP_WINDOW_MS) {
        step->phase = YAW_STEP_DONE; step->pending = 1U;
        if (step->band_active && end - step->band_since_ms >= YAW_STEP_SETTLE_HOLD_MS) {
            step->settle_ms = step->band_since_ms;
        }
        step->sse_valid = (uint8_t)(step->ss_duration_ms == YAW_STEP_STEADY_MS &&
            step->ss_samples >= 10U && step->ss_max_error <= YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG);
        step->pass = YawStep_Passed(step, YAW_STEP_RESPONSE_LIMIT_MS);
    }
}
/* APP_SECTION_END yaw_step.c */

/* APP_SECTION_BEGIN yaw_backend.c */
/* Yaw 会话：许可、握手、相对阶跃、归位与锁轴。 */
#include "yaw_backend.h"
#include "app_config.h"
#include "can_bus.h"
#include "task.h"
#include <string.h>
#include <math.h>

static YawBackendState state;
static YawStep step;
/* Preserve the latest interrupted/unread report when starting a new step. */
static YawStep previous_step;
static YawRemoteStepEvent remote_step_event;
static uint32_t last_step_sequence;
static uint8_t remote_event_pending, step_from_remote;
static GimbalMode last_mode;
static uint8_t monitoring, cancel_pending, sent, rearm_request;
static uint8_t preparation_hold;
static float last_submitted_torque_nm;
static uint8_t last_submitted_limited;
static uint32_t last_tx_ms, enable_ms, disable_ms;
static uint8_t clear_waiting_disable, clear_stationary_samples;
static uint32_t clear_stationary_since;
static uint32_t last_clear_blocks;
static uint8_t last_clear_result, last_disable_result;
static uint32_t off_motion_faults;
/* FC尝试或实际状态1即置位，STOP/OFF不清；健康失能静止参考捕获后开新会话。 */
static uint8_t actuation_started;

static uint8_t Online(uint32_t now)
{
    CanBusState bus;
    CanBus_GetState(&bus);
    return state.controller.have_feedback &&
        (uint32_t)(now - state.feedback.received_ms) < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        state.feedback.fault_epoch == bus.fault_epoch;
}

static uint32_t StepBlocks(uint32_t now)
{
    uint32_t blocks = 0U;
    if (!state.zero_set || !state.controller.have_reference) { blocks |= YAW_STEP_BLOCK_REFERENCE; }
    if (!state.requested || !state.active || !state.controller.active) { blocks |= YAW_STEP_BLOCK_INACTIVE; }
    if (state.enabling || state.stop_pending || clear_waiting_disable) { blocks |= YAW_STEP_BLOCK_TRANSITION; }
    if (!Online(now) || state.feedback.status != 1U) { blocks |= YAW_STEP_BLOCK_FEEDBACK; }
    if (state.controller.faults) { blocks |= YAW_STEP_BLOCK_FAULT; }
    if (!CanBus_YawAllowed()) { blocks |= YAW_STEP_BLOCK_PERMISSION; }
    if (fabsf(state.controller.angle_deg - state.controller.target_deg) >
        YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG) { blocks |= YAW_STEP_BLOCK_POSITION; }
#if YAW_CONTROLLER_CASCADE_ENABLED
    if (fabsf(state.controller.cascade.velocity_dps) > YAW_CONTROLLER_STATIONARY_SPEED_DPS ||
        fabsf(state.controller.velocity_dps) > YAW_STEP_RAW_SPEED_SANITY_DPS) { blocks |= YAW_STEP_BLOCK_MOVING; }
    if (state.step_still_samples < 2U || state.controller.last_feedback_ms - state.step_still_since_ms <
        YAW_CONTROLLER_STATIONARY_HOLD_MS) { blocks |= YAW_STEP_BLOCK_STILL; }
#else
    if (fabsf(state.controller.velocity_dps) > YAW_CONTROLLER_STATIONARY_SPEED_DPS) { blocks |= YAW_STEP_BLOCK_MOVING; }
    if (state.controller.stationary_samples < 2U || state.controller.last_feedback_ms - state.controller.stationary_started_ms <
        YAW_CONTROLLER_STATIONARY_HOLD_MS) { blocks |= YAW_STEP_BLOCK_STILL; }
#endif
    if (state.view_locked) { blocks |= YAW_STEP_BLOCK_VIEW; }
    return blocks;
}

static uint8_t CaptureForStart(void)
{
    float reference = state.controller.reference_rad;
    float edge = YAW_CONTROLLER_TARGET_LIMIT_DEG;
    uint8_t keep = state.zero_set && state.controller.have_reference;
    uint8_t ff_enabled = state.controller.ff.enabled;
    uint32_t ff_revision = state.controller.ff.revision;
#if YAW_CONTROLLER_CASCADE_ENABLED
    /* 端点反馈允许原到位误差带，避免目标限位附近的量化/微小静差阻断重启。
     * 目标仍裁到配置的限位；更远的位置继续要求人工处理，不沿旧目标使能。 */
    edge += YAW_STEP_BAND_DEG;
#endif
    if (keep && fabsf(state.controller.angle_deg) > edge) {
        state.controller.faults |= YAW_CONTROLLER_FAULT_LIMIT; return 0U;
    }
    if (!YawController_CaptureReference(&state.controller)) { return 0U; }
    if (keep) {
        state.controller.ff.enabled = ff_enabled;
        state.controller.ff.revision = ff_revision;
        state.controller.reference_rad = reference;
        state.controller.angle_deg = (state.controller.continuous_rad - reference) * 57.2957795f;
        /* 保留人工零点，但使能时先保持当前位置，不自行追赶旧零点。 */
        state.controller.target_deg = state.controller.angle_deg;
        if (state.controller.target_deg > YAW_CONTROLLER_TARGET_LIMIT_DEG) {
            state.controller.target_deg = YAW_CONTROLLER_TARGET_LIMIT_DEG;
        } else if (state.controller.target_deg < -YAW_CONTROLLER_TARGET_LIMIT_DEG) {
            state.controller.target_deg = -YAW_CONTROLLER_TARGET_LIMIT_DEG;
        }
    }
    return 1U;
}

void YawBackend_Init(void)
{
    taskENTER_CRITICAL();
    memset(&state, 0, sizeof(state));
    YawController_Init(&state.controller);
    YawStep_Init(&step);
    YawStep_Init(&previous_step);
    memset(&remote_step_event, 0, sizeof(remote_step_event));
    last_step_sequence = 0U; remote_event_pending = step_from_remote = 0U;
    last_mode = GIMBAL_MANUAL;
    state.controller.motion_guard_active = 0U;
    monitoring = cancel_pending = sent = rearm_request = 0U;
    preparation_hold = 0U;
    last_submitted_torque_nm = 0.0f;
    last_submitted_limited = 0U;
    last_tx_ms = enable_ms = disable_ms = 0U;
    clear_waiting_disable = clear_stationary_samples = 0U;
    clear_stationary_since = 0U;
    last_clear_blocks = 0U; last_clear_result = 4U; /* 4=尚未尝试发送FB */
    last_disable_result = (uint8_t)HAL_ERROR;
    off_motion_faults = 0U; actuation_started = 0U;
    taskEXIT_CRITICAL();
}

void YawBackend_GetState(YawBackendState *out)
{
    if (!out) { return; }
    taskENTER_CRITICAL();
    state.online = Online(HAL_GetTick());
    state.feedback_age_ms = state.controller.have_feedback ?
        (uint32_t)(HAL_GetTick() - state.feedback.received_ms) : 0U;
    state.step_blocks = StepBlocks(HAL_GetTick());
    state.step_still_ms = state.step_still_samples ? state.controller.last_feedback_ms - state.step_still_since_ms : 0U;
    state.step_window_left_ms = step.phase == YAW_STEP_RUNNING &&
        (uint32_t)(HAL_GetTick() - step.start_ms) < YAW_STEP_WINDOW_MS ?
        YAW_STEP_WINDOW_MS - (uint32_t)(HAL_GetTick() - step.start_ms) : 0U;
    *out = state;
    taskEXIT_CRITICAL();
}

void YawBackend_Stop(void)
{
    taskENTER_CRITICAL();
    YawStep_Abort(&step, HAL_GetTick(), state.controller.faults ? YAW_STEP_FAULT : YAW_STEP_STOP,
                  state.controller.faults);
    state.step_hold = 0U;
    state.view_hold_pending = state.view_locked;
    last_mode = GIMBAL_MANUAL;
    state.step_still_samples = 0U;
    step_from_remote = 0U;
    CanBus_SetYawPermission(0U);
    YawController_Stop(&state.controller);
    state.active = state.enabling = 0U;
    clear_waiting_disable = 0U; /* 新STOP取消待完成的人工恢复，不续发清错。 */
    if (monitoring) {
        state.stop_pending = 1U; cancel_pending = 1U; sent = 0U;
        disable_ms = HAL_GetTick();
    }
    taskEXIT_CRITICAL();
}

uint8_t YawBackend_PrepareStop(void)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (APP_YAW_ENABLED && bus.started && !bus.fault_epoch) {
        monitoring = 1U; state.requested = 0U; state.zero_set = 0U;
        YawBackend_Stop(); preparation_hold = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
}

void YawBackend_ReleasePreparation(void)
{
    taskENTER_CRITICAL(); preparation_hold = 0U; taskEXIT_CRITICAL();
}

uint8_t YawBackend_Request(uint8_t enabled)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (!enabled) {
        preparation_hold = 0U;
        state.requested = 0U;
        YawBackend_Stop(); accepted = 1U;
    } else if (APP_YAW_ENABLED && bus.started && !bus.fault_epoch && !state.active && !state.enabling &&
               !clear_waiting_disable) {
        monitoring = 1U;
        /* 不通过ON清除未知故障；新鲜静止参考捕获才可清控制器故障。 */
        if (!state.requested && Online(HAL_GetTick()) && state.feedback.status == 0U) {
            uint32_t previous_faults = state.controller.faults;
            /* 人工OFF后，失能且静止可恢复使能超时/意外失能；TX/CAN仍须复位检查。 */
            state.controller.faults &= ~(YAW_BACKEND_FAULT_ENABLE | YAW_BACKEND_FAULT_DISABLED);
            if (!CaptureForStart()) { state.controller.faults |= previous_faults; }
            else {
                off_motion_faults = 0U; actuation_started = 0U;
                state.controller.motion_guard_active = 0U;
            }
        }
        if (!(state.controller.faults & (YAW_BACKEND_FAULT_ENABLE |
              YAW_BACKEND_FAULT_TX | YAW_BACKEND_FAULT_DISABLED))) {
            state.requested = 1U;
            YawBackend_Stop(); accepted = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_OnFrame(const CanFrame *frame)
{
    Gl60Feedback feedback;
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    if (!Gl60_Decode(frame, &feedback)) { return 0U; }
    taskENTER_CRITICAL();
    /* 接收时刻由ISR提供，旧队列帧不允许延长反馈寿命。 */
    if ((uint32_t)(now - feedback.received_ms) < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
        (!state.controller.have_feedback ||
         (int32_t)(feedback.received_ms - state.feedback.received_ms) > 0)) {
        float delta = feedback.position_rad - state.feedback.position_rad;
        uint32_t gap = feedback.received_ms - state.feedback.received_ms;
        uint32_t previous_faults = state.controller.faults;
        float previous_torque = last_submitted_torque_nm;
        uint8_t previous_limited = last_submitted_limited;
        if (feedback.status == 1U) { actuation_started = 1U; }
        uint8_t off_motion = (uint8_t)(monitoring && !state.requested && !state.active &&
            !state.enabling && !state.controller.active && !actuation_started &&
            !CanBus_YawAllowed() && state.feedback.status == 13U && feedback.status == 13U);
        if (delta > GL60_POSITION_MAX_RAD) { delta -= 2.0f * GL60_POSITION_MAX_RAD; }
        else if (delta < -GL60_POSITION_MAX_RAD) { delta += 2.0f * GL60_POSITION_MAX_RAD; }
        /* 错误码13覆盖正常状态位，恢复用独立的连续静止观察；仍不视为失能确认。 */
        if (feedback.status == 13U && feedback.driver_temp_c < YAW_CONTROLLER_DRIVER_LIMIT_C &&
            feedback.motor_temp_c < YAW_CONTROLLER_MOTOR_LIMIT_C &&
            fabsf(feedback.velocity_rad_s * 57.2957795f) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS) {
            if (!clear_stationary_samples || state.feedback.status != 13U ||
                gap >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS ||
                fabsf(delta * 57.2957795f) > 0.05f + 0.5f * (float)gap / 1000.0f) {
                clear_stationary_since = feedback.received_ms; clear_stationary_samples = 1U;
            } else if (clear_stationary_samples < 255U) { clear_stationary_samples++; }
        } else { clear_stationary_samples = 0U; }
        state.feedback = feedback; state.rx_count++;
        (void)YawController_OnFeedback(&state.controller, &feedback);
        /* 重复STEP使用连续编码器静止观察；不改失能设零/清错的原始速度门禁。
         * 原速度近零有量化和抖动，仅作为较宽的一致性检查，不能单独触发。 */
        if (state.rx_count > 1U && gap && gap < YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS &&
            !state.controller.faults && feedback.status <= 1U &&
            fabsf(state.controller.cascade.velocity_dps) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS &&
            fabsf(state.controller.velocity_dps) <= YAW_STEP_RAW_SPEED_SANITY_DPS &&
            fabsf(delta * 57.2957795f) <= 0.05f + 0.5f * (float)gap / 1000.0f) {
            if (!state.step_still_samples) { state.step_still_since_ms = feedback.received_ms; }
            if (state.step_still_samples < 255U) { ++state.step_still_samples; }
        } else { state.step_still_samples = 0U; }
        if (!state.controller.have_reference) { state.zero_set = 0U; }
        /* 仅标记新发生且来源明确的关闭状态运动记录。
         * 先去掉已被健康参考捕获清除的旧标签，不能把后来的运行故障误标。 */
        off_motion_faults &= previous_faults;
        if (off_motion) {
            off_motion_faults |= (state.controller.faults & ~previous_faults & YAW_OFF_MOTION_FAULTS);
        }
        accepted = 1U;
        if (state.stop_pending && !clear_waiting_disable && feedback.status == 0U &&
            (int32_t)(feedback.received_ms - disable_ms) > 0 &&
            fabsf(feedback.velocity_rad_s * 57.2957795f) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS) {
            state.stop_pending = 0U;
        }
        if (state.active && feedback.status == 0U) {
            state.controller.faults |= YAW_BACKEND_FAULT_DISABLED;
        }
        if (step.phase == YAW_STEP_RUNNING) {
            if (state.controller.faults) {
                float velocity = fabsf(feedback.velocity_rad_s * 57.2957795f);
                if (velocity > step.peak_velocity_dps) { step.peak_velocity_dps = velocity; }
                YawStep_Abort(&step, now, YAW_STEP_FAULT, state.controller.faults);
            } else if (state.active && feedback.status == 1U && CanBus_YawAllowed()) {
                YawStep_Sample(&step, feedback.received_ms, state.controller.angle_deg,
                    state.controller.velocity_dps, previous_torque, previous_limited);
            } else { YawStep_Abort(&step, now, YAW_STEP_STOP, 0U); }
        }
        if (state.controller.faults && (state.active || state.enabling)) {
            YawBackend_Stop(); rearm_request = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetParameter(char parameter, float value)
{
    uint8_t accepted = 0U;
    if (YAW_CONTROLLER_CASCADE_ENABLED && parameter != 'T') { return 0U; }
    taskENTER_CRITICAL();
    if (parameter == 'P') {
        accepted = YawController_SetPid(&state.controller, value, state.controller.pid.ki, state.controller.pid.kd);
    } else if (parameter == 'I') {
        accepted = YawController_SetPid(&state.controller, state.controller.pid.kp, value, state.controller.pid.kd);
    } else if (parameter == 'D') {
        accepted = YawController_SetPid(&state.controller, state.controller.pid.kp, state.controller.pid.ki, value);
    } else if (parameter == 'T' && !state.view_locked && state.active && Online(HAL_GetTick()) && CanBus_YawAllowed()) {
        accepted = YawController_SetTarget(&state.controller, value);
    }
    if (accepted) {
        YawStep_Abort(&step, HAL_GetTick(), parameter == 'T' ? YAW_STEP_TARGET_CHANGE : YAW_STEP_PID_CHANGE, 0U);
        if (parameter == 'T') { state.step_hold = 0U; step_from_remote = 0U; }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetPid(float kp, float ki, float kd)
{
#if YAW_CONTROLLER_CASCADE_ENABLED
    (void)kp; (void)ki; (void)kd;
    return 0U;
#else
    uint8_t accepted;
    taskENTER_CRITICAL();
    accepted = YawController_SetPid(&state.controller, kp, ki, kd);
    if (accepted) { YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_PID_CHANGE, 0U); }
    taskEXIT_CRITICAL(); return accepted;
#endif
}

uint8_t YawBackend_SetLoopPid(char loop, float kp, float ki, float kd)
{
    uint8_t accepted;
    uint32_t revision;
    taskENTER_CRITICAL();
    if (state.controller.shape.active) { taskEXIT_CRITICAL(); return 0U; }
    revision = state.controller.cascade.revision;
    accepted = YawCascade_SetPid(&state.controller.cascade, loop, kp, ki, kd);
    if (accepted && revision != state.controller.cascade.revision) {
        state.controller.torque_nm = 0.0f;
        YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_PID_CHANGE, 0U);
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t YawBackend_SetLoopParameter(char loop, char parameter, float value)
{
    uint8_t accepted = 0U;
    YawCascadePid pid;
    taskENTER_CRITICAL();
    if (loop == 'P' || loop == 'V') {
        pid = loop == 'P' ? state.controller.cascade.position : state.controller.cascade.speed;
        if (parameter == 'P') { pid.kp = value; }
        else if (parameter == 'I') { pid.ki = value; }
        else if (parameter == 'D') { pid.kd = value; }
        if (parameter == 'P' || parameter == 'I' || parameter == 'D') {
            accepted = YawBackend_SetLoopPid(loop, pid.kp, pid.ki, pid.kd);
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint8_t YawBackend_SetVelocityLimit(float dps)
{
    uint8_t accepted;
    uint32_t revision;
    taskENTER_CRITICAL();
    revision = state.controller.cascade.revision;
    accepted = YawCascade_SetVelocityLimit(&state.controller.cascade, dps);
    if (accepted && revision != state.controller.cascade.revision) {
        state.controller.torque_nm = 0.0f;
        YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_PID_CHANGE, 0U);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetFeedforward(char parameter, float value)
{
    uint8_t accepted = 0U, off_allowed, active_allowed;
    uint32_t revision;
    taskENTER_CRITICAL();
    /* 五秒窗口内保持试验条件；显式关闭仍接受，并使本轮结果无效。 */
    if (step.phase == YAW_STEP_RUNNING && !(parameter == 'E' && value == 0.0f)) {
        taskEXIT_CRITICAL(); return 0U;
    }
    off_allowed = (uint8_t)(!state.active && !state.enabling && !state.stop_pending);
    active_allowed = (uint8_t)(state.active && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 1U && !state.controller.faults && CanBus_YawAllowed());
    if ((parameter == 'E' && value == 0.0f) || off_allowed || active_allowed) {
        uint8_t can_enable = (uint8_t)(state.zero_set && state.controller.have_reference &&
            Online(HAL_GetTick()) && !state.controller.faults &&
            ((off_allowed && state.feedback.status == 0U) || active_allowed));
        if (!(parameter == 'E' && value == 1.0f) || can_enable) {
            revision = state.controller.ff.revision;
            accepted = YawController_SetFeedforward(&state.controller, parameter, value);
            if (accepted && revision != state.controller.ff.revision) {
                YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_FF_CHANGE, 0U);
            }
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetShape(uint32_t duration_ms)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (duration_ms == state.controller.shape.duration_ms) { accepted = 1U; }
    else if (step.phase != YAW_STEP_RUNNING) {
        accepted = YawController_SetShape(&state.controller, duration_ms);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_CaptureReference(void)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (!state.requested && !state.active && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 0U) {
        accepted = YawController_CaptureReference(&state.controller);
        if (accepted) {
            state.zero_set = 1U;
            off_motion_faults = 0U; actuation_started = 0U;
            state.controller.motion_guard_active = 0U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_SetShapeVelocityGain(float gain)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    if (gain == state.controller.shape.velocity_gain) { accepted = 1U; }
    else if (step.phase != YAW_STEP_RUNNING) {
        accepted = YawController_SetShapeVelocityGain(&state.controller, gain);
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint32_t YawBackend_StepBlocks(float delta_deg)
{
    uint32_t blocks;
    float target;
    taskENTER_CRITICAL();
    blocks = StepBlocks(HAL_GetTick());
    target = state.controller.target_deg + delta_deg;
    if ((delta_deg != 30.0f && delta_deg != -30.0f) || target != target ||
        target < -YAW_CONTROLLER_TARGET_LIMIT_DEG || target > YAW_CONTROLLER_TARGET_LIMIT_DEG) {
        blocks |= YAW_STEP_BLOCK_TARGET_LIMIT;
    }
    taskEXIT_CRITICAL(); return blocks;
}

uint8_t YawBackend_StartStep(float degrees)
{
    uint8_t accepted = 0U;
    float target;
    taskENTER_CRITICAL();
    target = state.controller.target_deg + degrees;
    if (!YawBackend_StepBlocks(degrees) && YawController_SetTarget(&state.controller, target)) {
        YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_TARGET_CHANGE, 0U);
        if (step.pending) { previous_step = step; }
        accepted = YawStep_Start(&step, HAL_GetTick(), state.controller.angle_deg, target);
        if (accepted) {
            YawController_StartShape(&state.controller, step.start_ms);
            step.shape_ms = state.controller.shape.active ? state.controller.shape.duration_ms : 0U;
            step.shape_velocity_gain = state.controller.shape.active ? state.controller.shape.velocity_gain : 0.0f;
#if YAW_CONTROLLER_CASCADE_ENABLED
            step.kp = state.controller.cascade.position.kp;
            step.ki = state.controller.cascade.position.ki;
            step.kd = state.controller.cascade.position.kd;
            step.vkp = state.controller.cascade.speed.kp;
            step.vki = state.controller.cascade.speed.ki;
            step.vkd = state.controller.cascade.speed.kd;
            step.vmax_dps = state.controller.cascade.velocity_limit_dps;
            step.cascade = 1U;
            step.output_limits = YAW_CASCADE_OUTPUT_LIMITS_ENABLED;
            step.ff_enabled = state.controller.ff.enabled;
            step.ff_acceleration_gain = state.controller.ff.acceleration_gain;
            step.ff_limit_nm = state.controller.ff.limit_nm;
#else
            step.kp = state.controller.pid.kp; step.ki = state.controller.pid.ki; step.kd = state.controller.pid.kd;
#endif
            step.last_velocity_dps = state.controller.velocity_dps;
            step.last_torque_nm = last_submitted_torque_nm;
            state.step_hold = 1U;
            step_from_remote = 0U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}
uint32_t YawBackend_GetStepId(void)
{
    uint32_t id;
    taskENTER_CRITICAL(); id = step.id; taskEXIT_CRITICAL();
    return id;
}
void YawBackend_GetStep(YawStep *out)
{
    if (!out) { return; }
    taskENTER_CRITICAL(); *out = step; taskEXIT_CRITICAL();
}
uint8_t YawBackend_TakeStepReport(YawStep *out)
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (previous_step.pending) { *out = previous_step; previous_step.pending = 0U; ready = 1U; }
    else if (step.pending) { *out = step; step.pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}
uint8_t YawBackend_TakeRemoteStepEvent(YawRemoteStepEvent *out)
{
    uint8_t ready = 0U;
    if (!out) { return 0U; }
    taskENTER_CRITICAL();
    if (remote_event_pending) { *out = remote_step_event; remote_event_pending = 0U; ready = 1U; }
    taskEXIT_CRITICAL(); return ready;
}
static void RemoteNotice(const TargetRequest *request, uint8_t result)
{
    remote_step_event.sequence = request->step_sequence;
    remote_step_event.target_deg = result == 1U || result == 2U ? state.controller.target_deg :
        state.controller.target_deg + request->step_delta_deg;
    remote_step_event.result = result; remote_event_pending = 1U;
    remote_step_event.blocks = result == 3U ? YawBackend_StepBlocks(request->step_delta_deg) : 0U;
    if (result == 3U && !remote_step_event.blocks) { remote_step_event.blocks = YAW_STEP_BLOCK_PERMISSION; }
}

uint8_t YawBackend_TakeRearmRequest(void)
{
    uint8_t out;
    taskENTER_CRITICAL(); out = rearm_request; rearm_request = 0U; taskEXIT_CRITICAL();
    return out;
}

void YawBackend_GetClearCheck(YawClearCheck *out)
{
    uint32_t now = HAL_GetTick();
    CanBusState bus;
    if (!out) { return; }
    taskENTER_CRITICAL();
    memset(out, 0, sizeof(*out)); CanBus_GetState(&bus);
    out->hold_ms = clear_stationary_samples ?
        (uint32_t)(state.feedback.received_ms - clear_stationary_since) : 0U;
    out->fd_age_ms = sent ? (uint32_t)(now - last_tx_ms) : 0U;
    out->mailbox_pending = CanBus_YawTxPending(); out->can_epoch = bus.fault_epoch;
    out->samples = clear_stationary_samples; out->monitoring = monitoring;
    out->fd_sent = sent; out->fd_cancel = cancel_pending;
    out->fd_result = last_disable_result; out->clear_result = last_clear_result;
    out->last_blocks = last_clear_blocks;
    out->off_motion_faults = off_motion_faults & state.controller.faults;
    out->actuation_started = actuation_started;
    out->required_hold_ms = (state.controller.faults & YAW_OFF_MOTION_FAULTS) ?
        YAW_CLEAR_OFF_MOTION_HOLD_MS : YAW_CONTROLLER_STATIONARY_HOLD_MS;
    if (!monitoring) { out->blocks |= YAW_CLEAR_NOT_MONITORING; }
    if (state.requested) { out->blocks |= YAW_CLEAR_REQUESTED; }
    if (state.active || state.enabling) { out->blocks |= YAW_CLEAR_ACTIVE; }
    if (clear_waiting_disable) { out->blocks |= YAW_CLEAR_WAITING_DISABLE; }
    if (CanBus_YawAllowed() || CanBus_OutputAllowed()) { out->blocks |= YAW_CLEAR_OUTPUT_ALLOWED; }
    if (!Online(now)) { out->blocks |= YAW_CLEAR_FEEDBACK_STALE; }
    if (state.feedback.status != 13U) { out->blocks |= YAW_CLEAR_STATUS_NOT_13; }
    if (!(state.controller.faults & YAW_CONTROLLER_FAULT_STATUS) ||
        (state.controller.faults & ~(YAW_CONTROLLER_FAULT_STATUS | out->off_motion_faults)) ||
        (actuation_started && (state.controller.faults & YAW_OFF_MOTION_FAULTS))) {
        out->blocks |= YAW_CLEAR_OTHER_FAULT;
    }
    if (clear_stationary_samples < 2U || out->hold_ms < out->required_hold_ms) {
        out->blocks |= YAW_CLEAR_NOT_STATIONARY;
    }
    if (!sent || cancel_pending || out->fd_age_ms >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
        out->blocks |= YAW_CLEAR_DISABLE_UNCONFIRMED;
    }
    if (!bus.started || bus.fault_epoch) { out->blocks |= YAW_CLEAR_CAN_FAULT; }
    if (out->mailbox_pending) { out->blocks |= YAW_CLEAR_MAILBOX_BUSY; }
    taskEXIT_CRITICAL();
}

uint8_t YawBackend_ClearCommsFault(void)
{
    uint8_t accepted = 0U;
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    YawClearCheck check;
    taskENTER_CRITICAL();
    /* 单次FB供人工READY或受控上电准备；关闭监测的运动记录按来源和500ms静止门禁审核。
     * 软件锁存仍不在此清除，需后续新的FD/状态0和人工参考捕获。 */
    YawBackend_GetClearCheck(&check);
    last_clear_blocks = check.blocks; last_clear_result = 4U;
    if (!check.blocks) {
        result = CanBus_YawClearComms(); last_clear_result = (uint8_t)result;
        if (result == HAL_OK) {
            clear_waiting_disable = 1U;
            state.stop_pending = 1U; disable_ms = now;
            /* 不abort刚提交的FB；下一控制周期等待邮箱空后补FD，不重发FB。 */
            cancel_pending = 0U; sent = 0U; accepted = 1U;
        } else {
            last_clear_blocks = result == HAL_BUSY ? YAW_CLEAR_MAILBOX_BUSY : YAW_CLEAR_TX_FAILED;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

uint8_t YawBackend_HoldCurrent(void)
{
    uint8_t accepted = 0U;
    float target;
    taskENTER_CRITICAL();
    target = state.controller.angle_deg;
    /* Match the existing start-capture allowance for endpoint quantization. */
    if (target > YAW_CONTROLLER_TARGET_LIMIT_DEG &&
        target <= YAW_CONTROLLER_TARGET_LIMIT_DEG + YAW_STEP_BAND_DEG) {
        target = YAW_CONTROLLER_TARGET_LIMIT_DEG;
    } else if (target < -YAW_CONTROLLER_TARGET_LIMIT_DEG &&
        target >= -YAW_CONTROLLER_TARGET_LIMIT_DEG - YAW_STEP_BAND_DEG) {
        target = -YAW_CONTROLLER_TARGET_LIMIT_DEG;
    }
    if (state.active && state.requested && !state.enabling && !state.stop_pending &&
        Online(HAL_GetTick()) && state.feedback.status == 1U && CanBus_YawAllowed() &&
        !state.controller.faults) {
        if (state.view_locked && !state.view_hold_pending) { accepted = 1U; }
        else if (YawController_SetTarget(&state.controller, target)) {
            YawStep_Abort(&step, HAL_GetTick(), YAW_STEP_MANUAL, 0U);
            /* Preserve observer and inner load compensation. */
            state.controller.cascade.position_i = 0.0f;
            state.controller.integral = 0.0f;
            state.step_hold = 1U; step_from_remote = (uint8_t)!state.view_locked;
            state.view_hold_pending = 0U; accepted = 1U;
        }
    }
    taskEXIT_CRITICAL(); return accepted;
}

void YawBackend_SetViewLock(uint8_t locked)
{
    taskENTER_CRITICAL(); locked = (uint8_t)(locked != 0U);
    if (state.view_locked != locked) {
        state.view_locked = state.view_hold_pending = locked;
        if (!locked) { state.step_hold = step_from_remote = 0U; }
    }
    taskEXIT_CRITICAL();
}

void YawBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic)
{
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    uint8_t wanted, new_step = 0U;
    taskENTER_CRITICAL();
    /* 事件不排队：未就绪/握手时消费并拒绝，不能稍后自动补发。 */
    if (request && request->step_event && request->step_sequence != last_step_sequence) {
        last_step_sequence = request->step_sequence; new_step = 1U;
    }
    if ((state.active || state.enabling) && !Online(now)) {
        state.controller.faults |= YAW_CONTROLLER_FAULT_FEEDBACK;
        YawBackend_Stop(); rearm_request = 1U;
    }
    wanted = APP_YAW_ENABLED && !preparation_hold && state.requested && allowed && !diagnostic && request &&
        Online(now) && !state.controller.faults && !state.stop_pending && !clear_waiting_disable;
    if (!wanted) {
        if (new_step) { RemoteNotice(request, 3U); }
        if (state.active || state.enabling) { YawBackend_Stop(); }
        CanBus_SetYawPermission(0U);
        if (monitoring && (!sent || (uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS)) {
            result = CanBus_YawDisable(cancel_pending);
            last_disable_result = (uint8_t)result;
            last_tx_ms = now; sent = 1U;
            if (result == HAL_OK) {
                cancel_pending = 0U; clear_waiting_disable = 0U; last_submitted_torque_nm = 0.0f;
                last_submitted_limited = 0U;
            }
            /* 失能提交失败保留pending/撤权，后续继续尝试。 */
            else { state.stop_pending = 1U; }
        }
        taskEXIT_CRITICAL(); return;
    }
    CanBus_SetYawPermission(1U);
    if (!CanBus_YawAllowed()) { YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return; }
    if (!state.active && !state.enabling) {
        if (new_step) { RemoteNotice(request, 3U); }
        if (!CaptureForStart()) {
            YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
        off_motion_faults = 0U; actuation_started = 1U;
        state.controller.motion_guard_active = 1U;
        result = CanBus_YawEnable(); last_tx_ms = now; sent = 1U;
        if (result != HAL_OK) {
            state.controller.faults |= YAW_BACKEND_FAULT_TX;
            YawBackend_Stop(); rearm_request = 1U;
        } else { state.enabling = 1U; enable_ms = now; }
        taskEXIT_CRITICAL(); return;
    }
    if (state.enabling) {
        if (new_step) { RemoteNotice(request, 3U); }
        if (state.feedback.status == 1U && (int32_t)(state.feedback.received_ms - enable_ms) > 0) {
            state.enabling = 0U; state.active = 1U;
        } else if ((uint32_t)(now - enable_ms) >= APP_YAW_ENABLE_TIMEOUT_MS) {
            state.controller.faults |= YAW_BACKEND_FAULT_ENABLE;
            YawBackend_Stop(); rearm_request = 1U;
        } else if ((uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS) {
            if (CanBus_YawEnable() != HAL_OK) {
                state.controller.faults |= YAW_BACKEND_FAULT_TX;
                YawBackend_Stop(); rearm_request = 1U;
            }
            last_tx_ms = now;
        }
        taskEXIT_CRITICAL(); return;
    }
    if ((last_mode == GIMBAL_TEST || (state.step_hold && step_from_remote)) &&
        request->gimbal_mode != GIMBAL_TEST) {
        YawStep_Abort(&step, now, YAW_STEP_MANUAL, 0U);
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (state.view_locked && state.view_hold_pending && !YawBackend_HoldCurrent()) {
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if (new_step && state.view_locked && request->step_axis == GIMBAL_STEP_YAW) {
        RemoteNotice(request, 3U); new_step = 0U;
    }
#if APP_REMOTE_PROFILE_ASSESSMENT
    if (request->gimbal_mode == GIMBAL_TEST &&
        (last_mode != GIMBAL_TEST || (new_step && request->step_axis == GIMBAL_STEP_PITCH))) {
        if (!YawBackend_HoldCurrent()) {
            if (new_step) { RemoteNotice(request, 3U); }
            YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
        }
    }
#endif
    last_mode = request->gimbal_mode;
    if (new_step) {
        if (request->gimbal_mode != GIMBAL_TEST) { RemoteNotice(request, 3U); }
        else if (request->step_return_event
#if APP_PITCH_ENABLED
                 && request->step_axis == GIMBAL_STEP_YAW
#endif
                 ) {
            /* A return is a retarget request, not a new settled-start test.
             * Do not drop the edge while the old motion is still settling. */
            if (YawBackend_SetParameter('T', state.controller.target_deg + request->step_delta_deg)) {
                state.step_hold = step_from_remote = 1U; RemoteNotice(request, 2U);
            }
            else { RemoteNotice(request, 3U); }
        } else if (request->step_axis != GIMBAL_STEP_YAW) { RemoteNotice(request, 4U); }
        else if (YawBackend_StartStep(request->step_delta_deg)) {
            step_from_remote = 1U; RemoteNotice(request, 1U);
        } else { RemoteNotice(request, 3U); }
    }
    /* 测试模式保留TargetManager的CH4选方向、CH7确认/回参考；回中不清目标。 */
    if (!state.view_locked && state.step_hold && request->gimbal_mode == GIMBAL_MANUAL && request->yaw_rate_dps != 0.0f) {
        YawStep_Abort(&step, now, YAW_STEP_MANUAL, 0U);
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    (void)YawController_Update(&state.controller,
        request->gimbal_mode == GIMBAL_MANUAL && !state.view_locked ? request->yaw_rate_dps : 0.0f, now, 1U);
    if (state.controller.faults) {
        YawBackend_Stop(); rearm_request = 1U; taskEXIT_CRITICAL(); return;
    }
    if ((uint32_t)(now - last_tx_ms) >= APP_YAW_COMMAND_PERIOD_MS) {
        result = CanBus_YawTorque(state.controller.torque_nm); last_tx_ms = now;
        if (result != HAL_OK) {
            state.controller.faults |= YAW_BACKEND_FAULT_TX;
            YawBackend_Stop(); rearm_request = 1U;
        } else {
            last_submitted_torque_nm = state.controller.torque_nm;
            last_submitted_limited = YAW_CONTROLLER_CASCADE_ENABLED ?
                (uint8_t)(state.controller.cascade.torque_limited || state.controller.cascade.velocity_limited) :
                state.controller.speed_limited;
            YawStep_Command(&step, now, last_submitted_torque_nm);
        }
    }
    taskEXIT_CRITICAL();
}
/* APP_SECTION_END yaw_backend.c */
