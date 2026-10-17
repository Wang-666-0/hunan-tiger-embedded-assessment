#include "yaw_controller.h"
#include <float.h>
#include <string.h>

#define PI_RAD 3.141593f
#define TWO_PI_RAD (2.0f * PI_RAD)
#define RAD_TO_DEG (180.0f / PI_RAD)
#define DEG_TO_RAD (PI_RAD / 180.0f)
#define HALF_TICK_RANGE 0x80000000UL
#define RECOVERABLE_FAULTS (YAW_CONTROLLER_FAULT_FEEDBACK | YAW_CONTROLLER_FAULT_DATA | \
    YAW_CONTROLLER_FAULT_STATUS | YAW_CONTROLLER_FAULT_TEMPERATURE | \
    YAW_CONTROLLER_FAULT_SPEED | YAW_CONTROLLER_FAULT_JUMP | \
    YAW_CONTROLLER_FAULT_LIMIT | YAW_CONTROLLER_FAULT_TIMING)

static uint8_t finite_value(float value)
{
    return (uint8_t)(value == value && value >= -FLT_MAX && value <= FLT_MAX);
}

static float absolute(float value) { return value < 0.0f ? -value : value; }
static float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
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
        absolute(yaw->velocity_dps) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS);
}

void YawController_Init(YawController *yaw)
{
    if (yaw == 0) { return; }
    memset(yaw, 0, sizeof(*yaw));
    yaw->pid.kp = 1.0f;
    yaw->pid.kd = 0.015f;
    YawCascade_Init(&yaw->cascade);
    yaw->motion_guard_active = 1U;
}

void YawController_Stop(YawController *yaw)
{
    if (yaw == 0) { return; }
    yaw->active = 0U;
    YawCascade_Stop(&yaw->cascade);
    yaw->speed_limited = 0U;
    yaw->torque_nm = 0.0f;
    yaw->integral = 0.0f;
    yaw->have_pid_feedback = 0U;
    yaw->have_control_tick = 0U;
    yaw->target_deg = finite_value(yaw->angle_deg) ?
        clamp(yaw->angle_deg, -YAW_CONTROLLER_TARGET_LIMIT_DEG,
              YAW_CONTROLLER_TARGET_LIMIT_DEG) : 0.0f;
}

uint8_t YawController_OnFeedback(YawController *yaw, const Gl60Feedback *feedback)
{
    uint32_t gap = 0U;
    float delta = 0.0f;
    float speed_limit, jump_margin, angle_limit;
    uint8_t first, strict, rebase = 0U;
    if (yaw == 0 || feedback == 0) { return 0U; }
    if (!finite_value(feedback->position_rad) ||
        !finite_value(feedback->velocity_rad_s) ||
        !finite_value(feedback->torque_nm) ||
        absolute(feedback->position_rad) > GL60_POSITION_MAX_RAD + 0.00001f ||
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
        if (gap == 0U || gap >= HALF_TICK_RANGE) { return 0U; }
        delta = feedback->position_rad - yaw->last_position_rad;
        if (delta > PI_RAD) { delta -= TWO_PI_RAD; }
        else if (delta < -PI_RAD) { delta += TWO_PI_RAD; }
        if (gap >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
            rebase = 1U;
            fault(yaw, YAW_CONTROLLER_FAULT_FEEDBACK);
        } else if (absolute(delta) * RAD_TO_DEG >
                   speed_limit * (float)gap / 1000.0f + jump_margin) {
            rebase = 1U;
            fault(yaw, YAW_CONTROLLER_FAULT_JUMP);
        }
    }
    yaw->last_position_rad = feedback->position_rad;
    yaw->last_feedback_ms = feedback->received_ms;
    ++yaw->last_sequence;
    yaw->velocity_dps = feedback->velocity_rad_s * RAD_TO_DEG;
    yaw->last_status = feedback->status;
    yaw->last_driver_temp_c = feedback->driver_temp_c;
    yaw->last_motor_temp_c = feedback->motor_temp_c;
    yaw->have_feedback = 1U;
    if (first || rebase) {
        yaw->continuous_rad = feedback->position_rad;
        yaw->reference_rad = feedback->position_rad;
        yaw->angle_deg = 0.0f;
        yaw->target_deg = 0.0f;
        /* 首次静止反馈可显示相对零位。断线/跳变后必须人工重新捕获。 */
        yaw->have_reference = (uint8_t)(first && sample_safe(yaw));
        yaw->stationary_samples = 0U;
    } else {
        yaw->continuous_rad += delta;
        yaw->angle_deg = (yaw->continuous_rad - yaw->reference_rad) * RAD_TO_DEG;
    }
    if (first || rebase) { YawCascade_ResetObserver(&yaw->cascade); }
    else { YawCascade_Observe(&yaw->cascade, delta * RAD_TO_DEG, gap); }
    if (sample_safe(yaw) && (first || rebase || absolute(delta) * RAD_TO_DEG <=
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
    if ((!strict || YAW_CONTROLLER_SPEED_GUARD_ENABLED) && absolute(yaw->velocity_dps) > speed_limit) {
        fault(yaw, YAW_CONTROLLER_FAULT_SPEED);
    }
    if (yaw->have_reference && absolute(yaw->angle_deg) > angle_limit) {
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
        (yaw->faults & ~(uint32_t)RECOVERABLE_FAULTS) != 0U) { return 0U; }
    YawController_Stop(yaw);
    yaw->reference_rad = yaw->continuous_rad;
    yaw->angle_deg = 0.0f;
    yaw->target_deg = 0.0f;
    yaw->have_reference = 1U;
    YawCascade_ResetObserver(&yaw->cascade);
    yaw->faults &= ~(uint32_t)RECOVERABLE_FAULTS;
    return 1U;
}

uint8_t YawController_SetPid(YawController *yaw, float kp, float ki, float kd)
{
    if (yaw == 0 || !finite_value(kp) || !finite_value(ki) || !finite_value(kd) ||
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
    if (yaw == 0 || !finite_value(degrees) ||
        degrees < -YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        degrees > YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        !yaw->have_reference || yaw->faults != 0U) { return 0U; }
    yaw->target_deg = degrees;
    return 1U;
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
    if (!finite_value(rate_dps)) { fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f; }
    age = now_ms - yaw->last_feedback_ms;
    if (!yaw->have_feedback || age >= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS ||
        age >= HALF_TICK_RANGE) {
        if (allowed || yaw->active) { fault(yaw, YAW_CONTROLLER_FAULT_FEEDBACK); }
        else { YawController_Stop(yaw); }
        return 0.0f;
    }
    if (!allowed || !yaw->have_reference || yaw->faults != 0U) {
        YawController_Stop(yaw);
        return 0.0f;
    }
    /* 放宽的观察样本不能直接进入闭环；许可转换时重查运行速度。 */
    if (YAW_CONTROLLER_SPEED_GUARD_ENABLED && absolute(yaw->velocity_dps) > YAW_CONTROLLER_SPEED_LIMIT_DPS) {
        fault(yaw, YAW_CONTROLLER_FAULT_SPEED); return 0.0f;
    }
    if (absolute(yaw->angle_deg) > YAW_CONTROLLER_HARD_LIMIT_DEG) {
        fault(yaw, YAW_CONTROLLER_FAULT_LIMIT); return 0.0f;
    }
    elapsed = yaw->have_control_tick ? now_ms - yaw->last_control_ms : 0U;
    yaw->last_control_ms = now_ms;
    yaw->have_control_tick = 1U;
    if (elapsed > YAW_CONTROLLER_CONTROL_MAX_GAP_MS) {
        fault(yaw, YAW_CONTROLLER_FAULT_TIMING); return 0.0f;
    }
    yaw->active = 1U;
    rate_dps = clamp(rate_dps, -YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS,
                     YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS);
    yaw->target_deg = clamp(yaw->target_deg + rate_dps * (float)elapsed / 1000.0f,
        -YAW_CONTROLLER_TARGET_LIMIT_DEG, YAW_CONTROLLER_TARGET_LIMIT_DEG);
#if YAW_CONTROLLER_CASCADE_ENABLED
    yaw->torque_nm = YawCascade_Update(&yaw->cascade, yaw->target_deg - yaw->angle_deg,
        yaw->last_feedback_ms, yaw->last_sequence, YAW_CONTROLLER_TORQUE_LIMIT_NM);
    return yaw->torque_nm;
#else
    error = (yaw->target_deg - yaw->angle_deg) * DEG_TO_RAD;
    damping = yaw->pid.kd * yaw->velocity_dps * DEG_TO_RAD;
    yaw->speed_limited = (uint8_t)(YAW_CONTROLLER_SPEED_GUARD_ENABLED &&
        absolute(yaw->velocity_dps) > YAW_CONTROLLER_SOFT_SPEED_DPS);
    if (yaw->pid.ki == 0.0f) { yaw->integral = 0.0f; }
    else if (!yaw->speed_limited && yaw->have_pid_feedback && yaw->pid_sequence != yaw->last_sequence) {
        elapsed = yaw->last_feedback_ms - yaw->pid_feedback_ms;
        if (elapsed <= YAW_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
            candidate = clamp(yaw->integral + error * (float)elapsed / 1000.0f,
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
        brake = -(yaw->velocity_dps - YAW_CONTROLLER_SOFT_SPEED_DPS) * DEG_TO_RAD * YAW_CONTROLLER_SPEED_BRAKE_KD;
        if (wanted > brake) { wanted = brake; }
    } else if (YAW_CONTROLLER_SPEED_GUARD_ENABLED && yaw->velocity_dps < -YAW_CONTROLLER_SOFT_SPEED_DPS) {
        brake = -(yaw->velocity_dps + YAW_CONTROLLER_SOFT_SPEED_DPS) * DEG_TO_RAD * YAW_CONTROLLER_SPEED_BRAKE_KD;
        if (wanted < brake) { wanted = brake; }
    }
    yaw->torque_nm = clamp(wanted, -YAW_CONTROLLER_TORQUE_LIMIT_NM,
                           YAW_CONTROLLER_TORQUE_LIMIT_NM);
    return yaw->torque_nm;
#endif
}
