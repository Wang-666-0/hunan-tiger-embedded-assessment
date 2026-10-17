#include "yaw_cascade.h"
#include <float.h>
#include <string.h>

static float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static uint8_t finite_value(float x) { return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
static uint8_t winds_up(float wanted, float change, float limit)
{ return (uint8_t)((wanted > limit && change > 0.0f) || (wanted < -limit && change < 0.0f)); }

void YawCascade_Stop(YawCascade *loop)
{
    loop->position_i = loop->speed_i = 0.0f;
    loop->torque_nm = loop->unsaturated_torque_nm = loop->target_velocity_dps = 0.0f;
    loop->have_update = loop->torque_limited = loop->velocity_limited = 0U;
}
void YawCascade_ResetObserver(YawCascade *loop)
{ loop->velocity_dps = loop->acceleration_dps2 = 0.0f; YawCascade_Stop(loop); }
void YawCascade_Init(YawCascade *loop)
{
    memset(loop, 0, sizeof(*loop));
    loop->position.kp = 8.0f;
    loop->speed.kp = 0.001f; loop->speed.ki = 0.004f;
    loop->velocity_limit_dps = 180.0f;
}
void YawCascade_Observe(YawCascade *loop, float delta_deg, uint32_t gap_ms)
{
    float previous, dt, alpha;
    if (!gap_ms || gap_ms >= 100U || !finite_value(delta_deg)) { YawCascade_ResetObserver(loop); return; }
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
    if (!finite_value(kp) || !finite_value(ki) || !finite_value(kd) || kp < 0.0f || ki < 0.0f || kd < 0.0f) { return 0U; }
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
    if (!finite_value(dps) || dps < 10.0f || dps > 360.0f) { return 0U; }
    if (loop->velocity_limit_dps != dps) {
        loop->velocity_limit_dps = dps; YawCascade_Stop(loop); ++loop->revision;
    }
    return 1U;
}
float YawCascade_Update(YawCascade *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm)
{
    float dt = 0.0f, wanted, candidate, change, speed_error;
    uint32_t gap = feedback_ms - loop->feedback_ms;
    if (loop->have_update && sequence != loop->sequence && gap < 100U) { dt = (float)gap * 0.001f; }
    loop->feedback_ms = feedback_ms; loop->sequence = sequence; loop->have_update = 1U;
    if (loop->position.ki == 0.0f) { loop->position_i = 0.0f; }
    change = loop->position.ki * error_deg * dt;
    candidate = clamp(loop->position_i + change, -loop->velocity_limit_dps, loop->velocity_limit_dps);
    wanted = loop->position.kp * error_deg + candidate - loop->position.kd * loop->velocity_dps;
    /* 外环同时跟踪速度参考及内环力矩饱和；允许反向误差解积分。 */
    if (!winds_up(wanted, change, loop->velocity_limit_dps) &&
        !winds_up(loop->unsaturated_torque_nm, change, torque_limit_nm)) { loop->position_i = candidate; }
    wanted = loop->position.kp * error_deg + loop->position_i - loop->position.kd * loop->velocity_dps;
    loop->target_velocity_dps = clamp(wanted, -loop->velocity_limit_dps, loop->velocity_limit_dps);
    loop->velocity_limited = (uint8_t)(wanted > loop->velocity_limit_dps || wanted < -loop->velocity_limit_dps);
    speed_error = loop->target_velocity_dps - loop->velocity_dps;
    if (loop->speed.ki == 0.0f) { loop->speed_i = 0.0f; }
    change = loop->speed.ki * speed_error * dt;
    candidate = clamp(loop->speed_i + change, -torque_limit_nm, torque_limit_nm);
    wanted = loop->speed.kp * speed_error + candidate - loop->speed.kd * loop->acceleration_dps2;
    if (!winds_up(wanted, change, torque_limit_nm)) { loop->speed_i = candidate; }
    loop->unsaturated_torque_nm = loop->speed.kp * speed_error + loop->speed_i - loop->speed.kd * loop->acceleration_dps2;
    loop->torque_nm = clamp(loop->unsaturated_torque_nm, -torque_limit_nm, torque_limit_nm);
    loop->torque_limited = (uint8_t)(loop->unsaturated_torque_nm > torque_limit_nm || loop->unsaturated_torque_nm < -torque_limit_nm);
    return loop->torque_nm;
}
