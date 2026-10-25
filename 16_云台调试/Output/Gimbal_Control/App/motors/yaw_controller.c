/**
 * @file yaw_controller.c
 * @brief Yaw 双环目标与反馈保护
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（6）：
 *   YawController_Init / YawController_Stop / YawController_OnFeedback /
 *   YawController_CaptureReference / YawController_SetTarget / YawController_Update
 * 内部函数（5）：
 *   yaw_controller_finite_value / yaw_controller_absolute / yaw_controller_clamp / fault /
 *   sample_safe
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* Yaw 控制数学：软件零点、目标、轨迹与保护。 */
#include "yaw_controller.h"
#include <float.h>
#include <string.h>

static uint8_t yaw_controller_finite_value(float value)
/* 功能：检查浮点数是否有限，拒绝 NaN 和无穷值进入控制。 */
{
    return (uint8_t)(value == value && value >= -FLT_MAX && value <= FLT_MAX);
}

static float yaw_controller_absolute(float value)
/* 功能：返回数值的绝对值，用于误差或速度幅值比较。 */
{ return value < 0.0f ? -value : value; }
static float yaw_controller_clamp(float value, float low, float high)
/* 功能：把数值限制在指定上下界内，避免控制请求越界。 */
{
    return value < low ? low : (value > high ? high : value);
}

static void fault(YawController *yaw, uint32_t reasons)
/* 功能：锁存 Yaw 控制故障并沿原停止路径清理当前输出。 */
{
    yaw->faults |= reasons;
    YawController_Stop(yaw);
}

static uint8_t sample_safe(const YawController *yaw)
/* 功能：核对 Yaw 最近反馈的状态、温度和静止速度是否适合建立参考。 */
{
    return (uint8_t)(yaw->have_feedback && yaw->last_status <= 1U &&
        yaw->last_driver_temp_c < YAW_CONTROLLER_DRIVER_LIMIT_C &&
        yaw->last_motor_temp_c < YAW_CONTROLLER_MOTOR_LIMIT_C &&
        yaw_controller_absolute(yaw->velocity_dps) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS);
}

void YawController_Init(YawController *yaw)
/* 功能：载入Yaw默认双环参数，初始化参考、观测和故障状态。 */
{
    if (yaw == 0) { return; }
    memset(yaw, 0, sizeof(*yaw));
    YawCascade_Init(&yaw->cascade);
    yaw->motion_guard_active = 1U;
}

void YawController_Stop(YawController *yaw)
/* 功能：停止Yaw当前控制并清理对应历史，保留原参数及安全参考规则。 */
{
    if (yaw == 0) { return; }
    yaw->active = 0U;
    YawCascade_Stop(&yaw->cascade);
    yaw->torque_nm = 0.0f;
    yaw->pid_torque_nm = 0.0f;
    yaw->have_control_tick = 0U;
    yaw->target_deg = yaw_controller_finite_value(yaw->angle_deg) ?
        yaw_controller_clamp(yaw->angle_deg, -YAW_CONTROLLER_TARGET_LIMIT_DEG,
              YAW_CONTROLLER_TARGET_LIMIT_DEG) : 0.0f;
}

uint8_t YawController_OnFeedback(YawController *yaw, const Gl60Feedback *feedback)
/* 功能：检查Yaw反馈有效性与连续性，更新实际角度、观测和运行故障。 */
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
        yaw->continuous_rad = feedback->position_rad;
        yaw->reference_rad = feedback->position_rad;
        yaw->angle_deg = 0.0f;
        yaw->target_deg = 0.0f;
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
/* 功能：在原健康静止条件下建立Yaw软件零点，保留不可自动清除的故障。 */
{
    if (yaw == 0 || yaw->active || !sample_safe(yaw) ||
        yaw->stationary_samples < 2U ||
        yaw->last_feedback_ms - yaw->stationary_started_ms <
            YAW_CONTROLLER_STATIONARY_HOLD_MS ||
        (yaw->faults & ~(uint32_t)YAW_RECOVERABLE_FAULTS) != 0U) { return 0U; }
    YawController_Stop(yaw);
        yaw->reference_rad = yaw->continuous_rad;
    yaw->angle_deg = 0.0f;
    yaw->target_deg = 0.0f;
    yaw->have_reference = 1U;
    YawCascade_ResetObserver(&yaw->cascade);
    yaw->faults &= ~(uint32_t)YAW_RECOVERABLE_FAULTS;
    return 1U;
}

uint8_t YawController_SetTarget(YawController *yaw, float degrees)
/* 功能：核对Yaw目标范围和参考有效性后，设置最终位置目标。 */
{
    if (yaw == 0 || !yaw_controller_finite_value(degrees) ||
        degrees < -YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        degrees > YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        !yaw->have_reference || yaw->faults != 0U) { return 0U; }
    yaw->target_deg = degrees;
    return 1U;
}

float YawController_Update(YawController *yaw, float rate_dps,
                           uint32_t now_ms, uint8_t allowed)
/* 功能：在实时许可和反馈有效时执行Yaw控制，返回协议力矩；CAN 发送由后端负责。 */
{
    uint32_t elapsed, age;
    if (yaw == 0) { return 0.0f; }
    if (allowed) { yaw->motion_guard_active = 1U; }
    if (!yaw_controller_finite_value(rate_dps)) { fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f; }
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
    {
        yaw->torque_nm = YawCascade_Update(&yaw->cascade, yaw->target_deg - yaw->angle_deg,
            yaw->last_feedback_ms, yaw->last_sequence, YAW_CONTROLLER_TORQUE_LIMIT_NM);
    }
    yaw->pid_torque_nm = yaw->cascade.unsaturated_torque_nm - yaw->cascade.feedforward_nm;
    if (!yaw_controller_finite_value(yaw->torque_nm) || !yaw_controller_finite_value(yaw->pid_torque_nm) ||
        !yaw_controller_finite_value(yaw->cascade.position_i) || !yaw_controller_finite_value(yaw->cascade.speed_i) ||
        !yaw_controller_finite_value(yaw->cascade.target_velocity_dps)) { fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f; }
    return yaw->torque_nm;
}
