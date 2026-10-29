/**
 * @file yaw_controller.c
 * 作用：Yaw 双环目标与反馈保护
 * 对外函数：
 *   YawController_Update / YawController_OnFeedback / YawController_CaptureReference /
 *   YawController_SetTarget / YawController_Init / YawController_Stop
 * 内部函数：
 *   yaw_controller_finite_value / yaw_controller_absolute / yaw_controller_clamp / fault / sample_safe
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "yaw_controller.h"
#include <float.h>
#include <string.h>

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static uint8_t yaw_controller_finite_value(float value);
static float yaw_controller_absolute(float value);
static float yaw_controller_clamp(float value, float low, float high);
static void fault(YawController *yaw, uint32_t reasons);
static uint8_t sample_safe(const YawController *yaw);

float YawController_Update(YawController *yaw, float rate_dps,
                           uint32_t now_ms, uint8_t allowed)
/*
 * 作用：在许可和反馈有效时累加手动角度目标，执行双环 PID 并返回力矩。
 * 参数 yaw：Yaw 控制器实例。
 * 参数 rate_dps：摇杆请求的角速度，deg/s。
 * 参数 now_ms：当前毫秒时间。
 * 参数 allowed：本轮是否允许输出动力。
 * 流程：目标增量 = rate_dps × 控制间隔；角度目标保持在软件目标范围内。
 * 流程：摇杆回中后 rate_dps 为零，目标不再变化，位置环继续保持当前目标。
 * 流程：位置误差和 CAN 反馈序号交给共用 PID；本函数不发送 CAN。
 * 返回本次计算的协议力矩，N*m；此函数本身不提交 CAN。
 */
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

    if (yaw_controller_absolute(yaw->angle_deg) > YAW_CONTROLLER_HARD_LIMIT_DEG) {
        fault(yaw, YAW_CONTROLLER_FAULT_LIMIT); return 0.0f;
    }
    elapsed = yaw->have_control_tick ? now_ms - yaw->last_control_ms : 0U;
    yaw->last_control_ms = now_ms;
    yaw->have_control_tick = 1U;
    if (elapsed > 10U) {
        fault(yaw, YAW_CONTROLLER_FAULT_TIMING); return 0.0f;
    }
    yaw->active = 1U;
    rate_dps = yaw_controller_clamp(rate_dps, -YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS,
                     YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS);
    yaw->target_deg = yaw_controller_clamp(yaw->target_deg + rate_dps * (float)elapsed / 1000.0f, // 速度请求积分成角度目标。
        -YAW_CONTROLLER_TARGET_LIMIT_DEG, YAW_CONTROLLER_TARGET_LIMIT_DEG);
    {
        yaw->torque_nm = GimbalPid_Update(&yaw->cascade, yaw->target_deg - yaw->angle_deg,
            yaw->last_feedback_ms, yaw->last_sequence, YAW_CONTROLLER_TORQUE_LIMIT_NM);
    }
    yaw->pid_torque_nm = yaw->cascade.unsaturated_torque_nm - yaw->cascade.feedforward_nm;
    if (!yaw_controller_finite_value(yaw->torque_nm) || !yaw_controller_finite_value(yaw->pid_torque_nm) ||
        !yaw_controller_finite_value(yaw->cascade.position_i) || !yaw_controller_finite_value(yaw->cascade.speed_i) ||
        !yaw_controller_finite_value(yaw->cascade.target_velocity_dps)) { fault(yaw, YAW_CONTROLLER_FAULT_DATA); return 0.0f; }
    return yaw->torque_nm;
}

uint8_t YawController_OnFeedback(YawController *yaw, const Gl60Feedback *feedback)
/*
 * 作用：检查Yaw反馈有效性与连续性，更新实际角度、观测和运行故障。
 * 参数 yaw：Yaw 控制器实例。
 * 参数 feedback：解码后的电机实际反馈。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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
    speed_limit = strict ? 1200.0f : 360.0f;
    jump_margin = strict ? 1.0f : 3.0f;
    angle_limit = strict ? YAW_CONTROLLER_HARD_LIMIT_DEG : 160.0f;
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
    if (first || rebase) { GimbalPid_ResetObserver(&yaw->cascade); }
    else { GimbalPid_Observe(&yaw->cascade, delta * YAW_RAD_TO_DEG, gap); }
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
    if (!strict && yaw_controller_absolute(yaw->velocity_dps) > speed_limit) {
        fault(yaw, YAW_CONTROLLER_FAULT_SPEED);
    }
    if (yaw->have_reference && yaw_controller_absolute(yaw->angle_deg) > angle_limit) {
        fault(yaw, YAW_CONTROLLER_FAULT_LIMIT);
    }
    return 1U;
}

uint8_t YawController_CaptureReference(YawController *yaw)
/*
 * 作用：在原健康静止条件下建立Yaw软件零点，保留不可自动清除的故障。
 * 参数 yaw：Yaw 控制器实例。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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
    GimbalPid_ResetObserver(&yaw->cascade);
    yaw->faults &= ~(uint32_t)YAW_RECOVERABLE_FAULTS;
    return 1U;
}

uint8_t YawController_SetTarget(YawController *yaw, float degrees)
/*
 * 作用：核对Yaw目标范围和参考有效性后，设置最终位置目标。
 * 参数 yaw：Yaw 控制器实例。
 * 参数 degrees：软件坐标中的目标角度，deg。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (yaw == 0 || !yaw_controller_finite_value(degrees) ||
        degrees < -YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        degrees > YAW_CONTROLLER_TARGET_LIMIT_DEG ||
        !yaw->have_reference || yaw->faults != 0U) { return 0U; }
    yaw->target_deg = degrees;
    return 1U;
}

void YawController_Init(YawController *yaw)
/*
 * 作用：载入Yaw默认双环参数，初始化参考、观测和故障状态。
 * 参数 yaw：Yaw 控制器实例。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (yaw == 0) { return; }
    memset(yaw, 0, sizeof(*yaw));
    GimbalPid_Init(&yaw->cascade);
    yaw->motion_guard_active = 1U;
}

void YawController_Stop(YawController *yaw)
/*
 * 作用：停止Yaw当前控制并清理对应历史，保留原参数及安全参考规则。
 * 参数 yaw：Yaw 控制器实例。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (yaw == 0) { return; }
    yaw->active = 0U;
    GimbalPid_Stop(&yaw->cascade);
    yaw->torque_nm = 0.0f;
    yaw->pid_torque_nm = 0.0f;
    yaw->have_control_tick = 0U;
    yaw->target_deg = yaw_controller_finite_value(yaw->angle_deg) ?
        yaw_controller_clamp(yaw->angle_deg, -YAW_CONTROLLER_TARGET_LIMIT_DEG,
              YAW_CONTROLLER_TARGET_LIMIT_DEG) : 0.0f;
}

static uint8_t yaw_controller_finite_value(float value)
/*
 * 作用：检查浮点数是否有限，拒绝 NaN 和无穷值进入控制。
 * 参数 value：待处理的数值。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    return (uint8_t)(value == value && value >= -FLT_MAX && value <= FLT_MAX);
}

static float yaw_controller_absolute(float value)
/*
 * 作用：返回数值的绝对值，用于误差或速度幅值比较。
 * 参数 value：待处理的数值。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{ return value < 0.0f ? -value : value; }

static float yaw_controller_clamp(float value, float low, float high)
/*
 * 作用：把数值限制在指定上下界内，避免控制请求越界。
 * 参数 value：待处理的数值。
 * 参数 low：下界。
 * 参数 high：上界。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{
    return value < low ? low : (value > high ? high : value);
}

static void fault(YawController *yaw, uint32_t reasons)
/*
 * 作用：锁存 Yaw 控制故障并沿原停止路径清理当前输出。
 * 参数 yaw：Yaw 控制器实例。
 * 参数 reasons：本次要锁存的控制器故障位集合。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    yaw->faults |= reasons;
    YawController_Stop(yaw);
}

static uint8_t sample_safe(const YawController *yaw)
/*
 * 作用：核对 Yaw 最近反馈的状态、温度和静止速度是否适合建立参考。
 * 参数 yaw：Yaw 控制器实例。
 * 返回布尔判断结果，非零为满足条件，0 为不满足。
 */
{
    return (uint8_t)(yaw->have_feedback && yaw->last_status <= 1U &&
        yaw->last_driver_temp_c < YAW_CONTROLLER_DRIVER_LIMIT_C &&
        yaw->last_motor_temp_c < YAW_CONTROLLER_MOTOR_LIMIT_C &&
        yaw_controller_absolute(yaw->velocity_dps) <= YAW_CONTROLLER_STATIONARY_SPEED_DPS);
}
