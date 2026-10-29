/**
 * @file yaw_step.c
 * 作用：两轴真实阶跃统计核心
 * 对外函数：
 *   YawStep_Start / YawStep_Sample / YawStep_Passed / YawStep_Command / YawStep_Abort / YawStep_Init
 * 内部函数：
 *   yaw_step_absolute / yaw_step_finite_value
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "yaw_step.h"
#include <float.h>
#include <string.h>

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static float yaw_step_absolute(float value);
static uint8_t yaw_step_finite_value(float value);

uint8_t YawStep_Start(YawStep *step, uint32_t now_ms, float start_deg, float target_deg)
/*
 * 作用：建立Yaw/共用新阶跃的起点、最终目标、计时和判据快照。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 now_ms：当前毫秒时间。
 * 参数 start_deg：动作开始时的实际角度，deg。
 * 参数 target_deg：动作最终目标角度，deg。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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

void YawStep_Sample(YawStep *step, uint32_t received_ms, float angle_deg,
                    float velocity_dps, float torque_nm, uint8_t limited)
/*
 * 作用：依据真实 CAN 样本统计首次入带、最终稳定时间、超调及稳态误差。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 received_ms：样本实际接收的毫秒时间戳。
 * 参数 angle_deg：本帧实际相对角度，deg。
 * 参数 velocity_dps：本帧实际角速度，deg/s。
 * 参数 torque_nm：协议力矩，N*m。
 * 参数 limited：本帧对应的控制输出是否限幅，用于统计。
 * 流程：总窗口为 5 s；末 1 s 用角度的时间积分求均值，避免不均匀采样造成偏差。
 * 流程：最终稳定时间取最后一段持续在 ±0.2 deg 内的起点，首次入带并不等于稳定。
 * 流程：样本间隔异常或数据无效会中止记录，不把中止记录作为通过成绩。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
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
        step->ss_integral += (left + angle) * 0.5f * (float)duration; // 梯形积分，按时间加权而非按采样点数平均。
        step->ss_duration_ms += duration; ++step->ss_samples;
        if (yaw_step_absolute(step->target_deg - left) > step->ss_max_error) { step->ss_max_error = yaw_step_absolute(step->target_deg - left); }
        if (yaw_step_absolute(step->target_deg - angle) > step->ss_max_error) { step->ss_max_error = yaw_step_absolute(step->target_deg - angle); }
        step->steady_error_deg = step->target_deg - step->ss_integral / (float)step->ss_duration_ms;
    }
    step->last_ms = received_ms; step->last_angle_deg = angle_deg;
    step->last_velocity_dps = velocity_dps; step->last_torque_nm = torque_nm; step->elapsed_ms = end;
    if (end == YAW_STEP_WINDOW_MS) {
        step->phase = YAW_STEP_DONE; step->pending = 1U;
        if (step->band_active && end - step->band_since_ms >= 50U) {
            step->settle_ms = step->band_since_ms;
        }
        step->sse_valid = (uint8_t)(step->ss_duration_ms == YAW_STEP_STEADY_MS &&
            step->ss_samples >= 10U && step->ss_max_error <= YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG);
        step->pass = YawStep_Passed(step, 250U);
    }
}

uint8_t YawStep_Passed(const YawStep *step, uint32_t limit_ms)
/*
 * 作用：按当前考核条件判断一份已完成的真实阶跃记录是否通过。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 limit_ms：最终稳定时间的考核上限，ms。
 * 流程：同时检查约 30 deg 步长、无故障、最终稳定时间、超调和有效稳态误差。
 * 流程：Yaw 时间上限为 250 ms，Pitch 通过包装层传入独立的 300 ms 上限。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    return (uint8_t)(step && step->phase == YAW_STEP_DONE && !step->faults &&
        yaw_step_absolute(step->span_deg - 30.0f) <= YAW_STEP_BAND_DEG + YAW_STEP_COMPARE_EPS_DEG &&
        step->settle_ms != YAW_STEP_UNREACHED && step->settle_ms <= limit_ms &&
        step->overshoot_deg <= YAW_STEP_OVERSHOOT_LIMIT_DEG + YAW_STEP_COMPARE_EPS_DEG &&
        step->sse_valid && yaw_step_absolute(step->steady_error_deg) <= 0.2f + YAW_STEP_COMPARE_EPS_DEG);
}

void YawStep_Command(YawStep *step, uint32_t now_ms, float torque_nm)
/*
 * 作用：记录Yaw/共用本轮实际提交的控制量、命令次数及首次输出。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 now_ms：当前毫秒时间。
 * 参数 torque_nm：协议力矩，N*m。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (!step || step->phase != YAW_STEP_RUNNING ||
        (uint32_t)(now_ms - step->start_ms) >= YAW_STEP_WINDOW_MS ||
        !yaw_step_finite_value(torque_nm) || yaw_step_absolute(torque_nm) > 10.0f) { return; }
    if (!step->command_samples) { step->first_command_nm = torque_nm; }
    ++step->command_samples;
    if (yaw_step_absolute(torque_nm) > step->peak_torque_nm) { step->peak_torque_nm = yaw_step_absolute(torque_nm); }
}

void YawStep_Abort(YawStep *step, uint32_t now_ms, YawStepReason reason, uint32_t faults)
/*
 * 作用：用指定原因结束Yaw/共用当前阶跃统计，保留中断及故障证据。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 now_ms：当前毫秒时间。
 * 参数 reason：本次停止、中止或状态原因。
 * 参数 faults：当前故障位集合。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (!step || step->phase != YAW_STEP_RUNNING) { return; }
    step->phase = YAW_STEP_ABORTED; step->reason = (uint8_t)reason;
    step->faults = faults; step->elapsed_ms = now_ms - step->start_ms;
    step->pending = 1U; step->sse_valid = step->pass = 0U;
}

void YawStep_Init(YawStep *step)
/*
 * 作用：初始化Yaw/共用真实阶跃统计及未到位标记。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (!step) { return; }
    memset(step, 0, sizeof(*step));
    step->t10_ms = step->t90_ms = step->response_ms = step->settle_ms = YAW_STEP_UNREACHED;
}

static float yaw_step_absolute(float value)
/*
 * 作用：返回数值的绝对值，用于误差或速度幅值比较。
 * 参数 value：待处理的数值。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{ return value < 0.0f ? -value : value; }

static uint8_t yaw_step_finite_value(float value)
/*
 * 作用：检查浮点数是否有限，拒绝 NaN 和无穷值进入控制。
 * 参数 value：待处理的数值。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{ return (uint8_t)(value == value && value >= -FLT_MAX && value <= FLT_MAX); }
