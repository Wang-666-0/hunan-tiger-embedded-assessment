/**
 * @file pitch_step.c
 * 作用：Pitch 真实阶跃判据适配
 * 对外函数：
 *   PitchStep_Start / PitchStep_Sample / PitchStep_Command / PitchStep_Abort / PitchStep_Init
 * 内部函数：
 *   pitch_step_absolute
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "pitch_step.h"

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static float pitch_step_absolute(float x);

uint8_t PitchStep_Start(PitchStep *step, uint32_t now_ms, float start_deg, float target_deg)
/*
 * 作用：建立Pitch新阶跃的起点、最终目标、计时和判据快照。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 now_ms：当前毫秒时间。
 * 参数 start_deg：动作开始时的实际角度，deg。
 * 参数 target_deg：动作最终目标角度，deg。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (pitch_step_absolute(start_deg) > 50.0f || pitch_step_absolute(target_deg) > 50.0f) { return 0U; }
    return YawStep_Start(step, now_ms, start_deg, target_deg);
}

void PitchStep_Sample(PitchStep *step, uint32_t received_ms, float angle_deg,
    float velocity_dps, float torque_nm, uint8_t limited)
/*
 * 作用：接收Pitch真实反馈样本，更新首次入带、稳定时间、超调及稳态误差。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 received_ms：样本实际接收的毫秒时间戳。
 * 参数 angle_deg：本帧实际相对角度，deg。
 * 参数 velocity_dps：本帧实际角速度，deg/s。
 * 参数 torque_nm：协议力矩，N*m。
 * 参数 limited：本帧对应的控制输出是否限幅，用于统计。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint8_t running = (uint8_t)(step && step->phase == PITCH_STEP_RUNNING);
    YawStep_Sample(step, received_ms, angle_deg, velocity_dps, torque_nm, limited);
    if (running && step->phase == PITCH_STEP_DONE) {
        step->pass = YawStep_Passed(step, 300U);
    }
}

void PitchStep_Command(PitchStep *step, uint32_t now_ms, float torque_nm)
/*
 * 作用：记录Pitch本轮实际提交的控制量、命令次数及首次输出。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 now_ms：当前毫秒时间。
 * 参数 torque_nm：协议力矩，N*m。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ YawStep_Command(step, now_ms, torque_nm); }

void PitchStep_Abort(PitchStep *step, uint32_t now_ms, PitchStepReason reason, uint32_t faults)
/*
 * 作用：用指定原因结束Pitch当前阶跃统计，保留中断及故障证据。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 参数 now_ms：当前毫秒时间。
 * 参数 reason：本次停止、中止或状态原因。
 * 参数 faults：当前故障位集合。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ YawStep_Abort(step, now_ms, reason, faults); }

void PitchStep_Init(PitchStep *step)
/*
 * 作用：初始化Pitch真实阶跃统计及未到位标记。
 * 参数 step：本次真实阶跃的起点、目标、采样和结果状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ YawStep_Init(step); }

static float pitch_step_absolute(float x)
/*
 * 作用：返回数值的绝对值，用于误差或速度幅值比较。
 * 参数 x：待处理的数值。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{ return x < 0.0f ? -x : x; }
