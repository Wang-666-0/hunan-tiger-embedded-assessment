/**
 * @file pitch_step.c
 * @brief Pitch 真实阶跃判据适配
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（5）：
 *   PitchStep_Init / PitchStep_Start / PitchStep_Sample / PitchStep_Command / PitchStep_Abort
 * 内部函数（1）：
 *   pitch_step_absolute
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* Pitch 实测统计适配：使用独立判定参数与实例。 */
#include "pitch_step.h"

static float pitch_step_absolute(float x)
/* 功能：返回数值的绝对值，用于误差或速度幅值比较。 */
{ return x < 0.0f ? -x : x; }
void PitchStep_Init(PitchStep *step)
/* 功能：初始化Pitch真实阶跃统计及未到位标记。 */
{ YawStep_Init(step); }
uint8_t PitchStep_Start(PitchStep *step, uint32_t now_ms, float start_deg, float target_deg)
/* 功能：建立Pitch新阶跃的起点、最终目标、计时和判据快照。 */
{
    if (pitch_step_absolute(start_deg) > 50.0f || pitch_step_absolute(target_deg) > 50.0f) { return 0U; }
    return YawStep_Start(step, now_ms, start_deg, target_deg);
}
void PitchStep_Sample(PitchStep *step, uint32_t received_ms, float angle_deg,
    float velocity_dps, float torque_nm, uint8_t limited)
/* 功能：接收Pitch真实反馈样本，更新首次入带、稳定时间、超调及稳态误差。 */
{
    uint8_t running = (uint8_t)(step && step->phase == PITCH_STEP_RUNNING);
    YawStep_Sample(step, received_ms, angle_deg, velocity_dps, torque_nm, limited);
    if (running && step->phase == PITCH_STEP_DONE) {
        step->pass = YawStep_Passed(step, PITCH_STEP_RESPONSE_LIMIT_MS);
    }
}
void PitchStep_Command(PitchStep *step, uint32_t now_ms, float torque_nm)
/* 功能：记录Pitch本轮实际提交的控制量、命令次数及首次输出。 */
{ YawStep_Command(step, now_ms, torque_nm); }
void PitchStep_Abort(PitchStep *step, uint32_t now_ms, PitchStepReason reason, uint32_t faults)
/* 功能：用指定原因结束Pitch当前阶跃统计，保留中断及故障证据。 */
{ YawStep_Abort(step, now_ms, reason, faults); }
