/**
 * @file pitch_step.h
 * @brief Pitch 实测阶跃接口
 * 职责：复用 YawStep 数据与统计核心，声明 Pitch 参数和结果接口。
 * 主要接口：PitchStep_Init, PitchStep_Start, PitchStep_Sample, PitchStep_Command, PitchStep_Abort。
 * 实现：control/pitch_step.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef PITCH_STEP_H
#define PITCH_STEP_H
#include "app_config.h"
#include "yaw_step.h"

/* 共用纯测量实现，实例完全独立；Yaw继续250ms，Pitch独立300ms。 */
typedef YawStep PitchStep;
typedef YawStepPhase PitchStepPhase;
typedef YawStepReason PitchStepReason;
void PitchStep_Init(PitchStep *step);
uint8_t PitchStep_Start(PitchStep *step, uint32_t now_ms, float start_deg, float target_deg);
void PitchStep_Sample(PitchStep *step, uint32_t received_ms, float angle_deg,
    float velocity_dps, float torque_nm, uint8_t limited);
void PitchStep_Command(PitchStep *step, uint32_t now_ms, float torque_nm);
void PitchStep_Abort(PitchStep *step, uint32_t now_ms, PitchStepReason reason, uint32_t faults);
#endif
