/**
 * @file yaw_step.h
 * @brief 共用实测阶跃统计
 * 职责：定义阶跃阶段、结束原因、测量快照及首次到位/稳定/超调判定。
 * 主要接口：YawStep_Init, YawStep_Start, YawStep_Sample, YawStep_Command, YawStep_Abort 等。
 * 实现/调用：yaw_controller.c；Yaw/Pitch 保留独立实例，只采真实反馈与实际提交命令。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef YAW_STEP_H
#define YAW_STEP_H
#include "app_config.h"
#include <stdint.h>
/* 正式考核：最终进入±0.2后保持到窗口末，同时限制超调和稳态误差。 */
/* 仅吸收单精度相减的边界舍入，远小于一个编码器刻度。 */
typedef enum { YAW_STEP_IDLE, YAW_STEP_RUNNING, YAW_STEP_DONE, YAW_STEP_ABORTED } YawStepPhase;
typedef enum {
    YAW_STEP_NONE, YAW_STEP_STOP, YAW_STEP_FAULT, YAW_STEP_PID_CHANGE,
    YAW_STEP_TARGET_CHANGE, YAW_STEP_MANUAL, YAW_STEP_GAP, YAW_STEP_DATA, YAW_STEP_FF_CHANGE
} YawStepReason;
typedef struct {
    uint32_t id, start_ms, last_ms, elapsed_ms, samples, max_gap_ms;
    uint32_t t10_ms, t90_ms, response_ms, settle_ms, band_since_ms;
    uint32_t ss_duration_ms, ss_samples, faults, limited_samples;
    uint32_t command_samples;
    uint32_t shape_ms;
    float shape_velocity_gain;
    float start_deg, target_deg, span_deg, direction, last_angle_deg;
    float last_velocity_dps, last_torque_nm, overshoot_deg, overshoot_pct;
    float steady_error_deg, peak_velocity_dps, peak_torque_nm;
    float first_command_nm;
    float ss_integral, ss_max_error, kp, ki, kd;
    float vkp, vki, vkd, vmax_dps;
    float ff_acceleration_gain, ff_limit_nm;
    uint8_t ff_enabled;
    /* Pitch冻结试验条件；Yaw保持0，不使用这些字段。 */
    float gravity_kg_nm, gravity_offset_deg;
    uint8_t gravity_calibrated;
    uint8_t pitch_brake_enabled;
    uint8_t cascade, output_limits;
    uint8_t phase, reason, pending, band_active, sse_valid, pass;
} YawStep;
/* 纯测量，不调用HAL、不发指令。上层在同一临界区串行化。
 * t0为命令接受时刻，后续时间来自CAN实际接收戳；不使用VOFA绘图dt。
 * RESP=首次反馈进入目标±0.2度的时刻，不要求持续保持；一旦取得不改写。
 * SETTLE=窗口末仍在带内至少50ms的最后一次进入时刻；
 * 稳态值是第4~5秒的时间加权均值；有效还要求该段始终在±0.2内。
 * PASS用SETTLE判时间；首次进入很快但随后离带不算通过。 */
void YawStep_Init(YawStep *step);
uint8_t YawStep_Start(YawStep *step, uint32_t now_ms, float start_deg, float target_deg);
void YawStep_Sample(YawStep *step, uint32_t received_ms, float angle_deg,
                    float velocity_dps, float torque_nm, uint8_t limited);
/* 仅在HAL_OK提交后记MCU请求；不等同于总线发送完成或实际机械力矩。
 * 独立于反馈采样，首个反馈即故障也不丢失已提交的首段指令。 */
void YawStep_Command(YawStep *step, uint32_t now_ms, float torque_nm);
void YawStep_Abort(YawStep *step, uint32_t now_ms, YawStepReason reason, uint32_t faults);
/* 共用完成判定，调用轴提供250/300ms时限；无输出副作用。 */
uint8_t YawStep_Passed(const YawStep *step, uint32_t limit_ms);
#endif
