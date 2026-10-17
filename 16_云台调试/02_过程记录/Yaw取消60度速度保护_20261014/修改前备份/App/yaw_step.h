#ifndef YAW_STEP_H
#define YAW_STEP_H
#include <stdint.h>
#define YAW_STEP_WINDOW_MS 5000U
#define YAW_STEP_STEADY_MS 1000U
#define YAW_STEP_BAND_DEG 0.2f
#define YAW_STEP_BAND_HOLD_MS 50U
#define YAW_STEP_UNREACHED 0xFFFFFFFFUL
typedef enum { YAW_STEP_IDLE, YAW_STEP_RUNNING, YAW_STEP_DONE, YAW_STEP_ABORTED } YawStepPhase;
typedef enum {
    YAW_STEP_NONE, YAW_STEP_STOP, YAW_STEP_FAULT, YAW_STEP_PID_CHANGE,
    YAW_STEP_TARGET_CHANGE, YAW_STEP_MANUAL, YAW_STEP_GAP, YAW_STEP_DATA
} YawStepReason;
typedef struct {
    uint32_t id, start_ms, last_ms, elapsed_ms, samples, max_gap_ms;
    uint32_t t10_ms, t90_ms, response_ms, settle_ms, band_since_ms;
    uint32_t ss_duration_ms, ss_samples, faults, limited_samples;
    float start_deg, target_deg, span_deg, direction, last_angle_deg;
    float last_velocity_dps, last_torque_nm, overshoot_deg, overshoot_pct;
    float steady_error_deg, peak_velocity_dps, peak_torque_nm;
    float ss_integral, ss_max_error, kp, ki, kd;
    uint8_t phase, reason, pending, band_active, sse_valid, pass;
} YawStep;
/* 纯测量，不调用HAL、不发指令。上层在同一临界区串行化。
 * t0为命令接受时刻，后续时间来自CAN实际接收戳；不使用VOFA绘图dt。
 * RESP=首次进入±0.2度且连续保持50ms的进入时刻，SETTLE=窗口末持续
 * 在带内的最后一次进入时刻；稳态值是第4~5秒的时间加权均值。 */
void YawStep_Init(YawStep *step);
uint8_t YawStep_Start(YawStep *step, uint32_t now_ms, float start_deg, float target_deg);
void YawStep_Sample(YawStep *step, uint32_t received_ms, float angle_deg,
                    float velocity_dps, float torque_nm, uint8_t limited);
void YawStep_Abort(YawStep *step, uint32_t now_ms, YawStepReason reason, uint32_t faults);
#endif
