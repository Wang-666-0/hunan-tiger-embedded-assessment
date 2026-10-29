/* 摩擦轮速度 PI */
#ifndef FLYWHEEL_SPEED_H
#define FLYWHEEL_SPEED_H
#include "app_config.h"
#include <stdint.h>

/* 单轮、正电流幅值速度环。上层负责遥控许可、两轮反馈健康和硬保护。
 * 不调用HAL/RTOS、不发送CAN；实际极对数未知，控制单位始终为ERPM。 */
typedef struct {
    float target_erpm, filtered_erpm, integral_mA;
    int32_t requested_mA;
    uint32_t started_ms, last_update_ms, last_feedback_ms;
    uint32_t run_ms, peak_erpm, band_ms, longest_band_ms, limited_ms;
    uint32_t last_call_ms, coast_ms, zero_current_ms, low_speed_ms;
    uint32_t rise90_ms, band_first_ms; /* 启动到90%原始速度/首次连续500ms入带的确认时间。 */
    uint8_t running, coasting, at_limit, in_band;
    uint8_t rise90_seen, band_first_seen; /* 区分未达到与起始就已达到的0ms。 */
} FlywheelSpeed;

void FlywheelSpeed_Init(FlywheelSpeed *speed);
/* 每次人工重新启动清空控制量和本次统计。feedback_ms为有效STATUS1时间戳。 */
void FlywheelSpeed_Start(FlywheelSpeed *speed, uint32_t now_ms,
                        int32_t erpm, uint32_t feedback_ms);
/* 立即归零并清积分；保留本次持续时长、峰值和区间统计，便于停机后查询。 */
void FlywheelSpeed_Stop(FlywheelSpeed *speed);
/* 上层在每个有效STATUS1到来时调用，原始峰值及首次90%不等20ms PI周期。
 * 时间戳由反馈接收提供；早于启动的样本不会产生启动响应时间。 */
void FlywheelSpeed_RecordFeedback(FlywheelSpeed *speed, int32_t erpm, uint32_t feedback_ms);
/* 20ms且有新反馈时更新PI；重复反馈不重复积分，长间隔不补积累的时间。
 * 电流增大保留5mA/ms斜坡，减小可立即执行；软上界立即给零并清积分。
 * 返回非负电流幅值，上层再施加固定节点极性；不会自动启动已停止会话。 */
int32_t FlywheelSpeed_Step(FlywheelSpeed *speed, uint32_t now_ms,
                           int32_t erpm, uint32_t feedback_ms,
                           int32_t current_limit_mA);
#endif
