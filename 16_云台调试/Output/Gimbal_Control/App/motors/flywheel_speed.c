/**
 * @file flywheel_speed.c
 * @brief 摩擦轮速度 PI 与现用速度斜坡
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（5）：
 *   FlywheelSpeed_Init / FlywheelSpeed_Start / FlywheelSpeed_Stop / FlywheelSpeed_RecordFeedback /
 *   FlywheelSpeed_Step
 * 内部函数（3）：
 *   magnitude / clamp / add_saturated
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 共用轮速 PI：目标爬升、观测、限幅及抗饱和。 */
#include "flywheel_speed.h"
#include <string.h>

static uint32_t magnitude(int32_t erpm)
/* 功能：计算转速或电流的无符号幅值，用于运行统计和边界比较。 */
{
    /* 包括INT_MIN，不能对有符号最小值直接取负。 */
    return erpm >= 0 ? (uint32_t)erpm : (uint32_t)(-1 - erpm) + 1U;
}

static float clamp(float value, float low, float high)
/* 功能：把数值限制在指定上下界内，避免控制请求越界。 */
{
    return value < low ? low : (value > high ? high : value);
}

static uint32_t add_saturated(uint32_t value, uint32_t increment)
/* 功能：累加统计计数，达到整数上限后保持饱和，避免回绕。 */
{
    return increment > UINT32_MAX - value ? UINT32_MAX : value + increment;
}

void FlywheelSpeed_Init(FlywheelSpeed *speed)
/* 功能：载入现用摩擦轮速度环参数并初始化滤波、积分和统计状态。 */
{
    if (speed != 0) { memset(speed, 0, sizeof(*speed)); }
}

void FlywheelSpeed_Start(FlywheelSpeed *speed, uint32_t now_ms,
                        int32_t erpm, uint32_t feedback_ms)
/* 功能：以当前反馈建立一次速度运行会话，初始化目标斜坡和计时。 */
{
    if (speed == 0) { return; }
    FlywheelSpeed_Init(speed);
    speed->running = 1U;
    speed->started_ms = speed->last_update_ms = now_ms;
    speed->last_call_ms = now_ms;
    speed->last_feedback_ms = feedback_ms;
    speed->filtered_erpm = (float)magnitude(erpm);
    FlywheelSpeed_RecordFeedback(speed, erpm, feedback_ms);
}

void FlywheelSpeed_Stop(FlywheelSpeed *speed)
/* 功能：清除当前速度控制请求与积分，并保留需要读取的运行统计。 */
{
    if (speed == 0) { return; }
    speed->running = speed->coasting = speed->at_limit = speed->in_band = 0U;
    speed->requested_mA = 0;
    speed->target_erpm = speed->integral_mA = 0.0f;
    speed->band_ms = 0U;
}

void FlywheelSpeed_RecordFeedback(FlywheelSpeed *speed, int32_t erpm, uint32_t feedback_ms)
/* 功能：记录新反馈对应的达速、带内保持、峰值和限流统计。 */
{
    uint32_t value, since_start;
    if (speed == 0 || !speed->running) { return; }
    value = magnitude(erpm);
    if (value > speed->peak_erpm) { speed->peak_erpm = value; }
    since_start = feedback_ms - speed->started_ms;
    if (!speed->rise90_seen && since_start < 0x80000000UL &&
        value >= (uint32_t)APP_FLYWHEEL_SPEED_TARGET_ERPM * 9U / 10U) {
        speed->rise90_ms = since_start;
        speed->rise90_seen = 1U;
    }
}

int32_t FlywheelSpeed_Step(FlywheelSpeed *speed, uint32_t now_ms,
                           int32_t erpm, uint32_t feedback_ms,
                           int32_t current_limit_mA)
/* 功能：推进现用速度斜坡和滤波 PI，结合软滑行与电流限制返回 mA 请求。 */
{
    uint32_t elapsed, raw, call_elapsed;
    float error, proportional, integral, wanted, limit, alpha, band_low, band_high;
    if (speed == 0 || !speed->running) { return 0; }
    if (current_limit_mA <= 0) { FlywheelSpeed_Stop(speed); return 0; }
    if (current_limit_mA > APP_FLYWHEEL_JOG_MAX_MA) {
        current_limit_mA = APP_FLYWHEEL_JOG_MAX_MA;
    }
    limit = (float)current_limit_mA;
    speed->integral_mA = clamp(speed->integral_mA, 0.0f, limit);
    if (speed->requested_mA > current_limit_mA) { speed->requested_mA = current_limit_mA; }
    speed->run_ms = now_ms - speed->started_ms;
    raw = magnitude(erpm);
    FlywheelSpeed_RecordFeedback(speed, erpm, feedback_ms);
    /* 统计逻辑给定/反馈状态的时长，不当成电流实测或机械停转时间。
     * 小周期调用之间按上一给定累计；异常长空窗不补计。 */
    call_elapsed = now_ms - speed->last_call_ms;
    speed->last_call_ms = now_ms;
    if (call_elapsed > 2U * APP_FLYWHEEL_SPEED_PERIOD_MS) { call_elapsed = 0U; }
    if (speed->coasting) { speed->coast_ms = add_saturated(speed->coast_ms, call_elapsed); }
    if (speed->requested_mA == 0) {
        speed->zero_current_ms = add_saturated(speed->zero_current_ms, call_elapsed);
    }
    if (speed->target_erpm >= (float)APP_FLYWHEEL_SPEED_TARGET_ERPM &&
        raw < (uint32_t)APP_FLYWHEEL_SPEED_TARGET_ERPM / 5U && speed->requested_mA > 0) {
        speed->low_speed_ms = add_saturated(speed->low_speed_ms, call_elapsed);
    }

    /* 未滤波的软上界先于PI周期检查；硬超速仍由上层锁存并双轮停机。 */
    if (raw >= APP_FLYWHEEL_SPEED_COAST_ERPM) { speed->coasting = 1U; }
    else if (raw <= APP_FLYWHEEL_SPEED_RESUME_ERPM) { speed->coasting = 0U; }
    if (speed->coasting) {
        speed->requested_mA = 0; speed->integral_mA = 0.0f;
        speed->at_limit = speed->in_band = 0U; speed->band_ms = 0U;
        speed->filtered_erpm = (float)raw;
        speed->last_update_ms = now_ms; speed->last_feedback_ms = feedback_ms;
        return 0;
    }

    elapsed = now_ms - speed->last_update_ms;
    if (elapsed < APP_FLYWHEEL_SPEED_PERIOD_MS ||
        feedback_ms == speed->last_feedback_ms) { return speed->requested_mA; }
    speed->last_update_ms = now_ms; speed->last_feedback_ms = feedback_ms;
    if (elapsed > 2U * APP_FLYWHEEL_SPEED_PERIOD_MS) {
        /* 反馈/调度有空窗时不追补斜坡、积分或连续稳定时长。 */
        elapsed = APP_FLYWHEEL_SPEED_PERIOD_MS;
        speed->band_ms = 0U;
    }
    speed->target_erpm += (float)APP_FLYWHEEL_SPEED_RAMP_ERPMS * (float)elapsed / 1000.0f;
    speed->target_erpm = clamp(speed->target_erpm, 0.0f, (float)APP_FLYWHEEL_SPEED_TARGET_ERPM);
    alpha = (float)elapsed / ((float)APP_FLYWHEEL_SPEED_FILTER_MS + (float)elapsed);
    speed->filtered_erpm += alpha * ((float)raw - speed->filtered_erpm);
    error = speed->target_erpm - speed->filtered_erpm;
    proportional = APP_FLYWHEEL_SPEED_KP * error;
    integral = clamp(speed->integral_mA + APP_FLYWHEEL_SPEED_KI * error *
                     (float)elapsed / 1000.0f, 0.0f, limit);
    wanted = proportional + integral;
    /* 已饱和且误差还推向饱和端时，不继续堆积积分。 */
    if (!((wanted > limit && error > 0.0f) || (wanted < 0.0f && error < 0.0f))) {
        speed->integral_mA = integral;
    }
    wanted = clamp(wanted, 0.0f, limit);
    if (wanted > (float)speed->requested_mA + 5.0f * (float)elapsed) {
        wanted = (float)speed->requested_mA + 5.0f * (float)elapsed;
    }
    speed->requested_mA = (int32_t)wanted;
    speed->at_limit = (uint8_t)(speed->requested_mA >= current_limit_mA);
    if (speed->at_limit) { speed->limited_ms = add_saturated(speed->limited_ms, elapsed); }

    band_low = (float)(APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM);
    band_high = (float)(APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM);
    if (speed->target_erpm >= (float)APP_FLYWHEEL_SPEED_TARGET_ERPM &&
        (float)raw >= band_low && (float)raw <= band_high &&
        speed->filtered_erpm >= band_low && speed->filtered_erpm <= band_high) {
        speed->band_ms = add_saturated(speed->band_ms, elapsed);
        if (speed->band_ms > speed->longest_band_ms) { speed->longest_band_ms = speed->band_ms; }
    } else { speed->band_ms = 0U; }
    speed->in_band = (uint8_t)(speed->band_ms >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS);
    if (speed->in_band && !speed->band_first_seen) {
        /* 首次连续入带确认，不宣称此后永不离带或完成机械稳速验收。 */
        speed->band_first_ms = speed->run_ms;
        speed->band_first_seen = 1U;
    }
    return speed->requested_mA;
}
