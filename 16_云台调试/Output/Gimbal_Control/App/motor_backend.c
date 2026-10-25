/**
 * @file motor_backend.c
 * @brief 双摩擦轮控制
 *
 * 职责：管理左右摩擦轮会话、反馈健康、速度环、受限输出及停机确认。
 * 主要函数：
 *   - FlywheelSpeed_*：目标爬升、速度滤波、PI 和电流限幅。
 *   - FlywheelJog_*：点动/持续运行、双轮反馈及故障处理。
 *   - MotorBackend_Apply / OnFrame：对接遥控请求与 CAN 反馈。
 *   - MotorBackend_GetHealth / Stop：提供健康状态并安排双轮停机。
 * 调用关系：控制任务 → MotorBackend → FlywheelJog/Speed → CanBus；反馈反向回流。
 * 维护约束：ERPM 与机械 RPM 按极对数换算；左右电流方向和互锁沿用当前配置。
 * 参数入口：app_config.h；对外声明仍在原头文件中。
 */

/* APP_SECTION_BEGIN flywheel_speed.c */
/* 共用轮速 PI：目标爬升、观测、限幅及抗饱和。 */
#include "flywheel_speed.h"
#include <string.h>

static uint32_t magnitude(int32_t erpm)
{
    /* 包括INT_MIN，不能对有符号最小值直接取负。 */
    return erpm >= 0 ? (uint32_t)erpm : (uint32_t)(-1 - erpm) + 1U;
}

static float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static uint32_t add_saturated(uint32_t value, uint32_t increment)
{
    return increment > UINT32_MAX - value ? UINT32_MAX : value + increment;
}

void FlywheelSpeed_Init(FlywheelSpeed *speed)
{
    if (speed != 0) { memset(speed, 0, sizeof(*speed)); }
}

void FlywheelSpeed_Start(FlywheelSpeed *speed, uint32_t now_ms,
                        int32_t erpm, uint32_t feedback_ms)
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
{
    if (speed == 0) { return; }
    speed->running = speed->coasting = speed->at_limit = speed->in_band = 0U;
    speed->requested_mA = 0;
    speed->target_erpm = speed->integral_mA = 0.0f;
    speed->band_ms = 0U;
}

void FlywheelSpeed_RecordFeedback(FlywheelSpeed *speed, int32_t erpm, uint32_t feedback_ms)
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
/* APP_SECTION_END flywheel_speed.c */

/* APP_SECTION_BEGIN flywheel_jog.c */
/* 双轮会话：反馈健康、方向、软滑行与停机确认。 */
#include "flywheel_jog.h"
#include "app_config.h"
#include <string.h>


static uint8_t age_ok(uint32_t now, uint32_t received, uint32_t limit)
{
    /* 有符号半区之外的时间既不能当新鲜也不能当单调时钟。 */
    uint32_t age = now - received;
    return (uint8_t)(age < APP_TICK_HALF_RANGE && age <= limit);
}

static uint8_t newer(uint32_t next, uint32_t previous)
{
    uint32_t difference = next - previous;
    return (uint8_t)(difference != 0U && difference < APP_TICK_HALF_RANGE);
}

static int32_t read_i32(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) | data[3];
    return value <= 0x7FFFFFFFUL ? (int32_t)value : -1 - (int32_t)(~value);
}

static int16_t read_i16(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 8) | data[1];
    return (int16_t)(value < 0x8000U ? (int32_t)value : (int32_t)value - 65536);
}

static uint8_t magnitude_greater(int32_t value, int32_t limit)
{
    return (uint8_t)(value > limit || value < -limit);
}

static uint32_t feedback_faults(const FlywheelJogFeedback *feedback)
{
    uint32_t faults = FLYWHEEL_JOG_FAULT_NONE;
    if (feedback->has_status1) {
        if (magnitude_greater(feedback->erpm, FLYWHEEL_JOG_MAX_ERPM)) {
            faults |= FLYWHEEL_JOG_FAULT_OVERSPEED;
        }
        if (magnitude_greater(feedback->motor_current_mA,
                              FLYWHEEL_JOG_MAX_FEEDBACK_CURRENT_MA)) {
            faults |= FLYWHEEL_JOG_FAULT_CURRENT;
        }
    }
    if (feedback->has_status4) {
        if (feedback->fet_deci_c >= FLYWHEEL_JOG_MAX_FET_DECI_C) {
            faults |= FLYWHEEL_JOG_FAULT_TEMPERATURE;
        }
        if (magnitude_greater(feedback->input_current_mA,
                              FLYWHEEL_JOG_MAX_FEEDBACK_CURRENT_MA)) {
            faults |= FLYWHEEL_JOG_FAULT_CURRENT;
        }
    }
    return faults;
}

void FlywheelJog_Init(FlywheelJog *jog)
{
    if (jog != 0) { memset(jog, 0, sizeof(*jog)); }
}

void FlywheelJog_Stop(FlywheelJog *jog)
{
    if (jog == 0) { return; }
    if (jog->active && jog->stop_reason == FLYWHEEL_STOP_NONE) {
        jog->stop_reason = FLYWHEEL_STOP_MANUAL;
    }
    jog->current_mA = 0;
    jog->right_current_mA = 0;
    jog->active = 0U;
    jog->ready = 0U;
    jog->reset_seen = 0U;
    jog->coasting = 0U;
#if APP_FLYWHEEL_SPEED_ACTIVE
    FlywheelSpeed_Stop(&jog->speed);
    FlywheelSpeed_Stop(&jog->speed_right);
#endif
    jog->pending_mask = FLYWHEEL_BOTH_NODES;
}

void FlywheelJog_Fault(FlywheelJog *jog, uint32_t faults)
{
    if (jog == 0 || faults == 0U) { return; }
    jog->faults |= faults;
    jog->stop_reason = FLYWHEEL_STOP_FAULT;
    FlywheelJog_Stop(jog);
}

static void synchronize_epoch(FlywheelJog *jog, uint32_t can_epoch)
{
    if (!jog->has_epoch || jog->can_epoch != can_epoch) {
        uint8_t had_epoch = jog->has_epoch;
        jog->can_epoch = can_epoch;
        jog->has_epoch = 1U;
        memset(jog->feedback, 0, sizeof(jog->feedback));
        if (jog->selection != FLYWHEEL_JOG_OFF && (had_epoch || can_epoch != 0U)) {
            FlywheelJog_Fault(jog, FLYWHEEL_JOG_FAULT_CAN_EPOCH);
        }
    }
    if (can_epoch != 0U && jog->selection != FLYWHEEL_JOG_OFF) {
        FlywheelJog_Fault(jog, FLYWHEEL_JOG_FAULT_CAN_EPOCH);
    } else if (can_epoch != 0U) {
        /* 未选轮只锁状态，不排CAN命令；不能把epoch重置0现场解锁。 */
        jog->faults |= FLYWHEEL_JOG_FAULT_CAN_EPOCH;
    }
}

static uint8_t damaged(FlywheelJog *jog, FlywheelJogFeedback *feedback,
                       uint32_t packet)
{
    if (packet == FLYWHEEL_STATUS1_PACKET) { feedback->has_status1 = 0U; }
    else { feedback->has_status4 = 0U; }
    if (jog->selection != FLYWHEEL_JOG_OFF) {
        FlywheelJog_Fault(jog, FLYWHEEL_JOG_FAULT_FRAME);
    }
    return 0U;
}

uint8_t FlywheelJog_OnFrame(FlywheelJog *jog, const CanFrame *frame,
                          uint32_t now_ms, uint32_t can_epoch)
{
    FlywheelJogFeedback *feedback;
    uint32_t packet, previous, limit, faults;
    uint8_t node, seen;
    int16_t duty, fet;
    if (jog == 0) { return 0U; }
    synchronize_epoch(jog, can_epoch);
    if (frame == 0 || frame->bus != APP_FLYWHEEL_CAN_BUS ||
        frame->extended != 1U || frame->id > 0x1FFFFFFFU ||
        frame->fault_epoch != can_epoch) { return 0U; }
    node = (uint8_t)(frame->id & 0xFFU);
    if (node != APP_FLYWHEEL_LEFT_ID && node != APP_FLYWHEEL_RIGHT_ID) { return 0U; }
    packet = frame->id >> 8;
    if (packet != FLYWHEEL_STATUS1_PACKET && packet != FLYWHEEL_STATUS4_PACKET) { return 0U; }
    feedback = &jog->feedback[node == APP_FLYWHEEL_LEFT_ID ? 0U : 1U];
    previous = packet == FLYWHEEL_STATUS1_PACKET ? feedback->status1_ms : feedback->status4_ms;
    seen = packet == FLYWHEEL_STATUS1_PACKET ? feedback->seen_status1 : feedback->seen_status4;
    limit = packet == FLYWHEEL_STATUS1_PACKET ? FLYWHEEL_JOG_STATUS1_TIMEOUT_MS :
                                     FLYWHEEL_JOG_STATUS4_TIMEOUT_MS;
    /* 重复/旧包不刷新原始接收时间。错误DLC不读取任何payload。 */
    if (seen && !newer(frame->received_ms, previous)) { return 0U; }
    if (!age_ok(now_ms, frame->received_ms, limit)) { return 0U; }
    if (packet == FLYWHEEL_STATUS1_PACKET) {
        feedback->status1_ms = frame->received_ms;
        feedback->seen_status1 = 1U;
    } else {
        feedback->status4_ms = frame->received_ms;
        feedback->seen_status4 = 1U;
    }
    if (frame->dlc != 8U) { return damaged(jog, feedback, packet); }
    if (packet == FLYWHEEL_STATUS1_PACKET) {
        duty = read_i16(frame->data + 6U);
        if (duty < -1000 || duty > 1000) { return damaged(jog, feedback, packet); }
        feedback->erpm = read_i32(frame->data);
        feedback->motor_current_mA = (int32_t)read_i16(frame->data + 4U) * 100;
        feedback->duty_milli = duty;
        feedback->status1_ms = frame->received_ms;
        feedback->has_status1 = 1U;
#if APP_FLYWHEEL_SPEED_ACTIVE
        if (jog->active) {
            if (jog->selection == (node == APP_FLYWHEEL_LEFT_ID ?
                FLYWHEEL_JOG_LEFT : FLYWHEEL_JOG_RIGHT)) {
                FlywheelSpeed_RecordFeedback(&jog->speed, feedback->erpm, feedback->status1_ms);
            } else if (jog->selection == FLYWHEEL_JOG_BOTH) {
                FlywheelSpeed_RecordFeedback(node == APP_FLYWHEEL_LEFT_ID ?
                    &jog->speed : &jog->speed_right, feedback->erpm, feedback->status1_ms);
            }
        }
#endif
    } else {
        fet = read_i16(frame->data);
        /* FET负值不是可用于放行的传感器读数；motor负值是缺失值。 */
        if (fet < 0) { return damaged(jog, feedback, packet); }
        feedback->fet_deci_c = fet;
        feedback->motor_deci_c = read_i16(frame->data + 2U);
        feedback->motor_temperature_valid = (uint8_t)(feedback->motor_deci_c >= 0);
        feedback->input_current_mA = (int32_t)read_i16(frame->data + 4U) * 100;
        feedback->status4_ms = frame->received_ms;
        feedback->has_status4 = 1U;
    }
    faults = feedback_faults(feedback);
    if (faults != 0U && jog->selection != FLYWHEEL_JOG_OFF) {
        FlywheelJog_Fault(jog, faults);
    }
    return 1U;
}

void FlywheelJog_GetHealth(FlywheelJog *jog, uint32_t now_ms,
                          uint32_t can_epoch, FlywheelJogHealth *health)
{
    uint8_t index;
    if (health == 0) { return; }
    memset(health, 0, sizeof(*health));
    if (jog == 0) { return; }
    synchronize_epoch(jog, can_epoch);
    memcpy(health->feedback, jog->feedback, sizeof(health->feedback));
    health->faults = jog->faults;
    if (can_epoch != 0U) { health->faults |= FLYWHEEL_JOG_FAULT_CAN_EPOCH; }
    health->stopped = 1U;
    for (index = 0U; index < 2U; ++index) {
        const FlywheelJogFeedback *feedback = &jog->feedback[index];
        uint8_t bit = (uint8_t)(1U << index);
        if (feedback->has_status1 && age_ok(now_ms, feedback->status1_ms,
                                          FLYWHEEL_JOG_STATUS1_TIMEOUT_MS)) {
            health->status1_fresh_mask |= bit;
        }
        if (feedback->has_status4 && age_ok(now_ms, feedback->status4_ms,
                                          FLYWHEEL_JOG_STATUS4_TIMEOUT_MS)) {
            health->status4_fresh_mask |= bit;
        }
        if (!feedback->has_status1 || feedback->erpm >= FLYWHEEL_JOG_STOPPED_ERPM ||
            feedback->erpm <= -FLYWHEEL_JOG_STOPPED_ERPM) { health->stopped = 0U; }
        health->faults |= feedback_faults(feedback);
    }
    health->fresh = (uint8_t)(health->status1_fresh_mask == FLYWHEEL_BOTH_NODES &&
                             health->status4_fresh_mask == FLYWHEEL_BOTH_NODES);
    if (!health->fresh) { health->faults |= FLYWHEEL_JOG_FAULT_FEEDBACK; }
    if (health->status1_fresh_mask != FLYWHEEL_BOTH_NODES) { health->stopped = 0U; }
    health->valid = (uint8_t)(health->fresh && health->faults == 0U);
}

uint8_t FlywheelJog_Select(FlywheelJog *jog, FlywheelJogSelection selection,
                         uint32_t now_ms, uint32_t can_epoch)
{
    FlywheelJogHealth health;
    if (jog == 0 || (selection != FLYWHEEL_JOG_OFF &&
        selection != FLYWHEEL_JOG_LEFT && selection != FLYWHEEL_JOG_RIGHT &&
        selection != FLYWHEEL_JOG_BOTH)) { return 0U; }
    if (selection == FLYWHEEL_JOG_BOTH && !APP_FLYWHEEL_SPEED_ACTIVE) { return 0U; }
    synchronize_epoch(jog, can_epoch);
    if (selection == FLYWHEEL_JOG_OFF) {
        FlywheelJog_Stop(jog);
        jog->selection = FLYWHEEL_JOG_OFF;
        return 1U;
    }
    FlywheelJog_GetHealth(jog, now_ms, can_epoch, &health);
    if (jog->selection != FLYWHEEL_JOG_OFF || jog->last_allowed ||
        !jog->last_centered || jog->active || jog->current_mA != 0 || jog->right_current_mA != 0 ||
        !health.valid || !health.stopped || can_epoch != 0U) { return 0U; }
    jog->selection = selection;
    jog->selected_ms = now_ms;
    jog->ready = 0U;
    jog->reset_seen = 0U;
    jog->pending_mask = FLYWHEEL_BOTH_NODES;
    return 1U;
}

void FlywheelJog_Step(FlywheelJog *jog, uint32_t now_ms,
                     uint8_t output_allowed, uint8_t positive_request,
                     uint8_t centered, uint32_t can_epoch)
{
    FlywheelJogHealth health;
    uint8_t rising;
#if !APP_FLYWHEEL_SPEED_ACTIVE
    uint32_t elapsed, increment;
#endif
    int32_t current_limit;
#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U && !APP_FLYWHEEL_SPEED_ACTIVE
    int32_t observation_erpm;
#endif
    if (jog == 0) { return; }
    output_allowed = (uint8_t)(output_allowed != 0U);
    positive_request = (uint8_t)(positive_request != 0U);
    centered = (uint8_t)(centered != 0U && !positive_request);
    rising = (uint8_t)(output_allowed && !jog->last_allowed);
    current_limit = jog->selection != FLYWHEEL_JOG_RIGHT ? APP_FLYWHEEL_JOG_LEFT_MAX_MA :
                    APP_FLYWHEEL_JOG_RIGHT_MAX_MA;
    FlywheelJog_GetHealth(jog, now_ms, can_epoch, &health);

    if (jog->selection != FLYWHEEL_JOG_OFF && !health.valid) {
        FlywheelJog_Fault(jog, health.faults);
    }
    if (!output_allowed) {
        if (jog->active || jog->ready || jog->current_mA != 0 || jog->right_current_mA != 0) {
            if (jog->active && jog->stop_reason == FLYWHEEL_STOP_NONE) {
                jog->stop_reason = FLYWHEEL_STOP_PERMISSION;
            }
            FlywheelJog_Stop(jog);
        }
        if (centered && !jog->faults && newer(now_ms, jog->selected_ms)) {
            jog->reset_seen = 1U;
        }
    } else if (rising) {
        jog->ready = (uint8_t)(jog->reset_seen && centered &&
            jog->selection != FLYWHEEL_JOG_OFF && health.valid && health.stopped);
        jog->reset_seen = 0U;
    }

    if (output_allowed && jog->selection != FLYWHEEL_JOG_OFF && !jog->faults) {
        if (jog->active) {
            /* 按住观察仅去掉计时结束；实时请求消失仍立即进入原停机路径。
             * 预处理分支保留500ms回归测试，避免无符号数与0比较的恒真条件。 */
            if (!positive_request
#if FLYWHEEL_JOG_MAX_DURATION_MS > 0U
                || now_ms - jog->started_ms >= FLYWHEEL_JOG_MAX_DURATION_MS
#endif
                ) {
                jog->stop_reason = !positive_request ? FLYWHEEL_STOP_INPUT : FLYWHEEL_STOP_DURATION;
                FlywheelJog_Stop(jog);
            } else {
#if APP_FLYWHEEL_SPEED_ACTIVE
                const FlywheelJogFeedback *selected = &jog->feedback[
                    jog->selection != FLYWHEEL_JOG_RIGHT ? 0U : 1U];
                jog->current_mA = FlywheelSpeed_Step(&jog->speed, now_ms,
                    selected->erpm, selected->status1_ms, current_limit);
                jog->coasting = jog->speed.coasting;
                if (jog->selection == FLYWHEEL_JOG_BOTH) {
                    jog->right_current_mA = FlywheelSpeed_Step(&jog->speed_right, now_ms,
                        jog->feedback[1].erpm, jog->feedback[1].status1_ms,
                        APP_FLYWHEEL_JOG_RIGHT_MAX_MA);
                    jog->coasting = (uint8_t)(jog->coasting || jog->speed_right.coasting);
                } else { jog->right_current_mA = 0; }
#else
                elapsed = now_ms - jog->last_ramp_ms;
#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U
                observation_erpm = jog->feedback[jog->selection == FLYWHEEL_JOG_LEFT ? 0U : 1U].erpm;
                if (observation_erpm >= FLYWHEEL_OBSERVE_COAST_ERPM ||
                    observation_erpm <= -FLYWHEEL_OBSERVE_COAST_ERPM) {
                    jog->coasting = 1U;
                } else if (observation_erpm <= FLYWHEEL_OBSERVE_RESUME_ERPM &&
                           observation_erpm >= -FLYWHEEL_OBSERVE_RESUME_ERPM) {
                    jog->coasting = 0U;
                }
                /* 滑行是本次观察内的零给定，不结束会话、不解除遥控保护。
                 * 零给定时也更新时间，下界恢复后不能补发累积斜坡。 */
                if (jog->coasting) { jog->current_mA = 0; }
                else {
#endif
                /* 5mA/ms = 100mA/20ms；先限乘数以免大tick差溢出。 */
                increment = elapsed > (uint32_t)current_limit / 5U ?
                            (uint32_t)current_limit : elapsed * 5U;
                jog->current_mA += (int32_t)increment;
                if (jog->current_mA > current_limit) {
                    jog->current_mA = current_limit;
                }
#if FLYWHEEL_JOG_MAX_DURATION_MS == 0U
                }
#endif
                jog->last_ramp_ms = now_ms;
#endif
            }
        } else if (jog->ready && positive_request) {
            if (!health.stopped) {
                /* 等待起动期间轮子已转动，要求完整重新触发。 */
                jog->stop_reason = FLYWHEEL_STOP_NOT_STOPPED;
                FlywheelJog_Stop(jog);
            } else {
                jog->active = 1U;
                jog->ready = 0U;
                jog->started_ms = now_ms;
                jog->last_ramp_ms = now_ms;
                jog->current_mA = 0;
                jog->right_current_mA = 0;
                jog->stop_reason = FLYWHEEL_STOP_NONE;
#if APP_FLYWHEEL_SPEED_ACTIVE
                jog->last_speed_selection = jog->selection;
                FlywheelSpeed_Start(&jog->speed, now_ms,
                    jog->feedback[jog->selection != FLYWHEEL_JOG_RIGHT ? 0U : 1U].erpm,
                    jog->feedback[jog->selection != FLYWHEEL_JOG_RIGHT ? 0U : 1U].status1_ms);
                FlywheelSpeed_Init(&jog->speed_right);
                if (jog->selection == FLYWHEEL_JOG_BOTH) {
                    FlywheelSpeed_Start(&jog->speed_right, now_ms,
                        jog->feedback[1].erpm, jog->feedback[1].status1_ms);
                }
#endif
                jog->pending_mask = FLYWHEEL_BOTH_NODES;
            }
        }
    }
    if (jog->selection != FLYWHEEL_JOG_OFF && (!jog->command_started ||
        now_ms - jog->last_command_ms >= FLYWHEEL_JOG_COMMAND_PERIOD_MS)) {
        jog->command_started = 1U;
        jog->last_command_ms = now_ms;
        jog->pending_mask = FLYWHEEL_BOTH_NODES;
    }
    jog->last_allowed = output_allowed;
    jog->last_centered = centered;
}

int32_t FlywheelJog_GetRequestedCurrent(const FlywheelJog *jog)
{
    if (jog == 0) { return 0; }
    if (jog->selection == FLYWHEEL_JOG_LEFT || jog->selection == FLYWHEEL_JOG_BOTH) {
        return jog->current_mA * APP_FLYWHEEL_LEFT_CURRENT_SIGN;
    }
    if (jog->selection == FLYWHEEL_JOG_RIGHT) {
        return jog->current_mA * APP_FLYWHEEL_RIGHT_CURRENT_SIGN;
    }
    return 0;
}

int32_t FlywheelJog_GetNodeCurrent(const FlywheelJog *jog, uint8_t node)
{
    if (jog == 0) { return 0; }
    if (node == APP_FLYWHEEL_LEFT_ID &&
        (jog->selection == FLYWHEEL_JOG_LEFT || jog->selection == FLYWHEEL_JOG_BOTH)) {
        return jog->current_mA * APP_FLYWHEEL_LEFT_CURRENT_SIGN;
    }
    if (node == APP_FLYWHEEL_RIGHT_ID) {
        if (jog->selection == FLYWHEEL_JOG_RIGHT) {
            return jog->current_mA * APP_FLYWHEEL_RIGHT_CURRENT_SIGN;
        }
        if (jog->selection == FLYWHEEL_JOG_BOTH) {
            return jog->right_current_mA * APP_FLYWHEEL_RIGHT_CURRENT_SIGN;
        }
    }
    return 0;
}

uint8_t FlywheelJog_TakeCommand(FlywheelJog *jog, uint8_t *node,
                               int32_t *current_mA)
{
    uint8_t index, bit;
    if (jog == 0 || node == 0 || current_mA == 0 || jog->pending_mask == 0U) { return 0U; }
    index = (uint8_t)((jog->pending_mask & 1U) ? 0U : 1U);
    bit = (uint8_t)(1U << index);
    jog->pending_mask &= (uint8_t)~bit;
    *node = index == 0U ? APP_FLYWHEEL_LEFT_ID : APP_FLYWHEEL_RIGHT_ID;
    *current_mA = FlywheelJog_GetNodeCurrent(jog, *node);
    return 1U;
}

uint8_t FlywheelJog_ClearFault(FlywheelJog *jog, uint32_t now_ms,
                             uint32_t can_epoch)
{
    FlywheelJogHealth health;
    uint32_t saved_faults;
    if (jog == 0 || jog->selection != FLYWHEEL_JOG_OFF || jog->last_allowed ||
        !jog->last_centered || jog->active || jog->current_mA != 0 || jog->right_current_mA != 0 ||
        can_epoch != 0U || (jog->faults & FLYWHEEL_JOG_FAULT_CAN_EPOCH) != 0U) { return 0U; }
    saved_faults = jog->faults;
    jog->faults = 0U;
    FlywheelJog_GetHealth(jog, now_ms, can_epoch, &health);
    if (!health.valid || !health.stopped) {
        jog->faults = saved_faults;
        return 0U;
    }
    jog->ready = 0U;
    jog->reset_seen = 0U;
    return 1U;
}
/* APP_SECTION_END flywheel_jog.c */

/* APP_SECTION_BEGIN motor_backend.c */
/* 双摩擦轮后端：遥控准备和应用层接口。 */
#include "motor_backend.h"
#include "app_config.h"
#include <string.h>

#if APP_FLYWHEEL_JOG_ENABLED
#include "can_bus.h"
#include "task.h"
static FlywheelJog jog;
/* owned保留到复位：DIAG取消邮箱后仍须重发停机双零。 */
static uint8_t initialized, owned, stop_pending, stop_attempted, rearm_requested;
static uint32_t last_stop_ms;
#if APP_FEEDER_ENABLED && APP_FLYWHEEL_SPEED_ACTIVE
/* 独立拨盘互锁计时，不改轮子PI/斜坡/统计。 */
static uint32_t wheel_stable_since[2], wheel_sample_ms[2];
static uint8_t wheel_stable_mask;
#endif
#if APP_REMOTE_PROFILE_ASSESSMENT
static uint8_t remote_primed, have_prepare_sequence;
static uint32_t prepare_sequence;
#endif

void MotorBackend_Init(void)
{
    taskENTER_CRITICAL();
    FlywheelJog_Init(&jog);
    initialized = 1U; owned = stop_pending = stop_attempted = rearm_requested = 0U;
    last_stop_ms = 0U;
#if APP_FEEDER_ENABLED && APP_FLYWHEEL_SPEED_ACTIVE
    memset(wheel_stable_since, 0, sizeof(wheel_stable_since));
    memset(wheel_sample_ms, 0, sizeof(wheel_sample_ms)); wheel_stable_mask = 0U;
#endif
#if APP_REMOTE_PROFILE_ASSESSMENT
    remote_primed = have_prepare_sequence = 0U;
    prepare_sequence = 0U;
#endif
    taskEXIT_CRITICAL();
}

void MotorBackend_GetJogState(MotorJogState *out)
{
    CanBusState bus;
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    memset(out, 0, sizeof(*out));
    CanBus_GetState(&bus);
    FlywheelJog_GetHealth(&jog, HAL_GetTick(), bus.fault_epoch, &out->health);
    out->selection = jog.selection; out->requested_mA = FlywheelJog_GetRequestedCurrent(&jog);
    out->active = jog.active; out->rearm_ready = jog.ready; out->stop_pending = stop_pending;
    out->speed = jog.speed; out->last_speed_selection = jog.last_speed_selection;
    out->speed_right = jog.last_speed_selection == FLYWHEEL_JOG_RIGHT ? jog.speed : jog.speed_right;
    out->requested_left_mA = FlywheelJog_GetNodeCurrent(&jog, APP_FLYWHEEL_LEFT_ID);
    out->requested_right_mA = FlywheelJog_GetNodeCurrent(&jog, APP_FLYWHEEL_RIGHT_ID);
    out->stop_reason = jog.stop_reason;
    taskEXIT_CRITICAL();
}

void MotorBackend_GetHealth(MotorHealth *health)
{
    MotorJogState state;
    CanBusState bus;
    if (health == 0) { return; }
    taskENTER_CRITICAL();
    MotorBackend_GetJogState(&state); CanBus_GetState(&bus);
    memset(health, 0, sizeof(*health));
    health->ready = initialized && state.selection != FLYWHEEL_JOG_OFF && !stop_pending;
    health->feedback_ready = state.health.valid && bus.started && bus.fault_epoch == 0U;
    health->actuator_fault = state.selection != FLYWHEEL_JOG_OFF && state.health.faults != 0U;
#if APP_FLYWHEEL_SPEED_ACTIVE
    /* 实际双轮均连续入带500ms且最新原始反馈也在带内；不以目标值代替反馈。 */
    health->flywheels_ready = health->ready && health->feedback_ready &&
        !health->actuator_fault && state.selection == FLYWHEEL_JOG_BOTH && state.active &&
        state.speed.running && state.speed_right.running &&
        state.speed.in_band && state.speed_right.in_band &&
#if APP_FEEDER_ENABLED
        wheel_stable_mask == 3U &&
        HAL_GetTick() - wheel_stable_since[0] >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS &&
        HAL_GetTick() - wheel_stable_since[1] >= APP_FLYWHEEL_SPEED_BAND_HOLD_MS &&
#endif
        !state.speed.coasting && !state.speed_right.coasting &&
        state.health.feedback[0].erpm >= APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM &&
        state.health.feedback[0].erpm <= APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM &&
        state.health.feedback[1].erpm >= APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM &&
        state.health.feedback[1].erpm <= APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM;
#endif
    taskEXIT_CRITICAL();
}

uint8_t MotorBackend_OnFrame(const CanFrame *frame)
{
    uint8_t accepted;
    uint32_t previous_faults;
    CanBusState bus;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    previous_faults = jog.faults;
    accepted = FlywheelJog_OnFrame(&jog, frame, HAL_GetTick(), bus.fault_epoch);
#if APP_FEEDER_ENABLED && APP_FLYWHEEL_SPEED_ACTIVE
    /* 新鲜STATUS1短暂离带也打断独立500ms稳速窗口，不等20ms PI更新。 */
    if (accepted && frame && frame->extended && frame->bus == APP_FLYWHEEL_CAN_BUS &&
        (frame->id >> 8) == 9U) {
        uint8_t index;
        for (index = 0U; index < 2U; ++index) {
            uint8_t bit = (uint8_t)(1U << index);
            uint8_t node = index ? APP_FLYWHEEL_RIGHT_ID : APP_FLYWHEEL_LEFT_ID;
            int32_t erpm = jog.feedback[index].erpm;
            if ((frame->id & 255U) != node) { continue; }
            if (!jog.active || erpm < APP_FLYWHEEL_SPEED_TARGET_ERPM - APP_FLYWHEEL_SPEED_BAND_ERPM ||
                erpm > APP_FLYWHEEL_SPEED_TARGET_ERPM + APP_FLYWHEEL_SPEED_BAND_ERPM) {
                wheel_stable_mask &= (uint8_t)~bit;
            } else {
                if (!(wheel_stable_mask & bit) ||
                    frame->received_ms - wheel_sample_ms[index] >= FLYWHEEL_JOG_STATUS1_TIMEOUT_MS) {
                    wheel_stable_since[index] = frame->received_ms;
                }
                wheel_stable_mask |= bit;
            }
            wheel_sample_ms[index] = frame->received_ms;
        }
    }
#endif
    if (owned && jog.selection != FLYWHEEL_JOG_OFF && jog.faults != previous_faults) {
        CanBus_SetOutputPermission(0U);
        stop_pending = 1U; stop_attempted = 0U; rearm_requested = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void MotorBackend_Stop(void)
{
    taskENTER_CRITICAL();
#if APP_FEEDER_ENABLED && APP_FLYWHEEL_SPEED_ACTIVE
    wheel_stable_mask = 0U;
#endif
#if APP_REMOTE_PROFILE_ASSESSMENT
    remote_primed = 0U;
#endif
    if (owned) {
        FlywheelJog_Stop(&jog);
        stop_pending = 1U; stop_attempted = 0U;
    }
    taskEXIT_CRITICAL();
}

uint8_t MotorBackend_PrepareRemote(const RemoteFrame *remote,
    uint32_t barrier_ms, uint8_t barrier_valid)
{
#if APP_REMOTE_PROFILE_ASSESSMENT
    RemoteCommand command;
    uint32_t now = HAL_GetTick();
    uint8_t result = 0U;
    if (remote == 0 || !remote->has_frame ||
        (uint32_t)(now - remote->received_ms) >= REMOTE_TIMEOUT_MS ||
        (barrier_valid && (int32_t)(remote->received_ms - barrier_ms) <= 0)) { return 0U; }
    RemoteMapper_Map(remote, &command);
    if (!command.valid || !command.failsafe_ok || command.arm != RC_LOW || !command.neutral) {
        return 0U;
    }
    taskENTER_CRITICAL();
    if (!have_prepare_sequence || prepare_sequence != remote->sequence) {
        have_prepare_sequence = 1U; prepare_sequence = remote->sequence;
        CanBus_SetOutputPermission(0U);
        if (!jog.active && !jog.last_allowed && !stop_pending) {
            if (jog.selection != FLYWHEEL_JOG_OFF &&
                (jog.selection != FLYWHEEL_JOG_BOTH || jog.faults)) {
                result = MotorBackend_SelectJog(FLYWHEEL_JOG_OFF);
            } else if (jog.selection == FLYWHEEL_JOG_OFF) {
                result = MotorBackend_SelectJog(FLYWHEEL_JOG_BOTH);
            }
        }
    }
    taskEXIT_CRITICAL();
    return result;
#else
    (void)remote; (void)barrier_ms; (void)barrier_valid;
    return 0U;
#endif
}

uint8_t MotorBackend_SelectJog(FlywheelJogSelection selection)
{
    CanBusState bus;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    CanBus_GetState(&bus);
    if (selection == FLYWHEEL_JOG_OFF) {
        accepted = FlywheelJog_Select(&jog, selection, HAL_GetTick(), bus.fault_epoch);
        if (owned) { stop_pending = 1U; stop_attempted = 0U; }
    } else if (initialized && !stop_pending && bus.started && !bus.fault_epoch &&
               !CanBus_OutputAllowed()) {
        if (jog.faults) { (void)FlywheelJog_ClearFault(&jog, HAL_GetTick(), bus.fault_epoch); }
        accepted = FlywheelJog_Select(&jog, selection, HAL_GetTick(), bus.fault_epoch);
        if (accepted) { owned = 1U; stop_pending = 1U; stop_attempted = 0U; }
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint8_t MotorBackend_TakeRearmRequest(void)
{
    uint8_t requested;
    taskENTER_CRITICAL();
    requested = rearm_requested; rearm_requested = 0U;
    taskEXIT_CRITICAL();
    return requested;
}

uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t output_allowed)
{
    CanBusState bus;
    uint8_t was_active, nodes[2], count = 0U, index, submitted = 0U;
    int32_t currents[2];
    uint32_t now;
#if APP_REMOTE_PROFILE_ASSESSMENT
    uint8_t wanted, ordinary_off;
#endif
    if (request == 0) { MotorBackend_Stop(); return 0U; }
    /* 外层临界区核对遥控、STOP及UART故障代次。Step->Take->Send不排队，
     * 避免旧非零电流迟到发送；这里的HAL CAN操作不等待总线发送完成。 */
    taskENTER_CRITICAL();
    now = HAL_GetTick(); CanBus_GetState(&bus); was_active = jog.active;
#if APP_REMOTE_PROFILE_ASSESSMENT
    wanted = request->friction_requested && request->gimbal_mode == GIMBAL_MANUAL;
    ordinary_off = output_allowed && !wanted;
    output_allowed = output_allowed && wanted && !stop_pending;
    if (!output_allowed) {
        remote_primed = 0U;
        FlywheelJog_Step(&jog, now, 0U, 0U, request->wheel_reset_allowed, bus.fault_epoch);
    } else if (!jog.active && !remote_primed) {
        /* 适配既有会话接口：ON边沿先以零请求确认一次就绪，下一周期才运行。
         * 未停稳时本次ON不会就绪，须用户CH6 OFF后再次ON，不能迟到自启。 */
        FlywheelJog_Step(&jog, now, 1U, 0U, 1U, bus.fault_epoch);
        remote_primed = 1U;
    } else {
        FlywheelJog_Step(&jog, now, 1U, 1U, 0U, bus.fault_epoch);
    }
#else
    FlywheelJog_Step(&jog, now, output_allowed && !stop_pending,
                    request->yaw_rate_dps > 0.0f, request->yaw_rate_dps == 0.0f,
                    bus.fault_epoch);
#endif
    if (was_active && !jog.active) {
#if APP_REMOTE_PROFILE_ASSESSMENT
        /* CH6正常关闭或进入测试不取消总解锁；故障/撤权仍要求CH5重解锁。 */
        if (!ordinary_off || jog.faults) { rearm_requested = 1U; }
        stop_pending = owned; stop_attempted = 0U;
#else
        rearm_requested = 1U; stop_pending = owned; stop_attempted = 0U;
#endif
    }
    while (count < 2U && FlywheelJog_TakeCommand(&jog, &nodes[count], &currents[count])) { count++; }
#if APP_FEEDER_ENABLED && APP_FLYWHEEL_SPEED_ACTIVE
    if (!jog.active) { wheel_stable_mask = 0U; }
#endif
    if (stop_pending) {
        /* 停机提交失败保留pending，每20ms重试；此期间禁止非零输出。 */
        if (!stop_attempted || (uint32_t)(now - last_stop_ms) >= FLYWHEEL_JOG_COMMAND_PERIOD_MS) {
            stop_attempted = 1U; last_stop_ms = now;
            if (CanBus_StopFlywheels() == HAL_OK) { stop_pending = 0U; }
#if APP_REMOTE_PROFILE_ASSESSMENT
            else { rearm_requested = 1U; } /* 停机提交失败恢复后不得自动重新加力。 */
#endif
        }
    } else if (owned && !output_allowed && (!stop_attempted ||
               (uint32_t)(now - last_stop_ms) >= FLYWHEEL_JOG_COMMAND_PERIOD_MS)) {
        HAL_StatusTypeDef result;
        /* 初次STOP已取消旧正流。后续双零不abort，只读采集可正常完成。
         * 邮箱忙先跳过；真正发送/总线失败才重新进入严格STOP重试。 */
        stop_attempted = 1U; last_stop_ms = now;
        result = CanBus_KeepFlywheelsStopped();
        if (result != HAL_OK && result != HAL_BUSY) { stop_pending = 1U; }
    } else if (owned && jog.selection != FLYWHEEL_JOG_OFF && output_allowed) {
        for (index = 0U; index < count; ++index) {
            if (CanBus_JogVescCurrent(nodes[index], currents[index]) != HAL_OK) {
                FlywheelJog_Fault(&jog, FLYWHEEL_JOG_FAULT_EXTERNAL);
                stop_pending = 1U; stop_attempted = 1U; last_stop_ms = now; rearm_requested = 1U;
                CanBus_SetOutputPermission(0U);
                if (CanBus_StopFlywheels() == HAL_OK) { stop_pending = 0U; }
                submitted = 0U; break;
            }
            submitted = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return submitted;
}
#else
void MotorBackend_Init(void) { }
void MotorBackend_Stop(void) { }
uint8_t MotorBackend_SelectJog(FlywheelJogSelection selection) { return selection == FLYWHEEL_JOG_OFF; }
uint8_t MotorBackend_PrepareRemote(const RemoteFrame *remote,
    uint32_t barrier_ms, uint8_t barrier_valid)
{ (void)remote; (void)barrier_ms; (void)barrier_valid; return 0U; }
uint8_t MotorBackend_TakeRearmRequest(void) { return 0U; }
void MotorBackend_GetJogState(MotorJogState *state) { if (state) { memset(state, 0, sizeof(*state)); } }
void MotorBackend_GetHealth(MotorHealth *health) { if (health) { memset(health, 0, sizeof(*health)); } }
uint8_t MotorBackend_OnFrame(const CanFrame *frame) { (void)frame; return 0U; }
uint8_t MotorBackend_Apply(const TargetRequest *request, uint8_t allowed) { (void)request; (void)allowed; return 0U; }
#endif
/* APP_SECTION_END motor_backend.c */
