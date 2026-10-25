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
