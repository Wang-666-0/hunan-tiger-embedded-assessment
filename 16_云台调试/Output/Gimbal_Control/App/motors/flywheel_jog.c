/**
 * @file flywheel_jog.c
 * @brief 摩擦轮运行状态与反馈保护
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（11）：
 *   FlywheelJog_Init / FlywheelJog_Stop / FlywheelJog_Fault / FlywheelJog_OnFrame /
 *   FlywheelJog_GetHealth / FlywheelJog_Select / FlywheelJog_Step / FlywheelJog_GetRequestedCurrent
 *   / FlywheelJog_GetNodeCurrent / FlywheelJog_TakeCommand / FlywheelJog_ClearFault
 * 内部函数（8）：
 *   age_ok / newer / read_i32 / read_i16 / magnitude_greater / feedback_faults / synchronize_epoch /
 *   damaged
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 双轮会话：反馈健康、方向、软滑行与停机确认。 */
#include "flywheel_jog.h"
#include "app_config.h"
#include <string.h>

static uint8_t age_ok(uint32_t now, uint32_t received, uint32_t limit)
/* 功能：检查反馈年龄是否小于允许超时，并处理计时回绕。 */
{
    /* 有符号半区之外的时间既不能当新鲜也不能当单调时钟。 */
    uint32_t age = now - received;
    return (uint8_t)(age < APP_TICK_HALF_RANGE && age <= limit);
}

static uint8_t newer(uint32_t next, uint32_t previous)
/* 功能：比较两个毫秒时间戳，判断是否属于更新的数据。 */
{
    uint32_t difference = next - previous;
    return (uint8_t)(difference != 0U && difference < APP_TICK_HALF_RANGE);
}

static int32_t read_i32(const uint8_t *data)
/* 功能：按摩擦轮反馈协议读取有符号 32 位整数。 */
{
    uint32_t value = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) | data[3];
    return value <= 0x7FFFFFFFUL ? (int32_t)value : -1 - (int32_t)(~value);
}

static int16_t read_i16(const uint8_t *data)
/* 功能：按摩擦轮反馈协议读取有符号 16 位整数。 */
{
    uint32_t value = ((uint32_t)data[0] << 8) | data[1];
    return (int16_t)(value < 0x8000U ? (int32_t)value : (int32_t)value - 65536);
}

static uint8_t magnitude_greater(int32_t value, int32_t limit)
/* 功能：在不依赖数值符号的情况下判断幅值是否超过限制。 */
{
    return (uint8_t)(value > limit || value < -limit);
}

static uint32_t feedback_faults(const FlywheelJogFeedback *feedback)
/* 功能：从摩擦轮速度、电流和温度反馈中汇总运行故障位。 */
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
/* 功能：初始化左右轮反馈、速度环及会话和停机状态。 */
{
    if (jog != 0) { memset(jog, 0, sizeof(*jog)); }
}

void FlywheelJog_Stop(FlywheelJog *jog)
/* 功能：结束当前摩擦轮运行请求并将两侧电流请求置零。 */
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
    FlywheelSpeed_Stop(&jog->speed);
    FlywheelSpeed_Stop(&jog->speed_right);
    jog->pending_mask = FLYWHEEL_BOTH_NODES;
}

void FlywheelJog_Fault(FlywheelJog *jog, uint32_t faults)
/* 功能：锁存摩擦轮故障位，并沿原停止路径清除动力请求。 */
{
    if (jog == 0 || faults == 0U) { return; }
    jog->faults |= faults;
    jog->stop_reason = FLYWHEEL_STOP_FAULT;
    FlywheelJog_Stop(jog);
}

static void synchronize_epoch(FlywheelJog *jog, uint32_t can_epoch)
/* 功能：同步 CAN 故障代次，防止旧反馈继续支撑当前轮子会话。 */
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
/* 功能：对损坏或不符合预期的摩擦轮反馈锁存帧错误。 */
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
/* 功能：识别两侧 VESC 状态广播，更新反馈并立即检查运行故障。 */
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
        if (jog->active) {
            if (jog->selection == (node == APP_FLYWHEEL_LEFT_ID ?
                FLYWHEEL_JOG_LEFT : FLYWHEEL_JOG_RIGHT)) {
                FlywheelSpeed_RecordFeedback(&jog->speed, feedback->erpm, feedback->status1_ms);
            } else if (jog->selection == FLYWHEEL_JOG_BOTH) {
                FlywheelSpeed_RecordFeedback(node == APP_FLYWHEEL_LEFT_ID ?
                    &jog->speed : &jog->speed_right, feedback->erpm, feedback->status1_ms);
            }
        }
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
/* 功能：根据两侧反馈时效、故障和转速汇总健康与停转状态。 */
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
/* 功能：核对停止、健康和重新触发条件后选择运行的摩擦轮。 */
{
    FlywheelJogHealth health;
    if (jog == 0 || (selection != FLYWHEEL_JOG_OFF &&
        selection != FLYWHEEL_JOG_LEFT && selection != FLYWHEEL_JOG_RIGHT &&
        selection != FLYWHEEL_JOG_BOTH)) { return 0U; }
    if (selection == FLYWHEEL_JOG_BOTH && !1U) { return 0U; }
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
/* 功能：按实时许可和输入推进摩擦轮速度控制、停止及待发电流请求。 */
{
    FlywheelJogHealth health;
    uint8_t rising;
    int32_t current_limit;
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
                ) {
                jog->stop_reason = !positive_request ? FLYWHEEL_STOP_INPUT : FLYWHEEL_STOP_DURATION;
                FlywheelJog_Stop(jog);
            } else {
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
                jog->last_speed_selection = jog->selection;
                FlywheelSpeed_Start(&jog->speed, now_ms,
                    jog->feedback[jog->selection != FLYWHEEL_JOG_RIGHT ? 0U : 1U].erpm,
                    jog->feedback[jog->selection != FLYWHEEL_JOG_RIGHT ? 0U : 1U].status1_ms);
                FlywheelSpeed_Init(&jog->speed_right);
                if (jog->selection == FLYWHEEL_JOG_BOTH) {
                    FlywheelSpeed_Start(&jog->speed_right, now_ms,
                        jog->feedback[1].erpm, jog->feedback[1].status1_ms);
                }
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
/* 功能：读取当前选中轮的有符号电流请求，用于状态发布。 */
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
/* 功能：按具体 VESC 节点读取对应左右轮的有符号电流请求。 */
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
/* 功能：取出一条待发的摩擦轮电流请求，并消耗对应待发标记。 */
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
/* 功能：仅在关闭、回中且反馈健康停转时清除可恢复故障。 */
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
