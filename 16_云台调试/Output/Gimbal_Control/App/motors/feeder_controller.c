/**
 * @file feeder_controller.c
 * @brief 拨盘单发/连发位置速度控制
 * 配置入口：App/app_config.h；接口声明见同目录对应头文件。
 * 对外函数（10）：
 *   Feeder_Stop / Feeder_ConfigValid / Feeder_Init / Feeder_SetConfig / Feeder_OnFrame /
 *   Feeder_Clear / Feeder_Zero / Feeder_ContinuousRpm / Feeder_Run / Feeder_Step
 * 内部函数（14）：
 *   absolute / abs_double / clamp / finite_value / within / positions / reset_output / fault / fresh
 *   / consume_event / completed_step / stream_crossings / finish_stream / single_speed_limit
 * 文件级变量/常量（0；局部变量见对应函数）：
 *   无；状态由调用方传入的结构体保存。
 * 维护约束：函数前的功能说明对应当前实现，修改行为时同步更新。
 */
/* 拨盘控制数学：累计角度、单发减速、连续跟踪及保护。 */
#include "feeder_controller.h"
#include "app_config.h"
#include <float.h>
#include <math.h>
#include <string.h>

static float absolute(float x)
/* 功能：返回数值的绝对值，用于误差或速度幅值比较。 */
{ return x < 0.0f ? -x : x; }
static double abs_double(double x)
/* 功能：返回双精度数值的绝对值，保留连续编码器计算精度。 */
{ return x < 0.0 ? -x : x; }
static float clamp(float x, float low, float high)
/* 功能：把数值限制在指定上下界内，避免控制请求越界。 */
{ return x < low ? low : (x > high ? high : x); }
static uint8_t finite_value(float x)
/* 功能：检查浮点数是否有限，拒绝 NaN 和无穷值进入控制。 */
{ return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
static uint8_t within(float x, float low, float high)
/* 功能：检查数值是否有限且处于指定闭区间。 */
{ return (uint8_t)(finite_value(x) && x >= low && x <= high); }
static void positions(FeederController *c)
/* 功能：由连续编码器计数和传动配置换算拨盘实际及目标角度。 */
{
    c->position_deg = (double)(c->accumulated_counts - c->zero_counts) * FEEDER_COUNTS_TO_DEGREES;
    c->axis_position_deg = c->position_deg / (double)c->config.gear_ratio;
    c->axis_target_deg = c->target_deg / (double)c->config.gear_ratio;
}
static void reset_output(FeederController *c)
/* 功能：清除拨盘当前控制输出、速度参考及对应积分历史。 */
{
    c->command_raw = 0; c->target_rpm = c->integral = 0.0f;
    c->single_braking = 0U;
    c->settling = c->jamming = c->have_control_tick = 0U;
}
void Feeder_Stop(FeederController *c)
/* 功能：结束拨盘当前动作并清除控制输出，保留原配置和后续恢复所需状态。 */
{
    if (!c) { return; }
    if (c->busy && c->outstanding) {
        ++c->aborted;
        if (c->outstanding > 1U) { c->cancelled += c->outstanding - 1U; }
    }
    c->busy = 0U; reset_output(c);
    c->outstanding = 0U; c->motion_kind = FEEDER_MOTION_IDLE;
    c->target_deg = c->position_deg;
    c->axis_target_deg = c->axis_position_deg;
}
static void fault(FeederController *c, uint32_t reason)
/* 功能：锁存拨盘运行故障，并清除当前控制输出和动作请求。 */
{
    c->faults |= reason; Feeder_Stop(c); c->have_zero = 0U;
    c->recovery_samples = 0U;
}
static uint8_t fresh(const FeederController *c, uint32_t now)
/* 功能：判断拨盘控制器最近一帧反馈是否仍在有效时间范围内。 */
{
    uint32_t age = now - c->last_feedback_ms;
    return (uint8_t)(c->have_feedback && age <= FEEDER_FEEDBACK_TIMEOUT_MS);
}
uint8_t Feeder_ConfigValid(const FeederConfig *p)
/* 功能：检查拨盘传动、方向、PID、速度、电流、到位和堵转参数的合法范围。 */
{
    if (!p) { return 0U; }
    return (uint8_t)(within(p->step_deg, 0.1f, 360.0f) &&
        within(p->gear_ratio, 0.1f, 100.0f) &&
        (p->direction == -1 || p->direction == 0 || p->direction == 1) &&
        p->calibrated <= 1U && (!p->calibrated || p->direction != 0) &&
        within(p->pos_kp, 0.001f, 100.0f) &&
        within(p->speed_kp, 0.001f, 100.0f) && within(p->speed_ki, 0.0f, 1000.0f) &&
        within(p->max_rpm, 1.0f, 6000.0f) &&
        within(p->max_feedback_rpm, p->max_rpm, 6000.0f) &&
        within(p->continuous_max_rpm, 1.0f, p->max_feedback_rpm) &&
        within(p->continuous_brake_kp, 0.001f, 1000.0f) &&
        within(p->continuous_phase_ms, 0.0f, (float)APP_FIRE_INTERVAL_MS * 0.8f) &&
        (p->single_decel_rpm_s == 0.0f || within(p->single_decel_rpm_s, 1000.0f, 200000.0f)) &&
        within(p->current_limit, 1.0f, (float)FEEDER_CURRENT_CONFIG_LIMIT) &&
        within(p->tolerance_deg, 0.05f, 5.0f) &&
        within(p->settle_rpm, 0.0f, 100.0f) &&
        within(p->jam_rpm, 0.0f, 100.0f) &&
        within(p->jam_current_raw, 1.0f, (float)FEEDER_CURRENT_CONFIG_LIMIT) &&
        p->settle_ms >= 5U && p->settle_ms <= 500U &&
        p->motion_timeout_ms >= 50U && p->motion_timeout_ms <= 5000U &&
        p->jam_ms >= 20U && p->jam_ms <= 2000U && p->jam_ms < p->motion_timeout_ms);
}
void Feeder_Init(FeederController *c)
/* 功能：载入拨盘默认配置，并初始化编码器、动作和故障状态。 */
{
    if (!c) { return; }
    memset(c, 0, sizeof(*c));
    c->config.step_deg = APP_FEEDER_STEP_DEG; c->config.gear_ratio = APP_FEEDER_GEAR_RATIO;
    c->config.direction = APP_FEEDER_DIRECTION; c->config.calibrated = APP_FEEDER_CALIBRATED;
    c->config.pos_kp = APP_FEEDER_POSITION_KP; c->config.speed_kp = APP_FEEDER_SPEED_KP; c->config.speed_ki = APP_FEEDER_SPEED_KI;
    c->config.max_rpm = APP_FEEDER_MAX_RPM; c->config.max_feedback_rpm = APP_FEEDER_MAX_FEEDBACK_RPM;
    c->config.continuous_max_rpm = APP_FEEDER_CONTINUOUS_MAX_RPM;
    /* Track from the trigger time; the previous 10ms lag intentionally lost
     * 8 shaft degrees at 40deg/50ms. Keep the existing boundary brake. */
    c->config.continuous_brake_kp = APP_FEEDER_CONTINUOUS_BRAKE_KP; c->config.continuous_phase_ms = APP_FEEDER_CONTINUOUS_PHASE_MS;
    c->config.single_decel_rpm_s = APP_FEEDER_SINGLE_DECEL_RPM_S;
    c->config.current_limit = APP_FEEDER_CURRENT_LIMIT; c->config.tolerance_deg = APP_FEEDER_TOLERANCE_DEG;
    c->config.settle_rpm = APP_FEEDER_SETTLE_RPM; c->config.settle_ms = APP_FEEDER_SETTLE_MS;
    c->config.motion_timeout_ms = APP_FEEDER_MOTION_TIMEOUT_MS;
    c->config.jam_rpm = APP_FEEDER_JAM_RPM; c->config.jam_current_raw = APP_FEEDER_JAM_CURRENT_RAW; c->config.jam_ms = APP_FEEDER_JAM_MS;
}
uint8_t Feeder_SetConfig(FeederController *c, const FeederConfig *p)
/* 功能：在控制器允许修改时应用合法配置，并同步必要的控制状态。 */
{
    uint8_t invalidate;
    if (!c || c->busy || !Feeder_ConfigValid(p)) { return 0U; }
    invalidate = (uint8_t)(p->gear_ratio != c->config.gear_ratio ||
        p->direction != c->config.direction || p->calibrated != c->config.calibrated);
    c->config = *p; Feeder_Stop(c);
    if (invalidate) { c->have_zero = 0U; }
    positions(c); c->axis_target_deg = c->axis_position_deg;
    return 1U;
}
uint8_t Feeder_OnFrame(FeederController *c, const CanFrame *f, uint32_t now, uint32_t epoch)
/* 功能：解析 C610 反馈，进行解环、时效、跳变、速度和电流检查。 */
{
    uint16_t angle;
    int16_t rpm, current;
    int32_t delta = 0;
    uint32_t gap = 0U, bound_gap, reasons = 0U, jump_flags;
    float rpm_bound, counts_bound, exact_gap_bound, expected_counts;
    uint8_t first, rebase = 0U;
    if (!c || !f || f->bus != 2U || f->id != 0x205U || f->extended) { return 0U; }
    if (f->dlc != 8U) { fault(c, FEEDER_FAULT_DATA); return 0U; }
    if (c->have_feedback && c->bus_epoch != epoch) {
        fault(c, FEEDER_FAULT_CAN); rebase = 1U;
    }
    if (f->fault_epoch != epoch) { return 0U; }
    if (now - f->received_ms > FEEDER_FEEDBACK_TIMEOUT_MS) {
        fault(c, FEEDER_FAULT_FEEDBACK); return 0U;
    }
    angle = (uint16_t)(((uint16_t)f->data[0] << 8) | f->data[1]);
    rpm = (int16_t)(((uint16_t)f->data[2] << 8) | f->data[3]);
    current = (int16_t)(((uint16_t)f->data[4] << 8) | f->data[5]);
    if (angle >= FEEDER_ENCODER_COUNTS) { fault(c, FEEDER_FAULT_DATA); return 0U; }
    if (absolute((float)rpm) > c->config.max_feedback_rpm) { reasons |= FEEDER_FAULT_OVERSPEED; }
    if (absolute((float)current) > (float)FEEDER_CURRENT_PROTOCOL_LIMIT) { reasons |= FEEDER_FAULT_CURRENT; }
    first = (uint8_t)!c->have_feedback;
    if (!first) {
        gap = f->received_ms - c->last_feedback_ms;
        if (gap >= APP_TICK_HALF_RANGE || (!gap && !rebase)) { return 0U; }
        if (gap > FEEDER_FEEDBACK_TIMEOUT_MS) { reasons |= FEEDER_FAULT_FEEDBACK; rebase = 1U; }
        delta = (int32_t)angle - (int32_t)c->raw_angle;
        if (delta > 4096) { delta -= 8192; }
        else if (delta < -4096) { delta += 8192; }
        rpm_bound = absolute((float)rpm);
        if (absolute((float)c->raw_rpm) > rpm_bound) { rpm_bound = absolute((float)c->raw_rpm); }
        bound_gap = gap + FEEDER_GAP_UNCERTAINTY_MS;
        expected_counts = rpm_bound * (float)bound_gap * ((float)FEEDER_ENCODER_COUNTS / 60000.0f);
        /* At >= half a rotor revolution between samples, shortest-path unwrap
         * is ambiguous. Lose the zero rather than guessing unseen revolutions. */
        /* Integer receipt timestamps are quantized, not exact frame periods.
         * V1E field sample: -604 counts, tick gap 1, speeds -2048/-2234rpm.
         * Bound with gap+1ms; accumulate the actual encoder delta unchanged.
         * Keep the existing 256-count discrepancy allowance, and use the same
         * conservative time bound for half-turn ambiguity (never guess turns). */
        exact_gap_bound = (rpm_bound + 120.0f) * (float)gap *
            ((float)FEEDER_ENCODER_COUNTS / 60000.0f) + (float)FEEDER_JUMP_SLACK_COUNTS;
        counts_bound = (rpm_bound + 120.0f) * (float)bound_gap *
            ((float)FEEDER_ENCODER_COUNTS / 60000.0f) + (float)FEEDER_JUMP_SLACK_COUNTS;
        jump_flags = 0U;
        if (expected_counts >= 4064.0f) { jump_flags |= FEEDER_JUMP_HALF_TURN; }
        if (absolute((float)delta) > counts_bound) { jump_flags |= FEEDER_JUMP_DELTA; }
        if (!rebase && !reasons && !jump_flags && absolute((float)delta) > exact_gap_bound) {
            ++c->timestamp_allowances;
        }
        if (!rebase && jump_flags) {
            ++c->jump.events;
            if (!c->jump.valid) {
                c->jump.valid = 1U; c->jump.at_ms = f->received_ms;
                c->jump.gap_ms = gap; c->jump.bound_gap_ms = bound_gap;
                c->jump.age_ms = now - f->received_ms;
                c->jump.sequence = c->feedback_sequence + 1U; c->jump.flags = jump_flags;
                c->jump.previous_angle = c->raw_angle; c->jump.angle = angle;
                c->jump.previous_rpm = c->raw_rpm; c->jump.rpm = rpm;
                c->jump.feedback_raw = current; c->jump.command_raw = c->command_raw;
                c->jump.delta_counts = delta; c->jump.limit_counts = counts_bound;
                c->jump.expected_counts = expected_counts;
                c->jump.busy = c->busy; c->jump.zero_valid = c->have_zero;
                memcpy(c->jump.data, f->data, sizeof(c->jump.data));
            }
            reasons |= FEEDER_FAULT_JUMP; rebase = 1U;
        }
    }
    if (reasons) { fault(c, reasons); rebase = 1U; }
    if (!first && !rebase) { c->accumulated_counts += delta; }
    c->raw_angle = angle; c->raw_rpm = rpm; c->raw_current = current;
    c->last_feedback_ms = f->received_ms; c->bus_epoch = epoch;
    c->have_feedback = 1U; ++c->feedback_sequence;
    positions(c);
    if (!c->busy && !c->have_zero) {
        c->target_deg = c->position_deg; c->axis_target_deg = c->axis_position_deg;
    }
    if (!reasons && !rebase && absolute((float)rpm) <= c->config.settle_rpm &&
        absolute((float)delta) <= 2.0f) {
        if (c->recovery_samples < 255U) { ++c->recovery_samples; }
    } else { c->recovery_samples = 0U; }
    return 1U;
}
uint8_t Feeder_Clear(FeederController *c, uint32_t now)
/* 功能：在反馈健康且允许恢复时清除可恢复的拨盘故障。 */
{
    if (!c || c->busy || !fresh(c, now) || c->recovery_samples < 3U ||
        absolute((float)c->raw_rpm) > c->config.settle_rpm ||
        absolute((float)c->raw_current) > (float)FEEDER_CURRENT_PROTOCOL_LIMIT) { return 0U; }
    Feeder_Stop(c); c->faults = 0U; c->have_zero = 0U;
    c->jump.valid = 0U; /* Lifetime count remains; next trial captures its own first jump. */
    return 1U;
}
uint8_t Feeder_Zero(FeederController *c, uint32_t now)
/* 功能：在原静止门禁允许时建立拨盘软件零点。 */
{
    if (!c || c->busy || c->faults || !fresh(c, now) || c->recovery_samples < 3U ||
        !c->config.calibrated || !c->config.direction ||
        absolute((float)c->raw_rpm) > c->config.settle_rpm) { return 0U; }
    Feeder_Stop(c); c->zero_counts = c->accumulated_counts;
    c->position_deg = c->target_deg = c->axis_position_deg = c->axis_target_deg = 0.0;
    c->have_zero = 1U; return 1U;
}
static uint8_t consume_event(FeederController *c, uint8_t event, uint32_t sequence)
/* 功能：按事件序号识别新单次请求，避免重复执行同一个发射事件。 */
{
    uint32_t distance;
    if (!event) { return 0U; }
    distance = sequence - c->last_shot_sequence;
    if (c->have_shot_sequence && (!distance || distance >= APP_TICK_HALF_RANGE)) { return 0U; }
    c->have_shot_sequence = 1U; c->last_shot_sequence = sequence; return 1U;
}
float Feeder_ContinuousRpm(const FeederController *c)
/* 功能：按每发角度、传动比和连发间隔计算所需转子转速。 */
{
    return c ? c->config.step_deg * c->config.gear_ratio *
        (1000.0f / (6.0f * (float)APP_FIRE_INTERVAL_MS)) : 0.0f;
}

static void completed_step(FeederController *c, uint32_t now)
/* 功能：记录一发完成后的用时、误差、超调及累计完成计数。 */
{
    uint32_t i;
    if (!c->outstanding) { return; }
    ++c->completed;
    c->last_motion_ms = now - c->pending_started_ms[0]; c->last_completed_ms = now;
    c->last_motion_error_deg = (float)((c->position_deg - c->next_step_deg) /
                                      (double)c->config.gear_ratio);
    if (c->motion_kind == FEEDER_MOTION_SINGLE) { ++c->single_completed; }
    else {
        ++c->continuous_completed; ++c->burst_completed;
        if (c->burst_completed == 1U) { c->burst_first_ms = now; }
        c->burst_elapsed_ms = now - c->burst_first_ms;
        c->measured_hz = c->burst_completed > 1U && c->burst_elapsed_ms ?
            (float)(c->burst_completed - 1U) * 1000.0f / (float)c->burst_elapsed_ms : 0.0f;
    }
    --c->outstanding;
    for (i = 0U; i < c->outstanding; ++i) { c->pending_started_ms[i] = c->pending_started_ms[i + 1U]; }
}

static void stream_crossings(FeederController *c)
/* 功能：依据实测连续角度统计连发过程中已经跨过的发射边界。 */
{
    double step = (double)c->config.step_deg * (double)c->config.gear_ratio * c->config.direction;
    while (c->outstanding && (c->position_deg - c->next_step_deg) * c->config.direction >= 0.0) {
        completed_step(c, c->last_feedback_ms);
        if (c->outstanding) { c->next_step_deg += step; }
    }
}

static void finish_stream(FeederController *c, uint32_t now)
/* 功能：在释放连发时收回多余前瞻目标，保留已接受的尾步目标。 */
{
    if (c->outstanding > 1U) {
        c->cancelled += c->outstanding - 1U; c->outstanding = 1U;
    }
    /* Finish only the boundary currently being traversed. No lookahead survives
     * a trigger release or wheel speed dip, even if it was previously accepted. */
    c->target_deg = c->next_step_deg;
    c->axis_target_deg = c->target_deg / (double)c->config.gear_ratio;
    c->motion_kind = FEEDER_MOTION_TAIL; c->motion_started_ms = now; c->settling = 0U;
}

static float single_speed_limit(const FeederController *c, float error)
/* 功能：依据剩余角度和减速参数计算单发接近目标时的速度上限。 */
{
    /* Rotor distance = 6*v*delay + 3*v*v/decel, with v in rpm.
     * Solve for a speed that leaves a stopping margin. This is a controller
     * setting, not a measurement of available mechanical deceleration. */
    float lead = c->config.single_decel_rpm_s *
        ((float)FEEDER_SINGLE_BRAKE_DELAY_MS * 0.001f);
    float distance_term = c->config.single_decel_rpm_s * absolute(error) / 3.0f;
    float root = sqrtf(lead * lead + distance_term);
    /* Rationalized form retains precision for very small position errors. */
    return distance_term / (root + lead);
}

int16_t Feeder_Run(FeederController *c, uint32_t now, uint8_t allowed,
                   uint8_t wheels_ready, uint8_t shot_event, uint32_t sequence, uint8_t continuous)
/* 功能：执行拨盘请求接收、单发/连发目标推进、位置速度控制和运行保护。 */
{
    uint8_t event, new_feedback, started = 0U;
    uint32_t tick_gap = 0U, feedback_gap;
    float error, speed_error, p, candidate, output, dt, feedforward, forward, ceiling;
    double step;
    if (!c) { return 0; }
    event = consume_event(c, shot_event, sequence);
    if (c->have_feedback && !fresh(c, now)) { fault(c, FEEDER_FAULT_FEEDBACK); }
    if (!allowed || c->faults || !c->have_zero ||
        !c->config.calibrated || !c->config.direction || !fresh(c, now)) {
        Feeder_Stop(c); if (event) { ++c->rejected; } return 0;
    }
    if (c->have_control_tick) { tick_gap = now - c->last_control_ms; }
    if (c->busy && c->have_control_tick && tick_gap > FEEDER_CONTROL_MAX_GAP_MS) {
        fault(c, FEEDER_FAULT_TIMING); if (event) { ++c->rejected; } return 0;
    }
    c->last_control_ms = now; c->have_control_tick = 1U;
    new_feedback = c->last_control_sequence != c->feedback_sequence;
    step = (double)c->config.step_deg * (double)c->config.gear_ratio * c->config.direction;
    feedforward = Feeder_ContinuousRpm(c);
    /* Consume actual crossings before considering another pulse at the same tick. */
    if (c->busy && new_feedback && c->motion_kind != FEEDER_MOTION_SINGLE) { stream_crossings(c); }
    if (c->busy && c->motion_kind == FEEDER_MOTION_STREAM &&
        (!continuous || !wheels_ready ||
         now - c->stream_last_event_ms > 2U * APP_FIRE_INTERVAL_MS + FEEDER_CONTROL_MAX_GAP_MS)) {
        finish_stream(c, now);
    }
    if (event) {
        c->rate_blocked = continuous && feedforward > c->config.continuous_max_rpm;
        if (!wheels_ready || c->rate_blocked ||
            (c->busy && (!continuous || c->motion_kind != FEEDER_MOTION_STREAM))) { ++c->rejected; }
        else if (c->busy && c->outstanding >= FEEDER_STREAM_MAX_PENDING) {
            ++c->rejected; c->stream_last_event_ms = now;
        } else {
            if (!c->busy) {
                c->busy = 1U; c->motion_started_ms = now; c->outstanding = 0U;
                c->motion_kind = continuous ? FEEDER_MOTION_STREAM : FEEDER_MOTION_SINGLE;
                c->integral = 0.0f; c->settling = c->jamming = 0U; started = 1U;
                c->single_braking = 0U;
                if (!continuous) { c->single_brake_frames = 0U; c->single_overshoot_deg = 0.0f; }
                c->last_control_sequence = c->feedback_sequence - 1U;
                c->last_pi_feedback_ms = c->last_feedback_ms;
                if (continuous) {
                    c->burst_completed = c->burst_elapsed_ms = 0U; c->measured_hz = 0.0f;
                    /* Software reference phase lag, not an assumed mechanical delay. */
                    c->stream_reference_deg = c->position_deg - step *
                        (double)c->config.continuous_phase_ms / (double)APP_FIRE_INTERVAL_MS;
                }
            }
            c->target_deg += step;
            if (!c->outstanding) { c->next_step_deg = c->target_deg; }
            c->pending_started_ms[c->outstanding++] = now;
            c->axis_target_deg = c->target_deg / (double)c->config.gear_ratio;
            c->stream_last_event_ms = now; ++c->accepted;
        }
    }
    if (!c->busy) { c->command_raw = 0; return 0; }
    if ((c->outstanding && now - c->pending_started_ms[0] >= c->config.motion_timeout_ms) ||
        (!c->outstanding && c->motion_kind == FEEDER_MOTION_TAIL &&
         now - c->motion_started_ms >= c->config.motion_timeout_ms)) {
        fault(c, FEEDER_FAULT_TIMEOUT); return 0;
    }
    if (c->motion_kind == FEEDER_MOTION_STREAM && !started) {
        c->stream_reference_deg += step * (double)tick_gap / (double)APP_FIRE_INTERVAL_MS;
        if ((c->stream_reference_deg - c->target_deg) * c->config.direction > 0.0) {
            c->stream_reference_deg = c->target_deg;
        }
    }
    error = (float)(c->target_deg - c->position_deg);
    new_feedback = c->last_control_sequence != c->feedback_sequence;
    if (new_feedback) {
        c->last_control_sequence = c->feedback_sequence;
        c->single_braking = 0U;
        if (c->motion_kind == FEEDER_MOTION_SINGLE) {
            forward = -error * c->config.direction / c->config.gear_ratio;
            if (forward > c->single_overshoot_deg) { c->single_overshoot_deg = forward; }
        }
        if (c->motion_kind != FEEDER_MOTION_STREAM &&
            abs_double(c->target_deg - c->position_deg) <=
            (double)c->config.tolerance_deg * (double)c->config.gear_ratio &&
            absolute((float)c->raw_rpm) <= c->config.settle_rpm) {
            if (!c->settling) { c->settling = 1U; c->settle_started_ms = c->last_feedback_ms; }
            else if (c->last_feedback_ms - c->settle_started_ms >= c->config.settle_ms) {
                completed_step(c, c->last_feedback_ms);
                c->busy = 0U; c->motion_kind = FEEDER_MOTION_IDLE; reset_output(c); return 0;
            }
        } else { c->settling = 0U; }
        c->target_rpm = clamp(error * c->config.pos_kp, -c->config.max_rpm, c->config.max_rpm);
        if (c->motion_kind == FEEDER_MOTION_SINGLE && c->config.single_decel_rpm_s > 0.0f) {
            ceiling = single_speed_limit(c, error);
            /* Velocity damping through a short position preview. At rest the
             * original full-speed launch is retained; moving toward the goal
             * lowers vref before crossing it. 6 converts rotor rpm to deg/s. */
            p = c->config.pos_kp * (error - 6.0f * (float)c->raw_rpm *
                ((float)FEEDER_SINGLE_PREVIEW_MS * 0.001f));
            c->target_rpm = clamp(p, -c->config.max_rpm, c->config.max_rpm);
            c->target_rpm = clamp(c->target_rpm, -ceiling, ceiling);
            /* Unload stored driving current only when speed feedback calls
             * for braking. No fixed reverse-current pulse and no sign latch. */
            c->single_braking = (uint8_t)(absolute((float)c->raw_rpm) > c->config.settle_rpm &&
                (c->target_rpm - (float)c->raw_rpm) * (float)c->raw_rpm < 0.0f);
        }
        if (c->motion_kind == FEEDER_MOTION_STREAM && c->outstanding && error * c->config.direction > 0.0f) {
            p = feedforward * c->config.direction +
                c->config.pos_kp * (float)(c->stream_reference_deg - c->position_deg);
            c->target_rpm = clamp(p, -c->config.continuous_max_rpm, c->config.continuous_max_rpm);
            /* Bound speed near the furthest accepted boundary. Never drive at
             * feedforward speed into a missing/rejected future request. */
            forward = c->target_rpm * c->config.direction;
            ceiling = absolute(error) * c->config.continuous_brake_kp;
            if (forward > ceiling) { c->target_rpm = ceiling * c->config.direction; }
        }
        speed_error = c->target_rpm - (float)c->raw_rpm;
        feedback_gap = c->last_feedback_ms - c->last_pi_feedback_ms;
        c->last_pi_feedback_ms = c->last_feedback_ms; dt = (float)feedback_gap * 0.001f;
        if (c->single_braking) {
            c->integral *= (float)FEEDER_SINGLE_I_RELAX_MS /
                ((float)FEEDER_SINGLE_I_RELAX_MS + (float)feedback_gap);
            ++c->single_brake_frames;
        }
        p = c->config.speed_kp * speed_error;
        /* During braking, release toward zero instead of accumulating an
         * opposite integral that would provoke another return swing. */
        candidate = clamp(c->integral + (c->single_braking ? 0.0f : c->config.speed_ki * speed_error * dt),
            -c->config.current_limit, c->config.current_limit);
        output = p + candidate;
        if ((output <= c->config.current_limit && output >= -c->config.current_limit) ||
            (output > c->config.current_limit && speed_error < 0.0f) ||
            (output < -c->config.current_limit && speed_error > 0.0f)) { c->integral = candidate; }
        output = clamp(p + c->integral, -c->config.current_limit, c->config.current_limit);
        c->command_raw = (int16_t)output;
        if (!c->settling && absolute((float)c->raw_rpm) <= c->config.jam_rpm &&
            (absolute(output) >= c->config.jam_current_raw ||
             absolute((float)c->raw_current) >= c->config.jam_current_raw)) {
            if (!c->jamming) { c->jamming = 1U; c->jam_started_ms = c->last_feedback_ms; }
            else if (c->last_feedback_ms - c->jam_started_ms >= c->config.jam_ms) {
                fault(c, FEEDER_FAULT_JAM); return 0;
            }
        } else { c->jamming = 0U; }
    }
    return c->command_raw;
}

int16_t Feeder_Step(FeederController *c, uint32_t now, uint8_t allowed,
                    uint8_t wheels_ready, uint8_t shot_event, uint32_t sequence)
/* 功能：以非连续模式调用拨盘统一控制入口，保留单步调用兼容性。 */
{
    return Feeder_Run(c, now, allowed, wheels_ready, shot_event, sequence, 0U);
}
