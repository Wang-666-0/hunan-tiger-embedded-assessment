/*
 * 文件功能：实现 GM6020 的多圈角度—转速双环串级控制。
 * 1. A 多圈角度 -> 角度 PID -> 目标 rpm -> 转速 PID -> CAN 原始给定。
 * 2. 每 2 ms 执行双环；S 仍可独立运行内环，P/I/D 和 AP/AI/AD 在线调参。
 * 3. 反馈、温度、控制时序或 CAN 发送异常时撤销请求；不再等待 K 续期。
 * 4. 提供一致的控制状态，供 VOFA 显示目标、实际值和调参信息。
 * 5. S 反转只清内环 I；角度环显著制动阶段只在入口清一次内环 I。
 * 6. 反馈失效后圈数不自动恢复；STOP 且静止时 Z 重建参考，再重新 A。
 * 7. 实测入带时间及超调，不能仅靠软件保证 300 ms / 无超调指标。
 * 设计意义：PID 只管数学，本文只管电机条件与任务间请求；串口任务
 * 不直接修改 PID 积分状态，也不直接发送 CAN。S0 是零转速闭环，
 * STOP 才退出控制并发送零给定；零给定不等于机械抱闸。
 * 上电不自动启动，断开电脑也不会自动停止；启动与停止由明确的命令决定。
 */
#include "motor_control.h"
#include "pid.h"
#include "can_transport.h"
#include "can_debug.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

typedef struct
{
    uint32_t start_ms;
    uint32_t band_entry_ms;
    float direction;
    uint8_t band_active;
} AngleStepMetrics;

volatile MotorControl_State motor_control;
static PID_Controller speed_pid; // 只有控制任务读写 PID 的积分和微分历史。
static PID_Controller angle_pid;
static AngleStepMetrics angle_metrics;
static uint8_t reset_requested;  // 串口要求清空历史，由下一轮控制任务执行。
static uint8_t clear_integral_requested; // 反转/零速请求只清旧积分，不丢失实际值和 D 历史。
static uint8_t clear_angle_integral_requested; // 新角度目标只清外环 I，保留 D。
static uint8_t brake_latched; // 同一显著制动阶段只清一次内环 I。
static uint8_t have_step_time;   // 第一次启用时还没有可用的上轮控制时间。
static uint32_t last_step_ms;    // 用真实运行间隔计算 dt，而不是盲用固定常数。
static uint32_t request_revision; // 防止计算时出现的新停止请求被旧结果覆盖。
static uint32_t active_angle_epoch; // 接受 A 时的零点代次，运行途中不能偷偷换零点。

static float MotorControl_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float MotorControl_OutputLimit(void)
{
    return (GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE) ?
        (float)MOTOR_VOLTAGE_DEMO_LIMIT : (float)MOTOR_CURRENT_DEMO_LIMIT;
}

static PID_Config MotorControl_Config(float kp, float ki, float kd)
{
    PID_Config config;
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
    config.output_limit = MotorControl_OutputLimit(); // 限幅保护实际发送的原始给定。
    config.integral_limit = SPEED_PID_INTEGRAL_LIMIT; // 单独限制积分项的累计贡献。
    config.derivative_tau_s = SPEED_PID_DERIVATIVE_TAU_S; // D 项滤波，减少转速量化噪声。
    return config;
}

static PID_Config MotorControl_AngleConfig(float kp, float ki, float kd, float speed_limit)
{
    PID_Config config;
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
    config.output_limit = speed_limit; // 外环输出 rpm，不是电机 raw。
    config.integral_limit = ANGLE_PID_INTEGRAL_LIMIT_RPM;
    config.derivative_tau_s = ANGLE_PID_DERIVATIVE_TAU_S;
    return config;
}

void MotorControl_Init(void)
{
    PID_Config config = MotorControl_Config(SPEED_PID_DEFAULT_KP,
        SPEED_PID_DEFAULT_KI, SPEED_PID_DEFAULT_KD);
    PID_Config angle_config = MotorControl_AngleConfig(ANGLE_PID_DEFAULT_KP,
        ANGLE_PID_DEFAULT_KI, ANGLE_PID_DEFAULT_KD, ANGLE_DEFAULT_SPEED_LIMIT_RPM);
    GM6020_ResetFeedback(); // 任务启动前调用，旧反馈与旧圈数不得带入新实验。
    memset((void *)&motor_control, 0, sizeof(motor_control)); // 创建任务前调用，初始输出为零。
    memset(&angle_metrics, 0, sizeof(angle_metrics));
    motor_control.kp = config.kp;
    motor_control.ki = config.ki;
    motor_control.kd = config.kd;
    motor_control.angle_kp = angle_config.kp;
    motor_control.angle_ki = angle_config.ki;
    motor_control.angle_kd = angle_config.kd;
    motor_control.speed_limit_rpm = angle_config.output_limit;
    motor_control.control_mode = MOTOR_CONTROL_SPEED_MODE;
    motor_control.mode = GM6020_CONTROL_MODE;
    motor_control.motor_id = GM6020_MOTOR_ID;
    motor_control.stop_reason = MOTOR_STOP_POWER_ON;
    (void)PID_Init(&speed_pid, &config); // 初值在 app_config.h，暂不自动启动电机。
    (void)PID_Init(&angle_pid, &angle_config);
    reset_requested = 1U;
    clear_integral_requested = 0U;
    clear_angle_integral_requested = 0U;
    brake_latched = 0U;
    have_step_time = 0U;
    last_step_ms = 0U;
    request_revision = 0U;
    active_angle_epoch = 0U;
}

/* 必须在短临界区内调用。只撤销请求，硬件发送由 Step 统一处理。 */
static void MotorControl_StopLocked(MotorStopReason reason)
{
    motor_control.enabled = 0U;
    motor_control.target_rpm = 0.0f; // 清掉旧目标，条件恢复后不会自行重启。
    motor_control.error_rpm = -motor_control.actual_rpm;
    motor_control.target_angle_deg = motor_control.actual_angle_deg; // 不留下待恢复的旧角度请求。
    motor_control.error_angle_deg = 0.0f;
    motor_control.output_raw = 0;
    motor_control.p_term = 0.0f;
    motor_control.i_term = 0.0f;
    motor_control.d_term = 0.0f;
    motor_control.angle_p_term = 0.0f;
    motor_control.angle_i_term = 0.0f;
    motor_control.angle_d_term = 0.0f;
    motor_control.settled = 0U;
    motor_control.settling_time_ms = 0U;
    angle_metrics.band_active = 0U;
    motor_control.stop_reason = (uint32_t)reason;
    reset_requested = 1U; // 退出控制后不能把以前累积的积分带入下次启动。
    clear_integral_requested = 0U;
    clear_angle_integral_requested = 0U;
    brake_latched = 0U;
    request_revision++;
}

void MotorControl_Stop(MotorStopReason reason)
{
    taskENTER_CRITICAL(); // 一起修改请求字段，防止读取到半次更新。
    MotorControl_StopLocked(reason);
    taskEXIT_CRITICAL();
}

uint8_t MotorControl_SetSpeed(float rpm)
{
    GM6020_Feedback feedback;
    uint32_t now_ms;
    /* 有序比较同时拒绝 NaN、无穷和超限；不静默截断目标。 */
    if (!(rpm >= -MOTOR_SPEED_LIMIT_RPM && rpm <= MOTOR_SPEED_LIMIT_RPM))
    {
        return 0U;
    }
    taskENTER_CRITICAL(); // 校验反馈与发布请求作为一项事务，接收任务不能夹入失效反馈。
    GM6020_GetFeedback(&feedback); // 先取反馈再取时间，避免抢占引起负年龄。
    now_ms = HAL_GetTick();
    if (can_ready == 0U || GM6020_IsOnline(&feedback, now_ms) == 0U ||
        feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C)
    {
        taskEXIT_CRITICAL();
        return 0U; // 未收到新鲜反馈时，即使是 S0 也不能启动闭环。
    }
    if (motor_control.enabled != 0U &&
        motor_control.control_mode == MOTOR_CONTROL_SPEED_MODE && motor_control.target_rpm == rpm)
    {
        taskEXIT_CRITICAL();
        return 1U; // 重复滑块值不会重建状态，也不会重复清除零速积分。
    }
    if (motor_control.enabled == 0U || motor_control.control_mode != MOTOR_CONTROL_SPEED_MODE)
    {
        reset_requested = 1U; // 从停止状态重新启动时清零历史。
        have_step_time = 0U; // 首轮以名义 2 ms 建立时间基准。
        brake_latched = 0U;
    }
    else if ((motor_control.target_rpm > 0.0f && rpm <= 0.0f) ||
             (motor_control.target_rpm < 0.0f && rpm >= 0.0f))
    {
        /* 例如旧 I=1800、P50、15→-10 时，P=-1250，却仍输出 +550。
         * 下一控制周期去掉旧 I，制动不再等待积分慢慢反向累积。
         * 同向变速保留原 I；串口仅置位，由控制任务处理 PID 历史。
         */
        clear_integral_requested = 1U;
    }
    motor_control.control_mode = MOTOR_CONTROL_SPEED_MODE;
    motor_control.target_rpm = rpm;
    motor_control.enabled = 1U; // S0 也启用：控制器可反向施力降低实际转速。
    motor_control.stop_reason = MOTOR_STOP_NONE;
    motor_control.settled = 0U;
    motor_control.settling_time_ms = 0U;
    motor_control.overshoot_deg = 0.0f;
    angle_metrics.band_active = 0U;
    request_revision++;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t MotorControl_SetAngle(float degrees)
{
    GM6020_Feedback feedback;
    uint32_t now_ms;
    float step_error;
    if (!(degrees >= -ANGLE_TARGET_LIMIT_DEG && degrees <= ANGLE_TARGET_LIMIT_DEG))
    {
        return 0U;
    }
    taskENTER_CRITICAL();
    GM6020_GetFeedback(&feedback);
    now_ms = HAL_GetTick();
    if (can_ready == 0U || GM6020_IsOnline(&feedback, now_ms) == 0U ||
        feedback.angle_valid == 0U || feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C)
    {
        taskEXIT_CRITICAL();
        return 0U; // 新鲜速度反馈也不能替代连续可信的多圈位置。
    }
    if (motor_control.enabled != 0U && motor_control.control_mode == MOTOR_CONTROL_ANGLE_MODE)
    {
        if (active_angle_epoch != feedback.angle_epoch)
        {
            MotorControl_StopLocked(MOTOR_STOP_ANGLE_INVALID);
            taskEXIT_CRITICAL();
            return 0U; // 先停止旧零点的请求，不能直接恢复成新的同模式运行。
        }
        if (motor_control.target_angle_deg == degrees)
        {
            taskEXIT_CRITICAL();
            return 1U; // 重复滑块值不重计阶跃时间，不反复清历史。
        }
        clear_angle_integral_requested = 1U; // 新目标不继承旧偏置，D 与内环历史保留。
    }
    else
    {
        reset_requested = 1U; // 从 STOP 或速度模式切换：两环全清。
        have_step_time = 0U;
        brake_latched = 0U;
    }
    active_angle_epoch = feedback.angle_epoch;
    motor_control.control_mode = MOTOR_CONTROL_ANGLE_MODE;
    motor_control.target_angle_deg = degrees;
    motor_control.actual_angle_deg = feedback.relative_angle_deg;
    motor_control.error_angle_deg = degrees - feedback.relative_angle_deg;
    motor_control.angle_valid = 1U;
    motor_control.target_rpm = 0.0f; // 下轮由外环计算，绝不当作 S0 重建内环。
    motor_control.enabled = 1U;
    motor_control.stop_reason = MOTOR_STOP_NONE;
    motor_control.settled = 0U;
    motor_control.settling_time_ms = 0U;
    motor_control.overshoot_deg = 0.0f;
    step_error = degrees - feedback.relative_angle_deg;
    angle_metrics.start_ms = now_ms;
    angle_metrics.band_entry_ms = now_ms;
    angle_metrics.direction = (step_error > 0.0f) ? 1.0f : ((step_error < 0.0f) ? -1.0f : 0.0f);
    angle_metrics.band_active = 0U;
    request_revision++;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t MotorControl_ZeroAngle(void)
{
    GM6020_Feedback feedback;
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    GM6020_GetFeedback(&feedback); // 嵌套临界区：STOP 检查和换零点必须是同一事务。
    if (motor_control.enabled == 0U &&
        GM6020_IsOnline(&feedback, HAL_GetTick()) != 0U &&
        MotorControl_Abs((float)feedback.speed_rpm) <= ANGLE_ZERO_SPEED_RPM &&
        GM6020_ZeroAngle() != 0U)
    {
        GM6020_GetFeedback(&feedback);
        active_angle_epoch = feedback.angle_epoch;
        motor_control.target_angle_deg = 0.0f;
        motor_control.actual_angle_deg = 0.0f;
        motor_control.error_angle_deg = 0.0f;
        motor_control.angle_valid = feedback.angle_valid;
        motor_control.settled = 0U;
        motor_control.settling_time_ms = 0U;
        motor_control.overshoot_deg = 0.0f;
        memset(&angle_metrics, 0, sizeof(angle_metrics));
        reset_requested = 1U;
        brake_latched = 0U;
        request_revision++;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static uint8_t MotorControl_SetGainForLoop(char term, float value, uint8_t outer_loop)
{
    float maximum;
    volatile float *gain; // 指向共享请求字段，保留 volatile 属性。
    if (term == 'P') { maximum = outer_loop ? ANGLE_PID_KP_MAX : SPEED_PID_KP_MAX; }
    else if (term == 'I') { maximum = outer_loop ? ANGLE_PID_KI_MAX : SPEED_PID_KI_MAX; }
    else if (term == 'D') { maximum = outer_loop ? ANGLE_PID_KD_MAX : SPEED_PID_KD_MAX; }
    else { return 0U; }
    if (!(value >= 0.0f && value <= maximum))
    {
        return 0U; // 拒绝负参数、非有限数或超出教学输入范围的参数。
    }
    taskENTER_CRITICAL();
    /* 不直接操作两个 PID；参数在下一轮由控制任务统一应用。 */
    if (outer_loop != 0U)
    {
        if (term == 'P') { gain = &motor_control.angle_kp; }
        else if (term == 'I') { gain = &motor_control.angle_ki; }
        else { gain = &motor_control.angle_kd; }
    }
    else
    {
        if (term == 'P') { gain = &motor_control.kp; }
        else if (term == 'I') { gain = &motor_control.ki; }
        else { gain = &motor_control.kd; }
    }
    if (*gain != value) // 滑块重复发送相同值时不产生额外状态变化。
    {
        *gain = value;
        request_revision++;
    }
    taskEXIT_CRITICAL();
    /* 不改变 enabled，也不清历史；Ki=0 时由 PID_SetGains 明确清除积分。 */
    return 1U;
}

uint8_t MotorControl_SetGain(char term, float value)
{
    return MotorControl_SetGainForLoop(term, value, 0U);
}

uint8_t MotorControl_SetAngleGain(char term, float value)
{
    return MotorControl_SetGainForLoop(term, value, 1U);
}

uint8_t MotorControl_SetSpeedLimit(float rpm)
{
    if (!(rpm >= 1.0f && rpm <= MOTOR_SPEED_LIMIT_RPM))
    {
        return 0U;
    }
    taskENTER_CRITICAL();
    if (motor_control.speed_limit_rpm != rpm)
    {
        motor_control.speed_limit_rpm = rpm;
        request_revision++;
    }
    taskEXIT_CRITICAL();
    return 1U; // 只限角度外环速度，S 仍允许 ±100 rpm。
}

uint8_t MotorControl_KeepAlive(void)
{
    uint8_t accepted = 0U;
    taskENTER_CRITICAL();
    accepted = (motor_control.enabled != 0U) ? 1U : 0U; // 兼容旧命令，不续期、不启动。
    taskEXIT_CRITICAL();
    return accepted;
}

void MotorControl_GetState(MotorControl_State *snapshot)
{
    if (snapshot != NULL)
    {
        taskENTER_CRITICAL();
        *snapshot = motor_control; // 一次复制目标、实际和计算项，VOFA 两曲线使用同一轮状态。
        taskEXIT_CRITICAL();
    }
}

static void MotorControl_UpdateAngleMetrics(MotorControl_State *state,
    AngleStepMetrics *metrics, uint32_t now_ms)
{
    float overshoot = metrics->direction * (state->actual_angle_deg - state->target_angle_deg);
    if (overshoot > state->overshoot_deg) { state->overshoot_deg = overshoot; }
    if (MotorControl_Abs(state->error_angle_deg) <= ANGLE_SETTLE_ERROR_DEG)
    {
        if (metrics->band_active == 0U)
        {
            metrics->band_entry_ms = now_ms;
            metrics->band_active = 1U;
        }
        if ((uint32_t)(now_ms - metrics->band_entry_ms) >= ANGLE_SETTLE_HOLD_MS)
        {
            state->settled = 1U;
            state->settling_time_ms = (uint32_t)(metrics->band_entry_ms - metrics->start_ms);
        }
    }
    else
    {
        metrics->band_active = 0U;
        state->settled = 0U;
        state->settling_time_ms = 0U;
    }
}

void MotorControl_Step(void)
{
    GM6020_Feedback feedback;
    MotorControl_State state;
    PID_Controller candidate; // 本轮运算副本；新请求到达时丢弃，旧计算不污染已提交历史。
    PID_Controller angle_candidate;
    AngleStepMetrics metrics_candidate;
    uint32_t now_ms, elapsed_ms, revision, id;
    uint8_t online, rebuild, clear_integral, clear_angle_integral, brake_candidate;
    uint8_t data[8];
    float dt_s, desired_rpm, output = 0.0f;
    int16_t raw = 0;

    GM6020_GetFeedback(&feedback);
    now_ms = HAL_GetTick();
    online = GM6020_IsOnline(&feedback, now_ms);
    elapsed_ms = have_step_time ? (uint32_t)(now_ms - last_step_ms) : MOTOR_CONTROL_PERIOD_MS;

    taskENTER_CRITICAL();
    motor_control.cycle_count++;
    motor_control.actual_rpm = (float)feedback.speed_rpm;
    motor_control.actual_angle_deg = feedback.relative_angle_deg;
    motor_control.angle_valid = (online != 0U && feedback.angle_valid != 0U) ? 1U : 0U;
    motor_control.online = online;
    if (motor_control.enabled != 0U)
    {
        MotorStopReason reason = MOTOR_STOP_NONE;
        if (motor_control.control_mode == MOTOR_CONTROL_ANGLE_MODE &&
            (motor_control.angle_valid == 0U || feedback.angle_epoch != active_angle_epoch))
        {
            reason = MOTOR_STOP_ANGLE_INVALID; // 掉线也使圈数不可证明，不允许自动恢复。
        }
        else if (online == 0U) { reason = MOTOR_STOP_FEEDBACK; }
        else if (feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C) { reason = MOTOR_STOP_TEMPERATURE; }
        else if (elapsed_ms > MOTOR_CONTROL_MAX_GAP_MS) { reason = MOTOR_STOP_CONTROL_TIMING; }
        if (reason != MOTOR_STOP_NONE) { MotorControl_StopLocked(reason); }
    }
    motor_control.error_rpm = motor_control.target_rpm - motor_control.actual_rpm;
    motor_control.error_angle_deg = motor_control.target_angle_deg - motor_control.actual_angle_deg;
    state = motor_control; // 数学运算使用这一份请求快照。
    revision = request_revision;
    metrics_candidate = angle_metrics;
    brake_candidate = brake_latched;
    if (state.enabled != 0U && elapsed_ms == 0U)
    {
        /* 同一毫秒不重复积分，也不把旧方向输出发送成新目标的结果。
         * 不消费待处理标记，下个有真实 dt 的控制周期再计算。
         */
        taskEXIT_CRITICAL();
        return;
    }
    rebuild = reset_requested;
    reset_requested = 0U;
    clear_integral = clear_integral_requested;
    clear_integral_requested = 0U;
    clear_angle_integral = clear_angle_integral_requested;
    clear_angle_integral_requested = 0U;
    taskEXIT_CRITICAL();

    candidate = speed_pid;
    angle_candidate = angle_pid;
    if (rebuild != 0U)
    {
        PID_Config config = MotorControl_Config(state.kp, state.ki, state.kd);
        PID_Config angle_config = MotorControl_AngleConfig(state.angle_kp, state.angle_ki,
            state.angle_kd, state.speed_limit_rpm);
        (void)PID_Init(&candidate, &config); // 停止后重新启动才清空全部历史。
        (void)PID_Init(&angle_candidate, &angle_config);
        brake_candidate = 0U;
    }
    else
    {
        (void)PID_SetGains(&candidate, state.kp, state.ki, state.kd);
        (void)PID_SetGains(&angle_candidate, state.angle_kp, state.angle_ki, state.angle_kd);
        angle_candidate.config.output_limit = state.speed_limit_rpm; // V 在线改限幅，保留历史。
        // 在线调参保留已有积分与测量历史；I0 清积分，重复增益不重置。
    }
    if (clear_integral != 0U)
    {
        PID_ClearIntegral(&candidate); // 反转/零速只去掉原方向积分，D 继续用实际值变化。
    }
    if (clear_angle_integral != 0U) { PID_ClearIntegral(&angle_candidate); }
    if (state.enabled == 0U)
    {
        PID_Reset(&candidate);
        PID_Reset(&angle_candidate);
        brake_candidate = 0U;
    }
    else
    {
        dt_s = (float)elapsed_ms * 0.001f;
        desired_rpm = state.target_rpm;
        if (state.control_mode == MOTOR_CONTROL_ANGLE_MODE)
        {
            uint8_t braking;
            desired_rpm = PID_Update(&angle_candidate, state.target_angle_deg,
                state.actual_angle_deg, dt_s);
            /* 外环减速时旧内环 I 可能继续驱动。显著制动阶段只在入口
             * 清一次 I，随后正常累积，不随每次外环速度微调清积分。
             * 阈值避开 ±1 rpm 量化噪声；D 历史始终保留。
             */
            braking = (uint8_t)((state.actual_rpm > ANGLE_BRAKE_MIN_SPEED_RPM &&
                desired_rpm < state.actual_rpm - ANGLE_BRAKE_SPEED_ERROR_RPM) ||
                (state.actual_rpm < -ANGLE_BRAKE_MIN_SPEED_RPM &&
                desired_rpm > state.actual_rpm + ANGLE_BRAKE_SPEED_ERROR_RPM));
            if (braking != 0U && brake_candidate == 0U &&
                candidate.integral * state.actual_rpm > 0.0f)
            {
                PID_ClearIntegral(&candidate);
            }
            brake_candidate = braking;
            state.target_rpm = desired_rpm; // 速度环曲线，不调用公共 SetSpeed。
            state.error_rpm = desired_rpm - state.actual_rpm;
            MotorControl_UpdateAngleMetrics(&state, &metrics_candidate, now_ms);
        }
        output = PID_Update(&candidate, desired_rpm, state.actual_rpm, dt_s);
        raw = (int16_t)(output >= 0.0f ? output + 0.5f : output - 0.5f); // 限幅后四舍五入成协议整数。
    }

    taskENTER_CRITICAL();
    /* CAN 接收任务优先级高于控制任务，计算过程中可能发布新的失效帧。
     * 只再次校验条件，不用新测量重算 PID；正常提交仍是一轮一致快照。
     */
    if (motor_control.enabled != 0U)
    {
        GM6020_Feedback latest_feedback;
        MotorStopReason reason = MOTOR_STOP_NONE;
        uint8_t latest_online;
        GM6020_GetFeedback(&latest_feedback);
        latest_online = GM6020_IsOnline(&latest_feedback, HAL_GetTick());
        if (motor_control.control_mode == MOTOR_CONTROL_ANGLE_MODE &&
            (latest_online == 0U || latest_feedback.angle_valid == 0U ||
             latest_feedback.angle_epoch != active_angle_epoch))
        {
            reason = MOTOR_STOP_ANGLE_INVALID;
        }
        else if (latest_online == 0U) { reason = MOTOR_STOP_FEEDBACK; }
        else if (latest_feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C)
        {
            reason = MOTOR_STOP_TEMPERATURE;
        }
        if (reason != MOTOR_STOP_NONE)
        {
            online = latest_online; // 已掉线时沿用原有不向无 ACK 总线塞帧的路径。
            motor_control.actual_rpm = (float)latest_feedback.speed_rpm;
            motor_control.actual_angle_deg = latest_feedback.relative_angle_deg;
            motor_control.online = latest_online;
            motor_control.angle_valid =
                (latest_online != 0U && latest_feedback.angle_valid != 0U) ? 1U : 0U;
            MotorControl_StopLocked(reason); // revision 增加，下面统一丢弃计算副本。
        }
    }
    if (revision != request_revision)
    {
        raw = 0; // 如果已有新的请求，下一轮重新计算，避免发送旧结果。
        /* 丢弃副本而非清空原 PID；同向改目标/调参不会意外丢失旧积分。
         * 将本轮消费的标记与新请求合并，STOP 或反转不能被旧计算覆盖。
         */
        reset_requested |= rebuild;
        clear_integral_requested |= clear_integral;
        clear_angle_integral_requested |= clear_angle_integral;
    }
    else
    {
        speed_pid = candidate; // 只有请求版本一致时才提交历史和控制时间。
        angle_pid = angle_candidate;
        angle_metrics = metrics_candidate;
        brake_latched = brake_candidate;
        have_step_time = (state.enabled != 0U) ? 1U : 0U;
        if (have_step_time != 0U) { last_step_ms = now_ms; }
        motor_control.target_rpm = state.target_rpm;
        motor_control.error_rpm = state.error_rpm;
        motor_control.p_term = candidate.p_term;
        motor_control.i_term = candidate.i_term;
        motor_control.d_term = candidate.d_term;
        motor_control.angle_p_term = angle_candidate.p_term;
        motor_control.angle_i_term = angle_candidate.i_term;
        motor_control.angle_d_term = angle_candidate.d_term;
        motor_control.settled = state.settled;
        motor_control.settling_time_ms = state.settling_time_ms;
        motor_control.overshoot_deg = state.overshoot_deg;
    }
    motor_control.output_raw = raw;
    taskEXIT_CRITICAL();

    if (can_ready == 0U) { return; }
    /* 正常 1 Mbps 下，上轮命令应在下个 2 ms 周期前发完；不堆积过期控制量。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U)
    {
        can1_tx_busy_count++;
        MotorControl_Stop(MOTOR_STOP_CAN_TX);
        CanTransport_AbortPending(); // 取消旧帧自动重发，避免恢复连接后继续发送旧控制量。
        return;
    }
    if (online == 0U) { return; } // 未见反馈时不向无 ACK 总线反复塞报文。
    if (GM6020_BuildCommand(GM6020_MOTOR_ID, GM6020_CONTROL_MODE, raw, &id, data) == 0U ||
        CanTransport_Send(id, data) != HAL_OK)
    {
        MotorControl_Stop(MOTOR_STOP_CAN_TX);
        CanTransport_AbortPending();
        return;
    }
    taskENTER_CRITICAL();
    motor_control.last_sent_raw = raw; // 记录提交成功，不能据此宣称电机已执行。
    motor_control.last_tx_id = id;
    for (uint8_t i = 0U; i < 8U; i++) { motor_control.last_tx_data[i] = data[i]; }
    taskEXIT_CRITICAL();
}
