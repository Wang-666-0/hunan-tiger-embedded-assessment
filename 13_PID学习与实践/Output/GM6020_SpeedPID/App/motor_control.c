/*
 * 文件功能：实现 GM6020 的转速单环控制。
 * 1. S 命令锁存目标 rpm 并持续运行；P/I/D 可由 VOFA 滑块在线改变。
 * 2. 每 2 ms 用 CAN 转速反馈计算误差，调用 pid.c，再发送限幅控制量。
 * 3. 反馈、温度、控制时序或 CAN 发送异常时撤销请求；不再等待 K 续期。
 * 4. 提供一致的控制状态，供 VOFA 显示目标、实际值和调参信息。
 * 5. 反转或非零目标切到零时只清积分，保留微分；同向变速保留历史。
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

volatile MotorControl_State motor_control;
static PID_Controller speed_pid; // 只有控制任务读写 PID 的积分和微分历史。
static uint8_t reset_requested;  // 串口要求清空历史，由下一轮控制任务执行。
static uint8_t clear_integral_requested; // 反转/零速请求只清旧积分，不丢失实际值和 D 历史。
static uint8_t have_step_time;   // 第一次启用时还没有可用的上轮控制时间。
static uint32_t last_step_ms;    // 用真实运行间隔计算 dt，而不是盲用固定常数。
static uint32_t request_revision; // 防止计算时出现的新停止请求被旧结果覆盖。

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

void MotorControl_Init(void)
{
    PID_Config config = MotorControl_Config(SPEED_PID_DEFAULT_KP,
        SPEED_PID_DEFAULT_KI, SPEED_PID_DEFAULT_KD);
    memset((void *)&motor_control, 0, sizeof(motor_control)); // 创建任务前调用，初始输出为零。
    motor_control.kp = config.kp;
    motor_control.ki = config.ki;
    motor_control.kd = config.kd;
    motor_control.mode = GM6020_CONTROL_MODE;
    motor_control.motor_id = GM6020_MOTOR_ID;
    motor_control.stop_reason = MOTOR_STOP_POWER_ON;
    (void)PID_Init(&speed_pid, &config); // 初值在 app_config.h，暂不自动启动电机。
    reset_requested = 1U;
    clear_integral_requested = 0U;
    have_step_time = 0U;
    last_step_ms = 0U;
    request_revision = 0U;
}

/* 必须在短临界区内调用。只撤销请求，硬件发送由 Step 统一处理。 */
static void MotorControl_StopLocked(MotorStopReason reason)
{
    motor_control.enabled = 0U;
    motor_control.target_rpm = 0.0f; // 清掉旧目标，条件恢复后不会自行重启。
    motor_control.error_rpm = -motor_control.actual_rpm;
    motor_control.output_raw = 0;
    motor_control.p_term = 0.0f;
    motor_control.i_term = 0.0f;
    motor_control.d_term = 0.0f;
    motor_control.stop_reason = (uint32_t)reason;
    reset_requested = 1U; // 退出控制后不能把以前累积的积分带入下次启动。
    clear_integral_requested = 0U;
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
    GM6020_GetFeedback(&feedback); // 先取反馈再取时间，避免抢占引起负年龄。
    now_ms = HAL_GetTick();
    if (can_ready == 0U || GM6020_IsOnline(&feedback, now_ms) == 0U ||
        feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C)
    {
        return 0U; // 未收到新鲜反馈时，即使是 S0 也不能启动闭环。
    }
    taskENTER_CRITICAL();
    if (motor_control.enabled != 0U && motor_control.target_rpm == rpm)
    {
        taskEXIT_CRITICAL();
        return 1U; // 重复滑块值不会重建状态，也不会重复清除零速积分。
    }
    if (motor_control.enabled == 0U)
    {
        reset_requested = 1U; // 从停止状态重新启动时清零历史。
        have_step_time = 0U; // 首轮以名义 2 ms 建立时间基准。
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
    motor_control.target_rpm = rpm;
    motor_control.enabled = 1U; // S0 也启用：控制器可反向施力降低实际转速。
    motor_control.stop_reason = MOTOR_STOP_NONE;
    request_revision++;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t MotorControl_SetGain(char term, float value)
{
    float maximum;
    volatile float *gain; // 指向共享请求字段，保留 volatile 属性。
    if (term == 'P') { maximum = SPEED_PID_KP_MAX; }
    else if (term == 'I') { maximum = SPEED_PID_KI_MAX; }
    else if (term == 'D') { maximum = SPEED_PID_KD_MAX; }
    else { return 0U; }
    if (!(value >= 0.0f && value <= maximum))
    {
        return 0U; // 拒绝负参数、非有限数或超出教学输入范围的参数。
    }
    taskENTER_CRITICAL();
    /* 不直接操作 speed_pid；参数在下一轮由控制任务统一应用。 */
    if (term == 'P') { gain = &motor_control.kp; }
    else if (term == 'I') { gain = &motor_control.ki; }
    else { gain = &motor_control.kd; }
    if (*gain != value) // 滑块重复发送相同值时不产生额外状态变化。
    {
        *gain = value;
        request_revision++;
    }
    taskEXIT_CRITICAL();
    /* 不改变 enabled，也不清历史；Ki=0 时由 PID_SetGains 明确清除积分。 */
    return 1U;
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

void MotorControl_Step(void)
{
    GM6020_Feedback feedback;
    MotorControl_State state;
    PID_Controller candidate; // 本轮运算副本；新请求到达时丢弃，旧计算不污染已提交历史。
    uint32_t now_ms, elapsed_ms, revision, id;
    uint8_t online, rebuild, clear_integral;
    uint8_t data[8];
    float output = 0.0f;
    int16_t raw = 0;

    GM6020_GetFeedback(&feedback);
    now_ms = HAL_GetTick();
    online = GM6020_IsOnline(&feedback, now_ms);
    elapsed_ms = have_step_time ? (uint32_t)(now_ms - last_step_ms) : MOTOR_CONTROL_PERIOD_MS;

    taskENTER_CRITICAL();
    motor_control.cycle_count++;
    motor_control.actual_rpm = (float)feedback.speed_rpm;
    motor_control.online = online;
    if (motor_control.enabled != 0U)
    {
        MotorStopReason reason = MOTOR_STOP_NONE;
        if (online == 0U) { reason = MOTOR_STOP_FEEDBACK; }
        else if (feedback.temperature >= MOTOR_TEMPERATURE_LIMIT_C) { reason = MOTOR_STOP_TEMPERATURE; }
        else if (elapsed_ms > MOTOR_CONTROL_MAX_GAP_MS) { reason = MOTOR_STOP_CONTROL_TIMING; }
        if (reason != MOTOR_STOP_NONE) { MotorControl_StopLocked(reason); }
    }
    motor_control.error_rpm = motor_control.target_rpm - motor_control.actual_rpm;
    state = motor_control; // 数学运算使用这一份请求快照。
    revision = request_revision;
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
    taskEXIT_CRITICAL();

    candidate = speed_pid;
    if (rebuild != 0U)
    {
        PID_Config config = MotorControl_Config(state.kp, state.ki, state.kd);
        (void)PID_Init(&candidate, &config); // 停止后重新启动才清空全部历史。
    }
    else
    {
        (void)PID_SetGains(&candidate, state.kp, state.ki, state.kd);
        // 在线调参保留已有积分与测量历史；I0 清积分，重复增益不重置。
    }
    if (clear_integral != 0U)
    {
        PID_ClearIntegral(&candidate); // 反转/零速只去掉原方向积分，D 继续用实际值变化。
    }
    if (state.enabled == 0U)
    {
        PID_Reset(&candidate);
    }
    else
    {
        output = PID_Update(&candidate, state.target_rpm, state.actual_rpm,
            (float)elapsed_ms * 0.001f); // 毫秒换秒，使 Ki/Kd 的含义不依赖循环频率。
        raw = (int16_t)(output >= 0.0f ? output + 0.5f : output - 0.5f); // 限幅后四舍五入成协议整数。
    }

    taskENTER_CRITICAL();
    if (revision != request_revision)
    {
        raw = 0; // 如果已有新的请求，下一轮重新计算，避免发送旧结果。
        /* 丢弃副本而非清空原 PID；同向改目标/调参不会意外丢失旧积分。
         * 将本轮消费的标记与新请求合并，STOP 或反转不能被旧计算覆盖。
         */
        reset_requested |= rebuild;
        clear_integral_requested |= clear_integral;
    }
    else
    {
        speed_pid = candidate; // 只有请求版本一致时才提交历史和控制时间。
        have_step_time = (state.enabled != 0U) ? 1U : 0U;
        if (have_step_time != 0U) { last_step_ms = now_ms; }
        motor_control.p_term = candidate.p_term;
        motor_control.i_term = candidate.i_term;
        motor_control.d_term = candidate.d_term;
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
