/**
 * @file pitch_controller.c
 * 作用：Pitch 直接阶跃与手动控制，包含双环 PID、重力补偿和末段制动。
 * 对外函数：
 *   PitchController_StartMove / PitchController_Update / PitchController_ReturnTo /
 *   PitchController_OnFeedback / PitchController_CaptureReference / PitchController_SetTarget /
 *   PitchController_Init / PitchController_Stop / PitchController_SetFeedbackDivisor /
 *   PitchController_SetPid / PitchController_SetBrakeEnabled / PitchController_SetGravityOffset /
 *   PitchController_CaptureHorizontal / PitchController_SetGravityGain /
 *   PitchController_InvalidateGravity
 * 内部函数：
 *   terminal_brake / gravity_at / normalize_degrees / observe_pitch / reset_brake /
 *   clear_brake_statistics / finite_value / pitch_controller_absolute / clamp / fault / stationary_safe
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "pitch_controller.h"
#include <float.h>
#include <math.h>
#include <string.h>

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static uint8_t finite_value(float x);
static float pitch_controller_absolute(float x);
static float clamp(float x, float lo, float hi);
static float normalize_degrees(float x);
static void observe_pitch(PitchController *pitch, float delta_deg, uint32_t gap_ms);
static float gravity_at(const PitchController *pitch, float kg);
static void reset_brake(PitchController *pitch);
static void clear_brake_statistics(PitchController *pitch);
static float terminal_brake(PitchController *pitch, uint32_t age);
static void fault(PitchController *pitch, uint32_t reasons);
static uint8_t stationary_safe(const PitchController *pitch);

uint8_t PitchController_StartMove(PitchController *pitch, float degrees, uint32_t now_ms)
/*
 * 作用：接受 Pitch 的直接角度目标，并为这次离散运动建立制动方向。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 degrees：软件坐标中的目标角度，deg。
 * 参数 now_ms：当前毫秒时间。
 * 流程：目标必须在范围内，控制器已运行且反馈新鲜；相同目标不重复启动动作。
 * 流程：直接设置最终目标，不生成轨迹整形；保留已有 PID 参数和积分。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    float span;
    if (!pitch || !finite_value(degrees) || !pitch->active || !pitch->have_reference ||
        !pitch->have_feedback || pitch->faults || pitch->last_status != 1U ||
        now_ms - pitch->last_feedback_ms >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS ||
        degrees < PITCH_CONTROLLER_TARGET_MIN_DEG || degrees > PITCH_CONTROLLER_TARGET_MAX_DEG) {
        return 0U;
    }
    if (degrees == pitch->target_deg) { return 1U; }
    span = degrees - pitch->angle_deg;
    if (!PitchController_SetTarget(pitch, degrees)) { return 0U; }
    if (pitch->brake.enabled && pitch_controller_absolute(span) >= 1.0f) {
        pitch->brake.direction = span > 0.0f ? 1.0f : -1.0f;
        pitch->brake.target_deg = degrees;
        pitch->brake.tracking = 1U;
    }
    return 1U;
}

float PitchController_Update(PitchController *pitch, float rate_dps, uint32_t now_ms, uint8_t allowed)
/*
 * 作用：执行 Pitch 手动目标更新、重力补偿、末段制动和共用双环 PID。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 rate_dps：摇杆请求的角速度，deg/s。
 * 参数 now_ms：当前毫秒时间。
 * 参数 allowed：本轮是否允许输出动力。
 * 流程：使用电机真实反馈计算误差；控制台的模拟显示角度不会进入此函数。
 * 流程：手动接管撤去旧动作的制动状态，目标按实际控制间隔累加并限制在行程内。
 * 流程：重力项与制动项在力矩层相加，再由共用 PID 统一限幅和处理积分饱和。
 * 返回本次计算的协议力矩，N*m；此函数本身不提交 CAN。
 */
{
    uint32_t age, elapsed;
    float torque, feedforward;
    if (!pitch) { return 0.0f; }
    if (!finite_value(rate_dps) || !finite_value(pitch->gravity_kg_nm) ||
        !finite_value(pitch->gravity_offset_deg) || pitch_controller_absolute(pitch->gravity_kg_nm) > PITCH_CONTROLLER_TORQUE_LIMIT_NM ||
        pitch_controller_absolute(pitch->gravity_offset_deg) > 180.0f) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0.0f;
    }
    age = now_ms - pitch->last_feedback_ms;
    if (!pitch->have_feedback || age >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
        if (allowed || pitch->active) { fault(pitch, PITCH_CONTROLLER_FAULT_FEEDBACK); }
        else { PitchController_Stop(pitch); }
        return 0.0f;
    }
    if (!allowed || !pitch->have_reference || pitch->faults ||
        (pitch->gravity_kg_nm != 0.0f && !pitch->gravity_calibrated)) {
        PitchController_Stop(pitch); return 0.0f;
    }
    if (pitch->last_status != 1U) { fault(pitch, PITCH_CONTROLLER_FAULT_STATUS); return 0.0f; }
    if (pitch->angle_deg < PITCH_CONTROLLER_HARD_MIN_DEG || pitch->angle_deg > PITCH_CONTROLLER_HARD_MAX_DEG) {
        fault(pitch, PITCH_CONTROLLER_FAULT_LIMIT); return 0.0f;
    }
    elapsed = pitch->have_control_tick ? now_ms - pitch->last_control_ms : 0U;
    pitch->last_control_ms = now_ms; pitch->have_control_tick = 1U;
    if (elapsed > 10U) { fault(pitch, PITCH_CONTROLLER_FAULT_TIMING); return 0.0f; }
    pitch->active = 1U;
    rate_dps = clamp(rate_dps, -APP_PITCH_RATE_LIMIT_DPS, APP_PITCH_RATE_LIMIT_DPS);
    if (rate_dps != 0.0f) { reset_brake(pitch); } // 手动接管撤去旧动作的制动力矩。
    pitch->target_deg = clamp(pitch->target_deg + rate_dps * (float)elapsed * 0.001f,
        PITCH_CONTROLLER_TARGET_MIN_DEG, PITCH_CONTROLLER_TARGET_MAX_DEG);
    /* 重力项和末段制动在力矩层叠加；串级 PID 内统一限幅并处理抗饱和。
     * 此处角度来自电机反馈，绝不使用控制台的演示角度。 */
    pitch->gravity_torque_nm = gravity_at(pitch, pitch->gravity_kg_nm);
    feedforward = pitch->gravity_torque_nm + terminal_brake(pitch, age);
    torque = GimbalPid_UpdateWithFeedforward(&pitch->cascade, pitch->target_deg - pitch->angle_deg,
        pitch->last_feedback_ms, pitch->last_sequence, PITCH_CONTROLLER_TORQUE_LIMIT_NM,
        feedforward);
    pitch->pid_torque_nm = pitch->cascade.unsaturated_torque_nm - feedforward;
    if (!finite_value(torque) || !finite_value(pitch->cascade.position_i) ||
        !finite_value(pitch->cascade.speed_i) || !finite_value(pitch->cascade.target_velocity_dps) ||
        !finite_value(pitch->cascade.unsaturated_torque_nm)) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0.0f;
    }
    pitch->torque_nm = torque;
    return torque;
}

static float terminal_brake(PitchController *pitch, uint32_t age)
/*
 * 作用：在接近目标或首次到位后回退时，计算限幅且缓升的附加制动力矩。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 age：最近反馈距当前的时间，ms。
 * 流程：接近阶段用剩余角度估计允许速度；实测接近速度过大时增加反向阻尼。
 * 流程：首次到位后的短窗口内，仅在原始速度、滤波速度和位置增量一致表明回退时抑制回退。
 * 流程：反馈过旧时撤去附加制动；正常 PID 和重力补偿仍由上层统一计算。
 * 返回本次计算的协议力矩，N*m；此函数本身不提交 CAN。
 */
{
    PitchTerminalBrake *brake = &pitch->brake;
    float remaining, closing, raw_closing, delta, weight, wanted = 0.0f, rise, previous;
    float output_direction, return_speed;
    uint32_t gap;
    uint8_t returning = 0U;
    if (!brake->enabled || !brake->tracking ||
        pitch->target_deg != brake->target_deg) { reset_brake(pitch); return 0.0f; }
    if (age > 10U) {
        brake->torque_nm = 0.0f; brake->active = 0U; return 0.0f;
    }
    if (brake->sequence == pitch->last_sequence) { return brake->torque_nm; }
    gap = pitch->last_feedback_ms - brake->feedback_ms;
    delta = brake->direction * (pitch->angle_deg - brake->previous_angle_deg);
    brake->sequence = pitch->last_sequence; brake->feedback_ms = pitch->last_feedback_ms;
    brake->previous_angle_deg = pitch->angle_deg;
    remaining = brake->direction * (pitch->target_deg - pitch->angle_deg);
    closing = brake->direction * pitch->cascade.velocity_dps;
    raw_closing = brake->direction * pitch->velocity_dps;
    if (!brake->arrived && remaining <= 0.2f) {
        brake->arrived = 1U; brake->arrival_ms = pitch->last_feedback_ms;
    }
    if (brake->arrived) {
        if (pitch->last_feedback_ms - brake->arrival_ms > 250U) {
            reset_brake(pitch); return 0.0f;
        }
        if (closing <= APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS ||
            raw_closing <= APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS || delta < -0.0001f) {
            brake->approach_done = 1U;
        }
    }
    output_direction = -brake->direction;
    if (brake->approach_done && remaining > 0.03f &&
        remaining < APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG && delta < -0.0001f &&
        closing < -APP_PITCH_CAPTURE_BRAKE_FLOOR_DPS &&
        raw_closing < -APP_PITCH_CAPTURE_BRAKE_FLOOR_DPS) {
        /* 只抵抗实际远离目标的回退；两路速度与位置增量必须一致。 */
        return_speed = -closing < -raw_closing ? -closing : -raw_closing;
        weight = clamp(APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG - remaining, 0.0f, 1.0f);
        weight = weight * weight * (3.0f - 2.0f * weight);
        wanted = clamp(0.010f *
            (return_speed - APP_PITCH_CAPTURE_BRAKE_FLOOR_DPS) * weight,
            0.0f, 0.30f);
        output_direction = brake->direction; returning = 1U;
    }
    /* 两路速度均指向目标才加阻尼；不沿用滤波滞后在静止/反向时继续加力。 */
    if (raw_closing < closing) { closing = raw_closing; }
    brake->safe_velocity_dps = sqrtf(2.0f * 3000.0f *
        (remaining > 0.0f ? remaining : 0.0f));
    if (brake->safe_velocity_dps < APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS) {
        brake->safe_velocity_dps = APP_PITCH_TERMINAL_BRAKE_FLOOR_DPS;
    }
    if (!brake->approach_done && pitch_controller_absolute(remaining) < APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG &&
        delta >= -0.0001f && closing > brake->safe_velocity_dps) {
        weight = 1.0f - pitch_controller_absolute(remaining) / APP_PITCH_TERMINAL_BRAKE_WINDOW_DEG;
        weight = weight * weight * (3.0f - 2.0f * weight);
        wanted = clamp(0.002f * (closing - brake->safe_velocity_dps) * weight,
            0.0f, 0.12f);
    }
    if (wanted <= 0.0f) {
        brake->torque_nm = 0.0f; brake->active = 0U; return 0.0f;
    }
    rise = (returning ? 12.0f : 6.0f) *
        (float)(gap > 10U ? 10U : gap) * 0.001f;
    /* 换向时先撤掉旧方向力矩，再从0缓升，避免跨零突跳。 */
    previous = brake->torque_nm * output_direction > 0.0f ? pitch_controller_absolute(brake->torque_nm) : 0.0f;
    if (wanted > previous + rise) { wanted = previous + rise; }
    brake->torque_nm = output_direction * wanted;
    brake->active = (uint8_t)(wanted > 0.0f);
    if (wanted > brake->peak_abs_nm) { brake->peak_abs_nm = wanted; }
    ++brake->active_frames;
    if (returning) {
        if (wanted > brake->return_peak_nm) { brake->return_peak_nm = wanted; }
        ++brake->return_frames;
    }
    return brake->torque_nm;
}

static float gravity_at(const PitchController *pitch, float kg)
/*
 * 作用：根据当前物理俯仰角计算重力补偿力矩。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 kg：重力补偿系数，N*m。
 * 流程：物理角度 = 软件相对角度 + 水平偏置；水平附近的力臂最大，因此使用余弦。
 * 流程：cosf 使用弧度，输入角度先由 deg 转换；Kg 的符号对应本机电机正方向。
 * 返回本次计算的协议力矩，N*m；此函数本身不提交 CAN。
 */
{ return kg * cosf((pitch->angle_deg + pitch->gravity_offset_deg) / PITCH_RAD_TO_DEG); }

uint8_t PitchController_ReturnTo(PitchController *pitch, float degrees, uint32_t now_ms)
/*
 * 作用：将归位位置作为新的直接目标接管当前动作。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 degrees：软件坐标中的目标角度，deg。
 * 参数 now_ms：当前毫秒时间。
 * 流程：复用 StartMove，不要求先完成旧动作；旧测量是否中止由后端负责记录。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    return PitchController_StartMove(pitch, degrees, now_ms);
}

uint8_t PitchController_OnFeedback(PitchController *pitch, const Dm3507Feedback *feedback)
/*
 * 作用：检查Pitch反馈有效性与连续性，更新实际角度、观测和运行故障。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 feedback：解码后的电机实际反馈。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    float delta = 0.0f, divisor;
    uint32_t gap = 0U;
    uint8_t first, rebase = 0U, was_active;
    if (!pitch || !feedback) { return 0U; }
    if (!finite_value(feedback->position_rad) || !finite_value(feedback->velocity_rad_s) ||
        !finite_value(feedback->torque_nm) || feedback->motor_id != DM3507_MOTOR_ID ||
        pitch_controller_absolute(feedback->position_rad) > DM3507_POSITION_MAX_RAD + 0.00001f ||
        pitch_controller_absolute(feedback->velocity_rad_s) > DM3507_VELOCITY_MAX_RAD_S + 0.00001f ||
        pitch_controller_absolute(feedback->torque_nm) > DM3507_TORQUE_MAX_NM + 0.00001f) {
        fault(pitch, PITCH_CONTROLLER_FAULT_DATA); pitch->stationary_samples = 0U; return 0U;
    }
    divisor = (float)pitch->feedback_divisor;
    if (divisor != 1.0f && divisor != 7.0f) { fault(pitch, PITCH_CONTROLLER_FAULT_DATA); return 0U; }
    first = (uint8_t)!pitch->have_feedback; was_active = pitch->active;
    if (!first) {
        gap = feedback->received_ms - pitch->last_feedback_ms;
        if (!gap || gap >= APP_TICK_HALF_RANGE) { return 0U; }
        delta = feedback->position_rad - pitch->last_position_rad;
        if (delta > DM3507_POSITION_MAX_RAD) { delta -= 2.0f * DM3507_POSITION_MAX_RAD; }
        else if (delta < -DM3507_POSITION_MAX_RAD) { delta += 2.0f * DM3507_POSITION_MAX_RAD; }
        if (gap >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS) {
            rebase = 1U; fault(pitch, PITCH_CONTROLLER_FAULT_FEEDBACK);
        }
    }
    pitch->last_position_rad = feedback->position_rad; pitch->last_feedback_ms = feedback->received_ms;
    ++pitch->last_sequence; pitch->have_feedback = 1U;
    pitch->velocity_dps = feedback->velocity_rad_s * PITCH_RAD_TO_DEG / divisor;
    pitch->last_status = feedback->status; pitch->temp_mos = feedback->temp_mos;
    pitch->temp_rotor = feedback->temp_rotor;
    if (first || rebase) {
        PitchController_InvalidateGravity(pitch);
        pitch->continuous_rad = pitch->reference_rad = feedback->position_rad;
        pitch->angle_deg = pitch->target_deg = 0.0f;

        pitch->have_reference = 0U; pitch->stationary_samples = 0U;
        GimbalPid_ResetObserver(&pitch->cascade);
    } else {
        pitch->continuous_rad += delta;
        pitch->angle_deg = (pitch->continuous_rad - pitch->reference_rad) * PITCH_RAD_TO_DEG / divisor;
        observe_pitch(pitch, delta * PITCH_RAD_TO_DEG / divisor, gap);
    }
    if (stationary_safe(pitch) && (first || rebase ||
        pitch_controller_absolute(delta) * PITCH_RAD_TO_DEG / divisor <= 0.05f + 0.5f * (float)gap * 0.001f)) {
        if (!pitch->stationary_samples) { pitch->stationary_started_ms = feedback->received_ms; }
        if (pitch->stationary_samples < 255U) { ++pitch->stationary_samples; }
    } else { pitch->stationary_samples = 0U; }
    if (feedback->fault_epoch) { fault(pitch, PITCH_CONTROLLER_FAULT_CAN); }
    if (feedback->status > 1U || (was_active && feedback->status != 1U)) {
        fault(pitch, PITCH_CONTROLLER_FAULT_STATUS);
    }
    if (feedback->temp_mos >= PITCH_CONTROLLER_DRIVER_LIMIT_C ||
        feedback->temp_rotor >= PITCH_CONTROLLER_MOTOR_LIMIT_C) {
        fault(pitch, PITCH_CONTROLLER_FAULT_TEMPERATURE);
    }
    /* 未捕获软件零点时允许观察绝对反馈，不猜水平或套GL的±60。 */
    if (pitch->have_reference && (pitch->angle_deg < PITCH_CONTROLLER_HARD_MIN_DEG ||
        pitch->angle_deg > PITCH_CONTROLLER_HARD_MAX_DEG)) {
        fault(pitch, PITCH_CONTROLLER_FAULT_LIMIT);
    }
    return 1U;
}

uint8_t PitchController_CaptureReference(PitchController *pitch)
/*
 * 作用：在原健康静止条件下建立Pitch软件零点，保留不可自动清除的故障。
 * 参数 pitch：Pitch 控制器实例。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (!pitch || pitch->active || pitch->last_status != 0U || !stationary_safe(pitch) || pitch->stationary_samples < 2U ||
        pitch->last_feedback_ms - pitch->stationary_started_ms < PITCH_CONTROLLER_STATIONARY_HOLD_MS ||
        (pitch->faults & ~(uint32_t)PITCH_RECOVERABLE_FAULTS)) { return 0U; }
    /* 改软件零时保持已标定的物理水平，不把新零误认为水平。 */
    if (pitch->have_reference && pitch->gravity_calibrated) {
        pitch->gravity_offset_deg = normalize_degrees(pitch->gravity_offset_deg + pitch->angle_deg);
    } else { PitchController_InvalidateGravity(pitch); }
    PitchController_Stop(pitch); pitch->reference_rad = pitch->continuous_rad;
    pitch->angle_deg = pitch->target_deg = 0.0f; pitch->have_reference = 1U;

    GimbalPid_ResetObserver(&pitch->cascade); pitch->faults &= ~(uint32_t)PITCH_RECOVERABLE_FAULTS;
    return 1U;
}

uint8_t PitchController_SetTarget(PitchController *pitch, float degrees)
/*
 * 作用：核对Pitch目标范围和参考有效性后，设置最终位置目标。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 degrees：软件坐标中的目标角度，deg。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (!pitch || !finite_value(degrees) || degrees < PITCH_CONTROLLER_TARGET_MIN_DEG ||
        degrees > PITCH_CONTROLLER_TARGET_MAX_DEG || !pitch->have_reference || pitch->faults) { return 0U; }
    pitch->target_deg = degrees;
    reset_brake(pitch);
    clear_brake_statistics(pitch);
    return 1U;
}

void PitchController_Init(PitchController *pitch)
/*
 * 作用：载入Pitch默认双环参数，初始化参考、观测和故障状态。
 * 参数 pitch：Pitch 控制器实例。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (!pitch) { return; }
    memset(pitch, 0, sizeof(*pitch)); pitch->feedback_divisor = 1U;
    pitch->brake.enabled = 1U;
    GimbalPid_Init(&pitch->cascade);
    /* 共用 PID 状态初始化后，加载 Pitch 自身的默认增益。 */
    pitch->cascade.position.kp = APP_PITCH_POSITION_KP;
    pitch->cascade.position.ki = APP_PITCH_POSITION_KI;
    pitch->cascade.position.kd = APP_PITCH_POSITION_KD;
    pitch->cascade.speed.kp = APP_PITCH_SPEED_KP;
    pitch->cascade.speed.ki = APP_PITCH_SPEED_KI;
    pitch->cascade.speed.kd = APP_PITCH_SPEED_KD;
    pitch->gravity_kg_nm = APP_PITCH_GRAVITY_KG_NM;
    pitch->cascade.velocity_limit_dps = 0.0f;
}

static float normalize_degrees(float x)
/*
 * 作用：把角度归一到连续等价的正负 180° 范围。
 * 参数 x：待处理的数值。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{ x = fmodf(x, 360.0f); if (x > 180.0f) { x -= 360.0f; } else if (x < -180.0f) { x += 360.0f; } return x; }

static void observe_pitch(PitchController *pitch, float delta_deg, uint32_t gap_ms)
/*
 * 作用：按 Pitch 独立滤波周期，由位置差分更新速度和加速度观测。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 delta_deg：相邻有效位置增量，deg。
 * 参数 gap_ms：相邻有效反馈的间隔，ms。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    GimbalPid *loop = &pitch->cascade;
    float previous, dt, alpha;
    if (!gap_ms || gap_ms >= PITCH_CONTROLLER_FEEDBACK_TIMEOUT_MS || !finite_value(delta_deg)) {
        GimbalPid_ResetObserver(loop); return;
    }
    dt = (float)gap_ms * 0.001f;
    previous = loop->velocity_dps;
    alpha = (float)gap_ms / ((float)2U + (float)gap_ms);
    loop->velocity_dps += alpha * (delta_deg / dt - loop->velocity_dps);
    /* 保留测量微分的平滑，短速度滤波不等于直接放大速度D噪声。 */
    alpha = (float)gap_ms / ((float)20U + (float)gap_ms);
    loop->acceleration_dps2 += alpha * ((loop->velocity_dps - previous) / dt - loop->acceleration_dps2);
}

static void reset_brake(PitchController *pitch)
/*
 * 作用：清除当前运动的制动跟踪状态和附加力矩。
 * 参数 pitch：Pitch 控制器实例。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    pitch->brake.torque_nm = pitch->brake.safe_velocity_dps = 0.0f;
    pitch->brake.tracking = pitch->brake.arrived = pitch->brake.active = pitch->brake.approach_done = 0U;
    pitch->brake.sequence = pitch->last_sequence;
    pitch->brake.feedback_ms = pitch->last_feedback_ms;
    pitch->brake.previous_angle_deg = pitch->angle_deg;
}

static void clear_brake_statistics(PitchController *pitch)
/*
 * 作用：清除制动峰值和生效帧数，供下一次动作独立统计。
 * 参数 pitch：Pitch 控制器实例。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    pitch->brake.peak_abs_nm = pitch->brake.return_peak_nm = 0.0f;
    pitch->brake.active_frames = pitch->brake.return_frames = 0U;
}

void PitchController_Stop(PitchController *pitch)
/*
 * 作用：停止Pitch当前控制并清理对应历史，保留原参数及安全参考规则。
 * 参数 pitch：Pitch 控制器实例。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (!pitch) { return; }
    pitch->active = pitch->have_control_tick = 0U; pitch->torque_nm = 0.0f;
    pitch->gravity_torque_nm = pitch->pid_torque_nm = 0.0f;
    pitch->gravity_i_transferred = 0U;
    GimbalPid_Stop(&pitch->cascade);
    pitch->target_deg = finite_value(pitch->angle_deg) ?
        clamp(pitch->angle_deg, PITCH_CONTROLLER_TARGET_MIN_DEG, PITCH_CONTROLLER_TARGET_MAX_DEG) : 0.0f;

    reset_brake(pitch);
}

uint8_t PitchController_SetFeedbackDivisor(PitchController *pitch, uint8_t divisor)
/*
 * 作用：仅接受合法的反馈轴侧换算，修改后使旧软件零点失效。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 divisor：反馈坐标除数，1 表示协议已为输出轴，7 表示需按电机轴换算。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (!pitch || (divisor != 1U && divisor != 7U) || pitch->active ||
        (pitch->have_feedback && pitch->last_status != 0U)) { return 0U; }
    if (pitch->feedback_divisor == divisor) { return 1U; }
    PitchController_Stop(pitch); pitch->feedback_divisor = divisor; pitch->have_reference = 0U;
    PitchController_InvalidateGravity(pitch);
    pitch->reference_rad = pitch->continuous_rad; pitch->angle_deg = pitch->target_deg = 0.0f;

    pitch->velocity_dps = pitch->velocity_dps * (divisor == 7U ? 1.0f / 7.0f : 7.0f);
    pitch->stationary_samples = 0U; GimbalPid_ResetObserver(&pitch->cascade);
    return 1U;
}

uint8_t PitchController_SetPid(PitchController *pitch, char which, float kp, float ki, float kd)
/*
 * 作用：校验并更新 Pitch 指定环 PID，按原规则处理控制历史。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 which：环选择符，P 为位置环、V 为速度环。
 * 参数 kp：比例增益。
 * 参数 ki：积分增益，计算时再乘 dt。
 * 参数 kd：测量微分增益。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    GimbalPidGains *pid;
    if (!pitch || !finite_value(kp) || !finite_value(ki) || !finite_value(kd) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f) { return 0U; }
    if (which == 'P') {
        if (kp > 100.0f || ki > 30.0f || kd > 10.0f) { return 0U; }
        pid = &pitch->cascade.position;
    } else if (which == 'V') {
        if (kp > 1.0f || ki > 10.0f || kd > 0.02f) { return 0U; }
        pid = &pitch->cascade.speed;
    } else { return 0U; }
    if (pid->kp == kp && pid->ki == ki && pid->kd == kd) { return 1U; }
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    reset_brake(pitch);
    GimbalPid_Stop(&pitch->cascade); ++pitch->cascade.revision;
    return 1U;
}

uint8_t PitchController_SetBrakeEnabled(PitchController *pitch, uint8_t enabled)
/*
 * 作用：切换 Pitch 附加制动；关闭立即撤去附加力矩，开启由后续动作触发。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 enabled：请求开关，非零表示开启请求。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (!pitch || enabled > 1U) { return 0U; }
    if (pitch->brake.enabled != enabled) { reset_brake(pitch); pitch->brake.enabled = enabled; }
    return 1U;
}

uint8_t PitchController_SetGravityOffset(PitchController *pitch, float offset_deg)
/*
 * 作用：校验并设置水平角偏置，更新重力标定和必要的支撑状态。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 offset_deg：软件零点相对物理水平的角度偏置，deg。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (!pitch || !finite_value(offset_deg) || pitch_controller_absolute(offset_deg) > 180.0f || pitch->active ||
        !pitch->have_reference || !pitch->have_feedback || pitch->last_status != 0U || pitch->faults) { return 0U; }
    pitch->gravity_offset_deg = offset_deg; pitch->gravity_calibrated = 1U;
    pitch->gravity_i_transferred = 0U; return 1U;
}

uint8_t PitchController_CaptureHorizontal(PitchController *pitch)
/*
 * 作用：以当前已确认软件坐标建立水平重力基准。
 * 参数 pitch：Pitch 控制器实例。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    if (!pitch || !stationary_safe(pitch) || pitch->stationary_samples < 2U ||
        pitch->last_feedback_ms - pitch->stationary_started_ms < PITCH_CONTROLLER_STATIONARY_HOLD_MS) { return 0U; }
    return PitchController_SetGravityOffset(pitch, -pitch->angle_deg);
}

uint8_t PitchController_SetGravityGain(PitchController *pitch, float kg_nm)
/*
 * 作用：校验并更新重力补偿系数，同时按原规则处理支撑力矩衔接。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 kg_nm：重力补偿系数，符号对应电机正方向，N*m。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    float delta, previous_i, next_i;
    if (!pitch || !finite_value(kg_nm) || pitch_controller_absolute(kg_nm) > PITCH_CONTROLLER_TORQUE_LIMIT_NM ||
        (kg_nm != 0.0f && (!pitch->have_reference || !pitch->gravity_calibrated || pitch->faults ||
         !finite_value(pitch->gravity_offset_deg) || !finite_value(pitch->angle_deg)))) { return 0U; }
    if (pitch->gravity_kg_nm == kg_nm) { return 1U; }
    pitch->gravity_i_transferred = 0U;
    if (pitch->active && pitch->cascade.speed.ki > 0.0f) {
        delta = gravity_at(pitch, pitch->gravity_kg_nm) - gravity_at(pitch, kg_nm);
        previous_i = pitch->cascade.speed_i;
        next_i = clamp(previous_i + delta, -PITCH_CONTROLLER_TORQUE_LIMIT_NM, PITCH_CONTROLLER_TORQUE_LIMIT_NM);
        pitch->cascade.speed_i = next_i;
        pitch->gravity_i_transferred = (uint8_t)(pitch_controller_absolute((next_i - previous_i) - delta) < 0.00001f);
    }
    pitch->gravity_kg_nm = kg_nm;
    return 1U;
}

static uint8_t finite_value(float x)
/*
 * 作用：检查浮点数是否有限，拒绝 NaN 和无穷值进入控制。
 * 参数 x：待处理的数值。
 * 返回布尔判断结果，非零为满足条件，0 为不满足。
 */
{ return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }

static float pitch_controller_absolute(float x)
/*
 * 作用：返回数值的绝对值，用于误差或速度幅值比较。
 * 参数 x：待处理的数值。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{ return x < 0.0f ? -x : x; }

static float clamp(float x, float lo, float hi)
/*
 * 作用：把数值限制在指定上下界内，避免控制请求越界。
 * 参数 x：待处理的数值。
 * 参数 lo：下界。
 * 参数 hi：上界。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{ return x < lo ? lo : (x > hi ? hi : x); }

void PitchController_InvalidateGravity(PitchController *pitch)
/*
 * 作用：使 Pitch 水平重力标定失效，后续使能仍需满足原标定条件。
 * 参数 pitch：Pitch 控制器实例。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (!pitch) { return; }
    /* 坐标失效只取消物理相位，不丢用户的力矩系数。 */
    pitch->gravity_offset_deg = pitch->gravity_torque_nm = 0.0f;
    pitch->pid_torque_nm = 0.0f; pitch->gravity_calibrated = pitch->gravity_i_transferred = 0U;
}

static void fault(PitchController *pitch, uint32_t reasons)
/*
 * 作用：锁存 Yaw 控制故障并沿原停止路径清理当前输出。
 * 参数 pitch：Pitch 控制器实例。
 * 参数 reasons：本次要锁存的控制器故障位集合。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ pitch->faults |= reasons; PitchController_Stop(pitch); }

static uint8_t stationary_safe(const PitchController *pitch)
/*
 * 作用：检查 Pitch 的反馈、状态、温度和静止条件是否允许建立参考。
 * 参数 pitch：Pitch 控制器实例。
 * 返回布尔判断结果，非零为满足条件，0 为不满足。
 */
{
    return (uint8_t)(pitch->have_feedback && pitch->last_status <= 1U &&
        pitch->temp_mos < PITCH_CONTROLLER_DRIVER_LIMIT_C &&
        pitch->temp_rotor < PITCH_CONTROLLER_MOTOR_LIMIT_C &&
        pitch_controller_absolute(pitch->velocity_dps) <= PITCH_CONTROLLER_STATIONARY_SPEED_DPS);
}
