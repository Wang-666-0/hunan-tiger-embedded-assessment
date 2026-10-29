/**
 * @file gimbal_pid.c
 * 作用：Yaw / Pitch 共用的位置—速度串级 PID、速度观测和抗积分饱和。
 * 对外函数：
 *   GimbalPid_UpdateWithFeedforward / GimbalPid_Update / GimbalPid_Observe / GimbalPid_SetPid /
 *   GimbalPid_Init / GimbalPid_Stop / GimbalPid_ResetObserver
 * 内部函数：
 *   winds_up / gimbal_pid_clamp / gimbal_pid_finite_value
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "gimbal_pid.h"
#include "app_config.h"
#include <float.h>
#include <string.h>

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static float gimbal_pid_clamp(float x, float lo, float hi);
static uint8_t gimbal_pid_finite_value(float x);
static uint8_t winds_up(float wanted, float change, float limit);

float GimbalPid_UpdateWithFeedforward(GimbalPid *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm, float feedforward_nm)
/*
 * 作用：执行位置—速度串级 PID，并把附加力矩一并纳入输出限幅与抗积分饱和。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 参数 error_deg：目标减实测的角度误差，deg。
 * 参数 feedback_ms：当前反馈的接收时间戳，ms。
 * 参数 sequence：本次数据或事件序号，用于区分新旧输入。
 * 参数 torque_limit_nm：总力矩的对称限幅，N*m。
 * 参数 feedforward_nm：叠加在 PID 上的附加力矩，N*m。
 * 流程：位置环：角度误差 → 目标角速度，单位 deg/s；速度环：速度误差 → 协议力矩，单位 N*m。
 * 流程：积分仅按新 CAN 反馈的实际 dt 推进；重复读取同一反馈时 dt 为零。
 * 流程：位置 D 使用实测速度，速度 D 使用滤波加速度，避免目标阶跃造成微分冲击。
 * 流程：积分会继续加重饱和时拒绝本次增量，最后对总力矩限幅。
 * 返回本次计算的协议力矩，N*m；此函数本身不提交 CAN。
 */
{
    float dt = 0.0f, wanted, candidate, change, speed_error;
    uint32_t gap = feedback_ms - loop->feedback_ms;
    if (loop->have_update && sequence != loop->sequence && gap < APP_CASCADE_OBSERVER_MAX_GAP_MS) {
        dt = (float)gap * 0.001f; // 仅新反馈推进积分，毫秒换算为秒。
    }
    loop->feedback_ms = feedback_ms; loop->sequence = sequence; loop->have_update = 1U;
    if (loop->position.ki == 0.0f) { loop->position_i = 0.0f; }
    change = loop->position.ki * error_deg * dt;
    /* 不截断外环速度参考或位置积分；仅在内环协议不可表达时防积累。 */
    candidate = loop->position_i + change;
    if (!winds_up(loop->unsaturated_torque_nm, change, torque_limit_nm)) { loop->position_i = candidate; }
    loop->target_velocity_dps = loop->position.kp * error_deg + loop->position_i
        - loop->position.kd * loop->velocity_dps; // 外环输出 deg/s；D 对测量微分。
    loop->velocity_limited = 0U;

    loop->feedforward_limited = 0U;

    loop->feedforward_nm = feedforward_nm;
    speed_error = loop->target_velocity_dps - loop->velocity_dps;
    if (loop->speed.ki == 0.0f) { loop->speed_i = 0.0f; }
    change = loop->speed.ki * speed_error * dt;
    candidate = gimbal_pid_clamp(loop->speed_i + change, -torque_limit_nm, torque_limit_nm);
    wanted = loop->speed.kp * speed_error + candidate - loop->speed.kd * loop->acceleration_dps2 + feedforward_nm;
    if (!winds_up(wanted, change, torque_limit_nm)) { loop->speed_i = candidate; }
    loop->unsaturated_torque_nm = loop->speed.kp * speed_error + loop->speed_i
        - loop->speed.kd * loop->acceleration_dps2 + feedforward_nm; // 内环与补偿统一在力矩层相加。
    loop->torque_nm = gimbal_pid_clamp(loop->unsaturated_torque_nm, -torque_limit_nm, torque_limit_nm);
    loop->torque_limited = (uint8_t)(loop->unsaturated_torque_nm > torque_limit_nm || loop->unsaturated_torque_nm < -torque_limit_nm);
    return loop->torque_nm;
}

float GimbalPid_Update(GimbalPid *loop, float error_deg, uint32_t feedback_ms,
                        uint32_t sequence, float torque_limit_nm)
/*
 * 作用：以直接位置误差执行双环控制，不额外添加前馈力矩。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 参数 error_deg：目标减实测的角度误差，deg。
 * 参数 feedback_ms：当前反馈的接收时间戳，ms。
 * 参数 sequence：本次数据或事件序号，用于区分新旧输入。
 * 参数 torque_limit_nm：总力矩的对称限幅，N*m。
 * 返回本次计算的协议力矩，N*m；此函数本身不提交 CAN。
 */
{
    return GimbalPid_UpdateWithFeedforward(loop, error_deg, feedback_ms, sequence, torque_limit_nm, 0.0f);
}

void GimbalPid_Observe(GimbalPid *loop, float delta_deg, uint32_t gap_ms)
/*
 * 作用：由解环后的位置差分估计速度，再由速度变化估计加速度。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 参数 delta_deg：相邻有效位置增量，deg。
 * 参数 gap_ms：相邻有效反馈的间隔，ms。
 * 流程：只在新反馈到达时调用，dt 来自实际接收间隔。
 * 流程：一阶低通的系数为 dt/(T+dt)，兼顾量化噪声和观测滞后。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    float previous, dt, alpha;
    if (!gap_ms || gap_ms >= APP_CASCADE_OBSERVER_MAX_GAP_MS || !gimbal_pid_finite_value(delta_deg)) { GimbalPid_ResetObserver(loop); return; }
    dt = (float)gap_ms * 0.001f;
    alpha = (float)gap_ms / ((float)YAW_CASCADE_VELOCITY_FILTER_MS + (float)gap_ms);
    previous = loop->velocity_dps;
    loop->velocity_dps += alpha * (delta_deg / dt - loop->velocity_dps);
    alpha = (float)gap_ms / ((float)20U + (float)gap_ms);
    loop->acceleration_dps2 += alpha * ((loop->velocity_dps - previous) / dt - loop->acceleration_dps2);
}

uint8_t GimbalPid_SetPid(GimbalPid *loop, char which, float kp, float ki, float kd)
/*
 * 作用：校验并更新指定环的 PID；参数改变时重置控制历史并递增版本。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 参数 which：环选择符，P 为位置环、V 为速度环。
 * 参数 kp：比例增益。
 * 参数 ki：积分增益，计算时再乘 dt。
 * 参数 kd：测量微分增益。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    GimbalPidGains *pid;
    if (!gimbal_pid_finite_value(kp) || !gimbal_pid_finite_value(ki) || !gimbal_pid_finite_value(kd) || kp < 0.0f || ki < 0.0f || kd < 0.0f) { return 0U; }
    if (which == 'P') {
        if (kp > 30.0f || ki > 10.0f || kd > 2.0f) { return 0U; }
        pid = &loop->position;
    } else if (which == 'V') {
        if (kp > 0.02f || ki > 0.2f || kd > 0.001f) { return 0U; }
        pid = &loop->speed;
    } else { return 0U; }
    if (pid->kp == kp && pid->ki == ki && pid->kd == kd) { return 1U; }
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    GimbalPid_Stop(loop); ++loop->revision;
    return 1U;
}

void GimbalPid_Init(GimbalPid *loop)
/*
 * 作用：载入默认双环参数并初始化独立的控制与观测状态。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    memset(loop, 0, sizeof(*loop));
    loop->position.kp = APP_YAW_POSITION_KP;
    loop->position.ki = APP_YAW_POSITION_KI;
    loop->position.kd = APP_YAW_POSITION_KD;
    loop->speed.kp = APP_YAW_SPEED_KP;
    loop->speed.ki = APP_YAW_SPEED_KI;
    loop->speed.kd = APP_YAW_SPEED_KD;
    loop->velocity_limit_dps = 0.0f;
}

void GimbalPid_Stop(GimbalPid *loop)
/*
 * 作用：清除双环积分、目标速度和控制输出，保留参数与已有速度观测。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 流程：重复 OFF 不会把实测速度估计反复清零；需要清观测时使用 ResetObserver。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    loop->position_i = loop->speed_i = 0.0f;
    loop->torque_nm = loop->unsaturated_torque_nm = loop->target_velocity_dps = 0.0f;
    loop->feedforward_nm = 0.0f;
    loop->have_update = loop->torque_limited = loop->velocity_limited = loop->feedforward_limited = 0U;
}

void GimbalPid_ResetObserver(GimbalPid *loop)
/*
 * 作用：清除速度、加速度观测及对应控制历史。
 * 参数 loop：该轴独立的 PID 参数、积分与观测状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ loop->velocity_dps = loop->acceleration_dps2 = 0.0f; GimbalPid_Stop(loop); }

static uint8_t winds_up(float wanted, float change, float limit)
/*
 * 作用：判断本次积分增量是否会继续加重已饱和的输出。
 * 参数 wanted：尚未限幅的计算输出。
 * 参数 change：拟增加的积分输出，符号用于判断是否加重饱和。
 * 参数 limit：该判断/换算所使用的上限。
 * 返回布尔判断结果，非零为满足条件，0 为不满足。
 */
{ return (uint8_t)((wanted > limit && change > 0.0f) || (wanted < -limit && change < 0.0f)); }

static float gimbal_pid_clamp(float x, float lo, float hi)
/*
 * 作用：把数值限制在指定上下界内，避免控制请求越界。
 * 参数 x：待处理的数值。
 * 参数 lo：下界。
 * 参数 hi：上界。
 * 返回计算后的浮点数；具体物理单位与输入量程一致。
 */
{ return x < lo ? lo : (x > hi ? hi : x); }

static uint8_t gimbal_pid_finite_value(float x)
/*
 * 作用：检查浮点数是否有限，拒绝 NaN 和无穷值进入控制。
 * 参数 x：待处理的数值。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{ return (uint8_t)(x == x && x >= -FLT_MAX && x <= FLT_MAX); }
