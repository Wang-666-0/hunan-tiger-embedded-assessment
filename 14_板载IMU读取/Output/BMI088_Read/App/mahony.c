#include "mahony.h"  // 声明本模块对外使用的状态结构和三个 API。

#include <float.h>   // FLT_MAX 用于兼容 ARM Compiler 5 的有限浮点数检查。
#include <math.h>    // 使用标准数学函数，不使用类型重解释的快速平方根技巧。
#include <stddef.h>  // 使用 NULL 检查调用者传入的指针。

#define MAHONY_RAD_TO_DEG       57.29577951308232f // 将弧度换算成 VOFA 控件需要的度。
#define MAHONY_DEFAULT_KP       2.0f              // 完整重力叉积的比例增益，不是 twoKp。
#define MAHONY_DEFAULT_KI       0.0f              // 默认不积分，减少教学初期的积分饱和问题。
#define MAHONY_INTEGRAL_LIMIT   0.5f              // 允许的积分反馈绝对上限，单位 rad/s。
#define MAHONY_MAX_DT_S         0.1f              // 拒绝异常长周期，避免恢复任务时突然转动。

/* 自编实现说明：按 Mahony 的重力叉积反馈和四元数运动学结构组织，
 * 增加实际 dt、实例化状态、静态重力初始化、失败不提交和有限数值保护。
 * 原始算法说明及公开参考实现链接见 mahony.h；本模块不含磁力计分支。
 */

static bool FloatIsFinite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX); // NaN 不等于自身，正负无穷超出 FLT_MAX。
}

static float ClampFloat(float value, float lower, float upper)
{
    if (value < lower) // 当值低于安全范围时使用下界。
    {
        return lower;
    }
    if (value > upper) // 当值超过安全范围时使用上界。
    {
        return upper;
    }
    return value;     // 正常值无需修改。
}

static bool NormalizeVector(const float *input, float *output, unsigned int count)
{
    float largest = 0.0f; // 先用最大绝对值缩放，避免大浮点输入在平方时溢出。
    float norm_sq = 0.0f;
    float inverse_norm;
    unsigned int i;

    for (i = 0U; i < count; ++i)
    {
        if (!FloatIsFinite(input[i])) // 任一 NaN / Inf 都不能进入姿态运算。
        {
            return false;
        }
        if (fabsf(input[i]) > largest) // 找到当前向量的最大分量。
        {
            largest = fabsf(input[i]);
        }
    }
    if (!(largest > 1.0e-12f)) // 零向量或过小向量无法给出可靠方向。
    {
        return false;
    }
    for (i = 0U; i < count; ++i)
    {
        output[i] = input[i] / largest; // 把分量缩放到 [-1, 1] 内。
        norm_sq += output[i] * output[i];
    }
    inverse_norm = 1.0f / sqrtf(norm_sq); // 缩放后的模长可安全计算。
    for (i = 0U; i < count; ++i)
    {
        output[i] *= inverse_norm; // 输出单位长度向量或单位四元数。
    }
    return true;
}

bool Mahony_Init(mahony_t *state, const float accel[3])
{
    mahony_t candidate; // 只在所有检查通过后提交，初始化失败不会破坏已有姿态。
    float measured[3];
    float half_roll, half_pitch;
    float sr, cr, sp, cp;
    float initial_q[4];

    if ((state == NULL) || (accel == NULL)) // 在读取数据之前检查空指针。
    {
        return false;
    }
    if (!NormalizeVector(accel, measured, 3U)) // 静止加速度必须能够归一化。
    {
        return false;
    }

    half_roll = 0.5f * atan2f(measured[1], measured[2]); // 用 Y、Z 分量确定横滚，倒放时也能初始化。
    half_pitch = 0.5f * atan2f(-measured[0], sqrtf(measured[1] * measured[1] + measured[2] * measured[2])); // 用 X 分量确定俯仰。
    sr = sinf(half_roll);  // 欧拉角转四元数使用半角的正弦和余弦。
    cr = cosf(half_roll);
    sp = sinf(half_pitch);
    cp = cosf(half_pitch);
    initial_q[0] = cr * cp; // 初始 Yaw 固定为 0：只能建立相对航向，不能得出绝对北向。
    initial_q[1] = sr * cp;
    initial_q[2] = cr * sp;
    initial_q[3] = -sr * sp;
    if (!NormalizeVector(initial_q, candidate.q, 4U)) // 消除三角函数舍入造成的四元数模长误差。
    {
        return false;
    }
    candidate.kp = MAHONY_DEFAULT_KP; // 设置可由调用者调整的实例比例增益。
    candidate.ki = MAHONY_DEFAULT_KI; // 默认关闭积分反馈。
    candidate.integral[0] = 0.0f;    // 初始化时清零一次，不要每次更新都重新初始化状态。
    candidate.integral[1] = 0.0f;
    candidate.integral[2] = 0.0f;
    candidate.initialized = true;   // 标记为可持续更新的有效姿态。
    *state = candidate;             // 一次性提交完成的状态。
    return true;
}

bool Mahony_UpdateIMU(mahony_t *state, const float gyro[3], const float accel[3], float dt_s)
{
    mahony_t next; // 后续计算写入副本；任何失败都保留原始 state。
    float measured[3], predicted[3], error[3], rate[3];
    float q_now[4], q_new[4];
    float half_dt;
    unsigned int i;

    if ((state == NULL) || (gyro == NULL) || (accel == NULL)) // 不允许读取空指针。
    {
        return false;
    }
    if (!state->initialized || !FloatIsFinite(dt_s) || !(dt_s > 0.0f) || (dt_s > MAHONY_MAX_DT_S)) // 本次时间必须可信。
    {
        return false;
    }
    if (!FloatIsFinite(state->kp) || !FloatIsFinite(state->ki) || (state->kp < 0.0f) || (state->ki < 0.0f)) // 反馈增益必须有效。
    {
        return false;
    }
    if (!NormalizeVector(state->q, q_now, 4U) || !NormalizeVector(accel, measured, 3U)) // 归一化旧姿态和本次重力方向。
    {
        return false;
    }
    for (i = 0U; i < 3U; ++i)
    {
        if (!FloatIsFinite(gyro[i]) || !FloatIsFinite(state->integral[i])) // 输入角速度与历史积分都不能含非有限值。
        {
            return false;
        }
    }
    next = *state; // 检查完原始输入后再建立可提交的副本。

    predicted[0] = 2.0f * (q_now[1] * q_now[3] - q_now[0] * q_now[2]); // 将世界 +Z 方向转换到机体坐标。
    predicted[1] = 2.0f * (q_now[0] * q_now[1] + q_now[2] * q_now[3]);
    predicted[2] = 1.0f - 2.0f * (q_now[1] * q_now[1] + q_now[2] * q_now[2]);
    error[0] = measured[1] * predicted[2] - measured[2] * predicted[1]; // 测量方向叉乘估计方向，形成 X 轴纠偏量。
    error[1] = measured[2] * predicted[0] - measured[0] * predicted[2]; // 同样计算 Y 轴纠偏量。
    error[2] = measured[0] * predicted[1] - measured[1] * predicted[0]; // 重力反馈不能提供绝对航向参考。
    for (i = 0U; i < 3U; ++i)
    {
        if (next.ki > 0.0f) // 只有主动设置 Ki 后才积累误差。
        {
            float integral_step = next.ki * error[i] * dt_s; // 按实际秒数积累，避免固定频率假设。
            if (!FloatIsFinite(integral_step)) // 极端增益导致溢出时拒绝更新。
            {
                return false;
            }
            next.integral[i] = ClampFloat(next.integral[i] + integral_step, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT); // 限制积分反馈。
        }
        else
        {
            next.integral[i] = 0.0f; // Ki 关闭后清除先前积分，防止残余偏置继续生效。
        }
        rate[i] = gyro[i] + next.kp * error[i] + next.integral[i]; // 原始角速度加上重力方向反馈。
        if (!FloatIsFinite(rate[i])) // 溢出后的角速度不能用于积分。
        {
            return false;
        }
    }

    half_dt = 0.5f * dt_s; // 四元数导数系数为 1/2；dt_s 由任务的实际采样间隔提供。
    q_new[0] = q_now[0] - half_dt * (q_now[1] * rate[0] + q_now[2] * rate[1] + q_now[3] * rate[2]); // 积分四元数实部。
    q_new[1] = q_now[1] + half_dt * (q_now[0] * rate[0] + q_now[2] * rate[2] - q_now[3] * rate[1]); // 积分四元数 X 分量。
    q_new[2] = q_now[2] + half_dt * (q_now[0] * rate[1] - q_now[1] * rate[2] + q_now[3] * rate[0]); // 积分四元数 Y 分量。
    q_new[3] = q_now[3] + half_dt * (q_now[0] * rate[2] + q_now[1] * rate[1] - q_now[2] * rate[0]); // 积分四元数 Z 分量。
    if (!NormalizeVector(q_new, next.q, 4U)) // 每周期恢复单位模长，防止数值误差持续累积。
    {
        return false;
    }
    *state = next; // 所有计算有效之后，才提交新的姿态和积分状态。
    return true;
}

bool Mahony_GetEulerDegrees(const mahony_t *state, float euler_deg[3])
{
    float q[4];
    float angles[3];
    float pitch_sin;
    unsigned int i;

    if ((state == NULL) || (euler_deg == NULL) || !state->initialized) // 不读取尚未初始化的姿态。
    {
        return false;
    }
    if (!NormalizeVector(state->q, q, 4U)) // 读取时仍进行校验，拒绝外部误改的非法四元数。
    {
        return false;
    }
    pitch_sin = ClampFloat(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f); // 限幅，避免 asinf 因舍入越界生成 NaN。
    angles[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * MAHONY_RAD_TO_DEG; // Roll：绕 X 轴。
    angles[1] = asinf(pitch_sin) * MAHONY_RAD_TO_DEG; // Pitch：绕 Y 轴，ZYX 定义下范围为 [-90, 90] 度。
    angles[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * MAHONY_RAD_TO_DEG; // Yaw：绕 Z 轴。
    for (i = 0U; i < 3U; ++i)
    {
        if (!FloatIsFinite(angles[i])) // 失败时不改写调用者的输出数组。
        {
            return false;
        }
    }
    for (i = 0U; i < 3U; ++i)
    {
        euler_deg[i] = angles[i]; // 一次验证后输出供 JustFloat 发送的三个角度。
    }
    return true;
}
