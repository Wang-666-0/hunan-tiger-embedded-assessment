/*
 * 文件功能：实现位置式 PID、输出/积分限幅、条件积分抗饱和及微分低通。
 * 每次输出 u = Kp * error + integral + Kd * filtered_derivative。
 * integral 按 Ki * error * dt 累加；derivative 采用 -实际值变化率，避免目标阶跃冲击。
 * 在线增益更新保留积分输出贡献与测量历史，避免滑块每次发送都清空控制状态。
 * 算法不访问硬件；NaN、无穷和计算溢出会清除历史状态并返回零输出。
 */
#include "pid.h"

#include <float.h>
#include <stddef.h>

/* NaN 与任何数的有序比较都为假，无穷超出 FLT_MAX。
 * 这样可以检查有限数，并且不依赖不同编译器的 isfinite 宏。
 */
static uint8_t PID_IsFinite(float value)
{
    return (uint8_t)((value >= -FLT_MAX) && (value <= FLT_MAX));
}

/* 所有参数先检查有限性，再检查符号，避免 NaN 漏过范围比较。 */
static uint8_t PID_ConfigIsValid(const PID_Config *config)
{
    if (config == NULL)
    {
        return 0U;
    }

    if ((!PID_IsFinite(config->kp)) || (!PID_IsFinite(config->ki)) ||
        (!PID_IsFinite(config->kd)) || (!PID_IsFinite(config->output_limit)) ||
        (!PID_IsFinite(config->integral_limit)) ||
        (!PID_IsFinite(config->derivative_tau_s)))
    {
        return 0U;
    }

    return (uint8_t)((config->kp >= 0.0f) && (config->ki >= 0.0f) &&
                     (config->kd >= 0.0f) && (config->output_limit > 0.0f) &&
                     (config->integral_limit > 0.0f) &&
                     (config->derivative_tau_s >= 0.0f));
}

/* 对称限幅，适用于正转和反转使用同一输出范围的控制器。 */
static float PID_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PID_ClearIntegral(PID_Controller *controller)
{
    if (controller == NULL)
    {
        return;
    }
    controller->integral = 0.0f;
    controller->i_term = 0.0f;
}

void PID_Reset(PID_Controller *controller)
{
    if (controller == NULL)
    {
        return;
    }

    /* 参数不变，历史状态全部清零。 */
    PID_ClearIntegral(controller);
    controller->previous_measurement = 0.0f;
    controller->filtered_derivative = 0.0f;
    controller->p_term = 0.0f;
    controller->d_term = 0.0f;
    controller->output = 0.0f;
    controller->initialized = 0U;
}

uint8_t PID_Init(PID_Controller *controller, const PID_Config *config)
{
    if ((controller == NULL) || (!PID_ConfigIsValid(config)))
    {
        /* 检查通过之前不写控制器，非法调参不会破坏已有参数。 */
        return 0U;
    }

    controller->config = *config;
    PID_Reset(controller);
    return 1U;
}

uint8_t PID_SetGains(PID_Controller *controller, float kp, float ki, float kd)
{
    PID_Config candidate;

    if (controller == NULL)
    {
        return 0U;
    }

    /* 先在副本中替换增益，复用完整配置检查；失败之前不写任何旧状态。 */
    candidate = controller->config;
    candidate.kp = kp;
    candidate.ki = ki;
    candidate.kd = kd;
    if (!PID_ConfigIsValid(&candidate))
    {
        return 0U;
    }

    /* 滑块会反复发送同一个值，重复参数直接成功返回，不触碰运行历史。 */
    if ((kp == controller->config.kp) && (ki == controller->config.ki) &&
        (kd == controller->config.kd))
    {
        return 1U;
    }

    controller->config = candidate;
    if (ki == 0.0f)
    {
        /* I0 明确关闭积分，不能只停止累加却保留此前的积分输出。 */
        PID_ClearIntegral(controller);
    }
    /* Ki 非零时沿用旧积分输出；后续仅以新 Ki 改变每轮新增积分。
     * 不调用 Reset，保留 previous_measurement、filtered_derivative 和
     * initialized。比例、微分和总输出在下一轮 Update 使用新增益计算。
     */
    return 1U;
}

/* 低通系数 alpha = tau / (tau + dt)。
 * 分别除以较大的数，避免两个有限的大数相加得到无穷。
 */
static float PID_DerivativeAlpha(float tau_s, float dt_s)
{
    float ratio;

    if (tau_s == 0.0f)
    {
        return 0.0f;
    }
    if (tau_s > dt_s)
    {
        return 1.0f / (1.0f + dt_s / tau_s);
    }

    ratio = tau_s / dt_s;
    return ratio / (1.0f + ratio);
}

float PID_Update(PID_Controller *controller, float target,
                 float measurement, float dt_s)
{
    float error;
    float p_term;
    float derivative = 0.0f;
    float d_term;
    float integral_delta;
    float integral_candidate;
    float output_candidate;
    float output;

    if (controller == NULL)
    {
        return 0.0f;
    }
    if ((!PID_ConfigIsValid(&controller->config)) ||
        (!PID_IsFinite(target)) || (!PID_IsFinite(measurement)) ||
        (!PID_IsFinite(dt_s)) || (dt_s <= 0.0f) ||
        (!PID_IsFinite(controller->integral)))
    {
        PID_Reset(controller);
        return 0.0f;
    }

    error = target - measurement;      /* 误差为目标减实际；正误差产生正向输出。 */
    p_term = controller->config.kp * error;
    if ((!PID_IsFinite(error)) || (!PID_IsFinite(p_term)))
    {
        PID_Reset(controller);
        return 0.0f;
    }

    /* 第一帧还没有历史实际值，因此微分从零开始。Kd 为零时不计算差分。 */
    if ((controller->initialized != 0U) && (controller->config.kd > 0.0f))
    {
        float measurement_delta = measurement - controller->previous_measurement;
        float raw_derivative = -measurement_delta / dt_s;
        float alpha = PID_DerivativeAlpha(controller->config.derivative_tau_s, dt_s);

        if ((!PID_IsFinite(measurement_delta)) || (!PID_IsFinite(raw_derivative)) ||
            (!PID_IsFinite(controller->filtered_derivative)))
        {
            PID_Reset(controller);
            return 0.0f;
        }

        /* 实际值上升时产生负微分项；alpha 越大，对旧结果保留得越多。 */
        derivative = alpha * controller->filtered_derivative +
                     (1.0f - alpha) * raw_derivative;
    }

    d_term = controller->config.kd * derivative;
    integral_delta = controller->config.ki * error * dt_s;
    integral_candidate = controller->integral + integral_delta;
    if ((!PID_IsFinite(derivative)) || (!PID_IsFinite(d_term)) ||
        (!PID_IsFinite(integral_delta)) || (!PID_IsFinite(integral_candidate)))
    {
        PID_Reset(controller);
        return 0.0f;
    }

    /* 先限制积分项自身，再判断新增积分是否加重最终输出的饱和。 */
    integral_candidate = PID_Clamp(integral_candidate, controller->config.integral_limit);
    output_candidate = p_term + integral_candidate + d_term;
    if (!PID_IsFinite(output_candidate))
    {
        PID_Reset(controller);
        return 0.0f;
    }

    if (((output_candidate > controller->config.output_limit) && (error > 0.0f)) ||
        ((output_candidate < -controller->config.output_limit) && (error < 0.0f)))
    {
        /* 输出已经过大且误差还想推向同一方向：拒绝本次新增积分。
         * 反向误差可以减小旧积分，所以允许积分退出饱和。
         */
        integral_candidate = controller->integral;
    }

    output = p_term + integral_candidate + d_term;
    if (!PID_IsFinite(output))
    {
        PID_Reset(controller);
        return 0.0f;
    }

    /* 计算完整成功后才发布本次状态，保留各项有助于复盘 PID 的作用。 */
    controller->integral = integral_candidate;
    controller->previous_measurement = measurement;
    controller->filtered_derivative = derivative;
    controller->p_term = p_term;
    controller->i_term = integral_candidate;
    controller->d_term = d_term;
    controller->output = PID_Clamp(output, controller->config.output_limit);
    controller->initialized = 1U;
    return controller->output;
}
