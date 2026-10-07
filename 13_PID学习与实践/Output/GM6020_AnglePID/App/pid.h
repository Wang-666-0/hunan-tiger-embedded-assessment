/*
 * 文件功能：定义一个可以重复使用的位置式 PID 控制器。
 * 本模块只进行数学计算，不依赖电机、CAN、串口或 FreeRTOS。
 * 转速环调用时：target/measurement 的单位为 rpm，output 为电机控制指令原始值。
 * 这里使用连续形式增益：积分每次累加 Ki * error * dt，微分除以 dt。
 * 支持保留运行历史的在线增益更新，VOFA 滑块重复发送参数不会重置积分。
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float kp;                /* 比例增益，单位：输出单位 / 输入单位。 */
    float ki;                /* 积分增益，单位：输出单位 / (输入单位 * 秒)。 */
    float kd;                /* 微分增益，单位：输出单位 * 秒 / 输入单位。 */
    float output_limit;      /* 输出对称限幅，最终输出属于 [-limit, +limit]。 */
    float integral_limit;    /* 积分项对称限幅，限制的是积分产生的输出贡献。 */
    float derivative_tau_s;  /* 微分低通时间常数，单位秒；0 表示不滤波。 */
} PID_Config;

typedef struct
{
    PID_Config config;           /* 参数副本；每个控制器保存自己的参数。 */
    float integral;              /* 积分状态，已经乘入 Ki，单位与输出相同。 */
    float previous_measurement;  /* 上次实际值，用于计算实际值变化率。 */
    float filtered_derivative;   /* 滤波后的负实际值变化率，尚未乘入 Kd。 */
    float p_term;                /* 本次比例项，便于理解和查看计算过程。 */
    float i_term;                /* 本次积分项，与 integral 相同。 */
    float d_term;                /* 本次微分项，已经乘入 Kd。 */
    float output;                /* 本次经过输出限幅的计算结果。 */
    uint8_t initialized;         /* 已经记录第一帧实际值时为 1。 */
} PID_Controller;

/* 检查参数并复制到控制器，然后清除历史状态。
 * 增益和时间常数必须非负，两个限幅必须为正，所有参数必须是有限数。
 * 返回 1 表示成功；失败返回 0，保留控制器原有参数和状态。
 */
uint8_t PID_Init(PID_Controller *controller, const PID_Config *config);

/* 在线修改三个增益，保留限幅、时间常数、历史实际值和微分滤波状态。
 * 积分保存的是输出贡献，改变 Ki 不对已有积分重新缩放；Ki=0 时清零
 * integral 和 i_term，明确表示关闭积分。重复相同增益不改变任何状态。
 * 返回 1 表示接受；非法参数返回 0，控制器参数与运行状态全部不变。
 * p_term、d_term 和 output 在下一次 Update 时重新计算；改变 Kp/Kd
 * 仍可能使输出发生变化，保留历史并不保证任意调参都能无扰动。
 * 调用者应与 Update 使用同一任务，或自行同步访问。
 */
uint8_t PID_SetGains(PID_Controller *controller, float kp, float ki, float kd);

/* 清除积分、微分和历史实际值；保留参数。
 * 停止电机或切换控制方式时调用，避免再次启动继承旧积分。
 */
void PID_Reset(PID_Controller *controller);

/* 只清除积分输出贡献，保留实际值与微分滤波历史。
 * 目标反转或首次切到零速时使用，避免旧方向积分抵消制动力。
 * output 与 P/D 项在下次 Update 刷新；不能把旧 output 当成新目标的结果。
 * 和 Update 一样，应由控制器的唯一计算任务调用。
 */
void PID_ClearIntegral(PID_Controller *controller);

/* 计算一次 PID，dt_s 是两次计算之间的秒数，例如 2 ms 对应 0.002f。
 * 微分作用于实际值，目标阶跃不会直接引起微分冲击。
 * 如果输入、时间间隔或中间结果无效，清除历史状态并返回 0。
 * 本模块没有内部锁；同一个控制器应由一个任务计算，参数更新由调用者同步。
 */
float PID_Update(PID_Controller *controller, float target,
                 float measurement, float dt_s);

#endif /* PID_H */
