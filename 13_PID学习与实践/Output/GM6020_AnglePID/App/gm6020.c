/*
 * 文件功能：维护并提供 GM6020 的最新有效反馈。
 * 1. 调用 gm6020_codec.c 解码报文，累计反馈帧数和连续多圈编码位置。
 * 2. 在临界区中发布完整反馈结构体，供控制任务及串口遥测读取。
 * 3. 提供快照接口，让控制任务和串口任务读取同一帧的各项数据。
 * 模块关系：接收任务直接调用 GM6020_ParseFeedback；
 * motor_control.c 和 motor_console.c 通过 GM6020_GetFeedback 读取反馈。
 * 阅读重点：时间戳来自 CAN 中断实际接收时刻，排队不会使旧帧变新；
 * volatile 不能保证整个结构体读取一致，因此更新和复制都使用临界区。
 */
#include "gm6020.h"
#include "angle_tracker.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

volatile GM6020_Feedback gm6020_feedback = {0}; // 最新反馈共享存储；初始帧数为零，表示尚未收到反馈。
static AngleTracker angle_tracker;

/* 调用者持有任务临界区；tracker 和反馈必须发布同一代位置。 */
static void PublishAngle(GM6020_Feedback *feedback)
{
    feedback->accumulated_counts = angle_tracker.accumulated_counts;
    feedback->relative_angle_deg = AngleTracker_AngleDeg(&angle_tracker);
    feedback->angle_valid = angle_tracker.valid;
    feedback->angle_epoch = angle_tracker.epoch;
}

void GM6020_ResetFeedback(void)
{
    GM6020_Feedback empty = {0};
    taskENTER_CRITICAL();
    AngleTracker_Init(&angle_tracker);
    gm6020_feedback = empty;
    taskEXIT_CRITICAL();
}

uint8_t GM6020_ParseFeedback(const CanRxMessage *message)//解析反馈报文
{
    GM6020_Feedback decoded; // 先在局部变量中完成解码，不让其他任务看到半帧数据。
    if (GM6020_DecodeFeedback(message, &decoded) == 0U) // 错误帧不能覆盖上一条有效电机反馈。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }

    decoded.last_rx_ms = message->received_ms; // 使用中断搬运时刻，不能用任务处理时刻刷新旧帧。
    /* 协议解包在临界区外；短临界区内计算差分并发布结构体。
     * 差分与发布不能分开，否则 Z 与接收任务交错会混合新旧零点。
     * 此函数仅在任务中调用，不在 CAN 中断中调用。 */
    taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
    decoded.rx_count = gm6020_feedback.rx_count + 1U; // 仅成功解码并发布的反馈才累计一次。
    AngleTracker_Update(&angle_tracker, decoded.encoder, decoded.speed_rpm,
                        decoded.last_rx_ms);
    PublishAngle(&decoded);
    gm6020_feedback = decoded; // 在临界区一次发布这一帧的所有字段。
    taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    return 1U; // 通知调用方本次处理成功或输入已被接受。
}

void GM6020_GetFeedback(GM6020_Feedback *snapshot)//获取反馈快照
{
    if (snapshot != NULL) // 调用方必须提供存放快照的结构体。
    {
        taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
        /* 没有新帧时也锁存超时，防止断电后仍把旧多圈位置显示为有效。 */
        AngleTracker_CheckAge(&angle_tracker, HAL_GetTick());
        gm6020_feedback.angle_valid = angle_tracker.valid;
        *snapshot = gm6020_feedback;// 复制整个结构体，保证一致性
        taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    }
}

uint8_t GM6020_ZeroAngle(void)
{
    GM6020_Feedback feedback;
    uint32_t now_ms;
    float speed;
    uint8_t accepted = 0U;

    /* 在线检查、低速检查、rebase 和反馈发布都在同一短临界区中完成，
     * 接收任务不能在检查之后先发布另一条帧而使零点对应错误编码值。 */
    taskENTER_CRITICAL();
    feedback = gm6020_feedback;
    now_ms = HAL_GetTick();
    speed = (float)feedback.speed_rpm;
    if (speed < 0.0f)
    {
        speed = -speed;
    }
    if (GM6020_IsOnline(&feedback, now_ms) != 0U &&
        speed <= ANGLE_ZERO_SPEED_RPM)
    {
        accepted = AngleTracker_Zero(&angle_tracker, feedback.encoder, now_ms);
        if (accepted != 0U)
        {
            PublishAngle(&feedback);
            gm6020_feedback = feedback;
        }
    }
    taskEXIT_CRITICAL();
    return accepted;
}
