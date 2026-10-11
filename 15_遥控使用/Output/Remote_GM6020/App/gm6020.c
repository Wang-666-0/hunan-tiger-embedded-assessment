/*
 * 文件功能：维护并提供 GM6020 的最新有效反馈。
 * 1. 调用 gm6020_codec.c 解码报文，成功后累计反馈帧数，保存接收时间。
 * 2. 在临界区中发布完整反馈结构体，供控制任务及串口遥测读取。
 * 3. 提供快照接口，让控制任务和串口任务读取同一帧的各项数据。
 * 模块关系：接收任务直接调用 GM6020_ParseFeedback；
 * motor_control.c 和 motor_console.c 通过 GM6020_GetFeedback 读取反馈。
 * 阅读重点：时间戳来自 CAN 中断实际接收时刻，排队不会使旧帧变新；
 * volatile 不能保证整个结构体读取一致，因此更新和复制都使用临界区。
 */
#include "gm6020.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

volatile GM6020_Feedback gm6020_feedback = {0}; // 最新反馈共享存储；初始帧数为零，表示尚未收到反馈。

uint8_t GM6020_ParseFeedback(const CanRxMessage *message)//解析反馈报文
{
    GM6020_Feedback decoded; // 先在局部变量中完成解码，不让其他任务看到半帧数据。
    if (GM6020_DecodeFeedback(message, &decoded) == 0U) // 错误帧不能覆盖上一条有效电机反馈。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }

    decoded.last_rx_ms = message->received_ms; // 使用中断搬运时刻，不能用任务处理时刻刷新旧帧。
    /* 解包和浮点换算在临界区外，只在临界区中发布结构体。
     * 此函数仅在任务中调用，不在 CAN 中断中调用。 */
    taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
    decoded.rx_count = gm6020_feedback.rx_count + 1U; // 仅成功解码并发布的反馈才累计一次。
    gm6020_feedback = decoded; // 在临界区一次发布这一帧的所有字段。
    taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    return 1U; // 通知调用方本次处理成功或输入已被接受。
}

void GM6020_GetFeedback(GM6020_Feedback *snapshot)//获取反馈快照
{
    if (snapshot != NULL) // 调用方必须提供存放快照的结构体。
    {
        taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
        *snapshot = gm6020_feedback;// 复制整个结构体，保证一致性
        taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    }
}
