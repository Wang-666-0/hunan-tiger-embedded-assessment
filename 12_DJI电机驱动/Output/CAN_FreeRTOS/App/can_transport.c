/*
 * 文件功能：连接 STM32 CAN1 硬件与应用任务，完成报文收发。
 * 1. 创建接收队列，配置 CAN1 过滤器，启动 CAN1 并开启接收/发送通知。
 * 2. 接收中断读取 FIFO0，筛选标准数据帧，记录时间戳后复制到队列；
 *    接收任务再从队列取帧，避免在中断里解码和打印。
 * 3. 将应用提供的 ID 和 8 字节数据提交到发送邮箱，支持取消待发报文。
 * 4. 在 HAL 回调中记录发送完成、取消、接收丢帧和错误等诊断信息。
 * 模块关系：can_tasks.c 取接收帧，motor_control.c 提交/取消发送，
 * can_debug.c 提供相关计数器；GM6020 协议内容由上层模块处理。
 * 阅读重点：HAL_OK 表示报文已提交邮箱，发送完成要看完成回调；
 * 本工程只启动 CAN1，CAN2 不参与此前的两路互发测试。
 */
#include "can_transport.h"
#include "can_debug.h"
#include "app_config.h"
#include "queue.h"

/* FIFO 是硬件缓冲，queue 是 ISR 与任务之间的软件缓冲。 */
static QueueHandle_t can_rx_queue; // 持久 CAN 软件队列，连接接收中断与接收任务。

void CanTransport_CreateQueue(void)
{
    // 按完整报文结构体大小创建队列，入队时复制所有字段。
    can_rx_queue = xQueueCreate(CAN_RX_QUEUE_LENGTH, sizeof(CanRxMessage));
    if (can_rx_queue == NULL) // 分配失败时不能开启依赖此队列的接收流程。
    {
        CanFail(CAN_FAULT_QUEUE); // 队列失败记录故障码并进入统一错误处理。
    }
    vQueueAddToRegistry(can_rx_queue, "CAN_RX_QUEUE"); // 注册队列名称，不影响报文格式和队列容量。
}

/* message：调用方提供的报文输出缓冲；wait_ticks：最多等待的 RTOS tick。
 * 返回 pdPASS 表示取到完整副本；portMAX_DELAY 用于阻塞等待下一帧。 */
BaseType_t CanTransport_Receive(CanRxMessage *message, TickType_t wait_ticks)
{
    // 队列复制到调用方结构体；wait_ticks 决定最多阻塞多久。
    return xQueueReceive(can_rx_queue, message, wait_ticks);
}

void CanStart(void)
{
    CAN_FilterTypeDef filter = {0}; // 先清零过滤器配置，避免未赋值字段带入随机数据。
    const uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING | // 组合接收有帧、FIFO 溢出和发送完成通知。
        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_TX_MAILBOX_EMPTY; // 按位或生成通知掩码，供 HAL 启用对应中断。

    filter.SlaveStartFilterBank = 14U; // 共享过滤器组的分界：0～13 给 CAN1，14～27 给 CAN2。
    filter.FilterBank = 0U; // 本工程使用 CAN1 的第零组过滤器。
    filter.FilterMode = CAN_FILTERMODE_IDMASK; // 选择标识符与掩码匹配模式。
    filter.FilterScale = CAN_FILTERSCALE_32BIT; // 使用 32 位过滤器格式。
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 匹配报文放到 FIFO0，与 RX0 回调对应。
    filter.FilterActivation = ENABLE; // 使这组过滤器生效。
    /* 零掩码：方便用 Watch 看到实际 ID。软件再筛选标准数据帧和目标电机。
     * CAN2 保留 CubeMX 初始化，但不启动，不参与旧的互发测试。 */
    filter.FilterIdHigh = 0U; // 标识符高半部分清零。
    filter.FilterIdLow = 0U; // 标识符低半部分清零。
    filter.FilterMaskIdHigh = 0U; // 高半掩码为零，对应位不参与硬件筛选。
    filter.FilterMaskIdLow = 0U; // 全零掩码接受全部 ID，电机模块再软件筛选。
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) // 将过滤器配置写入 CAN 外设，失败不能继续启动。
    {
        CanFail(CAN_FAULT_FILTER1); // 记录 CAN1 过滤器配置失败。
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) // 使 CAN1 离开初始化状态并进入正常通信。
    {
        CanFail(CAN_FAULT_START1); // 记录 CAN1 启动失败。
    }
    if (HAL_CAN_ActivateNotification(&hcan1, notifications) != HAL_OK) // 开启所选通知，收到反馈才会进入回调搬运。
    {
        CanFail(CAN_FAULT_NOTIFY1); // 记录通知开启失败。
    }
    can_ready = 1U; // 只有前面的步骤全部成功，控制任务才可以开始访问 CAN。
}

/* id：11 位标准帧 ID；data：八字节输入；返回 HAL 的邮箱提交状态。
 * 此函数不等待总线发送完成，完成统计由后面的 HAL 回调更新。 */
HAL_StatusTypeDef CanTransport_Send(uint32_t id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0}; // 标准数据帧的头部配置，先完整清零。
    uint32_t mailbox; // HAL 输出实际选择的发送邮箱。
    HAL_StatusTypeDef status; // 保存提交结果并返回调用者。
    if (can_ready == 0U || data == NULL || id > 0x7FFU) // 要求 CAN 已启动、数据存在、ID 是 11 位标准 ID。
    {
        return HAL_ERROR; // 参数或启动状态不满足，不提交报文。
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) // 没有空闲邮箱时不在此忙等。
    {
        can1_tx_busy_count++; // 记录邮箱暂时不可用。
        return HAL_BUSY; // 交给控制任务撤销或处理，不阻塞接收路径。
    }
    header.StdId = id; // 设置应用层组帧选出的标准报文 ID。
    header.IDE = CAN_ID_STD; // 明确采用标准帧，而不是 29 位扩展帧。
    header.RTR = CAN_RTR_DATA; // 发送携带数据的帧，不是远程请求帧。
    header.DLC = 8U; // GM6020 控制报文固定为八字节。
    header.TransmitGlobalTime = DISABLE; // 不向数据字段写入时间触发通信时间信息。
    status = HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox); // 复制帧到硬件邮箱；HAL_OK 仅表示已提交。
    if (status == HAL_OK) // 提交计数与后面的硬件完成计数分别统计。
    {
        can1_tx_queued_count++; // 成功提交一帧，不代表已得到总线 ACK。
    }
    else
    {
        can1_tx_submit_error_count++; // 记录 HAL 提交或取消操作失败。
    }
    return status; // 控制任务根据实际 HAL 状态处理本轮请求。
}

void CanTransport_AbortPending(void)//取消所有待发报文，避免旧帧继续自动重发。
{
    if (can_ready != 0U && // 只在 CAN 已启动时检查并取消硬件邮箱。
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U) // 三个邮箱未全空表示仍存在挂起发送。
    {
        if (HAL_CAN_AbortTxRequest(&hcan1, // 请求取消旧邮箱，不让旧给定继续自动重发。
                // 覆盖三个发送邮箱，取消失败计入错误统计。
                CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) != HAL_OK)
        {
            can1_tx_submit_error_count++; // 记录 HAL 提交或取消操作失败。
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//收到报文后进入
{
    BaseType_t task_woken = pdFALSE; // 记录入队是否唤醒更高优先级任务，稍后通知调度器。
    if (hcan != &hcan1) // 其他 CAN 的回调不属于本工程接收路径。
    {
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    /* ISR 只搬运，不解包、不打印、不等待发送。 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) // 本次回调尽量搬空 FIFO0，减少硬件积压。
    {
        CAN_RxHeaderTypeDef header; // HAL 返回接收帧的 ID、类型和 DLC。
        CanRxMessage message = {0}; // 局部软件消息清零，短帧未使用字节不会留下旧内容。
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, message.data) != HAL_OK)//读取FIFO
        {
            can_rx_read_error_count++; // 记录硬件取帧失败或帧长度异常。
            break; // 结束本轮循环，避免无数据或读取失败时继续处理。
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA) // 只搬运标准数据帧，过滤扩展帧和远程帧。
        {
            can_rx_ignored_count++; // 记录因帧类型不符而被忽略的帧数。
            continue; // 忽略当前帧，继续检查下一帧。
        }
        if (header.DLC > 8U) // 经典 CAN 最多八字节，异常长度不能交给应用解析。
        {
            can_rx_read_error_count++; // 记录硬件取帧失败或帧长度异常。
            continue; // 忽略当前帧，继续检查下一帧。
        }

        //随后将数据搬运到队列中，供任务处理
        message.source = 1U; // 标记接收控制器是 CAN1，不是电机 ID 1。
        message.id = header.StdId; // 保留实际标准报文 ID，供电机模块匹配。
        message.dlc = (uint8_t)header.DLC; // 保存有效数据长度。
        message.received_ms = HAL_GetTick(); // 记录中断搬运时刻，用于反馈新鲜度判断。
        for (uint32_t i = message.dlc; i < 8U; i++) // 短帧剩余部分逐字节补零，完整反馈仍要求 DLC=8。
        {
            message.data[i] = 0U; // 补零不改变真实 DLC，不能把短帧伪装成有效反馈。
        }
        /* FromISR 复制结构体，局部变量退出中断后不会影响队列内容。
         * 中断优先级为 5，符合本工程 FreeRTOS 的系统调用限制。 */
        if (xQueueSendFromISR(can_rx_queue, &message, &task_woken) != pdPASS)//发送到队列
        {
            can_rx_queue_full_count++; // 队列满时丢帧并计数，中断不能等待空间。
        }
    }
    portYIELD_FROM_ISR(task_woken); // 若唤醒了更高优先级任务，中断退出时请求切换。
}

static void CanTxCompleteCount(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) // 只累计本工程 CAN1 的发送或错误状态。
    {
        can1_tx_count++; // 完成回调代表硬件发送成功，区别于提交邮箱。
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CanTxCompleteCount(hcan); // 三个邮箱共用同一个发送完成统计入口。
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CanTxCompleteCount(hcan); // 三个邮箱共用同一个发送完成统计入口。
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CanTxCompleteCount(hcan); // 三个邮箱共用同一个发送完成统计入口。
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) /* 只统计本工程 CAN1 的取消通知。 */
    {
        can1_tx_abort_count++; /* 邮箱 0 已完成取消，旧报文不再挂起。 */
    }
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) /* 只统计本工程 CAN1 的取消通知。 */
    {
        can1_tx_abort_count++; /* 邮箱 1 的取消结果使用同一计数器。 */
    }
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) /* 只统计本工程 CAN1 的取消通知。 */
    {
        can1_tx_abort_count++; /* 邮箱 2 的取消结果使用同一计数器。 */
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) // 只累计本工程 CAN1 的发送或错误状态。
    {
        can1_error_callback_count++; // 记录 CAN 错误回调的累计次数。
        can1_last_hal_error = HAL_CAN_GetError(hcan); // 保存最近 HAL CAN 错误信息。
    }
}
