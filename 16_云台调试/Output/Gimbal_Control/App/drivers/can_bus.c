/**
 * @file can_bus.c
 * 作用：CAN 接收与各电机专用发送权限
 * 对外函数：
 *   HAL_CAN_RxFifo0MsgPendingCallback / CanBus_Start / CanBus_YawTorque / CanBus_PitchTorque /
 *   CanBus_FeederCurrent / CanBus_JogVescCurrent / CanBus_YawEnable / CanBus_PitchEnable /
 *   CanBus_Receive / CanBus_CreateQueue / CanBus_GetState / CanBus_GetDiagnostics /
 *   CanBus_SetOutputPermission / CanBus_OutputAllowed / CanBus_SetYawPermission / CanBus_YawAllowed /
 *   CanBus_YawDisable / CanBus_YawTxPending / CanBus_YawClearComms / CanBus_SetPitchPermission /
 *   CanBus_PitchAllowed / CanBus_PitchDisable / CanBus_PitchReadFeedback / CanBus_PitchTxPending /
 *   CanBus_CancelPitchTx / CanBus_SetFeederPermission / CanBus_FeederAllowed / CanBus_StopFeeder /
 *   CanBus_StopFlywheels / CanBus_KeepFlywheelsStopped / HAL_CAN_TxMailbox0CompleteCallback /
 *   HAL_CAN_TxMailbox1CompleteCallback / HAL_CAN_TxMailbox2CompleteCallback /
 *   HAL_CAN_TxMailbox0AbortCallback / HAL_CAN_TxMailbox1AbortCallback / HAL_CAN_TxMailbox2AbortCallback
 *   / CanBus_Send / HAL_CAN_ErrorCallback
 * 内部函数：
 *   PendingOwned / CancelOwned / SubmitOwned / HasTwoFreeMailboxes / AbortOutputs / AbortFlywheelOutputs
 *   / BusIndex / SendYawFrame / YawSendResult / SendPitchFrame / PitchSendResult / SendFeederCurrent /
 *   SendJogCurrent / RecordJogTxFailure / TxComplete / TxAbortComplete / RememberFirstError /
 *   RecordError / HasCanFault
 * 文件级变量（函数局部变量在各函数内定义）：
 *   rx_queue：接收队列，连接中断生产者与任务消费者。
 *   state：本模块维护并向外发布的设备或流程状态快照。
 *   output_permission：对应动力路径的独立 CAN 发送许可。
 *   yaw_permission：对应动力路径的独立 CAN 发送许可。
 *   pitch_permission：对应动力路径的独立 CAN 发送许可。
 *   feeder_permission：对应动力路径的独立 CAN 发送许可。
 *   tx_owner：各 CAN 邮箱对应的执行者归属表。
 */
#include "can_bus.h"
#include "app_config.h"
#include "app_fault.h"
#include "dm3507.h"
#include "gl60_ii.h"
#include "yaw_controller.h"
#include "queue.h"
#include <string.h>

static QueueHandle_t rx_queue;
static volatile CanBusState state;
static volatile uint8_t output_permission;
static volatile uint8_t yaw_permission;
static volatile uint8_t pitch_permission;
static volatile uint8_t feeder_permission;

/* bxCAN返回实际占用邮箱位。只在提交临界区记录归属，完成/取消回调清除。
 * 单轴撤权不得取消另一轴；只有共同总线故障允许取消全部邮箱。 */
typedef enum {
    TX_OWNER_NONE = 0U, TX_OWNER_YAW, TX_OWNER_PITCH,
    TX_OWNER_FLYWHEEL, TX_OWNER_GENERIC, TX_OWNER_FEEDER
} TxOwner;
static volatile uint8_t tx_owner[2][3];

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static uint32_t PendingOwned(uint32_t index, TxOwner owner);
static HAL_StatusTypeDef CancelOwned(uint32_t index, TxOwner owner);
static HAL_StatusTypeDef SubmitOwned(CAN_HandleTypeDef *handle,
    const CAN_TxHeaderTypeDef *header, const uint8_t *data, TxOwner owner);
static uint8_t HasTwoFreeMailboxes(CAN_HandleTypeDef *handle);
static void RememberFirstError(uint32_t index, CanBusErrorSource source, uint32_t code);
static void RecordError(uint32_t index, CanBusErrorSource source, CAN_HandleTypeDef *handle);
static HAL_StatusTypeDef AbortOutputs(void);
static uint8_t HasCanFault(void);
static HAL_StatusTypeDef AbortFlywheelOutputs(void);
static int32_t BusIndex(CAN_HandleTypeDef *hcan);
static HAL_StatusTypeDef SendYawFrame(const CanFrame *frame);
static HAL_StatusTypeDef YawSendResult(HAL_StatusTypeDef result);
static HAL_StatusTypeDef SendPitchFrame(const CanFrame *frame);
static HAL_StatusTypeDef PitchSendResult(HAL_StatusTypeDef result);
static HAL_StatusTypeDef SendFeederCurrent(int16_t current);
static HAL_StatusTypeDef SendJogCurrent(uint8_t node, int32_t milliamps);
static void RecordJogTxFailure(void);
static void TxComplete(CAN_HandleTypeDef *hcan, uint32_t mailbox);
static void TxAbortComplete(CAN_HandleTypeDef *hcan, uint32_t mailbox);

/* 仅这两个公开接口能进入只读发送路径；其构造器拒绝写入、使能、
 * 设零、检测电机以及转速/电流命令。点动不能扩展读白名单。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：在 CAN 接收中断中读取 FIFO，附加时间戳和故障代次后入队。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 流程：这里只搬运和做基本帧检查；设备协议解码交给 CanTask 和各后端。
 * 流程：队列交接使用 FromISR 接口，必要时请求调度器切换到被唤醒的任务。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    BaseType_t woken = pdFALSE;
    int32_t index = BusIndex(hcan);
    if (index < 0) { return; }
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
        CAN_RxHeaderTypeDef header;
        CanFrame frame = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
            RecordError((uint32_t)index, CAN_BUS_SOURCE_RX_READ, hcan);
            (void)AbortOutputs();
            break;
        }
        if (header.RTR != CAN_RTR_DATA || header.DLC > 8U ||
            (header.IDE != CAN_ID_STD && header.IDE != CAN_ID_EXT)) { continue; }
        frame.bus = (uint8_t)(index + 1);
        frame.extended = header.IDE == CAN_ID_EXT;
        frame.id = frame.extended ? header.ExtId : header.StdId;
        frame.dlc = (uint8_t)header.DLC;
        /* HAL可能复制整个邮箱的8字节；DLC之外清零，避免短帧带入残留数据。 */
        if (frame.dlc < 8U) { memset(&frame.data[frame.dlc], 0, 8U - frame.dlc); }
        frame.received_ms = HAL_GetTick(); // 保存真实接收时刻，排队延迟不能延长反馈寿命。
        frame.fault_epoch = state.fault_epoch;
        if (!CanFrame_IsValid(&frame)) { continue; }
        state.rx_frames[index]++;
        if (xQueueSendFromISR(rx_queue, &frame, &woken) != pdPASS) {
            RememberFirstError((uint32_t)index, CAN_BUS_SOURCE_RX_QUEUE, HAL_CAN_GetError(hcan));
            state.dropped++;
            state.fault_epoch++;
            (void)AbortOutputs();
        }
    }
    portYIELD_FROM_ISR(woken);
}

void CanBus_Start(void)
/*
 * 作用：配置双 CAN 过滤器、启动接收和通知，并记录启动失败。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    CAN_FilterTypeDef filter = {0};
    CAN_HandleTypeDef *handles[2] = { &hcan1, &hcan2 };
    uint32_t i;
    const uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
        CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE |
        CAN_IT_ERROR_WARNING | CAN_IT_LAST_ERROR_CODE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    /* 零掩码同时接收标准/扩展帧；软件校验IDE、ID、RTR和DLC。
     * 共享过滤器0..13给CAN1、14..27给CAN2。不得从旧工程复制单CAN过滤。 */
    for (i = 0U; i < 2U; ++i) {
        filter.FilterBank = i == 0U ? 0U : 14U;
        if (HAL_CAN_ConfigFilter(handles[i], &filter) != HAL_OK ||
            HAL_CAN_Start(handles[i]) != HAL_OK ||
            HAL_CAN_ActivateNotification(handles[i], notifications) != HAL_OK) {
            AppFault_Fatal(APP_FAULT_CAN_START);
        }
    }
    state.started = 1U;
}

HAL_StatusTypeDef CanBus_YawTorque(float torque_nm)
/*
 * 作用：校验和限幅 Yaw 力矩，生成纯力矩帧并受专属许可约束提交。
 * 参数 torque_nm：协议力矩，N*m。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CanFrame frame;
    HAL_StatusTypeDef result;
    /* V2B完整MIT协议范围，移除原0.10校验和raw2028..2067截断。
     * NaN/Inf仍由编码器拒绝，协议范围之外编码到0/4095端点。 */
    if (!Gl60_BuildTorque(torque_nm, &frame)) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!CanBus_YawAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = YawSendResult(SendYawFrame(&frame));
    taskEXIT_CRITICAL(); return result;
}

HAL_StatusTypeDef CanBus_PitchTorque(float torque_nm)
/*
 * 作用：校验和限幅 Pitch 力矩，按固定协议编码后受专属许可约束提交。
 * 参数 torque_nm：协议力矩，N*m。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CanFrame frame;
    HAL_StatusTypeDef result;
    /* 构造器拒绝NaN/Inf；只在本机DM协议±10Nm端点饱和，无额外业务限幅。 */
    if (!Dm3507_MakeTorque(torque_nm, &frame)) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!CanBus_PitchAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = PitchSendResult(SendPitchFrame(&frame));
    taskEXIT_CRITICAL();
    return result;
}

HAL_StatusTypeDef CanBus_FeederCurrent(int16_t raw_current)
/*
 * 作用：检查拨盘许可和电流边界后提交 C610 电流，不占用其他设备槽位。
 * 参数 raw_current：C610 协议原始电流值，并非 mA。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    HAL_StatusTypeDef result;
    if (raw_current < -APP_FEEDER_CURRENT_HARD_LIMIT ||
        raw_current > APP_FEEDER_CURRENT_HARD_LIMIT) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!CanBus_FeederAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    /* 不在硬件邮箱积压位置环旧电流；拥塞时下周期仅提交最新控制量。 */
    if (PendingOwned(1U, TX_OWNER_FEEDER)) { taskEXIT_CRITICAL(); return HAL_BUSY; }
    result = SendFeederCurrent(raw_current);
    if (result == HAL_ERROR) {
        RecordError(1U, CAN_BUS_SOURCE_TX, &hcan2); (void)AbortOutputs();
    }
    taskEXIT_CRITICAL(); return result;
}

HAL_StatusTypeDef CanBus_JogVescCurrent(uint8_t node, int32_t milliamps)
/*
 * 作用：校验摩擦轮节点、电流及许可后，发送固定 VESC 电流命令。
 * 参数 node：VESC 节点 ID。
 * 参数 milliamps：摩擦轮电流请求，mA。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    HAL_StatusTypeDef result;
    int32_t limit;
    if ((node != 70U && node != 71U)) { return HAL_ERROR; }
    limit = node == 70U ? APP_FLYWHEEL_JOG_LEFT_MAX_MA : APP_FLYWHEEL_JOG_RIGHT_MAX_MA;
    if (milliamps < -limit || milliamps > limit) { return HAL_ERROR; }
    /* 先限幅再乘符号，INT32_MIN也不会溢出。每个节点仅开放编译时方向，
     * 不能借signed电流接口任意换向；零仍须门禁，撤权只走固定STOP。 */
    if (milliamps * (node == 70U ? APP_FLYWHEEL_LEFT_CURRENT_SIGN :
        APP_FLYWHEEL_RIGHT_CURRENT_SIGN) < 0) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!state.started || !output_permission || HasCanFault()) {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
    result = SendJogCurrent(node, milliamps);
    if (result != HAL_OK) {
        RecordJogTxFailure();
        (void)AbortOutputs();
    }
    taskEXIT_CRITICAL();
    return result;
}

HAL_StatusTypeDef CanBus_YawEnable(void)
/*
 * 作用：在许可有效时提交 Yaw 固定使能帧，不开放任意协议报文。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CanFrame frame;
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    if (!CanBus_YawAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    Gl60_BuildEnable(&frame);
    result = YawSendResult(SendYawFrame(&frame));
    taskEXIT_CRITICAL(); return result;
}

HAL_StatusTypeDef CanBus_PitchEnable(void)
/*
 * 作用：在许可有效时提交 Pitch 固定使能帧。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CanFrame frame;
    HAL_StatusTypeDef result;
    if (!Dm3507_MakeEnable(&frame)) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!CanBus_PitchAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = PitchSendResult(SendPitchFrame(&frame));
    taskEXIT_CRITICAL();
    return result;
}

BaseType_t CanBus_Receive(CanFrame *frame, TickType_t wait_ticks)
/*
 * 作用：从接收队列取出一帧，可按指定 RTOS 等待时间阻塞。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 参数 wait_ticks：队列最多阻塞的 FreeRTOS tick 数，当前 1 tick 为 1 ms。
 * 返回队列接收结果；pdPASS 表示成功取到一帧。
 */
{
    return xQueueReceive(rx_queue, frame, wait_ticks);
}

void CanBus_CreateQueue(void)
/*
 * 作用：创建 CAN 接收帧队列，供接收中断与 CAN 任务交接。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    rx_queue = xQueueCreate(64U, sizeof(CanFrame));
    if (rx_queue == 0) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "GIMBAL_CAN");
}

static uint32_t PendingOwned(uint32_t index, TxOwner owner)
/*
 * 作用：汇总指定 CAN 总线上属于某个执行者的待发邮箱掩码。
 * 参数 index：CAN 总线下标，0 对应 CAN1，1 对应 CAN2。
 * 参数 owner：待发邮箱归属的执行器，避免取消另一台电机的报文。
 * 返回位掩码；每一位表示一个待发邮箱或阻塞原因，0 表示没有。
 */
{
    CAN_HandleTypeDef *handle = index == 0U ? &hcan1 : &hcan2;
    uint32_t mailbox, mask = 0U;
    if (!state.started) { return 0U; }
    for (mailbox = 0U; mailbox < 3U; ++mailbox) {
        uint32_t bit = 1U << mailbox;
        if (tx_owner[index][mailbox] != (uint8_t)owner) { continue; }
        if (HAL_CAN_IsTxMessagePending(handle, bit)) { mask |= bit; }
        else { tx_owner[index][mailbox] = TX_OWNER_NONE; }
    }
    return mask;
}

static HAL_StatusTypeDef CancelOwned(uint32_t index, TxOwner owner)
/*
 * 作用：只取消指定执行者拥有的待发邮箱，保持其他电机发送归属。
 * 参数 index：CAN 总线下标，0 对应 CAN1，1 对应 CAN2。
 * 参数 owner：待发邮箱归属的执行器，避免取消另一台电机的报文。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    uint32_t mask;
    if (!state.started) { return HAL_ERROR; }
    mask = PendingOwned(index, owner);
    if (!mask) { return HAL_OK; }
    return HAL_CAN_AbortTxRequest(index == 0U ? &hcan1 : &hcan2, mask);
}

/* 调用方持有任务临界区；硬件拥塞返回BUSY，不伪造一次总线故障。 */
static HAL_StatusTypeDef SubmitOwned(CAN_HandleTypeDef *handle,
    const CAN_TxHeaderTypeDef *header, const uint8_t *data, TxOwner owner)
/*
 * 作用：提交专用 CAN 帧并记录邮箱所有者，供停止和完成中断核对。
 * 参数 handle：对应外设的 HAL 句柄。
 * 参数 header：已构造好的 CAN 发送帧头。
 * 参数 data：协议原始字节缓冲区。
 * 参数 owner：待发邮箱归属的执行器，避免取消另一台电机的报文。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    uint32_t mailbox, bit, mask = 0U, index = handle == &hcan1 ? 0U : 1U;
    HAL_StatusTypeDef result;
    for (bit = 0U; bit < 3U; ++bit) {
        if (HAL_CAN_IsTxMessagePending(handle, 1U << bit)) { mask |= 1U << bit; }
    }
    if (mask == 7U) { return HAL_BUSY; }
    result = HAL_CAN_AddTxMessage(handle, header, data, &mailbox);
    if (result == HAL_OK) {
        for (bit = 0U; bit < 3U; ++bit) {
            if (mailbox & (1U << bit)) { tx_owner[index][bit] = (uint8_t)owner; }
        }
    }
    return result;
}

static uint8_t HasTwoFreeMailboxes(CAN_HandleTypeDef *handle)
/*
 * 作用：检查同一 CAN 控制器是否能完整提交摩擦轮双零帧。
 * 参数 handle：对应外设的 HAL 句柄。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    uint32_t bit, free_count = 0U;
    for (bit = 0U; bit < 3U; ++bit) {
        if (!HAL_CAN_IsTxMessagePending(handle, 1U << bit)) { free_count++; }
    }
    return free_count >= 2U;
}

static HAL_StatusTypeDef AbortOutputs(void)
/*
 * 作用：撤销双 CAN 已提交的输出，配合故障处理取消旧动力命令。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    HAL_StatusTypeDef first, second;
    output_permission = 0U;
    yaw_permission = 0U;
    pitch_permission = 0U;
    feeder_permission = 0U;
    if (state.started) {
        first = HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        second = HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        return first == HAL_OK && second == HAL_OK ? HAL_OK : HAL_ERROR;
    }
    return HAL_ERROR;
}

/* CH6普通关闭仅撤双轮；统一STOP/ISR故障仍走AbortOutputs撤两路。 */
static HAL_StatusTypeDef AbortFlywheelOutputs(void)
/*
 * 作用：仅撤销摩擦轮拥有的输出，保留云台和拨盘的独立发送归属。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    output_permission = 0U;
    return CancelOwned(1U, TX_OWNER_FLYWHEEL);
}

static int32_t BusIndex(CAN_HandleTypeDef *hcan)
/*
 * 作用：把 CAN 句柄映射为内部总线下标，拒绝不属于本工程的句柄。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 返回转换或计算后的整数值，供调用方继续处理。
 */
{
    if (hcan == &hcan1) { return 0; }
    if (hcan == &hcan2) { return 1; }
    return -1;
}

void CanBus_GetState(CanBusState *out)
/*
 * 作用：复制 CAN 启动状态、帧计数、丢帧和故障代次。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值，将当前快照写入输出参数；不会启动新的运动。
 */
{
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    *out = state;
    taskEXIT_CRITICAL();
}

void CanBus_GetDiagnostics(CanBusDiagnostics *out)
/*
 * 作用：复制 CAN 运行计数、硬件错误和邮箱等只读诊断快照。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 不返回数值，将当前快照写入输出参数；不会启动新的运动。
 */
{
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    out->state = state;
    out->hal_error[0] = HAL_CAN_GetError(&hcan1);
    out->hal_error[1] = HAL_CAN_GetError(&hcan2);
    out->pending[0] = state.started ? HAL_CAN_IsTxMessagePending(&hcan1, mailboxes) : 0U;
    out->pending[1] = state.started ? HAL_CAN_IsTxMessagePending(&hcan2, mailboxes) : 0U;
    taskEXIT_CRITICAL();
}

void CanBus_SetOutputPermission(uint8_t allowed)
/*
 * 作用：更新摩擦轮输出许可；独立拨盘模式下保持轮动力关闭。
 * 参数 allowed：本轮是否允许输出动力。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint8_t previous;
    taskENTER_CRITICAL();
    previous = output_permission;
    output_permission = allowed && !APP_FEEDER_STANDALONE_TEST_ENABLED &&
        state.started && !HasCanFault();
    if (previous && !output_permission) {
        /* 撤权与取消旧请求同处临界区，禁止非零帧穿插入邮箱。 */
        (void)AbortFlywheelOutputs();
    }
    taskEXIT_CRITICAL();
}

uint8_t CanBus_OutputAllowed(void)
/*
 * 作用：查询摩擦轮输出许可是否仍满足启动及总线健康条件。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    uint8_t allowed;
    taskENTER_CRITICAL();
    allowed = output_permission && state.started && !HasCanFault();
    taskEXIT_CRITICAL();
    return allowed;
}

void CanBus_SetYawPermission(uint8_t allowed)
/*
 * 作用：更新 Yaw 专用发送许可，撤权时取消该轴旧输出。
 * 参数 allowed：本轮是否允许输出动力。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint8_t previous;
    taskENTER_CRITICAL();
    previous = yaw_permission;
    yaw_permission = allowed && state.started && !HasCanFault();
    if (previous && !yaw_permission) {
        (void)CancelOwned(0U, TX_OWNER_YAW);
    }
    taskEXIT_CRITICAL();
}

uint8_t CanBus_YawAllowed(void)
/*
 * 作用：查询 Yaw 专用发送许可及总线健康状态。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    return yaw_permission && state.started && !HasCanFault();
}

static HAL_StatusTypeDef SendYawFrame(const CanFrame *frame)
/*
 * 作用：通过 Yaw 专属发送路径提交固定协议 CAN 帧。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    header.IDE = CAN_ID_STD; header.StdId = 1U;
    header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    (void)mailbox;
    return SubmitOwned(&hcan1, &header, frame->data, TX_OWNER_YAW);
}

static HAL_StatusTypeDef YawSendResult(HAL_StatusTypeDef result)
/*
 * 作用：统一处理 Yaw 发送结果；真正的传输错误进入总线故障路径。
 * 参数 result：发送或业务动作的结果码，供上层处理及串口回执。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    if (result != HAL_OK
        && result != HAL_BUSY
        ) {
        RecordError(0U, CAN_BUS_SOURCE_TX, &hcan1);
        (void)AbortOutputs();
    }
    return result;
}

HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel_old)
/*
 * 作用：撤销 Yaw 许可并按需取消旧帧，提交固定失能帧。
 * 参数 cancel_old：是否先取消该轴未完成的旧报文。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CanFrame frame;
    HAL_StatusTypeDef result;
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    taskENTER_CRITICAL();
    yaw_permission = 0U;
    if (!state.started) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    (void)mailboxes;
    if (cancel_old && CancelOwned(0U, TX_OWNER_YAW) != HAL_OK) {
        taskEXIT_CRITICAL(); return HAL_ERROR;
    }
    if (PendingOwned(0U, TX_OWNER_YAW)) {
        taskEXIT_CRITICAL(); return HAL_BUSY;
    }
    Gl60_BuildDisable(&frame);
    result = SendYawFrame(&frame);
    /* 固定FD始终可尝试；不从一次提交推断机械已停或清CAN故障。 */
    taskEXIT_CRITICAL(); return result;
}

uint32_t CanBus_YawTxPending(void)
/*
 * 作用：读取当前归属于 Yaw 的待发邮箱掩码。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回位掩码；每一位表示一个待发邮箱或阻塞原因，0 表示没有。
 */
{
    uint32_t pending;
    taskENTER_CRITICAL();
    pending = PendingOwned(0U, TX_OWNER_YAW);
    taskEXIT_CRITICAL(); return pending;
}

HAL_StatusTypeDef CanBus_YawClearComms(void)
/*
 * 作用：在原清错门禁允许时提交 Yaw 固定通信错误清除帧。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CanFrame frame;
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    if (!state.started || output_permission || yaw_permission || pitch_permission || HasCanFault()) {
        taskEXIT_CRITICAL(); return HAL_ERROR;
    }
    if (HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)) {
        taskEXIT_CRITICAL(); return HAL_BUSY;
    }
    Gl60_BuildClearError(&frame);
    result = YawSendResult(SendYawFrame(&frame));
    taskEXIT_CRITICAL(); return result;
}

void CanBus_SetPitchPermission(uint8_t allowed)
/*
 * 作用：更新 Pitch 专用发送许可，撤权时取消该轴旧输出。
 * 参数 allowed：本轮是否允许输出动力。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint8_t previous;
    taskENTER_CRITICAL();
    previous = pitch_permission;
    pitch_permission = allowed && state.started && !HasCanFault();
    if (previous && !pitch_permission) { (void)CancelOwned(0U, TX_OWNER_PITCH); }
    taskEXIT_CRITICAL();
}

uint8_t CanBus_PitchAllowed(void)
/*
 * 作用：查询 Pitch 专用发送许可及总线健康状态。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    uint8_t allowed;
    taskENTER_CRITICAL();
    allowed = pitch_permission && state.started && !HasCanFault();
    taskEXIT_CRITICAL();
    return allowed;
}

static HAL_StatusTypeDef SendPitchFrame(const CanFrame *frame)
/*
 * 作用：通过 Pitch 专属发送路径提交固定协议 CAN 帧。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CAN_TxHeaderTypeDef header = {0};
    if (!CanFrame_IsValid(frame) || frame->bus != 1U || frame->extended ||
        frame->id != 2U || frame->dlc != 8U) { return HAL_ERROR; }
    header.IDE = CAN_ID_STD; header.StdId = 2U;
    header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    return SubmitOwned(&hcan1, &header, frame->data, TX_OWNER_PITCH);
}

static HAL_StatusTypeDef PitchSendResult(HAL_StatusTypeDef result)
/*
 * 作用：统一处理 Pitch 发送结果；真正的传输错误进入总线故障路径。
 * 参数 result：发送或业务动作的结果码，供上层处理及串口回执。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    if (result != HAL_OK && result != HAL_BUSY) {
        RecordError(0U, CAN_BUS_SOURCE_TX, &hcan1);
        (void)AbortOutputs();
    }
    return result;
}

HAL_StatusTypeDef CanBus_PitchDisable(uint8_t cancel_old)
/*
 * 作用：撤销 Pitch 许可并按需取消旧帧，提交固定失能帧。
 * 参数 cancel_old：是否先取消该轴未完成的旧报文。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CanFrame frame;
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    pitch_permission = 0U;
    if (!state.started) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    if (cancel_old && CancelOwned(0U, TX_OWNER_PITCH) != HAL_OK) {
        taskEXIT_CRITICAL(); return HAL_ERROR;
    }
    if (PendingOwned(0U, TX_OWNER_PITCH)) { taskEXIT_CRITICAL(); return HAL_BUSY; }
    if (!Dm3507_MakeDisable(&frame)) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = SendPitchFrame(&frame);
    /* 即便已经锁存CAN故障仍可尝试固定FD；不从HAL_OK推断下垂负载被保持。 */
    taskEXIT_CRITICAL();
    return result;
}

HAL_StatusTypeDef CanBus_PitchReadFeedback(void)
/*
 * 作用：提交 Pitch 固定状态查询帧，供未使能时监测及准备流程使用。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    static const uint8_t payload[8] = {2U, 0U, 0xCCU, 0U, 0U, 0U, 0U, 0U};
    CanFrame frame;
    CAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef result;
    /* dmBots官方SDK refresh_motor_status：0x7FF、SlaveID小端、0xCC。
     * 此查询不使能、不更改输出参考，也不写参数/EEPROM。 */
    if (!Dm3507_MakeReadFeedback(&frame) || !CanFrame_IsValid(&frame) ||
        frame.bus != 1U || frame.extended || frame.id != 0x7FFU || frame.dlc != 8U ||
        memcmp(frame.data, payload, sizeof(payload)) != 0) { return HAL_ERROR; }
    header.IDE = CAN_ID_STD; header.StdId = 0x7FFU;
    header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    taskENTER_CRITICAL();
    if (!state.started || HasCanFault()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = PitchSendResult(SubmitOwned(&hcan1, &header, frame.data, TX_OWNER_PITCH));
    taskEXIT_CRITICAL();
    return result;
}

uint32_t CanBus_PitchTxPending(void)
/*
 * 作用：读取当前归属于 Pitch 的待发邮箱掩码。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回位掩码；每一位表示一个待发邮箱或阻塞原因，0 表示没有。
 */
{
    uint32_t pending;
    taskENTER_CRITICAL();
    pending = PendingOwned(0U, TX_OWNER_PITCH);
    taskEXIT_CRITICAL();
    return pending;
}

HAL_StatusTypeDef CanBus_CancelPitchTx(void)
/*
 * 作用：只取消 Pitch 归属的待发帧，返回邮箱是否已经清空。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    result = CancelOwned(0U, TX_OWNER_PITCH);
    if (result == HAL_OK && PendingOwned(0U, TX_OWNER_PITCH)) { result = HAL_BUSY; }
    taskEXIT_CRITICAL();
    return result;
}

void CanBus_SetFeederPermission(uint8_t allowed)
/*
 * 作用：更新拨盘专用电流许可，撤权时取消拨盘旧输出。
 * 参数 allowed：本轮是否允许输出动力。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    taskENTER_CRITICAL();
    feeder_permission = allowed && state.started && !HasCanFault();
    if (!feeder_permission) { (void)CancelOwned(1U, TX_OWNER_FEEDER); }
    taskEXIT_CRITICAL();
}

uint8_t CanBus_FeederAllowed(void)
/*
 * 作用：查询拨盘专用许可及 CAN 健康状态。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{ return feeder_permission && state.started && !HasCanFault(); }

static HAL_StatusTypeDef SendFeederCurrent(int16_t current)
/*
 * 作用：把拨盘电流原始值编码到 C610 固定控制槽并提交。
 * 参数 current：C610 原始电流指令，并非 mA。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint16_t raw = (uint16_t)current;
    header.IDE = CAN_ID_STD; header.StdId = 0x1FFU;
    header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    data[0] = (uint8_t)(raw >> 8); data[1] = (uint8_t)raw;
    return SubmitOwned(&hcan2, &header, data, TX_OWNER_FEEDER);
}

HAL_StatusTypeDef CanBus_StopFeeder(void)
/*
 * 作用：撤销拨盘许可，取消该设备旧输出并提交零电流。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    HAL_StatusTypeDef result = HAL_ERROR;
    taskENTER_CRITICAL();
    feeder_permission = 0U;
    if (state.started) {
        result = CancelOwned(1U, TX_OWNER_FEEDER);
        if (result == HAL_OK && PendingOwned(1U, TX_OWNER_FEEDER)) { result = HAL_BUSY; }
        if (result == HAL_OK) { result = SendFeederCurrent(0); }
        if (result == HAL_ERROR) { RecordError(1U, CAN_BUS_SOURCE_TX, &hcan2); (void)AbortOutputs(); }
    }
    taskEXIT_CRITICAL(); return result;
}

/* 调用方持有任务临界区。协议固定为VESC SET_CURRENT=1，
 * int32电流*1000，即mA大端；+0.2A=00 00 00 C8，-0.2A=FF FF FF 38。 */
static HAL_StatusTypeDef SendJogCurrent(uint8_t node, int32_t milliamps)
/*
 * 作用：按 VESC 固定电流协议编码指定摩擦轮的 mA 请求。
 * 参数 node：VESC 节点 ID。
 * 参数 milliamps：摩擦轮电流请求，mA。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint32_t raw = (uint32_t)milliamps, mailbox;
    header.IDE = CAN_ID_EXT;
    header.ExtId = (1U << 8) | node;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 4U;
    header.TransmitGlobalTime = DISABLE;
    data[0] = (uint8_t)(raw >> 24);
    data[1] = (uint8_t)(raw >> 16);
    data[2] = (uint8_t)(raw >> 8);
    data[3] = (uint8_t)raw;
    (void)mailbox;
    return SubmitOwned(&hcan2, &header, data, TX_OWNER_FLYWHEEL);
}

static void RecordJogTxFailure(void)
/*
 * 作用：记录摩擦轮传输错误并使原输出进入故障撤权路径。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    RecordError(1U, CAN_BUS_SOURCE_TX, &hcan2);
}

HAL_StatusTypeDef CanBus_StopFlywheels(void)
/*
 * 作用：取消摩擦轮旧输出，并在邮箱足够时完整提交两侧零电流。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    HAL_StatusTypeDef result, first, second;
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    taskENTER_CRITICAL();
    result = AbortFlywheelOutputs();
    /* bxCAN取消是异步操作。旧请求未清完时不复用邮箱，交由上层下周期重试。 */
    if (result != HAL_OK ||
        PendingOwned(1U, TX_OWNER_FLYWHEEL)) {
        (void)mailboxes;
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
    /* 双零需要两个空邮箱；保留其他会话，不能半途拥塞并误记为CAN故障。 */
    if (!HasTwoFreeMailboxes(&hcan2)) { taskEXIT_CRITICAL(); return HAL_BUSY; }
    first = SendJogCurrent(70U, 0);
    second = SendJogCurrent(71U, 0);
    if (first != HAL_OK) { RecordJogTxFailure(); }
    if (second != HAL_OK) { RecordJogTxFailure(); }
    /* 已报告的CAN故障不妨碍尝试固定回零，但不能报告停机提交成功。 */
    result = first == HAL_OK && second == HAL_OK && HAL_CAN_GetError(&hcan1) == 0U &&
        HAL_CAN_GetError(&hcan2) == 0U ? HAL_OK : HAL_ERROR;
    taskEXIT_CRITICAL();
    return result;
}

HAL_StatusTypeDef CanBus_KeepFlywheelsStopped(void)
/*
 * 作用：维持两侧摩擦轮零电流，避免半组零帧或影响拨盘归属。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{
    HAL_StatusTypeDef first, second, result;
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    taskENTER_CRITICAL();
    if (!state.started || output_permission) {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
    /* 首次STOP已负责取消旧动力请求。刷新只等待邮箱空闲，保留正常读查询。 */
    if (
        HAL_CAN_IsTxMessagePending(&hcan2, mailboxes)) {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    first = SendJogCurrent(70U, 0);
    second = SendJogCurrent(71U, 0);
    if (first != HAL_OK) { RecordJogTxFailure(); }
    if (second != HAL_OK) { RecordJogTxFailure(); }
    result = first == HAL_OK && second == HAL_OK && HAL_CAN_GetError(&hcan1) == 0U &&
        HAL_CAN_GetError(&hcan2) == 0U ? HAL_OK : HAL_ERROR;
    taskEXIT_CRITICAL();
    return result;
}

static void TxComplete(CAN_HandleTypeDef *hcan, uint32_t mailbox)
/*
 * 作用：核对硬件待发位后处理邮箱完成事件，保留已被复用的邮箱归属。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 参数 mailbox：邮箱下标，0、1、2 分别对应三个硬件发送邮箱。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    int32_t index = BusIndex(hcan);
    if (index >= 0) {
        state.tx_frames[index]++;
        /* 完成IRQ可能在临界区结束后才到达；旧邮箱已复用时保留新归属。 */
        if (!HAL_CAN_IsTxMessagePending(hcan, 1U << mailbox)) {
            tx_owner[index][mailbox] = TX_OWNER_NONE;
        }
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：把 CAN 邮箱 0 的发送完成中断交给归属核对逻辑。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ TxComplete(hcan, 0U); }

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：把 CAN 邮箱 1 的发送完成中断交给归属核对逻辑。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ TxComplete(hcan, 1U); }

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：把 CAN 邮箱 2 的发送完成中断交给归属核对逻辑。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ TxComplete(hcan, 2U); }

static void TxAbortComplete(CAN_HandleTypeDef *hcan, uint32_t mailbox)
/*
 * 作用：处理邮箱取消完成事件，防止迟到中断清除新帧的归属。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 参数 mailbox：邮箱下标，0、1、2 分别对应三个硬件发送邮箱。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    int32_t index = BusIndex(hcan);
    if (index >= 0 && !HAL_CAN_IsTxMessagePending(hcan, 1U << mailbox)) {
        tx_owner[index][mailbox] = TX_OWNER_NONE;
    }
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：把 CAN 邮箱 0 的取消完成中断交给归属核对逻辑。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ TxAbortComplete(hcan, 0U); }

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：把 CAN 邮箱 1 的取消完成中断交给归属核对逻辑。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ TxAbortComplete(hcan, 1U); }

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：把 CAN 邮箱 2 的取消完成中断交给归属核对逻辑。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{ TxAbortComplete(hcan, 2U); }

/* 任意帧接口始终拒绝；仅保留兼容性负向检查，不提交 CAN。 */
HAL_StatusTypeDef CanBus_Send(const CanFrame *frame)
/*
 * 作用：始终拒绝任意 CAN 帧请求；该兼容接口没有动力发送实现。
 * 参数 frame：包含总线/通道数据和接收时间的输入帧。
 * 返回 HAL 状态；HAL_OK 表示本次请求已提交，不等于电机已完成动作。
 */
{ (void)frame; return HAL_ERROR; }

static void RememberFirstError(uint32_t index, CanBusErrorSource source, uint32_t code)
/*
 * 作用：保存当前总线首次错误的来源、时间和硬件错误码。
 * 参数 index：CAN 总线下标，0 对应 CAN1，1 对应 CAN2。
 * 参数 source：本次 CAN 故障来源类别。
 * 参数 code：HAL 提供的硬件错误位集合。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (state.first_error_source[index] == CAN_BUS_SOURCE_NONE) {
        state.first_error[index] = code;
        state.first_error_ms[index] = HAL_GetTick();
        state.first_error_source[index] = (uint8_t)source;
    }
}

/* 调用方处于临界区或同优先级CAN ISR；先留证据，再由原路径撤权。 */
static void RecordError(uint32_t index, CanBusErrorSource source, CAN_HandleTypeDef *handle)
/*
 * 作用：累计总线错误和故障代次，并保存首次错误证据。
 * 参数 index：CAN 总线下标，0 对应 CAN1，1 对应 CAN2。
 * 参数 source：本次 CAN 故障来源类别。
 * 参数 handle：对应外设的 HAL 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    uint32_t code = HAL_CAN_GetError(handle);
    RememberFirstError(index, source, code);
    state.errors[index]++;
    state.last_error[index] = code;
    state.fault_epoch++;
}

static uint8_t HasCanFault(void)
/*
 * 作用：汇总双 CAN 的软件及硬件故障状态。
 * 输入：无显式参数，读取或维护本模块状态；相关变量见文件头。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
{
    return state.fault_epoch != 0U || HAL_CAN_GetError(&hcan1) != 0U ||
        HAL_CAN_GetError(&hcan2) != 0U;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
/*
 * 作用：记录 CAN 硬件错误并撤销对应旧输出。
 * 参数 hcan：产生中断的 CAN 句柄。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    int32_t index = BusIndex(hcan);
    if (index < 0) { return; }
    RecordError((uint32_t)index, CAN_BUS_SOURCE_IRQ, hcan);
    (void)AbortOutputs();
}
