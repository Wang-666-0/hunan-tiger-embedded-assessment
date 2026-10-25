/* CAN 传输：输出许可、邮箱所有权、故障代次及中断回调。 */
#include "can_bus.h"
#include "app_config.h"
#include "app_fault.h"
#include "dm_diag.h"
#include "vesc_diag.h"
#if APP_PITCH_ENABLED
#include "dm3507.h"
#endif
#if APP_YAW_ENABLED
#include "gl60_ii.h"
#include "yaw_controller.h"
#endif
#include "queue.h"
#include <string.h>

static QueueHandle_t rx_queue;
static volatile CanBusState state;
static volatile uint8_t output_permission;
static volatile uint8_t yaw_permission;
static volatile uint8_t pitch_permission;
static volatile uint8_t feeder_permission;

#if APP_PITCH_ENABLED
/* bxCAN返回实际占用邮箱位。只在提交临界区记录归属，完成/取消回调清除。
 * 单轴撤权不得取消另一轴；只有共同总线故障允许取消全部邮箱。 */
typedef enum {
    TX_OWNER_NONE = 0U, TX_OWNER_YAW, TX_OWNER_PITCH, TX_OWNER_DIAGNOSTIC,
    TX_OWNER_FLYWHEEL, TX_OWNER_GENERIC, TX_OWNER_FEEDER
} TxOwner;
static volatile uint8_t tx_owner[2][3];

static uint32_t PendingOwned(uint32_t index, TxOwner owner)
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
{
    uint32_t bit, free_count = 0U;
    for (bit = 0U; bit < 3U; ++bit) {
        if (!HAL_CAN_IsTxMessagePending(handle, 1U << bit)) { free_count++; }
    }
    return free_count >= 2U;
}
#endif

static void RememberFirstError(uint32_t index, CanBusErrorSource source, uint32_t code)
{
    if (state.first_error_source[index] == CAN_BUS_SOURCE_NONE) {
        state.first_error[index] = code;
        state.first_error_ms[index] = HAL_GetTick();
        state.first_error_source[index] = (uint8_t)source;
    }
}

/* 调用方处于临界区或同优先级CAN ISR；先留证据，再由原路径撤权。 */
static void RecordError(uint32_t index, CanBusErrorSource source, CAN_HandleTypeDef *handle)
{
    uint32_t code = HAL_CAN_GetError(handle);
    RememberFirstError(index, source, code);
    state.errors[index]++;
    state.last_error[index] = code;
    state.fault_epoch++;
}

static HAL_StatusTypeDef AbortOutputs(void)
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

static uint8_t HasCanFault(void)
{
    return state.fault_epoch != 0U || HAL_CAN_GetError(&hcan1) != 0U ||
        HAL_CAN_GetError(&hcan2) != 0U;
}

/* CH6普通关闭仅撤双轮；统一STOP/ISR故障仍走AbortOutputs撤两路。 */
static HAL_StatusTypeDef AbortFlywheelOutputs(void)
{
#if APP_PITCH_ENABLED
    output_permission = 0U;
    return CancelOwned(1U, TX_OWNER_FLYWHEEL);
#elif APP_YAW_ENABLED
    output_permission = 0U;
    return state.started ? HAL_CAN_AbortTxRequest(&hcan2,
        CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) : HAL_ERROR;
#else
    return AbortOutputs();
#endif
}

static int32_t BusIndex(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) { return 0; }
    if (hcan == &hcan2) { return 1; }
    return -1;
}

void CanBus_CreateQueue(void)
{
    rx_queue = xQueueCreate(APP_CAN_RX_QUEUE_LENGTH, sizeof(CanFrame));
    if (rx_queue == 0) { AppFault_Fatal(APP_FAULT_QUEUE); }
    vQueueAddToRegistry(rx_queue, "GIMBAL_CAN");
}

void CanBus_Start(void)
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

BaseType_t CanBus_Receive(CanFrame *frame, TickType_t wait_ticks)
{
    return xQueueReceive(rx_queue, frame, wait_ticks);
}

void CanBus_GetState(CanBusState *out)
{
    if (out == 0) { return; }
    taskENTER_CRITICAL();
    *out = state;
    taskEXIT_CRITICAL();
}

void CanBus_GetDiagnostics(CanBusDiagnostics *out)
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
{
    uint8_t previous;
    taskENTER_CRITICAL();
    previous = output_permission;
    output_permission = allowed && !APP_FEEDER_STANDALONE_TEST_ACTIVE &&
        (APP_MOTOR_DRIVERS_IMPLEMENTED || APP_FLYWHEEL_JOG_ENABLED) &&
        state.started && !HasCanFault();
    if (previous && !output_permission) {
        /* 撤权与取消旧请求同处临界区，禁止非零帧穿插入邮箱。 */
        (void)AbortFlywheelOutputs();
    }
    taskEXIT_CRITICAL();
}

uint8_t CanBus_OutputAllowed(void)
{
    uint8_t allowed;
    taskENTER_CRITICAL();
    allowed = output_permission && state.started && !HasCanFault();
    taskEXIT_CRITICAL();
    return allowed;
}

void CanBus_SetYawPermission(uint8_t allowed)
{
    uint8_t previous;
    taskENTER_CRITICAL();
    previous = yaw_permission;
    yaw_permission = allowed && APP_YAW_ENABLED && state.started && !HasCanFault();
    if (previous && !yaw_permission) {
#if APP_PITCH_ENABLED
        (void)CancelOwned(0U, TX_OWNER_YAW);
#else
        (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
#endif
    }
    taskEXIT_CRITICAL();
}

uint8_t CanBus_YawAllowed(void)
{
    return yaw_permission && state.started && !HasCanFault();
}

#if APP_YAW_ENABLED
static HAL_StatusTypeDef SendYawFrame(const CanFrame *frame)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    header.IDE = CAN_ID_STD; header.StdId = 1U;
    header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
#if APP_PITCH_ENABLED
    (void)mailbox;
    return SubmitOwned(&hcan1, &header, frame->data, TX_OWNER_YAW);
#else
    return HAL_CAN_AddTxMessage(&hcan1, &header, frame->data, &mailbox);
#endif
}

static HAL_StatusTypeDef YawSendResult(HAL_StatusTypeDef result)
{
    if (result != HAL_OK
#if APP_PITCH_ENABLED
        && result != HAL_BUSY
#endif
        ) {
        RecordError(0U, CAN_BUS_SOURCE_TX, &hcan1);
        (void)AbortOutputs();
    }
    return result;
}
#endif

HAL_StatusTypeDef CanBus_YawEnable(void)
{
#if APP_YAW_ENABLED
    CanFrame frame;
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    if (!CanBus_YawAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    Gl60_BuildEnable(&frame);
    result = YawSendResult(SendYawFrame(&frame));
    taskEXIT_CRITICAL(); return result;
#else
    return HAL_ERROR;
#endif
}

HAL_StatusTypeDef CanBus_YawTorque(float torque_nm)
{
#if APP_YAW_ENABLED
    CanFrame frame;
    HAL_StatusTypeDef result;
    /* V2B完整MIT协议范围，移除原0.10校验和raw2028..2067截断。
     * NaN/Inf仍由编码器拒绝，协议范围之外编码到0/4095端点。 */
#if !YAW_CONTROLLER_CASCADE_ENABLED || YAW_CASCADE_OUTPUT_LIMITS_ENABLED
    if (!(torque_nm >= -YAW_CONTROLLER_TORQUE_LIMIT_NM &&
          torque_nm <= YAW_CONTROLLER_TORQUE_LIMIT_NM) || !Gl60_BuildTorque(torque_nm, &frame)) {
        return HAL_ERROR;
    }
    {
        uint16_t raw = (uint16_t)(((frame.data[6] & 15U) << 8) | frame.data[7]);
        const uint16_t lower = 2028U, upper = 2067U; /* +/-0.095238Nm，均小于0.10 */
        if (raw < lower) { raw = lower; }
        if (raw > upper) { raw = upper; }
        frame.data[6] = (uint8_t)((frame.data[6] & 0xF0U) | (raw >> 8));
        frame.data[7] = (uint8_t)raw;
    }
#else
    if (!Gl60_BuildTorque(torque_nm, &frame)) { return HAL_ERROR; }
#endif
    taskENTER_CRITICAL();
    if (!CanBus_YawAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = YawSendResult(SendYawFrame(&frame));
    taskEXIT_CRITICAL(); return result;
#else
    (void)torque_nm; return HAL_ERROR;
#endif
}

HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel_old)
{
#if APP_YAW_ENABLED
    CanFrame frame;
    HAL_StatusTypeDef result;
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    taskENTER_CRITICAL();
    yaw_permission = 0U;
    if (!state.started) { taskEXIT_CRITICAL(); return HAL_ERROR; }
#if APP_PITCH_ENABLED
    (void)mailboxes;
    if (cancel_old && CancelOwned(0U, TX_OWNER_YAW) != HAL_OK) {
#else
    if (cancel_old && HAL_CAN_AbortTxRequest(&hcan1, mailboxes) != HAL_OK) {
#endif
        taskEXIT_CRITICAL(); return HAL_ERROR;
    }
#if APP_PITCH_ENABLED
    if (PendingOwned(0U, TX_OWNER_YAW)) {
#else
    if (HAL_CAN_IsTxMessagePending(&hcan1, mailboxes)) {
#endif
        taskEXIT_CRITICAL(); return HAL_BUSY;
    }
    Gl60_BuildDisable(&frame);
    result = SendYawFrame(&frame);
    /* 固定FD始终可尝试；不从一次提交推断机械已停或清CAN故障。 */
    taskEXIT_CRITICAL(); return result;
#else
    (void)cancel_old; return HAL_ERROR;
#endif
}

uint32_t CanBus_YawTxPending(void)
{
    uint32_t pending;
    taskENTER_CRITICAL();
#if APP_PITCH_ENABLED
    pending = PendingOwned(0U, TX_OWNER_YAW);
#else
    pending = state.started ? HAL_CAN_IsTxMessagePending(&hcan1,
        CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) : 0U;
#endif
    taskEXIT_CRITICAL(); return pending;
}

HAL_StatusTypeDef CanBus_YawClearComms(void)
{
#if APP_YAW_ENABLED
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
#else
    return HAL_ERROR;
#endif
}

void CanBus_SetPitchPermission(uint8_t allowed)
{
    uint8_t previous;
    taskENTER_CRITICAL();
    previous = pitch_permission;
    pitch_permission = allowed && APP_PITCH_ENABLED && state.started && !HasCanFault();
#if APP_PITCH_ENABLED
    if (previous && !pitch_permission) { (void)CancelOwned(0U, TX_OWNER_PITCH); }
#else
    (void)previous;
#endif
    taskEXIT_CRITICAL();
}

uint8_t CanBus_PitchAllowed(void)
{
    uint8_t allowed;
    taskENTER_CRITICAL();
    allowed = pitch_permission && APP_PITCH_ENABLED && state.started && !HasCanFault();
    taskEXIT_CRITICAL();
    return allowed;
}

#if APP_PITCH_ENABLED
static HAL_StatusTypeDef SendPitchFrame(const CanFrame *frame)
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
{
    if (result != HAL_OK && result != HAL_BUSY) {
        RecordError(0U, CAN_BUS_SOURCE_TX, &hcan1);
        (void)AbortOutputs();
    }
    return result;
}
#endif

HAL_StatusTypeDef CanBus_PitchEnable(void)
{
#if APP_PITCH_ENABLED
    CanFrame frame;
    HAL_StatusTypeDef result;
    if (!Dm3507_MakeEnable(&frame)) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!CanBus_PitchAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = PitchSendResult(SendPitchFrame(&frame));
    taskEXIT_CRITICAL();
    return result;
#else
    return HAL_ERROR;
#endif
}

HAL_StatusTypeDef CanBus_PitchTorque(float torque_nm)
{
#if APP_PITCH_ENABLED
    CanFrame frame;
    HAL_StatusTypeDef result;
    /* 构造器拒绝NaN/Inf；只在本机DM协议±10Nm端点饱和，无额外业务限幅。 */
    if (!Dm3507_MakeTorque(torque_nm, &frame)) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!CanBus_PitchAllowed()) { taskEXIT_CRITICAL(); return HAL_ERROR; }
    result = PitchSendResult(SendPitchFrame(&frame));
    taskEXIT_CRITICAL();
    return result;
#else
    (void)torque_nm;
    return HAL_ERROR;
#endif
}

HAL_StatusTypeDef CanBus_PitchDisable(uint8_t cancel_old)
{
#if APP_PITCH_ENABLED
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
#else
    (void)cancel_old;
    return HAL_ERROR;
#endif
}

HAL_StatusTypeDef CanBus_PitchReadFeedback(void)
{
#if APP_PITCH_ENABLED
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
#else
    return HAL_ERROR;
#endif
}

uint32_t CanBus_PitchTxPending(void)
{
#if APP_PITCH_ENABLED
    uint32_t pending;
    taskENTER_CRITICAL();
    pending = PendingOwned(0U, TX_OWNER_PITCH);
    taskEXIT_CRITICAL();
    return pending;
#else
    return 0U;
#endif
}

HAL_StatusTypeDef CanBus_CancelPitchTx(void)
{
#if APP_PITCH_ENABLED
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    result = CancelOwned(0U, TX_OWNER_PITCH);
    if (result == HAL_OK && PendingOwned(0U, TX_OWNER_PITCH)) { result = HAL_BUSY; }
    taskEXIT_CRITICAL();
    return result;
#else
    return HAL_ERROR;
#endif
}

void CanBus_SetFeederPermission(uint8_t allowed)
{
    taskENTER_CRITICAL();
    feeder_permission = allowed && APP_FEEDER_ENABLED && state.started && !HasCanFault();
#if APP_FEEDER_ENABLED
    if (!feeder_permission) { (void)CancelOwned(1U, TX_OWNER_FEEDER); }
#endif
    taskEXIT_CRITICAL();
}

uint8_t CanBus_FeederAllowed(void)
{ return feeder_permission && state.started && !HasCanFault(); }

#if APP_FEEDER_ENABLED
static HAL_StatusTypeDef SendFeederCurrent(int16_t current)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint16_t raw = (uint16_t)current;
    header.IDE = CAN_ID_STD; header.StdId = 0x1FFU;
    header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    data[0] = (uint8_t)(raw >> 8); data[1] = (uint8_t)raw;
    return SubmitOwned(&hcan2, &header, data, TX_OWNER_FEEDER);
}
#endif

HAL_StatusTypeDef CanBus_FeederCurrent(int16_t raw_current)
{
#if APP_FEEDER_ENABLED
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
#else
    (void)raw_current; return HAL_ERROR;
#endif
}

HAL_StatusTypeDef CanBus_StopFeeder(void)
{
    HAL_StatusTypeDef result = HAL_ERROR;
    taskENTER_CRITICAL();
    feeder_permission = 0U;
#if APP_FEEDER_ENABLED
    if (state.started) {
        result = CancelOwned(1U, TX_OWNER_FEEDER);
        if (result == HAL_OK && PendingOwned(1U, TX_OWNER_FEEDER)) { result = HAL_BUSY; }
        if (result == HAL_OK) { result = SendFeederCurrent(0); }
        if (result == HAL_ERROR) { RecordError(1U, CAN_BUS_SOURCE_TX, &hcan2); (void)AbortOutputs(); }
    }
#endif
    taskEXIT_CRITICAL(); return result;
}

HAL_StatusTypeDef CanBus_Send(const CanFrame *frame)
{
    CAN_TxHeaderTypeDef header = {0};
    CAN_HandleTypeDef *handle;
    uint32_t mailbox;
    HAL_StatusTypeDef result;
    if (!CanFrame_IsValid(frame)) { return HAL_ERROR; }
    taskENTER_CRITICAL();
    if (!output_permission || !APP_MOTOR_DRIVERS_IMPLEMENTED || !state.started || HasCanFault()) {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
    handle = frame->bus == 1U ? &hcan1 : &hcan2;
    header.IDE = frame->extended ? CAN_ID_EXT : CAN_ID_STD;
    header.ExtId = frame->extended ? frame->id : 0U;
    header.StdId = frame->extended ? 0U : frame->id;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame->dlc;
    header.TransmitGlobalTime = DISABLE;
#if APP_PITCH_ENABLED
    (void)mailbox;
    result = SubmitOwned(handle, &header, frame->data, TX_OWNER_GENERIC);
#else
    result = HAL_CAN_AddTxMessage(handle, &header, frame->data, &mailbox);
#endif
    taskEXIT_CRITICAL();
    return result; /* HAL_OK仅代表已提交，tx_frames由完成中断增加。 */
}

/* 调用方持有任务临界区。协议固定为VESC SET_CURRENT=1，
 * int32电流*1000，即mA大端；+0.2A=00 00 00 C8，-0.2A=FF FF FF 38。 */
static HAL_StatusTypeDef SendJogCurrent(uint8_t node, int32_t milliamps)
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
#if APP_PITCH_ENABLED
    (void)mailbox;
    return SubmitOwned(&hcan2, &header, data, TX_OWNER_FLYWHEEL);
#else
    return HAL_CAN_AddTxMessage(&hcan2, &header, data, &mailbox);
#endif
}

static void RecordJogTxFailure(void)
{
    RecordError(1U, CAN_BUS_SOURCE_TX, &hcan2);
}

HAL_StatusTypeDef CanBus_JogVescCurrent(uint8_t node, int32_t milliamps)
{
    HAL_StatusTypeDef result;
    int32_t limit;
    if (!APP_FLYWHEEL_JOG_ENABLED || (node != 70U && node != 71U)) { return HAL_ERROR; }
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

HAL_StatusTypeDef CanBus_StopFlywheels(void)
{
    HAL_StatusTypeDef result, first, second;
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    taskENTER_CRITICAL();
    result = AbortFlywheelOutputs();
    /* bxCAN取消是异步操作。旧请求未清完时不复用邮箱，交由上层下周期重试。 */
    if (!APP_FLYWHEEL_JOG_ENABLED || result != HAL_OK ||
#if APP_PITCH_ENABLED
        PendingOwned(1U, TX_OWNER_FLYWHEEL)) {
        (void)mailboxes;
#else
        (!APP_YAW_ENABLED && HAL_CAN_IsTxMessagePending(&hcan1, mailboxes)) || HAL_CAN_IsTxMessagePending(&hcan2, mailboxes)) {
#endif
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
#if APP_PITCH_ENABLED
    /* 双零需要两个空邮箱；保留诊断查询，不能半途拥塞并误记为CAN故障。 */
    if (!HasTwoFreeMailboxes(&hcan2)) { taskEXIT_CRITICAL(); return HAL_BUSY; }
#endif
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
{
    HAL_StatusTypeDef first, second, result;
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    taskENTER_CRITICAL();
    if (!APP_FLYWHEEL_JOG_ENABLED || !state.started || output_permission) {
        taskEXIT_CRITICAL();
        return HAL_ERROR;
    }
    /* 首次STOP已负责取消旧动力请求。刷新只等待邮箱空闲，保留正常读查询。 */
    if (
#if !APP_PITCH_ENABLED
        (!APP_YAW_ENABLED && HAL_CAN_IsTxMessagePending(&hcan1, mailboxes)) ||
#endif
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

/* 仅这两个公开接口能进入只读发送路径；其构造器拒绝写入、使能、
 * 设零、检测电机以及转速/电流命令。点动不能扩展读白名单。 */
static HAL_StatusTypeDef SendRead(const CanFrame *frame)
{
    CAN_TxHeaderTypeDef header = {0};
    CAN_HandleTypeDef *handle;
    HAL_StatusTypeDef result;
    uint32_t mailbox;
    if (!CanFrame_IsValid(frame)) { return HAL_ERROR; }
    header.IDE = frame->extended ? CAN_ID_EXT : CAN_ID_STD;
    header.ExtId = frame->extended ? frame->id : 0U;
    header.StdId = frame->extended ? 0U : frame->id;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame->dlc;
    header.TransmitGlobalTime = DISABLE;
    handle = frame->bus == 1U ? &hcan1 : &hcan2;
    taskENTER_CRITICAL();
#if APP_PITCH_ENABLED
    (void)mailbox;
    result = state.started ? SubmitOwned(handle, &header, frame->data, TX_OWNER_DIAGNOSTIC) : HAL_ERROR;
#else
    result = state.started ? HAL_CAN_AddTxMessage(handle, &header, frame->data, &mailbox) : HAL_ERROR;
#endif
    taskEXIT_CRITICAL();
    return result;
}

HAL_StatusTypeDef CanBus_ReadDm(uint8_t rid)
{
    CanFrame frame;
    if (!DmDiag_BuildRead(rid, &frame)) { return HAL_ERROR; }
    return SendRead(&frame);
}

HAL_StatusTypeDef CanBus_ReadVesc(uint8_t node, uint8_t command)
{
    CanFrame frame;
    if (!VescDiag_BuildRead(node, command, &frame)) { return HAL_ERROR; }
    return SendRead(&frame);
}

void CanBus_CancelReads(void)
{
    taskENTER_CRITICAL();
#if APP_PITCH_ENABLED
    (void)CancelOwned(0U, TX_OWNER_DIAGNOSTIC);
    (void)CancelOwned(1U, TX_OWNER_DIAGNOSTIC);
#else
    (void)AbortOutputs();
#endif
    taskEXIT_CRITICAL();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
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
        frame.received_ms = HAL_GetTick();
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

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    int32_t index = BusIndex(hcan);
    if (index < 0) { return; }
    RecordError((uint32_t)index, CAN_BUS_SOURCE_IRQ, hcan);
    (void)AbortOutputs();
}

static void TxComplete(CAN_HandleTypeDef *hcan, uint32_t mailbox)
{
    int32_t index = BusIndex(hcan);
    if (index >= 0) {
        state.tx_frames[index]++;
#if APP_PITCH_ENABLED
        /* 完成IRQ可能在临界区结束后才到达；旧邮箱已复用时保留新归属。 */
        if (!HAL_CAN_IsTxMessagePending(hcan, 1U << mailbox)) {
            tx_owner[index][mailbox] = TX_OWNER_NONE;
        }
#else
        (void)mailbox;
#endif
    }
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { TxComplete(hcan, 0U); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { TxComplete(hcan, 1U); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { TxComplete(hcan, 2U); }
#if APP_PITCH_ENABLED
static void TxAbortComplete(CAN_HandleTypeDef *hcan, uint32_t mailbox)
{
    int32_t index = BusIndex(hcan);
    if (index >= 0 && !HAL_CAN_IsTxMessagePending(hcan, 1U << mailbox)) {
        tx_owner[index][mailbox] = TX_OWNER_NONE;
    }
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) { TxAbortComplete(hcan, 0U); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) { TxAbortComplete(hcan, 1U); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) { TxAbortComplete(hcan, 2U); }
#endif
