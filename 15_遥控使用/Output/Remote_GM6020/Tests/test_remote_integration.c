#include "remote_input.h" /* 实际 UART 回调、解析任务、遥控应用；HAL/队列/电机提交为 mock。 */
#include "remote_app.h"
#include "motor_console.h"
#include "motor_control.h"
#include "can_debug.h"
#include "queue.h"
#include "usart.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

volatile uint32_t test_checks;
volatile uint32_t test_failures;
volatile uint32_t test_failed_line;
volatile uint32_t test_done;
volatile uint32_t test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (test_failed_line == 0U) { test_failed_line = __LINE__; } } } while (0)

UART_HandleTypeDef huart1, huart4;
volatile uint32_t can_ready = 1U;
volatile uint32_t can1_error_callback_count;
static uint32_t tick_ms, critical_depth, abort1, abort4, receive1, receive4;
static uint32_t stop_calls, speed_calls, fail_uart4_rearm, fail_queue_send;
static uint8_t *rx_target1, *rx_target4;
static GM6020_Feedback feedback;
static MotorControl_State motor;

typedef struct {
    uint32_t count, size, capacity, head, tail;
    uint8_t bytes[256U * 16U];
} MockQueue;
static MockQueue queues[2];
static uint32_t queue_count;

uint32_t HAL_GetTick(void) { return tick_ms; }
void Test_EnterCritical(void) { critical_depth++; }
void Test_ExitCritical(void) { CHECK(critical_depth != 0U); critical_depth--; }
void CanFail(uint32_t code) { (void)code; CHECK(0); }

QueueHandle_t xQueueCreate(uint32_t count, uint32_t size)
{
    MockQueue *queue;
    CHECK(queue_count < 2U);
    CHECK(count <= 256U && size <= 16U);
    queue = &queues[queue_count++];
    queue->capacity = count; queue->size = size;
    return queue;
}
void vQueueAddToRegistry(QueueHandle_t queue, const char *name) { (void)queue; (void)name; }
BaseType_t xQueueReceive(QueueHandle_t handle, void *data, TickType_t wait)
{
    MockQueue *queue = (MockQueue *)handle;
    (void)wait;
    if (queue->count == 0U) { return pdFALSE; }
    memcpy(data, queue->bytes + queue->head * queue->size, queue->size);
    queue->head = (queue->head + 1U) % queue->capacity;
    queue->count--;
    return pdPASS;
}
BaseType_t xQueueSendFromISR(QueueHandle_t handle, const void *data, BaseType_t *woken)
{
    MockQueue *queue = (MockQueue *)handle;
    *woken = pdFALSE;
    if (fail_queue_send || queue->count == queue->capacity) { return pdFALSE; }
    memcpy(queue->bytes + queue->tail * queue->size, data, queue->size);
    queue->tail = (queue->tail + 1U) % queue->capacity;
    queue->count++;
    return pdPASS;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    CHECK(length == 1U);
    if (uart == &huart4) {
        receive4++;
        if (fail_uart4_rearm) { return HAL_ERROR; }
        rx_target4 = data;
    } else if (uart == &huart1) { receive1++; rx_target1 = data; }
    else { CHECK(0); }
    /* 真实 HAL 的 UART_Start_Receive_IT 会清除 ErrorCode。 */
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    if (uart == &huart4) { abort4++; } else if (uart == &huart1) { abort1++; }
    else { CHECK(0); }
    return HAL_OK;
}
uint32_t HAL_UART_GetError(UART_HandleTypeDef *uart) { return uart->ErrorCode; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t length, uint32_t timeout)
{
    (void)uart; (void)data; (void)length; (void)timeout; return HAL_OK;
}
void GM6020_GetFeedback(GM6020_Feedback *out) { *out = feedback; }
void MotorControl_GetState(MotorControl_State *out) { *out = motor; }
uint8_t MotorControl_SetRemoteSpeed(float rpm)
{
    speed_calls++;
    motor.enabled = 1U; motor.target_rpm = rpm; motor.stop_reason = MOTOR_STOP_NONE;
    return 1U;
}
void MotorControl_Stop(MotorStopReason reason)
{
    stop_calls++;
    motor.enabled = 0U; motor.target_rpm = 0.0f; motor.stop_reason = reason;
}

static void UpdateFeedback(void)
{
    feedback.rx_count++;
    feedback.last_rx_ms = tick_ms;
    feedback.temperature = 25U;
}

static void QueueFrame(uint16_t arm, uint16_t stick)
{
    uint8_t frame[32];
    uint16_t channels[14];
    uint16_t sum = 0U;
    uint16_t checksum;
    uint8_t index;
    for (index = 0U; index < 14U; index++) { channels[index] = 1500U; }
    channels[0] = stick; channels[4] = arm; channels[5] = 2000U;
    frame[0] = 0x20U; frame[1] = 0x40U;
    for (index = 0U; index < 14U; index++) {
        frame[2U + 2U * index] = (uint8_t)channels[index];
        frame[3U + 2U * index] = (uint8_t)(channels[index] >> 8U);
    }
    for (index = 0U; index < 30U; index++) { sum = (uint16_t)(sum + frame[index]); }
    checksum = (uint16_t)(0xFFFFU - sum);
    frame[30] = (uint8_t)checksum; frame[31] = (uint8_t)(checksum >> 8U);
    for (index = 0U; index < 32U; index++) {
        *rx_target4 = frame[index];
        HAL_UART_RxCpltCallback(&huart4); /* 必须经过实际路由回调。 */
    }
}
static void FrameAndStep(uint16_t arm, uint16_t stick)
{
    tick_ms += 7U;
    QueueFrame(arm, stick);
    RemoteInput_Poll();
    UpdateFeedback();
    RemoteApp_Step();
}
static void CheckGuard(uint8_t armed, RemoteControl_Fault fault)
{
    RemoteControl guard;
    RemoteApp_GetState(&guard);
    CHECK(guard.armed == armed);
    CHECK(guard.fault == (uint32_t)fault);
    CHECK(motor.enabled == armed);
    if (!armed) { CHECK(fabsf(motor.target_rpm) < 0.0001f); }
}
static void Rearm(void)
{
    FrameAndStep(2000U, 1500U);
    CHECK(motor.enabled == 0U);
    FrameAndStep(1000U, 1500U);
    CHECK(motor.enabled == 0U);
    FrameAndStep(2000U, 1500U);
    CheckGuard(1U, REMOTE_FAULT_NONE);
}

static void TestActualErrorRouteAndRecovery(void)
{
    RemoteInput_State before, after;
    uint32_t old_abort, old_receive, old_stop;
    UART_HandleTypeDef other_uart;
    RemoteInput_CreateQueue();
    MotorConsole_CreateQueue();
    RemoteInput_Start();
    MotorConsole_Start();
    RemoteApp_Init();
    CHECK(receive4 == 1U && receive1 == 1U);
    CHECK(rx_target4 != NULL && rx_target1 != NULL);
    /* 迁移到 UART4 后，USART1 的正常字节仍只进入上位机命令队列。 */
    RemoteInput_GetState(&before);
    old_receive = receive4;
    *rx_target1 = 0x20U;
    HAL_UART_RxCpltCallback(&huart1);
    RemoteInput_GetState(&after);
    CHECK(motor_console_debug.rx_byte_count == 1U);
    CHECK(receive1 == 2U);
    CHECK(receive4 == old_receive);
    CHECK(after.rx_bytes == before.rx_bytes);
    CHECK(after.valid_frames == before.valid_frames);
    CHECK(after.fault_epoch == before.fault_epoch);
    tick_ms = 100U;
    FrameAndStep(2000U, 1500U);
    CheckGuard(0U, REMOTE_FAULT_POWER_ON);
    FrameAndStep(1000U, 1500U);
    FrameAndStep(2000U, 1500U);
    FrameAndStep(2000U, 2000U);
    CheckGuard(1U, REMOTE_FAULT_NONE);
    CHECK(fabsf(motor.target_rpm - 100.0f) < 0.0001f);
    RemoteInput_GetState(&before);
    CHECK(before.rx_bytes == 128U && before.valid_frames == 4U);
    old_abort = abort4; old_receive = receive4; old_stop = stop_calls;
    tick_ms++;
    huart4.ErrorCode = HAL_UART_ERROR_ORE;
    HAL_UART_ErrorCallback(&huart4);
    RemoteInput_GetState(&after);
    CHECK(after.rx_bytes == before.rx_bytes); /* 错误不能被伪装成完成一个字节。 */
    CHECK(after.valid_frames == before.valid_frames);
    CHECK(after.rx_errors == before.rx_errors + 1U);
    CHECK(after.last_uart_error == 0x08U);
    CHECK(after.fault_epoch == before.fault_epoch + 1U);
    CHECK(abort4 == old_abort + 1U);
    CHECK(receive4 == old_receive); /* 接收重启交给任务完成。 */
    CHECK(motor_console_debug.rx_error_count == 0U);
    UpdateFeedback();
    RemoteApp_Step(); /* 不等解析任务也必须看见 ISR 故障代次并停机。 */
    CheckGuard(0U, REMOTE_FAULT_UART);
    CHECK(stop_calls == old_stop + 1U);
    RemoteInput_Poll();
    RemoteInput_GetState(&after);
    CHECK(after.has_frame == 0U);
    CHECK(after.rx_bytes == before.rx_bytes);
    CHECK(abort4 == old_abort + 2U);
    CHECK(receive4 == old_receive + 1U);
    Rearm(); /* 新合法 ARM 高帧仍保持停止，随后 OFF->ON 才恢复。 */

    RemoteInput_GetState(&before);
    old_abort = abort4;
    HAL_UART_ErrorCallback(&other_uart);
    HAL_UART_RxCpltCallback(&other_uart);
    RemoteInput_GetState(&after);
    CHECK(after.rx_bytes == before.rx_bytes && after.rx_errors == before.rx_errors);
    CHECK(after.fault_epoch == before.fault_epoch && abort4 == old_abort);
}

/* 模拟真实 HAL 同一 IRQ 的顺序：错误字节完成 -> 错误回调。
 * 早重启接收会清错误位；错误字节不得进入解析队列。 */
static void TestRxCompleteBeforeErrorCallback(void)
{
    RemoteInput_State before, after;
    uint32_t old_receive, old_abort;
    RemoteInput_GetState(&before);
    old_receive = receive4; old_abort = abort4;
    tick_ms++;
    *rx_target4 = 0x00U;
    huart4.ErrorCode = HAL_UART_ERROR_FE | HAL_UART_ERROR_NE;
    HAL_UART_RxCpltCallback(&huart4);
    CHECK(huart4.ErrorCode == (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE));
    CHECK(receive4 == old_receive);
    RemoteInput_GetState(&after);
    CHECK(after.rx_bytes == before.rx_bytes + 1U);
    CHECK(after.last_rx_byte == 0x00U);
    CHECK(after.rx_errors == before.rx_errors);
    HAL_UART_ErrorCallback(&huart4);
    RemoteInput_GetState(&after);
    CHECK(after.last_uart_error == (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE));
    CHECK(after.rx_errors == before.rx_errors + 1U);
    CHECK(after.fault_epoch == before.fault_epoch + 1U);
    CHECK(abort4 == old_abort + 1U);
    /* 真实 HAL 的非阻塞 FE/NE 分支在错误回调返回后清除 ErrorCode。 */
    huart4.ErrorCode = HAL_UART_ERROR_NONE;
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_UART);
    RemoteInput_Poll();
    RemoteInput_GetState(&after);
    CHECK(after.valid_frames == before.valid_frames);
    CHECK(after.bad_frames == before.bad_frames);
    CHECK(after.has_frame == 0U);
    CHECK(receive4 == old_receive + 1U);
    CHECK(huart4.ErrorCode == HAL_UART_ERROR_NONE);
    /* 恢复接收和新的合法帧不能覆盖最近硬件错误诊断。 */
    Rearm();
    RemoteInput_GetState(&after);
    CHECK(after.last_uart_error == (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE));
}

static void TestFaultEpochDiscardsQueuedOldFrames(void)
{
    RemoteInput_State before, after;
    tick_ms += 7U;
    QueueFrame(1000U, 1500U); /* 错误前队列里已有 OFF 帧，暂未被任务处理。 */
    RemoteInput_GetState(&before);
    huart4.ErrorCode = HAL_UART_ERROR_FE;
    HAL_UART_ErrorCallback(&huart4);
    UpdateFeedback();
    RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_UART);
    RemoteInput_Poll();
    RemoteInput_GetState(&after);
    CHECK(after.valid_frames == before.valid_frames);
    CHECK(after.has_frame == 0U);
    FrameAndStep(2000U, 1500U);
    CHECK(motor.enabled == 0U); /* 旧 OFF 不得授权错误后的高位帧。 */
    Rearm();
}

static void TestReceiveRearmFailureAndQueueLoss(void)
{
    RemoteInput_State before, after;
    uint32_t old_abort, old_receive;
    RemoteInput_GetState(&before);
    old_abort = abort4; old_receive = receive4;
    fail_uart4_rearm = 1U;
    *rx_target4 = 0x20U;
    HAL_UART_RxCpltCallback(&huart4);
    RemoteInput_GetState(&after);
    CHECK(after.rx_bytes == before.rx_bytes + 1U);
    CHECK(after.fault_epoch == before.fault_epoch + 1U);
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_UART);
    fail_uart4_rearm = 0U;
    RemoteInput_Poll();
    CHECK(abort4 == old_abort + 1U && receive4 == old_receive + 2U);
    Rearm();

    RemoteInput_GetState(&before);
    fail_queue_send = 1U;
    *rx_target4 = 0x20U;
    HAL_UART_RxCpltCallback(&huart4);
    fail_queue_send = 0U;
    RemoteInput_GetState(&after);
    CHECK(after.rx_bytes == before.rx_bytes + 1U);
    CHECK(after.rx_drops == before.rx_drops + 1U);
    CHECK(after.fault_epoch == before.fault_epoch + 1U);
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_UART);
    RemoteInput_Poll();
    Rearm();
}

static void TestConsoleErrorAndStop(void)
{
    RemoteInput_State before, after;
    uint32_t old_abort = abort1;
    RemoteInput_GetState(&before);
    huart1.ErrorCode = HAL_UART_ERROR_ORE;
    HAL_UART_ErrorCallback(&huart1);
    CHECK(abort1 == old_abort + 1U);
    CHECK(motor_console_debug.rx_error_count == 1U);
    CHECK(motor_console_debug.rx_fault_epoch == 1U);
    CHECK(motor_console_debug.last_uart_error == 0x08U);
    RemoteInput_GetState(&after);
    CHECK(after.rx_errors == before.rx_errors && after.fault_epoch == before.fault_epoch);
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_UART);
    Rearm();

    tick_ms += 7U;
    QueueFrame(1000U, 1500U); /* STOP 前排队但序号尚未被 guard 观察的 OFF。 */
    RemoteApp_Stop(REMOTE_FAULT_STOP); /* 同毫秒到达也不算 STOP 后的新动作。 */
    CheckGuard(0U, REMOTE_FAULT_STOP);
    CHECK(motor.stop_reason == MOTOR_STOP_USER);
    RemoteInput_Poll();
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_STOP);
    FrameAndStep(2000U, 1500U);
    CheckGuard(0U, REMOTE_FAULT_STOP); /* 旧 OFF 不能授权新的高位。 */
    Rearm();
    tick_ms += 7U;
    QueueFrame(1000U, 1500U);
    tick_ms++;
    RemoteApp_Stop(REMOTE_FAULT_STOP); /* 明确早于 STOP 的排队帧。 */
    RemoteInput_Poll();
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_STOP);
    FrameAndStep(2000U, 1500U);
    CheckGuard(0U, REMOTE_FAULT_STOP);
    Rearm();
}

static void TestQueueAgeAndDelayedMotorFaults(void)
{
    RemoteInput_State before, after;
    tick_ms += 7U;
    QueueFrame(1000U, 1500U);
    RemoteInput_GetState(&before);
    tick_ms += 100U;
    RemoteInput_Poll();
    RemoteInput_GetState(&after);
    CHECK(after.valid_frames == before.valid_frames); /* 挤在队列里的旧帧不续命。 */
    CHECK(after.received_ms == before.received_ms);
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_LINK);
    Rearm();
    motor.enabled = 0U; motor.stop_reason = MOTOR_STOP_CONTROL_TIMING;
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_CONTROL_TIMING);
    Rearm();
    motor.enabled = 0U; motor.stop_reason = MOTOR_STOP_CAN_TX;
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_CAN_TX);
    Rearm();
    can1_error_callback_count++;
    UpdateFeedback(); RemoteApp_Step();
    CheckGuard(0U, REMOTE_FAULT_CAN_TX);
    Rearm();
}

void test_run(void)
{
    test_stage = 1U; TestActualErrorRouteAndRecovery();
    test_stage = 2U; TestRxCompleteBeforeErrorCallback();
    test_stage = 3U; TestFaultEpochDiscardsQueuedOldFrames();
    test_stage = 4U; TestReceiveRearmFailureAndQueueLoss();
    test_stage = 5U; TestConsoleErrorAndStop();
    test_stage = 6U; TestQueueAgeAndDelayedMotorFaults();
    CHECK(critical_depth == 0U);
    CHECK(speed_calls > 0U && stop_calls > 0U);
    test_done = 1U;
}
