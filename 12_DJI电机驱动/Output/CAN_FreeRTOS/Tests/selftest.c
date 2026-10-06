/* 编译真实 App 源码，在 ARM 指令模拟器中运行；HAL/队列由本文件替代。
 * 不访问真实串口、ST-Link 或电机，不能代替板上时序和电气验收。 */
#include "gm6020.h"
#include "motor_control.h"
#include "motor_console.h"
#include "can_transport.h"
#include "can_debug.h"
#include "queue.h"
#include "usart.h"
#include <string.h>

volatile uint32_t test_checks;
volatile uint32_t test_failures;
volatile uint32_t test_failed_line;
volatile uint32_t test_stage;
volatile uint32_t test_done;
static uint32_t now_ms;
static uint32_t critical_depth;
static uint32_t advance_tick_on_snapshot;
static uint32_t free_mailboxes = 3U;
static uint32_t send_count;
static uint32_t abort_count;
static HAL_StatusTypeDef send_status = HAL_OK;
static uint8_t *rx_destination;
static char transmitted[128];

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart1;
volatile uint32_t can_ready = 1U;
volatile uint32_t can1_tx_busy_count;

static struct
{
    uint8_t data[128][16];
    uint32_t size, head, tail, count;
} fake_queue;

#define CHECK(x) do { test_checks++; if (!(x)) { test_failures++; if (test_failed_line == 0U) test_failed_line = __LINE__; } } while (0)

uint32_t HAL_GetTick(void) { return now_ms; }
void Test_EnterCritical(void)
{
    /* 模拟读取反馈之前发生抢占：接收时间推进一毫秒。 */
    if (advance_tick_on_snapshot != 0U)
    {
        advance_tick_on_snapshot = 0U;
        now_ms++;
        gm6020_feedback.last_rx_ms = now_ms;
    }
    critical_depth++;
}
void Test_ExitCritical(void) { CHECK(critical_depth != 0U); critical_depth--; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{ (void)h; return free_mailboxes; }
HAL_StatusTypeDef CanTransport_Send(uint32_t id, const uint8_t data[8])
{ (void)id; (void)data; if (send_status == HAL_OK) send_count++; return send_status; }
void CanTransport_AbortPending(void) { abort_count++; free_mailboxes = 3U; }
void CanFail(uint32_t code) { (void)code; CHECK(0); }
QueueHandle_t xQueueCreate(uint32_t count, uint32_t size)
{
    CHECK(count <= 128U && size <= 16U);
    memset(&fake_queue, 0, sizeof(fake_queue));
    fake_queue.size = size;
    return &fake_queue;
}
void vQueueAddToRegistry(QueueHandle_t q, const char *name)
{ (void)q; (void)name; }
BaseType_t xQueueReceive(QueueHandle_t q, void *data, TickType_t wait)
{
    (void)q; (void)wait;
    if (fake_queue.count == 0U) return 0;
    memcpy(data, fake_queue.data[fake_queue.head], fake_queue.size);
    fake_queue.head = (fake_queue.head + 1U) % 128U;
    fake_queue.count--;
    return pdPASS;
}
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *data, BaseType_t *woken)
{
    (void)q; (void)woken;
    if (fake_queue.count == 128U) return 0;
    memcpy(fake_queue.data[fake_queue.tail], data, fake_queue.size);
    fake_queue.tail = (fake_queue.tail + 1U) % 128U;
    fake_queue.count++;
    return pdPASS;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t length)
{ (void)h; CHECK(length == 1U); rx_destination = data; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{ (void)h; rx_destination = 0; return HAL_OK; }
uint32_t HAL_UART_GetError(UART_HandleTypeDef *h) { (void)h; return 7U; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data, uint16_t length, uint32_t timeout)
{
    (void)h; (void)timeout;
    CHECK(length < sizeof(transmitted));
    memcpy(transmitted, data, length);
    transmitted[length] = 0;
    return HAL_OK;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);

static CanRxMessage FeedbackMessage(void)
{
    CanRxMessage m = {0};
    m.source = 1U;
    m.id = GM6020_FEEDBACK_ID;
    m.dlc = 8U;
    m.received_ms = now_ms;
    m.data[0] = 0x10U; m.data[1] = 0;
    m.data[2] = 0xFFU; m.data[3] = 0x9CU;
    m.data[4] = 0x80U; m.data[5] = 0;
    m.data[6] = 35U; m.data[7] = 0xFEU;
    return m;
}

static void FreshFeedback(uint8_t temperature)
{
    CanRxMessage m = FeedbackMessage();
    m.data[6] = temperature;
    CHECK(GM6020_ParseFeedback(&m) == 1U);
}
static void ResetMotor(void)
{
    now_ms = 1000U;
    free_mailboxes = 3U;
    send_status = HAL_OK;
    can_ready = 1U;
    MotorControl_Stop(MOTOR_STOP_USER);
    FreshFeedback(35U);
}
static const char *ModeCommand(void)
{
    return GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE ? "V500" : "C500";
}
static void InjectText(const char *text)
{
    while (*text != 0)
    {
        CHECK(rx_destination != 0);
        *rx_destination = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(&huart1);
    }
}

void test_run(void)
{
    CanRxMessage m;
    GM6020_Feedback f;
    uint32_t id, previous;
    uint8_t bytes[8];
    MotorControl_State state;

    test_stage = 1U;
    CHECK(motor_control.enabled == 0U && motor_control.output_raw == 0);
    CHECK(motor_control.stop_reason == MOTOR_STOP_POWER_ON);
    m = FeedbackMessage();
    CHECK(GM6020_DecodeFeedback(&m, &f) == 1U);
    CHECK(f.encoder == 4096U && f.angle_deg == 180.0f);
    CHECK(f.speed_rpm == -100 && f.current_raw == -32768);
    CHECK(f.temperature == 35U);
    f.encoder = 777U;
    m.dlc = 7U; CHECK(GM6020_DecodeFeedback(&m, &f) == 0U && f.encoder == 777U);
    m.dlc = 8U; m.source = 2U; CHECK(GM6020_DecodeFeedback(&m, &f) == 0U);
    m.source = 1U; m.id++; CHECK(GM6020_DecodeFeedback(&m, &f) == 0U);
    m.id = GM6020_FEEDBACK_ID; m.data[0] = 0x20U;
    CHECK(GM6020_DecodeFeedback(&m, &f) == 0U);
    m.data[0] = 0x1FU; m.data[1] = 0xFFU; m.data[2] = 0x7FU; m.data[3] = 0xFFU;
    CHECK(GM6020_DecodeFeedback(&m, &f) == 1U);
    CHECK(f.angle_deg > 359.9f && f.angle_deg < 360.0f && f.speed_rpm == 32767);
    CHECK(GM6020_DecodeFeedback(0, &f) == 0U);
    CHECK(GM6020_DecodeFeedback(&m, 0) == 0U);
    now_ms = 1500U; m.received_ms = 1400U;
    CHECK(GM6020_ParseFeedback(&m) == 1U);
    GM6020_GetFeedback(&f);
    CHECK(f.last_rx_ms == 1400U); /* 排队延迟不能伪装为新反馈。 */
    CHECK(GM6020_IsOnline(&f, 1599U) == 1U);
    CHECK(GM6020_IsOnline(&f, 1600U) == 0U);
    f.last_rx_ms = 0xFFFFFFF0U;
    CHECK(GM6020_IsOnline(&f, 0x20U) == 1U);
    CHECK(GM6020_IsOnline(&f, 0xC0U) == 0U);

    test_stage = 2U;
    for (uint8_t mode = 0U; mode < 2U; mode++)
    {
        for (uint8_t motor = 1U; motor <= 7U; motor++)
        {
            uint8_t offset = (uint8_t)(((motor <= 4U) ? motor - 1U : motor - 5U) * 2U);
            CHECK(GM6020_BuildCommand(motor, mode, -500, &id, bytes) == 1U);
            CHECK(id == ((motor <= 4U) ? (mode == 0U ? 0x1FFU : 0x1FEU) :
                        (mode == 0U ? 0x2FFU : 0x2FEU)));
            for (uint8_t i = 0U; i < 8U; i++)
                CHECK(bytes[i] == (i == offset ? 0xFEU : (i == offset + 1U ? 0x0CU : 0U)));
        }
    }
    CHECK(GM6020_BuildCommand(1U, 0U, 25000, &id, bytes) == 1U);
    CHECK(GM6020_BuildCommand(1U, 0U, -25000, &id, bytes) == 1U);
    CHECK(GM6020_BuildCommand(1U, 0U, 25001, &id, bytes) == 0U);
    CHECK(GM6020_BuildCommand(7U, 1U, 16384, &id, bytes) == 1U);
    CHECK(GM6020_BuildCommand(7U, 1U, -16384, &id, bytes) == 1U);
    CHECK(GM6020_BuildCommand(7U, 1U, 16385, &id, bytes) == 0U);
    CHECK(GM6020_BuildCommand(0U, 0U, 0, &id, bytes) == 0U);
    CHECK(GM6020_BuildCommand(8U, 0U, 0, &id, bytes) == 0U);
    CHECK(GM6020_BuildCommand(1U, 2U, 0, &id, bytes) == 0U);
    CHECK(GM6020_BuildCommand(1U, 0U, 0, 0, bytes) == 0U);

    test_stage = 3U;
    ResetMotor();
    advance_tick_on_snapshot = 1U;
    CHECK(MotorControl_SetOutput(500) == 1U);
    advance_tick_on_snapshot = 1U;
    MotorControl_Step();
    CHECK(motor_control.enabled == 1U && motor_control.last_sent_raw == 500);
    now_ms = motor_control.last_command_ms + 2999U; FreshFeedback(35U);
    CHECK(MotorControl_KeepAlive() == 1U);
    now_ms += 3000U; FreshFeedback(35U);
    CHECK(MotorControl_KeepAlive() == 0U);
    MotorControl_Step();
    CHECK(motor_control.enabled == 0U && motor_control.output_raw == 0);
    CHECK(motor_control.stop_reason == MOTOR_STOP_COMMAND_TIMEOUT);
    FreshFeedback(35U); MotorControl_Step();
    CHECK(motor_control.enabled == 0U); /* 不自动重启。 */
    CHECK(MotorControl_SetOutput(500) == 1U);
    now_ms += 200U;
    MotorControl_Step();
    CHECK(motor_control.stop_reason == MOTOR_STOP_FEEDBACK && motor_control.output_raw == 0);
    ResetMotor(); CHECK(MotorControl_SetOutput(500) == 1U);
    FreshFeedback(80U); MotorControl_Step();
    CHECK(motor_control.stop_reason == MOTOR_STOP_TEMPERATURE && motor_control.enabled == 0U);
    CHECK(MotorControl_SetOutput(500) == 0U);
    ResetMotor(); CHECK(MotorControl_SetOutput(500) == 1U);
    free_mailboxes = 2U; previous = abort_count; MotorControl_Step();
    CHECK(motor_control.stop_reason == MOTOR_STOP_CAN_TX && motor_control.enabled == 0U);
    CHECK(abort_count == previous + 1U);
    MotorControl_Step(); CHECK(motor_control.last_sent_raw == 0);
    ResetMotor(); CHECK(MotorControl_SetOutput(500) == 1U);
    send_status = HAL_ERROR; MotorControl_Step();
    CHECK(motor_control.stop_reason == MOTOR_STOP_CAN_TX && motor_control.output_raw == 0);
    ResetMotor(); now_ms = 0xFFFFFFF0U; FreshFeedback(35U);
    CHECK(MotorControl_SetOutput(500) == 1U);
    now_ms = 0x10U; FreshFeedback(35U); MotorControl_Step();
    CHECK(motor_control.enabled == 1U);
    MotorControl_GetState(&state); CHECK(state.output_raw == 500);

    test_stage = 4U;
    ResetMotor();
    MotorConsole_ProcessLine(ModeCommand());
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK && motor_control.enabled == 1U);
    MotorConsole_ProcessLine("STOP");
    CHECK(motor_control.enabled == 0U && motor_control.requested_raw == 0);
    MotorConsole_ProcessLine("K");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_NOT_READY);
    MotorConsole_ProcessLine(GM6020_CONTROL_MODE == 0U ? "C500" : "V500");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_MODE);
    MotorConsole_ProcessLine(GM6020_CONTROL_MODE == 0U ? "V2001" : "C1001");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_RANGE);
    MotorConsole_ProcessLine("V99999999999999999999999");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_SYNTAX);
    MotorConsole_ProcessLine("V12abc");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_SYNTAX);
    MotorConsole_ProcessLine(0);
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_SYNTAX && motor_control.enabled == 0U);

    test_stage = 5U;
    MotorConsole_CreateQueue(); MotorConsole_Start();
    InjectText(GM6020_CONTROL_MODE == 0U ? "V50" : "C50");
    MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    previous = motor_console_debug.command_count;
    InjectText("0\r\n"); MotorConsole_Poll();
    CHECK(motor_control.enabled == 1U && motor_control.requested_raw == 500);
    CHECK(motor_console_debug.command_count == previous + 1U);
    MotorControl_Step(); MotorConsole_SendTelemetry();
    CHECK(strcmp(transmitted, "motor:180.00,-100,-32768,35,1,500,1,0,1\r\n") == 0);
    InjectText("STOP\n"); MotorConsole_Poll();
    CHECK(motor_control.enabled == 0U);
    InjectText("VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV\n");
    MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    InjectText(ModeCommand()); InjectText("\n"); now_ms += 1001U;
    MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    InjectText("\n"); MotorConsole_Poll(); ResetMotor();
    InjectText(ModeCommand()); InjectText("\n"); MotorConsole_Poll();
    CHECK(motor_control.enabled == 1U);

    test_stage = 6U;
    for (uint32_t i = 0U; i < 129U; i++) InjectText("X");
    MotorConsole_Poll();
    CHECK(motor_console_debug.rx_drop_count == 1U && motor_control.enabled == 0U);
    InjectText("\n"); MotorConsole_Poll();
    InjectText(ModeCommand()); InjectText("\n"); MotorConsole_Poll();
    CHECK(motor_control.enabled == 1U);
    HAL_UART_ErrorCallback(&huart1); MotorConsole_Poll();
    CHECK(motor_control.enabled == 0U && motor_console_debug.rx_error_count == 1U);
    CHECK(rx_destination != 0);
    InjectText("\n"); MotorConsole_Poll();
    CHECK(motor_control.enabled == 0U);
    CHECK(critical_depth == 0U);
    test_done = 1U;
}
