/* 编译真实 App 源码，在 ARM 指令模拟器中运行；HAL/队列由本文件替代。
 * 不访问真实串口、ST-Link 或电机，不能代替板上时序和电气验收。 */
#include "pid.h"
#include "gm6020.h"
#include <float.h>
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
static uint32_t critical_calls_until_request;
static char request_to_inject;
static float injected_value;
static uint32_t free_mailboxes = 3U;
static uint32_t send_count;
static uint32_t abort_count;
static HAL_StatusTypeDef send_status = HAL_OK;
static uint8_t *rx_destination;
static char transmitted[128];
static uint32_t last_can_id;
static uint8_t last_can_data[8];

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
    /* 在控制请求快照与数学计算之后、发布临界区之前插入新请求。
     * 先清触发器，避免 SetSpeed/GetFeedback 的嵌套临界区再次触发。
     */
    if (critical_calls_until_request != 0U)
    {
        critical_calls_until_request--;
        if (critical_calls_until_request == 0U)
        {
            if (request_to_inject == 'S') CHECK(MotorControl_SetSpeed(injected_value) == 1U);
            else if (request_to_inject == 'P') CHECK(MotorControl_SetGain('P', injected_value) == 1U);
            else if (request_to_inject == 'X') MotorControl_Stop(MOTOR_STOP_USER);
            else CHECK(0);
        }
    }
}
void Test_ExitCritical(void) { CHECK(critical_depth != 0U); critical_depth--; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{ (void)h; return free_mailboxes; }
HAL_StatusTypeDef CanTransport_Send(uint32_t id, const uint8_t data[8])
{
    if (send_status == HAL_OK)
    {
        send_count++;
        last_can_id = id;
        memcpy(last_can_data, data, sizeof(last_can_data));
    }
    return send_status;
}
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

static void FreshFeedback(int16_t speed, uint8_t temperature)
{
    CanRxMessage m = FeedbackMessage();
    m.data[2] = (uint8_t)((uint16_t)speed >> 8);
    m.data[3] = (uint8_t)speed;
    m.data[6] = temperature;
    CHECK(GM6020_ParseFeedback(&m) == 1U);
}

static uint8_t Near(float actual, float expected, float tolerance)
{
    float difference = actual - expected;
    return (uint8_t)((difference >= -tolerance) && (difference <= tolerance));
}
static float FloatFromBits(uint32_t bits)
{
    union { uint32_t bits; float value; } number;
    number.bits = bits;
    return number.value;
}
static void ResetMotor(void)
{
    now_ms = 1000U;
    free_mailboxes = 3U;
    send_status = HAL_OK;
    can_ready = 1U;
    critical_calls_until_request = 0U;
    MotorControl_Init();
    CHECK(motor_control.kp == SPEED_PID_DEFAULT_KP);
    CHECK(motor_control.ki == SPEED_PID_DEFAULT_KI);
    CHECK(motor_control.kd == SPEED_PID_DEFAULT_KD);
    /* 旧逻辑测试显式选择简单 P-only fixture，不能把教学初值写死在期望中。 */
    CHECK(MotorControl_SetGain('P', 5.0f) == 1U);
    CHECK(MotorControl_SetGain('I', 0.0f) == 1U);
    CHECK(MotorControl_SetGain('D', 0.0f) == 1U);
    FreshFeedback(0, 35U);
}
static void StepAfter(uint32_t elapsed_ms, int16_t speed, uint8_t temperature)
{
    now_ms += elapsed_ms;
    FreshFeedback(speed, temperature);
    MotorControl_Step();
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
/* 同样的总时间、不同的采样间隔，应有相同积分贡献。 */
static void TestPid(void)
{
    PID_Config config = {2.0f, 0.0f, 0.0f, 100.0f, 50.0f, 0.0f};
    PID_Controller controller = {0};
    PID_Controller previous;
    float nan_value = FloatFromBits(0x7FC00000U);
    float infinity = FloatFromBits(0x7F800000U);
    test_stage = 1U;
    CHECK(PID_Init(&controller, &config) == 1U);
    CHECK(Near(PID_Update(&controller, 30.0f, 20.0f, 0.002f), 20.0f, 0.001f));
    CHECK(controller.p_term == 20.0f && controller.i_term == 0.0f && controller.d_term == 0.0f);
    CHECK(PID_Update(&controller, 100.0f, 0.0f, 0.002f) == 100.0f);
    CHECK(PID_Update(&controller, -100.0f, 0.0f, 0.002f) == -100.0f);
    previous = controller;
    config.kp = -1.0f;
    CHECK(PID_Init(&controller, &config) == 0U);
    CHECK(memcmp(&previous, &controller, sizeof(controller)) == 0);
    config.kp = nan_value; CHECK(PID_Init(&controller, &config) == 0U);
    config.kp = infinity; CHECK(PID_Init(&controller, &config) == 0U);
    config.kp = 2.0f; config.ki = -1.0f;
    CHECK(PID_Init(&controller, &config) == 0U);
    config.ki = 0.0f; config.kd = infinity;
    CHECK(PID_Init(&controller, &config) == 0U);
    config.kd = 0.0f; config.output_limit = nan_value;
    CHECK(PID_Init(&controller, &config) == 0U);
    config.kp = 2.0f; config.output_limit = 0.0f;
    CHECK(PID_Init(&controller, &config) == 0U);
    config.output_limit = 100.0f; config.integral_limit = -1.0f;
    CHECK(PID_Init(&controller, &config) == 0U);
    config.integral_limit = 50.0f; config.derivative_tau_s = -0.1f;
    CHECK(PID_Init(&controller, &config) == 0U);
    CHECK(PID_Init(0, &config) == 0U && PID_Init(&controller, 0) == 0U);
    PID_Reset(0); CHECK(PID_Update(0, 1.0f, 0.0f, 0.002f) == 0.0f);
    config.kp = 0.0f; config.ki = 10.0f; config.derivative_tau_s = 0.0f;
    CHECK(PID_Init(&controller, &config) == 1U);
    for (uint32_t i = 0U; i < 100U; i++) PID_Update(&controller, 2.0f, 0.0f, 0.002f);
    CHECK(Near(controller.integral, 4.0f, 0.001f));
    CHECK(PID_Init(&controller, &config) == 1U);
    for (uint32_t i = 0U; i < 50U; i++) PID_Update(&controller, 2.0f, 0.0f, 0.004f);
    CHECK(Near(controller.integral, 4.0f, 0.001f));
    config.integral_limit = 3.0f;
    CHECK(PID_Init(&controller, &config) == 1U);
    for (uint32_t i = 0U; i < 200U; i++) PID_Update(&controller, 10.0f, 0.0f, 0.002f);
    CHECK(controller.integral == 3.0f);
    for (uint32_t i = 0U; i < 200U; i++) PID_Update(&controller, -10.0f, 0.0f, 0.002f);
    CHECK(controller.integral == -3.0f);
    /* 饱和时拒绝加重饱和的积分，反向误差可以减少旧积分。 */
    config.kp = 10.0f; config.ki = 20.0f; config.integral_limit = 100.0f;
    CHECK(PID_Init(&controller, &config) == 1U);
    for (uint32_t i = 0U; i < 100U; i++) CHECK(PID_Update(&controller, 100.0f, 0.0f, 0.002f) == 100.0f);
    CHECK(controller.integral == 0.0f);
    controller.integral = 50.0f;
    CHECK(Near(PID_Update(&controller, -1.0f, 0.0f, 0.1f), 38.0f, 0.001f));
    CHECK(Near(controller.integral, 48.0f, 0.001f));
    CHECK(PID_Init(&controller, &config) == 1U);
    CHECK(PID_Update(&controller, -100.0f, 0.0f, 0.002f) == -100.0f && controller.integral == 0.0f);
    /* 只改变目标不产生微分冲击，实际值上升产生负微分项。 */
    config.kp = 0.0f; config.ki = 0.0f; config.kd = 2.0f; config.output_limit = 1000.0f;
    CHECK(PID_Init(&controller, &config) == 1U);
    CHECK(PID_Update(&controller, 0.0f, 10.0f, 0.01f) == 0.0f);
    CHECK(PID_Update(&controller, 100.0f, 10.0f, 0.01f) == 0.0f);
    CHECK(Near(PID_Update(&controller, 100.0f, 11.0f, 0.01f), -200.0f, 0.01f));
    config.derivative_tau_s = 0.02f;
    CHECK(PID_Init(&controller, &config) == 1U);
    PID_Update(&controller, 0.0f, 10.0f, 0.01f);
    CHECK(Near(PID_Update(&controller, 0.0f, 11.0f, 0.01f), -66.6667f, 0.01f));
    CHECK(Near(PID_Update(&controller, 0.0f, 11.0f, 0.01f), -44.4444f, 0.01f));
    PID_Reset(&controller);
    CHECK(controller.initialized == 0U && controller.integral == 0.0f);
    CHECK(controller.config.kd == 2.0f && controller.config.derivative_tau_s == 0.02f);
    CHECK(PID_Update(&controller, nan_value, 0.0f, 0.002f) == 0.0f && controller.initialized == 0U);
    CHECK(PID_Update(&controller, 0.0f, infinity, 0.002f) == 0.0f);
    CHECK(PID_Update(&controller, 0.0f, 0.0f, nan_value) == 0.0f);
    CHECK(PID_Update(&controller, 0.0f, 0.0f, 0.0f) == 0.0f);
    CHECK(PID_Update(&controller, 0.0f, 0.0f, -0.002f) == 0.0f);
    CHECK(PID_Update(&controller, FLT_MAX, -FLT_MAX, 0.002f) == 0.0f);
    config.kp = FLT_MAX; config.ki = 0.0f; config.kd = 0.0f;
    CHECK(PID_Init(&controller, &config) == 1U);
    CHECK(PID_Update(&controller, 2.0f, 0.0f, 0.002f) == 0.0f && controller.initialized == 0U);
    config.kp = 0.0f; config.ki = FLT_MAX;
    CHECK(PID_Init(&controller, &config) == 1U);
    CHECK(PID_Update(&controller, 2.0f, 0.0f, 1.0f) == 0.0f);
    config.ki = 0.0f; config.kd = 1.0f;
    CHECK(PID_Init(&controller, &config) == 1U);
    PID_Update(&controller, 0.0f, -FLT_MAX, 1.0f);
    CHECK(PID_Update(&controller, 0.0f, FLT_MAX, 1.0f) == 0.0f && controller.initialized == 0U);

    /* 滑块调参只更新增益，已经累积的积分贡献和微分历史需要连续。 */
    config.kp = 2.0f; config.ki = 10.0f; config.kd = 2.0f;
    config.output_limit = 1000.0f; config.integral_limit = 100.0f;
    config.derivative_tau_s = 0.02f;
    CHECK(PID_Init(&controller, &config) == 1U);
    PID_Update(&controller, 10.0f, 3.0f, 0.01f);
    PID_Update(&controller, 10.0f, 4.0f, 0.01f);
    previous = controller;
    CHECK(PID_SetGains(&controller, 3.0f, 20.0f, 1.0f) == 1U);
    CHECK(controller.config.kp == 3.0f && controller.config.ki == 20.0f && controller.config.kd == 1.0f);
    CHECK(controller.integral == previous.integral && controller.i_term == previous.i_term);
    CHECK(controller.previous_measurement == previous.previous_measurement);
    CHECK(controller.filtered_derivative == previous.filtered_derivative && controller.initialized == previous.initialized);
    CHECK(controller.config.output_limit == previous.config.output_limit);
    CHECK(controller.config.integral_limit == previous.config.integral_limit);
    CHECK(controller.config.derivative_tau_s == previous.config.derivative_tau_s);
    PID_Update(&controller, 10.0f, 4.0f, 0.01f);
    CHECK(Near(controller.integral, previous.integral + 1.2f, 0.001f));
    CHECK(Near(controller.filtered_derivative, previous.filtered_derivative * (2.0f / 3.0f), 0.001f));
    CHECK(Near(controller.d_term, controller.filtered_derivative, 0.001f));
    previous = controller;
    CHECK(PID_SetGains(&controller, 3.0f, 20.0f, 1.0f) == 1U);
    CHECK(memcmp(&controller, &previous, sizeof(controller)) == 0); /* 重复滑块值完全不改变状态。 */
    CHECK(PID_SetGains(&controller, -1.0f, 20.0f, 1.0f) == 0U);
    CHECK(memcmp(&controller, &previous, sizeof(controller)) == 0);
    CHECK(PID_SetGains(&controller, 3.0f, nan_value, 1.0f) == 0U);
    CHECK(memcmp(&controller, &previous, sizeof(controller)) == 0);
    CHECK(PID_SetGains(&controller, 3.0f, 20.0f, infinity) == 0U);
    CHECK(memcmp(&controller, &previous, sizeof(controller)) == 0);
    CHECK(PID_SetGains(0, 3.0f, 20.0f, 1.0f) == 0U);
    CHECK(PID_SetGains(&controller, 3.0f, 0.0f, 1.0f) == 1U);
    CHECK(controller.integral == 0.0f && controller.i_term == 0.0f); /* I0 明确关闭积分贡献。 */
    CHECK(controller.previous_measurement == previous.previous_measurement);
    CHECK(controller.filtered_derivative == previous.filtered_derivative && controller.initialized == previous.initialized);
    /* 方向切换只清 I，不清除用于减噪和避免 D 冲击的测量历史。 */
    CHECK(PID_SetGains(&controller, 3.0f, 20.0f, 1.0f) == 1U);
    PID_Update(&controller, 10.0f, 4.0f, 0.01f);
    previous = controller;
    CHECK(previous.integral > 0.0f);
    PID_ClearIntegral(&controller);
    CHECK(controller.integral == 0.0f && controller.i_term == 0.0f);
    CHECK(controller.previous_measurement == previous.previous_measurement);
    CHECK(controller.filtered_derivative == previous.filtered_derivative);
    CHECK(controller.initialized == previous.initialized);
    CHECK(memcmp(&controller.config, &previous.config, sizeof(controller.config)) == 0);
    PID_ClearIntegral(0);
}
static void TestProtocol(void)
{
    CanRxMessage message;
    GM6020_Feedback feedback;
    uint32_t id;
    uint8_t bytes[8];
    test_stage = 2U;
    message = FeedbackMessage();
    CHECK(GM6020_DecodeFeedback(&message, &feedback) == 1U);
    CHECK(feedback.encoder == 4096U && feedback.angle_deg == 180.0f);
    CHECK(feedback.speed_rpm == -100 && feedback.current_raw == -32768);
    feedback.encoder = 777U; message.dlc = 7U;
    CHECK(GM6020_DecodeFeedback(&message, &feedback) == 0U && feedback.encoder == 777U);
    message.dlc = 8U; message.source = 2U;
    CHECK(GM6020_DecodeFeedback(&message, &feedback) == 0U);
    message.source = 1U; message.id++;
    CHECK(GM6020_DecodeFeedback(&message, &feedback) == 0U);
    message.id = GM6020_FEEDBACK_ID; message.data[0] = 0x20U;
    CHECK(GM6020_DecodeFeedback(&message, &feedback) == 0U);
    CHECK(GM6020_DecodeFeedback(0, &feedback) == 0U);
    CHECK(GM6020_DecodeFeedback(&message, 0) == 0U);
    message = FeedbackMessage(); now_ms = 1500U; message.received_ms = 1400U;
    CHECK(GM6020_ParseFeedback(&message) == 1U);
    GM6020_GetFeedback(&feedback); CHECK(feedback.last_rx_ms == 1400U);
    CHECK(GM6020_IsOnline(&feedback, 1400U + MOTOR_FEEDBACK_TIMEOUT_MS - 1U) == 1U);
    CHECK(GM6020_IsOnline(&feedback, 1400U + MOTOR_FEEDBACK_TIMEOUT_MS) == 0U);
    feedback.last_rx_ms = 0xFFFFFFF0U;
    CHECK(GM6020_IsOnline(&feedback, 0xFFFFFFF0U + MOTOR_FEEDBACK_TIMEOUT_MS - 1U) == 1U);
    CHECK(GM6020_IsOnline(&feedback, 0xFFFFFFF0U + MOTOR_FEEDBACK_TIMEOUT_MS) == 0U);
    for (uint8_t mode = 0U; mode < 2U; mode++)
    {
        for (uint8_t motor = 1U; motor <= 7U; motor++)
        {
            uint8_t offset = (uint8_t)(((motor <= 4U) ? motor - 1U : motor - 5U) * 2U);
            CHECK(GM6020_BuildCommand(motor, mode, -500, &id, bytes) == 1U);
            CHECK(id == ((motor <= 4U) ? (mode == 0U ? 0x1FFU : 0x1FEU) : (mode == 0U ? 0x2FFU : 0x2FEU)));
            for (uint8_t i = 0U; i < 8U; i++) CHECK(bytes[i] == (i == offset ? 0xFEU : (i == offset + 1U ? 0x0CU : 0U)));
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
}
static void TestControl(void)
{
    MotorControl_State state;
    uint32_t previous;
    float previous_i;
    float previous_d;
    test_stage = 3U;
    ResetMotor();
    CHECK(motor_control.enabled == 0U && motor_control.output_raw == 0);
    CHECK(motor_control.stop_reason == MOTOR_STOP_POWER_ON);
    CHECK(motor_control.kp == 5.0f && motor_control.ki == 0.0f && motor_control.kd == 0.0f);
    CHECK(MotorControl_SetSpeed(30.0f) == 1U);
    StepAfter(2U, 0, 35U);
    CHECK(motor_control.enabled == 1U && motor_control.output_raw == 150 && motor_control.last_sent_raw == 150);
    CHECK(motor_control.target_rpm == 30.0f && motor_control.error_rpm == 30.0f);
    uint32_t expected_id;
    uint8_t expected_bytes[8];
    CHECK(GM6020_BuildCommand(GM6020_MOTOR_ID, GM6020_CONTROL_MODE, 150, &expected_id, expected_bytes) == 1U);
    CHECK(last_can_id == expected_id && memcmp(last_can_data, expected_bytes, sizeof(expected_bytes)) == 0);
    StepAfter(2U, 10, 35U);
    CHECK(motor_control.actual_rpm == 10.0f && motor_control.output_raw == 100);
    CHECK(MotorControl_SetSpeed(-30.0f) == 1U);
    StepAfter(2U, 0, 35U); CHECK(motor_control.output_raw == -150);
    CHECK(MotorControl_SetSpeed(0.0f) == 1U);
    StepAfter(2U, 10, 35U);
    CHECK(motor_control.enabled == 1U && motor_control.target_rpm == 0.0f && motor_control.output_raw == -50);
    MotorControl_Stop(MOTOR_STOP_USER); StepAfter(2U, 10, 35U);
    CHECK(motor_control.enabled == 0U && motor_control.output_raw == 0 && motor_control.last_sent_raw == 0);
    CHECK(MotorControl_KeepAlive() == 0U);
    CHECK(MotorControl_SetSpeed(100.1f) == 0U && MotorControl_SetSpeed(-100.1f) == 0U);
    CHECK(MotorControl_SetSpeed(FloatFromBits(0x7FC00000U)) == 0U);
    CHECK(MotorControl_SetSpeed(FloatFromBits(0x7F800000U)) == 0U);
    CHECK(MotorControl_SetGain('P', -1.0f) == 0U && MotorControl_SetGain('P', 200.1f) == 0U);
    CHECK(MotorControl_SetGain('I', 1000.1f) == 0U && MotorControl_SetGain('D', 10.1f) == 0U);
    CHECK(MotorControl_SetGain('X', 1.0f) == 0U);
    CHECK(MotorControl_SetGain('P', FloatFromBits(0x7FC00000U)) == 0U);
    CHECK(MotorControl_SetGain('I', FloatFromBits(0x7F800000U)) == 0U);
    CHECK(MotorControl_SetGain('P', 10.0f) == 1U);
    CHECK(motor_control.enabled == 0U); /* 调参不能从停止状态启动。 */
    CHECK(MotorControl_SetSpeed(30.0f) == 1U);
    StepAfter(2U, 0, 35U); CHECK(motor_control.output_raw == 300);
    CHECK(MotorControl_SetGain('P', SPEED_PID_KP_MAX) == 1U);
    CHECK(MotorControl_SetSpeed(MOTOR_SPEED_LIMIT_RPM) == 1U);
    StepAfter(2U, 0, 35U);
    CHECK(motor_control.output_raw == (GM6020_CONTROL_MODE == 0U ? MOTOR_VOLTAGE_DEMO_LIMIT : MOTOR_CURRENT_DEMO_LIMIT));
    CHECK(MotorControl_SetSpeed(-MOTOR_SPEED_LIMIT_RPM) == 1U);
    StepAfter(2U, 0, 35U);
    CHECK(motor_control.output_raw == -(GM6020_CONTROL_MODE == 0U ? MOTOR_VOLTAGE_DEMO_LIMIT : MOTOR_CURRENT_DEMO_LIMIT));
    ResetMotor();
    CHECK(MotorControl_SetGain('P', 0.0f) == 1U && MotorControl_SetGain('I', 10.0f) == 1U);
    CHECK(MotorControl_SetSpeed(10.0f) == 1U);
    StepAfter(2U, 0, 35U); previous_i = motor_control.i_term;
    CHECK(previous_i > 0.0f);
    MotorControl_Step(); CHECK(motor_control.i_term == previous_i);
    StepAfter(4U, 0, 35U); CHECK(Near(motor_control.i_term - previous_i, 0.4f, 0.001f));
    previous_i = motor_control.i_term;
    CHECK(MotorControl_SetGain('P', 5.0f) == 1U);
    CHECK(motor_control.enabled == 1U);
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, previous_i + 0.2f, 0.001f));
    previous_i = motor_control.i_term;
    CHECK(MotorControl_SetGain('P', 5.0f) == 1U); /* 重复值同样不能清除旧积分。 */
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, previous_i + 0.2f, 0.001f));
    previous_i = motor_control.i_term;
    CHECK(MotorControl_SetGain('I', 20.0f) == 1U);
    StepAfter(4U, 0, 35U); /* 改参不重置上一轮时间，仍按真实 4 ms 积分。 */
    CHECK(Near(motor_control.i_term, previous_i + 0.8f, 0.001f));
    previous_i = motor_control.i_term;
    CHECK(MotorControl_SetSpeed(20.0f) == 1U);
    StepAfter(2U, 0, 35U); CHECK(motor_control.i_term > previous_i);
    CHECK(MotorControl_SetGain('I', 0.0f) == 1U);
    CHECK(motor_control.enabled == 1U);
    StepAfter(2U, 0, 35U); CHECK(motor_control.i_term == 0.0f && motor_control.output_raw == 100);
    /* 改 P 和 D 后继续沿用原实际值与滤波历史，不能让 D 突然从零重建。 */
    ResetMotor();
    CHECK(MotorControl_SetGain('P', 0.0f) == 1U && MotorControl_SetGain('D', 0.1f) == 1U);
    CHECK(MotorControl_SetSpeed(10.0f) == 1U);
    StepAfter(2U, 0, 35U);
    StepAfter(2U, 1, 35U);
    previous_d = motor_control.d_term;
    CHECK(previous_d < -1.0f);
    CHECK(MotorControl_SetGain('P', 1.0f) == 1U);
    StepAfter(2U, 1, 35U);
    CHECK(Near(motor_control.d_term, previous_d * (SPEED_PID_DERIVATIVE_TAU_S /
        (SPEED_PID_DERIVATIVE_TAU_S + 0.002f)), 0.001f));
    previous_d = motor_control.d_term;
    CHECK(MotorControl_SetGain('D', 0.2f) == 1U);
    StepAfter(2U, 1, 35U);
    CHECK(Near(motor_control.d_term, previous_d * 2.0f * (SPEED_PID_DERIVATIVE_TAU_S /
        (SPEED_PID_DERIVATIVE_TAU_S + 0.002f)), 0.001f));
    ResetMotor(); CHECK(MotorControl_SetSpeed(30.0f) == 1U); StepAfter(2U, 0, 35U);
    previous = now_ms;
    /* 连续反馈与控制周期正常时，超过原先三秒也无需串口续期。 */
    for (uint32_t i = 0U; i < 2000U; i++) StepAfter(2U, 0, 35U);
    CHECK((uint32_t)(now_ms - previous) == 4000U);
    CHECK(motor_control.enabled == 1U && motor_control.output_raw == 150);
    CHECK(motor_control.stop_reason == MOTOR_STOP_NONE);
    CHECK(MotorControl_KeepAlive() == 1U);
    StepAfter(2U, 0, 35U); CHECK(motor_control.enabled == 1U && motor_control.output_raw == 150);
    MotorControl_Stop(MOTOR_STOP_USER);
    StepAfter(2U, 0, 35U);
    CHECK(MotorControl_KeepAlive() == 0U && motor_control.enabled == 0U && motor_control.output_raw == 0);
    ResetMotor(); CHECK(MotorControl_SetSpeed(30.0f) == 1U); StepAfter(2U, 0, 35U);
    now_ms += MOTOR_FEEDBACK_TIMEOUT_MS; MotorControl_Step();
    CHECK(motor_control.stop_reason == MOTOR_STOP_FEEDBACK && motor_control.output_raw == 0);
    FreshFeedback(0, 35U); MotorControl_Step(); CHECK(motor_control.enabled == 0U);
    ResetMotor(); CHECK(MotorControl_SetSpeed(30.0f) == 1U); StepAfter(2U, 0, 80U);
    CHECK(motor_control.stop_reason == MOTOR_STOP_TEMPERATURE && motor_control.enabled == 0U);
    CHECK(MotorControl_SetSpeed(30.0f) == 0U);
    ResetMotor(); CHECK(MotorControl_SetSpeed(30.0f) == 1U); StepAfter(2U, 0, 35U);
    free_mailboxes = 2U; previous = abort_count; StepAfter(2U, 0, 35U);
    CHECK(motor_control.stop_reason == MOTOR_STOP_CAN_TX && motor_control.enabled == 0U);
    CHECK(abort_count == previous + 1U);
    StepAfter(2U, 0, 35U); CHECK(motor_control.last_sent_raw == 0);
    ResetMotor(); CHECK(MotorControl_SetSpeed(30.0f) == 1U);
    send_status = HAL_ERROR; StepAfter(2U, 0, 35U);
    CHECK(motor_control.stop_reason == MOTOR_STOP_CAN_TX && motor_control.output_raw == 0);
    ResetMotor(); CHECK(MotorControl_SetSpeed(30.0f) == 1U); StepAfter(2U, 0, 35U);
    StepAfter(12U, 0, 35U);
    CHECK(motor_control.stop_reason == MOTOR_STOP_CONTROL_TIMING && motor_control.enabled == 0U);
    ResetMotor(); now_ms = 0xFFFFFFF0U; FreshFeedback(0, 35U);
    CHECK(MotorControl_SetSpeed(30.0f) == 1U);
    for (uint32_t i = 0U; i < 10U; i++) StepAfter(2U, 0, 35U);
    CHECK(motor_control.enabled == 1U && motor_control.output_raw == 150);
    MotorControl_GetState(&state); CHECK(state.output_raw == 150);
    ResetMotor(); can_ready = 0U;
    CHECK(MotorControl_SetSpeed(30.0f) == 0U && motor_control.enabled == 0U);
    can_ready = 1U;
    /* SetSpeed 和 Step 取快照时被抢占，负反馈年龄不能误判为掉线。 */
    ResetMotor(); advance_tick_on_snapshot = 1U;
    CHECK(MotorControl_SetSpeed(30.0f) == 1U);
    advance_tick_on_snapshot = 1U;
    MotorControl_Step(); CHECK(motor_control.enabled == 1U && motor_control.output_raw == 150);
}
/* 回归实机发现的瓶颈：先前 I 限幅 1000，随后总电压限幅 2000。
 * 固定反馈分别验证积分权限与提高后的总给定边界，不模拟机械响应。
 */
static void TestIntegralAuthority(void)
{
    const float limit = (GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE) ?
        (float)MOTOR_VOLTAGE_DEMO_LIMIT : (float)MOTOR_CURRENT_DEMO_LIMIT;
    PID_Config config = {50.0f, 20.0f, 0.0f, limit,
        SPEED_PID_INTEGRAL_LIMIT, SPEED_PID_DERIVATIVE_TAU_S};
    PID_Controller controller = {0};
    float held_integral;
    float output;
    test_stage = 7U;

    CHECK(config.integral_limit == limit);
    CHECK(PID_Init(&controller, &config) == 1U);
    for (uint32_t i = 0U; i < 800U; i++)
    {
        output = PID_Update(&controller, 20.0f, 12.0f, 0.01f);
        CHECK(output >= 0.0f && output <= limit);
    }
    if (GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE)
    {
        CHECK(controller.integral > 1000.0f);
        CHECK(Near(controller.integral, 1280.0f, 0.1f));
    }
    else
    {
        /* 电流模式仍受 1000 总给定限制：P=400 后 I 约停在 600。 */
        CHECK(controller.integral <= limit - 400.0f);
        CHECK(controller.integral >= limit - 400.0f - 1.7f);
    }
    held_integral = controller.integral;
    for (uint32_t i = 0U; i < 20U; i++)
    {
        output = PID_Update(&controller, 20.0f, 20.0f, 0.01f);
        CHECK(Near(output, held_integral, 0.001f));
        CHECK(controller.p_term == 0.0f && controller.integral == held_integral);
    }
    /* 零误差时仍需维持原积分偏置；它不是只能在非零误差时输出的量。 */

    ResetMotor();
    CHECK(MotorControl_SetGain('P', 50.0f) == 1U);
    CHECK(MotorControl_SetGain('I', 20.0f) == 1U);
    CHECK(MotorControl_SetGain('D', 0.0f) == 1U);
    CHECK(MotorControl_SetSpeed(20.0f) == 1U);
    for (uint32_t i = 0U; i < 3550U; i++)
    {
        StepAfter(2U, 12, 35U); /* 每轮提供新鲜反馈，避免被掉线保护打断。 */
        CHECK(motor_control.output_raw >= 0 && motor_control.output_raw <= (int16_t)limit);
    }
    CHECK(motor_control.enabled == 1U && motor_control.stop_reason == MOTOR_STOP_NONE);
    CHECK(motor_control.p_term == 400.0f);
    if (GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE)
    {
        CHECK(motor_control.i_term > 1000.0f);
        CHECK(Near(motor_control.i_term, 1136.0f, 0.2f));
        CHECK(motor_control.output_raw > 1400);
    }
    else
    {
        CHECK(motor_control.i_term <= limit - 400.0f);
        CHECK(motor_control.i_term >= limit - 400.0f - 0.4f);
    }
    held_integral = motor_control.i_term;
    StepAfter(2U, 20, 35U);
    CHECK(motor_control.p_term == 0.0f && motor_control.i_term == held_integral);
    CHECK(Near((float)motor_control.output_raw, held_integral, 0.5f));
    CHECK(motor_control.last_sent_raw == motor_control.output_raw);

    /* 按当前给定边界安排足够步数，电压扩大到 15000 后仍真正到达饱和区。 */
    CHECK(MotorControl_SetGain('I', 1000.0f) == 1U);
    for (uint32_t i = 0U; i < (uint32_t)(limit / 16.0f) + 100U; i++)
    {
        StepAfter(2U, 12, 35U);
        CHECK(motor_control.output_raw >= 0 && motor_control.output_raw <= (int16_t)limit);
    }
    CHECK(motor_control.i_term <= limit - 400.0f);
    CHECK(motor_control.i_term >= limit - 400.0f - 16.1f);
    if (GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE)
    {
        CHECK(motor_control.output_raw > 2000);
        CHECK(motor_control.i_term > 2000.0f);
    }
    held_integral = motor_control.i_term;
    StepAfter(2U, 21, 35U);
    CHECK(motor_control.i_term < held_integral);
    CHECK(motor_control.enabled == 1U && motor_control.stop_reason == MOTOR_STOP_NONE);
}

static void TestDefaultResponse(void)
{
    const float limit = (GM6020_CONTROL_MODE == GM6020_MODE_VOLTAGE) ?
        (float)MOTOR_VOLTAGE_DEMO_LIMIT : (float)MOTOR_CURRENT_DEMO_LIMIT;
    float expected_p;
    float expected_i;
    float expected_output;
    test_stage = 8U;
    ResetMotor();
    MotorControl_Init(); /* 本例使用真实新默认值，而不是旧测试 fixture。 */
    CHECK(motor_control.enabled == 0U && motor_control.output_raw == 0);
    CHECK(motor_control.kp == SPEED_PID_DEFAULT_KP);
    CHECK(motor_control.ki == SPEED_PID_DEFAULT_KI);
    CHECK(motor_control.kd == SPEED_PID_DEFAULT_KD);
    CHECK(MotorControl_SetSpeed(30.0f) == 1U);
    StepAfter(2U, 0, 35U);
    expected_p = SPEED_PID_DEFAULT_KP * 30.0f;
    expected_i = SPEED_PID_DEFAULT_KI * 30.0f * 0.002f;
    if (expected_p + expected_i > limit) expected_i = 0.0f;
    expected_output = expected_p + expected_i;
    if (expected_output > limit) expected_output = limit;
    CHECK(Near(motor_control.p_term, expected_p, 0.001f));
    CHECK(Near(motor_control.i_term, expected_i, 0.001f));
    CHECK(motor_control.d_term == 0.0f);
    CHECK(Near((float)motor_control.output_raw, expected_output, 0.5f));
    CHECK(motor_control.output_raw > 150); /* 旧默认 P5 的同条件输出为 150。 */
    CHECK(motor_control.last_sent_raw == motor_control.output_raw);
}

static void TestTargetTransitions(void)
{
    float previous_i;
    float previous_d;
    const float alpha = SPEED_PID_DERIVATIVE_TAU_S /
        (SPEED_PID_DERIVATIVE_TAU_S + 0.002f);
    uint32_t previous_send_count;
    test_stage = 9U;
    ResetMotor();
    CHECK(MotorControl_SetGain('P', 20.0f) == 1U); /* 两种协议模式均避开 P 自身饱和。 */
    CHECK(MotorControl_SetGain('I', 200.0f) == 1U);
    CHECK(MotorControl_SetGain('D', 0.002f) == 1U);
    CHECK(MotorControl_SetSpeed(15.0f) == 1U);
    for (uint32_t i = 0U; i < 200U; i++) StepAfter(2U, 10, 35U);
    CHECK(Near(motor_control.i_term, 400.0f, 0.05f));
    StepAfter(2U, 11, 35U);
    previous_d = motor_control.d_term;
    CHECK(previous_d < 0.0f);
    CHECK(MotorControl_SetSpeed(-10.0f) == 1U);
    previous_send_count = send_count;
    previous_i = motor_control.i_term;
    MotorControl_Step(); /* 同一毫秒不消费待清 I，也不重发旧正向给定。 */
    CHECK(send_count == previous_send_count && motor_control.i_term == previous_i);
    StepAfter(2U, 11, 35U);
    CHECK(motor_control.output_raw < 0);
    CHECK(Near(motor_control.i_term, -8.4f, 0.001f));
    CHECK(Near(motor_control.d_term, previous_d * alpha, 0.00001f));
    CHECK(motor_control.last_sent_raw == motor_control.output_raw);

    /* 负向历史与正向切换对称，且不带入旧负积分。 */
    for (uint32_t i = 0U; i < 100U; i++) StepAfter(2U, -5, 35U);
    CHECK(motor_control.i_term < -100.0f);
    CHECK(MotorControl_SetSpeed(15.0f) == 1U);
    StepAfter(2U, -5, 35U);
    CHECK(Near(motor_control.i_term, 8.0f, 0.001f));
    CHECK(motor_control.output_raw > 0);

    /* 同向增速、减速和重复相同目标都保留已有积分贡献。 */
    CHECK(MotorControl_SetGain('D', 0.0f) == 1U);
    for (uint32_t i = 0U; i < 50U; i++) StepAfter(2U, 10, 35U);
    previous_i = motor_control.i_term;
    CHECK(MotorControl_SetSpeed(20.0f) == 1U);
    StepAfter(2U, 10, 35U);
    CHECK(Near(motor_control.i_term, previous_i + 4.0f, 0.01f));
    previous_i = motor_control.i_term;
    CHECK(MotorControl_SetSpeed(12.0f) == 1U);
    StepAfter(2U, 10, 35U);
    CHECK(Near(motor_control.i_term, previous_i + 0.8f, 0.01f));
    previous_i = motor_control.i_term;
    CHECK(MotorControl_SetSpeed(12.0f) == 1U);
    StepAfter(2U, 10, 35U);
    CHECK(Near(motor_control.i_term, previous_i + 0.8f, 0.01f));

    /* 首次 S0 清掉原运行偏置后制动；重复 S0 保留新形成的制动积分。 */
    CHECK(MotorControl_SetSpeed(0.0f) == 1U);
    StepAfter(2U, 10, 35U);
    CHECK(motor_control.enabled == 1U && motor_control.output_raw < 0);
    CHECK(Near(motor_control.i_term, -4.0f, 0.001f));
    CHECK(MotorControl_SetSpeed(0.0f) == 1U);
    StepAfter(2U, 10, 35U);
    CHECK(Near(motor_control.i_term, -8.0f, 0.001f));
    MotorControl_Stop(MOTOR_STOP_USER);
    MotorControl_Step(); /* STOP 即使发生在同一毫秒也要发送零，不能被 dt 门槛吞掉。 */
    CHECK(motor_control.enabled == 0U && motor_control.last_sent_raw == 0);
    CHECK(motor_control.i_term == 0.0f);
}

static void InjectAtPublish(char request, float value)
{
    request_to_inject = request;
    injected_value = value;
    /* Step：反馈快照、请求快照、最后的结果发布。 */
    critical_calls_until_request = 3U;
    MotorControl_Step();
    CHECK(critical_calls_until_request == 0U);
}

static void TestConcurrentRequests(void)
{
    float previous_i;
    uint32_t previous_send_count;
    test_stage = 10U;
    ResetMotor();
    CHECK(MotorControl_SetGain('P', 5.0f) == 1U);
    CHECK(MotorControl_SetGain('I', 20.0f) == 1U);
    CHECK(MotorControl_SetSpeed(10.0f) == 1U);
    for (uint32_t i = 0U; i < 50U; i++) StepAfter(2U, 0, 35U);
    previous_i = motor_control.i_term;
    now_ms += 2U; FreshFeedback(0, 35U);
    InjectAtPublish('S', 20.0f);
    CHECK(motor_control.enabled == 1U && motor_control.target_rpm == 20.0f);
    CHECK(motor_control.output_raw == 0 && motor_control.last_sent_raw == 0);
    CHECK(motor_control.i_term == previous_i);
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, previous_i + 1.6f, 0.001f)); /* 整段真实 4 ms。 */

    previous_i = motor_control.i_term;
    now_ms += 2U; FreshFeedback(0, 35U);
    InjectAtPublish('P', 10.0f);
    CHECK(motor_control.output_raw == 0 && motor_control.i_term == previous_i);
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, previous_i + 1.6f, 0.001f));
    CHECK(motor_control.p_term == 200.0f);

    /* 重复当前目标不会改变 revision，因此不会丢掉本轮输出或积分。 */
    previous_i = motor_control.i_term;
    now_ms += 2U; FreshFeedback(0, 35U);
    InjectAtPublish('S', 20.0f);
    CHECK(motor_control.output_raw > 0);
    CHECK(Near(motor_control.i_term, previous_i + 0.8f, 0.001f));

    /* 新反转请求必须保留清 I 标记；当前候选的正积分不能提交。 */
    now_ms += 2U; FreshFeedback(0, 35U);
    InjectAtPublish('S', -10.0f);
    CHECK(motor_control.target_rpm == -10.0f && motor_control.last_sent_raw == 0);
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, -0.8f, 0.001f));
    CHECK(motor_control.output_raw < 0);

    /* 本轮已经消费清 I，但参数在发布前变化，下一轮仍须重新执行清 I。 */
    CHECK(MotorControl_SetSpeed(10.0f) == 1U);
    now_ms += 2U; FreshFeedback(0, 35U);
    InjectAtPublish('P', 5.0f);
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, 0.8f, 0.001f));
    CHECK(motor_control.output_raw > 0);

    /* STOP 的请求与重建标记不能被正在运行的旧候选覆盖。 */
    now_ms += 2U; FreshFeedback(0, 35U);
    InjectAtPublish('X', 0.0f);
    CHECK(motor_control.enabled == 0U && motor_control.last_sent_raw == 0);
    CHECK(motor_control.stop_reason == MOTOR_STOP_USER);
    previous_send_count = send_count;
    MotorControl_Step();
    CHECK(send_count == previous_send_count + 1U);
    CHECK(motor_control.enabled == 0U && motor_control.i_term == 0.0f);
    CHECK(MotorControl_SetSpeed(10.0f) == 1U);
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, 0.4f, 0.001f));

    /* 启动首轮的 rebuild 也要在并发修改参数时重排，避免使用上次运行的 I。 */
    MotorControl_Stop(MOTOR_STOP_USER);
    CHECK(MotorControl_SetSpeed(10.0f) == 1U);
    now_ms += 2U; FreshFeedback(0, 35U);
    InjectAtPublish('P', 10.0f);
    StepAfter(2U, 0, 35U);
    CHECK(Near(motor_control.i_term, 0.4f, 0.001f));
}

static void TestConsole(void)
{
    uint32_t previous, comma_count;
    const char *text;
    const char *bad_lines[] = {"", "S", "S.", "S1.2.3", "S1e2", "Snan", "Sinf", "S12abc", "S 30", " S30", "S30 ", "V500", "C500", "s30", "T2", "T1.0", "P99999999999999999999999"};
    test_stage = 4U;
    ResetMotor();
    MotorConsole_ProcessLine("S30");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK && motor_control.enabled == 1U);
    CHECK(motor_control.target_rpm == 30.0f);
    MotorConsole_ProcessLine("STOP"); CHECK(motor_control.enabled == 0U);
    MotorConsole_ProcessLine("K"); CHECK(motor_console_debug.last_result == MOTOR_COMMAND_NOT_READY);
    MotorConsole_ProcessLine("S-30.5");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK && motor_control.target_rpm == -30.5f);
    MotorConsole_ProcessLine("S+.5");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK && motor_control.target_rpm == 0.5f);
    MotorConsole_ProcessLine("S0"); CHECK(motor_control.enabled == 1U && motor_control.target_rpm == 0.0f);
    MotorConsole_ProcessLine("STOP");
    MotorConsole_ProcessLine("P5.5"); CHECK(motor_control.kp == 5.5f && motor_control.enabled == 0U);
    MotorConsole_ProcessLine("I0.25"); CHECK(motor_control.ki == 0.25f);
    MotorConsole_ProcessLine("D0.02"); CHECK(Near(motor_control.kd, 0.02f, 0.00001f));
    MotorConsole_ProcessLine("S100.01"); CHECK(motor_console_debug.last_result == MOTOR_COMMAND_RANGE);
    MotorConsole_ProcessLine("P200.01"); CHECK(motor_console_debug.last_result == MOTOR_COMMAND_RANGE);
    MotorConsole_ProcessLine("I1000.01"); CHECK(motor_console_debug.last_result == MOTOR_COMMAND_RANGE);
    MotorConsole_ProcessLine("D10.01"); CHECK(motor_console_debug.last_result == MOTOR_COMMAND_RANGE);
    MotorConsole_ProcessLine("P-0.1"); CHECK(motor_console_debug.last_result == MOTOR_COMMAND_RANGE);
    for (uint32_t i = 0U; i < sizeof(bad_lines) / sizeof(bad_lines[0]); i++)
    {
        MotorConsole_ProcessLine(bad_lines[i]);
        CHECK(motor_console_debug.last_result != MOTOR_COMMAND_OK && motor_control.enabled == 0U);
    }
    MotorConsole_ProcessLine(0); CHECK(motor_console_debug.last_result == MOTOR_COMMAND_SYNTAX);
    test_stage = 5U;
    ResetMotor(); MotorConsole_CreateQueue(); MotorConsole_Start();
    InjectText("S3"); MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    previous = motor_console_debug.command_count;
    InjectText("0\r\n"); MotorConsole_Poll();
    CHECK(motor_control.enabled == 1U && motor_control.target_rpm == 30.0f);
    CHECK(motor_console_debug.command_count == previous + 1U);
    StepAfter(2U, 0, 35U);
    /* 默认上电输出 T1，便于滑块参数和命令结果直接绑定数值通道。 */
    MotorConsole_SendTelemetry();
    CHECK(motor_console_debug.telemetry_mode == 1U);
    CHECK(strcmp(transmitted, "speed:30.00,0.00,150,5.0000,0.0000,0.0000,1,1,0,1\r\n") == 0);
    MotorConsole_ProcessLine("T0"); MotorConsole_SendTelemetry();
    CHECK(strncmp(transmitted, "speed:30.00,0.00", 15U) == 0);
    CHECK(strchr(transmitted, '\r') != 0 && strchr(transmitted, '\n') != 0);
    comma_count = 0U;
    for (text = transmitted; *text != 0; text++) if (*text == ',') comma_count++;
    CHECK(comma_count == 1U);
    MotorConsole_ProcessLine("T1"); MotorConsole_SendTelemetry();
    comma_count = 0U;
    for (text = transmitted; *text != 0; text++) if (*text == ',') comma_count++;
    CHECK(comma_count == 9U);
    CHECK(strcmp(transmitted, "speed:30.00,0.00,150,5.0000,0.0000,0.0000,1,1,0,1\r\n") == 0);
    /* 千分之一的 Kd 必须在遥测中看得到，不能被两位小数显示成零。 */
    MotorConsole_ProcessLine("D0.001");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK);
    CHECK(Near(motor_control.kd, 0.001f, 0.0000001f));
    MotorConsole_SendTelemetry();
    CHECK(strcmp(transmitted, "speed:30.00,0.00,150,5.0000,0.0000,0.0010,1,1,0,1\r\n") == 0);
    /* 所有增益上界在扩展帧中保留四位小数，格式化不会截断或越界。 */
    MotorConsole_ProcessLine("P200");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK && motor_control.kp == SPEED_PID_KP_MAX);
    MotorConsole_ProcessLine("I1000");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK && motor_control.ki == SPEED_PID_KI_MAX);
    MotorConsole_ProcessLine("D10");
    CHECK(motor_console_debug.last_result == MOTOR_COMMAND_OK && motor_control.kd == SPEED_PID_KD_MAX);
    MotorConsole_SendTelemetry();
    CHECK(strcmp(transmitted, "speed:30.00,0.00,150,200.0000,1000.0000,10.0000,1,1,0,1\r\n") == 0);
    previous = motor_control.output_raw;
    MotorConsole_ProcessLine("T0");
    CHECK(motor_control.output_raw == (int16_t)previous && motor_control.enabled == 1U);
    InjectText("STOP\n"); MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    InjectText("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
    MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    InjectText("S30\n"); now_ms += 1001U;
    MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    InjectText("\n"); MotorConsole_Poll(); ResetMotor();
    InjectText("S30\n"); MotorConsole_Poll(); CHECK(motor_control.enabled == 1U);
    test_stage = 6U;
    for (uint32_t i = 0U; i < 129U; i++) InjectText("X");
    MotorConsole_Poll(); CHECK(motor_console_debug.rx_drop_count == 1U && motor_control.enabled == 0U);
    InjectText("\n"); MotorConsole_Poll();
    InjectText("S30\n"); MotorConsole_Poll(); CHECK(motor_control.enabled == 1U);
    HAL_UART_ErrorCallback(&huart1); MotorConsole_Poll();
    CHECK(motor_control.enabled == 0U && motor_console_debug.rx_error_count == 1U);
    CHECK(rx_destination != 0);
    InjectText("\n"); MotorConsole_Poll(); CHECK(motor_control.enabled == 0U);
    CHECK(critical_depth == 0U);
}
void test_run(void)
{
    TestPid();
    TestProtocol();
    TestControl();
    TestIntegralAuthority();
    TestDefaultResponse();
    TestTargetTransitions();
    TestConcurrentRequests();
    TestConsole();
    test_done = 1U;
}
