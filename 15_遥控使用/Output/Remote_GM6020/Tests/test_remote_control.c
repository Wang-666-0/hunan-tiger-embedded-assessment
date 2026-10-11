#include "remote_control.h" /* 测真实许可状态机，不接电机。 */
#include "app_config.h"
#include "ibus.h"
#include "gm6020.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

volatile uint32_t test_checks;
volatile uint32_t test_failures;
volatile uint32_t test_failed_line;
volatile uint32_t test_done;
volatile uint32_t test_stage;

#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (test_failed_line == 0U) { test_failed_line = __LINE__; } } } while (0)

static void Near(float actual, float expected)
{
    CHECK(fabsf(actual - expected) < 0.0001f);
}

static void HealthyInput(RemoteControl_Input *input, uint32_t now)
{
    uint8_t index;
    memset(input, 0, sizeof(*input));
    for (index = 0U; index < 14U; index++) { input->channels[index] = 1500U; }
    input->channels[4] = 2000U;
    input->channels[5] = 2000U;
    input->now_ms = now;
    input->received_ms = now;
    input->frame_sequence = 1U;
    input->has_frame = 1U;
    input->can_online = 1U;
    input->can_temperature = 25U;
}

static void NewFrame(RemoteControl_Input *input, uint16_t arm, uint16_t stick)
{
    input->now_ms += 7U;
    input->received_ms = input->now_ms;
    input->frame_sequence++;
    input->channels[4] = arm;
    input->channels[0] = stick;
}

static void CheckStopped(const RemoteControl *control, RemoteControl_Fault fault)
{
    CHECK(control->armed == 0U);
    CHECK(control->off_seen == 0U);
    CHECK(control->fault == (uint32_t)fault);
    Near(control->target_rpm, 0.0f);
}

static void Start(RemoteControl *control, RemoteControl_Input *input, uint32_t now)
{
    RemoteControl_Init(control);
    HealthyInput(input, now);
    NewFrame(input, 1000U, 1500U);
    RemoteControl_Step(control, input);
    CHECK(control->armed == 0U);
    CHECK(control->off_seen == 1U);
    NewFrame(input, 2000U, 1500U);
    RemoteControl_Step(control, input);
    CHECK(control->armed == 1U);
    CHECK(control->fault == REMOTE_FAULT_NONE);
    NewFrame(input, 2000U, 2000U);
    RemoteControl_Step(control, input);
    CHECK(control->armed == 1U);
    Near(control->target_rpm, 100.0f);
}

static void RequireRearm(RemoteControl *control, RemoteControl_Input *input)
{
    /* 故障恢复但开关保持高，不可自动重启；仍须新 OFF->ON 帧。 */
    NewFrame(input, 2000U, 1500U);
    RemoteControl_Step(control, input);
    CHECK(control->armed == 0U);
    Near(control->target_rpm, 0.0f);
    NewFrame(input, 1000U, 1500U);
    RemoteControl_Step(control, input);
    CHECK(control->off_seen == 1U);
    NewFrame(input, 2000U, 1500U);
    RemoteControl_Step(control, input);
    CHECK(control->armed == 1U);
    CHECK(control->fault == REMOTE_FAULT_NONE);
    Near(control->target_rpm, 0.0f);
}

static void TestStartupAndArming(void)
{
    RemoteControl control;
    RemoteControl_Input input;
    RemoteControl_Init(&control);
    CheckStopped(&control, REMOTE_FAULT_POWER_ON);
    HealthyInput(&input, 100U);
    RemoteControl_Step(&control, &input); /* 上电已高，无法启动。 */
    CheckStopped(&control, REMOTE_FAULT_POWER_ON);
    NewFrame(&input, 2000U, 2000U);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_POWER_ON);
    NewFrame(&input, 1000U, 2000U);
    RemoteControl_Step(&control, &input);
    CHECK(control.off_seen == 1U);
    CHECK(control.armed == 0U);
    NewFrame(&input, 2000U, 2000U);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_NOT_CENTERED);
    NewFrame(&input, 2000U, 1500U); /* 移回中心也不能补做同一次开启动作。 */
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_NOT_CENTERED);
    RequireRearm(&control, &input);
    NewFrame(&input, 2000U, 1000U);
    RemoteControl_Step(&control, &input);
    Near(control.target_rpm, -100.0f);
    NewFrame(&input, 2000U, 1500U);
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    Near(control.target_rpm, 0.0f); /* 中位是已许可的 0rpm 目标。 */
    NewFrame(&input, 1000U, 1500U);
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 0U);
    CHECK(control.fault == REMOTE_FAULT_DISARM);
    CHECK(control.off_seen == 1U);
}

static void TestSwitchEdgesAndOldFrames(void)
{
    RemoteControl control;
    RemoteControl_Input input;
    HealthyInput(&input, 1U);
    RemoteControl_Init(&control);
    NewFrame(&input, 1300U, 1470U);
    RemoteControl_Step(&control, &input);
    CHECK(control.off_seen == 1U);
    NewFrame(&input, 1700U, 1470U);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_DISARM);
    NewFrame(&input, 1701U, 1470U);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_DISARM);
    NewFrame(&input, 1300U, 1530U);
    RemoteControl_Step(&control, &input);
    CHECK(control.off_seen == 1U);
    NewFrame(&input, 1701U, 1530U);
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    Near(control.target_rpm, 0.0f);
    input.channels[0] = 2000U; /* 没有新帧，故意改变输入副本也不能更新目标。 */
    RemoteControl_Step(&control, &input);
    Near(control.target_rpm, 0.0f);
    input.frame_sequence++;
    RemoteControl_Step(&control, &input);
    Near(control.target_rpm, 100.0f);
    /* 故障发生在 OFF 帧处理后，恢复时同一旧 OFF 帧不能解除锁定。 */
    NewFrame(&input, 1000U, 1500U);
    RemoteControl_Step(&control, &input);
    CHECK(control.off_seen == 1U);
    input.uart_fault = 1U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_UART);
    input.uart_fault = 0U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_UART);
    RequireRearm(&control, &input);
    NewFrame(&input, 1301U, 1500U); /* 中间档必须停机，不能冒充 OFF。 */
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_DISARM);
}

static void TestGuardAndFaults(void)
{
    RemoteControl control;
    RemoteControl_Input input;
    Start(&control, &input, 100U);
    NewFrame(&input, 2000U, 1500U);
    input.channels[5] = 1700U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_FAILSAFE);
    input.channels[5] = 1701U;
    RequireRearm(&control, &input);
    input.has_frame = 0U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_LINK);
    input.has_frame = 1U;
    RequireRearm(&control, &input);
    input.can_online = 0U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_CAN);
    input.can_online = 1U;
    RequireRearm(&control, &input);
    input.can_temperature = 79U;
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    input.can_temperature = 80U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_TEMPERATURE);
    input.can_temperature = 25U;
    RequireRearm(&control, &input);
    input.uart_fault = 1U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_UART);
    input.uart_fault = 0U;
    RequireRearm(&control, &input);
    input.can_tx_fault = 1U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_CAN_TX);
    input.can_tx_fault = 0U;
    RequireRearm(&control, &input);
    input.control_late = 1U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_CONTROL_TIMING);
    input.control_late = 0U;
    RequireRearm(&control, &input);
    RemoteControl_ForceStop(&control, REMOTE_FAULT_STOP);
    CheckStopped(&control, REMOTE_FAULT_STOP);
    RemoteControl_Step(&control, &input); /* STOP 后旧快照不能恢复。 */
    CheckStopped(&control, REMOTE_FAULT_STOP);
    RequireRearm(&control, &input);
}

static void TestChannelValidityAndCenter(void)
{
    RemoteControl control;
    RemoteControl_Input input;
    uint8_t index;
    static const uint8_t tested_channels[] = {0U, 4U, 5U};
    for (index = 0U; index < sizeof(tested_channels); index++)
    {
        Start(&control, &input, 100U);
        input.channels[tested_channels[index]] = 799U;
        RemoteControl_Step(&control, &input);
        CheckStopped(&control, REMOTE_FAULT_LINK);
        Start(&control, &input, 100U);
        input.channels[tested_channels[index]] = 2201U;
        RemoteControl_Step(&control, &input);
        CheckStopped(&control, REMOTE_FAULT_LINK);
    }
    RemoteControl_Init(&control);
    HealthyInput(&input, 100U);
    NewFrame(&input, 1000U, 1500U);
    RemoteControl_Step(&control, &input);
    NewFrame(&input, 2000U, 1531U);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_NOT_CENTERED);
    RemoteControl_Init(&control);
    HealthyInput(&input, 100U);
    NewFrame(&input, 1000U, 1500U);
    RemoteControl_Step(&control, &input);
    NewFrame(&input, 2000U, 1469U);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_NOT_CENTERED);
    Near(RemoteControl_MapSpeed(1000U), -100.0f);
    Near(RemoteControl_MapSpeed(2000U), 100.0f);
    Near(RemoteControl_MapSpeed(1470U), 0.0f);
    Near(RemoteControl_MapSpeed(1530U), 0.0f);
    Near(RemoteControl_MapSpeed(1469U), -6.2f);
    Near(RemoteControl_MapSpeed(1531U), 6.2f);
    Near(RemoteControl_MapSpeed(0U), -100.0f);
    Near(RemoteControl_MapSpeed(UINT16_MAX), 100.0f);
}

static void TestFeedbackThresholdAndTimeout(void)
{
    RemoteControl control;
    RemoteControl_Input input;
    GM6020_Feedback feedback;
    memset(&feedback, 0, sizeof(feedback));
    CHECK(GM6020_IsOnline(&feedback, 0U) == 0U);
    CHECK(GM6020_IsOnline(NULL, 0U) == 0U);
    feedback.rx_count = 1U;
    feedback.last_rx_ms = 100U;
    CHECK(GM6020_IsOnline(&feedback, 119U) == 1U);
    CHECK(GM6020_IsOnline(&feedback, 120U) == 0U);
    Start(&control, &input, 100U);
    feedback.last_rx_ms = input.now_ms;
    input.now_ms += 19U;
    input.can_online = GM6020_IsOnline(&feedback, input.now_ms);
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    input.now_ms++;
    input.can_online = GM6020_IsOnline(&feedback, input.now_ms);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_CAN);

    Start(&control, &input, 1000U);
    input.now_ms = input.received_ms + 99U;
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    input.now_ms++;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_LINK);
    RequireRearm(&control, &input);
}

static void EncodeFrame(uint8_t frame[32], const RemoteControl_Input *input)
{
    uint8_t index;
    uint16_t sum = 0U;
    uint16_t checksum;
    frame[0] = 0x20U; frame[1] = 0x40U;
    for (index = 0U; index < 14U; index++) {
        frame[2U + 2U * index] = (uint8_t)input->channels[index];
        frame[3U + 2U * index] = (uint8_t)(input->channels[index] >> 8U);
    }
    for (index = 0U; index < 30U; index++) { sum = (uint16_t)(sum + frame[index]); }
    checksum = (uint16_t)(0xFFFFU - sum);
    frame[30] = (uint8_t)checksum; frame[31] = (uint8_t)(checksum >> 8U);
}

static void InputFromParser(RemoteControl_Input *input, const ibus_parser_t *parser, uint32_t now)
{
    memcpy(input->channels, parser->latest.channels, sizeof(input->channels));
    input->has_frame = parser->has_frame;
    input->received_ms = parser->latest.received_ms;
    input->frame_sequence = parser->stats.valid_frames;
    input->now_ms = now;
}

static void TestBadFramesCannotRenewLink(void)
{
    ibus_parser_t parser;
    RemoteControl control;
    RemoteControl_Input input;
    uint8_t frame[32];
    uint32_t time;
    HealthyInput(&input, 0U);
    RemoteControl_Init(&control);
    ibus_init(&parser);
    input.channels[4] = 1000U;
    EncodeFrame(frame, &input);
    CHECK(ibus_feed(&parser, frame, 32U, 100U, NULL) == 1U);
    InputFromParser(&input, &parser, 100U);
    RemoteControl_Step(&control, &input);
    input.channels[4] = 2000U;
    EncodeFrame(frame, &input);
    CHECK(ibus_feed(&parser, frame, 32U, 107U, NULL) == 1U);
    InputFromParser(&input, &parser, 107U);
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    input.channels[0] = 2000U;
    EncodeFrame(frame, &input);
    CHECK(ibus_feed(&parser, frame, 32U, 114U, NULL) == 1U);
    InputFromParser(&input, &parser, 114U);
    RemoteControl_Step(&control, &input);
    Near(control.target_rpm, 100.0f);
    frame[30] ^= 1U;
    for (time = 121U; time < 214U; time += 7U)
    {
        CHECK(ibus_feed(&parser, frame, 32U, time, NULL) == 0U);
        InputFromParser(&input, &parser, time);
        RemoteControl_Step(&control, &input);
        CHECK(control.armed == 1U);
        CHECK(input.received_ms == 114U);
        CHECK(input.frame_sequence == 3U);
    }
    CHECK(ibus_feed(&parser, frame, 32U, 214U, NULL) == 0U);
    InputFromParser(&input, &parser, 214U);
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_LINK);
    CHECK(parser.stats.valid_frames == 3U);
    CHECK(parser.stats.checksum_errors > 0U);
}

static void TestMillisAndSequenceWrap(void)
{
    RemoteControl control;
    RemoteControl_Input input;
    GM6020_Feedback feedback;
    Start(&control, &input, UINT32_MAX - 31U);
    /* 最后接收 UINT32_MAX-10；now=88 时年龄99，89时年龄100。 */
    input.now_ms = 88U;
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    input.now_ms = 89U;
    RemoteControl_Step(&control, &input);
    CheckStopped(&control, REMOTE_FAULT_LINK);
    memset(&feedback, 0, sizeof(feedback));
    feedback.rx_count = 1U;
    feedback.last_rx_ms = UINT32_MAX - 9U;
    CHECK(GM6020_IsOnline(&feedback, 9U) == 1U);
    CHECK(GM6020_IsOnline(&feedback, 10U) == 0U);

    RemoteControl_Init(&control);
    HealthyInput(&input, 100U);
    input.frame_sequence = UINT32_MAX;
    input.channels[4] = 1000U;
    RemoteControl_Step(&control, &input);
    CHECK(control.off_seen == 1U);
    NewFrame(&input, 2000U, 1500U); /* valid_frames 从 UINT32_MAX 回卷到 0。 */
    CHECK(input.frame_sequence == 0U);
    RemoteControl_Step(&control, &input);
    CHECK(control.armed == 1U);
    CHECK(control.fault == REMOTE_FAULT_NONE);
}

static void TestNullArguments(void)
{
    RemoteControl control, before;
    RemoteControl_Input input;
    RemoteControl_Init(NULL);
    RemoteControl_ForceStop(NULL, REMOTE_FAULT_STOP);
    HealthyInput(&input, 1U);
    RemoteControl_Step(NULL, &input);
    RemoteControl_Init(&control);
    before = control;
    RemoteControl_Step(&control, NULL);
    CHECK(memcmp(&control, &before, sizeof(control)) == 0);
}

void test_run(void)
{
    test_stage = 1U; TestStartupAndArming();
    test_stage = 2U; TestSwitchEdgesAndOldFrames();
    test_stage = 3U; TestGuardAndFaults();
    test_stage = 4U; TestChannelValidityAndCenter();
    test_stage = 5U; TestFeedbackThresholdAndTimeout();
    test_stage = 6U; TestBadFramesCannotRenewLink();
    test_stage = 7U; TestMillisAndSequenceWrap();
    test_stage = 8U; TestNullArguments();
    test_done = 1U;
}
