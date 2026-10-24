#include "dm3507.h"
#include <string.h>
volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static CanFrame frame, command;
static Dm3507Feedback feedback;
static float absolute(float x) { return x < 0.0f ? -x : x; }
static void raw(uint16_t p, uint16_t v, uint16_t t, uint8_t status)
{
    memset(&frame, 0, sizeof(frame)); frame.bus = 1U; frame.id = 2U; frame.dlc = 8U;
    frame.received_ms = 123U; frame.fault_epoch = 7U;
    frame.data[0] = (uint8_t)((status << 4) | 2U);
    frame.data[1] = (uint8_t)(p >> 8); frame.data[2] = (uint8_t)p;
    frame.data[3] = (uint8_t)(v >> 4); frame.data[4] = (uint8_t)((v << 4) | (t >> 8));
    frame.data[5] = (uint8_t)t; frame.data[6] = 200U; frame.data[7] = 210U;
}
void test_run(void)
{
    uint32_t k;
    uint16_t t;
    float requested, decoded;
    union { uint32_t bits; float value; } bad;
    test_stage = 1U;
    raw(0U, 0U, 0U, 0U); CHECK(Dm3507_Decode(&frame, &feedback));
    CHECK(feedback.position_rad == -DM3507_POSITION_MAX_RAD && feedback.velocity_rad_s == -200.0f);
    CHECK(feedback.torque_nm == -10.0f && feedback.status == 0U && feedback.motor_id == 2U);
    CHECK(feedback.temp_mos == 200U && feedback.temp_rotor == 210U);
    CHECK(feedback.received_ms == 123U && feedback.fault_epoch == 7U);
    raw(65535U, 4095U, 4095U, 15U); CHECK(Dm3507_Decode(&frame, &feedback));
    CHECK(feedback.position_rad == DM3507_POSITION_MAX_RAD && feedback.velocity_rad_s == 200.0f);
    CHECK(feedback.torque_nm == 10.0f && feedback.status == 15U);
    CHECK(!Dm3507_Decode(0, &feedback) && !Dm3507_Decode(&frame, 0));
    test_stage = 2U;
    frame.bus = 2U; CHECK(!Dm3507_Decode(&frame, &feedback)); frame.bus = 1U;
    frame.id = 0U; CHECK(!Dm3507_Decode(&frame, &feedback)); frame.id = 2U;
    frame.extended = 1U; CHECK(!Dm3507_Decode(&frame, &feedback)); frame.extended = 0U;
    frame.dlc = 7U; CHECK(!Dm3507_Decode(&frame, &feedback)); frame.dlc = 8U;
    frame.data[0] = 0x11U; CHECK(!Dm3507_Decode(&frame, &feedback));
    raw(0x0033U, 0U, 0U, 0U); CHECK(!Dm3507_Decode(&frame, &feedback));
    raw(0x0055U, 0U, 0U, 0U); CHECK(!Dm3507_Decode(&frame, &feedback));
    /* 普通反馈位置低字节恰为操作字仍有效。 */
    raw(0x8033U, 2048U, 2048U, 0U); CHECK(Dm3507_Decode(&frame, &feedback));
    raw(0x0055U, 2048U, 2048U, 1U); CHECK(Dm3507_Decode(&frame, &feedback));
    test_stage = 3U;
    CHECK(Dm3507_MakeEnable(&command) && command.bus == 1U && command.id == 2U && command.dlc == 8U);
    for (k = 0U; k < 7U; ++k) { CHECK(command.data[k] == 0xFFU); }
    CHECK(command.data[7] == 0xFCU && !command.extended);
    CHECK(Dm3507_MakeDisable(&command) && command.data[7] == 0xFDU);
    CHECK(Dm3507_MakeReadFeedback(&command) && command.id == 0x7FFU && command.data[0] == 2U);
    CHECK(command.data[1] == 0U && command.data[2] == 0xCCU);
    for (k = 3U; k < 8U; ++k) { CHECK(command.data[k] == 0U); }
    CHECK(!Dm3507_MakeEnable(0) && !Dm3507_MakeDisable(0) && !Dm3507_MakeReadFeedback(0));
    test_stage = 4U;
    CHECK(Dm3507_MakeTorque(0.0f, &command));
    CHECK(command.data[0] == 0x7FU && command.data[1] == 0xFFU && command.data[2] == 0x7FU);
    CHECK(command.data[3] == 0xF0U && !command.data[4] && !command.data[5] && command.data[6] == 7U);
    CHECK(command.data[7] == 0xFFU);
    CHECK(Dm3507_MakeTorque(1000.0f, &command) && command.data[6] == 15U && command.data[7] == 255U);
    CHECK(Dm3507_MakeTorque(-1000.0f, &command) && !command.data[6] && !command.data[7]);
    CHECK(!Dm3507_MakeTorque(1.0f, 0));
    bad.bits = 0x7FC00000U; CHECK(!Dm3507_MakeTorque(bad.value, &command));
    bad.bits = 0x7F800000U; CHECK(!Dm3507_MakeTorque(bad.value, &command));
    bad.bits = 0xFF800000U; CHECK(!Dm3507_MakeTorque(bad.value, &command));
    /* 全部12位给定跨量程检查，验证无暗藏低幅业务限幅。 */
    test_stage = 5U;
    for (k = 0U; k <= 4095U; ++k) {
        requested = -10.0f + 20.0f * (float)k / 4095.0f;
        CHECK(Dm3507_MakeTorque(requested, &command));
        t = (uint16_t)(((uint16_t)(command.data[6] & 15U) << 8) | command.data[7]);
        decoded = -10.0f + 20.0f * (float)t / 4095.0f;
        CHECK(absolute(decoded - requested) <= 20.0f / 4095.0f + 0.00001f);
        CHECK((command.data[3] & 15U) == 0U && !command.data[4] && !command.data[5] && (command.data[6] >> 4) == 0U);
        CHECK(command.bus == 1U && command.id == 2U && command.dlc == 8U && !command.extended);
    }
    test_done = 1U;
}
