#include "gl60_ii.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

static uint8_t Near(float actual, float wanted, float tolerance)
{
    float difference = actual - wanted;
    return difference >= -tolerance && difference <= tolerance;
}

static float FloatBits(uint32_t raw)
{
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static CanFrame Reply(uint8_t status, uint16_t p, uint16_t v, uint16_t t)
{
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.bus = 1U; frame.id = 0U; frame.dlc = 8U;
    frame.received_ms = 1234U; frame.fault_epoch = 17U;
    frame.data[0] = (uint8_t)((status << 4) | 1U);
    frame.data[1] = (uint8_t)(p >> 8); frame.data[2] = (uint8_t)p;
    frame.data[3] = (uint8_t)(v >> 4);
    frame.data[4] = (uint8_t)(((v & 15U) << 4) | (t >> 8));
    frame.data[5] = (uint8_t)t;
    frame.data[6] = 0xF6U; frame.data[7] = 0x7FU;
    return frame;
}

static uint16_t TorqueRaw(const CanFrame *frame)
{
    return (uint16_t)(((uint16_t)(frame->data[6] & 15U) << 8) | frame->data[7]);
}

static void CheckReject(const CanFrame *frame)
{
    Gl60Feedback actual, untouched;
    memset(&actual, 0xA5, sizeof(actual)); untouched = actual;
    CHECK(Gl60_Decode(frame, &actual) == 0U);
    CHECK(memcmp(&actual, &untouched, sizeof(actual)) == 0);
}

void test_run(void)
{
    CanFrame frame, original, untouched;
    Gl60Feedback feedback;
    uint32_t i;
    uint16_t raw, previous = 0U;
    float requested, decoded;
    static const uint8_t zero_torque[8] = {0x7FU, 0xFFU, 0x7FU, 0xF0U, 0U, 0U, 7U, 0xFFU};

    test_stage = 1U;
    Gl60_BuildEnable(0); Gl60_BuildDisable(0);
    Gl60_BuildEnable(&frame);
    CHECK(CanFrame_IsValid(&frame));
    CHECK(frame.bus == 1U && frame.extended == 0U && frame.id == 1U && frame.dlc == 8U);
    CHECK(frame.received_ms == 0U && frame.fault_epoch == 0U);
    for (i = 0U; i < 7U; ++i) { CHECK(frame.data[i] == 0xFFU); }
    CHECK(frame.data[7] == 0xFCU);
    Gl60_BuildDisable(&frame);
    CHECK(frame.bus == 1U && frame.extended == 0U && frame.id == 1U && frame.dlc == 8U);
    for (i = 0U; i < 7U; ++i) { CHECK(frame.data[i] == 0xFFU); }
    CHECK(frame.data[7] == 0xFDU);
    CHECK(Gl60_BuildTorque(0.0f, 0) == 0U);
    CHECK(Gl60_BuildTorque(0.0f, &frame) == 1U);
    CHECK(memcmp(frame.data, zero_torque, sizeof(zero_torque)) == 0);
    CHECK(frame.received_ms == 0U && frame.fault_epoch == 0U);
    CHECK(Gl60_BuildTorque(-10.0f, &frame) == 1U && TorqueRaw(&frame) == 0U);
    CHECK(Gl60_BuildTorque(10.0f, &frame) == 1U && TorqueRaw(&frame) == 4095U);
    CHECK(Gl60_BuildTorque(-100.0f, &frame) == 1U && TorqueRaw(&frame) == 0U);
    CHECK(Gl60_BuildTorque(100.0f, &frame) == 1U && TorqueRaw(&frame) == 4095U);
    CHECK(Gl60_BuildTorque(FloatBits(0x7F7FFFFFU), &frame) == 1U && TorqueRaw(&frame) == 4095U);
    CHECK(Gl60_BuildTorque(FloatBits(0xFF7FFFFFU), &frame) == 1U && TorqueRaw(&frame) == 0U);
    memset(&frame, 0xA5, sizeof(frame)); untouched = frame;
    CHECK(Gl60_BuildTorque(FloatBits(0x7FC00000U), &frame) == 0U);
    CHECK(memcmp(&frame, &untouched, sizeof(frame)) == 0);
    CHECK(Gl60_BuildTorque(FloatBits(0x7F800000U), &frame) == 0U);
    CHECK(memcmp(&frame, &untouched, sizeof(frame)) == 0);
    CHECK(Gl60_BuildTorque(FloatBits(0xFF800000U), &frame) == 0U);
    CHECK(memcmp(&frame, &untouched, sizeof(frame)) == 0);

    test_stage = 2U;
    frame = Reply(1U, 0U, 0U, 0U);
    CHECK(Gl60_Decode(&frame, &feedback) == 1U);
    CHECK(Near(feedback.position_rad, -GL60_POSITION_MAX_RAD, 0.000001f));
    CHECK(Near(feedback.velocity_rad_s, -200.0f, 0.00001f));
    CHECK(Near(feedback.torque_nm, -10.0f, 0.00001f));
    CHECK(feedback.status == 1U && feedback.motor_id == 1U);
    CHECK(feedback.driver_temp_c == -10 && feedback.motor_temp_c == 127);
    CHECK(feedback.received_ms == 1234U && feedback.fault_epoch == 17U);
    frame = Reply(0U, 65535U, 4095U, 4095U);
    frame.data[6] = 0x80U; frame.data[7] = 0xFFU;
    CHECK(Gl60_Decode(&frame, &feedback) == 1U);
    CHECK(Near(feedback.position_rad, GL60_POSITION_MAX_RAD, 0.000001f));
    CHECK(Near(feedback.velocity_rad_s, 200.0f, 0.00001f));
    CHECK(Near(feedback.torque_nm, 10.0f, 0.00001f));
    CHECK(feedback.driver_temp_c == -128 && feedback.motor_temp_c == -1);
    frame = Reply(1U, 32767U, 2047U, 2047U);
    CHECK(Gl60_Decode(&frame, &feedback) == 1U);
    CHECK(feedback.position_rad < 0.0f && Near(feedback.position_rad, -0.00004794f, 0.000001f));
    CHECK(feedback.velocity_rad_s < 0.0f && Near(feedback.velocity_rad_s, -0.048840f, 0.00002f));
    CHECK(feedback.torque_nm < 0.0f && Near(feedback.torque_nm, -0.002442f, 0.00001f));
    frame = Reply(1U, 32768U, 2048U, 2048U);
    CHECK(Gl60_Decode(&frame, &feedback) == 1U);
    CHECK(feedback.position_rad > 0.0f && feedback.velocity_rad_s > 0.0f && feedback.torque_nm > 0.0f);
    frame = Reply(1U, 0xE321U, 0xABCU, 0x123U);
    CHECK(Gl60_Decode(&frame, &feedback) == 1U);
    CHECK(feedback.position_rad > 2.4f && feedback.velocity_rad_s > 60.0f && feedback.torque_nm < -8.0f);

    test_stage = 3U;
    for (i = 0U; i < 16U; ++i) {
        frame = Reply((uint8_t)i, 65535U, 4095U, 4095U);
        if (i == 0U || i == 1U || (i >= 8U && i <= 14U)) {
            CHECK(Gl60_Decode(&frame, &feedback) == 1U && feedback.status == i);
        } else { CheckReject(&frame); }
    }
    original = Reply(1U, 12345U, 3456U, 2345U);
    CheckReject(0);
    CHECK(Gl60_Decode(&original, 0) == 0U);
    frame = original; frame.bus = 2U; CheckReject(&frame);
    frame.bus = 0U; CheckReject(&frame);
    frame = original; frame.extended = 1U; CheckReject(&frame);
    frame.extended = 2U; CheckReject(&frame);
    frame = original; frame.id = 1U; CheckReject(&frame);
    frame.id = 0x800U; CheckReject(&frame);
    frame = original;
    for (i = 0U; i < 10U; ++i) { if (i != 8U) { frame.dlc = (uint8_t)i; CheckReject(&frame); } }
    for (i = 0U; i < 16U; ++i) {
        if (i != 1U) { frame = original; frame.data[0] = (uint8_t)(0x10U | i); CheckReject(&frame); }
    }
    /* DM parameter replies with ID2 and echoed special commands cannot match. */
    frame = original; frame.data[0] = 2U; frame.data[2] = 0x33U; CheckReject(&frame);
    Gl60_BuildEnable(&frame); frame.id = 0U; CheckReject(&frame);
    Gl60_BuildDisable(&frame); frame.id = 0U; CheckReject(&frame);

    test_stage = 4U;
    for (i = 0U; i <= 4000U; ++i) {
        requested = -10.0f + (float)i * 0.005f;
        CHECK(Gl60_BuildTorque(requested, &frame) == 1U);
        CHECK(frame.bus == 1U && frame.extended == 0U && frame.id == 1U && frame.dlc == 8U);
        CHECK(frame.data[0] == 0x7FU && frame.data[1] == 0xFFU && frame.data[2] == 0x7FU);
        CHECK(frame.data[3] == 0xF0U && frame.data[4] == 0U && frame.data[5] == 0U && (frame.data[6] & 0xF0U) == 0U);
        raw = TorqueRaw(&frame);
        CHECK(raw >= previous && raw <= 4095U); previous = raw;
        decoded = (float)raw * 20.0f / 4095.0f - 10.0f;
        CHECK(decoded <= requested + 0.00001f);
        CHECK(requested - decoded <= 20.0f / 4095.0f + 0.00001f);
    }
    CHECK(Gl60_BuildTorque(0.03f, &frame) == 1U);
    CHECK(TorqueRaw(&frame) == 2053U); /* quantized +0.026862 Nm. */
    CHECK(Gl60_BuildTorque(-0.03f, &frame) == 1U);
    CHECK(TorqueRaw(&frame) == 2041U); /* quantized -0.031746 Nm; sender must cap. */
    test_done = 1U;
}
