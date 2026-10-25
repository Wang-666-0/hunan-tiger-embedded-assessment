#define test_run limited_can_fixture_not_run
#include "test_yaw_can.c"
#undef test_run
#include "yaw_controller.h"

static uint16_t RawTorque(void)
{ return (uint16_t)(((tx[sends - 1U].data[6] & 15U) << 8) | tx[sends - 1U].data[7]); }
void test_run(void)
{
    uint32_t before, i;
    uint16_t previous = 0U;
    static const float requests[] = {-12, -10, -5, -2.45f, -0.24f, -0.1f, 0, 0.1f, 0.24f, 2.45f, 5, 10, 12};
    test_stage = 1U;
    CHECK(!YAW_CASCADE_OUTPUT_LIMITS_ENABLED && YAW_CONTROLLER_TORQUE_LIMIT_NM == 10);
    CanBus_CreateQueue(); CanBus_Start();
    CHECK(CanBus_YawTorque(2.45f) == HAL_ERROR && !sends);
    CanBus_SetYawPermission(1U);
    for (i = 0U; i < sizeof(requests) / sizeof(requests[0]); ++i) {
        uint16_t raw;
        CHECK(CanBus_YawTorque(requests[i]) == HAL_OK);
        raw = RawTorque(); CHECK(raw >= previous); previous = raw;
        CHECK(tx[sends - 1U].bus == 1U && tx[sends - 1U].header.StdId == 1U);
        CHECK(tx[sends - 1U].header.IDE == CAN_ID_STD && tx[sends - 1U].header.DLC == 8);
        CHECK(tx[sends - 1U].data[3] == 0xF0 && tx[sends - 1U].data[4] == 0 && tx[sends - 1U].data[5] == 0);
        CHECK((tx[sends - 1U].data[6] & 0xF0) == 0); /* MIT内部KP/KD仍0。 */
        if (requests[i] <= -10) { CHECK(raw == 0); }
        if (requests[i] >= 10) { CHECK(raw == 4095); }
        if (requests[i] == 2.45f) { CHECK(raw >= 2548 && raw <= 2550); }
        if (requests[i] == -2.45f) { CHECK(raw >= 1545 && raw <= 1547); }
        if (requests[i] > 0.1f) { CHECK(raw > 2067); }
        if (requests[i] < -0.1f) { CHECK(raw < 2028); }
        ClearPending();
    }
    before = sends;
    CHECK(CanBus_YawTorque(FloatBits(0x7FC00000U)) == HAL_ERROR);
    CHECK(CanBus_YawTorque(FloatBits(0x7F800000U)) == HAL_ERROR);
    CHECK(CanBus_YawTorque(FloatBits(0xFF800000U)) == HAL_ERROR);
    CHECK(sends == before && CanBus_YawAllowed());
    test_stage = 2U;
    CanBus_SetOutputPermission(1U);
    CHECK(CanBus_JogVescCurrent(70U, 2000) == HAL_OK); ClearPending();
    before = sends;
    CHECK(CanBus_JogVescCurrent(70U, 2001) == HAL_ERROR && sends == before);
    Gl60_BuildEnable(&queued); CHECK(CanBus_Send(&queued) == HAL_ERROR && sends == before);
    CanBus_SetYawPermission(0U);
    CHECK(CanBus_YawTorque(2.45f) == HAL_ERROR && CanBus_OutputAllowed());
    CHECK(CanBus_YawDisable(1U) == HAL_OK); CheckYaw(sends - 1U, 0xFD); ClearPending();
    CanBus_SetYawPermission(1U); fail_send_call = sends + 1U;
    CHECK(CanBus_YawTorque(2.45f) == HAL_ERROR);
    CHECK(!CanBus_YawAllowed() && !CanBus_OutputAllowed());
    fail_send_call = 0U; CHECK(CanBus_YawDisable(1U) == HAL_OK);
    test_done = 1U;
}
