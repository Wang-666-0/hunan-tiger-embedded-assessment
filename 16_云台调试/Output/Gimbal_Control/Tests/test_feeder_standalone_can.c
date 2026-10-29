/**
 * @file test_feeder_standalone_can.c
 * @brief 独立单发配置的真实 CAN 传输隔离回归。
 * 复用邮箱模拟，验证摩擦轮许可无法开启，拨盘许可和固定停止帧仍可使用。
 */
#define test_run normal_feeder_transport_fixture
#include "test_feeder_can.c"
#undef test_run

void test_run(void)
{
    uint32_t before;
    test_stage = 1U;
    CanBus_CreateQueue(); CanBus_Start();
    CHECK(APP_FEEDER_STANDALONE_TEST_ENABLED);
    CanBus_SetOutputPermission(1U);
    CHECK(!CanBus_OutputAllowed()); before = sends;
    CHECK(CanBus_JogVescCurrent(70U, 1) == HAL_ERROR);
    CHECK(CanBus_JogVescCurrent(71U, 1) == HAL_ERROR && sends == before);
    CanBus_SetFeederPermission(1U); CHECK(CanBus_FeederAllowed());
    CHECK(CanBus_FeederCurrent(-3000) == HAL_OK); CheckFeeder(sends - 1U, -3000);
    CompleteAll(); CanBus_SetOutputPermission(1U);
    CHECK(!CanBus_OutputAllowed() && CanBus_FeederAllowed());
    CHECK(CanBus_StopFlywheels() == HAL_OK);
    CHECK(CanBus_FeederAllowed()); /* Fixed wheel zero must not revoke feeder ownership. */
    CompleteAll(); CHECK(CanBus_FeederCurrent(1500) == HAL_OK);
    CheckFeeder(sends - 1U, 1500); CompleteAll();
    CanBus_SetFeederPermission(0U); before = sends;
    CHECK(CanBus_FeederCurrent(1) == HAL_ERROR && sends == before);
    CheckHealthy(); test_done = 1U;
}
