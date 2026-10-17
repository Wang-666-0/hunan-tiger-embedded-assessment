#include "flywheel_speed.h"

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static FlywheelSpeed speed;

void test_run(void)
{
    uint32_t time, coast, zero, low;
    int32_t current, previous;
    float plant, gain, integral;

    test_stage = 1U;
    CHECK(APP_FLYWHEEL_SPEED_TARGET_ERPM == 7000);
    CHECK(APP_FLYWHEEL_JOG_MAX_MA == 1000);
    FlywheelSpeed_Start(&speed, 0U, 0, 0U); previous = 0;
    /* 模拟2ms控制任务、20ms新反馈；同一反馈不能反复推进斜坡/积分。 */
    for (time = 2U; time <= 8000U; time += 2U) {
        current = FlywheelSpeed_Step(&speed, time, 0, time - time % 20U, 1000);
        CHECK(speed.running && current >= 0 && current <= 1000);
        CHECK(current <= previous + (time % 20U == 0U ? 100 : 0));
        CHECK(speed.target_erpm <= 7000.0f);
        if (time == 3980U) { CHECK(speed.target_erpm < 7000.0f); }
        if (time == 4000U) { CHECK(speed.target_erpm == 7000.0f); }
        previous = current;
    }
    CHECK(current == 1000 && speed.at_limit && speed.limited_ms > 1000U);
    CHECK(speed.low_speed_ms >= 4000U && speed.zero_current_ms == 20U);
    integral = speed.integral_mA;
    CHECK(FlywheelSpeed_Step(&speed, 8020U, 0, 8020U, 5000) == 1000);
    CHECK(speed.integral_mA == integral); /* 电流受限时不会无限堆积积分。 */

    test_stage = 2U;
    CHECK(FlywheelSpeed_Step(&speed, 8022U, 8000, 8022U, 1000) == 0);
    CHECK(speed.coasting && speed.running && speed.integral_mA == 0.0f);
    CHECK(FlywheelSpeed_Step(&speed, 8042U, 7001, 8042U, 1000) == 0);
    CHECK(speed.coasting && speed.coast_ms == 20U);
    CHECK(FlywheelSpeed_Step(&speed, 8062U, 7000, 8062U, 1000) == 0);
    CHECK(!speed.coasting && speed.running && speed.coast_ms == 40U);
    CHECK(speed.zero_current_ms == 60U);
    for (time = 8082U; time <= 10002U; time += 20U) {
        current = FlywheelSpeed_Step(&speed, time, 0, time, 1000);
        CHECK(current >= 0 && current <= 1000 && speed.running);
    }
    CHECK(current == 1000 && !speed.coasting);
    coast = speed.coast_ms; zero = speed.zero_current_ms; low = speed.low_speed_ms;
    (void)FlywheelSpeed_Step(&speed, 11002U, 0, 11002U, 1000);
    CHECK(speed.coast_ms == coast && speed.zero_current_ms == zero && speed.low_speed_ms == low);
    FlywheelSpeed_Stop(&speed);
    CHECK(!speed.running && speed.requested_mA == 0 && speed.integral_mA == 0.0f);
    CHECK(speed.coast_ms == coast && speed.zero_current_ms == zero && speed.low_speed_ms == low);
    CHECK(FlywheelSpeed_Step(&speed, 11022U, 0, 11022U, 1000) == 0);

    test_stage = 3U;
    FlywheelSpeed_Start(&speed, 0U, 7000, 0U);
    for (time = 20U; time <= 5000U; time += 20U) {
        CHECK(FlywheelSpeed_Step(&speed, time, 7000, time, 1000) == 0);
    }
    CHECK(speed.in_band && speed.longest_band_ms >= 500U && speed.low_speed_ms == 0U);
    speed.zero_current_ms = UINT32_MAX - 10U;
    (void)FlywheelSpeed_Step(&speed, 5020U, 7000, 5020U, 1000);
    CHECK(speed.zero_current_ms == UINT32_MAX);
    FlywheelSpeed_Start(&speed, 0xFFFFFFF0UL, 0, 0xFFFFFFF0UL);
    CHECK(FlywheelSpeed_Step(&speed, 16U, 0, 16U, 1000) > 0);
    CHECK(speed.run_ms == 32U && speed.zero_current_ms == 32U);

    test_stage = 4U;
    /* 两种可控虚拟负载，验证1000rpm档的闭环方向与负载变化后恢复。
     * 该模型不代表未知实机电机，也不验收真实机械转速。 */
    FlywheelSpeed_Start(&speed, 0U, 0, 0U); plant = 0.0f; previous = 0;
    for (time = 20U; time <= 40000U; time += 20U) {
        gain = time < 20000U ? 14.0f : 10.0f;
        current = FlywheelSpeed_Step(&speed, time, (int32_t)plant, time, 1000);
        CHECK(current >= 0 && current <= 1000 && current <= previous + 100);
        plant += (gain * (float)current - plant) * 0.02f / 0.30f;
        CHECK(plant >= 0.0f && plant < 9000.0f);
        if (time == 19000U) { CHECK(plant > 6650.0f && plant < 7350.0f); }
        previous = current;
    }
    CHECK(plant > 6650.0f && plant < 7350.0f && speed.in_band);
    CHECK(speed.longest_band_ms >= 500U);
    test_done = 1U;
}
