#include "flywheel_speed.h"

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static FlywheelSpeed speed;

void test_run(void)
{
    uint32_t time, first_band, saved_rise;
    int32_t current, previous;
    float integral, plant, gain;

    test_stage = 1U;
    CHECK(APP_FLYWHEEL_SPEED_TARGET_ERPM == 42000 && APP_FLYWHEEL_JOG_MAX_MA == 2000);
    CHECK(APP_FLYWHEEL_SPEED_COAST_ERPM == 44100 && APP_FLYWHEEL_JOG_HARD_ERPM == 46200);
    CHECK(APP_FLYWHEEL_JOG_FEEDBACK_MAX_MA == 3000 && APP_FLYWHEEL_SPEED_BAND_ERPM == 700);
    FlywheelSpeed_Start(&speed, 0U, 0, 0U); previous = 0;
    for (time = 2U; time <= 8000U; time += 2U) {
        current = FlywheelSpeed_Step(&speed, time, 0, time - time % 20U, 2000);
        CHECK(speed.running && current >= 0 && current <= 2000);
        CHECK(current <= previous + (time % 20U == 0U ? 100 : 0));
        CHECK(speed.target_erpm <= 42000.0f);
        if (time == 20U) { CHECK(current == 100); }
        if (time == 400U) { CHECK(current == 2000); }
        if (time == 980U) { CHECK(speed.target_erpm < 42000.0f); }
        if (time == 1000U) { CHECK(speed.target_erpm == 42000.0f); }
        previous = current;
    }
    CHECK(current == 2000 && speed.at_limit && speed.limited_ms > 1000U);
    CHECK(speed.low_speed_ms >= 7000U && !speed.rise90_seen && !speed.band_first_seen);
    integral = speed.integral_mA;
    CHECK(FlywheelSpeed_Step(&speed, 8020U, 0, 8020U, 20000) == 2000);
    CHECK(speed.integral_mA == integral); /* 不能因速度不足堆积积分或突破2A。 */
    CHECK(FlywheelSpeed_Step(&speed, 8022U, 44100, 8022U, 2000) == 0);
    CHECK(speed.coasting && speed.running && speed.integral_mA == 0.0f);
    CHECK(FlywheelSpeed_Step(&speed, 8042U, 42001, 8042U, 2000) == 0 && speed.coasting);
    CHECK(FlywheelSpeed_Step(&speed, 8062U, 42000, 8062U, 2000) == 0 && !speed.coasting);
    CHECK(speed.coast_ms == 40U);

    test_stage = 2U;
    /* T90以有效反馈接收时间计时，不能拿较晚的控制调用时间替代。 */
    FlywheelSpeed_Start(&speed, 1000U, 0, 990U);
    FlywheelSpeed_RecordFeedback(&speed, 37800, 999U); CHECK(!speed.rise90_seen);
    FlywheelSpeed_RecordFeedback(&speed, 37799, 1200U); CHECK(!speed.rise90_seen);
    FlywheelSpeed_RecordFeedback(&speed, -37800, 1234U);
    CHECK(speed.rise90_seen && speed.rise90_ms == 234U);
    (void)FlywheelSpeed_Step(&speed, 1250U, -37800, 1234U, 2000);
    FlywheelSpeed_RecordFeedback(&speed, 42000, 1300U);
    CHECK(speed.rise90_ms == 234U && speed.peak_erpm == 42000U);
    FlywheelSpeed_Stop(&speed);
    FlywheelSpeed_RecordFeedback(&speed, 45000, 1400U);
    CHECK(speed.rise90_seen && speed.rise90_ms == 234U && speed.peak_erpm == 42000U);
    FlywheelSpeed_Start(&speed, 1500U, 0, 1500U);
    CHECK(!speed.rise90_seen && !speed.band_first_seen);

    test_stage = 3U;
    FlywheelSpeed_Start(&speed, 0U, 42000, 0U);
    CHECK(speed.rise90_seen && speed.rise90_ms == 0U);
    for (time = 20U; time <= 2000U; time += 20U) {
        CHECK(FlywheelSpeed_Step(&speed, time, 42000, time, 2000) == 0);
    }
    CHECK(speed.in_band && speed.band_first_seen && speed.band_first_ms == 1480U);
    first_band = speed.band_first_ms; saved_rise = speed.rise90_ms;
    CHECK(FlywheelSpeed_Step(&speed, 2001U, 44100, 2001U, 2000) == 0);
    CHECK(!speed.in_band && speed.band_first_seen && speed.band_first_ms == first_band);
    FlywheelSpeed_Stop(&speed);
    CHECK(speed.band_first_ms == first_band && speed.rise90_ms == saved_rise);
    CHECK(!speed.running && speed.requested_mA == 0 && speed.integral_mA == 0.0f);
    FlywheelSpeed_Start(&speed, 0xFFFFFFF0UL, 0, 0xFFFFFFE0UL);
    FlywheelSpeed_RecordFeedback(&speed, 37800, 16U);
    CHECK(speed.rise90_seen && speed.rise90_ms == 32U);

    test_stage = 4U;
    /* 可控虚拟负载验证快速目标下的限幅、闭环方向和负载改变后的恢复。
     * 此模型不是实机参数，不能用这里的T90代替实际加速成绩。 */
    FlywheelSpeed_Start(&speed, 0U, 0, 0U); plant = 0.0f; previous = 0;
    for (time = 20U; time <= 40000U; time += 20U) {
        gain = time < 20000U ? 30.0f : 24.0f;
        current = FlywheelSpeed_Step(&speed, time, (int32_t)plant, time, 2000);
        CHECK(current >= 0 && current <= 2000 && current <= previous + 100);
        plant += (gain * (float)current - plant) * 0.02f / 0.80f;
        CHECK(plant >= 0.0f && plant < 46200.0f);
        if (time == 19000U) { CHECK(plant > 41300.0f && plant < 42700.0f); }
        previous = current;
    }
    CHECK(plant > 41300.0f && plant < 42700.0f && speed.in_band);
    CHECK(speed.rise90_seen && speed.band_first_seen && speed.rise90_ms < speed.band_first_ms);
    CHECK(speed.longest_band_ms >= 500U);
    test_done = 1U;
}
