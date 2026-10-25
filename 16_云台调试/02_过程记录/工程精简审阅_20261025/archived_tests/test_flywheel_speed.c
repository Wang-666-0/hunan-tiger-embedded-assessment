#include "flywheel_speed.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

static FlywheelSpeed speed, negative;

void test_run(void)
{
    uint32_t time, saved_peak, saved_band;
    int32_t current, previous;
    float integral, target, plant, gain;

    test_stage = 1U;
    FlywheelSpeed_Init(&speed);
    CHECK(!speed.running && speed.requested_mA == 0);
    CHECK(FlywheelSpeed_Step(&speed, 100U, 0, 100U, 500) == 0);
    FlywheelSpeed_Init(0); FlywheelSpeed_Stop(0);
    FlywheelSpeed_Start(0, 0U, 0, 0U); FlywheelSpeed_RecordFeedback(0, 0, 0U);
    CHECK(FlywheelSpeed_Step(0, 0U, 0, 0U, 500) == 0);
    FlywheelSpeed_Start(&speed, 100U, 0, 100U);
    CHECK(speed.running && speed.target_erpm == 0.0f);
    CHECK(FlywheelSpeed_Step(&speed, 119U, 0, 119U, 500) == 0);
    CHECK(FlywheelSpeed_Step(&speed, 120U, 0, 120U, 500) == 3);
    CHECK(speed.target_erpm > 13.9f && speed.target_erpm < 14.1f);
    integral = speed.integral_mA; target = speed.target_erpm;
    CHECK(FlywheelSpeed_Step(&speed, 140U, 0, 120U, 500) == 3);
    CHECK(speed.integral_mA == integral && speed.target_erpm == target);
    CHECK(FlywheelSpeed_Step(&speed, 160U, 0, 160U, 500) == 10);
    CHECK(speed.target_erpm > 41.9f && speed.target_erpm < 42.1f);

    test_stage = 2U;
    /* 软上界不用等满20ms，且不锁存或结束会话。 */
    CHECK(FlywheelSpeed_Step(&speed, 162U, 1600, 162U, 500) == 0);
    CHECK(speed.running && speed.coasting && speed.integral_mA == 0.0f);
    CHECK(speed.peak_erpm == 1600U);
    CHECK(FlywheelSpeed_Step(&speed, 164U, 1401, 164U, 500) == 0 && speed.coasting);
    CHECK(FlywheelSpeed_Step(&speed, 166U, 1400, 166U, 500) == 0 && !speed.coasting);
    current = FlywheelSpeed_Step(&speed, 186U, 1000, 186U, 500);
    CHECK(current == 0 && !speed.coasting && speed.running);
    CHECK(speed.filtered_erpm > 1000.0f); /* PI滤波没有绕过原始峰值保护。 */
    FlywheelSpeed_RecordFeedback(&speed, -2001, 186U);
    CHECK(speed.peak_erpm == 2001U);
    CHECK(FlywheelSpeed_Step(&speed, 187U, (-2147483647 - 1), 187U, 500) == 0);
    CHECK(speed.peak_erpm == 2147483648UL && speed.coasting);

    test_stage = 3U;
    FlywheelSpeed_Start(&speed, 0U, 0, 0U);
    previous = 0;
    for (time = 20U; time <= 12000U; time += 20U) {
        current = FlywheelSpeed_Step(&speed, time, 0, time, 500);
        CHECK(current >= 0 && current <= 500);
        CHECK(current <= previous + 100);
        CHECK(speed.target_erpm <= 1400.0f);
        CHECK(speed.integral_mA >= 0.0f && speed.integral_mA <= 500.0f);
        previous = current;
    }
    CHECK(current == 500 && speed.at_limit && speed.limited_ms > 500U);
    integral = speed.integral_mA;
    for (time = 12020U; time <= 14000U; time += 20U) {
        CHECK(FlywheelSpeed_Step(&speed, time, 0, time, 500) == 500);
    }
    CHECK(speed.integral_mA == integral); /* 饱和时没有积累到500mA积分。 */
    CHECK(FlywheelSpeed_Step(&speed, 14001U, 0, 14000U, 100) == 100);
    CHECK(speed.integral_mA <= 100.0f);
    CHECK(FlywheelSpeed_Step(&speed, 14020U, 0, 14020U, 1000) <= 500);
    CHECK(FlywheelSpeed_Step(&speed, 14021U, 0, 14020U, 0) == 0);
    CHECK(!speed.running && speed.integral_mA == 0.0f);
    CHECK(FlywheelSpeed_Step(&speed, 14040U, 0, 14040U, 500) == 0);

    test_stage = 4U;
    FlywheelSpeed_Start(&speed, 0U, 1400, 0U);
    for (time = 20U; time <= 3000U; time += 20U) {
        CHECK(FlywheelSpeed_Step(&speed, time, 1400, time, 500) == 0);
    }
    CHECK(speed.in_band && speed.longest_band_ms >= 500U);
    saved_peak = speed.peak_erpm; saved_band = speed.longest_band_ms;
    FlywheelSpeed_Stop(&speed);
    CHECK(!speed.running && !speed.in_band && speed.requested_mA == 0);
    CHECK(speed.peak_erpm == saved_peak && speed.longest_band_ms == saved_band);
    CHECK(speed.run_ms == 3000U);
    FlywheelSpeed_Start(&speed, 4000U, 0, 4000U);
    CHECK(speed.peak_erpm == 0U && speed.longest_band_ms == 0U && speed.run_ms == 0U);
    CHECK(speed.integral_mA == 0.0f && speed.target_erpm == 0.0f);

    test_stage = 5U;
    FlywheelSpeed_Start(&speed, 0xFFFFFFF0UL, 0, 0xFFFFFFF0UL);
    current = FlywheelSpeed_Step(&speed, 16U, 0, 16U, 500);
    CHECK(current > 0 && current <= 160 && speed.run_ms == 32U);
    FlywheelSpeed_Start(&speed, 100U, 0, 100U);
    CHECK(FlywheelSpeed_Step(&speed, 1100U, 0, 1100U, 500) == 3);
    CHECK(speed.target_erpm < 15.0f); /* 不补发一秒斜坡/积分。 */
    FlywheelSpeed_Start(&speed, 0U, 0, 0U);
    FlywheelSpeed_Start(&negative, 0U, 0, 0U);
    for (time = 20U; time <= 4000U; time += 20U) {
        int32_t measurement = (int32_t)(time / 4U);
        current = FlywheelSpeed_Step(&speed, time, measurement, time, 500);
        CHECK(FlywheelSpeed_Step(&negative, time, -measurement, time, 500) == current);
        CHECK(speed.filtered_erpm == negative.filtered_erpm);
    }

    test_stage = 6U;
    /* 可控的一阶虚拟负载仅验证闭环方向、限幅和负载改变后的恢复，非实机模型。 */
    FlywheelSpeed_Start(&speed, 0U, 0, 0U); plant = 0.0f; previous = 0;
    for (time = 20U; time <= 40000U; time += 20U) {
        gain = time < 20000U ? 6.0f : 4.0f;
        current = FlywheelSpeed_Step(&speed, time, (int32_t)plant, time, 500);
        CHECK(current >= 0 && current <= 500 && current <= previous + 100);
        plant += (gain * (float)current - plant) * 0.02f / 0.30f;
        CHECK(plant >= 0.0f && plant < 2000.0f);
        if (time == 19000U) { CHECK(plant > 1260.0f && plant < 1540.0f); }
        previous = current;
    }
    CHECK(plant > 1260.0f && plant < 1540.0f && speed.in_band);
    CHECK(speed.longest_band_ms >= 500U);
    test_done = 1U;
}
