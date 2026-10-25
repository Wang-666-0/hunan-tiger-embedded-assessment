/* Actual controller with fresh encoded feedback. These illustrative motor
 * models are robustness regressions, not identified hardware or test results. */
#include "feeder_controller.h"
#include <math.h>
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; if (!test_failed_line) test_failed_line = __LINE__; } } while (0)
static FeederController c;
static CanFrame frame;
volatile float model_old_overshoot[8], model_new_overshoot[8];
volatile uint32_t model_old_ms[8], model_new_ms[8];

static void feed(uint32_t ms, float position, float rpm)
{
    int64_t counts = (int64_t)((double)position * (8192.0 / 360.0));
    uint16_t angle = (uint16_t)((counts % 8192 + 8192) % 8192);
    int16_t speed = (int16_t)rpm;
    memset(&frame, 0, sizeof(frame));
    frame.bus = 2U; frame.id = 0x205U; frame.dlc = 8U; frame.received_ms = ms;
    frame.data[0] = (uint8_t)(angle >> 8); frame.data[1] = (uint8_t)angle;
    frame.data[2] = (uint8_t)((uint16_t)speed >> 8); frame.data[3] = (uint8_t)speed;
    CHECK(Feeder_OnFrame(&c, &frame, ms, 0U));
}

static void prepare(uint32_t base, int8_t direction, uint8_t previous)
{
    FeederConfig config;
    Feeder_Init(&c); config = c.config; config.direction = direction;
    if (previous) config.single_decel_rpm_s = 0.0f;
    CHECK(Feeder_SetConfig(&c, &config));
    feed(base, 0.0f, 0.0f); feed(base + 1U, 0.0f, 0.0f); feed(base + 2U, 0.0f, 0.0f);
    CHECK(Feeder_Zero(&c, base + 2U));
    CHECK(Feeder_Step(&c, base + 2U, 1U, 1U, 1U, 1U) == direction * 3000);
}

static void model(uint32_t slot, uint8_t previous, int8_t direction,
                  float gain, float drag, float lag_s)
{
    uint32_t t, base = 1000U + slot * 3000U + previous * 1500U;
    float position = 0.0f, rpm = 0.0f, actuator = 0.0f, old_rpm, peak = 0.0f, over;
    prepare(base, direction, previous);
    for (t = 2U; t <= 1000U && c.busy; t += 2U) {
        actuator = lag_s == 0.0f ? (float)c.command_raw :
            actuator + 0.002f / (lag_s + 0.002f) * ((float)c.command_raw - actuator);
        old_rpm = rpm;
        rpm += 0.002f * (gain * actuator - drag * rpm);
        position += 3.0f * (rpm + old_rpm) * 0.002f;
        over = position * direction / c.config.gear_ratio - 40.0f;
        if (over > peak) peak = over;
        feed(base + 2U + t, position, rpm);
        (void)Feeder_Step(&c, base + 2U + t, 1U, 1U, 0U, 0U);
        CHECK(!c.faults && c.command_raw >= -3000 && c.command_raw <= 3000);
        CHECK(fabsf(c.target_rpm) <= c.config.max_rpm + 0.01f);
        CHECK(c.accepted == 1U && c.outstanding <= 1U && !c.rejected);
    }
    CHECK(!c.faults && c.completed == 1U && c.single_completed == 1U && !c.busy);
    CHECK(fabs(c.axis_target_deg - direction * 40.0) < 0.001);
    if (previous) { model_old_overshoot[slot] = peak; model_old_ms[slot] = c.last_motion_ms; }
    else {
        model_new_overshoot[slot] = peak; model_new_ms[slot] = c.last_motion_ms;
        CHECK(c.single_brake_frames > 0U);
    }
}

void test_run(void)
{
    uint32_t i, j, base;
    int8_t direction;
    float integral;
    FeederConfig config;
    const float gains[] = {15.0f, 20.0f, 30.0f, 40.0f};
    const float drags[] = {5.0f, 10.0f, 5.0f, 5.0f};
    const float lags[] = {0.008f, 0.004f, 0.0f, 0.008f};
    test_stage = 1U;
    Feeder_Init(&c); config = c.config;
    CHECK(config.single_decel_rpm_s == 50000.0f);
    config.single_decel_rpm_s = -1.0f; CHECK(!Feeder_ConfigValid(&config));
    config.single_decel_rpm_s = 999.0f; CHECK(!Feeder_ConfigValid(&config));
    config.single_decel_rpm_s = 200001.0f; CHECK(!Feeder_ConfigValid(&config));
    config.single_decel_rpm_s = NAN; CHECK(!Feeder_ConfigValid(&config));
    config.single_decel_rpm_s = 0.0f; CHECK(Feeder_ConfigValid(&config));
    config.single_decel_rpm_s = 1000.0f; CHECK(Feeder_ConfigValid(&config));

    test_stage = 2U;
    for (j = 0U; j < 2U; ++j) {
        direction = j ? 1 : -1; base = 30000U + j * 100U;
        prepare(base, direction, 0U);
        /* Same pose, speed and stored drive: the braking tick unloads I,
         * commands opposing measured motion, and does not repeat without a frame. */
        c.integral = direction * 2000.0f;
        c.accumulated_counts = (int64_t)(direction * 35.0 * c.config.gear_ratio * 8192.0 / 360.0);
        c.raw_angle = (uint16_t)((c.accumulated_counts % 8192 + 8192) % 8192);
        feed(base + 4U, direction * 35.0f * c.config.gear_ratio, direction * 2500.0f);
        (void)Feeder_Step(&c, base + 4U, 1U, 1U, 0U, 0U);
        CHECK(c.single_braking && c.single_brake_frames == 1U && c.command_raw * direction < 0);
        CHECK(fabsf(c.integral) < 2000.0f && c.integral * direction > 0.0f);
        integral = c.integral;
        (void)Feeder_Step(&c, base + 5U, 1U, 1U, 0U, 0U);
        CHECK(c.integral == integral && c.single_brake_frames == 1U);
        CHECK(Feeder_Step(&c, base + 6U, 0U, 1U, 0U, 0U) == 0 && !c.busy && !c.integral);
        CHECK(!c.single_braking && c.aborted == 1U);
    }

    test_stage = 3U;
    for (i = 0U; i < 8U; ++i) {
        direction = i < 4U ? -1 : 1; j = i % 4U;
        model(i, 1U, direction, gains[j], drags[j], lags[j]);
        model(i, 0U, direction, gains[j], drags[j], lags[j]);
        CHECK(model_new_overshoot[i] < model_old_overshoot[i] * 0.7f + 0.3f);
        CHECK(model_new_ms[i] <= model_old_ms[i] + 30U);
    }
    test_done = 1U;
}
