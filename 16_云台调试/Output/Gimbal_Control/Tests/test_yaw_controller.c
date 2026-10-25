#include "yaw_controller.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
#define PI_RAD 3.141593f
#define DEG_TO_RAD (PI_RAD / 180.0f)

static YawController yaw;
static Gl60Feedback feedback;
static float absolute(float value) { return value < 0.0f ? -value : value; }
static uint8_t near(float first, float second, float tolerance)
{
    return (uint8_t)(absolute(first - second) < tolerance);
}

static void frame(uint32_t time, float position, float velocity)
{
    memset(&feedback, 0, sizeof(feedback));
    feedback.motor_id = 1U;
    feedback.driver_temp_c = 25;
    feedback.motor_temp_c = 25;
    feedback.position_rad = position;
    feedback.velocity_rad_s = velocity;
    feedback.received_ms = time;
}

static void ready(uint32_t time, float position)
{
    YawController_Init(&yaw);
    frame(time, position, 0.04884f); /* MIT零速量化约2.8deg/s，允许静止。 */
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(!YawController_CaptureReference(&yaw));
    frame(time + 20U, position, -0.04884f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(YawController_CaptureReference(&yaw));
    CHECK(yaw.have_reference && yaw.faults == 0U && !yaw.active);
}
