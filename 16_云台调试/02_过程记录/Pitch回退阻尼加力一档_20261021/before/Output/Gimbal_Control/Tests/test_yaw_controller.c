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

void test_run(void)
{
    float torque, saved, integral, previous;
    uint32_t time, sequence;
    union { uint32_t bits; float value; } special;
    test_stage = 1U;
    YawController_Init(0); YawController_Stop(0);
    CHECK(YawController_Update(0, 0.0f, 0U, 1U) == 0.0f);
    CHECK(!YawController_OnFeedback(0, &feedback));
    CHECK(!YawController_CaptureReference(0));
    ready(100U, 0.0f);
    CHECK(near(yaw.pid.kp, 1.0f, 0.0001f) && yaw.pid.ki == 0.0f);
    CHECK(near(yaw.pid.kd, 0.015f, 0.0001f));
    CHECK(YawController_SetTarget(&yaw, 2.0f));
    torque = YawController_Update(&yaw, 0.0f, 120U, 1U);
    CHECK(torque > 0.0f && torque <= YAW_CONTROLLER_TORQUE_LIMIT_NM && yaw.active);
    CHECK(YawController_SetTarget(&yaw, -2.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 122U, 1U) < 0.0f);
    CHECK(!YawController_SetTarget(&yaw, 30.01f) && yaw.target_deg == -2.0f);
    CHECK(YawController_SetTarget(&yaw, 5.0f));
    CHECK(YawController_SetPid(&yaw, 2.0f, 1.0f, 0.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 124U, 1U) == YAW_CONTROLLER_TORQUE_LIMIT_NM);
    CHECK(!YawController_CaptureReference(&yaw));
    ready(100U, 0.0f);
    CHECK(YawController_SetPid(&yaw, 0.0f, 0.0f, 0.2f));
    frame(140U, 0.0f, 0.8f); /* 45.8deg/s，低于速度保护但阻尼输出超过0.10。 */
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(YawController_Update(&yaw, 0.0f, 140U, 1U) == -YAW_CONTROLLER_TORQUE_LIMIT_NM);
    frame(142U, 0.0f, -0.8f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(YawController_Update(&yaw, 0.0f, 142U, 1U) == YAW_CONTROLLER_TORQUE_LIMIT_NM);

    test_stage = 2U;
    special.bits = 0x7FC00000U;
    saved = yaw.pid.kp;
    CHECK(!YawController_SetPid(&yaw, special.value, 0.0f, 0.0f));
    CHECK(yaw.pid.kp == saved);
    CHECK(!YawController_SetPid(&yaw, 2.01f, 0.0f, 0.0f));
    CHECK(!YawController_SetPid(&yaw, 0.35f, -0.01f, 0.0f));
    CHECK(!YawController_SetPid(&yaw, 0.35f, 0.0f, 0.201f));
    CHECK(!YawController_SetTarget(&yaw, special.value));
    special.bits = 0x7F800000U;
    CHECK(!YawController_SetPid(&yaw, 0.35f, special.value, 0.0f));
    CHECK(!YawController_SetTarget(&yaw, special.value));
    CHECK(YawController_Update(&yaw, special.value, 126U, 1U) == 0.0f);
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_DATA);
    CHECK(!yaw.active && yaw.integral == 0.0f);

    test_stage = 3U;
    ready(100U, PI_RAD - 0.01f);
    frame(140U, -PI_RAD + 0.01f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(near(yaw.angle_deg, 0.02f / DEG_TO_RAD, 0.002f));
    CHECK(yaw.faults == 0U);
    frame(160U, PI_RAD - 0.01f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(near(yaw.angle_deg, 0.0f, 0.002f));
    sequence = yaw.last_sequence;
    frame(150U, -1.0f, 0.0f);
    CHECK(!YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.last_sequence == sequence && yaw.last_feedback_ms == 160U);
    frame(160U, -1.0f, 0.0f);
    CHECK(!YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults == 0U);

    test_stage = 4U;
    ready(100U, 0.0f);
    CHECK(YawController_SetTarget(&yaw, 2.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 120U, 1U) > 0.0f);
    YawController_Stop(&yaw);
    CHECK(!yaw.active && yaw.torque_nm == 0.0f && yaw.integral == 0.0f);
    CHECK(yaw.target_deg == yaw.angle_deg);
    CHECK(YawController_Update(&yaw, 0.0f, 122U, 0U) == 0.0f);
    YawController_Update(&yaw, 0.0f, 219U, 1U);
    CHECK(yaw.faults == 0U); /* 接收年龄99ms尚有效，100ms边界撤权。 */
    CHECK(YawController_Update(&yaw, 0.0f, 220U, 1U) == 0.0f);
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_FEEDBACK);
    frame(220U, 1.0f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(!yaw.have_reference && yaw.angle_deg == 0.0f && !yaw.active);
    CHECK(!YawController_SetTarget(&yaw, 1.0f));
    frame(240U, 1.0f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(YawController_CaptureReference(&yaw));
    CHECK(yaw.faults == 0U);
    CHECK(YawController_SetTarget(&yaw, 1.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 240U, 1U) > 0.0f);
    frame(242U, 1.2f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_JUMP);
    CHECK(yaw.torque_nm == 0.0f && !yaw.active && !yaw.have_reference);

    test_stage = 5U;
    ready(100U, 0.0f);
    frame(140U, 0.0f, 0.0f); feedback.driver_temp_c = 60;
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_TEMPERATURE);
    CHECK(!YawController_CaptureReference(&yaw));
    frame(160U, 0.0f, 0.0f); feedback.motor_temp_c = 70;
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_TEMPERATURE);
    frame(180U, 0.0f, 0.0f); feedback.status = 4U;
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_STATUS);
    frame(200U, 0.0f, 61.0f * DEG_TO_RAD);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_SPEED);
    frame(220U, 0.0f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    frame(240U, 0.0f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(YawController_CaptureReference(&yaw) && yaw.faults == 0U);
    yaw.faults |= 0x40000000U;
    CHECK(!YawController_CaptureReference(&yaw));
    CHECK(yaw.faults == 0x40000000U);
    ready(100U, 0.0f);
    frame(140U, 0.0f, 0.0f); feedback.fault_epoch = 1U;
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_CAN);
    CHECK(!YawController_CaptureReference(&yaw));

    test_stage = 6U;
    ready(100U, 0.0f);
    CHECK(YawController_SetPid(&yaw, 0.0f, 0.1f, 0.0f));
    CHECK(YawController_SetTarget(&yaw, 2.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 120U, 1U) == 0.0f);
    frame(140U, 0.0f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    /* 未来接收时间不可用于放行；下一段重新建立正常时间序列。 */
    CHECK(YawController_Update(&yaw, 0.0f, 122U, 1U) == 0.0f);
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_FEEDBACK);
    ready(100U, 0.0f);
    CHECK(YawController_SetPid(&yaw, 0.0f, 0.1f, 0.0f));
    CHECK(YawController_SetTarget(&yaw, 2.0f));
    YawController_Update(&yaw, 0.0f, 120U, 1U);
    for (time = 122U; time <= 140U; time += 2U) {
        if (time == 140U) {
            frame(time, 0.0f, 0.0f);
            CHECK(YawController_OnFeedback(&yaw, &feedback));
        }
        YawController_Update(&yaw, 0.0f, time, 1U);
    }
    CHECK(yaw.integral > 0.0f);
    integral = yaw.integral;
    torque = yaw.torque_nm;
    for (time = 142U; time <= 158U; time += 2U) {
        CHECK(YawController_Update(&yaw, 0.0f, time, 1U) == torque);
        CHECK(yaw.integral == integral);
    }
    CHECK(YawController_SetPid(&yaw, 2.0f, 1.0f, 0.0f));
    CHECK(YawController_SetTarget(&yaw, 5.0f));
    for (time = 160U; time <= 220U; time += 2U) {
        if (time % 10U == 0U) {
            frame(time, 0.0f, 0.0f);
            CHECK(YawController_OnFeedback(&yaw, &feedback));
        }
        CHECK(YawController_Update(&yaw, 0.0f, time, 1U) == YAW_CONTROLLER_TORQUE_LIMIT_NM);
        CHECK(yaw.integral == 0.0f); /* 饱和误差不会不断积累。 */
    }

    test_stage = 7U;
    ready(100U, 0.0f);
    YawController_Update(&yaw, 99.0f, 120U, 1U);
    previous = yaw.target_deg;
    for (time = 122U; time <= 3220U; time += 2U) {
        if (time % 10U == 0U) {
            frame(time, 0.0f, 0.0f);
            CHECK(YawController_OnFeedback(&yaw, &feedback));
        }
        torque = YawController_Update(&yaw, 99.0f, time, 1U);
        CHECK(absolute(torque) <= YAW_CONTROLLER_TORQUE_LIMIT_NM);
        CHECK(yaw.target_deg >= previous && yaw.target_deg - previous <= 0.0201f);
        CHECK(yaw.target_deg <= YAW_CONTROLLER_TARGET_LIMIT_DEG && yaw.faults == 0U);
        previous = yaw.target_deg;
    }
    CHECK(yaw.target_deg == YAW_CONTROLLER_TARGET_LIMIT_DEG);
    CHECK(YawController_Update(&yaw, 10.0f, 3231U, 1U) == 0.0f);
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_TIMING);
    CHECK(!yaw.active);

    test_stage = 8U;
    ready(100U, 0.0f);
    for (time = 140U; time <= 820U; time += 20U) {
        frame(time, (float)(time - 120U) * 0.06f * DEG_TO_RAD, 60.0f * DEG_TO_RAD);
        CHECK(YawController_OnFeedback(&yaw, &feedback));
    }
    CHECK(yaw.angle_deg > YAW_CONTROLLER_HARD_LIMIT_DEG && (yaw.faults & YAW_CONTROLLER_FAULT_LIMIT));
    CHECK(!yaw.active && yaw.torque_nm == 0.0f);
    CHECK(YawController_Update(&yaw, 0.0f, 820U, 1U) == 0.0f);
    ready(0xFFFFFFE0UL, 0.0f);
    frame(8U, 0.0f, 0.0f);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults == 0U && yaw.last_feedback_ms == 8U);
    CHECK(YawController_SetTarget(&yaw, -2.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 8U, 1U) < 0.0f);
    frame(10U, 0.0f, 0.0f);
    special.bits = 0x7FC00000U; feedback.position_rad = special.value;
    CHECK(!YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_DATA);
    CHECK(yaw.last_feedback_ms == 8U && yaw.torque_nm == 0.0f);
    test_stage = 9U;
    ready(5000U, 0.0f); yaw.motion_guard_active = 0U;
    frame(5030U, 2.0f * DEG_TO_RAD, 100.0f * DEG_TO_RAD);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.faults == 0U && near(yaw.angle_deg, 2.0f, 0.001f));
    CHECK(YawController_Update(&yaw, 0.0f, 5030U, 0U) == 0.0f && !yaw.motion_guard_active);
    CHECK(YawController_Update(&yaw, 0.0f, 5030U, 1U) == 0.0f);
    CHECK(yaw.motion_guard_active && (yaw.faults & YAW_CONTROLLER_FAULT_SPEED) && !yaw.active);

    ready(5100U, 0.0f); yaw.motion_guard_active = 0U;
    for (time = 5130U; time <= 5330U; time += 10U) {
        frame(time, (float)(time - 5120U) * 0.2f * DEG_TO_RAD, 0.0f);
        CHECK(YawController_OnFeedback(&yaw, &feedback));
        CHECK(yaw.faults == 0U && !yaw.active && yaw.torque_nm == 0.0f);
    }
    CHECK(near(yaw.angle_deg, 42.0f, 0.001f));
    CHECK(YawController_Update(&yaw, 0.0f, 5330U, 1U) == 0.0f);
    CHECK(yaw.motion_guard_active && (yaw.faults & YAW_CONTROLLER_FAULT_LIMIT));

    ready(5300U, 0.0f); yaw.motion_guard_active = 0U;
    frame(5330U, 10.0f * DEG_TO_RAD, 400.0f * DEG_TO_RAD);
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK((yaw.faults & (YAW_CONTROLLER_FAULT_JUMP | YAW_CONTROLLER_FAULT_SPEED)) ==
        (YAW_CONTROLLER_FAULT_JUMP | YAW_CONTROLLER_FAULT_SPEED));
    CHECK(!yaw.active && yaw.torque_nm == 0.0f);

    ready(5400U, 0.0f); yaw.motion_guard_active = 0U;
    frame(5430U, 2.0f * DEG_TO_RAD, 100.0f * DEG_TO_RAD); feedback.status = 1U;
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.motion_guard_active && (yaw.faults & YAW_CONTROLLER_FAULT_SPEED));
    YawController_Stop(&yaw);
    frame(5440U, 2.0f * DEG_TO_RAD, 100.0f * DEG_TO_RAD); feedback.status = 13U;
    CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.motion_guard_active && !yaw.active && yaw.torque_nm == 0.0f);

    ready(5500U, 0.0f); yaw.motion_guard_active = 0U;
    for (time = 5530U; time <= 5680U; time += 10U) {
        frame(time, (float)(time - 5520U) * 0.3f * DEG_TO_RAD, 0.0f);
        CHECK(YawController_OnFeedback(&yaw, &feedback));
    }
    CHECK(yaw.angle_deg > YAW_CONTROLLER_OFF_ANGLE_LIMIT_DEG);
    CHECK(yaw.faults & YAW_CONTROLLER_FAULT_LIMIT);
    test_stage = 10U;
    ready(6000U, 0.0f);
    CHECK(YawController_SetTarget(&yaw, 15.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 6020U, 1U) == YAW_CONTROLLER_TORQUE_LIMIT_NM);
    CHECK(yaw.active && yaw.integral == 0.0f && yaw.faults == 0U);
    CHECK(!YawController_SetTarget(&yaw, 30.001f) && yaw.target_deg == 15.0f);
    CHECK(YawController_SetTarget(&yaw, -15.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 6022U, 1U) == -YAW_CONTROLLER_TORQUE_LIMIT_NM);
    CHECK(!YawController_SetTarget(&yaw, -30.001f) && yaw.target_deg == -15.0f);
    CHECK(YawController_SetTarget(&yaw, 0.0f));
    CHECK(absolute(YawController_Update(&yaw, 0.0f, 6024U, 1U)) < 0.001f);
    YawController_Stop(&yaw);
    CHECK(!yaw.active && yaw.torque_nm == 0.0f && yaw.motion_guard_active);
    ready(6100U, 0.0f);
    yaw.angle_deg = 40.0f; /* 严格等于边界允许；超过边界必须无输出。 */
    CHECK(YawController_Update(&yaw, 0.0f, 6120U, 1U) == -YAW_CONTROLLER_TORQUE_LIMIT_NM);
    yaw.angle_deg = 40.001f;
    CHECK(YawController_Update(&yaw, 0.0f, 6122U, 1U) == 0.0f);
    CHECK((yaw.faults & YAW_CONTROLLER_FAULT_LIMIT) && !yaw.active);
    ready(6200U, 0.0f); yaw.angle_deg = -40.001f;
    CHECK(YawController_Update(&yaw, 0.0f, 6220U, 1U) == 0.0f);
    CHECK((yaw.faults & YAW_CONTROLLER_FAULT_LIMIT) && !yaw.active);
    test_stage = 11U;
    ready(6300U, 0.0f); CHECK(YawController_SetTarget(&yaw, 30.0f));
    frame(6330U, 0.0f, 50.0f * DEG_TO_RAD); CHECK(YawController_OnFeedback(&yaw, &feedback));
    torque = YawController_Update(&yaw, 0.0f, 6330U, 1U);
    CHECK(yaw.speed_limited && torque < 0.0f && torque >= -YAW_CONTROLLER_TORQUE_LIMIT_NM && yaw.faults == 0U);
    CHECK(YawController_SetPid(&yaw, 0.0f, 0.2f, 0.0f)); yaw.integral = 0.1f;
    frame(6340U, 0.0f, 50.0f * DEG_TO_RAD); CHECK(YawController_OnFeedback(&yaw, &feedback));
    yaw.have_control_tick = 0U; CHECK(YawController_Update(&yaw, 0.0f, 6340U, 1U) < 0.0f);
    CHECK(yaw.integral == 0.1f);
    ready(6400U, 0.0f); CHECK(YawController_SetTarget(&yaw, -30.0f));
    frame(6430U, 0.0f, -50.0f * DEG_TO_RAD); CHECK(YawController_OnFeedback(&yaw, &feedback));
    torque = YawController_Update(&yaw, 0.0f, 6430U, 1U);
    CHECK(yaw.speed_limited && torque > 0.0f && torque <= YAW_CONTROLLER_TORQUE_LIMIT_NM && yaw.faults == 0U);
    frame(6440U, 0.0f, 61.0f * DEG_TO_RAD); CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK((yaw.faults & YAW_CONTROLLER_FAULT_SPEED) && !yaw.active && yaw.torque_nm == 0.0f);
    test_done = 1U;
}
