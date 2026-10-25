#define test_run legacy_controller_fixture_not_run
#include "test_yaw_controller.c"
#undef test_run
#include <math.h>

/* 假想惯量/摩擦只验证闭环方向、收敛及状态，不能证明实机性能。 */
static void Plant(float target, uint32_t base)
{
    uint32_t t;
    float position = 0.0f, speed = 0.0f, torque;
    ready(base, 0.0f); CHECK(YawController_SetTarget(&yaw, target));
    for (t = 0U; t <= 5000U; t += 2U) {
        if (t && t % 10U == 0U) {
            float quantized = floorf(position / 0.005493f + 0.5f) * 0.005493f;
            frame(base + 20U + t, quantized * DEG_TO_RAD, (speed > 0.0f ? 2.8f : -2.8f) * DEG_TO_RAD);
            CHECK(YawController_OnFeedback(&yaw, &feedback));
        }
        torque = YawController_Update(&yaw, 0.0f, base + 20U + t, 1U);
        CHECK(yaw.active && !yaw.faults && absolute(torque) <= 0.100001f);
        /* J=.002、粘性阻力、小恒定负载；协议请求在此虚拟模型按Nm用。 */
        speed += (torque - 0.00008f * speed - 0.004f) * 28647.89f * 0.002f;
        position += speed * 0.002f;
    }
    CHECK(absolute(position - target) < 0.2f && absolute(speed) < 1.0f);
    CHECK(absolute(yaw.cascade.speed_i) < 0.02f);
    CHECK(YawController_Update(&yaw, 0.0f, base + 5022U, 0U) == 0.0f);
    CHECK(!yaw.active && yaw.cascade.speed_i == 0.0f && yaw.cascade.position_i == 0.0f);
}
void test_run(void)
{
    uint32_t t, revision;
    float integral, previous, observer;
    union { uint32_t bits; float value; } special;
    test_stage = 1U;
    CHECK(YAW_CONTROLLER_CASCADE_ENABLED == 1U && YAW_CONTROLLER_SPEED_GUARD_ENABLED == 0U);
    ready(100U, 0.0f);
    CHECK(yaw.cascade.position.kp == 8 && yaw.cascade.position.ki == 0);
    CHECK(yaw.cascade.speed.kp == 0.001f && yaw.cascade.speed.ki == 0.004f && yaw.cascade.speed.kd == 0);
    CHECK(YawController_SetTarget(&yaw, 30.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 120U, 1U) == 0.1f);
    CHECK(yaw.cascade.target_velocity_dps == 180 && yaw.cascade.torque_limited && yaw.cascade.velocity_limited);
    for (t = 122U; t < 140U; t += 2U) { (void)YawController_Update(&yaw, 0.0f, t, 1U); }
    CHECK(yaw.cascade.speed_i == 0); /* 无新反馈不积分；饱和亦不蓄积。 */
    CHECK(YawController_SetTarget(&yaw, -30.0f));
    CHECK(YawController_Update(&yaw, 0.0f, 140U, 1U) == -0.1f);
    test_stage = 2U;
    ready(1000U, 0.0f); CHECK(YawController_SetTarget(&yaw, 1.0f));
    (void)YawController_Update(&yaw, 0.0f, 1020U, 1U);
    frame(1030U, 0.0f, -2.8f * DEG_TO_RAD); CHECK(YawController_OnFeedback(&yaw, &feedback));
    (void)YawController_Update(&yaw, 0.0f, 1030U, 1U); integral = yaw.cascade.speed_i;
    CHECK(near(integral, 0.004f * 8.0f * 0.010f, 0.000001f));
    for (t = 1032U; t < 1040U; t += 2U) { (void)YawController_Update(&yaw, 0.0f, t, 1U); }
    CHECK(yaw.cascade.speed_i == integral && yaw.cascade.velocity_dps == 0);
    revision = yaw.cascade.revision;
    CHECK(YawCascade_SetPid(&yaw.cascade, 'V', 0.001f, 0.004f, 0));
    CHECK(yaw.cascade.revision == revision && yaw.cascade.speed_i == integral);
    CHECK(YawCascade_SetPid(&yaw.cascade, 'V', 0.002f, 0.005f, 0));
    CHECK(yaw.cascade.revision == revision + 1 && yaw.cascade.speed_i == 0);
    special.bits = 0x7FC00000UL;
    CHECK(!YawCascade_SetPid(&yaw.cascade, 'P', 8, special.value, 0));
    CHECK(!YawCascade_SetPid(&yaw.cascade, 'V', 0.1f, 0, 0));
    CHECK(!YawCascade_SetPid(&yaw.cascade, 'X', 0, 0, 0));
    CHECK(!YawCascade_SetVelocityLimit(&yaw.cascade, 9));
    CHECK(!YawCascade_SetVelocityLimit(&yaw.cascade, 361));
    CHECK(!YawCascade_SetVelocityLimit(&yaw.cascade, special.value));
    CHECK(YawCascade_SetVelocityLimit(&yaw.cascade, 240));
    test_stage = 3U;
    ready(2000U, 0.0f); frame(2030U, DEG_TO_RAD, 2.8f * DEG_TO_RAD);
    CHECK(YawController_OnFeedback(&yaw, &feedback)); observer = yaw.cascade.velocity_dps;
    CHECK(near(observer, 50, 0.001f)); previous = yaw.cascade.acceleration_dps2;
    CHECK(!YawController_OnFeedback(&yaw, &feedback));
    CHECK(yaw.cascade.velocity_dps == observer && yaw.cascade.acceleration_dps2 == previous);
    YawController_Stop(&yaw); CHECK(yaw.cascade.velocity_dps == observer);
    frame(2130U, 2 * DEG_TO_RAD, 0); CHECK(YawController_OnFeedback(&yaw, &feedback));
    CHECK((yaw.faults & YAW_CONTROLLER_FAULT_FEEDBACK) && yaw.cascade.velocity_dps == 0);
    /* 解环后差分，无360度/10ms假速度。 */
    ready(3000U, PI_RAD - 0.001f); frame(3030U, -PI_RAD + 0.001f, 2.8f * DEG_TO_RAD);
    CHECK(YawController_OnFeedback(&yaw, &feedback)); CHECK(absolute(yaw.cascade.velocity_dps) < 6);
    test_stage = 4U; Plant(30, 4000U);
    test_stage = 5U; Plant(-30, 11000U);
    test_stage = 6U;
    ready(20000U, 0); CHECK(YawCascade_SetPid(&yaw.cascade, 'P', 8, 10, 0));
    CHECK(YawController_SetTarget(&yaw, 30));
    for (t = 20020U; t < 21020U; t += 2U) {
        if (t % 10U == 0U) { frame(t, 0, 0); (void)YawController_OnFeedback(&yaw, &feedback); }
        (void)YawController_Update(&yaw, 0, t, 1U);
    }
    CHECK(yaw.cascade.position_i == 0 && yaw.cascade.speed_i == 0);
    test_stage = 7U;
    /* 两环按实际接收间隔积分一次，重复control tick及tick回卷不多积。 */
    YawCascade_Init(&yaw.cascade);
    CHECK(YawCascade_SetPid(&yaw.cascade, 'P', 0, 1, 0));
    CHECK(YawCascade_SetPid(&yaw.cascade, 'V', 0, 0.004f, 0));
    (void)YawCascade_Update(&yaw.cascade, 1, 0xFFFFFFF0UL, 1U, 0.1f);
    (void)YawCascade_Update(&yaw.cascade, 1, 0x00000004UL, 2U, 0.1f);
    CHECK(near(yaw.cascade.position_i, 0.02f, 0.000001f));
    CHECK(near(yaw.cascade.speed_i, 0.004f * 0.02f * 0.02f, 0.0000001f));
    integral = yaw.cascade.speed_i;
    for (t = 0U; t < 10U; ++t) { (void)YawCascade_Update(&yaw.cascade, 1, 4U, 2U, 0.1f); }
    CHECK(yaw.cascade.speed_i == integral && near(yaw.cascade.position_i, 0.02f, 0.000001f));
    (void)YawCascade_Update(&yaw.cascade, 1, 19U, 3U, 0.1f);
    CHECK(near(yaw.cascade.position_i, 0.035f, 0.000001f));
    /* D只看测量，零运动时30度目标跳变不引入微分力矩。 */
    YawCascade_Init(&yaw.cascade);
    CHECK(YawCascade_SetPid(&yaw.cascade, 'P', 0, 0, 2));
    CHECK(YawCascade_SetPid(&yaw.cascade, 'V', 0.001f, 0, 0.00001f));
    CHECK(YawCascade_Update(&yaw.cascade, 30, 10U, 1U, 0.1f) == 0);
    CHECK(YawCascade_Update(&yaw.cascade, -30, 20U, 2U, 0.1f) == 0);
    YawCascade_Observe(&yaw.cascade, 0.1f, 10U);
    CHECK(YawCascade_Update(&yaw.cascade, 0, 30U, 3U, 0.1f) < 0);
    test_stage = 8U;
    ready(22000U, 0); CHECK(YawController_SetTarget(&yaw, 30));
    CHECK(YawController_Update(&yaw, 0, 22020U, 1U) > 0);
    (void)YawController_Update(&yaw, 0, 22031U, 1U);
    CHECK(!yaw.active && (yaw.faults & YAW_CONTROLLER_FAULT_TIMING));
    CHECK(yaw.cascade.torque_nm == 0 && yaw.cascade.speed_i == 0);
    test_done = 1U;
}
