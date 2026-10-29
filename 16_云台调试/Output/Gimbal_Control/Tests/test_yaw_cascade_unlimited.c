#define test_run limited_controller_fixture_not_run
#include "test_yaw_controller.c"
#undef test_run
#include <math.h>

static void Plant(float target, uint32_t base)
{
    uint32_t t;
    float p = 0, v = 0, torque, peak = 0;
    ready(base, 0);
    /* 原虚拟惯量仅作为V2B方向/收敛回归，固定该模型的旧增益。
     * 本轮实物固化值在stage1及正式console组独立验证。 */
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'P', 8, 0, 0));
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'V', 0.001f, 0.004f, 0));
    CHECK(YawController_SetTarget(&yaw, target));
    for (t = 0; t <= 5000; t += 2) {
        if (t && t % 10 == 0) {
            frame(base + 20 + t, floorf(p / 0.005493f + 0.5f) * 0.005493f * DEG_TO_RAD,
                (v >= 0 ? 2.8f : -2.8f) * DEG_TO_RAD);
            CHECK(YawController_OnFeedback(&yaw, &feedback));
        }
        torque = YawController_Update(&yaw, 0, base + 20 + t, 1);
        if (absolute(torque) > peak) { peak = absolute(torque); }
        CHECK(yaw.active && !yaw.faults && absolute(torque) <= 10);
        v += (torque - 0.00008f * v - 0.004f) * 28647.89f * 0.002f;
        p += v * 0.002f;
    }
    CHECK(peak > 0.2f && absolute(p - target) < 0.2f && absolute(v) < 1);
    YawController_Stop(&yaw); CHECK(!yaw.active && !yaw.cascade.speed_i && !yaw.cascade.position_i);
}
void test_run(void)
{
    uint32_t t;
    float before;
    test_stage = 1;
    CHECK(!0U && YAW_CONTROLLER_TORQUE_LIMIT_NM == 10);
    ready(100, 0); CHECK(yaw.cascade.velocity_limit_dps == 0);
    CHECK(yaw.cascade.position.kp == APP_YAW_POSITION_KP && yaw.cascade.position.ki == APP_YAW_POSITION_KI && yaw.cascade.position.kd == APP_YAW_POSITION_KD);
    CHECK(yaw.cascade.speed.kp == APP_YAW_SPEED_KP && yaw.cascade.speed.ki == APP_YAW_SPEED_KI && yaw.cascade.speed.kd == APP_YAW_SPEED_KD);
    CHECK(YawController_SetTarget(&yaw, 30));
    CHECK(near(YawController_Update(&yaw, 0, 120, 1), APP_YAW_POSITION_KP * 30.0f * APP_YAW_SPEED_KP, 0.000001f));
    CHECK(near(yaw.cascade.target_velocity_dps, APP_YAW_POSITION_KP * 30.0f, 0.0001f) && !yaw.cascade.velocity_limited);
    CHECK(yaw.cascade.velocity_limit_dps == 0.0f);
    /* 截图增益：首次510deg/s、2.448协议Nm，不再被180/.10截断。 */
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'P', 17, 1.25f, 0));
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'V', 0.0048f, 0.013f, 0));
    CHECK(near(YawController_Update(&yaw, 0, 122, 1), 2.448f, 0.00001f));
    CHECK(yaw.cascade.target_velocity_dps == 510 && !yaw.cascade.torque_limited);
    CHECK(YawController_SetTarget(&yaw, -30));
    CHECK(near(YawController_Update(&yaw, 0, 124, 1), -2.448f, 0.00001f));
    test_stage = 2;
    /* 外环积分可超过旧180输出，内环积分可超过旧.10，重复tick仍不积分。 */
    GimbalPid_Init(&yaw.cascade);
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'P', 0, 10, 0));
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'V', 0.001f, 0, 0));
    for (t = 0; t <= 1000; t += 10) { (void)GimbalPid_Update(&yaw.cascade, 30, t, t + 1, 10); }
    CHECK(yaw.cascade.position_i > 299 && yaw.cascade.target_velocity_dps > 299);
    CHECK(!yaw.cascade.velocity_limited && yaw.cascade.torque_nm > 0.299f);
    GimbalPid_Init(&yaw.cascade); CHECK(GimbalPid_SetPid(&yaw.cascade, 'V', 0, 0.2f, 0));
    for (t = 0; t <= 200; t += 10) { (void)GimbalPid_Update(&yaw.cascade, 1, t, t + 1, 10); }
    CHECK(yaw.cascade.speed_i > 0.29f && !yaw.cascade.torque_limited);
    before = yaw.cascade.speed_i;
    for (t = 0; t < 10; ++t) { (void)GimbalPid_Update(&yaw.cascade, 1, 200, 201, 10); }
    CHECK(yaw.cascade.speed_i == before);
    /* 唯一输出端点是MIT协议±10，其反向积分仍可解除协议饱和。 */
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'P', 30, 10, 0));
    CHECK(GimbalPid_SetPid(&yaw.cascade, 'V', 0.02f, 0.2f, 0));
    CHECK(GimbalPid_Update(&yaw.cascade, 30, 210, 210, 10) == 10);
    before = yaw.cascade.position_i;
    for (t = 220; t < 1220; t += 10) { CHECK(GimbalPid_Update(&yaw.cascade, 30, t, t, 10) == 10); }
    CHECK(yaw.cascade.position_i == before && yaw.cascade.speed_i == 0);
    CHECK(GimbalPid_Update(&yaw.cascade, -30, 1220, 1220, 10) == -10);
    test_stage = 3; Plant(30, 2000);
    test_stage = 4; Plant(-30, 8000);
    test_stage = 5;
    ready(15000, 0); CHECK(YawController_SetTarget(&yaw, 30));
    CHECK(YawController_Update(&yaw, 0, 15020, 1) > 0.1f);
    CHECK(YawController_Update(&yaw, 0, 15022, 0) == 0 && !yaw.active);
    ready(16000, 0); CHECK(YawController_SetTarget(&yaw, 30));
    (void)YawController_Update(&yaw, 0, 16020, 1);
    (void)YawController_Update(&yaw, 0, 16031, 1);
    CHECK((yaw.faults & YAW_CONTROLLER_FAULT_TIMING) && !yaw.active);
    test_done = 1;
}
