#include "mahony.h" /* 软件测试只进入模拟器，不进入正式固件。 */

#include <stdint.h> // 测试计数通过全局变量供 ARM 模拟器读取，不连接板子。
#include <float.h>
#include <math.h>
#include <string.h>

volatile uint32_t test_checks;       // 已执行的检查数。
volatile uint32_t test_failures;     // 不通过的检查数。
volatile uint32_t test_failed_line;  // 第一处失败的位置。
volatile uint32_t test_done;         // 模拟器通过此标志判断测试是否真正执行结束。

#define CHECK(expression) do { ++test_checks; if (!(expression)) { ++test_failures; if (test_failed_line == 0U) { test_failed_line = __LINE__; } } } while (0)

#define TEST_PI 3.14159265358979323846f

static void AssertNear(float actual, float expected, float tolerance)
{
    CHECK(fabsf(actual - expected) <= tolerance); // 数值必须在指定误差内。
}

static void AssertUnitQuaternion(const mahony_t *state)
{
    float norm_sq = state->q[0] * state->q[0] + state->q[1] * state->q[1] + state->q[2] * state->q[2] + state->q[3] * state->q[3];
    AssertNear(norm_sq, 1.0f, 0.00001f); // 更新后始终保持单位四元数。
}

static void TestInitialAttitudes(void)
{
    const float level[3] = {0.0f, 0.0f, 9.80665f};
    const float roll_90[3] = {0.0f, 9.80665f, 0.0f};
    const float pitch_90[3] = {-9.80665f, 0.0f, 0.0f};
    const float pitch_minus_90[3] = {9.80665f, 0.0f, 0.0f};
    const float inverted[3] = {0.0f, 0.0f, -9.80665f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    mahony_t state, backup;
    float angles[3];

    CHECK(Mahony_Init(&state, level));
    CHECK(Mahony_GetEulerDegrees(&state, angles));
    AssertNear(angles[0], 0.0f, 0.001f);
    AssertNear(angles[1], 0.0f, 0.001f);
    AssertNear(angles[2], 0.0f, 0.001f);
    CHECK(Mahony_Init(&state, roll_90));
    CHECK(Mahony_GetEulerDegrees(&state, angles));
    AssertNear(angles[0], 90.0f, 0.05f); // 校验正 90 度横滚，防止符号写反。
    CHECK(Mahony_Init(&state, pitch_90));
    CHECK(Mahony_GetEulerDegrees(&state, angles));
    AssertNear(angles[1], 90.0f, 0.05f); // 俯仰 ±90 度处 Roll / Yaw 不唯一，只检查 Pitch。
    CHECK(Mahony_Init(&state, pitch_minus_90));
    CHECK(Mahony_GetEulerDegrees(&state, angles));
    AssertNear(angles[1], -90.0f, 0.05f);
    CHECK(Mahony_Init(&state, inverted));
    CHECK(Mahony_GetEulerDegrees(&state, angles));
    AssertNear(fabsf(angles[0]), 180.0f, 0.05f); // 倒放时初始化不应产生 NaN。
    AssertUnitQuaternion(&state);
    backup = state;
    CHECK(!Mahony_Init(&state, zero));
    CHECK(memcmp(&state, &backup, sizeof(state)) == 0); // 无效初始化不应清掉旧姿态。
    CHECK(!Mahony_Init(NULL, level));
    CHECK(!Mahony_Init(&state, NULL));
}

static void TestYawAndTimeStep(void)
{
    const float accel[3] = {0.0f, 0.0f, 9.80665f};
    const float gyro[3] = {0.0f, 0.0f, TEST_PI / 2.0f}; // 绕 Z 轴以 90 度/秒转动。
    mahony_t state_5ms, state_10ms;
    float angles_5ms[3], angles_10ms[3];
    unsigned int i;

    CHECK(Mahony_Init(&state_5ms, accel));
    CHECK(Mahony_Init(&state_10ms, accel));
    state_5ms.kp = 0.0f; // 单独验证陀螺仪积分的单位与实际 dt。
    state_10ms.kp = 0.0f;
    for (i = 0U; i < 200U; ++i)
    {
        CHECK(Mahony_UpdateIMU(&state_5ms, gyro, accel, 0.005f)); // 200 次 × 5 ms = 1 秒。
        AssertUnitQuaternion(&state_5ms);
    }
    for (i = 0U; i < 100U; ++i)
    {
        CHECK(Mahony_UpdateIMU(&state_10ms, gyro, accel, 0.010f)); // 100 次 × 10 ms 也应积分 1 秒。
    }
    CHECK(Mahony_GetEulerDegrees(&state_5ms, angles_5ms));
    CHECK(Mahony_GetEulerDegrees(&state_10ms, angles_10ms));
    AssertNear(angles_5ms[2], 90.0f, 0.01f);
    AssertNear(angles_10ms[2], 90.0f, 0.01f);
    AssertNear(angles_5ms[2], angles_10ms[2], 0.01f); // 验证未把更新间隔写死为 5 ms。
}

static void TestTiltConvergence(void)
{
    const float level[3] = {0.0f, 0.0f, 9.80665f};
    const float tilted[3] = {-4.903325f, 4.246404f, 7.3550f}; // 对应 Roll≈30 度、Pitch≈30 度。
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    mahony_t state;
    float angles[3];
    unsigned int i;

    CHECK(Mahony_Init(&state, level));
    for (i = 0U; i < 1600U; ++i)
    {
        CHECK(Mahony_UpdateIMU(&state, gyro, tilted, 0.005f)); // 静止重力持续校正一个初始错误姿态。
    }
    CHECK(Mahony_GetEulerDegrees(&state, angles));
    AssertNear(angles[0], 30.0f, 0.1f);
    AssertNear(angles[1], 30.0f, 0.1f);
    AssertUnitQuaternion(&state);
}

static void TestInvalidInputsAreTransactional(void)
{
    const float accel[3] = {0.0f, 0.0f, 9.80665f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    const float zero_accel[3] = {0.0f, 0.0f, 0.0f};
    const float nan_accel[3] = {NAN, 0.0f, 9.80665f};
    const float inf_gyro[3] = {INFINITY, 0.0f, 0.0f};
    mahony_t state, backup;
    float angles[3] = {10.0f, 20.0f, 30.0f};

    CHECK(Mahony_Init(&state, accel));
    backup = state;
    CHECK(!Mahony_UpdateIMU(&state, gyro, accel, 0.0f));
    CHECK(!Mahony_UpdateIMU(&state, gyro, accel, -0.005f));
    CHECK(!Mahony_UpdateIMU(&state, gyro, accel, 0.2f));
    CHECK(!Mahony_UpdateIMU(&state, gyro, accel, NAN));
    CHECK(!Mahony_UpdateIMU(&state, gyro, zero_accel, 0.005f));
    CHECK(!Mahony_UpdateIMU(&state, gyro, nan_accel, 0.005f));
    CHECK(!Mahony_UpdateIMU(&state, inf_gyro, accel, 0.005f));
    CHECK(!Mahony_UpdateIMU(NULL, gyro, accel, 0.005f));
    CHECK(memcmp(&state, &backup, sizeof(state)) == 0); // 所有非法输入均不改变状态。
    state.kp = FLT_MAX;
    CHECK(!Mahony_UpdateIMU(&state, gyro, nan_accel, 0.005f));
    state = backup;
    state.q[0] = NAN;
    CHECK(!Mahony_GetEulerDegrees(&state, angles));
    AssertNear(angles[0], 10.0f, 0.0f); // 无效姿态也不能改写输出。
    state = backup;
    state.q[0] = 0.0f;
    CHECK(!Mahony_UpdateIMU(&state, gyro, accel, 0.005f)); // 拒绝零四元数。
    state = backup;
    state.initialized = false;
    CHECK(!Mahony_GetEulerDegrees(&state, angles));
    CHECK(!Mahony_UpdateIMU(&state, gyro, accel, 0.005f));
}

void test_run(void)
{
    test_checks = 0U;    // 允许同一个模拟器显式重新运行这组测试。
    test_failures = 0U;
    test_failed_line = 0U;
    test_done = 0U;
    TestInitialAttitudes();
    TestYawAndTimeStep();
    TestTiltConvergence();
    TestInvalidInputsAreTransactional();
    test_done = 1U;      // 只有全部用例已执行，才能标记完成。
}

int main(void)
{
    test_run();
    return (test_failures == 0U) ? 0 : 1; // 也兼容安装 native C 编译器后的普通主机执行。
}
