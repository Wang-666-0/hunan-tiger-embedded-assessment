/* 编译真实校准和协议模块，以 mock UART 检查结果；完全不连接真实板卡。 */
#include "imu_calibration.h"
#include "vofa_telemetry.h"
#include "usart.h"
#include <math.h>
#include <string.h>

volatile uint32_t test_checks;
volatile uint32_t test_failures;
volatile uint32_t test_failed_line;
volatile uint32_t test_done;
#define CHECK(condition) do { ++test_checks; if (!(condition)) { ++test_failures; \
    if (test_failed_line == 0U) { test_failed_line = __LINE__; } } } while (0)

UART_HandleTypeDef huart1;
static uint8_t sent[VOFA_EULER_FRAME_BYTES];
static uint16_t sent_count;
static uint32_t abort_count;
static HAL_StatusTypeDef uart_result;
static uint8_t received_stream[256]; /* 模拟上位机累积字节，而不仅仅检查 MCU 发送调用。 */
static uint32_t received_count;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, uint8_t *bytes, uint16_t count, uint32_t timeout)
{
    CHECK(handle == &huart1);
    CHECK((count == VOFA_EULER_FRAME_BYTES) || (count == 4U));
    CHECK(timeout == 3U);
    memcpy(sent, bytes, count);
    sent_count = count;
    if (uart_result == HAL_OK)
    {
        memcpy(&received_stream[received_count], bytes, count);
        received_count += count;
    }
    else if (count == VOFA_EULER_FRAME_BYTES)
    {
        memcpy(&received_stream[received_count], bytes, 4U); /* 假设超时前仅传出第一个角度。 */
        received_count += 4U;
    }
    return uart_result;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *handle)
{
    CHECK(handle == &huart1);
    ++abort_count;
    return HAL_OK;
}

static void CheckCalibration(void)
{
    imu_calibration_t state;
    float gyro[3] = {0.01f, -0.02f, 0.03f};
    float accel[3] = {0.0f, 0.0f, 9.80665f};
    float bias[3] = {-999.0f, -999.0f, -999.0f};
    float gravity[3] = {-999.0f, -999.0f, -999.0f};
    uint32_t i;
    ImuCalibration_Reset(&state);
    CHECK(!ImuCalibration_GetResult(&state, bias, gravity));
    CHECK(bias[0] == -999.0f);
    for (i = 0U; i < IMU_CALIBRATION_SAMPLES - 1U; ++i)
    {
        CHECK(!ImuCalibration_Push(&state, gyro, accel));
    }
    CHECK(ImuCalibration_Push(&state, gyro, accel));
    CHECK(ImuCalibration_GetResult(&state, bias, gravity));
    for (i = 0U; i < 3U; ++i)
    {
        CHECK(fabsf(bias[i] - gyro[i]) < 0.00001f);
        CHECK(fabsf(gravity[i] - accel[i]) < 0.001f);
    }
    CHECK(ImuCalibration_Push(&state, gyro, accel));
    CHECK(state.count == IMU_CALIBRATION_SAMPLES);
    gyro[0] = 0.5f;
    CHECK(!ImuCalibration_Push(&state, gyro, accel));
    CHECK(state.count == 0U);
    gyro[0] = 0.0f;
    accel[2] = 2.0f;
    CHECK(!ImuCalibration_Push(&state, gyro, accel));
    CHECK(state.count == 0U);
    accel[2] = 9.80665f;
    for (i = 0U; i < IMU_CALIBRATION_SAMPLES; ++i)
    {
        gyro[0] = (i & 1U) ? 0.03f : -0.03f;
        CHECK(!ImuCalibration_Push(&state, gyro, accel));
    }
    CHECK(state.count == 0U); /* 每帧角速度不大，但窗口方差应拒绝往复转动。 */
    gyro[0] = 0.0f;
    gyro[2] = 0.1f;
    for (i = 0U; i < IMU_CALIBRATION_SAMPLES; ++i)
    {
        CHECK(!ImuCalibration_Push(&state, gyro, accel)); /* 恒定 Z 转速也不能当作大幅零偏。 */
    }
    CHECK(state.count == 0U);
    gyro[2] = 0.03f;
    gyro[0] = 0.0f;
    for (i = 0U; i < IMU_CALIBRATION_SAMPLES; ++i)
    {
        accel[0] = (i & 1U) ? 0.3f : -0.3f;
        CHECK(!ImuCalibration_Push(&state, gyro, accel));
    }
    CHECK(state.count == 0U); /* 重力模长近似正常，但轴向晃动方差应被拒绝。 */
    gyro[0] = NAN;
    CHECK(!ImuCalibration_Push(&state, gyro, accel));
    CHECK(!ImuCalibration_Push(NULL, gyro, accel));
    CHECK(!ImuCalibration_GetResult(NULL, bias, gravity));
}

static void CheckTelemetry(void)
{
    float angles[3] = {1.0f, -2.0f, 90.0f};
    const uint8_t expected[16] = {0x00,0x00,0x80,0x3F, 0x00,0x00,0x00,0xC0,
                                  0x00,0x00,0xB4,0x42, 0x00,0x00,0x80,0x7F};
    uint8_t frame[16];
    uint8_t unchanged[16];
    memset(frame, 0xA5, sizeof(frame));
    CHECK(VOFA_BuildEulerFrame(angles, frame));
    CHECK(memcmp(frame, expected, sizeof(expected)) == 0);
    uart_result = HAL_OK;
    CHECK(VOFA_SendEuler(angles));
    CHECK(sent_count == 16U);
    CHECK(memcmp(sent, expected, sizeof(expected)) == 0);
    CHECK(abort_count == 0U);
    received_count = 0U; /* 模拟器已消费完第一帧，以后检查失败造成的残缺包。 */
    uart_result = HAL_TIMEOUT;
    CHECK(!VOFA_SendEuler(angles));
    CHECK(abort_count == 1U);
    CHECK(received_count == 4U);
    CHECK(!VOFA_SendEuler(angles)); /* 重同步帧尾也发送失败，不能继续发送有效负载。 */
    CHECK(sent_count == 4U);
    CHECK(abort_count == 2U);
    CHECK(received_count == 4U);
    uart_result = HAL_OK;
    CHECK(VOFA_SendEuler(angles));
    CHECK(memcmp(sent, expected, sizeof(expected)) == 0);
    CHECK(received_count == 24U);
    CHECK(memcmp(&received_stream[4], &expected[12], 4U) == 0); /* 先结束之前的半帧。 */
    CHECK(memcmp(&received_stream[8], expected, sizeof(expected)) == 0); /* 后一帧重新正确从 Roll 开始。 */
    memcpy(unchanged, frame, sizeof(frame));
    angles[1] = NAN;
    CHECK(!VOFA_BuildEulerFrame(angles, frame));
    CHECK(memcmp(frame, unchanged, sizeof(frame)) == 0);
    CHECK(!VOFA_SendEuler(angles));
    angles[1] = INFINITY;
    CHECK(!VOFA_BuildEulerFrame(angles, frame));
    CHECK(!VOFA_BuildEulerFrame(NULL, frame));
    CHECK(!VOFA_BuildEulerFrame(angles, NULL));
}

void test_run(void)
{
    CheckCalibration();
    CheckTelemetry();
    test_done = 1U;
}
