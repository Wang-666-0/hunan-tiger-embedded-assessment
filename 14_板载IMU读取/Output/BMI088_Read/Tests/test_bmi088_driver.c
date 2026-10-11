/* 主机侧寄存器模拟测试：只编译 bmi088.c，使用本文件模拟端口，不链接 STM32 HAL。 */
#include "bmi088.h"
#include "bmi088_port.h"
#include <math.h>
#include <string.h>

/* 无标准输出和 abort：主机通过 ARM 模拟器读取计数器，适合无操作系统的测试 ELF。 */
volatile uint32_t test_checks;
volatile uint32_t test_failures;
volatile uint32_t test_failed_line;
volatile uint32_t test_done;
#define TEST_CHECK(condition) do { ++test_checks; if (!(condition)) { ++test_failures; \
    if (test_failed_line == 0U) { test_failed_line = __LINE__; } } } while (0)
#define assert(condition) TEST_CHECK(condition) /* 本文件的断言仅记录失败，不调用 libc assert/abort。 */

static uint8_t registers_accel[128];
static uint8_t registers_gyro[128];
static bool accel_spi_enabled;
static bool next_transfer_fails;
static bool corrupt_accel_config;
static bool bad_accel_id;
static bool port_init_fails;
static uint64_t time_us;
static uint64_t gyro_ready_us;
static unsigned int transfers;

static void ResetRegisters(void)
{
    memset(registers_accel, 0, sizeof(registers_accel));
    memset(registers_gyro, 0, sizeof(registers_gyro));
    registers_accel[0x00] = 0x1E;
    registers_accel[0x7C] = 0x03;
    registers_gyro[0x00] = 0x0F;
    registers_gyro[0x10] = 0x80;
    accel_spi_enabled = false;
    gyro_ready_us = 0U;
}

bool BMI088_PortInit(void)
{
    time_us = 0U;
    transfers = 0U;
    ResetRegisters();
    return !port_init_fails;
}

void BMI088_PortDelayUs(uint32_t value) { time_us += value; }
void BMI088_PortDelayMs(uint32_t value) { time_us += (uint64_t)value * 1000U; }

bool BMI088_PortTransfer(bmi088_port_device_t device,
                         const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    uint8_t *registers = device == BMI088_PORT_ACCEL ? registers_accel : registers_gyro;
    uint8_t reg = tx[0] & 0x7FU;
    uint16_t offset = device == BMI088_PORT_ACCEL ? 2U : 1U;
    uint16_t i;

    ++transfers;
    if (next_transfer_fails)
    {
        next_transfer_fails = false;
        return false;
    }
    memset(rx, 0xA5, length); /* 地址响应和 dummy 都填 A5，错位读取会立刻暴露。 */
    if (device == BMI088_PORT_GYRO)
    {
        assert(time_us >= gyro_ready_us); /* 确认复位后没有提前访问陀螺仪。 */
    }
    if ((tx[0] & 0x80U) != 0U)
    {
        assert(length > offset);
        if (device == BMI088_PORT_ACCEL && !accel_spi_enabled)
        {
            accel_spi_enabled = true; /* 第一笔加速度计读无效，但其 CS 上升沿切换到 SPI。 */
            memset(rx, 0xFF, length);
            return true;
        }
        for (i = offset; i < length; ++i)
        {
            uint8_t address = (uint8_t)(reg + i - offset);
            rx[i] = registers[address];
            if (device == BMI088_PORT_ACCEL && address == 0x00U && bad_accel_id)
            {
                rx[i] = 0xFFU;
            }
            if (device == BMI088_PORT_GYRO && address == 0x10U)
            {
                rx[i] |= 0x80U; /* 手册规定该位恒 1，不能使用整个字节比较带宽配置。 */
            }
        }
    }
    else
    {
        assert(length == 2U);
        if (device == BMI088_PORT_ACCEL && reg == 0x7EU && tx[1] == 0xB6U)
        {
            memset(registers_accel, 0, sizeof(registers_accel));
            registers_accel[0x00] = 0x1E;
            registers_accel[0x7C] = 0x03;
            accel_spi_enabled = false;
        }
        else if (device == BMI088_PORT_GYRO && reg == 0x14U && tx[1] == 0xB6U)
        {
            memset(registers_gyro, 0, sizeof(registers_gyro));
            registers_gyro[0x00] = 0x0F;
            registers_gyro[0x10] = 0x80;
            gyro_ready_us = time_us + 30000U;
        }
        else
        {
            registers[reg] = tx[1];
            if (device == BMI088_PORT_ACCEL && reg == 0x40U && corrupt_accel_config)
            {
                registers[reg] ^= 0x01U; /* 模拟写入后读回不一致。 */
            }
            if (device == BMI088_PORT_GYRO && reg == 0x11U)
            {
                gyro_ready_us = time_us + 30000U;
            }
        }
    }
    return true;
}

static void Set16(uint8_t *registers, uint8_t reg, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    registers[reg] = (uint8_t)bits;
    registers[reg + 1U] = (uint8_t)(bits >> 8);
}

static void ExpectNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

void test_run(void)
{
    bmi088_sample_t sample;
    bmi088_sample_t previous;
    unsigned int before;

    test_checks = 0U;
    test_failures = 0U;
    test_failed_line = 0U;
    test_done = 0U;

    memset(&sample, 0x5A, sizeof(sample));
    previous = sample;
    assert(!BMI088_Read(&sample)); /* 初始化前不得读取。 */
    assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
    assert(!BMI088_Read(0));

    port_init_fails = true;
    assert(!BMI088_Init());
    port_init_fails = false;
    bad_accel_id = true;
    assert(!BMI088_Init());
    bad_accel_id = false;
    corrupt_accel_config = true;
    assert(!BMI088_Init());
    corrupt_accel_config = false;

    assert(BMI088_Init());
    assert(registers_accel[0x40] == 0x8A); /* 400 Hz + OSR4。 */
    assert(registers_accel[0x41] == 0x01); /* ±6g。 */
    assert(registers_accel[0x7D] == 0x04 && registers_accel[0x7C] == 0x00);
    assert(registers_gyro[0x0F] == 0x00 && registers_gyro[0x10] == 0x03);
    assert(time_us >= 105000U);

    Set16(registers_accel, 0x12, 16384);
    Set16(registers_accel, 0x14, -16384);
    Set16(registers_accel, 0x16, 0);
    Set16(registers_gyro, 0x02, 16384);
    Set16(registers_gyro, 0x04, -16384);
    Set16(registers_gyro, 0x06, 0);
    registers_accel[0x22] = 0x10;
    registers_accel[0x23] = 0x00;
    assert(BMI088_Read(&sample));
    ExpectNear(sample.accel_m_s2[0], 29.41995f);
    ExpectNear(sample.accel_m_s2[1], -29.41995f);
    ExpectNear(sample.accel_m_s2[2], 0.0f);
    ExpectNear(sample.gyro_rad_s[0], 17.45329252f);
    ExpectNear(sample.gyro_rad_s[1], -17.45329252f);
    ExpectNear(sample.gyro_rad_s[2], 0.0f);
    ExpectNear(sample.temperature_c, 39.0f);

    registers_accel[0x22] = 0xC1;
    assert(BMI088_Read(&sample));
    ExpectNear(sample.temperature_c, -40.0f);
    registers_accel[0x22] = 0x80;
    assert(BMI088_Read(&sample));
    assert(isnan(sample.temperature_c)); /* 初次温度未就绪不得阻塞六轴数据。 */

    previous = sample;
    next_transfer_fails = true;
    before = transfers;
    assert(!BMI088_Read(&sample));
    assert(transfers == before + 1U); /* 一笔失败就终止，不能继续等多个 SPI 超时。 */
    assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
    bad_accel_id = true;
    assert(!BMI088_Read(&sample));
    assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
    bad_accel_id = false;
    registers_accel[0x02] = 0x01U;
    assert(!BMI088_Read(&sample));
    assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
    registers_accel[0x02] = 0x04U;
    assert(!BMI088_Read(&sample));
    assert(memcmp(&sample, &previous, sizeof(sample)) == 0);
    test_done = 1U; /* 测试宿主检查 done 后读取失败计数和首次失败的源码行。 */
}
