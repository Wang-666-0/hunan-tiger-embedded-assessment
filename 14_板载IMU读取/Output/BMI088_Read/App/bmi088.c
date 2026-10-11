/*
 * 本板专用的小型 BMI088 SPI 驱动，独立实现，寄存器依据 Bosch BMI088 数据手册。
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 * 驱动只负责读出物理量；陀螺仪零偏、坐标变换、Mahony 和串口发送由上层负责。
 * SPI1 只由 imuTask 访问；若以后让其他任务共享 SPI，须在 port 外加入互斥保护。
 */
#include "bmi088.h"
#include "bmi088_port.h"
#include <math.h>
#include <stdint.h>

#define ACC_CHIP_ID_REG        0x00U
#define ACC_DATA_REG           0x12U
#define ACC_TEMP_REG           0x22U
#define ACC_CONF_REG           0x40U
#define ACC_RANGE_REG          0x41U
#define ACC_PWR_CONF_REG       0x7CU
#define ACC_PWR_CTRL_REG       0x7DU
#define ACC_SOFTRESET_REG      0x7EU
#define ACC_CHIP_ID_VALUE      0x1EU
#define ACC_CONF_400HZ_OSR4    0x8AU /* 400 Hz 输出、40 Hz 低通，任务仍然是 200 Hz。 */
#define ACC_RANGE_6G           0x01U /* 配置 ±6g 后，物理量换算必须使用相同量程。 */

#define GYRO_CHIP_ID_REG       0x00U
#define GYRO_RANGE_REG         0x0FU
#define GYRO_BANDWIDTH_REG     0x10U
#define GYRO_LPM1_REG          0x11U
#define GYRO_SOFTRESET_REG     0x14U
#define GYRO_CHIP_ID_VALUE     0x0FU
#define GYRO_RANGE_2000DPS     0x00U /* ±2000 °/s，避免快速手动旋转时超出量程。 */
#define GYRO_400HZ_47HZ        0x03U /* 400 Hz 输出、47 Hz 低通；bit7 只读，验证时忽略。 */

#define SOFTRESET_COMMAND     0xB6U
#define SPI_READ_FLAG          0x80U
#define MAX_REGISTER_BYTES     8U
#define ACC_SCALE_M_S2         (6.0f * 9.80665f / 32768.0f) /* 标准重力加速度乘以 ±6g 量程。 */
#define GYRO_SCALE_RAD_S       (2000.0f * 0.017453292519943295f / 32768.0f) /* 先换 °/s，再换 rad/s。 */

static bool initialized; /* 仅维护驱动可用状态，不增加调试专用变量。 */

static bool ReadRegisters(bmi088_port_device_t device, uint8_t reg,
                          uint8_t *values, uint16_t count)
{
    uint8_t tx[MAX_REGISTER_BYTES + 2U] = {0U};
    uint8_t rx[MAX_REGISTER_BYTES + 2U] = {0U};
    uint16_t offset;
    uint16_t i;

    if (values == 0 || count == 0U || count > MAX_REGISTER_BYTES)
    {
        return false;
    }
    offset = device == BMI088_PORT_ACCEL ? 2U : 1U; /* 加速度计多一个 dummy byte，陀螺仪没有。 */
    tx[0] = reg | SPI_READ_FLAG; /* 地址最高位 1 表示读；其余发送字节只用来产生 SPI 时钟。 */
    if (!BMI088_PortTransfer(device, tx, rx, (uint16_t)(count + offset)))
    {
        return false; /* SPI 失败时不把半帧或全零缓冲区返回给上层。 */
    }
    for (i = 0U; i < count; ++i)
    {
        values[i] = rx[i + offset]; /* 同一次 CS 低电平内连续读，保证一组三轴属于同一帧。 */
    }
    return true;
}

static bool WriteRegister(bmi088_port_device_t device, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];
    bool result;

    tx[0] = reg & 0x7FU; /* 地址最高位 0 表示写；写操作两个器件都不需要 dummy byte。 */
    tx[1] = value;
    result = BMI088_PortTransfer(device, tx, rx, 2U);
    BMI088_PortDelayUs(1000U); /* 同时满足活动/休眠状态写寄存器后的同步等待要求。 */
    return result;
}

static bool WriteAndVerify(bmi088_port_device_t device, uint8_t reg,
                           uint8_t value, uint8_t mask)
{
    uint8_t actual;

    if (!WriteRegister(device, reg, value) || !ReadRegisters(device, reg, &actual, 1U))
    {
        return false; /* 写失败或读失败都不能当作配置成功。 */
    }
    return (actual & mask) == (value & mask); /* 忽略保留位和只读位，验证实际可配置的字段。 */
}

static bool CheckChipId(bmi088_port_device_t device, uint8_t expected)
{
    uint8_t value;
    return ReadRegisters(device, 0x00U, &value, 1U) && value == expected; /* HAL 成功不等于传感器存在。 */
}

static int16_t DecodeSigned16(const uint8_t *bytes)
{
    int32_t value = (int32_t)bytes[0] | ((int32_t)bytes[1] << 8); /* BMI088 数据低字节在前。 */
    if (value >= 32768)
    {
        value -= 65536; /* 显式处理二补码，避免依赖无符号转有符号的实现细节。 */
    }
    return (int16_t)value;
}

bool BMI088_Init(void)
{
    uint8_t ignored;
    uint8_t accel_status[3];

    initialized = false; /* 重新初始化前禁止读取，任何中途错误都保持不可用。 */
    if (!BMI088_PortInit())
    {
        return false;
    }
    BMI088_PortDelayMs(35U); /* 也覆盖上电后陀螺仪最大 30 ms 启动时间。 */

    if (!ReadRegisters(BMI088_PORT_ACCEL, ACC_CHIP_ID_REG, &ignored, 1U))
    {
        return false;
    }
    BMI088_PortDelayUs(1000U); /* 第一次虚读只产生 CS 上升沿，将加速度计从 I2C 切至 SPI。 */
    if (!CheckChipId(BMI088_PORT_ACCEL, ACC_CHIP_ID_VALUE) ||
        !CheckChipId(BMI088_PORT_GYRO, GYRO_CHIP_ID_VALUE))
    {
        return false; /* 先检查两部分 ID，防止接错 SPI/片选时继续盲写配置。 */
    }

    if (!WriteRegister(BMI088_PORT_ACCEL, ACC_SOFTRESET_REG, SOFTRESET_COMMAND))
    {
        return false;
    }
    BMI088_PortDelayMs(5U); /* 加速度计复位至少等待 1 ms，给任务调度留出余量。 */
    if (!ReadRegisters(BMI088_PORT_ACCEL, ACC_CHIP_ID_REG, &ignored, 1U))
    {
        return false;
    }
    BMI088_PortDelayUs(1000U); /* 复位后再做一次虚读，兼容仍需 SPI 切换的启动状态。 */
    if (!CheckChipId(BMI088_PORT_ACCEL, ACC_CHIP_ID_VALUE))
    {
        return false;
    }

    if (!WriteRegister(BMI088_PORT_GYRO, GYRO_SOFTRESET_REG, SOFTRESET_COMMAND))
    {
        return false;
    }
    BMI088_PortDelayMs(35U); /* 陀螺仪复位期间不能通信，等待至少 30 ms。 */
    if (!CheckChipId(BMI088_PORT_GYRO, GYRO_CHIP_ID_VALUE))
    {
        return false;
    }

    if (!WriteAndVerify(BMI088_PORT_ACCEL, ACC_PWR_CTRL_REG, 0x04U, 0xFFU) || /* 复位后必须开启加速度计。 */
        !WriteAndVerify(BMI088_PORT_ACCEL, ACC_PWR_CONF_REG, 0x00U, 0xFFU) || /* 离开省电休眠模式。 */
        !WriteAndVerify(BMI088_PORT_ACCEL, ACC_CONF_REG, ACC_CONF_400HZ_OSR4, 0xFFU) ||
        !WriteAndVerify(BMI088_PORT_ACCEL, ACC_RANGE_REG, ACC_RANGE_6G, 0x03U) ||
        !WriteAndVerify(BMI088_PORT_GYRO, GYRO_RANGE_REG, GYRO_RANGE_2000DPS, 0x07U) ||
        !WriteAndVerify(BMI088_PORT_GYRO, GYRO_BANDWIDTH_REG, GYRO_400HZ_47HZ, 0x07U))
    {
        return false;
    }
    if (!WriteRegister(BMI088_PORT_GYRO, GYRO_LPM1_REG, 0x00U))
    {
        return false;
    }
    BMI088_PortDelayMs(35U); /* 即使目标是正常模式，写电源状态后仍按手册等待稳定。 */
    if (!CheckChipId(BMI088_PORT_GYRO, GYRO_CHIP_ID_VALUE) ||
        !ReadRegisters(BMI088_PORT_GYRO, GYRO_LPM1_REG, &ignored, 1U) || ignored != 0x00U ||
        !ReadRegisters(BMI088_PORT_ACCEL, ACC_CHIP_ID_REG, accel_status, 3U) ||
        accel_status[0] != ACC_CHIP_ID_VALUE || (accel_status[2] & 0x1DU) != 0U)
    {
        return false; /* 同时排除加速度计 fatal_err 和 ACC_CONF 配置错误。 */
    }
    initialized = true; /* 所有 ID、配置读回和电源状态检查通过后，才允许输出数据。 */
    return true;
}

bool BMI088_Read(bmi088_sample_t *sample)
{
    uint8_t accel_status[3];
    uint8_t accel_bytes[6];
    uint8_t gyro_bytes[8];
    uint8_t temp_bytes[2];
    int32_t temperature_raw;
    uint32_t axis;
    bmi088_sample_t next;

    if (!initialized || sample == 0)
    {
        return false;
    }
    if (!ReadRegisters(BMI088_PORT_ACCEL, ACC_CHIP_ID_REG, accel_status, 3U) ||
        accel_status[0] != ACC_CHIP_ID_VALUE || (accel_status[2] & 0x1DU) != 0U ||
        !ReadRegisters(BMI088_PORT_GYRO, GYRO_CHIP_ID_REG, gyro_bytes, 8U) ||
        gyro_bytes[0] != GYRO_CHIP_ID_VALUE ||
        !ReadRegisters(BMI088_PORT_ACCEL, ACC_DATA_REG, accel_bytes, 6U) ||
        !ReadRegisters(BMI088_PORT_ACCEL, ACC_TEMP_REG, temp_bytes, 2U))
    {
        return false; /* 缺失器件、HAL 超时或半帧都不覆盖调用方保存的上一帧。 */
    }
    for (axis = 0U; axis < 3U; ++axis)
    {
        next.accel_m_s2[axis] = (float)DecodeSigned16(&accel_bytes[axis * 2U]) * ACC_SCALE_M_S2;
        next.gyro_rad_s[axis] = (float)DecodeSigned16(&gyro_bytes[2U + axis * 2U]) * GYRO_SCALE_RAD_S;
        /* 陀螺仪从 0x00 连读，前两字节是 CHIP_ID 和保留寄存器，三轴从偏移 2 开始。 */
    }
    temperature_raw = ((int32_t)temp_bytes[0] << 3) | (temp_bytes[1] >> 5); /* 温度是 11 位二补码。 */
    if (temperature_raw == 1024)
    {
        next.temperature_c = NAN; /* 0x80/0x00 是尚无有效温度的标志，不阻塞六轴读取。 */
    }
    else
    {
        if (temperature_raw > 1023)
        {
            temperature_raw -= 2048; /* 对负温度做 11 位符号扩展。 */
        }
        next.temperature_c = (float)temperature_raw * 0.125f + 23.0f;
    }
    *sample = next; /* 所有 SPI 事务通过后，一次性提交完整结果。 */
    return true;
}
