/*
 * 固定 64 字节的双扇区 PID 日志。word 布局：
 * 0 magic，1 version，2 sequence，3..8 六个 IEEE754 float，
 * 9..13 reserved=0xFFFFFFFF，14 CRC32，15 commit（最后编程）。
 * CRC32/ISO-HDLC: reflected 0xEDB88320, init/xorout 0xFFFFFFFF；
 * 覆盖前 14 words 的低字节到高字节，共 56 字节。
 *
 * 先在最新记录同一扇区找全擦除槽。满后使用另一扇区；若另一扇区也满，
 * 仅擦另一扇区，再写新记录。旧最新记录的扇区不立即擦除，保留回退。
 * 掉电的半条记录与 CRC 损坏记录跳过；扫描其后槽，不能遇坏记录就停止。
 * 本模块不访问 CAN、UART 或控制器，只存调用者提供的增益快照。
 */
#include "pid_store.h"
#include <stddef.h>
#include <string.h>

#ifndef PID_STORE_TEST_BACKEND
#include "stm32f4xx_hal.h"
#if !defined(STM32F405xx)
#error PID_store_hardware_backend_requires_STM32F405xx
#endif
#endif

typedef char PidStore_FloatMustBe32Bits[(sizeof(float) == sizeof(uint32_t)) ? 1 : -1];
typedef char PidStore_RecordMustBe64Bytes[(PID_STORE_RECORD_WORDS * sizeof(uint32_t) ==
    PID_STORE_RECORD_SIZE_BYTES) ? 1 : -1];

typedef struct
{
    PidStore_Params latest_params;
    uint32_t latest_sequence;
    uint32_t latest_sector;
    uint32_t free_address[2];
    uint8_t have_latest;
} PidStore_Scan;

static const uint32_t sector_addresses[2] = {
    PID_STORE_SECTOR_A_ADDRESS, PID_STORE_SECTOR_B_ADDRESS
};
static const uint32_t sector_numbers[2] = {
    PID_STORE_SECTOR_A_NUMBER, PID_STORE_SECTOR_B_NUMBER
};

#ifndef PID_STORE_TEST_BACKEND
static uint8_t PidStore_AddressIsReserved(uint32_t address)
{
    return (uint8_t)(address >= PID_STORE_SECTOR_A_ADDRESS &&
        address <= PID_STORE_SECTOR_B_ADDRESS + PID_STORE_SECTOR_SIZE_BYTES - 4U &&
        (address & 3U) == 0U);
}

static uint8_t PidStore_BackendAvailable(void)
{
    /* F405 也有容量不同的型号，不能只依据设备系列就读取末端参数区。 */
    return (uint8_t)(*(volatile const uint16_t *)(uintptr_t)FLASHSIZE_BASE >= 1024U);
}

static uint32_t PidStore_Backend_ReadWord(uint32_t address)
{
    if (PidStore_AddressIsReserved(address) == 0U) { return 0xFFFFFFFFU; }
    return *(volatile const uint32_t *)(uintptr_t)address;
}

static uint8_t PidStore_Backend_Unlock(void)
{
    if (HAL_FLASH_Unlock() != HAL_OK) { return 0U; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    return 1U;
}

static uint8_t PidStore_Backend_Lock(void)
{
    return (uint8_t)(HAL_FLASH_Lock() == HAL_OK);
}

static uint8_t PidStore_Backend_ProgramWord(uint32_t address, uint32_t value)
{
    HAL_StatusTypeDef status;
    if (PidStore_AddressIsReserved(address) == 0U) { return 0U; }
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint64_t)value);
    /* HAL Program 不刷新 cache；读回必须看到真实的新字/部分失败字。 */
    FLASH_FlushCaches();
    return (uint8_t)(status == HAL_OK);
}

static uint8_t PidStore_Backend_EraseSector(uint32_t sector)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t failed_sector = 0xFFFFFFFFU;
    if (sector != PID_STORE_SECTOR_A_NUMBER && sector != PID_STORE_SECTOR_B_NUMBER)
    {
        return 0U;
    }
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = sector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 本工程 MCU 3.3 V，允许 32 bit 编程。
    return (uint8_t)(HAL_FLASHEx_Erase(&erase, &failed_sector) == HAL_OK &&
        failed_sector == 0xFFFFFFFFU);
}
#else
static uint8_t PidStore_BackendAvailable(void)
{
    return 1U; // 软件测试没有芯片容量寄存器，五个后端钩子只操作模拟数组。
}
#endif

uint8_t PidStore_Validate(const PidStore_Params *params)
{
    if (params == NULL) { return 0U; }
    /* 有序上下界比较同时拒绝 NaN 与 ±Inf，不依赖编译器的 isfinite。 */
    if (!(params->speed_kp >= 0.0f && params->speed_kp <= SPEED_PID_KP_MAX &&
          params->speed_ki >= 0.0f && params->speed_ki <= SPEED_PID_KI_MAX &&
          params->speed_kd >= 0.0f && params->speed_kd <= SPEED_PID_KD_MAX))
    {
        return 0U;
    }
#if PID_STORE_APP_KIND == 2U
    return (uint8_t)(params->angle_kp >= 0.0f && params->angle_kp <= ANGLE_PID_KP_MAX &&
        params->angle_ki >= 0.0f && params->angle_ki <= ANGLE_PID_KI_MAX &&
        params->angle_kd >= 0.0f && params->angle_kd <= ANGLE_PID_KD_MAX);
#else
    return (uint8_t)(params->angle_kp == 0.0f && params->angle_ki == 0.0f &&
        params->angle_kd == 0.0f);
#endif
}

static void PidStore_ParamsToWords(const PidStore_Params *params, uint32_t words[6])
{
    memcpy(&words[0], &params->speed_kp, sizeof(uint32_t));
    memcpy(&words[1], &params->speed_ki, sizeof(uint32_t));
    memcpy(&words[2], &params->speed_kd, sizeof(uint32_t));
    memcpy(&words[3], &params->angle_kp, sizeof(uint32_t));
    memcpy(&words[4], &params->angle_ki, sizeof(uint32_t));
    memcpy(&words[5], &params->angle_kd, sizeof(uint32_t));
}

static void PidStore_WordsToParams(const uint32_t words[6], PidStore_Params *params)
{
    memcpy(&params->speed_kp, &words[0], sizeof(uint32_t));
    memcpy(&params->speed_ki, &words[1], sizeof(uint32_t));
    memcpy(&params->speed_kd, &words[2], sizeof(uint32_t));
    memcpy(&params->angle_kp, &words[3], sizeof(uint32_t));
    memcpy(&params->angle_ki, &words[4], sizeof(uint32_t));
    memcpy(&params->angle_kd, &words[5], sizeof(uint32_t));
}

static uint8_t PidStore_ParamsEqual(const PidStore_Params *a, const PidStore_Params *b)
{
    /* 比较数值而非结构体 padding；+0 与 -0 不造成无意义的重复擦写。 */
    return (uint8_t)(a->speed_kp == b->speed_kp && a->speed_ki == b->speed_ki &&
        a->speed_kd == b->speed_kd && a->angle_kp == b->angle_kp &&
        a->angle_ki == b->angle_ki && a->angle_kd == b->angle_kd);
}

static uint32_t PidStore_Crc32(const uint32_t *words)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t word_index = 0U; word_index < 14U; word_index++)
    {
        for (uint32_t byte_index = 0U; byte_index < 4U; byte_index++)
        {
            crc ^= (words[word_index] >> (byte_index * 8U)) & 0xFFU;
            for (uint32_t bit = 0U; bit < 8U; bit++)
            {
                crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320U : 0U);
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

static uint8_t PidStore_ReadRecord(uint32_t address, uint32_t words[PID_STORE_RECORD_WORDS])
{
    uint8_t empty = 1U;
    for (uint32_t i = 0U; i < PID_STORE_RECORD_WORDS; i++)
    {
        words[i] = PidStore_Backend_ReadWord(address + i * 4U);
        if (words[i] != 0xFFFFFFFFU) { empty = 0U; }
    }
    return empty;
}

static uint8_t PidStore_RecordIsValid(const uint32_t words[PID_STORE_RECORD_WORDS],
    PidStore_Params *params)
{
    if (words[0] != PID_STORE_RECORD_MAGIC || words[1] != PID_STORE_FORMAT_VERSION ||
        words[15] != PID_STORE_COMMIT_WORD || words[14] != PidStore_Crc32(words))
    {
        return 0U;
    }
    PidStore_WordsToParams(&words[3], params);
    return PidStore_Validate(params);
}

static uint8_t PidStore_SequenceIsNewer(uint32_t candidate, uint32_t current)
{
    /* 保留日志最多 4096 条，正常序号跨度远小于半个 uint32 周期。 */
    return (uint8_t)(candidate != current && (uint32_t)(candidate - current) < 0x80000000U);
}

static void PidStore_ScanRecords(PidStore_Scan *scan)
{
    uint32_t words[PID_STORE_RECORD_WORDS];
    PidStore_Params candidate;
    memset(scan, 0, sizeof(*scan));
    for (uint32_t sector = 0U; sector < 2U; sector++)
    {
        for (uint32_t index = 0U; index < PID_STORE_RECORDS_PER_SECTOR; index++)
        {
            uint32_t address = sector_addresses[sector] + index * PID_STORE_RECORD_SIZE_BYTES;
            uint8_t empty = PidStore_ReadRecord(address, words);
            if (empty != 0U)
            {
                if (scan->free_address[sector] == 0U) { scan->free_address[sector] = address; }
            }
            else if (PidStore_RecordIsValid(words, &candidate) != 0U &&
                (scan->have_latest == 0U ||
                 PidStore_SequenceIsNewer(words[2], scan->latest_sequence) != 0U))
            {
                scan->latest_params = candidate;
                scan->latest_sequence = words[2];
                scan->latest_sector = sector;
                scan->have_latest = 1U;
            }
        }
    }
}

uint8_t PidStore_Load(PidStore_Params *params)
{
    PidStore_Scan scan;
    if (params == NULL || PidStore_BackendAvailable() == 0U) { return 0U; }
    PidStore_ScanRecords(&scan);
    if (scan.have_latest == 0U) { return 0U; }
    *params = scan.latest_params;
    return 1U;
}

PidStore_Result PidStore_Save(const PidStore_Params *params)
{
    PidStore_Scan scan;
    PidStore_Params snapshot, verified;
    uint32_t words[PID_STORE_RECORD_WORDS], readback[PID_STORE_RECORD_WORDS];
    uint32_t target_sector, target_address;
    uint8_t need_erase = 0U;
    PidStore_Result result = PID_STORE_FLASH_ERROR;

    if (params == NULL) { return PID_STORE_INVALID; }
    snapshot = *params; // 保存过程中不再次读取调用方正在用的结构体。
    if (PidStore_Validate(&snapshot) == 0U) { return PID_STORE_INVALID; }
    if (PidStore_BackendAvailable() == 0U) { return PID_STORE_FLASH_ERROR; }
    PidStore_ScanRecords(&scan);
    if (scan.have_latest != 0U && PidStore_ParamsEqual(&snapshot, &scan.latest_params) != 0U)
    {
        return PID_STORE_UNCHANGED; // 连 Flash 解锁也不做。
    }

    target_sector = (scan.have_latest != 0U) ? scan.latest_sector : 0U;
    target_address = scan.free_address[target_sector];
    if (target_address == 0U)
    {
        target_sector ^= 1U; // 当前满了，只考虑另一个不包含最新记录的扇区。
        target_address = scan.free_address[target_sector];
        if (target_address == 0U)
        {
            need_erase = 1U;
            target_address = sector_addresses[target_sector];
        }
    }
    /* 无 valid 记录时任一扇区都可回收；有 valid 时绝不能擦它所在扇区。 */
    if (need_erase != 0U && scan.have_latest != 0U && target_sector == scan.latest_sector)
    {
        return PID_STORE_FLASH_ERROR;
    }

    for (uint32_t i = 0U; i < PID_STORE_RECORD_WORDS; i++) { words[i] = 0xFFFFFFFFU; }
    words[0] = PID_STORE_RECORD_MAGIC;
    words[1] = PID_STORE_FORMAT_VERSION;
    words[2] = (scan.have_latest != 0U) ? scan.latest_sequence + 1U : 1U;
    PidStore_ParamsToWords(&snapshot, &words[3]);
    words[14] = PidStore_Crc32(words);
    words[15] = PID_STORE_COMMIT_WORD;

    if (PidStore_Backend_Unlock() == 0U)
    {
        (void)PidStore_Backend_Lock(); // 即使后端部分解锁后报错，也尝试恢复写保护。
        return PID_STORE_FLASH_ERROR;
    }
    if (need_erase != 0U && PidStore_Backend_EraseSector(sector_numbers[target_sector]) == 0U)
    {
        goto finish;
    }
    if (PidStore_ReadRecord(target_address, readback) == 0U) { goto finish; }

    for (uint32_t i = 0U; i < 15U; i++)
    {
        if (words[i] != 0xFFFFFFFFU &&
            PidStore_Backend_ProgramWord(target_address + i * 4U, words[i]) == 0U)
        {
            goto finish;
        }
        if (PidStore_Backend_ReadWord(target_address + i * 4U) != words[i]) { goto finish; }
    }
    /* 所有 payload/CRC 先读回完整，commit 最后写；半写记录永远不当作成功。 */
    for (uint32_t i = 0U; i < 15U; i++)
    {
        if (PidStore_Backend_ReadWord(target_address + i * 4U) != words[i]) { goto finish; }
    }
    if (PidStore_Backend_ProgramWord(target_address + 60U, words[15]) == 0U) { goto finish; }
    (void)PidStore_ReadRecord(target_address, readback);
    if (PidStore_RecordIsValid(readback, &verified) != 0U &&
        readback[2] == words[2] && PidStore_ParamsEqual(&verified, &snapshot) != 0U)
    {
        result = PID_STORE_SAVED;
    }
finish:
    if (PidStore_Backend_Lock() == 0U) { result = PID_STORE_FLASH_ERROR; }
    return result;
}

