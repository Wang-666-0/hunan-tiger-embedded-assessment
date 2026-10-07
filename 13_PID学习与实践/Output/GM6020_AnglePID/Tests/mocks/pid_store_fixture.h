#ifndef PID_STORE_FIXTURE_H
#define PID_STORE_FIXTURE_H

/* RAM-only Flash model. The production Flash HAL backend is excluded by
 * PID_STORE_TEST_BACKEND. Address checks keep both unrelated projects out. */
#include "pid_store.h"
#include <string.h>

#define TEST_FLASH_CACHED_RECORDS 32U
static uint32_t flash_records[2][TEST_FLASH_CACHED_RECORDS][PID_STORE_RECORD_WORDS];
static uint8_t flash_full[2];
static uint8_t flash_unlocked;
static uint8_t flash_fail_unlock, flash_fail_erase;
static uint32_t flash_program_calls, flash_erase_calls, flash_unlock_calls, flash_lock_calls;
static uint32_t flash_fail_program_at, flash_range_errors;
static uint32_t flash_last_erased_sector;

static void TestFlash_Reset(void)
{
    memset(flash_records, 0xFF, sizeof(flash_records));
    memset(flash_full, 0, sizeof(flash_full));
    flash_unlocked = flash_fail_unlock = flash_fail_erase = 0U;
    flash_program_calls = flash_erase_calls = flash_unlock_calls = flash_lock_calls = 0U;
    flash_fail_program_at = flash_range_errors = 0U;
    flash_last_erased_sector = 0xFFFFFFFFU;
}

static uint8_t TestFlash_Locate(uint32_t address, uint32_t *bank, uint32_t *record, uint32_t *word)
{
    uint32_t offset;
    if (address < PID_STORE_SECTOR_A_ADDRESS ||
        address >= PID_STORE_SECTOR_B_ADDRESS + PID_STORE_SECTOR_SIZE_BYTES ||
        (address & 3U) != 0U)
    {
        flash_range_errors++;
        return 0U;
    }
    offset = address - PID_STORE_SECTOR_A_ADDRESS;
    *bank = offset / PID_STORE_SECTOR_SIZE_BYTES;
    offset %= PID_STORE_SECTOR_SIZE_BYTES;
    *record = offset / PID_STORE_RECORD_SIZE_BYTES;
    *word = (offset % PID_STORE_RECORD_SIZE_BYTES) / 4U;
    return 1U;
}

uint32_t PidStore_Backend_ReadWord(uint32_t address)
{
    uint32_t bank, record, word;
    if (TestFlash_Locate(address, &bank, &record, &word) == 0U) return 0U;
    if (record < TEST_FLASH_CACHED_RECORDS) return flash_records[bank][record][word];
    return flash_full[bank] ? 0U : 0xFFFFFFFFU;
}

uint8_t PidStore_Backend_Unlock(void)
{
    flash_unlock_calls++;
    if (flash_fail_unlock != 0U) return 0U;
    flash_unlocked = 1U;
    return 1U;
}

uint8_t PidStore_Backend_Lock(void)
{
    flash_lock_calls++;
    flash_unlocked = 0U;
    return 1U;
}

uint8_t PidStore_Backend_ProgramWord(uint32_t address, uint32_t value)
{
    uint32_t bank, record, word, previous;
    flash_program_calls++;
    if (flash_unlocked == 0U || flash_program_calls == flash_fail_program_at) return 0U;
    if (TestFlash_Locate(address, &bank, &record, &word) == 0U) return 0U;
    if (record >= TEST_FLASH_CACHED_RECORDS) { flash_range_errors++; return 0U; }
    previous = flash_records[bank][record][word];
    if ((previous & value) != value) return 0U; /* Flash cannot change a zero bit back to one. */
    flash_records[bank][record][word] = previous & value;
    return 1U;
}

uint8_t PidStore_Backend_EraseSector(uint32_t sector)
{
    uint32_t bank;
    flash_erase_calls++;
    flash_last_erased_sector = sector;
    if (flash_unlocked == 0U || flash_fail_erase != 0U) return 0U;
    if (sector == PID_STORE_SECTOR_A_NUMBER) bank = 0U;
    else if (sector == PID_STORE_SECTOR_B_NUMBER) bank = 1U;
    else { flash_range_errors++; return 0U; }
    memset(flash_records[bank], 0xFF, sizeof(flash_records[bank]));
    flash_full[bank] = 0U;
    return 1U;
}

static uint32_t TestFlash_FloatBits(float value)
{
    union { float value; uint32_t bits; } converter;
    converter.value = value;
    return converter.bits;
}

static uint32_t TestFlash_Crc(const uint32_t *words)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0U; i < 14U; i++)
    {
        for (uint32_t byte = 0U; byte < 4U; byte++)
        {
            crc ^= (words[i] >> (byte * 8U)) & 0xFFU;
            for (uint32_t bit = 0U; bit < 8U; bit++)
                crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

static PidStore_Params TestFlash_Params(void)
{
    PidStore_Params params = {31.0f, 180.0f, 0.004f, 0.0f, 0.0f, 0.0f};
#if PID_STORE_APP_KIND == 2U
    params.angle_kp = 3.5f;
    params.angle_ki = 0.25f;
    params.angle_kd = 0.06f;
#endif
    return params;
}

static void TestFlash_Record(uint32_t bank, uint32_t index, const PidStore_Params *params, uint32_t sequence)
{
    uint32_t *words = flash_records[bank][index];
    memset(words, 0xFF, PID_STORE_RECORD_SIZE_BYTES);
    words[0] = PID_STORE_RECORD_MAGIC;
    words[1] = PID_STORE_FORMAT_VERSION;
    words[2] = sequence;
    words[3] = TestFlash_FloatBits(params->speed_kp);
    words[4] = TestFlash_FloatBits(params->speed_ki);
    words[5] = TestFlash_FloatBits(params->speed_kd);
    words[6] = TestFlash_FloatBits(params->angle_kp);
    words[7] = TestFlash_FloatBits(params->angle_ki);
    words[8] = TestFlash_FloatBits(params->angle_kd);
    words[14] = TestFlash_Crc(words);
    words[15] = PID_STORE_COMMIT_WORD;
}

static void TestFlash_FillRemainder(uint32_t bank, uint32_t keep)
{
    for (uint32_t index = keep; index < TEST_FLASH_CACHED_RECORDS; index++)
        memset(flash_records[bank][index], 0, PID_STORE_RECORD_SIZE_BYTES);
    flash_full[bank] = 1U;
}

static void TestPidStore(void)
{
    PidStore_Params original = TestFlash_Params(), changed = original, loaded = original, invalid;
    uint32_t writes, unlocks, count_per_save;
    test_stage = 16U;
    TestFlash_Reset();
    CHECK(PidStore_Load(&loaded) == 0U && memcmp(&loaded, &original, sizeof(loaded)) == 0);
    CHECK(PidStore_Load(0) == 0U && PidStore_Save(0) == PID_STORE_INVALID);
    CHECK(PidStore_Validate(&original) == 1U && PidStore_Validate(0) == 0U);
    CHECK(PidStore_Save(&original) == PID_STORE_SAVED);
    count_per_save = flash_program_calls;
    CHECK(count_per_save == 11U && flash_erase_calls == 0U && flash_unlocked == 0U);
    CHECK(flash_records[0][0][15] == PID_STORE_COMMIT_WORD && flash_records[0][0][14] == TestFlash_Crc(flash_records[0][0]));
    CHECK(PidStore_Load(&loaded) == 1U && memcmp(&loaded, &original, sizeof(loaded)) == 0);
    writes = flash_program_calls; unlocks = flash_unlock_calls;
    CHECK(PidStore_Save(&original) == PID_STORE_UNCHANGED);
    CHECK(flash_program_calls == writes && flash_unlock_calls == unlocks && flash_erase_calls == 0U);
    invalid = original; invalid.speed_kp = -1.0f;
    CHECK(PidStore_Save(&invalid) == PID_STORE_INVALID);
    invalid = original; invalid.speed_ki = SPEED_PID_KI_MAX + 1.0f;
    CHECK(PidStore_Save(&invalid) == PID_STORE_INVALID);
    invalid = original; invalid.speed_kd = SPEED_PID_KD_MAX + 1.0f;
    CHECK(PidStore_Save(&invalid) == PID_STORE_INVALID);
    invalid = original; invalid.speed_kp = FloatFromBits(0x7FC00000U);
    CHECK(PidStore_Save(&invalid) == PID_STORE_INVALID);
    invalid = original; invalid.speed_ki = FloatFromBits(0x7F800000U);
    CHECK(PidStore_Save(&invalid) == PID_STORE_INVALID);
    invalid = original; invalid.angle_kp = 1000.0f;
    CHECK(PidStore_Save(&invalid) == PID_STORE_INVALID);
    CHECK(flash_program_calls == writes && flash_unlock_calls == unlocks);

    changed.speed_kp += 1.0f;
    /* Interrupt every actually programmed word, including the final commit. */
    for (uint32_t word = 1U; word <= count_per_save; word++)
    {
        flash_fail_program_at = flash_program_calls + word;
        CHECK(PidStore_Save(&changed) == PID_STORE_FLASH_ERROR);
        CHECK(flash_unlocked == 0U);
        CHECK(PidStore_Load(&loaded) == 1U && memcmp(&loaded, &original, sizeof(loaded)) == 0);
    }
    flash_fail_program_at = 0U;
    CHECK(PidStore_Save(&changed) == PID_STORE_SAVED);
    CHECK(PidStore_Load(&loaded) == 1U && memcmp(&loaded, &changed, sizeof(loaded)) == 0);
    CHECK(flash_range_errors == 0U);

    /* Build records directly only to introduce corruption, never to replace Save tests. */
    TestFlash_Reset(); TestFlash_Record(0U, 0U, &original, 1U); TestFlash_Record(0U, 1U, &changed, 2U);
    flash_records[0][1][3] ^= 1U; /* Payload altered without matching CRC. */
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == original.speed_kp);
    TestFlash_Record(0U, 1U, &changed, 2U); flash_records[0][1][15] = 0xFFFFFFFFU;
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == original.speed_kp);
    TestFlash_Record(0U, 1U, &changed, 2U); flash_records[0][1][1]++;
    flash_records[0][1][14] = TestFlash_Crc(flash_records[0][1]);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == original.speed_kp);
    TestFlash_Record(0U, 1U, &changed, 2U); flash_records[0][1][0] ^= 0x10000000U;
    flash_records[0][1][14] = TestFlash_Crc(flash_records[0][1]);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == original.speed_kp);
    invalid = changed; invalid.speed_kp = FloatFromBits(0x7FC00000U);
    TestFlash_Record(0U, 1U, &invalid, 2U);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == original.speed_kp); /* CRC cannot legalize NaN. */
    TestFlash_Reset(); TestFlash_Record(0U, 0U, &original, 0xFFFFFFFFU); TestFlash_Record(1U, 0U, &changed, 0U);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == changed.speed_kp); /* Sequence wrap. */

    TestFlash_Reset(); TestFlash_Record(0U, 0U, &original, 10U);
    TestFlash_FillRemainder(0U, 1U);
    CHECK(PidStore_Save(&changed) == PID_STORE_SAVED && flash_erase_calls == 0U);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == changed.speed_kp);
    TestFlash_Reset(); TestFlash_Record(0U, 0U, &original, 10U);
    TestFlash_FillRemainder(0U, 1U); TestFlash_FillRemainder(1U, 0U);
    flash_fail_erase = 1U;
    CHECK(PidStore_Save(&changed) == PID_STORE_FLASH_ERROR && flash_unlocked == 0U);
    CHECK(flash_last_erased_sector == PID_STORE_SECTOR_B_NUMBER);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == original.speed_kp);
    flash_fail_erase = 0U; flash_fail_program_at = flash_program_calls + count_per_save;
    CHECK(PidStore_Save(&changed) == PID_STORE_FLASH_ERROR && flash_unlocked == 0U);
    CHECK(flash_last_erased_sector == PID_STORE_SECTOR_B_NUMBER);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == original.speed_kp); /* Erase+partial new record retains old bank. */
    flash_fail_program_at = 0U;
    CHECK(PidStore_Save(&changed) == PID_STORE_SAVED);
    CHECK(PidStore_Load(&loaded) == 1U && loaded.speed_kp == changed.speed_kp);
    CHECK(flash_records[0][0][15] == PID_STORE_COMMIT_WORD && flash_range_errors == 0U);
    TestFlash_Reset(); flash_fail_unlock = 1U;
    CHECK(PidStore_Save(&original) == PID_STORE_FLASH_ERROR && flash_program_calls == 0U);
}

#endif
