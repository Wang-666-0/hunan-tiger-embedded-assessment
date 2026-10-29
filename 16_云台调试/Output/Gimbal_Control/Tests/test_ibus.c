#include "ibus.h" /* 编译真实库，在 ARM 模拟器运行；不访问物理硬件。 */

#include <stdint.h>
#include <string.h>

volatile uint32_t test_checks;
volatile uint32_t test_failures;
volatile uint32_t test_failed_line;
volatile uint32_t test_done;
volatile uint32_t test_stage;

#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (test_failed_line == 0U) { test_failed_line = __LINE__; } } } while (0)

/* 测试端自行组包；已知帧另用固定字节，避免只测试同一份组包公式。 */
static void MakeFrame(uint8_t frame[32], uint16_t channels[14])
{
    uint16_t sum = 0U;
    uint16_t checksum;
    uint8_t index;
    frame[0] = 0x20U;
    frame[1] = 0x40U;
    for (index = 0U; index < 14U; index++)
    {
        frame[2U + 2U * index] = (uint8_t)channels[index];
        frame[3U + 2U * index] = (uint8_t)(channels[index] >> 8U);
    }
    for (index = 0U; index < 30U; index++)
    {
        sum = (uint16_t)(sum + frame[index]);
    }
    checksum = (uint16_t)(65535U - sum);
    frame[30] = (uint8_t)checksum;
    frame[31] = (uint8_t)(checksum >> 8U);
}

static void FillChannels(uint16_t channels[14], uint16_t first)
{
    uint8_t index;
    for (index = 0U; index < 14U; index++)
    {
        channels[index] = (uint16_t)(first + index * 37U);
    }
}

static void CheckFrame(const ibus_frame_t *actual, const uint16_t expected[14], uint32_t time)
{
    uint8_t index;
    for (index = 0U; index < 14U; index++)
    {
        CHECK(actual->channels[index] == expected[index]);
    }
    CHECK(actual->received_ms == time);
}

static void TestKnownAndStreaming(void)
{
    static const uint8_t known[32] = {
        0x20U,0x40U,0xDCU,0x05U,0xDCU,0x05U,0xDCU,0x05U,
        0xDCU,0x05U,0xDCU,0x05U,0xDCU,0x05U,0xDCU,0x05U,
        0xDCU,0x05U,0xDCU,0x05U,0xDCU,0x05U,0xDCU,0x05U,
        0xDCU,0x05U,0xDCU,0x05U,0xDCU,0x05U,0x51U,0xF3U
    };
    ibus_parser_t parser;
    ibus_frame_t out;
    uint16_t channels[14];
    uint8_t frame[32];
    uint8_t index;
    memset(&out, 0xA5, sizeof(out));
    ibus_init(&parser);
    CHECK(parser.has_frame == 0U);
    CHECK(parser.stats.valid_frames == 0U);
    CHECK(parser.checked_channels == 10U);
    CHECK(parser.inter_byte_timeout_ms == 5U);
    for (index = 0U; index < 31U; index++)
    {
        CHECK(ibus_feed_byte(&parser, known[index], 100U + index / 11U, &out) == 0U);
        CHECK(parser.has_frame == 0U);
        CHECK(parser.stats.valid_frames == 0U);
    }
    CHECK(ibus_feed_byte(&parser, known[31], 103U, &out) == 1U);
    CHECK(parser.has_frame == 1U);
    CHECK(parser.stats.valid_frames == 1U);
    CHECK(parser.buffered == 0U);
    for (index = 0U; index < 14U; index++)
    {
        channels[index] = 1500U;
    }
    CheckFrame(&out, channels, 103U);
    CheckFrame(&parser.latest, channels, 103U);

    FillChannels(channels, 1001U);
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, sizeof(frame), 110U, &out) == 1U);
    CheckFrame(&out, channels, 110U);
    CHECK(parser.stats.valid_frames == 2U);
    CHECK(parser.stats.bad_frames == 0U);
    CHECK(parser.stats.discarded_bytes == 0U);
}

static void TestBadChecksumsAndNoise(void)
{
    uint8_t frame[32];
    uint8_t damaged[32];
    uint16_t channels[14];
    ibus_parser_t parser;
    ibus_frame_t out;
    ibus_frame_t before;
    uint8_t index;
    static const uint8_t noise[] = {0xFFU,0x00U,0x40U,0x20U,0x20U,0x01U,0x20U};
    FillChannels(channels, 1103U);
    MakeFrame(frame, channels);
    for (index = 0U; index < 32U; index++)
    {
        ibus_init(&parser);
        CHECK(ibus_feed(&parser, frame, 32U, 1U, &out) == 1U);
        before = out;
        memcpy(damaged, frame, sizeof(damaged));
        damaged[index] ^= 1U;
        CHECK(ibus_feed(&parser, damaged, 32U, 2U, &out) == 0U);
        CHECK(parser.stats.valid_frames == 1U);
        CHECK(memcmp(&before, &out, sizeof(out)) == 0);
        CHECK(memcmp(&before, &parser.latest, sizeof(before)) == 0);
        if (index >= 2U)
        {
            CHECK(parser.stats.checksum_errors == 1U);
            CHECK(parser.stats.bad_frames == 1U);
        }
        CHECK(ibus_feed(&parser, frame, 32U, 3U, &out) == 1U);
        CHECK(parser.stats.valid_frames == 2U);
        CheckFrame(&out, channels, 3U);
    }
    ibus_init(&parser);
    CHECK(ibus_feed(&parser, noise, sizeof(noise), 1U, NULL) == 0U);
    CHECK(parser.buffered == 1U);
    CHECK(parser.has_frame == 0U);
    CHECK(parser.stats.discarded_bytes == sizeof(noise) - 1U);
    CHECK(ibus_feed(&parser, frame, sizeof(frame), 2U, &out) == 1U);
    CheckFrame(&out, channels, 2U);
    CHECK(parser.stats.discarded_bytes == sizeof(noise));
}

static void TestDroppedAndInsertedBytes(void)
{
    uint8_t first[32];
    uint8_t second[32];
    uint8_t damaged[33];
    uint16_t channels[14];
    ibus_parser_t parser;
    ibus_frame_t out;
    uint8_t omitted;
    uint8_t index;
    FillChannels(channels, 1201U);
    MakeFrame(first, channels);
    FillChannels(channels, 1307U);
    MakeFrame(second, channels);
    /* 每一个位置丢一个字节，下一帧都必须完整恢复。 */
    for (omitted = 0U; omitted < 32U; omitted++)
    {
        ibus_init(&parser);
        for (index = 0U; index < 32U; index++)
        {
            if (index != omitted)
            {
                CHECK(ibus_feed_byte(&parser, first[index], 1U, &out) == 0U);
            }
        }
        CHECK(ibus_feed(&parser, second, 32U, 2U, &out) == 1U);
        CHECK(parser.stats.valid_frames == 1U);
        CheckFrame(&out, channels, 2U);
    }
    /* 帧内每一个边界插入噪声；错误候选必须滑窗而非清空全部缓存。 */
    for (omitted = 1U; omitted < 32U; omitted++)
    {
        ibus_init(&parser);
        memcpy(damaged, first, omitted);
        damaged[omitted] = 0x7EU;
        memcpy(damaged + omitted + 1U, first + omitted, 32U - omitted);
        CHECK(ibus_feed(&parser, damaged, 33U, 1U, &out) == 0U);
        CHECK(ibus_feed(&parser, second, 32U, 2U, &out) == 1U);
        CHECK(parser.stats.valid_frames == 1U);
        CheckFrame(&out, channels, 2U);
    }
    /* 从前一帧中途接入流，下一帧仍然恢复。 */
    for (omitted = 1U; omitted < 32U; omitted++)
    {
        ibus_init(&parser);
        CHECK(ibus_feed(&parser, first + omitted, 32U - omitted, 1U, &out) == 0U);
        CHECK(ibus_feed(&parser, second, 32U, 2U, &out) == 1U);
        CheckFrame(&out, channels, 2U);
    }
}

static void TestChunksAndConcatenated(void)
{
    ibus_parser_t parser;
    ibus_frame_t out;
    uint8_t frame[32];
    uint8_t three[96];
    uint16_t channels[14];
    uint8_t split;
    FillChannels(channels, 1001U);
    MakeFrame(frame, channels);
    for (split = 0U; split <= 32U; split++)
    {
        ibus_init(&parser);
        CHECK(ibus_feed(&parser, frame, split, 10U, &out) == (size_t)(split == 32U));
        CHECK(ibus_feed(&parser, frame + split, 32U - split, 11U, &out) == (size_t)(split < 32U));
        CHECK(parser.stats.valid_frames == 1U);
        CheckFrame(&out, channels, split == 32U ? 10U : 11U);
    }
    memcpy(three, frame, 32U);
    FillChannels(channels, 1201U);
    MakeFrame(three + 32U, channels);
    FillChannels(channels, 1401U);
    MakeFrame(three + 64U, channels);
    ibus_init(&parser);
    CHECK(ibus_feed(&parser, three, sizeof(three), 20U, &out) == 3U);
    CHECK(parser.stats.valid_frames == 3U);
    CHECK(parser.stats.bad_frames == 0U);
    CheckFrame(&out, channels, 20U);
    CHECK(ibus_feed(&parser, three, sizeof(three), 21U, NULL) == 3U);
    CHECK(parser.stats.valid_frames == 6U);
}

static void TestTimeoutAndWrap(void)
{
    ibus_parser_t parser;
    ibus_frame_t out;
    ibus_frame_t before;
    uint8_t frame[32];
    uint16_t channels[14];
    FillChannels(channels, 1101U);
    MakeFrame(frame, channels);
    ibus_init(&parser);
    CHECK(ibus_feed(&parser, frame, 32U, 50U, &out) == 1U);
    before = out;
    CHECK(ibus_feed(&parser, frame, 16U, 100U, &out) == 0U);
    CHECK(ibus_poll_timeout(&parser, 104U) == 0U);
    CHECK(parser.buffered == 16U);
    CHECK(ibus_poll_timeout(&parser, 105U) == 1U);
    CHECK(parser.buffered == 0U);
    CHECK(parser.stats.partial_timeouts == 1U);
    CHECK(parser.stats.discarded_bytes == 16U);
    CHECK(ibus_poll_timeout(&parser, 200U) == 0U);
    CHECK(memcmp(&before, &parser.latest, sizeof(before)) == 0);
    CHECK(ibus_feed(&parser, frame + 16U, 16U, 106U, &out) == 0U);
    CHECK(ibus_feed(&parser, frame, 32U, 110U, &out) == 1U);
    CheckFrame(&out, channels, 110U);
    CHECK(parser.stats.valid_frames == 2U);
    CHECK(ibus_feed(&parser, frame, 3U, 200U, &out) == 0U);
    CHECK(ibus_feed(&parser, frame, 32U, 210U, &out) == 1U);
    CHECK(parser.stats.partial_timeouts == 2U);

    ibus_init(&parser);
    CHECK(ibus_feed(&parser, frame, 2U, UINT32_MAX - 1U, &out) == 0U);
    CHECK(ibus_poll_timeout(&parser, 2U) == 0U); /* 跨回卷实际只过了 4ms。 */
    CHECK(ibus_poll_timeout(&parser, 3U) == 1U);
    CHECK(parser.stats.partial_timeouts == 1U);
    CHECK(ibus_feed(&parser, frame, 32U, 4U, &out) == 1U);
    CheckFrame(&out, channels, 4U);
    parser.inter_byte_timeout_ms = 0U;
    CHECK(ibus_feed(&parser, frame, 2U, 100U, &out) == 0U);
    CHECK(ibus_poll_timeout(&parser, 999999U) == 0U);
    CHECK(ibus_feed(&parser, frame + 2U, 30U, 1000000U, &out) == 1U);
}

static void TestRangeAndFFFF(void)
{
    ibus_parser_t parser;
    ibus_frame_t out;
    uint16_t channels[14];
    uint8_t frame[32];
    uint8_t index;
    ibus_init(&parser);
    for (index = 0U; index < 14U; index++)
    {
        channels[index] = index % 2U == 0U ? 800U : 2200U;
    }
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, 32U, 1U, &out) == 1U);
    CheckFrame(&out, channels, 1U);
    channels[0] = 799U;
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, 32U, 2U, &out) == 0U);
    channels[0] = 2201U;
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, 32U, 3U, &out) == 0U);
    channels[0] = UINT16_MAX;
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, 32U, 4U, &out) == 0U);
    CHECK(parser.stats.range_errors == 3U);
    CHECK(parser.stats.bad_frames == 3U);
    CHECK(parser.stats.valid_frames == 1U);
    CHECK(parser.latest.received_ms == 1U);
    channels[0] = 1500U;
    for (index = 10U; index < 14U; index++)
    {
        channels[index] = UINT16_MAX; /* 未接的额外通道不应阻断 CH1～10。 */
    }
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, 32U, 5U, &out) == 1U);
    CheckFrame(&out, channels, 5U);
    CHECK(ibus_set_channel_validation(&parser, 14U, 800U, 2200U) == 1U);
    CHECK(ibus_feed(&parser, frame, 32U, 6U, &out) == 0U);
    CHECK(parser.stats.range_errors == 4U);
    CHECK(ibus_set_channel_validation(&parser, 15U, 0U, UINT16_MAX) == 0U);
    CHECK(parser.checked_channels == 14U);
    CHECK(ibus_set_channel_validation(&parser, 1U, 2200U, 800U) == 0U);
    CHECK(parser.channel_min == 800U);
    CHECK(ibus_set_channel_validation(&parser, 0U, 0U, UINT16_MAX) == 1U);
    for (index = 0U; index < 14U; index++)
    {
        channels[index] = UINT16_MAX;
    }
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, 32U, 7U, &out) == 1U);
    CheckFrame(&out, channels, 7U);
    CHECK(ibus_set_channel_validation(&parser, 14U, 0U, UINT16_MAX) == 1U);
    memset(channels, 0, sizeof(channels));
    MakeFrame(frame, channels);
    CHECK(ibus_feed(&parser, frame, 32U, 8U, &out) == 1U);
    CheckFrame(&out, channels, 8U);
}

static void TestNullArguments(void)
{
    ibus_parser_t parser;
    uint8_t byte = 0x20U;
    ibus_init(NULL);
    ibus_init(&parser);
    CHECK(ibus_set_channel_validation(NULL, 10U, 800U, 2200U) == 0U);
    CHECK(ibus_poll_timeout(NULL, 0U) == 0U);
    CHECK(ibus_feed_byte(NULL, byte, 0U, NULL) == 0U);
    CHECK(ibus_feed(NULL, &byte, 1U, 0U, NULL) == 0U);
    CHECK(ibus_feed(&parser, NULL, 1U, 0U, NULL) == 0U);
    CHECK(ibus_feed(&parser, &byte, 0U, 0U, NULL) == 0U);
    CHECK(parser.buffered == 0U);
}

void test_run(void)
{
    test_stage = 1U;
    TestKnownAndStreaming();
    test_stage = 2U;
    TestBadChecksumsAndNoise();
    test_stage = 3U;
    TestDroppedAndInsertedBytes();
    test_stage = 4U;
    TestChunksAndConcatenated();
    test_stage = 5U;
    TestTimeoutAndWrap();
    test_stage = 6U;
    TestRangeAndFFFF();
    test_stage = 7U;
    TestNullArguments();
    test_done = 1U;
}
