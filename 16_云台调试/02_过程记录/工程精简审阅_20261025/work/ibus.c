/* iBUS 帧解析与错误后重同步。 */
#include "ibus.h"

#include <string.h>

/* 丢弃一个候选起点后仍保留后续字节，坏帧/丢字节不会跳过下一帧。 */
static void ibus_drop_first(ibus_parser_t *parser)
{
    if (parser->buffered != 0U)
    {
        parser->buffered--;
        if (parser->buffered != 0U)
        {
            memmove(parser->buffer, parser->buffer + 1U, parser->buffered);
        }
        parser->stats.discarded_bytes++;
    }
}

/* 即使块在 0x20 后结束，也保留该字节等待下一次的 0x40。 */
static void ibus_find_header(ibus_parser_t *parser)
{
    while (parser->buffered != 0U)
    {
        if (parser->buffer[0] == 0x20U)
        {
            if ((parser->buffered == 1U) || (parser->buffer[1] == 0x40U))
            {
                break;
            }
        }
        ibus_drop_first(parser);
    }
}

void ibus_init(ibus_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->inter_byte_timeout_ms = IBUS_DEFAULT_BYTE_TIMEOUT_MS;
    parser->checked_channels = IBUS_DEFAULT_CHECKED_CHANNELS;
    parser->channel_min = IBUS_DEFAULT_CHANNEL_MIN;
    parser->channel_max = IBUS_DEFAULT_CHANNEL_MAX;
}

uint8_t ibus_set_channel_validation(ibus_parser_t *parser,
                                    uint8_t count, uint16_t min, uint16_t max)
{
    if ((parser == NULL) || (count > IBUS_CHANNEL_COUNT) || (min > max))
    {
        return 0U;
    }
    parser->checked_channels = count;
    parser->channel_min = min;
    parser->channel_max = max;
    return 1U;
}

uint8_t ibus_poll_timeout(ibus_parser_t *parser, uint32_t now_ms)
{
    if ((parser == NULL) || (parser->buffered == 0U) ||
        (parser->inter_byte_timeout_ms == 0U))
    {
        return 0U;
    }
    /* 无符号减法支持毫秒计数从 UINT32_MAX 回卷到 0。 */
    if ((uint32_t)(now_ms - parser->last_byte_ms) < parser->inter_byte_timeout_ms)
    {
        return 0U;
    }
    parser->stats.discarded_bytes += parser->buffered;
    parser->buffered = 0U;
    parser->stats.partial_timeouts++;
    return 1U;
}

uint8_t ibus_feed_byte(ibus_parser_t *parser, uint8_t byte, uint32_t now_ms,
                       ibus_frame_t *out)
{
    uint16_t sum;
    uint16_t checksum;
    uint16_t received_checksum;
    uint16_t value;
    uint8_t index;
    ibus_frame_t candidate;

    if (parser == NULL)
    {
        return 0U;
    }
    (void)ibus_poll_timeout(parser, now_ms);
    parser->last_byte_ms = now_ms;
    parser->buffer[parser->buffered++] = byte;
    ibus_find_header(parser);
    if (parser->buffered < IBUS_FRAME_SIZE)
    {
        return 0U;
    }

    sum = 0U;
    for (index = 0U; index < IBUS_FRAME_SIZE - 2U; index++)
    {
        sum = (uint16_t)(sum + parser->buffer[index]);
    }
    /* 校验和为前 30 字节之和的 16 位反码：0xFFFF - sum。 */
    checksum = (uint16_t)(0xFFFFU - sum);
    received_checksum = (uint16_t)((uint16_t)parser->buffer[30] |
                                   ((uint16_t)parser->buffer[31] << 8U));
    if (checksum != received_checksum)
    {
        parser->stats.checksum_errors++;
        parser->stats.bad_frames++;
        ibus_drop_first(parser);
        ibus_find_header(parser);
        return 0U;
    }

    for (index = 0U; index < IBUS_CHANNEL_COUNT; index++)
    {
        value = (uint16_t)((uint16_t)parser->buffer[2U + 2U * index] |
                          ((uint16_t)parser->buffer[3U + 2U * index] << 8U));
        if ((index < parser->checked_channels) &&
            ((value < parser->channel_min) || (value > parser->channel_max)))
        {
            parser->stats.range_errors++;
            parser->stats.bad_frames++;
            ibus_drop_first(parser);
            ibus_find_header(parser);
            return 0U;
        }
        candidate.channels[index] = value;
    }
    candidate.received_ms = now_ms;
    parser->latest = candidate;
    parser->has_frame = 1U;
    parser->stats.valid_frames++;
    parser->buffered = 0U;
    if (out != NULL)
    {
        *out = candidate;
    }
    return 1U;
}

size_t ibus_feed(ibus_parser_t *parser, const uint8_t *data, size_t length,
                 uint32_t now_ms, ibus_frame_t *out)
{
    size_t index;
    size_t frames;

    if ((parser == NULL) || (data == NULL))
    {
        return 0U;
    }
    frames = 0U;
    for (index = 0U; index < length; index++)
    {
        frames += ibus_feed_byte(parser, data[index], now_ms, out);
    }
    return frames;
}
