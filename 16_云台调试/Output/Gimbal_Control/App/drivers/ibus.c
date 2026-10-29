/**
 * @file ibus.c
 * 作用：iBUS 帧解析与校验
 * 对外函数：
 *   ibus_feed_byte / ibus_feed / ibus_init / ibus_set_channel_validation / ibus_poll_timeout
 * 内部函数：
 *   ibus_drop_first / ibus_find_header
 * 文件级变量（函数局部变量在各函数内定义）：
 *   无；控制状态由调用方提供的结构体保存。
 */
#include "ibus.h"

#include <string.h>

/* 丢弃一个候选起点后仍保留后续字节，坏帧/丢字节不会跳过下一帧。 */

/* 内部函数前置声明：主流程先展示，辅助实现放在文件后方。 */
static void ibus_drop_first(ibus_parser_t *parser);
static void ibus_find_header(ibus_parser_t *parser);

uint8_t ibus_feed_byte(ibus_parser_t *parser, uint8_t byte, uint32_t now_ms,
                       ibus_frame_t *out)
/*
 * 作用：处理一个带时间戳的字节，完成帧头、通道和校验和核对后输出有效帧。
 * 参数 parser：持续保留的 iBUS 流式解析状态。
 * 参数 byte：本次收到的一个字节。
 * 参数 now_ms：当前毫秒时间。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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
/*
 * 作用：依次把一批字节送入同一解析器，返回有效帧数量。
 * 参数 parser：持续保留的 iBUS 流式解析状态。
 * 参数 data：协议原始字节缓冲区。
 * 参数 length：输入数据的有效字节数。
 * 参数 now_ms：当前毫秒时间。
 * 参数 out：接收结果的输出结构体/缓冲区。
 * 返回本批输入中成功解析出的有效帧数量。
 */
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

void ibus_init(ibus_parser_t *parser)
/*
 * 作用：初始化 iBUS 解析状态、默认通道校验范围和接收统计。
 * 参数 parser：持续保留的 iBUS 流式解析状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
{
    if (parser == NULL)
    {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->inter_byte_timeout_ms = 5U;
    parser->checked_channels = 10U;
    parser->channel_min = 800U;
    parser->channel_max = 2200U;
}

uint8_t ibus_set_channel_validation(ibus_parser_t *parser,
                                    uint8_t count, uint16_t min, uint16_t max)
/*
 * 作用：校验并设置需要检查的通道数和通道数值范围。
 * 参数 parser：持续保留的 iBUS 流式解析状态。
 * 参数 count：需要检查数值范围的 iBUS 通道数。
 * 参数 min：允许的最小通道值。
 * 参数 max：允许的最大通道值。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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

static void ibus_drop_first(ibus_parser_t *parser)
/*
 * 作用：丢弃候选缓冲的首字节，继续在剩余数据中寻找有效帧。
 * 参数 parser：持续保留的 iBUS 流式解析状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
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
/*
 * 作用：在缓冲中定位 iBUS 帧头并保留可继续拼接的候选数据。
 * 参数 parser：持续保留的 iBUS 流式解析状态。
 * 不返回数值；处理结果保存在传入结构体或本文件维护的状态中。
 */
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

uint8_t ibus_poll_timeout(ibus_parser_t *parser, uint32_t now_ms)
/*
 * 作用：检查候选帧的字节间隔，超时后放弃未完成帧。
 * 参数 parser：持续保留的 iBUS 流式解析状态。
 * 参数 now_ms：当前毫秒时间。
 * 返回 1 表示本次校验/请求被接受，0 表示条件不满足；接受请求不代表动作已经完成。
 */
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
