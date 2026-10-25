/**
 * @file ibus.h
 * @brief iBUS 解析接口
 * 职责：定义解析器、有效帧和统计信息，支持按字节或块输入。
 * 主要接口：ibus_init, ibus_set_channel_validation, ibus_poll_timeout, ibus_feed_byte, ibus_feed。
 * 实现/调用：remote_input.c；处理帧头、校验和、通道范围、字节超时与丢字节重同步。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef IBUS_H
#define IBUS_H
#include "app_config.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 富斯 iBUS 舵机数据：115200、8N1、非反相，32 字节一帧。 */

typedef struct
{
    /* 下标 0 对应遥控器 CH1；值为微秒，通常约 1000～2000。 */
    uint16_t channels[IBUS_CHANNEL_COUNT];
    uint32_t received_ms;
} ibus_frame_t;

typedef struct
{
    uint32_t valid_frames;
    /* bad_frames = checksum_errors + range_errors，不含噪声和断帧。 */
    uint32_t bad_frames;
    uint32_t checksum_errors;
    uint32_t range_errors;
    uint32_t discarded_bytes;
    uint32_t partial_timeouts;
} ibus_stats_t;

typedef struct
{
    /* 只有通过校验和通道范围检查的帧才会更新 latest/has_frame。 */
    ibus_frame_t latest;
    ibus_stats_t stats;
    uint8_t has_frame;
    /* 允许调用方调整；0 表示关闭字节间隔超时。 */
    uint32_t inter_byte_timeout_ms;
    uint16_t channel_min;
    uint16_t channel_max;
    uint8_t checked_channels;

    /* 解析器内部状态：调用方无需修改。非线程安全，宜由同一任务操作。 */
    uint8_t buffer[IBUS_FRAME_SIZE];
    uint8_t buffered;
    uint32_t last_byte_ms;
} ibus_parser_t;

/* 默认仅检查 CH1～CH10 的 800～2200 范围；未使用的 CH11～14 仍被解包。 */
void ibus_init(ibus_parser_t *parser);

/* count=0 关闭范围检查；count>14 或 min>max 返回 0 且保持原配置。 */
uint8_t ibus_set_channel_validation(ibus_parser_t *parser,
                                    uint8_t count, uint16_t min, uint16_t max);

/* 残帧间隔达到阈值就丢弃，返回 1；保持最近合法帧供上层判断在线状态。
 * now_ms 使用 uint32_t 单调毫秒计数（如 HAL_GetTick），允许计数回卷。
 * 本函数不代替遥控断联判断；断联只能依据 latest.received_ms/合法帧事件。 */
uint8_t ibus_poll_timeout(ibus_parser_t *parser, uint32_t now_ms);

/* 逐字节滑窗重同步。完整合法帧返回 1，out 可为 NULL。
 * 出错帧不会覆盖 latest；now_ms 宜传入字节实际到达时刻。 */
uint8_t ibus_feed_byte(ibus_parser_t *parser, uint8_t byte, uint32_t now_ms,
                       ibus_frame_t *out);

/* 任意分块/粘包可调用此接口；返回合法帧数，out 保存该块最后的合法帧。
 * 同一块共享 now_ms；若逐字节记录了时间，优先使用 ibus_feed_byte。 */
size_t ibus_feed(ibus_parser_t *parser, const uint8_t *data, size_t length,
                 uint32_t now_ms, ibus_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IBUS_H */
