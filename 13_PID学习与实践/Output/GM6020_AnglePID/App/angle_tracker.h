#ifndef ANGLE_TRACKER_H
#define ANGLE_TRACKER_H

#include <stdint.h>

/* 编码器每圈 8192 格。只有连续且可信的帧才能维护多圈位置；
 * 反馈中断后禁止根据 speed_rpm 猜测丢失了几圈，必须重新置零。 */
typedef struct
{
    int64_t accumulated_counts;
    uint16_t last_encoder;
    uint32_t last_rx_ms;
    uint32_t epoch; /* 首帧建立参考或显式置零时增加，供控制层重置旧状态。 */
    uint8_t initialized;
    uint8_t valid; /* 失效后锁存为 0，只有显式 Zero 可以恢复。 */
} AngleTracker;

/* 纯数学模块：无 HAL、RTOS 和共享变量，可用 PC 测试回绕与异常帧。 */
void AngleTracker_Init(AngleTracker *tracker);
uint8_t AngleTracker_Update(AngleTracker *tracker, uint16_t encoder,
                            int16_t speed_rpm, uint32_t now_ms);
/* now_ms 是当前时间；参考帧仍保留真实接收时间，不用置零刷新在线状态。 */
uint8_t AngleTracker_Zero(AngleTracker *tracker, uint16_t encoder,
                          uint32_t now_ms);
void AngleTracker_CheckAge(AngleTracker *tracker, uint32_t now_ms);
float AngleTracker_AngleDeg(const AngleTracker *tracker);

#endif /* ANGLE_TRACKER_H */
