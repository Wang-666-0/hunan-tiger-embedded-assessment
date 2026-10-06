#ifndef GM6020_H
#define GM6020_H

#include "app_config.h"
#include "can_protocol.h"

#define GM6020_FEEDBACK_ID (0x204U + GM6020_MOTOR_ID)

typedef struct
{
    uint16_t encoder;       /* 单圈编码位置，0～8191。 */
    float angle_deg;        /* 单圈绝对机械角度，[0,360)，没有做零点校准。 */
    int16_t speed_rpm;      /* 带符号转速，rpm。 */
    int16_t current_raw;    /* 手册没有提供反馈电流的安培换算比例。 */
    uint8_t temperature;   /* 摄氏度。 */
    uint32_t rx_count; // 成功解码并发布的累计反馈数，零表示尚未收到有效反馈。
    uint32_t last_rx_ms; // 中断实际搬运反馈的毫秒时刻，用于在线检查。
} GM6020_Feedback;

extern volatile GM6020_Feedback gm6020_feedback;

/* 纯解包函数，不访问硬件/RTOS。失败时不改变 result。 */
uint8_t GM6020_DecodeFeedback(const CanRxMessage *message, GM6020_Feedback *result);
/* 接收任务调用：校验、解包、打时间戳、发布到全局 Watch 变量。 */
uint8_t GM6020_ParseFeedback(const CanRxMessage *message);
/* 任务中读取一致快照，volatile 本身不保证多个字段同时更新。 */
void GM6020_GetFeedback(GM6020_Feedback *snapshot);
uint8_t GM6020_IsOnline(const GM6020_Feedback *snapshot, uint32_t now_ms);

/* mode 是协议选择，不能通过本函数切换电机内部电流环。
 * raw 是带符号的手册给定值，其单位不是 rpm，也不是伏特。
 * 返回 1 成功；返回 0 表示 ID/mode/raw 或指针无效。 */
uint8_t GM6020_BuildCommand(uint8_t motor_id, uint8_t mode, int16_t raw,
                           uint32_t *can_id, uint8_t data[8]);

#endif /* GM6020_H */
