#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

typedef struct
{
    uint8_t source; /* 接收控制器编号：1=CAN1，2=CAN2，不是电机 ID。 */
    uint32_t id; // 收到的报文 ID，与电机编号不同。
    uint8_t dlc; // 报文的有效字节数，GM6020 反馈要求为 8。
    uint8_t data[8]; // 接收数据副本，高字节在前。
    uint32_t received_ms; /* 中断搬运时的时间，排队延迟不刷新在线时间。 */
} CanRxMessage;

/* 大端：数组中高字节在前。参数指向至少两个可访问字节。 */
uint16_t CanProtocol_ReadU16BE(const uint8_t *data);
int16_t CanProtocol_ReadI16BE(const uint8_t *data);
void CanProtocol_WriteI16BE(uint8_t *data, int16_t value);

#endif /* CAN_PROTOCOL_H */
