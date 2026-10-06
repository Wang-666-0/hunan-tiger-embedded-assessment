#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

#define CAN1_TEST_ID 0x201U
#define CAN2_TEST_ID 0x202U
#define CAN1_TEST_MARKER 0xA1U
#define CAN2_TEST_MARKER 0xA2U

typedef struct
{
    uint8_t source; /* 接收控制器编号：1=CAN1，2=CAN2，并非发送方编号 */
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanRxMessage;

void CanProtocol_Encode(uint8_t marker, uint32_t sequence, uint8_t data[8]);
uint32_t CanDecodeSequence(const uint8_t data[8]);
uint32_t CanProtocol_IsValid(const CanRxMessage *message);

#endif /* CAN_PROTOCOL_H */
