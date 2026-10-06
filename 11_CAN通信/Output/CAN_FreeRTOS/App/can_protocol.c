#include "can_protocol.h"

void CanProtocol_Encode(uint8_t marker, uint32_t sequence, uint8_t data[8])
{
    /* 固定 8 字节：来源标记 + 12 34 56 + 小端 uint32 序号。
     * 逐字节编码，不用强转指针，避免对齐要求和 CPU 字节序影响。
     */
    data[0] = marker;
    data[1] = 0x12;
    data[2] = 0x34;
    data[3] = 0x56;
    data[4] = (uint8_t)sequence;
    data[5] = (uint8_t)(sequence >> 8);
    data[6] = (uint8_t)(sequence >> 16);
    data[7] = (uint8_t)(sequence >> 24);
}

/* 每个字节先转 uint32，再移位，明确高字节的类型和宽度。 */
uint32_t CanDecodeSequence(const uint8_t data[8])
{
    return (uint32_t)data[4] | ((uint32_t)data[5] << 8) | ((uint32_t)data[6] << 16) |
           ((uint32_t)data[7] << 24);
}

uint32_t CanProtocol_IsValid(const CanRxMessage *message)
{
    /* CAN1 应收到 CAN2 的 ID/标记，CAN2 应收到 CAN1 的 ID/标记。
     * 序号用于观察更新；这里不把序号连续性作为有效帧条件。
     */
    uint32_t expected_id = (message->source == 1U) ? CAN2_TEST_ID : CAN1_TEST_ID;
    uint8_t expected_marker = (message->source == 1U) ? CAN2_TEST_MARKER : CAN1_TEST_MARKER;
    return (message->id == expected_id && message->dlc == 8U &&
            message->data[0] == expected_marker && message->data[1] == 0x12U &&
            message->data[2] == 0x34U && message->data[3] == 0x56U);
}
