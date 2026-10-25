#ifndef TEST_BUS_CONTENTION_H
#define TEST_BUS_CONTENTION_H
static uint32_t fixture_other[2];
static inline HAL_StatusTypeDef Fixture_OtherTx(CAN_HandleTypeDef *bus)
{
    CAN_TxHeaderTypeDef header = {0}; uint8_t data[8] = {0}; uint32_t mailbox;
    HAL_StatusTypeDef result;
    header.IDE = CAN_ID_STD; header.StdId = 0x123U; header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    result = HAL_CAN_AddTxMessage(bus, &header, data, &mailbox);
    if (result == HAL_OK) { fixture_other[bus == &hcan1 ? 0U : 1U] = mailbox; }
    return result;
}
static inline void Fixture_CancelOther(void)
{
    if (fixture_other[0]) { (void)HAL_CAN_AbortTxRequest(&hcan1, fixture_other[0]); }
    if (fixture_other[1]) { (void)HAL_CAN_AbortTxRequest(&hcan2, fixture_other[1]); }
    fixture_other[0] = fixture_other[1] = 0U;
}
#endif
