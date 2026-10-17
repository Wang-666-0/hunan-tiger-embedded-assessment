/* Run the production MX_CANx_Init and ST HAL against simulated registers.
 * This fixture does not model a physical CAN bus or motor. */
#include "can.h"
#include "app_config.h"

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(x) do { ++test_checks; if (!(x)) { ++test_failures; \
    if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)
static uint32_t error_callbacks, last_error, completed, tick;
static uint32_t gpio_calls, irq_enabled, irq_priorities;

void Error_Handler(void) { CHECK(0); }
uint32_t HAL_GetTick(void) { return tick++; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    CHECK(init->Mode == GPIO_MODE_AF_PP && init->Alternate == GPIO_AF9_CAN1);
    CHECK((port == GPIOA && init->Pin == (GPIO_PIN_11 | GPIO_PIN_12)) ||
          (port == GPIOB && init->Pin == (GPIO_PIN_12 | GPIO_PIN_13)));
    ++gpio_calls;
}
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub)
{
    (void)irq;
    CHECK(preempt == 5U && sub == 0U);
    ++irq_priorities;
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; ++irq_enabled; }
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle)
{
    ++error_callbacks;
    last_error = HAL_CAN_GetError(handle);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *handle)
{ CHECK(handle == &hcan1); ++completed; }

void test_run(void)
{
    const uint32_t all = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    const uint32_t errors = CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_WARNING |
        CAN_IT_ERROR_PASSIVE | CAN_IT_LAST_ERROR_CODE;
    const uint32_t timing = CAN_SJW_1TQ | CAN_BS1_11TQ | CAN_BS2_2TQ | 2U;
    test_stage = 1U;
    /* Simulated hardware initialization acknowledgements; actual init code runs. */
    CAN1->MCR = CAN_MCR_NART;
    CAN2->MCR = CAN_MCR_NART;
    CAN1->MSR = CAN_MSR_INAK;
    CAN2->MSR = CAN_MSR_INAK;
    MX_CAN1_Init();
    MX_CAN2_Init();
    CHECK(hcan1.State == HAL_CAN_STATE_READY && hcan2.State == HAL_CAN_STATE_READY);
    CHECK(hcan1.Init.AutoRetransmission == ENABLE && !(CAN1->MCR & CAN_MCR_NART));
    CHECK(hcan2.Init.AutoRetransmission == DISABLE && (CAN2->MCR & CAN_MCR_NART));
    CHECK(hcan1.Init.Prescaler == 3U && hcan2.Init.Prescaler == 3U);
    CHECK(CAN1->BTR == timing && CAN2->BTR == timing);
    CHECK((CAN1->MCR & CAN_MCR_ABOM) && (CAN2->MCR & CAN_MCR_ABOM));
    CHECK(!(CAN1->MCR & CAN_MCR_TXFP) && !(CAN2->MCR & CAN_MCR_TXFP));
    CHECK(gpio_calls == 2U && irq_enabled == 6U && irq_priorities == 6U);

    test_stage = 2U;
    hcan1.State = hcan2.State = HAL_CAN_STATE_LISTENING;
    CAN1->MSR = CAN1->ESR = CAN1->RF0R = CAN1->RF1R = 0U;
    CHECK(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | errors) == HAL_OK);
    CHECK((CAN1->IER & errors) == errors);
    /* Arbitration lost while the request is still pending: RQCP stays clear.
     * HAL must wait for completion instead of reporting a finished failure. */
    CAN1->TSR = CAN_TSR_ALST1 | CAN_TSR_TME0 | CAN_TSR_TME2;
    CHECK(HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX1) == 1U);
    HAL_CAN_IRQHandler(&hcan1);
    CHECK(error_callbacks == 0U && completed == 0U && HAL_CAN_GetError(&hcan1) == 0U);
    CAN1->MSR = 0U;
    CAN1->TSR = CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_ALST1;
    HAL_CAN_IRQHandler(&hcan1);
    CHECK(error_callbacks == 0U && completed == 1U && HAL_CAN_GetError(&hcan1) == 0U);

    test_stage = 3U;
    /* ACK still reaches the error callback before a transmit request completes. */
    CAN1->TSR = 0U;
    CAN1->MSR = CAN_MSR_ERRI;
    CAN1->ESR = CAN_ESR_LEC_1 | CAN_ESR_LEC_0;
    HAL_CAN_IRQHandler(&hcan1);
    CHECK(error_callbacks == 1U && last_error == HAL_CAN_ERROR_ACK);
    CHECK(HAL_CAN_IsTxMessagePending(&hcan1, all) == 1U);
    CHECK(HAL_CAN_AbortTxRequest(&hcan1, all) == HAL_OK);
    CHECK((CAN1->TSR & (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2)) ==
          (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2));
    /* Warning/passive/BusOff are also still reported, preserving earlier ACK. */
    CAN1->TSR = 0U;
    CAN1->MSR = CAN_MSR_ERRI;
    CAN1->ESR = CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF;
    HAL_CAN_IRQHandler(&hcan1);
    CHECK(error_callbacks == 2U);
    CHECK(last_error == (HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_EWG));
    /* An actually completed arbitration failure remains visible too. */
    CAN1->MSR = CAN1->ESR = 0U;
    CAN1->TSR = CAN_TSR_RQCP1 | CAN_TSR_ALST1;
    HAL_CAN_IRQHandler(&hcan1);
    CHECK(error_callbacks == 3U && (last_error & HAL_CAN_ERROR_TX_ALST1));
    CHECK(HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX1) == HAL_OK);
    CHECK(CAN1->TSR & CAN_TSR_ABRQ1);
    test_done = 1U;
}
