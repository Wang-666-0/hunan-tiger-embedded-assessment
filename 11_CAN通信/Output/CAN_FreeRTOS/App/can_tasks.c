#include "can_tasks.h"
#include "can_transport.h"
#include "can_debug.h"
#include "task.h"

#define CAN_TX_PERIOD_MS 100U
#define CAN_TASK_STACK_WORDS 256U

static void CanTxTask(void *argument)
{
    uint32_t sequence = 0;
    TickType_t last_wake;
    (void)argument;

    /* 队列已创建且调度器已运行，此时再启动 CAN 接收中断。 */
    CanStart();
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        CanSend(&hcan1, CAN1_TEST_ID, CAN1_TEST_MARKER, sequence);
        CanSend(&hcan2, CAN2_TEST_ID, CAN2_TEST_MARKER, sequence);
        sequence++;
        can_tx_task_cycle_count++;

        CanDebug_SampleHardware();

        /* 固定时间基准，避免把本轮执行时间累加进下轮周期。 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAN_TX_PERIOD_MS));
    }
}

static void CanRxTask(void *argument)
{
    CanRxMessage message;
    (void)argument;
    for (;;)
    {
        /* ISR 负责搬运，任务负责校验和更新 Watch，降低中断工作量。 */
        if (CanTransport_Receive(&message, portMAX_DELAY) == pdPASS)
            CanDebug_RecordRx(&message);
    }
}

void CanTasks_Create(void)
{
    /* 原生 FreeRTOS 栈深度 256 words = 1024 字节；CMSIS 栈属性单位不同。 */
    /* 接收优先级 4，发送优先级 3；接收任务无报文时阻塞。 */
    if (xTaskCreate(CanRxTask, "CAN_RX", CAN_TASK_STACK_WORDS, NULL, 4, NULL) != pdPASS)
    {
        CanFail(CAN_FAULT_RX_TASK);
    }
    if (xTaskCreate(CanTxTask, "CAN_TX", CAN_TASK_STACK_WORDS, NULL, 3, NULL) != pdPASS)
    {
        CanFail(CAN_FAULT_TX_TASK);
    }
}
