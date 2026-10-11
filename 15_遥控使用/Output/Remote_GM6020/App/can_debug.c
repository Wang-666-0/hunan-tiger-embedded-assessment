/*
 * 文件功能：集中保存基础收发统计，并提供默认关闭的额外诊断记录。
 * 1. 提供收发计数、最后接收帧、错误码、邮箱状态和任务栈余量等变量。
 * 2. 可选保存原始帧；反馈解析与接收计数由接收任务直接完成，
 *    本文件不负责正常电机功能。
 * 3. 周期读取 CAN1 错误状态寄存器、bus-off 状态及 HAL 错误信息。
 * 4. 初始化或任务创建失败时保存故障码，再进入 Error_Handler。
 * 模块关系：can_transport.c 和 can_tasks.c 更新诊断变量；
 * 接收任务只在 APP_ENABLE_DIAGNOSTICS=1 时调用额外诊断接口。
 * 阅读重点：rx_invalid_count 表示未通过目标电机反馈筛选，
 * 不一定是总线错误，也可能是同一总线上其他设备的合法报文。
 */
#include "can_debug.h"
#include "can.h"
#include "main.h"

/* volatile 保留跨任务和中断的读写；多个字段需要一致时，
 * 应用模块提供的临界区快照接口读取，不能只依靠 volatile。 */
volatile uint32_t can_ready = 0; // CAN1 启动完成标志，控制任务据此开始工作。
volatile uint32_t can_fault_code = 0; // 初始化或 RTOS 失败原因，正常为零。
volatile uint32_t can1_tx_queued_count = 0; // HAL 成功提交邮箱的帧数。
volatile uint32_t can1_tx_count = 0; // CAN 硬件成功完成发送的帧数。
volatile uint32_t can1_tx_busy_count = 0; // 邮箱挂起或无空闲的次数。
volatile uint32_t can1_tx_abort_count = 0; // 邮箱取消完成回调次数。
volatile uint32_t can1_tx_submit_error_count = 0; // 提交或取消请求失败次数。
volatile uint32_t can1_rx_count = 0; // 接收任务取出的 CAN1 标准数据帧数。
volatile uint32_t can1_rx_valid_count = 0; // 成功解析为目标电机反馈的帧数。
volatile uint32_t can1_rx_invalid_count = 0; // 未通过目标电机筛选的帧数，可能是其他设备合法帧。
#if APP_ENABLE_DIAGNOSTICS
volatile uint32_t can1_last_rx_id = 0; // 可选诊断：最近原始帧的 ID。
volatile uint8_t can1_last_rx_dlc = 0; // 可选诊断：最近原始帧的长度。
volatile uint8_t can1_last_rx_data[8] = {0}; // 可选诊断：最近原始帧的八个字节。
#endif
volatile uint32_t can_rx_read_error_count = 0; // 取 FIFO 失败或长度异常次数。
volatile uint32_t can_rx_queue_full_count = 0; // CAN 软件接收队列满造成的丢帧数。
volatile uint32_t can_rx_ignored_count = 0; // 因扩展/远程帧类型被忽略的帧数。
volatile uint32_t can1_error_callback_count = 0; // CAN 错误回调次数。
volatile uint32_t can1_last_hal_error = 0; // 最近 HAL CAN 错误信息。
#if APP_ENABLE_DIAGNOSTICS
volatile uint32_t can1_error_status = 0; // 可选诊断：硬件 ESR 快照。
volatile uint32_t can1_bus_off = 0; // 可选诊断：硬件 bus-off 状态。
volatile uint32_t can1_free_mailboxes = 3; // 可选诊断：空闲邮箱数量，初始三个。
volatile uint32_t can_rx_stack_free_words = 0; // 可选诊断：接收任务历史最小栈余量，单位 word。
volatile uint32_t motor_control_stack_free_words = 0; // 可选诊断：控制任务历史最小栈余量。
volatile uint32_t motor_console_stack_free_words = 0; // 可选诊断：串口任务历史最小栈余量。
#endif

void CanFail(uint32_t code)
{
    can_fault_code = code; // 初始化或 RTOS 故障保存非零原因，供问题定位。
    Error_Handler(); // 进入统一失败处理，不继续假定应用可正常运行。
}

#if APP_ENABLE_DIAGNOSTICS
void CanDebug_SampleHardware(void)
{
    /* ESR 包含错误计数和 bus-off 状态；HAL 错误码是另一种诊断信息。
     * 由控制任务每 100 ms 采样，CAN2 不参与本工程。 */
    can1_error_status = hcan1.Instance->ESR; // 可选诊断读取硬件错误状态寄存器。
    can1_bus_off = (can1_error_status & CAN_ESR_BOFF) != 0U; // 取 bus-off 位，转换为 0/1。
    can1_last_hal_error = HAL_CAN_GetError(&hcan1); // 可选诊断记录 HAL 的错误标志。
    can1_free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1); // 可选诊断记录三个邮箱中还有多少空闲。
}

void CanDebug_RecordRx(const CanRxMessage *message)
{
    if (message == NULL || message->source != 1U) // 原始记录只接受存在的 CAN1 消息。
    {
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }

    can1_last_rx_id = message->id; // 可选保存最后一帧实际 ID，不负责反馈筛选。
    can1_last_rx_dlc = message->dlc; // 可选保存最后一帧的有效长度。
    for (uint32_t i = 0U; i < 8U; i++) // 复制全部原始字节，仅用于进一步定位问题。
    {
        can1_last_rx_data[i] = message->data[i]; // 不在诊断模块中解包、更新时间戳或重复累计帧数。
    }
    /* 无效反馈也可能来自其他设备的合法 CAN 帧，不等于总线错误。
     * 有效/无效计数已在接收任务中完成，诊断记录不会重复计数。 */
}
#endif /* APP_ENABLE_DIAGNOSTICS：默认不编译额外原始帧和硬件诊断。 */
