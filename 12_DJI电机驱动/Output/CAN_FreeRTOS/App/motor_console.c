/*
 * 文件功能：通过 USART1 接收电机命令，并向电脑发送 FireWater 遥测。
 * 1. 串口中断将接收字节、时间戳和故障代次放入队列；任务按行组装命令。
 * 2. 解析 V 数值（电压模式）、C 数值（电流模式）、STOP 和 K，
 *    检查语法、模式和限幅后调用 motor_control.c；命令以 CR/LF 结束。
 * 3. 处理丢字节、串口错误、超长或过期输入，丢弃残缺命令并请求停止。
 * 4. 按 FireWater 格式输出角度、转速、电流原始值、温度、在线状态、
 *    输出给定、使能状态、停止原因和命令结果，共 9 个数值通道。
 * 模块关系：can_tasks.c 调用轮询和遥测接口；从 gm6020.c 读取反馈，
 * 从 motor_control.c 读取控制状态，所有串口打印在控制台任务中完成。
 * 阅读重点：中断只收字节，完整命令在任务中解析；接收故障代次用于
 * 防止丢字节前后的残片被拼接成另一条合法命令。
 */
#include "motor_console.h"
#include "motor_control.h"
#include "can_debug.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
    uint8_t byte; // 本次接收的一个原始串口字节。
    uint32_t received_ms; // 中断收到字节的时刻，用于识别陈旧输入。
    uint32_t epoch; // 接收故障代次；故障前后字节不能拼成一条命令。
} UartRxByte;// 串口接收字节及时间戳和故障代次

volatile MotorConsole_Debug motor_console_debug = {0}; // 串口处理结果及错误统计；故障代次参与输入恢复逻辑。
static QueueHandle_t uart_rx_queue; // 中断与任务之间传递字节的软件队列。
static uint8_t uart_rx_byte; // HAL 单字节中断接收使用的持久缓冲，不能用局部变量替代。
static volatile uint8_t rx_needs_rearm; // 错误回调置位，串口任务再重新启用接收。
static char command_line[UART_COMMAND_BUFFER_SIZE]; // 完整命令行缓冲，预留字符串结束符位置。
static uint32_t command_length; // 当前已经拼入缓冲的字符个数。
static uint32_t observed_fault_epoch; // 任务目前已经处理过的接收故障代次。
static uint32_t last_byte_ms; // 上一字节的接收时刻，用于片段间隔检查。
static uint8_t discard_until_eol; // 残缺或错误行丢弃到下个换行后，才重新接受命令。

void MotorConsole_CreateQueue(void)//创建串口接收队列
{
    // 每个队列项复制字节、时间戳和代次，不只复制字符。
    uart_rx_queue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(UartRxByte));
    if (uart_rx_queue == NULL) // 动态分配失败则无法建立可靠接收路径。
    {
        CanFail(CAN_FAULT_UART_QUEUE); // 保存故障码并进入统一初始化失败处理。
    }
    vQueueAddToRegistry(uart_rx_queue, "UART_COMMANDS"); // 给 RTOS 队列注册名称，不改变命令格式或收发行为。
}

void MotorConsole_Start(void)//启动串口接收
{
    // 首次或续接收一个字节，失败时不能假定输入仍正常。
    if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
    {
        CanFail(CAN_FAULT_UART_START); // 初次开启接收失败，记录初始化故障。
    }
}

/* USART1 中断只接收一个字节并入队。命令解析和所有打印均在任务中进行。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口接收中断回调
{
    BaseType_t task_woken = pdFALSE; // 记录入队是否唤醒更高优先级任务，稍后通知调度器。
    UartRxByte received; // 本次字节及元信息的局部副本。
    if (huart != &huart1) // 忽略其他 UART 的回调，只管理 USART1。
    {
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    received.byte = uart_rx_byte; // 在续接收覆盖缓冲之前保存当前字节。
    received.received_ms = HAL_GetTick(); // 时间戳来自接收回调，不是任务从队列取出时刻。
    received.epoch = motor_console_debug.rx_fault_epoch; // 标记字节属于哪个连续且未丢失的接收阶段。
    motor_console_debug.rx_byte_count++; // 累计中断接收到的字节数。
    // 中断不等待空间；满队列表示当前字节无法可靠保存。
    if (xQueueSendFromISR(uart_rx_queue, &received, &task_woken) != pdPASS)
    {
        /* 一旦丢字节，整条命令失效，不能把 V1000 的残片当成 V100。 */
        motor_console_debug.rx_drop_count++; // 记录队列满导致的丢字节。
        motor_console_debug.rx_fault_epoch++; // 增加故障代次，使之前排队的命令片段全部失效。
    }
    // 首次或续接收一个字节，失败时不能假定输入仍正常。
    if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
    {
        motor_console_debug.rx_rearm_fail_count++; // 记录续接收失败，稍后由任务重启 RX。
        motor_console_debug.rx_fault_epoch++; // 增加故障代次，使之前排队的命令片段全部失效。
        rx_needs_rearm = 1U; // 通知任务恢复接收，避免在回调里反复重启。
    }
    portYIELD_FROM_ISR(task_woken); // 若唤醒了更高优先级任务，中断退出时请求切换。
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)//串口错误中断回调
{
    if (huart != &huart1) // 忽略其他 UART 的回调，只管理 USART1。
    {
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    motor_console_debug.last_uart_error = HAL_UART_GetError(huart); // 保存 HAL 接收错误信息。
    motor_console_debug.rx_error_count++; // 累计 UART 错误回调次数。
    motor_console_debug.rx_fault_epoch++; // 增加故障代次，使之前排队的命令片段全部失效。
    /* 无 DMA，AbortReceive 只结束接收并清除接收错误，不中断正在进行的 TX。 */
    (void)HAL_UART_AbortReceive(huart); // 结束异常 RX；本工程不使用 RX DMA，不取消正在进行的 TX。
    rx_needs_rearm = 1U; /* 交给任务重新开启接收，避免在错误回调中反复嵌套。 */
}

static uint8_t MotorConsole_ParseNumber(const char *text, int16_t *result)//解析字符串为 16 位有符号整数
{
    int32_t value = 0; // 按十进制逐位累计数字，用 32 位量避免中间值过窄。
    int32_t sign = 1; // 默认正号，遇到负号后改为 -1。
    if (*text == '-' || *text == '+')// 处理正负号
    {
        if (*text == '-') // 负号只改变符号，后续仍按非负大小积累数字。
        {
            sign = -1; /* 记录符号，先积累数值大小，再恢复正负。 */
        }
        text++; // 移动到下一字符；解析前跳过符号，循环中跳过已处理数字。
    }
    if (*text == '\0') // 符号后没有数字属于语法错误。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    while (*text != '\0') // 必须检查整段文本，尾部杂字符也不能接受。
    {
        if (*text < '0' || *text > '9') // 只接受十进制数字，不接受空格、小数点或其他字符。
        {
            return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
        }
        value = value * 10 + (*text - '0'); // 原数乘十，再加新数字；例如 50 再读 0 得到 500。
        /* 每位后立即限制，避免任意长字符串导致整数溢出。 */
        if (value > 32768L) // 每处理一位就限制绝对值，下一位计算不会无限溢出。
        {
            return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
        }
        text++; // 移动到下一字符；解析前跳过符号，循环中跳过已处理数字。
    }
    value *= sign; // 数字大小检查后恢复原来的正负号。
    if (value < -32768L || value > 32767L) // 最后按有符号 16 位的非对称范围校验。
    {
        return 0U; // 拒绝当前输入或判定条件不满足；不继续处理。
    }
    *result = (int16_t)value; // 只有完全合法才写回输出，不改写失败结果。
    return 1U; // 通知调用方本次处理成功或输入已被接受。
}

static void MotorConsole_SetResult(MotorCommandResult result)//设置命令处理结果
{
    motor_console_debug.last_result = (uint32_t)result; // 最近命令状态会作为 FireWater 的最后一个通道。
    if (result != MOTOR_COMMAND_OK) // 失败结果计入拒绝次数，便于排查输入问题。
    {
        motor_console_debug.rejected_count++; // 累计未成功接受的命令或组行错误。
    }
}

void MotorConsole_ProcessLine(const char *line)//处理一行命令
{
    int16_t raw; // V/C 命令携带的有符号原始给定值。
    int32_t limit; // 当前协议的教学限幅。
    uint8_t mode; // 根据 V/C 前缀确定命令要求的协议模式。
    motor_console_debug.command_count++; // 统计交给行解析函数的命令数。
    if (line == NULL || *line == '\0') // 拒绝空指针或空字符串，后续才能访问首字符。
    {
        MotorConsole_SetResult(MOTOR_COMMAND_SYNTAX); // 记录格式不合法，给定请求不被新命令覆盖。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    if (strcmp(line, "STOP") == 0)// 停止命令
    {
        MotorControl_Stop(MOTOR_STOP_USER); // STOP 只撤销请求，CAN 操作仍由控制任务完成。
        MotorConsole_SetResult(MOTOR_COMMAND_OK); // 通知电脑这条命令已被接受。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    if (strcmp(line, "K") == 0)// 续期命令有效期
    {
        MotorConsole_SetResult(MotorControl_KeepAlive() ? // K 的接受结果取决于原请求是否仍启用且未超时。
            MOTOR_COMMAND_OK : MOTOR_COMMAND_NOT_READY); // 成功为结果 1，不能启用或续期为结果 5。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    if ((line[0] != 'V' && line[0] != 'C') || // 其他前缀未定义；只有 V/C 才尝试解析数值。
        MotorConsole_ParseNumber(&line[1], &raw) == 0U)// 解析 V 数值（电压模式）、C 数值（电流模式）失败
    {
        MotorConsole_SetResult(MOTOR_COMMAND_SYNTAX); // 记录格式不合法，给定请求不被新命令覆盖。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    // V 选择电压协议，C 选择电流协议，不切换电机内部设置。
    mode = (line[0] == 'V') ? GM6020_MODE_VOLTAGE : GM6020_MODE_CURRENT;
    if (mode != GM6020_CONTROL_MODE)// 当前控制模式与命令模式不匹配
    {
        MotorConsole_SetResult(MOTOR_COMMAND_MODE); // 编译模式与命令不匹配，保持原请求。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    limit = (mode == GM6020_MODE_VOLTAGE) ? // 按协议选择教学限幅，而不是使用手册最大值。
        MOTOR_VOLTAGE_DEMO_LIMIT : MOTOR_CURRENT_DEMO_LIMIT; // 当前默认 V 范围 ±2000，C 版本范围 ±1000。
    /* 超出教学限值直接拒绝，不悄悄截断为最大给定。 */
    if ((int32_t)raw < -limit || (int32_t)raw > limit) // 明确拒绝超限，不静默夹到最大值。
    {
        MotorConsole_SetResult(MOTOR_COMMAND_RANGE); // 超限返回结果 4，原请求仍受自身三秒有效期约束。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    MotorConsole_SetResult(MotorControl_SetOutput(raw) ? // 交给控制模块检查反馈和温度后保存请求。
        MOTOR_COMMAND_OK : MOTOR_COMMAND_NOT_READY); // 成功为结果 1，不能启用或续期为结果 5。
}

static void MotorConsole_SyncFault(void)//同步接收故障代次，避免丢字节前后的残片被拼接成另一条合法命令
{
    uint32_t epoch = motor_console_debug.rx_fault_epoch; // 取目前中断侧已记录的接收故障代次。
    if (epoch != observed_fault_epoch) // 出现新的丢字节、UART 错误或续接收错误。
    {
        observed_fault_epoch = epoch; // 标记任务已经处理这个故障阶段。
        command_length = 0U; // 清除当前拼接长度，不让残片成为下一条命令。
        discard_until_eol = 1U; // 等待新的换行边界，期间忽略命令字符。
        MotorControl_Stop(MOTOR_STOP_UART_RX); // 输入可靠性失效，撤销当前非零给定。
        MotorConsole_SetResult(MOTOR_COMMAND_RX_FAULT); // 向遥测报告接收故障，结果为 7。
    }
}

/* 接收重启与命令组行分开：这里仅处理 HAL 接收状态，不解释命令。 */
static void MotorConsole_RearmReceive(void)
{
    if (rx_needs_rearm != 0U) // 只有中断已报告异常时才重建 HAL 单字节接收。
    {
        /* 极短临界区，避免任务和 USART ISR 同时修改 HAL 的接收状态。 */
        taskENTER_CRITICAL(); // 进入短临界区，防止任务切换或相关中断打断共享结构体操作。
        (void)HAL_UART_AbortReceive(&huart1); // 先结束旧接收状态，再申请新的单字节接收。
        // 重启成功才清除恢复标记，否则下轮继续尝试。
        if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) == HAL_OK)
        {
            rx_needs_rearm = 0U; // 接收已恢复，不再重复重启。
        }
        taskEXIT_CRITICAL(); // 完成一致更新或复制后退出临界区，恢复正常调度。
    }
}

/* 只处理一个带时间戳的字节；完整行才交给 ProcessLine。
 * received 指向队列中取出的副本，返回后不再保存这个指针。 */
static void MotorConsole_HandleByte(const UartRxByte *received)
{
    MotorConsole_SyncFault(); // 先处理故障代次变化，旧字节不能重新启用电机。
    /* 丢字节前排队的旧命令也不能重新启用电机。 */
    if (received->epoch != observed_fault_epoch) // 丢字节之前的队列项已经失效，即使是换行也忽略。
    {
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    // 队列积压达到一秒，拒绝过期字节。
    if ((uint32_t)(HAL_GetTick() - received->received_ms) >= UART_INPUT_TIMEOUT_MS)
    {
        command_length = 0U; // 清除当前拼接长度，不让残片成为下一条命令。
        discard_until_eol = 1U; // 等待新的换行边界，期间忽略命令字符。
        MotorControl_Stop(MOTOR_STOP_UART_RX); // 输入可靠性失效，撤销当前非零给定。
        MotorConsole_SetResult(MOTOR_COMMAND_STALE); // 记录陈旧输入结果 6，不把旧片段当成新命令。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    if (command_length != 0U && // 正在拼接时，检查两片段的接收时间间隔。
        // 隔了一秒的两段字符不允许拼成同一条命令。
        (uint32_t)(received->received_ms - last_byte_ms) >= UART_INPUT_TIMEOUT_MS)
    {
        command_length = 0U; // 清除当前拼接长度，不让残片成为下一条命令。
        discard_until_eol = 1U; // 等待新的换行边界，期间忽略命令字符。
        MotorConsole_SetResult(MOTOR_COMMAND_STALE); // 记录陈旧输入结果 6，不把旧片段当成新命令。
    }
    last_byte_ms = received->received_ms; // 记住字节的真实接收时刻，不用任务处理时间。
    if (received->byte == '\r' || received->byte == '\n') // CR 或 LF 都作为一条命令的结束边界。
    {
        if (discard_until_eol != 0U) // 错误行遇到换行后恢复接收下一条命令。
        {
            discard_until_eol = 0U; // 恢复对下一条完整命令的组装。
        }
        else if (command_length != 0U) // 只有非空完整行才调用命令解析，忽略空行。
        {
            command_line[command_length] = '\0'; // 补字符串结束符，让 strcmp 等函数只访问有效命令。
            MotorConsole_ProcessLine(command_line); // 完整行就绪后再判断 STOP/K/V/C，避免解析半条输入。
        }
        command_length = 0U; /* CRLF 中的第二个空行自动忽略。 */
    }
    else if (discard_until_eol == 0U) // 只在当前行有效时积累普通字符。
    {
        if (received->byte < 0x20U || received->byte > 0x7EU || // 只接受可打印 ASCII，避免控制字符混进命令。
            command_length >= UART_COMMAND_BUFFER_SIZE - 1U) // 最后一位留给结束符，阻止缓冲区越界。
        {
            command_length = 0U; // 清除当前拼接长度，不让残片成为下一条命令。
            discard_until_eol = 1U; // 等待新的换行边界，期间忽略命令字符。
            MotorConsole_SetResult(MOTOR_COMMAND_SYNTAX); // 记录格式不合法，给定请求不被新命令覆盖。
        }
        else
        {
            command_line[command_length++] = (char)received->byte; // 先写本次字符再增加长度，下个字符写下一位置。
        }
    }
}

void MotorConsole_Poll(void)//轮询串口接收队列，组装命令行并处理
{
    UartRxByte received; // 本次字节及元信息的局部副本。
    MotorConsole_SyncFault(); // 先处理故障代次变化，旧字节不能重新启用电机。
    MotorConsole_RearmReceive(); // 恢复硬件接收状态，与命令组行分开处理。

    /* 每轮最多处理队列容量个字节，持续的输入噪声也不会无限占用任务。 */
    for (uint32_t i = 0U; i < UART_RX_QUEUE_LENGTH; i++) // 每轮有处理上限，连续输入也不会无限占用此任务。
    {
        if (xQueueReceive(uart_rx_queue, &received, 0U) != pdPASS) // 零等待取字节，队列空就结束本轮。
        {
            break; // 结束本轮循环，避免无数据或读取失败时继续处理。
        }
        MotorConsole_HandleByte(&received); // 逐个验证并拼接字节，Poll 本身只负责调度。
    }
}

void MotorConsole_SendTelemetry(void)//发送遥测数据
{
    GM6020_Feedback feedback; // 本帧遥测使用一份一致的电机反馈快照。
    MotorControl_State state; // 另取控制状态快照，与反馈可能相差一个周期。
    char line[128]; // 完整 FireWater 文本缓冲，同步发送返回前一直有效。
    uint32_t angle_hundredths; // 角度的百分之一度整数，用于显示两位小数。
    int length; // snprintf 返回所需长度，可能大于实际缓冲容量。

    GM6020_GetFeedback(&feedback); // 先读取有效反馈的完整快照。
    MotorControl_GetState(&state); // 再读取请求使能、给定与停止原因。
    /* 定点格式化两位小数，避免 snprintf 的 %f 引入不必要的浮点格式化开销。
     * 角度换算仍是 encoder * 360 / 8192，Watch 中保留 float 角度。 */
    // 先乘 100 换算角度，用整数拆整数部分和小数部分。
    angle_hundredths = (uint32_t)feedback.encoder * 36000U / 8192U;
    /* 所有通道都是数字，固定九通道，以真实的 CRLF 结束，符合 FireWater。 */
    length = snprintf(line, sizeof(line), // 限制可写容量，先把一整帧数据格式化到缓冲区。
        "motor:%lu.%02lu,%d,%d,%u,%lu,%d,%lu,%lu,%lu\r\n", // 固定九个数值通道，以真实 CRLF 结束。
        (unsigned long)(angle_hundredths / 100U), // 通道 0 的整数部分。
        (unsigned long)(angle_hundredths % 100U), // 通道 0 的两位小数部分，与上一项拼成一个数。
        (int)feedback.speed_rpm, // 通道 1：有符号转速，rpm。
        (int)feedback.current_raw, // 通道 2：反馈电流原始值，不直接当作安培。
        (unsigned int)feedback.temperature, // 通道 3：温度，摄氏度。
        (unsigned long)GM6020_IsOnline(&feedback, HAL_GetTick()), // 通道 4：此反馈是否在线，0 或 1。
        (int)state.output_raw, // 通道 5：本轮选择的给定，不是实测转速。
        (unsigned long)state.enabled, // 通道 6：非零请求是否启用。
        (unsigned long)state.stop_reason, // 通道 7：停止原因枚举。
        // 角度、转速、电流原始值、温度、在线状态、输出给定、使能状态、停止原因和命令结果
        (unsigned long)motor_console_debug.last_result);

    /* snprintf 返回希望写入的长度，可能大于缓冲区，不能直接拿来发送。 */
    if (length <= 0 || (size_t)length >= sizeof(line)) // 格式化失败或截断时不能发送返回长度，防止越界读取。
    {
        motor_console_debug.format_error_count++; // 累计格式化失败次数，丢弃这帧遥测。
        return; // 结束当前函数；错误分支不再执行后面的操作。
    }
    /* 仅本任务发送串口数据。50 Hz 的短文本允许阻塞 TX，控制任务优先级更高。
     * line 在 HAL_UART_Transmit 返回前始终有效，不存在异步局部缓冲区失效。 */
    // 单一任务同步发送整行，最长等待由配置宏决定。
    if (HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)length, UART_TRANSMIT_TIMEOUT_MS) == HAL_OK)
    {
        motor_console_debug.tx_frame_count++; // 记录已完成的 UART 遥测发送。
    }
    else
    {
        motor_console_debug.tx_error_count++; // 记录 UART 发送失败，不将失败帧计入成功数。
    }
}
