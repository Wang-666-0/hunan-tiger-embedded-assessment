/*
 * 文件功能：通过 USART1 在线设置角度/速度双环 PID，输出 FireWater 曲线。
 * 1. 串口中断将接收字节、时间戳和故障代次放入队列；任务按行组装命令。
 * 2. 解析 A角度、S转速、AP/AI/AD外环、P/I/D内环、V限速、Z、STOP、K、T0/T1
 *    与数值范围后调用 motor_control.c；每条命令以 CR/LF 结束。
 * 3. 处理丢字节、串口错误、超长或过期输入，丢弃残缺命令并请求停止。
 * 4. 默认 T1 输出21通道，便于双环滑块调参；T0仅输出目标和实际角度。
 *    转速显示两位小数，PID 增益显示四位小数，让 D0.001 等小增益仍可辨认。
 *    不在数据流混入文字应答；命令结果放在 T1 的最后一个数值通道。
 * 模块关系：can_tasks.c 调用轮询和遥测接口；从 motor_control.c 一次读取
 * 目标和实际转速的同一份快照，所有串口打印在控制台任务中完成。
 * 阅读重点：中断只收字节，完整命令在任务中解析；接收故障代次用于
 * 防止丢字节前后的残片被拼接成另一条合法命令。
 */
#include "motor_console.h"
#include "app_config.h"
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

volatile MotorConsole_Debug motor_console_debug = {
    .telemetry_mode = MOTOR_TELEMETRY_DEFAULT_MODE
}; // 默认 T1；参数与命令结果直接反馈到 VOFA，便于确认滑块绑定是否成功。
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
        /* 一旦丢字节，整条命令失效，不能把 S100 的残片当成 S10。 */
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

/* 接受可选正负号和普通十进制小数，至少含一个数字，例如 -30、0.5、.5。
 * 不使用 atof：它会接受部分字符串，难以区分 P5abc 和合法的 P5。
 * 将语法检查与数值范围检查分开；失败时完全不写 result。 */
static MotorCommandResult MotorConsole_ParseNumber(const char *text, float *result)
{
    uint32_t integer_part = 0U; // 整数部分用整数累计，乘十前先判断上界。
    float fractional_part = 0.0f; // 小数部分从十分位开始累计。
    float decimal_weight = 0.1f; // 每读一个小数数字，位权缩小十倍。
    float magnitude; // 解析完成后的绝对值，恢复正负号前先检查大小。
    uint8_t negative = 0U; // 只记录符号，不让累计过程混入负数。
    uint8_t dot_seen = 0U; // 只允许出现一个小数点。
    uint8_t digit_seen = 0U; // 单独的 +、- 或 . 都不算数字。
    uint8_t too_large = 0U; // 超上界后继续检查字符，避免跳过尾部语法错误。

    if (text == NULL || result == NULL) // 空指针必须在读取首字符之前拒绝。
    {
        return MOTOR_COMMAND_SYNTAX;
    }
    if (*text == '+' || *text == '-') // 可选符号只能位于数字开头。
    {
        negative = (*text == '-') ? 1U : 0U;
        text++;
    }
    while (*text != '\0') // 整段字符串都要检查，不能只接受开头的合法部分。
    {
        if (*text == '.' && dot_seen == 0U) // 第一个小数点将解析转到小数部分。
        {
            dot_seen = 1U;
        }
        else if (*text >= '0' && *text <= '9')
        {
            uint32_t digit = (uint32_t)(*text - '0');
            digit_seen = 1U;
            if (dot_seen == 0U)
            {
                // 先比较上界再乘十；任意长输入也不会让整数累计溢出。
                if (too_large == 0U)
                {
                    if (integer_part > (1000000U - digit) / 10U)
                    {
                        too_large = 1U;
                    }
                    else
                    {
                        integer_part = integer_part * 10U + digit;
                    }
                }
            }
            else
            {
                fractional_part += (float)digit * decimal_weight;
                decimal_weight *= 0.1f;
                if (integer_part == 1000000U && digit != 0U)
                {
                    too_large = 1U; // 1000000.01 也超过解析器上限，不能被 float 舍入掩盖。
                }
            }
        }
        else // 拒绝空格、指数、NaN、第二个小数点或任何尾部杂字符。
        {
            return MOTOR_COMMAND_SYNTAX;
        }
        text++;
    }
    if (digit_seen == 0U)
    {
        return MOTOR_COMMAND_SYNTAX;
    }
    if (too_large != 0U)
    {
        return MOTOR_COMMAND_RANGE;
    }
    magnitude = (float)integer_part + fractional_part;
    *result = (negative != 0U) ? -magnitude : magnitude;
    return MOTOR_COMMAND_OK;
}

static void MotorConsole_SetResult(MotorCommandResult result)//设置命令处理结果
{
    motor_console_debug.last_result = (uint32_t)result; // 最近命令结果位于新 T1 格式的 I14。
    if (result != MOTOR_COMMAND_OK) // 失败结果计入拒绝次数，便于排查输入问题。
    {
        motor_console_debug.rejected_count++; // 累计未成功接受的命令或组行错误。
    }
}

void MotorConsole_ProcessLine(const char *line)// 解析完整的一行，错误命令不会覆盖已有给定。
{
    float value; // 角度、转速及增益都使用普通十进制小数。
    float limit; // 不同参数具有不同的教学上界。
    uint8_t outer_gain = 0U; // AP/AI/AD是外环参数，与A角度命令区分。
    char term;
    MotorCommandResult parse_result;

    motor_console_debug.command_count++;
    if (line == NULL || *line == '\0') // 空指针与空字符串都作为语法错误处理。
    {
        MotorConsole_SetResult(MOTOR_COMMAND_SYNTAX);
        return;
    }
    if (strcmp(line, "STOP") == 0)
    {
        MotorControl_Stop(MOTOR_STOP_USER); // 撤销闭环请求，控制任务随后发送零给定。
        MotorConsole_SetResult(MOTOR_COMMAND_OK);
        return;
    }
    if (strcmp(line, "Z") == 0)
    {
        // 停机后且最新速度足够低才能改角度参考，不能运动中悄悄清圈数。
        MotorConsole_SetResult(MotorControl_ZeroAngle() ?
            MOTOR_COMMAND_OK : MOTOR_COMMAND_NOT_READY);
        return;
    }
    if (strcmp(line, "K") == 0)
    {
        // 兼容以前的 K 命令，只检查是否运行；本版不需要周期发送 K。
        MotorConsole_SetResult(MotorControl_KeepAlive() ?
            MOTOR_COMMAND_OK : MOTOR_COMMAND_NOT_READY);
        return;
    }
    if (strcmp(line, "T0") == 0 || strcmp(line, "T1") == 0)
    {
        motor_console_debug.telemetry_mode = (line[1] == '1') ? 1U : 0U;
        MotorConsole_SetResult(MOTOR_COMMAND_OK); // 切换帧格式只影响输出，不改变电机请求。
        return;
    }
    if (line[0] == 'A' && (line[1] == 'P' || line[1] == 'I' || line[1] == 'D'))
    {
        outer_gain = 1U;
    }
    if (line[0] != 'A' && line[0] != 'V' && line[0] != 'S' &&
        line[0] != 'P' && line[0] != 'I' && line[0] != 'D')
    {
        MotorConsole_SetResult(MOTOR_COMMAND_SYNTAX); // 大写前缀以外的命令都未定义。
        return;
    }
    parse_result = MotorConsole_ParseNumber(&line[outer_gain ? 2U : 1U], &value);
    if (parse_result != MOTOR_COMMAND_OK)
    {
        MotorConsole_SetResult(parse_result);
        return;
    }
    if (line[0] == 'S')
    {
        if (value < -MOTOR_SPEED_LIMIT_RPM || value > MOTOR_SPEED_LIMIT_RPM)
        {
            MotorConsole_SetResult(MOTOR_COMMAND_RANGE); // 超限直接拒绝，不能悄悄把 S1000 变成 S100。
            return;
        }
        // S0 也进入闭环：目标为零 rpm；STOP 才关闭闭环使能。
        MotorConsole_SetResult(MotorControl_SetSpeed(value) ?
            MOTOR_COMMAND_OK : MOTOR_COMMAND_NOT_READY);
        return;
    }

    if (line[0] == 'A' && outer_gain == 0U)
    {
        if (value < -ANGLE_TARGET_LIMIT_DEG || value > ANGLE_TARGET_LIMIT_DEG)
        {
            MotorConsole_SetResult(MOTOR_COMMAND_RANGE);
            return;
        }
        MotorConsole_SetResult(MotorControl_SetAngle(value) ?
            MOTOR_COMMAND_OK : MOTOR_COMMAND_NOT_READY);
        return;
    }
    if (line[0] == 'V')
    {
        if (value < 1.0f || value > MOTOR_SPEED_LIMIT_RPM)
        {
            MotorConsole_SetResult(MOTOR_COMMAND_RANGE);
            return;
        }
        MotorConsole_SetResult(MotorControl_SetSpeedLimit(value) ?
            MOTOR_COMMAND_OK : MOTOR_COMMAND_RANGE);
        return;
    }

    // 参数的上界统一来自 app_config.h，串口检查与控制模块检查使用同一份配置。
    term = outer_gain ? line[1] : line[0];
    if (term == 'P')
    {
        limit = outer_gain ? ANGLE_PID_KP_MAX : SPEED_PID_KP_MAX;
    }
    else if (term == 'I')
    {
        limit = outer_gain ? ANGLE_PID_KI_MAX : SPEED_PID_KI_MAX;
    }
    else
    {
        limit = outer_gain ? ANGLE_PID_KD_MAX : SPEED_PID_KD_MAX;
    }
    if (value < 0.0f || value > limit) // 本项只接受非负增益，负增益会改变反馈方向。
    {
        MotorConsole_SetResult(MOTOR_COMMAND_RANGE);
        return;
    }
    // 在线改参数不启动闭环，不改变目标，也不清空正在运行的 PID 历史。
    MotorConsole_SetResult((outer_gain ? MotorControl_SetAngleGain(term, value) :
        MotorControl_SetGain(term, value)) ?
        MOTOR_COMMAND_OK : MOTOR_COMMAND_RANGE);
}

static void MotorConsole_SyncFault(void)//同步接收故障代次，避免丢字节前后的残片被拼接成另一条合法命令
{
    uint32_t epoch = motor_console_debug.rx_fault_epoch; // 取目前中断侧已记录的接收故障代次。
    if (epoch != observed_fault_epoch) // 出现新的丢字节、UART 错误或续接收错误。
    {
        observed_fault_epoch = epoch; // 标记任务已经处理这个故障阶段。
        command_length = 0U; // 清除当前拼接长度，不让残片成为下一条命令。
        discard_until_eol = 1U; // 等待新的换行边界，期间忽略命令字符。
        MotorControl_Stop(MOTOR_STOP_UART_RX); // 输入可靠性失效，撤销当前闭环请求。
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
        MotorControl_Stop(MOTOR_STOP_UART_RX); // 输入可靠性失效，撤销当前闭环请求。
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
            MotorConsole_ProcessLine(command_line); // 完整行就绪后才解析角度/速度/增益/STOP命令。
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

/* 浮点值先四舍五入为百分之一单位，再用整数格式化。
 * 将符号单独输出，避免 -0.50 因整数部分为 0 而丢掉负号。
 * 不使用 %f；函数也会拒绝 NaN、无穷大或无法安全转成整数的值。 */
static uint8_t MotorConsole_FormatHundredths(float value, char *text, size_t capacity)
{
    int32_t rounded; // 有符号百分之一单位数，例如 -0.50 对应 -50。
    uint32_t magnitude; // 分离负号后，整数/小数部分都用非负数格式化。
    const char *sign;
    int length;

    if (value != value || value < -1000000.0f || value > 1000000.0f)
    {
        return 0U;
    }
    rounded = (int32_t)(value * 100.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
    sign = (rounded < 0) ? "-" : "";
    magnitude = (rounded < 0) ? (uint32_t)(-rounded) : (uint32_t)rounded;
    length = snprintf(text, capacity, "%s%lu.%02lu", sign,
        (unsigned long)(magnitude / 100U), // 百分之一单位除一百得到整数部分。
        (unsigned long)(magnitude % 100U)); // 余数始终补足两位，0.05 不能写成 0.5。
    return (length > 0 && (size_t)length < capacity) ? 1U : 0U;
}

/* 增益使用四位小数：例如 Kd=0.001 输出 0.0010，不会被两位格式显示为零。
 * 参数均为非负数且不超过 1000，放大一万倍后的整数仍在 uint32_t 范围内。 */
static uint8_t MotorConsole_FormatGain(float value, char *text, size_t capacity)
{
    uint32_t ten_thousandths; // 万分之一单位整数，先四舍五入再拆整数与小数部分。
    int length;

    if (value != value || value < 0.0f || value > 1000.0f)
    {
        return 0U; // 拒绝 NaN、无穷大或无效参数，不能将异常浮点值转成整数。
    }
    ten_thousandths = (uint32_t)(value * 10000.0f + 0.5f);
    length = snprintf(text, capacity, "%lu.%04lu",
        (unsigned long)(ten_thousandths / 10000U),
        (unsigned long)(ten_thousandths % 10000U)); // 小数固定四位，保留小增益的含义。
    return (length > 0 && (size_t)length < capacity) ? 1U : 0U;
}

void MotorConsole_SendTelemetry(void)// 发送一整帧 FireWater 数值，以 CRLF 结束。
{
    MotorControl_State state; // 角度、速度及两个PID的结果来自同一份控制快照。
    char numbers[12][24]; // 角度/rpm两位，六个增益四位，保留小D值。
    char line[256]; // 21通道帧；同步发送返回前保持有效。
    uint32_t telemetry_mode = motor_console_debug.telemetry_mode;
    int length;

    MotorControl_GetState(&state);
    if (MotorConsole_FormatHundredths(state.target_angle_deg, numbers[0], sizeof(numbers[0])) == 0U ||
        MotorConsole_FormatHundredths(state.actual_angle_deg, numbers[1], sizeof(numbers[1])) == 0U)
    {
        motor_console_debug.format_error_count++;
        return;
    }
    if (telemetry_mode == 0U)
    {
        // T0严格两条角度曲线；切换帧格式后需要清空VOFA旧数据。
        length = snprintf(line, sizeof(line), "angle:%s,%s\r\n", numbers[0], numbers[1]);
    }
    else
    {
        if (MotorConsole_FormatHundredths(state.target_rpm, numbers[2], sizeof(numbers[2])) == 0U ||
            MotorConsole_FormatHundredths(state.actual_rpm, numbers[3], sizeof(numbers[3])) == 0U ||
            MotorConsole_FormatGain(state.kp, numbers[4], sizeof(numbers[4])) == 0U ||
            MotorConsole_FormatGain(state.ki, numbers[5], sizeof(numbers[5])) == 0U ||
            MotorConsole_FormatGain(state.kd, numbers[6], sizeof(numbers[6])) == 0U ||
            MotorConsole_FormatGain(state.angle_kp, numbers[7], sizeof(numbers[7])) == 0U ||
            MotorConsole_FormatGain(state.angle_ki, numbers[8], sizeof(numbers[8])) == 0U ||
            MotorConsole_FormatGain(state.angle_kd, numbers[9], sizeof(numbers[9])) == 0U ||
            MotorConsole_FormatHundredths(state.speed_limit_rpm, numbers[10], sizeof(numbers[10])) == 0U ||
            MotorConsole_FormatHundredths(state.overshoot_deg, numbers[11], sizeof(numbers[11])) == 0U)
        {
            motor_console_debug.format_error_count++;
            return;
        }
        // T1固定21通道，角度与rpm分图显示；末尾是实测角度误差带/超调指标。
        length = snprintf(line, sizeof(line),
            "angle:%s,%s,%s,%s,%d,%s,%s,%s,%s,%s,%s,%lu,%lu,%lu,%lu,%lu,%lu,%s,%lu,%lu,%s\r\n",
            numbers[0], numbers[1], numbers[2], numbers[3], (int)state.output_raw,
            numbers[4], numbers[5], numbers[6], numbers[7], numbers[8], numbers[9],
            (unsigned long)state.enabled,
            (unsigned long)state.online,
            (unsigned long)state.stop_reason,
            (unsigned long)motor_console_debug.last_result,
            (unsigned long)state.control_mode,
            (unsigned long)state.angle_valid,
            numbers[10],
            (unsigned long)state.settled,
            (unsigned long)state.settling_time_ms,
            numbers[11]);
    }

    // snprintf 返回想要写入的长度；截断时不能用该长度发送，否则可能越界读缓冲。
    if (length <= 0 || (size_t)length >= sizeof(line))
    {
        motor_console_debug.format_error_count++;
        return;
    }
    // 50Hz的21通道文本由单一控制台发送；更高优先级闭环任务仍可抢占。
    if (HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)length, UART_TRANSMIT_TIMEOUT_MS) == HAL_OK)
    {
        motor_console_debug.tx_frame_count++;
    }
    else
    {
        motor_console_debug.tx_error_count++;
    }
}
