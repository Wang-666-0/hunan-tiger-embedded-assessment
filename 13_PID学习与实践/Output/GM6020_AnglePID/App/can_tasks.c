/*
 * 文件功能：创建并调度电机应用的三个 FreeRTOS 任务。
 * 1. CAN 接收任务：启动 CAN1，阻塞等待接收队列，处理反馈与诊断记录。
 * 2. 电机控制任务：默认每 2 ms 执行控制步骤，可选每 100 ms 采样诊断。
 * 3. 串口控制台任务：默认每 5 ms 处理输入，每 20 ms 发送双环遥测。
 * 4. 设置各任务优先级和栈大小，处理创建失败；额外诊断默认关闭。
 * 模块关系：本文件负责安排何时执行；具体收发、协议解码、输出管理
 * 和串口命令处理，分别交给 can_transport、gm6020、motor_control、
 * motor_console 等模块；可配置周期和栈大小见 app_config.h。
 * 阅读重点：接收任务优先级高于控制任务，控制任务高于串口任务；
 * vTaskDelayUntil 以固定基准安排周期，减少执行耗时造成的周期累积。
 */
#include "can_tasks.h"
#include "can_transport.h"
#include "can_debug.h"
#include "motor_control.h"
#include "motor_console.h"
#include "gm6020.h"
#include "task.h"

static void CanRxTask(void *argument)//接收CAN报文的任务
{
    CanRxMessage message; // 从队列接收一整条报文的局部副本。
    (void)argument; // 此任务不用启动参数，显式忽略以避免编译警告。
    /* MX_FREERTOS_Init 已创建队列；现在调度器运行，才能开启 FromISR 接收。 */
    CanStart(); // 调度器和队列就绪后才开启 CAN1 接收通知。
    for (;;) // 任务常驻循环；每轮通过队列等待或延时让出 CPU。
    {
        if (CanTransport_Receive(&message, portMAX_DELAY) == pdPASS) // 阻塞等帧，不轮询空队列；取到副本才解析。
        {
            /* 电机解析是正常功能，直接调用，不再隐藏在诊断模块里。 */
            uint8_t feedback_valid = GM6020_ParseFeedback(&message); // 直接完成目标电机反馈解析，不依赖额外诊断。
            can1_rx_count++; /* 每取出一条 CAN1 标准数据帧，记录一次接收。 */
            if (feedback_valid != 0U) // 按解析结果区分有效电机反馈与其他帧。
            {
                can1_rx_valid_count++; /* 只有目标电机的有效反馈才计入。 */
            }
            else
            {
                can1_rx_invalid_count++; /* 其他 ID 的合法帧也可能到这里。 */
            }
#if APP_ENABLE_DIAGNOSTICS
            CanDebug_RecordRx(&message);//记录接收报文
            can_rx_stack_free_words = uxTaskGetStackHighWaterMark(NULL);//记录接收任务栈剩余空间
#endif
        }
    }
}

static void MotorControlTask(void *argument)//电机控制任务
{
    TickType_t last_wake; // 固定周期的上次唤醒基准，交给 DelayUntil 更新。
#if APP_ENABLE_DIAGNOSTICS
    TickType_t last_sample; // 仅启用额外诊断时保存上次采样 tick。
#endif
    (void)argument; // 此任务不用启动参数，显式忽略以避免编译警告。
    while (can_ready == 0U) // 等接收任务完成 CAN1 启动，再进入周期控制。
    {
        vTaskDelay(pdMS_TO_TICKS(1U)); // 等待时让出 CPU，不能空转阻挡启动任务。
    }
    last_wake = xTaskGetTickCount(); // 建立本任务周期调度的初始基准。
#if APP_ENABLE_DIAGNOSTICS
    last_sample = last_wake; // 从任务启动时刻开始计算诊断采样间隔。
#endif
    for (;;) // 任务常驻循环；每轮通过队列等待或延时让出 CPU。
    {
        MotorControl_Step();//执行一次电机控制步骤
#if APP_ENABLE_DIAGNOSTICS
        // 每100ms采样一次硬件状态
        if ((TickType_t)(xTaskGetTickCount() - last_sample) >= pdMS_TO_TICKS(APP_DIAGNOSTIC_PERIOD_MS))
        {
            CanDebug_SampleHardware();//采样硬件状态
            // 可选诊断：单位 word，记录栈历史最小余量。
            motor_control_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
            last_sample = xTaskGetTickCount(); // 更新额外诊断的最近采样时刻。
        }
#endif
        /* 固定基准的周期调度，计算耗时不累加进下轮周期。 */
        // 名义每 2 ms 唤醒，而不是工作结束后再等 2 ms。
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}

static void MotorConsoleTask(void *argument)//电机控制台任务
{
    TickType_t last_wake; // 固定周期的上次唤醒基准，交给 DelayUntil 更新。
    TickType_t last_telemetry; // 上次遥测发送的调度时刻。
    (void)argument; // 此任务不用启动参数，显式忽略以避免编译警告。
    MotorConsole_Start(); // 在队列和调度器就绪后启用 USART1 接收。
    last_wake = xTaskGetTickCount(); // 建立本任务周期调度的初始基准。
    last_telemetry = last_wake; // 以控制台启动时刻作为第一次遥测周期的基准。
    for (;;) // 任务常驻循环；每轮通过队列等待或延时让出 CPU。
    {
        MotorConsole_Poll();//轮询串口输入，处理命令
        if ((TickType_t)(xTaskGetTickCount() - last_telemetry) >= // 串口输入每轮处理，遥测达到独立周期才发送。
            pdMS_TO_TICKS(MOTOR_TELEMETRY_PERIOD_MS))//每 20 ms 发送一次双环遥测数据
        {
            last_telemetry = xTaskGetTickCount(); // 先记录本次发送基准，避免把发送耗时重复累加。
            MotorConsole_SendTelemetry();//发送遥测数据
#if APP_ENABLE_DIAGNOSTICS
            // 额外诊断默认关闭，不影响正常串口功能。
            motor_console_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
        // 名义每 5 ms 处理一轮输入，遥测名义每 20 ms 输出。
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_CONSOLE_PERIOD_MS));
    }
}

void CanTasks_Create(void)
{
    MotorControl_Init(); // 创建任务前初始化 PID 和停止状态，电机不会因初始化自动启动。
    /* 接收 4 > 控制 3 > 串口 2。串口格式化不能拖慢控制和接收。 */
    // 接收任务参数为空、优先级 4，返回值用于检查创建是否成功。
    if (xTaskCreate(CanRxTask, "CAN_RX", CAN_RX_STACK_WORDS, NULL, CAN_RX_TASK_PRIORITY, NULL) != pdPASS)
    {
        CanFail(CAN_FAULT_RX_TASK); // 接收任务创建失败时停止初始化。
    }
    if (xTaskCreate(MotorControlTask, "MOTOR_CTRL", MOTOR_CONTROL_STACK_WORDS, // 创建控制任务，栈容量单位为 word。
                    // 启动参数为空、优先级 3，不需要保存任务句柄。
                    NULL, MOTOR_CONTROL_TASK_PRIORITY, NULL) != pdPASS)
    {
        CanFail(CAN_FAULT_CONTROL_TASK); // 控制任务创建失败，不能继续假定能撤销给定。
    }
    // 创建串口任务，承担命令解析和完整帧遥测。
    if (xTaskCreate(MotorConsoleTask, "MOTOR_UART", MOTOR_CONSOLE_STACK_WORDS,
                    NULL, MOTOR_CONSOLE_TASK_PRIORITY, NULL) != pdPASS) // 串口优先级 2，低于反馈接收和电机控制。
    {
        CanFail(CAN_FAULT_CONSOLE_TASK); // 串口任务创建失败，无法可靠接收操作命令。
    }
}
