/**
 * @file app_fault.h
 * @brief 致命故障接口
 * 职责：定义队列、任务、UART 初始化失败等不可继续运行的故障入口。
 * 主要接口：AppFault_Fatal。
 * 实现/调用：app_tasks.c；AppFault_Fatal 进入 Error_Handler，不依赖损坏的 RTOS 环境重试。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef APP_FAULT_H
#define APP_FAULT_H
#include "app_config.h"
typedef enum {
    APP_FAULT_QUEUE = 1, APP_FAULT_TASK = 2, APP_FAULT_CAN_START = 3,
    APP_FAULT_UART_START = 4, APP_FAULT_STACK = 5, APP_FAULT_MALLOC = 6
} AppFault;
void AppFault_Fatal(AppFault fault);
#endif
