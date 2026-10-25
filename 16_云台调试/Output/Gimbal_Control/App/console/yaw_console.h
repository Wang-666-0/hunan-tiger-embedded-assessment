/**
 * @file yaw_console.h
 * @brief Yaw 串口命令
 * 职责：声明 YAW 命令处理、准备状态轮询和准备取消接口。
 * 主要接口：YawConsole_Handle, YawConsole_Poll, YawConsole_CancelPreparation。
 * 实现：console/yaw_console.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef YAW_CONSOLE_H
#define YAW_CONSOLE_H
#include "app_config.h"
#include <stdint.h>
#include <stddef.h>
/* 单行ASCII命令；结果填入低优先级任务缓冲，函数内不发送串口。
 * YAW BUS只读取CAN计数/首次错误/HAL错误，不要求DIAG ON。 */
uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity);
/* 仅Console任务调用，返回一次待发送的完成/中止报告；不自行发UART。 */
uint8_t YawConsole_Poll(char *response, size_t capacity);
/* 全局STOP/诊断切换取消未完成的READY，后续不得自动继续准备。 */
void YawConsole_CancelPreparation(void);
#endif
