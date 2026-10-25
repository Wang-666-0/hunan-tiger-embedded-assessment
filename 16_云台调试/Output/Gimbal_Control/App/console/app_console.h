/**
 * @file app_console.h
 * @brief 控制台入口
 * 职责：声明命令接收、遥测轮询、诊断模式及视图锁定状态接口。
 * 主要接口：AppConsole_CreateQueue, AppConsole_Start, AppConsole_Poll, AppConsole_GetFaultEpoch, AppConsole_GetViewHoldMask。
 * 实现：console/app_console.c；调用关系见实现文件与 App/README。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H
#include "app_config.h"
#include <stdint.h>
void AppConsole_CreateQueue(void);
void AppConsole_Start(void);
void AppConsole_Poll(void);
uint32_t AppConsole_GetFaultEpoch(void);
/* Requested view lock; only the safety-gated control task applies it. */
uint8_t AppConsole_GetViewHoldMask(void);
#endif
