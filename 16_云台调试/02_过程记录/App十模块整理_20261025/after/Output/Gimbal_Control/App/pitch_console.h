/**
 * @file pitch_console.h
 * @brief Pitch 串口命令
 * 职责：声明 PITCH 命令、准备状态轮询及取消接口。
 * 主要接口：PitchConsole_Init, PitchConsole_Handle, PitchConsole_Poll, PitchConsole_CancelPreparation。
 * 实现/调用：motor_console.c；参数合法后仍由 PitchBackend 再验证当前会话与安全状态。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef PITCH_CONSOLE_H
#define PITCH_CONSOLE_H
#include "app_config.h"
#include <stdint.h>
#include <stddef.h>
void PitchConsole_Init(void);
uint8_t PitchConsole_Handle(const char *line, char *response, size_t capacity);
uint8_t PitchConsole_Poll(char *response, size_t capacity);
void PitchConsole_CancelPreparation(void);
#endif
