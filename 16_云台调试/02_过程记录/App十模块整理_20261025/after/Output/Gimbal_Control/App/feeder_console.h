/**
 * @file feeder_console.h
 * @brief 拨盘串口命令
 * 职责：声明 FEEDER 命令处理、角度遥测和速度遥测格式化接口。
 * 主要接口：FeederConsole_Handle, FeederConsole_Telemetry, FeederConsole_SpeedTelemetry。
 * 实现/调用：motor_console.c；只负责校验/报告，运动授权仍由后端检查。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef FEEDER_CONSOLE_H
#define FEEDER_CONSOLE_H
#include "app_config.h"
#include <stddef.h>
#include <stdint.h>
uint8_t FeederConsole_Handle(const char *line, char *response, size_t capacity);
void FeederConsole_Telemetry(char *response, size_t capacity);
void FeederConsole_SpeedTelemetry(char *response, size_t capacity);
#endif
