#ifndef YAW_CONSOLE_H
#define YAW_CONSOLE_H
#include <stdint.h>
#include <stddef.h>
/* 单行ASCII命令；结果填入低优先级任务缓冲，函数内不发送串口。 */
uint8_t YawConsole_Handle(const char *line, char *response, size_t capacity);
/* 仅Console任务调用，返回一次待发送的完成/中止报告；不自行发UART。 */
uint8_t YawConsole_Poll(char *response, size_t capacity);
#endif
