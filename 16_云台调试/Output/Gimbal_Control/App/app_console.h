#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H
#include <stdint.h>
void AppConsole_CreateQueue(void);
void AppConsole_Start(void);
void AppConsole_Poll(void);
uint32_t AppConsole_GetFaultEpoch(void);
uint8_t AppConsole_IsDiagnosticMode(void);
#endif
