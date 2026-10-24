#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H
#include <stdint.h>
#define APP_VIEW_HOLD_YAW 1U
#define APP_VIEW_HOLD_PITCH 2U
void AppConsole_CreateQueue(void);
void AppConsole_Start(void);
void AppConsole_Poll(void);
uint32_t AppConsole_GetFaultEpoch(void);
uint8_t AppConsole_IsDiagnosticMode(void);
/* Requested view lock; only the safety-gated control task applies it. */
uint8_t AppConsole_GetViewHoldMask(void);
#endif
