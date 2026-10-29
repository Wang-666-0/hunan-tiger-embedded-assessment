/* 控制台入口 */
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
