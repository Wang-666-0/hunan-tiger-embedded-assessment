#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
void Test_EnterCritical(void);
void Test_ExitCritical(void);
#define taskENTER_CRITICAL() Test_EnterCritical()
#define taskEXIT_CRITICAL() Test_ExitCritical()
#endif
