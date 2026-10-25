#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xFFFFFFFFU
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define portYIELD_FROM_ISR(x) ((void)(x))
#endif
