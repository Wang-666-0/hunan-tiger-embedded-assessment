#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdPASS 1
#define pdFALSE 0
#define portYIELD_FROM_ISR(x) ((void)(x))
#endif
