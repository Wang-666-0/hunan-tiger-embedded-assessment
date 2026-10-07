#ifndef TEST_QUEUE_H
#define TEST_QUEUE_H
#include "FreeRTOS.h"
typedef void * QueueHandle_t;
QueueHandle_t xQueueCreate(uint32_t count, uint32_t size);
void vQueueAddToRegistry(QueueHandle_t queue, const char *name);
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken);
#endif
