#ifndef IMU_ACQUISITION_H
#define IMU_ACQUISITION_H

#include <stdint.h>
#include "FreeRTOS.h"

typedef struct
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t sample_count;
    uint32_t read_errors;
} ImuMessage;

void ImuAcquisition_Init(void);
BaseType_t ImuAcquisition_TakeLatest(ImuMessage *message);
void ImuAcquisition_Task(void *argument);

#endif /* IMU_ACQUISITION_H */
