#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <stdint.h>
#include "FreeRTOS.h"

typedef enum
{
    PWM_SET_MAX_BRIGHTNESS, /* 呼吸灯峰值，范围 0..1000 */
    PWM_SET_BREATH_PERIOD   /* 完整呼吸周期，范围 200..10000 ms */
} PwmCommandType;

typedef struct
{
    PwmCommandType type;
    uint32_t value;
} PwmCommand;

void PwmControl_Init(void);
BaseType_t PwmControl_Submit(const PwmCommand *command);
void PwmControl_Task(void *argument);

#endif /* PWM_CONTROL_H */
