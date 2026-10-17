#ifndef APP_LOGIC_H
#define APP_LOGIC_H
#include "safety_manager.h"
#include "target_manager.h"
typedef struct {
    SafetyManager safety;
    TargetManager targets;
    RemoteCommand command;
} AppLogic;
void AppLogic_Init(AppLogic *logic);
/* 不依赖HAL/RTOS，正式固件与软件测试共同调用这份逻辑。 */
void AppLogic_Step(AppLogic *logic, const SafetyInput *input, uint8_t flywheels_ready);
#endif
