/* 安全许可与目标协调 */
#ifndef APP_LOGIC_H
#define APP_LOGIC_H
#include "app_config.h"
#include "safety_manager.h"
#include "target_manager.h"
typedef struct {
    SafetyManager safety;
    TargetManager targets;
    RemoteCommand command;
} AppLogic;
typedef struct { uint8_t requested, online, stop_pending, faulted; } AppAxisHealth;
/* input初始为摩擦轮健康。仅明确请求运行的轴参与共享许可检查。 */
void AppLogic_CombineActuatorHealth(SafetyInput *input, const AppAxisHealth *yaw,
                                   const AppAxisHealth *pitch, uint8_t wheels_wanted);
void AppLogic_Init(AppLogic *logic);
/* 不依赖HAL/RTOS，正式固件与软件测试共同调用这份逻辑。 */
void AppLogic_Step(AppLogic *logic, const SafetyInput *input, uint8_t flywheels_ready);

#endif
