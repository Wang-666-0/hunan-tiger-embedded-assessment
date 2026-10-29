/* 任务及应用快照 */
#ifndef APP_TASKS_H
#define APP_TASKS_H
#include "app_config.h"
#include "app_logic.h"
#include "motor_backend.h"
typedef struct {
    AppLogic logic;
    RemoteFrame remote;
    MotorHealth motors;
    uint32_t control_gap_ms, stop_sequence;
} AppState;
void AppTasks_Create(void);
void AppTasks_RequestStop(void);
/* 已关闭且安全OFF的准备流程只更新人工解锁屏障，不发送失能帧。 */
void AppTasks_RequireRearm(void);
void AppTasks_GetState(AppState *state);
#endif
