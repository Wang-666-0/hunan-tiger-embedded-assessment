#ifndef APP_TASKS_H
#define APP_TASKS_H
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
void AppTasks_GetState(AppState *state);
#endif
