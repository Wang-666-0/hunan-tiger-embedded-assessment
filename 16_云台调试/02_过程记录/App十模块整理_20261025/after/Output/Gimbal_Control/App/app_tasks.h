/**
 * @file app_tasks.h
 * @brief 任务及应用快照
 * 职责：声明任务创建、统一停机、重新解锁屏障及只读 AppState 快照。
 * 主要接口：AppTasks_Create, AppTasks_RequestStop, AppTasks_RequireRearm, AppTasks_GetState。
 * 实现/调用：app_tasks.c；AppTasks_GetState 在临界区内复制一致快照。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H
#include "app_config.h"
#include "app_logic.h"
#include "motor_backend.h"
typedef struct {
    AppLogic logic;
    RemoteFrame remote;
    MotorHealth motors;
    uint32_t control_gap_ms, stop_sequence, guard_control_gap_ms;
    uint32_t guard_remote_uart_epoch, guard_console_uart_epoch;
} AppState;
void AppTasks_Create(void);
void AppTasks_RequestStop(void);
/* 已关闭且安全OFF的准备流程只更新人工解锁屏障，不发送失能帧。 */
void AppTasks_RequireRearm(void);
void AppTasks_GetState(AppState *state);
#endif
