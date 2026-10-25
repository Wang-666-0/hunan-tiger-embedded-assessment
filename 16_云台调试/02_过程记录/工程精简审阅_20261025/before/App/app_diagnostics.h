/**
 * @file app_diagnostics.h
 * @brief 诊断会话接口
 * 职责：定义只读查询种类、结果事件及被动 CAN 帧快照。
 * 主要接口：AppDiagnostics_Request, AppDiagnostics_Cancel, AppDiagnostics_Poll, AppDiagnostics_OnFrame, AppDiagnostics_TakeEvent 等。
 * 实现/调用：app_diagnostics.c；Request/Poll 推进查询，TakeEvent 分段读取结果。
 * 参数：app_config.h 是 App 参数及常量的唯一宏定义入口。
 */
#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H
#include "app_config.h"
#include "can_frame.h"

typedef enum { DIAG_DM = 1, DIAG_VESC = 2 } DiagKind;
typedef enum {
    DIAG_EVENT_DATA = 1, DIAG_EVENT_DONE, DIAG_EVENT_ERROR
} DiagEventKind;
typedef enum {
    DIAG_ERROR_NONE = 0, DIAG_ERROR_TIMEOUT, DIAG_ERROR_CAN_FAULT,
    DIAG_ERROR_TX_FAILED, DIAG_ERROR_BAD_REPLY, DIAG_ERROR_CANCELLED
} DiagError;
typedef struct {
    DiagEventKind event;
    DiagKind kind;
    DiagError error;
    uint16_t total, offset;
    uint8_t node, field, length, data[32];
} DiagEvent;
typedef struct { CanFrame frame; uint32_t count; } DiagPassiveFrame;

/* Console提交请求/读取结果；CAN任务唯一负责查询与响应解析。
 * 同时只允许一笔请求。完成结果逐块交付，避免阻塞高优先级CAN接收。 */
uint8_t AppDiagnostics_Request(DiagKind kind, uint8_t node, uint8_t field);
void AppDiagnostics_Cancel(void);
void AppDiagnostics_Poll(uint32_t now);
void AppDiagnostics_OnFrame(const CanFrame *frame);
uint8_t AppDiagnostics_TakeEvent(DiagEvent *out);
uint8_t AppDiagnostics_GetPassive(uint32_t index, DiagPassiveFrame *out);
#endif
