#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H
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
#define APP_DIAG_PASSIVE_SLOTS 16U
#endif
