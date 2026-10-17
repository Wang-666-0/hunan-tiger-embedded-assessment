#ifndef YAW_BACKEND_H
#define YAW_BACKEND_H
#include "yaw_controller.h"
#include "target_manager.h"
typedef struct {
    YawController controller;
    Gl60Feedback feedback;
    uint32_t rx_count, feedback_age_ms;
    uint8_t online, requested, active, stop_pending, enabling;
} YawBackendState;
/* 位值可同时出现；CHECK只读，当前阻碍与上一次清错拒绝分别记录。 */
typedef enum {
    YAW_CLEAR_NOT_MONITORING = 1UL << 0,
    YAW_CLEAR_REQUESTED = 1UL << 1,
    YAW_CLEAR_ACTIVE = 1UL << 2,
    YAW_CLEAR_WAITING_DISABLE = 1UL << 3,
    YAW_CLEAR_OUTPUT_ALLOWED = 1UL << 4,
    YAW_CLEAR_FEEDBACK_STALE = 1UL << 5,
    YAW_CLEAR_STATUS_NOT_13 = 1UL << 6,
    YAW_CLEAR_OTHER_FAULT = 1UL << 7,
    YAW_CLEAR_NOT_STATIONARY = 1UL << 8,
    YAW_CLEAR_DISABLE_UNCONFIRMED = 1UL << 9,
    YAW_CLEAR_CAN_FAULT = 1UL << 10,
    YAW_CLEAR_MAILBOX_BUSY = 1UL << 11,
    YAW_CLEAR_TX_FAILED = 1UL << 12
} YawClearBlock;
typedef struct {
    uint32_t blocks, last_blocks, hold_ms, fd_age_ms, mailbox_pending, can_epoch;
    uint8_t samples, monitoring, fd_sent, fd_cancel, fd_result, clear_result;
} YawClearCheck;
void YawBackend_Init(void);
uint8_t YawBackend_OnFrame(const CanFrame *frame);
void YawBackend_GetState(YawBackendState *out);
/* ON仅准备链路；必须新的CH5 OFF->ON且反馈健康才会使能。 */
uint8_t YawBackend_Request(uint8_t enabled);
void YawBackend_Stop(void);
void YawBackend_Apply(const TargetRequest *request, uint8_t allowed, uint8_t diagnostic);
uint8_t YawBackend_SetParameter(char parameter, float value);
uint8_t YawBackend_CaptureReference(void);
/* 人工单次FB：Yaw请求关闭、通信故障13、连续静止、温度正常才接受。
 * 不直接清软件故障，不使能；提交后先完成新的FD才可再次准备。 */
uint8_t YawBackend_ClearCommsFault(void);
void YawBackend_GetClearCheck(YawClearCheck *out);
uint8_t YawBackend_TakeRearmRequest(void);
#endif
