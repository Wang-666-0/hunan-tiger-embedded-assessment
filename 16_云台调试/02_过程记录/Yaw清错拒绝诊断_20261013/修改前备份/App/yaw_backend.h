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
uint8_t YawBackend_TakeRearmRequest(void);
#endif
