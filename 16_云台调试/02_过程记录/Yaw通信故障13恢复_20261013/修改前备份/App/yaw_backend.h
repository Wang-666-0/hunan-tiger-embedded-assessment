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
uint8_t YawBackend_TakeRearmRequest(void);
#endif
